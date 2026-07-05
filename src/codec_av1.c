/*
 * AV1 VA-API to V4L2 stateless translation.
 *
 * A few sequence flags and the per-reference order hints are not part of
 * the VA-API AV1 interface; the flags are derived from frame level state
 * and the order hints are tracked per surface as frames are decoded.
 *
 * Copyright (C) 2026 Ondrej Jirman <megi@xff.cz>
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <stdlib.h>
#include <string.h>

#include <linux/videodev2.h>
#include <linux/v4l2-controls.h>

#include "v4l2_request.h"

#if HAVE_V4L2_CTRL_AV1

#include <va/va_dec_av1.h>

#define AV1_KEY_FRAME		0
#define AV1_INTRA_ONLY_FRAME	2
#define AV1_SUPERRES_NUM	8

struct av1_context {
	bool has_film_grain;

	VADecPictureParameterBufferAV1 va_pic;
	bool have_pic;

	struct v4l2_ctrl_av1_sequence sequence;
	struct v4l2_ctrl_av1_frame frame;
	struct v4l2_ctrl_av1_film_grain film_grain;

	struct v4l2_ctrl_av1_tile_group_entry *tiles;
	unsigned int alloc_tiles;
	unsigned int num_tiles;

	VASliceParameterBufferAV1 *va_slices;
	unsigned int nb_va_slices;
	unsigned int alloc_va_slices;
	unsigned int slices_consumed;

	/*
	 * One-frame submission deferral. refresh_frame_flags (which DPB slots
	 * the frame writes) is not exposed by VA-API and the AV1 frame header
	 * that carries it is not part of the slice data, so a frame's true
	 * refresh mask is only recoverable once the NEXT frame's ref_frame_map
	 * is seen: the slots that then hold this frame's surface are exactly
	 * the ones it refreshed. The kernel keys its per-slot CDF entropy
	 * context store/load on that mask, so getting it right is required for
	 * frames that inherit context from anything other than the most recent
	 * reference (e.g. a long-term/golden reference).
	 *
	 * Each fully assembled frame is therefore held back: its controls,
	 * tile list and the reserved OUTPUT/CAPTURE picture state are copied
	 * here at end_picture, and it is submitted when the next picture
	 * parameters arrive (exact mask) or when its surface is read first
	 * (heuristic fallback mask).
	 */
	bool pend_valid;
	struct v4l2_ctrl_av1_sequence pend_sequence;
	struct v4l2_ctrl_av1_frame pend_frame;
	struct v4l2_ctrl_av1_film_grain pend_film_grain;
	bool pend_has_film_grain;
	struct v4l2_ctrl_av1_tile_group_entry *pend_tiles;
	unsigned int pend_alloc_tiles;
	unsigned int pend_num_tiles;
	struct v4l2r_picture pend_pic;
	uint32_t pend_current;		/* held frame's current_frame surface */
	uint8_t pend_fallback;		/* mask to use if flushed before next frame */
};

static void av1_fill_sequence(struct av1_context *codec)
{
	const VADecPictureParameterBufferAV1 *pic = &codec->va_pic;
	struct v4l2_ctrl_av1_sequence *seq = &codec->sequence;
	static const uint8_t bit_depths[3] = { 8, 10, 12 };

	*seq = (struct v4l2_ctrl_av1_sequence) {
		.seq_profile = pic->profile,
		.order_hint_bits = pic->seq_info_fields.fields.enable_order_hint ?
				   pic->order_hint_bits_minus_1 + 1 : 0,
		.bit_depth = bit_depths[pic->bit_depth_idx < 3 ?
					pic->bit_depth_idx : 0],
		.max_frame_width_minus_1 = pic->frame_width_minus1,
		.max_frame_height_minus_1 = pic->frame_height_minus1,
	};

	if (pic->seq_info_fields.fields.still_picture)
		seq->flags |= V4L2_AV1_SEQUENCE_FLAG_STILL_PICTURE;
	if (pic->seq_info_fields.fields.use_128x128_superblock)
		seq->flags |= V4L2_AV1_SEQUENCE_FLAG_USE_128X128_SUPERBLOCK;
	if (pic->seq_info_fields.fields.enable_filter_intra)
		seq->flags |= V4L2_AV1_SEQUENCE_FLAG_ENABLE_FILTER_INTRA;
	if (pic->seq_info_fields.fields.enable_intra_edge_filter)
		seq->flags |= V4L2_AV1_SEQUENCE_FLAG_ENABLE_INTRA_EDGE_FILTER;
	if (pic->seq_info_fields.fields.enable_interintra_compound)
		seq->flags |= V4L2_AV1_SEQUENCE_FLAG_ENABLE_INTERINTRA_COMPOUND;
	if (pic->seq_info_fields.fields.enable_masked_compound)
		seq->flags |= V4L2_AV1_SEQUENCE_FLAG_ENABLE_MASKED_COMPOUND;
	if (pic->seq_info_fields.fields.enable_dual_filter)
		seq->flags |= V4L2_AV1_SEQUENCE_FLAG_ENABLE_DUAL_FILTER;
	if (pic->seq_info_fields.fields.enable_order_hint)
		seq->flags |= V4L2_AV1_SEQUENCE_FLAG_ENABLE_ORDER_HINT;
	if (pic->seq_info_fields.fields.enable_jnt_comp)
		seq->flags |= V4L2_AV1_SEQUENCE_FLAG_ENABLE_JNT_COMP;
	if (pic->seq_info_fields.fields.enable_cdef)
		seq->flags |= V4L2_AV1_SEQUENCE_FLAG_ENABLE_CDEF;
	if (pic->seq_info_fields.fields.mono_chrome)
		seq->flags |= V4L2_AV1_SEQUENCE_FLAG_MONO_CHROME;
	if (pic->seq_info_fields.fields.color_range)
		seq->flags |= V4L2_AV1_SEQUENCE_FLAG_COLOR_RANGE;
	if (pic->seq_info_fields.fields.subsampling_x)
		seq->flags |= V4L2_AV1_SEQUENCE_FLAG_SUBSAMPLING_X;
	if (pic->seq_info_fields.fields.subsampling_y)
		seq->flags |= V4L2_AV1_SEQUENCE_FLAG_SUBSAMPLING_Y;
	if (pic->seq_info_fields.fields.film_grain_params_present)
		seq->flags |= V4L2_AV1_SEQUENCE_FLAG_FILM_GRAIN_PARAMS_PRESENT;

	/*
	 * VA-API does not carry these sequence flags; enable them whenever
	 * the current frame makes use of the tool so drivers do not reject
	 * frame level state as inconsistent.
	 */
	if (pic->pic_info_fields.bits.use_superres)
		seq->flags |= V4L2_AV1_SEQUENCE_FLAG_ENABLE_SUPERRES;
	if (pic->pic_info_fields.bits.use_ref_frame_mvs)
		seq->flags |= V4L2_AV1_SEQUENCE_FLAG_ENABLE_REF_FRAME_MVS;
	if (pic->pic_info_fields.bits.allow_warped_motion)
		seq->flags |= V4L2_AV1_SEQUENCE_FLAG_ENABLE_WARPED_MOTION;
	if (pic->loop_restoration_fields.bits.yframe_restoration_type ||
	    pic->loop_restoration_fields.bits.cbframe_restoration_type ||
	    pic->loop_restoration_fields.bits.crframe_restoration_type)
		seq->flags |= V4L2_AV1_SEQUENCE_FLAG_ENABLE_RESTORATION;
	/* diff_uv_delta can only be coded when the sequence has
	 * separate_uv_delta_q; VA carries neither, so derive it from the same
	 * per-frame condition used for QUANTIZATION_FLAG_DIFF_UV_DELTA. */
	if (pic->u_dc_delta_q != pic->v_dc_delta_q ||
	    pic->u_ac_delta_q != pic->v_ac_delta_q)
		seq->flags |= V4L2_AV1_SEQUENCE_FLAG_SEPARATE_UV_DELTA_Q;
}

/* AV1 get_relative_dist() (spec 5.9.3). */
static int av1_relative_dist(int a, int b, unsigned int order_hint_bits)
{
	int diff, m;

	if (!order_hint_bits)
		return 0;

	diff = a - b;
	m = 1 << (order_hint_bits - 1);
	return (diff & (m - 1)) - (diff & m);
}

static void av1_fill_frame(struct v4l2r_context *ctx)
{
	struct av1_context *codec = ctx->codec_priv;
	const VADecPictureParameterBufferAV1 *pic = &codec->va_pic;
	struct v4l2_ctrl_av1_frame *frame = &codec->frame;
	uint32_t upscaled_width = pic->frame_width_minus1 + 1;
	uint32_t frame_width = upscaled_width;
	uint32_t frame_height = pic->frame_height_minus1 + 1;
	/* Superblock size in units of 4x4 mode info blocks (log2). */
	unsigned int sb_shift =
		pic->seq_info_fields.fields.use_128x128_superblock ? 5 : 4;
	uint32_t mi_start;
	/* tile_cols/tile_rows are client-supplied uint8_t; clamp to the control
	 * array bounds so the loops and terminator stores below cannot run off
	 * the end of tile_info on a malformed frame. */
	unsigned int tile_cols = pic->tile_cols <= V4L2_AV1_MAX_TILE_COLS ?
				 pic->tile_cols : V4L2_AV1_MAX_TILE_COLS;
	unsigned int tile_rows = pic->tile_rows <= V4L2_AV1_MAX_TILE_ROWS ?
				 pic->tile_rows : V4L2_AV1_MAX_TILE_ROWS;

	if (pic->pic_info_fields.bits.use_superres &&
	    pic->superres_scale_denominator > AV1_SUPERRES_NUM)
		frame_width = (upscaled_width * AV1_SUPERRES_NUM +
			       pic->superres_scale_denominator / 2) /
			      pic->superres_scale_denominator;

	*frame = (struct v4l2_ctrl_av1_frame) {
		.tile_info = {
			.context_update_tile_id = pic->context_update_tile_id,
			.tile_cols = tile_cols,
			.tile_rows = tile_rows,
			.tile_size_bytes = (tile_cols * tile_rows) > 1 ?
					   4 : 0,
		},

		.quantization = {
			.base_q_idx = pic->base_qindex,
			.delta_q_y_dc = pic->y_dc_delta_q,
			.delta_q_u_dc = pic->u_dc_delta_q,
			.delta_q_u_ac = pic->u_ac_delta_q,
			.delta_q_v_dc = pic->v_dc_delta_q,
			.delta_q_v_ac = pic->v_ac_delta_q,
			.qm_y = pic->qmatrix_fields.bits.qm_y,
			.qm_u = pic->qmatrix_fields.bits.qm_u,
			.qm_v = pic->qmatrix_fields.bits.qm_v,
			.delta_q_res = pic->mode_control_fields.bits.log2_delta_q_res,
		},

		.loop_filter = {
			.level = {
				pic->filter_level[0],
				pic->filter_level[1],
				pic->filter_level_u,
				pic->filter_level_v,
			},
			.sharpness = pic->loop_filter_info_fields.bits.sharpness_level,
			.mode_deltas = {
				pic->mode_deltas[0],
				pic->mode_deltas[1],
			},
			.delta_lf_res = pic->mode_control_fields.bits.log2_delta_lf_res,
		},

		.cdef = {
			.damping_minus_3 = pic->cdef_damping_minus_3,
			.bits = pic->cdef_bits,
		},

		.loop_restoration = {
			.lr_unit_shift = pic->loop_restoration_fields.bits.lr_unit_shift,
			.lr_uv_shift = pic->loop_restoration_fields.bits.lr_uv_shift,
			.frame_restoration_type = {
				pic->loop_restoration_fields.bits.yframe_restoration_type,
				pic->loop_restoration_fields.bits.cbframe_restoration_type,
				pic->loop_restoration_fields.bits.crframe_restoration_type,
			},
		},

		.superres_denom = pic->pic_info_fields.bits.use_superres ?
				  pic->superres_scale_denominator :
				  AV1_SUPERRES_NUM,
		.primary_ref_frame = pic->primary_ref_frame,
		.frame_type = pic->pic_info_fields.bits.frame_type,
		.order_hint = pic->order_hint,
		.upscaled_width = upscaled_width,
		.interpolation_filter = pic->interp_filter,
		.tx_mode = pic->mode_control_fields.bits.tx_mode,
		.frame_width_minus_1 = frame_width - 1,
		.frame_height_minus_1 = frame_height - 1,
		.render_width_minus_1 = upscaled_width - 1,
		.render_height_minus_1 = frame_height - 1,

		/*
		 * refresh_frame_flags is not transported by VA-API and the AV1
		 * frame header OBU that carries it is not part of the slice data
		 * (clients hand over only the tile group payload). Its true value
		 * is only revealed by the next frame's ref_frame_map, so it is
		 * filled in later at submit time (av1_submit_pending); the value
		 * set here is a placeholder that is always overwritten.
		 */
		.refresh_frame_flags = 0,
	};

	if (pic->seg_info.segment_info_fields.bits.enabled)
		frame->segmentation.flags |= V4L2_AV1_SEGMENTATION_FLAG_ENABLED;
	if (pic->seg_info.segment_info_fields.bits.update_map)
		frame->segmentation.flags |= V4L2_AV1_SEGMENTATION_FLAG_UPDATE_MAP;
	if (pic->seg_info.segment_info_fields.bits.temporal_update)
		frame->segmentation.flags |= V4L2_AV1_SEGMENTATION_FLAG_TEMPORAL_UPDATE;
	if (pic->seg_info.segment_info_fields.bits.update_data)
		frame->segmentation.flags |= V4L2_AV1_SEGMENTATION_FLAG_UPDATE_DATA;

	for (int i = 0; i < V4L2_AV1_MAX_SEGMENTS; i++) {
		for (int j = 0; j < V4L2_AV1_SEG_LVL_MAX; j++) {
			if (pic->seg_info.feature_mask[i] & (1 << j)) {
				frame->segmentation.feature_enabled[i] |=
					V4L2_AV1_SEGMENT_FEATURE_ENABLED(j);
				frame->segmentation.last_active_seg_id = i;
				if (j >= V4L2_AV1_SEG_LVL_REF_FRAME)
					frame->segmentation.flags |=
						V4L2_AV1_SEGMENTATION_FLAG_SEG_ID_PRE_SKIP;
			}
			frame->segmentation.feature_data[i][j] =
				pic->seg_info.feature_data[i][j];
		}
	}

	if (pic->pic_info_fields.bits.uniform_tile_spacing_flag)
		frame->tile_info.flags |= V4L2_AV1_TILE_INFO_FLAG_UNIFORM_TILE_SPACING;

	/* VA carries at most 63 explicit tile dimensions (width/height_in_sbs
	 * arrays); the final tile size is derived from the mi_*_starts
	 * terminator, so bound the reads to the VA array size. */
	mi_start = 0;
	for (unsigned int i = 0; i < tile_cols; i++) {
		frame->tile_info.mi_col_starts[i] = mi_start;
		if (i < 63) {
			frame->tile_info.width_in_sbs_minus_1[i] =
				pic->width_in_sbs_minus_1[i];
			mi_start += (pic->width_in_sbs_minus_1[i] + 1) << sb_shift;
		}
	}
	frame->tile_info.mi_col_starts[tile_cols] =
		2 * ((frame_width + 7) >> 3);

	mi_start = 0;
	for (unsigned int i = 0; i < tile_rows; i++) {
		frame->tile_info.mi_row_starts[i] = mi_start;
		if (i < 63) {
			frame->tile_info.height_in_sbs_minus_1[i] =
				pic->height_in_sbs_minus_1[i];
			mi_start += (pic->height_in_sbs_minus_1[i] + 1) << sb_shift;
		}
	}
	frame->tile_info.mi_row_starts[tile_rows] =
		2 * ((frame_height + 7) >> 3);

	if (pic->qmatrix_fields.bits.using_qmatrix)
		frame->quantization.flags |= V4L2_AV1_QUANTIZATION_FLAG_USING_QMATRIX;
	if (pic->mode_control_fields.bits.delta_q_present_flag)
		frame->quantization.flags |= V4L2_AV1_QUANTIZATION_FLAG_DELTA_Q_PRESENT;
	if (pic->u_dc_delta_q != pic->v_dc_delta_q ||
	    pic->u_ac_delta_q != pic->v_ac_delta_q)
		frame->quantization.flags |= V4L2_AV1_QUANTIZATION_FLAG_DIFF_UV_DELTA;

	if (pic->loop_filter_info_fields.bits.mode_ref_delta_enabled)
		frame->loop_filter.flags |= V4L2_AV1_LOOP_FILTER_FLAG_DELTA_ENABLED;
	if (pic->loop_filter_info_fields.bits.mode_ref_delta_update)
		frame->loop_filter.flags |= V4L2_AV1_LOOP_FILTER_FLAG_DELTA_UPDATE;
	if (pic->mode_control_fields.bits.delta_lf_present_flag)
		frame->loop_filter.flags |= V4L2_AV1_LOOP_FILTER_FLAG_DELTA_LF_PRESENT;
	if (pic->mode_control_fields.bits.delta_lf_multi)
		frame->loop_filter.flags |= V4L2_AV1_LOOP_FILTER_FLAG_DELTA_LF_MULTI;

	for (int i = 0; i < V4L2_AV1_TOTAL_REFS_PER_FRAME; i++)
		frame->loop_filter.ref_deltas[i] = pic->ref_deltas[i];

	if (frame->loop_restoration.frame_restoration_type[0] ||
	    frame->loop_restoration.frame_restoration_type[1] ||
	    frame->loop_restoration.frame_restoration_type[2]) {
		frame->loop_restoration.flags |=
			V4L2_AV1_LOOP_RESTORATION_FLAG_USES_LR;
		if (frame->loop_restoration.frame_restoration_type[1] ||
		    frame->loop_restoration.frame_restoration_type[2])
			frame->loop_restoration.flags |=
				V4L2_AV1_LOOP_RESTORATION_FLAG_USES_CHROMA_LR;

		frame->loop_restoration.loop_restoration_size[0] =
			1 << (6 + pic->loop_restoration_fields.bits.lr_unit_shift);
		frame->loop_restoration.loop_restoration_size[1] =
			1 << (6 + pic->loop_restoration_fields.bits.lr_unit_shift -
			      pic->loop_restoration_fields.bits.lr_uv_shift);
		frame->loop_restoration.loop_restoration_size[2] =
			frame->loop_restoration.loop_restoration_size[1];
	}

	for (int i = 0; i < (1 << pic->cdef_bits) && i < V4L2_AV1_CDEF_MAX; i++) {
		frame->cdef.y_pri_strength[i] = pic->cdef_y_strengths[i] >> 2;
		frame->cdef.y_sec_strength[i] = pic->cdef_y_strengths[i] & 3;
		frame->cdef.uv_pri_strength[i] = pic->cdef_uv_strengths[i] >> 2;
		frame->cdef.uv_sec_strength[i] = pic->cdef_uv_strengths[i] & 3;
	}

	/* Global motion: VA index 0 is LAST, V4L2 index 0 is INTRA. */
	for (int i = 0; i < 7; i++) {
		const VAWarpedMotionParamsAV1 *wm = &pic->wm[i];
		int ref = i + 1;
		static const uint8_t type_map[4] = {
			V4L2_AV1_WARP_MODEL_IDENTITY,
			V4L2_AV1_WARP_MODEL_TRANSLATION,
			V4L2_AV1_WARP_MODEL_ROTZOOM,
			V4L2_AV1_WARP_MODEL_AFFINE,
		};

		if (wm->wmtype > VAAV1TransformationAffine)
			continue;

		frame->global_motion.type[ref] = type_map[wm->wmtype];
		for (int j = 0; j < 6; j++)
			frame->global_motion.params[ref][j] = wm->wmmat[j];

		if (wm->invalid)
			frame->global_motion.invalid |=
				V4L2_AV1_GLOBAL_MOTION_IS_INVALID(ref);

		if (wm->wmtype != VAAV1TransformationIdentity)
			frame->global_motion.flags[ref] |=
				V4L2_AV1_GLOBAL_MOTION_FLAG_IS_GLOBAL;
		if (wm->wmtype == VAAV1TransformationRotzoom)
			frame->global_motion.flags[ref] |=
				V4L2_AV1_GLOBAL_MOTION_FLAG_IS_ROT_ZOOM;
		if (wm->wmtype == VAAV1TransformationTranslation)
			frame->global_motion.flags[ref] |=
				V4L2_AV1_GLOBAL_MOTION_FLAG_IS_TRANSLATION;
	}

	for (int i = 0; i < V4L2_AV1_TOTAL_REFS_PER_FRAME; i++) {
		frame->reference_frame_ts[i] =
			v4l2r_surface_timestamp(ctx->drv, pic->ref_frame_map[i]);

		/* Order hints are tracked per surface as frames complete;
		 * VA-API does not transport them. */
		if (i > 0) {
			/* ref_frame_idx is client-supplied; mask to the 8-entry
			 * ref_frame_map range before indexing it. */
			struct v4l2r_surface *ref = V4L2R_SURFACE_GET(ctx->drv,
				pic->ref_frame_map[pic->ref_frame_idx[i - 1] & 7]);
			frame->order_hints[i] = ref ? ref->codec_tag : 0;
		}
	}

	for (int i = 0; i < 7; i++)
		frame->ref_frame_idx[i] = pic->ref_frame_idx[i];

	if (pic->pic_info_fields.bits.show_frame)
		frame->flags |= V4L2_AV1_FRAME_FLAG_SHOW_FRAME;
	if (pic->pic_info_fields.bits.showable_frame)
		frame->flags |= V4L2_AV1_FRAME_FLAG_SHOWABLE_FRAME;
	if (pic->pic_info_fields.bits.error_resilient_mode)
		frame->flags |= V4L2_AV1_FRAME_FLAG_ERROR_RESILIENT_MODE;
	if (pic->pic_info_fields.bits.disable_cdf_update)
		frame->flags |= V4L2_AV1_FRAME_FLAG_DISABLE_CDF_UPDATE;
	if (pic->pic_info_fields.bits.allow_screen_content_tools)
		frame->flags |= V4L2_AV1_FRAME_FLAG_ALLOW_SCREEN_CONTENT_TOOLS;
	if (pic->pic_info_fields.bits.force_integer_mv)
		frame->flags |= V4L2_AV1_FRAME_FLAG_FORCE_INTEGER_MV;
	if (pic->pic_info_fields.bits.allow_intrabc)
		frame->flags |= V4L2_AV1_FRAME_FLAG_ALLOW_INTRABC;
	if (pic->pic_info_fields.bits.use_superres)
		frame->flags |= V4L2_AV1_FRAME_FLAG_USE_SUPERRES;
	if (pic->pic_info_fields.bits.allow_high_precision_mv)
		frame->flags |= V4L2_AV1_FRAME_FLAG_ALLOW_HIGH_PRECISION_MV;
	if (pic->pic_info_fields.bits.is_motion_mode_switchable)
		frame->flags |= V4L2_AV1_FRAME_FLAG_IS_MOTION_MODE_SWITCHABLE;
	if (pic->pic_info_fields.bits.use_ref_frame_mvs)
		frame->flags |= V4L2_AV1_FRAME_FLAG_USE_REF_FRAME_MVS;
	if (pic->pic_info_fields.bits.disable_frame_end_update_cdf)
		frame->flags |= V4L2_AV1_FRAME_FLAG_DISABLE_FRAME_END_UPDATE_CDF;
	if (pic->pic_info_fields.bits.allow_warped_motion)
		frame->flags |= V4L2_AV1_FRAME_FLAG_ALLOW_WARPED_MOTION;
	if (pic->mode_control_fields.bits.reference_select)
		frame->flags |= V4L2_AV1_FRAME_FLAG_REFERENCE_SELECT;
	if (pic->mode_control_fields.bits.reduced_tx_set_used)
		frame->flags |= V4L2_AV1_FRAME_FLAG_REDUCED_TX_SET;
	if (pic->mode_control_fields.bits.skip_mode_present) {
		frame->flags |= V4L2_AV1_FRAME_FLAG_SKIP_MODE_ALLOWED;
		frame->flags |= V4L2_AV1_FRAME_FLAG_SKIP_MODE_PRESENT;
	}

	/*
	 * skip_mode_params(): VA-API does not carry SkipModeFrame; the hardware
	 * defaults both skip references to LAST_FRAME when they are zero, so
	 * derive them from the order hints (spec 7.4.10). LAST_FRAME == 1.
	 */
	if (pic->mode_control_fields.bits.skip_mode_present) {
		unsigned int ohb = pic->seq_info_fields.fields.enable_order_hint ?
				   pic->order_hint_bits_minus_1 + 1 : 0;
		int cur = pic->order_hint;
		int fwd_idx = -1, bwd_idx = -1, fwd_hint = 0, bwd_hint = 0;

		for (int i = 0; i < 7; i++) {
			int hint = frame->order_hints[i + 1];

			if (av1_relative_dist(hint, cur, ohb) < 0) {
				if (fwd_idx < 0 ||
				    av1_relative_dist(hint, fwd_hint, ohb) > 0) {
					fwd_idx = i;
					fwd_hint = hint;
				}
			} else if (av1_relative_dist(hint, cur, ohb) > 0) {
				if (bwd_idx < 0 ||
				    av1_relative_dist(hint, bwd_hint, ohb) < 0) {
					bwd_idx = i;
					bwd_hint = hint;
				}
			}
		}

		if (fwd_idx >= 0 && bwd_idx >= 0) {
			frame->skip_mode_frame[0] =
				1 + (fwd_idx < bwd_idx ? fwd_idx : bwd_idx);
			frame->skip_mode_frame[1] =
				1 + (fwd_idx < bwd_idx ? bwd_idx : fwd_idx);
		} else if (fwd_idx >= 0) {
			int second = -1, second_hint = 0;

			for (int i = 0; i < 7; i++) {
				int hint = frame->order_hints[i + 1];

				if (av1_relative_dist(hint, fwd_hint, ohb) < 0 &&
				    (second < 0 ||
				     av1_relative_dist(hint, second_hint, ohb) > 0)) {
					second = i;
					second_hint = hint;
				}
			}

			if (second >= 0) {
				frame->skip_mode_frame[0] =
					1 + (fwd_idx < second ? fwd_idx : second);
				frame->skip_mode_frame[1] =
					1 + (fwd_idx < second ? second : fwd_idx);
			}
		}
	}
}

static void av1_fill_film_grain(struct av1_context *codec)
{
	const VAFilmGrainStructAV1 *fg = &codec->va_pic.film_grain_info;
	struct v4l2_ctrl_av1_film_grain *ctrl = &codec->film_grain;

	/* Point counts are bounded by the spec (14 luma, 10 chroma) and by the
	 * control arrays; clamp the client's values so the count can never
	 * exceed the entries actually filled below. */
	uint8_t num_y = fg->num_y_points < 14 ? fg->num_y_points : 14;
	uint8_t num_cb = fg->num_cb_points < 10 ? fg->num_cb_points : 10;
	uint8_t num_cr = fg->num_cr_points < 10 ? fg->num_cr_points : 10;

	*ctrl = (struct v4l2_ctrl_av1_film_grain) {
		.grain_seed = fg->grain_seed,
		.num_y_points = num_y,
		.num_cb_points = num_cb,
		.num_cr_points = num_cr,
		.grain_scaling_minus_8 =
			fg->film_grain_info_fields.bits.grain_scaling_minus_8,
		.ar_coeff_lag = fg->film_grain_info_fields.bits.ar_coeff_lag,
		.ar_coeff_shift_minus_6 =
			fg->film_grain_info_fields.bits.ar_coeff_shift_minus_6,
		.grain_scale_shift =
			fg->film_grain_info_fields.bits.grain_scale_shift,
		.cb_mult = fg->cb_mult,
		.cb_luma_mult = fg->cb_luma_mult,
		.cr_mult = fg->cr_mult,
		.cr_luma_mult = fg->cr_luma_mult,
		.cb_offset = fg->cb_offset,
		.cr_offset = fg->cr_offset,
	};

	if (fg->film_grain_info_fields.bits.apply_grain) {
		ctrl->flags |= V4L2_AV1_FILM_GRAIN_FLAG_APPLY_GRAIN;
		/* VA-API delivers fully resolved grain parameters (there is no
		 * update_grain / params_ref_idx in the interface). Mark them as
		 * an update so the decoder uses the values here instead of
		 * loading stale grain state from a reference slot. */
		ctrl->flags |= V4L2_AV1_FILM_GRAIN_FLAG_UPDATE_GRAIN;
	}
	if (fg->film_grain_info_fields.bits.chroma_scaling_from_luma)
		ctrl->flags |= V4L2_AV1_FILM_GRAIN_FLAG_CHROMA_SCALING_FROM_LUMA;
	if (fg->film_grain_info_fields.bits.overlap_flag)
		ctrl->flags |= V4L2_AV1_FILM_GRAIN_FLAG_OVERLAP;
	if (fg->film_grain_info_fields.bits.clip_to_restricted_range)
		ctrl->flags |= V4L2_AV1_FILM_GRAIN_FLAG_CLIP_TO_RESTRICTED_RANGE;

	if (!fg->film_grain_info_fields.bits.apply_grain)
		return;

	for (int i = 0; i < num_y; i++) {
		ctrl->point_y_value[i] = fg->point_y_value[i];
		ctrl->point_y_scaling[i] = fg->point_y_scaling[i];
	}
	for (int i = 0; i < num_cb; i++) {
		ctrl->point_cb_value[i] = fg->point_cb_value[i];
		ctrl->point_cb_scaling[i] = fg->point_cb_scaling[i];
	}
	for (int i = 0; i < num_cr; i++) {
		ctrl->point_cr_value[i] = fg->point_cr_value[i];
		ctrl->point_cr_scaling[i] = fg->point_cr_scaling[i];
	}

	for (int i = 0; i < 24; i++)
		ctrl->ar_coeffs_y_plus_128[i] = fg->ar_coeffs_y[i] + 128;
	for (int i = 0; i < 25; i++) {
		ctrl->ar_coeffs_cb_plus_128[i] = fg->ar_coeffs_cb[i] + 128;
		ctrl->ar_coeffs_cr_plus_128[i] = fg->ar_coeffs_cr[i] + 128;
	}
}

/* --- codec ops --- */

static VAStatus av1_init(struct v4l2r_context *ctx)
{
	struct av1_context *codec = ctx->codec_priv;
	struct v4l2_query_ext_ctrl film_grain = {
		.id = V4L2_CID_STATELESS_AV1_FILM_GRAIN,
	};

	codec->has_film_grain = !v4l2r_query_control(ctx, &film_grain);

	return VA_STATUS_SUCCESS;
}

static void av1_uninit(struct v4l2r_context *ctx)
{
	struct av1_context *codec = ctx->codec_priv;

	/* The video/media fds are already closed by the time uninit runs, so a
	 * frame still held for deferral cannot be submitted here; drop it. Any
	 * frame the client actually read was flushed via v4l2r_flush_surface(). */
	free(codec->tiles);
	free(codec->va_slices);
	free(codec->pend_tiles);
	codec->tiles = NULL;
	codec->va_slices = NULL;
	codec->pend_tiles = NULL;
	codec->alloc_tiles = 0;
	codec->alloc_va_slices = 0;
	codec->pend_alloc_tiles = 0;
	codec->pend_valid = false;
}

static VAStatus av1_begin_picture(struct v4l2r_context *ctx)
{
	struct av1_context *codec = ctx->codec_priv;

	codec->have_pic = false;
	codec->num_tiles = 0;
	codec->nb_va_slices = 0;
	codec->slices_consumed = 0;

	return VA_STATUS_SUCCESS;
}

static VAStatus av1_store_slice_params(struct av1_context *codec,
				       struct v4l2r_buffer *buf)
{
	const VASliceParameterBufferAV1 *elements = buf->data;
	unsigned int count = buf->nb_elements;

	if (buf->element_size != sizeof(*elements))
		return VA_STATUS_ERROR_INVALID_BUFFER;

	if (v4l2r_array_reserve((void **)&codec->va_slices,
				&codec->alloc_va_slices,
				codec->nb_va_slices + count,
				sizeof(*elements)) < 0)
		return VA_STATUS_ERROR_ALLOCATION_FAILED;

	memcpy(&codec->va_slices[codec->nb_va_slices], elements,
	       count * sizeof(*elements));
	codec->nb_va_slices += count;

	return VA_STATUS_SUCCESS;
}

static VAStatus av1_process_tiles(struct v4l2r_context *ctx,
				  struct v4l2r_buffer *buf)
{
	struct av1_context *codec = ctx->codec_priv;
	uint32_t base = ctx->pic.output ? ctx->pic.output->bytesused : 0;
	unsigned int pending = codec->nb_va_slices - codec->slices_consumed;
	VAStatus status;

	if (codec->num_tiles + pending > V4L2_AV1_MAX_TILE_COUNT)
		return VA_STATUS_ERROR_OPERATION_FAILED;

	if (v4l2r_array_reserve((void **)&codec->tiles, &codec->alloc_tiles,
				codec->num_tiles + pending,
				sizeof(*codec->tiles)) < 0)
		return VA_STATUS_ERROR_ALLOCATION_FAILED;

	while (codec->slices_consumed < codec->nb_va_slices) {
		const VASliceParameterBufferAV1 *va_slice =
			&codec->va_slices[codec->slices_consumed];

		codec->tiles[codec->num_tiles] =
			(struct v4l2_ctrl_av1_tile_group_entry) {
			.tile_offset = base + va_slice->slice_data_offset,
			.tile_size = va_slice->slice_data_size,
			.tile_row = va_slice->tile_row,
			.tile_col = va_slice->tile_column,
		};

		codec->num_tiles++;
		codec->slices_consumed++;
	}

	status = v4l2r_append_output(ctx, buf->data,
				     (size_t)buf->element_size *
				     buf->nb_elements);
	if (status != VA_STATUS_SUCCESS)
		return status;

	return VA_STATUS_SUCCESS;
}

/*
 * refresh_frame_flags for the held frame: exactly the DPB slots that hold its
 * surface in the freshly parsed frame's ref_frame_map (codec->va_pic must
 * already be the new frame's parameters).
 */
static uint8_t av1_pending_refresh(const struct av1_context *codec)
{
	uint8_t refresh = 0;

	for (int i = 0; i < 8; i++)
		if (codec->va_pic.ref_frame_map[i] == codec->pend_current)
			refresh |= 1u << i;

	return refresh;
}

/*
 * Best guess used only when a held frame must be flushed before the next
 * frame's DPB is known (a client that reads a surface without decoding ahead).
 * Key/intra frames refresh the whole DPB; otherwise refresh the slots the
 * frame does not use as references. Computed from the held frame's own
 * parameters at defer time.
 */
static uint8_t av1_fallback_refresh(const struct av1_context *codec)
{
	unsigned int type = codec->va_pic.pic_info_fields.bits.frame_type;
	uint8_t referenced = 0;

	if (type == AV1_KEY_FRAME || type == AV1_INTRA_ONLY_FRAME)
		return 0xff;

	for (int i = 0; i < 7; i++)
		referenced |= 1u << (codec->va_pic.ref_frame_idx[i] & 7);

	return (uint8_t)~referenced;
}

/* Submit the held frame with the given refresh_frame_flags, temporarily
 * pointing the decode engine at its reserved OUTPUT buffer and target. */
static VAStatus av1_submit_pending(struct v4l2r_context *ctx, uint8_t refresh)
{
	struct av1_context *codec = ctx->codec_priv;
	struct v4l2_ext_control controls[4];
	unsigned int count = 0;
	struct v4l2r_picture saved;
	VAStatus status;

	if (!codec->pend_valid)
		return VA_STATUS_SUCCESS;

	codec->pend_frame.refresh_frame_flags = refresh;

	controls[count++] = (struct v4l2_ext_control) {
		.id = V4L2_CID_STATELESS_AV1_SEQUENCE,
		.ptr = &codec->pend_sequence,
		.size = sizeof(codec->pend_sequence),
	};
	controls[count++] = (struct v4l2_ext_control) {
		.id = V4L2_CID_STATELESS_AV1_FRAME,
		.ptr = &codec->pend_frame,
		.size = sizeof(codec->pend_frame),
	};
	controls[count++] = (struct v4l2_ext_control) {
		.id = V4L2_CID_STATELESS_AV1_TILE_GROUP_ENTRY,
		.ptr = codec->pend_tiles,
		.size = sizeof(*codec->pend_tiles) * codec->pend_num_tiles,
	};
	if (codec->pend_has_film_grain) {
		controls[count++] = (struct v4l2_ext_control) {
			.id = V4L2_CID_STATELESS_AV1_FILM_GRAIN,
			.ptr = &codec->pend_film_grain,
			.size = sizeof(codec->pend_film_grain),
		};
	}

	codec->pend_valid = false;
	saved = ctx->pic;
	ctx->pic = codec->pend_pic;
	status = v4l2r_decode(ctx, controls, count, true, true);
	ctx->pic = saved;

	return status;
}

static VAStatus av1_flush(struct v4l2r_context *ctx,
			  struct v4l2r_surface *target)
{
	struct av1_context *codec = ctx->codec_priv;

	if (!codec->pend_valid)
		return VA_STATUS_SUCCESS;
	if (target && codec->pend_pic.target != target)
		return VA_STATUS_SUCCESS;

	return av1_submit_pending(ctx, codec->pend_fallback);
}

static VAStatus av1_render_buffer(struct v4l2r_context *ctx,
				  struct v4l2r_buffer *buf)
{
	struct av1_context *codec = ctx->codec_priv;

	switch (buf->type) {
	case VAPictureParameterBufferType:
		if (v4l2r_buffer_bytes(buf) < sizeof(codec->va_pic))
			return VA_STATUS_ERROR_INVALID_BUFFER;
		codec->va_pic = *(const VADecPictureParameterBufferAV1 *)buf->data;
		codec->have_pic = true;
		/* The previous frame was held back so its refresh_frame_flags
		 * could be recovered from this frame's DPB; submit it now. */
		if (codec->pend_valid) {
			VAStatus status =
				av1_submit_pending(ctx, av1_pending_refresh(codec));
			if (status != VA_STATUS_SUCCESS)
				return status;
		}
		av1_fill_sequence(codec);
		av1_fill_frame(ctx);
		if (codec->has_film_grain)
			av1_fill_film_grain(codec);
		return VA_STATUS_SUCCESS;
	case VASliceParameterBufferType:
		return av1_store_slice_params(codec, buf);
	case VASliceDataBufferType:
		if (!codec->have_pic)
			return VA_STATUS_ERROR_OPERATION_FAILED;
		return av1_process_tiles(ctx, buf);
	default:
		return VA_STATUS_ERROR_UNSUPPORTED_BUFFERTYPE;
	}
}

static VAStatus av1_end_picture(struct v4l2r_context *ctx)
{
	struct av1_context *codec = ctx->codec_priv;

	if (!codec->have_pic || !codec->num_tiles)
		return VA_STATUS_ERROR_OPERATION_FAILED;

	/* Normally the previous frame was submitted when this frame's picture
	 * parameters arrived; if one is somehow still held, flush it first so
	 * frames reach the decoder in order. */
	if (codec->pend_valid) {
		VAStatus status = av1_submit_pending(ctx, codec->pend_fallback);
		if (status != VA_STATUS_SUCCESS)
			return status;
	}

	/* Copy this frame's assembled state into the held slot instead of
	 * submitting it: refresh_frame_flags is only known once the next
	 * frame's ref_frame_map is seen (see struct av1_context). */
	if (v4l2r_array_reserve((void **)&codec->pend_tiles,
				&codec->pend_alloc_tiles, codec->num_tiles,
				sizeof(*codec->pend_tiles)) < 0)
		return VA_STATUS_ERROR_ALLOCATION_FAILED;
	memcpy(codec->pend_tiles, codec->tiles,
	       codec->num_tiles * sizeof(*codec->tiles));
	codec->pend_num_tiles = codec->num_tiles;

	codec->pend_sequence = codec->sequence;
	codec->pend_frame = codec->frame;
	codec->pend_film_grain = codec->film_grain;
	codec->pend_has_film_grain = codec->has_film_grain;
	codec->pend_pic = ctx->pic;
	codec->pend_current = codec->va_pic.current_frame;
	codec->pend_fallback = av1_fallback_refresh(codec);
	codec->pend_valid = true;

	/* Record the order hint now (later frames reference it while this one
	 * is still held), and mark the surface as decoding with its owning
	 * context so a read of it triggers the deferred submit before syncing. */
	if (ctx->pic.target) {
		ctx->pic.target->codec_tag = codec->va_pic.order_hint;
		ctx->pic.target->ctx = ctx;
		ctx->pic.target->status = VASurfaceRendering;
	}

	return VA_STATUS_SUCCESS;
}

static const VAProfile av1_profiles[] = {
	VAProfileAV1Profile0,
};

const struct v4l2r_codec v4l2r_codec_av1 = {
	.name = "av1",
	.pixelformat = V4L2_PIX_FMT_AV1_FRAME,
	.profiles = av1_profiles,
	.nb_profiles = 1,
	.priv_size = sizeof(struct av1_context),
	.init = av1_init,
	.uninit = av1_uninit,
	.begin_picture = av1_begin_picture,
	.render_buffer = av1_render_buffer,
	.end_picture = av1_end_picture,
	.flush = av1_flush,
};

#endif /* HAVE_V4L2_CTRL_AV1 */
