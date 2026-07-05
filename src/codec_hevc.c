/*
 * HEVC VA-API to V4L2 stateless translation.
 *
 * As with H.264, some fields wanted by the kernel interface are missing
 * from VA-API (NAL unit type, temporal id, the bit sizes of the short
 * and long term reference picture set syntax, num_delta_pocs of the
 * reference RPS); the slice segment header is re-parsed to recover them.
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
#include "bits.h"

#if HAVE_V4L2_CTRL_HEVC

#include <va/va_dec_hevc.h>

/*
 * Downstream Rockchip control carrying the full SPS short-term reference
 * picture set table (all st_ref_pic_set() definitions). Not always present in
 * the build headers; define the well-known id so we can probe for it.
 */
#ifndef V4L2_CID_STATELESS_HEVC_EXT_SPS_ST_RPS
#define V4L2_CID_STATELESS_HEVC_EXT_SPS_ST_RPS (V4L2_CID_CODEC_STATELESS_BASE + 408)
#endif

static const uint8_t annexb_start_code[3] = { 0x00, 0x00, 0x01 };

#define HEVC_NAL_IDR_W_RADL	19
#define HEVC_NAL_IDR_N_LP	20
#define HEVC_NAL_BLA_W_LP	16
#define HEVC_NAL_RSV_IRAP_VCL23	23

struct hevc_slice_info {
	uint32_t nal_unit_type;
	uint32_t temporal_id_plus1;
	bool no_output_of_prior_pics;
	uint32_t short_term_ref_pic_set_size;
	uint32_t long_term_ref_pic_set_size;
	uint32_t num_delta_pocs_of_ref_rps_idx;
	bool valid;
};

struct hevc_context {
	int64_t decode_mode;
	int64_t start_code;
	unsigned int max_slice_params;
	bool has_scaling_matrix;

	VAPictureParameterBufferHEVC va_pic;
	bool have_pic;
	uint8_t dpb_index_of_va[15];

	struct v4l2_ctrl_hevc_sps sps;
	struct v4l2_ctrl_hevc_pps pps;
	struct v4l2_ctrl_hevc_decode_params decode_params;
	struct v4l2_ctrl_hevc_scaling_matrix scaling_matrix;

	struct v4l2_ctrl_hevc_slice_params *slice_params;
	unsigned int alloc_slice_params;
	unsigned int num_slice_params;

	VASliceParameterBufferHEVC *va_slices;
	unsigned int nb_va_slices;
	unsigned int alloc_va_slices;
	unsigned int slices_consumed;

	bool first_slice;
	unsigned int num_slices;
};

static unsigned int ceil_log2(unsigned int value)
{
	unsigned int bits = 0;

	while ((1u << bits) < value)
		bits++;

	return bits;
}

/*
 * Parse the slice segment header far enough to measure the reference
 * picture set sections. The short term RPS length is cross-checked with
 * the st_rps_bits value provided through VA-API.
 */
static void hevc_parse_slice_header(struct hevc_context *codec,
				    const uint8_t *data, size_t size,
				    struct hevc_slice_info *info)
{
	const VAPictureParameterBufferHEVC *pic = &codec->va_pic;
	bool first_slice_in_pic, dependent_slice = false;
	bool idr, irap;
	struct v4l2r_bits b;
	size_t mark;

	memset(info, 0, sizeof(*info));

	v4l2r_bits_init(&b, data, size, true);

	v4l2r_bits_bit(&b);				/* forbidden_zero_bit */
	info->nal_unit_type = v4l2r_bits_read(&b, 6);
	v4l2r_bits_read(&b, 6);				/* nuh_layer_id */
	info->temporal_id_plus1 = v4l2r_bits_read(&b, 3);

	idr = info->nal_unit_type == HEVC_NAL_IDR_W_RADL ||
	      info->nal_unit_type == HEVC_NAL_IDR_N_LP;
	irap = info->nal_unit_type >= HEVC_NAL_BLA_W_LP &&
	       info->nal_unit_type <= HEVC_NAL_RSV_IRAP_VCL23;

	first_slice_in_pic = v4l2r_bits_bit(&b);

	if (irap)
		info->no_output_of_prior_pics = v4l2r_bits_bit(&b);

	v4l2r_bits_ue(&b);				/* pic_parameter_set_id */

	if (!first_slice_in_pic) {
		unsigned int ctb_log2 =
			pic->log2_min_luma_coding_block_size_minus3 + 3 +
			pic->log2_diff_max_min_luma_coding_block_size;
		unsigned int ctb = 1u << ctb_log2;
		unsigned int pic_size_in_ctbs =
			((pic->pic_width_in_luma_samples + ctb - 1) >> ctb_log2) *
			((pic->pic_height_in_luma_samples + ctb - 1) >> ctb_log2);

		if (pic->slice_parsing_fields.bits.dependent_slice_segments_enabled_flag)
			dependent_slice = v4l2r_bits_bit(&b);

		v4l2r_bits_read(&b, ceil_log2(pic_size_in_ctbs));
	}

	if (dependent_slice) {
		/* RPS syntax only lives in independent slice segments. */
		info->valid = !b.error;
		return;
	}

	v4l2r_bits_skip(&b, pic->num_extra_slice_header_bits);
	v4l2r_bits_ue(&b);				/* slice_type */

	if (pic->slice_parsing_fields.bits.output_flag_present_flag)
		v4l2r_bits_bit(&b);

	if (pic->pic_fields.bits.separate_colour_plane_flag)
		v4l2r_bits_read(&b, 2);

	if (idr) {
		info->valid = !b.error;
		return;
	}

	v4l2r_bits_read(&b, pic->log2_max_pic_order_cnt_lsb_minus4 + 4);

	/*
	 * short_term_ref_pic_set_sps_flag is NOT part of the st_ref_pic_set()
	 * section, so measure after it: the hardware skips over the RPS using
	 * short_term_ref_pic_set_size and accounts for this flag bit
	 * separately. Including it made the value one bit too large and the
	 * hardware started entropy decoding one bit early on every inter frame.
	 */
	if (!v4l2r_bits_bit(&b)) {	/* short_term_ref_pic_set_sps_flag */
		/*
		 * Inline st_ref_pic_set(): VA-API reports its exact bit size in
		 * st_rps_bits. Parse only to recover num_delta_pocs_of_ref_rps_idx
		 * (which VA-API does not carry) in the inter-prediction case, then
		 * resync to the known size so a heuristic mis-parse can never
		 * shift the rest of the slice header.
		 */
		mark = b.pos;

		if (pic->num_short_term_ref_pic_sets && v4l2r_bits_bit(&b)) {
			/* inter_ref_pic_set_prediction_flag == 1 */
			unsigned int count = 0;

			v4l2r_bits_ue(&b);		/* delta_idx_minus1 */
			v4l2r_bits_bit(&b);		/* delta_rps_sign */
			v4l2r_bits_ue(&b);		/* abs_delta_rps_minus1 */

			while (!b.error && b.pos - mark < pic->st_rps_bits) {
				if (!v4l2r_bits_bit(&b) &&
				    b.pos - mark < pic->st_rps_bits)
					v4l2r_bits_bit(&b);
				count++;
			}

			if (count)
				info->num_delta_pocs_of_ref_rps_idx = count - 1;
		}

		if (b.pos - mark < pic->st_rps_bits) {
			/* st_rps_bits is client-supplied; never skip past the end
			 * of the slice data (a bogus value would otherwise spin
			 * the bit reader for billions of no-op iterations). */
			size_t skip = pic->st_rps_bits - (b.pos - mark);
			size_t avail = size * 8;

			v4l2r_bits_skip(&b, skip < avail ? skip : avail);
		}

		info->short_term_ref_pic_set_size = pic->st_rps_bits;
	} else {
		mark = b.pos;
		if (pic->num_short_term_ref_pic_sets > 1)
			v4l2r_bits_read(&b,
					ceil_log2(pic->num_short_term_ref_pic_sets));
		info->short_term_ref_pic_set_size = b.pos - mark;
	}

	mark = b.pos;
	if (pic->slice_parsing_fields.bits.long_term_ref_pics_present_flag) {
		uint32_t num_lt_sps = 0, num_lt_pics;

		if (pic->num_long_term_ref_pic_sps > 0)
			num_lt_sps = v4l2r_bits_ue(&b);
		num_lt_pics = v4l2r_bits_ue(&b);

		for (uint32_t i = 0; i < num_lt_sps + num_lt_pics && !b.error; i++) {
			if (i < num_lt_sps) {
				if (pic->num_long_term_ref_pic_sps > 1)
					v4l2r_bits_read(&b,
						ceil_log2(pic->num_long_term_ref_pic_sps));
			} else {
				v4l2r_bits_read(&b,
					pic->log2_max_pic_order_cnt_lsb_minus4 + 4);
				v4l2r_bits_bit(&b);	/* used_by_curr_pic_lt */
			}

			if (v4l2r_bits_bit(&b))		/* delta_poc_msb_present */
				v4l2r_bits_ue(&b);
		}
	}
	info->long_term_ref_pic_set_size = b.pos - mark;

	info->valid = !b.error;
}

/* --- control fill --- */

static void hevc_fill_sps_pps(struct v4l2r_context *ctx,
			      const VAPictureParameterBufferHEVC *pic)
{
	struct hevc_context *codec = ctx->codec_priv;
	struct v4l2_ctrl_hevc_sps *sps = &codec->sps;
	struct v4l2_ctrl_hevc_pps *pps = &codec->pps;

	*sps = (struct v4l2_ctrl_hevc_sps) {
		.pic_width_in_luma_samples = pic->pic_width_in_luma_samples,
		.pic_height_in_luma_samples = pic->pic_height_in_luma_samples,
		.bit_depth_luma_minus8 = pic->bit_depth_luma_minus8,
		.bit_depth_chroma_minus8 = pic->bit_depth_chroma_minus8,
		.log2_max_pic_order_cnt_lsb_minus4 =
			pic->log2_max_pic_order_cnt_lsb_minus4,
		.sps_max_dec_pic_buffering_minus1 =
			pic->sps_max_dec_pic_buffering_minus1,
		.log2_min_luma_coding_block_size_minus3 =
			pic->log2_min_luma_coding_block_size_minus3,
		.log2_diff_max_min_luma_coding_block_size =
			pic->log2_diff_max_min_luma_coding_block_size,
		.log2_min_luma_transform_block_size_minus2 =
			pic->log2_min_transform_block_size_minus2,
		.log2_diff_max_min_luma_transform_block_size =
			pic->log2_diff_max_min_transform_block_size,
		.max_transform_hierarchy_depth_inter =
			pic->max_transform_hierarchy_depth_inter,
		.max_transform_hierarchy_depth_intra =
			pic->max_transform_hierarchy_depth_intra,
		.pcm_sample_bit_depth_luma_minus1 =
			pic->pcm_sample_bit_depth_luma_minus1,
		.pcm_sample_bit_depth_chroma_minus1 =
			pic->pcm_sample_bit_depth_chroma_minus1,
		.log2_min_pcm_luma_coding_block_size_minus3 =
			pic->log2_min_pcm_luma_coding_block_size_minus3,
		.log2_diff_max_min_pcm_luma_coding_block_size =
			pic->log2_diff_max_min_pcm_luma_coding_block_size,
		.num_short_term_ref_pic_sets = pic->num_short_term_ref_pic_sets,
		.num_long_term_ref_pics_sps = pic->num_long_term_ref_pic_sps,
		.chroma_format_idc = pic->pic_fields.bits.chroma_format_idc,
	};

	if (pic->pic_fields.bits.separate_colour_plane_flag)
		sps->flags |= V4L2_HEVC_SPS_FLAG_SEPARATE_COLOUR_PLANE;
	if (pic->pic_fields.bits.scaling_list_enabled_flag)
		sps->flags |= V4L2_HEVC_SPS_FLAG_SCALING_LIST_ENABLED;
	if (pic->pic_fields.bits.amp_enabled_flag)
		sps->flags |= V4L2_HEVC_SPS_FLAG_AMP_ENABLED;
	if (pic->slice_parsing_fields.bits.sample_adaptive_offset_enabled_flag)
		sps->flags |= V4L2_HEVC_SPS_FLAG_SAMPLE_ADAPTIVE_OFFSET;
	if (pic->pic_fields.bits.pcm_enabled_flag)
		sps->flags |= V4L2_HEVC_SPS_FLAG_PCM_ENABLED;
	if (pic->pic_fields.bits.pcm_loop_filter_disabled_flag)
		sps->flags |= V4L2_HEVC_SPS_FLAG_PCM_LOOP_FILTER_DISABLED;
	if (pic->slice_parsing_fields.bits.long_term_ref_pics_present_flag)
		sps->flags |= V4L2_HEVC_SPS_FLAG_LONG_TERM_REF_PICS_PRESENT;
	if (pic->slice_parsing_fields.bits.sps_temporal_mvp_enabled_flag)
		sps->flags |= V4L2_HEVC_SPS_FLAG_SPS_TEMPORAL_MVP_ENABLED;
	if (pic->pic_fields.bits.strong_intra_smoothing_enabled_flag)
		sps->flags |= V4L2_HEVC_SPS_FLAG_STRONG_INTRA_SMOOTHING_ENABLED;

	*pps = (struct v4l2_ctrl_hevc_pps) {
		.num_extra_slice_header_bits = pic->num_extra_slice_header_bits,
		.num_ref_idx_l0_default_active_minus1 =
			pic->num_ref_idx_l0_default_active_minus1,
		.num_ref_idx_l1_default_active_minus1 =
			pic->num_ref_idx_l1_default_active_minus1,
		.init_qp_minus26 = pic->init_qp_minus26,
		.diff_cu_qp_delta_depth = pic->diff_cu_qp_delta_depth,
		.pps_cb_qp_offset = pic->pps_cb_qp_offset,
		.pps_cr_qp_offset = pic->pps_cr_qp_offset,
		.pps_beta_offset_div2 = pic->pps_beta_offset_div2,
		.pps_tc_offset_div2 = pic->pps_tc_offset_div2,
		.log2_parallel_merge_level_minus2 =
			pic->log2_parallel_merge_level_minus2,
	};

	if (pic->slice_parsing_fields.bits.dependent_slice_segments_enabled_flag)
		pps->flags |= V4L2_HEVC_PPS_FLAG_DEPENDENT_SLICE_SEGMENT_ENABLED;
	if (pic->slice_parsing_fields.bits.output_flag_present_flag)
		pps->flags |= V4L2_HEVC_PPS_FLAG_OUTPUT_FLAG_PRESENT;
	if (pic->pic_fields.bits.sign_data_hiding_enabled_flag)
		pps->flags |= V4L2_HEVC_PPS_FLAG_SIGN_DATA_HIDING_ENABLED;
	if (pic->slice_parsing_fields.bits.cabac_init_present_flag)
		pps->flags |= V4L2_HEVC_PPS_FLAG_CABAC_INIT_PRESENT;
	if (pic->pic_fields.bits.constrained_intra_pred_flag)
		pps->flags |= V4L2_HEVC_PPS_FLAG_CONSTRAINED_INTRA_PRED;
	if (pic->pic_fields.bits.transform_skip_enabled_flag)
		pps->flags |= V4L2_HEVC_PPS_FLAG_TRANSFORM_SKIP_ENABLED;
	if (pic->pic_fields.bits.cu_qp_delta_enabled_flag)
		pps->flags |= V4L2_HEVC_PPS_FLAG_CU_QP_DELTA_ENABLED;
	if (pic->slice_parsing_fields.bits.pps_slice_chroma_qp_offsets_present_flag)
		pps->flags |= V4L2_HEVC_PPS_FLAG_PPS_SLICE_CHROMA_QP_OFFSETS_PRESENT;
	if (pic->pic_fields.bits.weighted_pred_flag)
		pps->flags |= V4L2_HEVC_PPS_FLAG_WEIGHTED_PRED;
	if (pic->pic_fields.bits.weighted_bipred_flag)
		pps->flags |= V4L2_HEVC_PPS_FLAG_WEIGHTED_BIPRED;
	if (pic->pic_fields.bits.transquant_bypass_enabled_flag)
		pps->flags |= V4L2_HEVC_PPS_FLAG_TRANSQUANT_BYPASS_ENABLED;
	if (pic->pic_fields.bits.tiles_enabled_flag)
		pps->flags |= V4L2_HEVC_PPS_FLAG_TILES_ENABLED;
	if (pic->pic_fields.bits.entropy_coding_sync_enabled_flag)
		pps->flags |= V4L2_HEVC_PPS_FLAG_ENTROPY_CODING_SYNC_ENABLED;
	if (pic->pic_fields.bits.loop_filter_across_tiles_enabled_flag)
		pps->flags |= V4L2_HEVC_PPS_FLAG_LOOP_FILTER_ACROSS_TILES_ENABLED;
	if (pic->pic_fields.bits.pps_loop_filter_across_slices_enabled_flag)
		pps->flags |= V4L2_HEVC_PPS_FLAG_PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED;
	if (pic->slice_parsing_fields.bits.deblocking_filter_override_enabled_flag)
		pps->flags |= V4L2_HEVC_PPS_FLAG_DEBLOCKING_FILTER_OVERRIDE_ENABLED;
	if (pic->slice_parsing_fields.bits.pps_disable_deblocking_filter_flag)
		pps->flags |= V4L2_HEVC_PPS_FLAG_PPS_DISABLE_DEBLOCKING_FILTER;
	if (pic->slice_parsing_fields.bits.lists_modification_present_flag)
		pps->flags |= V4L2_HEVC_PPS_FLAG_LISTS_MODIFICATION_PRESENT;
	if (pic->slice_parsing_fields.bits.slice_segment_header_extension_present_flag)
		pps->flags |= V4L2_HEVC_PPS_FLAG_SLICE_SEGMENT_HEADER_EXTENSION_PRESENT;

	/*
	 * VA-API does not expose deblocking_filter_control_present_flag;
	 * derive it: when either override or PPS level disable is set the
	 * control section must have been present.
	 */
	if (pic->slice_parsing_fields.bits.deblocking_filter_override_enabled_flag ||
	    pic->slice_parsing_fields.bits.pps_disable_deblocking_filter_flag ||
	    pic->pps_beta_offset_div2 || pic->pps_tc_offset_div2)
		pps->flags |= V4L2_HEVC_PPS_FLAG_DEBLOCKING_FILTER_CONTROL_PRESENT;

	if (pic->pic_fields.bits.tiles_enabled_flag) {
		pps->num_tile_columns_minus1 = pic->num_tile_columns_minus1;
		pps->num_tile_rows_minus1 = pic->num_tile_rows_minus1;

		/* The VA arrays are column_width_minus1[19] / row_height_minus1[21]
		 * (the last tile size is derived, not signalled); bound the reads
		 * to those sizes rather than to the larger V4L2 arrays. */
		for (int i = 0; i <= pic->num_tile_columns_minus1 && i < 19; i++)
			pps->column_width_minus1[i] = pic->column_width_minus1[i];
		for (int i = 0; i <= pic->num_tile_rows_minus1 && i < 21; i++)
			pps->row_height_minus1[i] = pic->row_height_minus1[i];

		/* Uniform spacing is not signalled by VA-API; tile sizes are
		 * explicit above so leave the flag cleared. */
	}
}

static void hevc_fill_decode_params(struct v4l2r_context *ctx,
				    const VAPictureParameterBufferHEVC *pic)
{
	struct hevc_context *codec = ctx->codec_priv;
	struct v4l2_ctrl_hevc_decode_params *decode = &codec->decode_params;
	unsigned int entries = 0;

	*decode = (struct v4l2_ctrl_hevc_decode_params) {
		.pic_order_cnt_val = pic->CurrPic.pic_order_cnt,
	};

	memset(codec->dpb_index_of_va, 0xff, sizeof(codec->dpb_index_of_va));

	for (int i = 0; i < 15; i++) {
		const VAPictureHEVC *ref = &pic->ReferenceFrames[i];
		struct v4l2_hevc_dpb_entry *entry;

		if (ref->flags & VA_PICTURE_HEVC_INVALID)
			continue;
		if (ref->picture_id == VA_INVALID_SURFACE)
			continue;

		entry = &decode->dpb[entries];
		entry->timestamp = v4l2r_surface_timestamp(ctx->drv,
							   ref->picture_id);
		entry->field_pic = !!(ref->flags & VA_PICTURE_HEVC_FIELD_PIC);
		entry->pic_order_cnt_val = ref->pic_order_cnt;
		entry->flags = 0;
		if (ref->flags & VA_PICTURE_HEVC_LONG_TERM_REFERENCE)
			entry->flags |= V4L2_HEVC_DPB_ENTRY_LONG_TERM_REFERENCE;

		codec->dpb_index_of_va[i] = entries;
		entries++;
	}

	decode->num_active_dpb_entries = entries;

	/*
	 * Rebuild the RPS current lists from the VA reference flags:
	 * before-list ordered by descending POC, after-list by ascending
	 * POC, matching the specification derivation.
	 */
	for (int pass = 0; pass < 3; pass++) {
		static const uint32_t flags[3] = {
			VA_PICTURE_HEVC_RPS_ST_CURR_BEFORE,
			VA_PICTURE_HEVC_RPS_ST_CURR_AFTER,
			VA_PICTURE_HEVC_RPS_LT_CURR,
		};
		uint8_t list[16];
		unsigned int count = 0;

		for (int i = 0; i < 15; i++) {
			const VAPictureHEVC *ref = &pic->ReferenceFrames[i];

			if (codec->dpb_index_of_va[i] == 0xff)
				continue;
			if (!(ref->flags & flags[pass]))
				continue;

			list[count++] = codec->dpb_index_of_va[i];
		}

		/* Sort by POC distance from the current picture. */
		for (unsigned int i = 0; i + 1 < count; i++) {
			for (unsigned int j = i + 1; j < count; j++) {
				int32_t poc_i = decode->dpb[list[i]].pic_order_cnt_val;
				int32_t poc_j = decode->dpb[list[j]].pic_order_cnt_val;
				bool swap = (pass == 0) ? (poc_j > poc_i) :
							  (poc_j < poc_i);
				if (swap) {
					uint8_t tmp = list[i];
					list[i] = list[j];
					list[j] = tmp;
				}
			}
		}

		for (unsigned int i = 0; i < count; i++) {
			switch (pass) {
			case 0:
				decode->poc_st_curr_before[i] = list[i];
				decode->num_poc_st_curr_before++;
				break;
			case 1:
				decode->poc_st_curr_after[i] = list[i];
				decode->num_poc_st_curr_after++;
				break;
			case 2:
				decode->poc_lt_curr[i] = list[i];
				decode->num_poc_lt_curr++;
				break;
			}
		}
	}

	if (pic->slice_parsing_fields.bits.RapPicFlag)
		decode->flags |= V4L2_HEVC_DECODE_PARAM_FLAG_IRAP_PIC;
	if (pic->slice_parsing_fields.bits.IdrPicFlag)
		decode->flags |= V4L2_HEVC_DECODE_PARAM_FLAG_IDR_PIC;
}

static void hevc_fill_scaling_matrix(struct hevc_context *codec,
				     const VAIQMatrixBufferHEVC *iq)
{
	struct v4l2_ctrl_hevc_scaling_matrix *m = &codec->scaling_matrix;

	memcpy(m->scaling_list_4x4, iq->ScalingList4x4, sizeof(m->scaling_list_4x4));
	memcpy(m->scaling_list_8x8, iq->ScalingList8x8, sizeof(m->scaling_list_8x8));
	memcpy(m->scaling_list_16x16, iq->ScalingList16x16, sizeof(m->scaling_list_16x16));
	memcpy(m->scaling_list_32x32, iq->ScalingList32x32, sizeof(m->scaling_list_32x32));
	memcpy(m->scaling_list_dc_coef_16x16, iq->ScalingListDC16x16,
	       sizeof(m->scaling_list_dc_coef_16x16));
	memcpy(m->scaling_list_dc_coef_32x32, iq->ScalingListDC32x32,
	       sizeof(m->scaling_list_dc_coef_32x32));
}

static void hevc_fill_pred_weights(struct v4l2_hevc_pred_weight_table *table,
				   const VASliceParameterBufferHEVC *slice)
{
	table->luma_log2_weight_denom = slice->luma_log2_weight_denom;
	table->delta_chroma_log2_weight_denom = slice->delta_chroma_log2_weight_denom;

	for (int i = 0; i < 15; i++) {
		table->delta_luma_weight_l0[i] = slice->delta_luma_weight_l0[i];
		table->luma_offset_l0[i] = slice->luma_offset_l0[i];
		table->delta_luma_weight_l1[i] = slice->delta_luma_weight_l1[i];
		table->luma_offset_l1[i] = slice->luma_offset_l1[i];
		for (int j = 0; j < 2; j++) {
			table->delta_chroma_weight_l0[i][j] =
				slice->delta_chroma_weight_l0[i][j];
			table->chroma_offset_l0[i][j] = slice->ChromaOffsetL0[i][j];
			table->delta_chroma_weight_l1[i][j] =
				slice->delta_chroma_weight_l1[i][j];
			table->chroma_offset_l1[i][j] = slice->ChromaOffsetL1[i][j];
		}
	}
}

static void hevc_fill_slice_params(struct v4l2r_context *ctx,
				   struct v4l2_ctrl_hevc_slice_params *params,
				   const VASliceParameterBufferHEVC *slice,
				   const struct hevc_slice_info *info,
				   uint32_t data_byte_offset)
{
	struct hevc_context *codec = ctx->codec_priv;
	const VAPictureParameterBufferHEVC *pic = &codec->va_pic;

	uint32_t payload_bytes =
		slice->slice_data_byte_offset <= slice->slice_data_size ?
		slice->slice_data_size - slice->slice_data_byte_offset : 0;

	*params = (struct v4l2_ctrl_hevc_slice_params) {
		.bit_size = payload_bytes * 8,
		.data_byte_offset = data_byte_offset + slice->slice_data_byte_offset,
		.num_entry_point_offsets = 0,

		.nal_unit_type = info->nal_unit_type,
		.nuh_temporal_id_plus1 = info->temporal_id_plus1,

		.slice_type = slice->LongSliceFlags.fields.slice_type,
		.colour_plane_id = slice->LongSliceFlags.fields.color_plane_id,
		.slice_pic_order_cnt = pic->CurrPic.pic_order_cnt,
		.num_ref_idx_l0_active_minus1 = slice->num_ref_idx_l0_active_minus1,
		.num_ref_idx_l1_active_minus1 = slice->num_ref_idx_l1_active_minus1,
		.collocated_ref_idx =
			slice->LongSliceFlags.fields.slice_temporal_mvp_enabled_flag ?
			slice->collocated_ref_idx : 0,
		.five_minus_max_num_merge_cand = slice->five_minus_max_num_merge_cand,
		.slice_qp_delta = slice->slice_qp_delta,
		.slice_cb_qp_offset = slice->slice_cb_qp_offset,
		.slice_cr_qp_offset = slice->slice_cr_qp_offset,
		.slice_beta_offset_div2 = slice->slice_beta_offset_div2,
		.slice_tc_offset_div2 = slice->slice_tc_offset_div2,
		.pic_struct = 0,

		.slice_segment_addr = slice->slice_segment_address,
		.short_term_ref_pic_set_size = info->short_term_ref_pic_set_size,
		.long_term_ref_pic_set_size = info->long_term_ref_pic_set_size,
	};

	if (slice->LongSliceFlags.fields.slice_sao_luma_flag)
		params->flags |= V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_SAO_LUMA;
	if (slice->LongSliceFlags.fields.slice_sao_chroma_flag)
		params->flags |= V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_SAO_CHROMA;
	if (slice->LongSliceFlags.fields.slice_temporal_mvp_enabled_flag)
		params->flags |= V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_TEMPORAL_MVP_ENABLED;
	if (slice->LongSliceFlags.fields.mvd_l1_zero_flag)
		params->flags |= V4L2_HEVC_SLICE_PARAMS_FLAG_MVD_L1_ZERO;
	if (slice->LongSliceFlags.fields.cabac_init_flag)
		params->flags |= V4L2_HEVC_SLICE_PARAMS_FLAG_CABAC_INIT;
	if (slice->LongSliceFlags.fields.collocated_from_l0_flag)
		params->flags |= V4L2_HEVC_SLICE_PARAMS_FLAG_COLLOCATED_FROM_L0;
	if (slice->LongSliceFlags.fields.slice_deblocking_filter_disabled_flag)
		params->flags |= V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_DEBLOCKING_FILTER_DISABLED;
	if (slice->LongSliceFlags.fields.slice_loop_filter_across_slices_enabled_flag)
		params->flags |= V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_LOOP_FILTER_ACROSS_SLICES_ENABLED;
	if (slice->LongSliceFlags.fields.dependent_slice_segment_flag)
		params->flags |= V4L2_HEVC_SLICE_PARAMS_FLAG_DEPENDENT_SLICE_SEGMENT;

	/* VA RefPicList entries index ReferenceFrames[], remap to the DPB. */
	if (slice->LongSliceFlags.fields.slice_type != V4L2_HEVC_SLICE_TYPE_I) {
		for (int i = 0; i <= slice->num_ref_idx_l0_active_minus1 && i < 15; i++) {
			uint8_t va_index = slice->RefPicList[0][i];
			if (va_index < 15 && codec->dpb_index_of_va[va_index] != 0xff)
				params->ref_idx_l0[i] = codec->dpb_index_of_va[va_index];
		}
	}
	if (slice->LongSliceFlags.fields.slice_type == V4L2_HEVC_SLICE_TYPE_B) {
		for (int i = 0; i <= slice->num_ref_idx_l1_active_minus1 && i < 15; i++) {
			uint8_t va_index = slice->RefPicList[1][i];
			if (va_index < 15 && codec->dpb_index_of_va[va_index] != 0xff)
				params->ref_idx_l1[i] = codec->dpb_index_of_va[va_index];
		}
	}

	hevc_fill_pred_weights(&params->pred_weight_table, slice);
}

/* --- submission --- */

static VAStatus hevc_submit(struct v4l2r_context *ctx, bool last_slice)
{
	struct hevc_context *codec = ctx->codec_priv;
	struct v4l2_ext_control controls[5];
	unsigned int count = 0;

	controls[count++] = (struct v4l2_ext_control) {
		.id = V4L2_CID_STATELESS_HEVC_SPS,
		.ptr = &codec->sps,
		.size = sizeof(codec->sps),
	};
	controls[count++] = (struct v4l2_ext_control) {
		.id = V4L2_CID_STATELESS_HEVC_PPS,
		.ptr = &codec->pps,
		.size = sizeof(codec->pps),
	};
	controls[count++] = (struct v4l2_ext_control) {
		.id = V4L2_CID_STATELESS_HEVC_DECODE_PARAMS,
		.ptr = &codec->decode_params,
		.size = sizeof(codec->decode_params),
	};

	if (codec->has_scaling_matrix) {
		controls[count++] = (struct v4l2_ext_control) {
			.id = V4L2_CID_STATELESS_HEVC_SCALING_MATRIX,
			.ptr = &codec->scaling_matrix,
			.size = sizeof(codec->scaling_matrix),
		};
	}

	if (codec->max_slice_params && codec->num_slice_params) {
		unsigned int nb = codec->num_slice_params;

		if (nb > codec->max_slice_params)
			nb = codec->max_slice_params;

		controls[count++] = (struct v4l2_ext_control) {
			.id = V4L2_CID_STATELESS_HEVC_SLICE_PARAMS,
			.ptr = codec->slice_params,
			.size = sizeof(*codec->slice_params) * nb,
		};
	}

	if (codec->decode_mode == V4L2_STATELESS_HEVC_DECODE_MODE_SLICE_BASED)
		return v4l2r_decode(ctx, controls, count,
				    codec->first_slice, last_slice);

	return v4l2r_decode(ctx, controls, count, true, true);
}

static VAStatus hevc_process_slice(struct v4l2r_context *ctx,
				   const VASliceParameterBufferHEVC *va_slice,
				   const uint8_t *data, size_t data_size)
{
	struct hevc_context *codec = ctx->codec_priv;
	const uint8_t *slice_data = data + va_slice->slice_data_offset;
	struct hevc_slice_info info;
	uint32_t data_byte_offset;
	VAStatus status;

	if (!codec->have_pic)
		return VA_STATUS_ERROR_OPERATION_FAILED;

	/* offset and size are client-supplied; reject a slice that would read
	 * past the slice-data buffer before parsing or copying it. */
	if (va_slice->slice_data_offset > data_size ||
	    va_slice->slice_data_size > data_size - va_slice->slice_data_offset)
		return VA_STATUS_ERROR_INVALID_BUFFER;

	/* Flush a full batch of slices as an intermediate request. */
	if (codec->decode_mode == V4L2_STATELESS_HEVC_DECODE_MODE_SLICE_BASED &&
	    codec->max_slice_params &&
	    codec->num_slice_params >= codec->max_slice_params) {
		status = hevc_submit(ctx, false);
		if (status != VA_STATUS_SUCCESS)
			return status;

		status = v4l2r_picture_next_output(ctx);
		if (status != VA_STATUS_SUCCESS)
			return status;

		codec->num_slice_params = 0;
		codec->first_slice = false;
	}

	hevc_parse_slice_header(codec, slice_data, va_slice->slice_data_size,
				&info);

	if (codec->num_slices == 0 && info.valid) {
		codec->decode_params.short_term_ref_pic_set_size =
			info.short_term_ref_pic_set_size;
		codec->decode_params.long_term_ref_pic_set_size =
			info.long_term_ref_pic_set_size;
#if HAVE_HEVC_DECODE_PARAMS_NUM_DELTA_POCS
		codec->decode_params.num_delta_pocs_of_ref_rps_idx =
			info.num_delta_pocs_of_ref_rps_idx;
#endif
		if (info.no_output_of_prior_pics)
			codec->decode_params.flags |=
				V4L2_HEVC_DECODE_PARAM_FLAG_NO_OUTPUT_OF_PRIOR;
	}

	if (codec->start_code == V4L2_STATELESS_HEVC_START_CODE_ANNEX_B) {
		status = v4l2r_append_output(ctx, annexb_start_code, 3);
		if (status != VA_STATUS_SUCCESS)
			return status;
	}

	data_byte_offset = ctx->pic.output->bytesused;

	status = v4l2r_append_output(ctx, slice_data, va_slice->slice_data_size);
	if (status != VA_STATUS_SUCCESS)
		return status;

	if (codec->max_slice_params) {
		if (v4l2r_array_reserve((void **)&codec->slice_params,
					&codec->alloc_slice_params,
					codec->num_slice_params + 1,
					sizeof(*codec->slice_params)) < 0)
			return VA_STATUS_ERROR_ALLOCATION_FAILED;

		hevc_fill_slice_params(ctx,
				       &codec->slice_params[codec->num_slice_params],
				       va_slice, &info, data_byte_offset);
		codec->num_slice_params++;
	}

	codec->num_slices++;
	return VA_STATUS_SUCCESS;
}

/* --- codec ops --- */

static VAStatus hevc_init(struct v4l2r_context *ctx)
{
	struct hevc_context *codec = ctx->codec_priv;
	struct v4l2_query_ext_ctrl scaling_matrix = {
		.id = V4L2_CID_STATELESS_HEVC_SCALING_MATRIX,
	};
	struct v4l2_query_ext_ctrl slice_params = {
		.id = V4L2_CID_STATELESS_HEVC_SLICE_PARAMS,
	};
	struct v4l2_query_ext_ctrl ext_sps_st_rps = {
		.id = V4L2_CID_STATELESS_HEVC_EXT_SPS_ST_RPS,
	};
	struct v4l2_ext_control controls[2];
	int ret;

	/*
	 * Some decoders (Rockchip rkvdec2 / vdpu38x on e.g. RK3588) decode HEVC
	 * fully statelessly and require the complete SPS short-term reference
	 * picture set table via V4L2_CID_STATELESS_HEVC_EXT_SPS_ST_RPS; without
	 * it they silently emit corrupted inter frames. That table is the raw
	 * SPS st_ref_pic_set() syntax, which VA-API never exposes (the picture
	 * parameter buffer only carries the set count) and which cannot be
	 * reconstructed from the per-frame data we do get. If the decoder
	 * advertises that control we cannot drive it correctly, so decline HEVC
	 * here and let the caller fall back (to software, or to another decoder
	 * once codec-to-device assignment is prioritised).
	 */
	if (!v4l2r_query_control(ctx, &ext_sps_st_rps)) {
		v4l2r_log("hevc: decoder requires SPS RPS tables (EXT_SPS_ST_RPS) "
			  "that VA-API cannot supply; declining HEVC\n");
		return VA_STATUS_ERROR_UNSUPPORTED_PROFILE;
	}

	ret = v4l2r_query_control_default(ctx,
			V4L2_CID_STATELESS_HEVC_DECODE_MODE, &codec->decode_mode);
	if (ret < 0 ||
	    (codec->decode_mode != V4L2_STATELESS_HEVC_DECODE_MODE_SLICE_BASED &&
	     codec->decode_mode != V4L2_STATELESS_HEVC_DECODE_MODE_FRAME_BASED))
		return VA_STATUS_ERROR_OPERATION_FAILED;

	ret = v4l2r_query_control_default(ctx,
			V4L2_CID_STATELESS_HEVC_START_CODE, &codec->start_code);
	if (ret < 0 ||
	    (codec->start_code != V4L2_STATELESS_HEVC_START_CODE_NONE &&
	     codec->start_code != V4L2_STATELESS_HEVC_START_CODE_ANNEX_B))
		return VA_STATUS_ERROR_OPERATION_FAILED;

	codec->has_scaling_matrix = !v4l2r_query_control(ctx, &scaling_matrix);

	if (!v4l2r_query_control(ctx, &slice_params))
		codec->max_slice_params = slice_params.dims[0] ?
					  slice_params.dims[0] : 1;
	else
		codec->max_slice_params = 0;

	controls[0] = (struct v4l2_ext_control) {
		.id = V4L2_CID_STATELESS_HEVC_DECODE_MODE,
		.value = codec->decode_mode,
	};
	controls[1] = (struct v4l2_ext_control) {
		.id = V4L2_CID_STATELESS_HEVC_START_CODE,
		.value = codec->start_code,
	};

	if (v4l2r_set_controls(ctx, -1, controls, 2) < 0)
		return VA_STATUS_ERROR_OPERATION_FAILED;

	return VA_STATUS_SUCCESS;
}

static void hevc_uninit(struct v4l2r_context *ctx)
{
	struct hevc_context *codec = ctx->codec_priv;

	free(codec->slice_params);
	free(codec->va_slices);
	codec->slice_params = NULL;
	codec->va_slices = NULL;
	codec->alloc_slice_params = 0;
	codec->alloc_va_slices = 0;
}

static VAStatus hevc_begin_picture(struct v4l2r_context *ctx)
{
	struct hevc_context *codec = ctx->codec_priv;

	codec->have_pic = false;
	codec->nb_va_slices = 0;
	codec->slices_consumed = 0;
	codec->num_slice_params = 0;
	codec->num_slices = 0;
	codec->first_slice = true;
	memset(&codec->decode_params, 0, sizeof(codec->decode_params));

	return VA_STATUS_SUCCESS;
}

static VAStatus hevc_store_slice_params(struct hevc_context *codec,
					struct v4l2r_buffer *buf)
{
	const VASliceParameterBufferHEVC *elements = buf->data;
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

static VAStatus hevc_render_buffer(struct v4l2r_context *ctx,
				   struct v4l2r_buffer *buf)
{
	struct hevc_context *codec = ctx->codec_priv;
	VAStatus status;

	switch (buf->type) {
	case VAPictureParameterBufferType:
		if (v4l2r_buffer_bytes(buf) < sizeof(codec->va_pic))
			return VA_STATUS_ERROR_INVALID_BUFFER;
		codec->va_pic = *(const VAPictureParameterBufferHEVC *)buf->data;
		codec->have_pic = true;
		hevc_fill_sps_pps(ctx, &codec->va_pic);
		hevc_fill_decode_params(ctx, &codec->va_pic);
		return VA_STATUS_SUCCESS;
	case VAIQMatrixBufferType:
		if (v4l2r_buffer_bytes(buf) < sizeof(VAIQMatrixBufferHEVC))
			return VA_STATUS_ERROR_INVALID_BUFFER;
		hevc_fill_scaling_matrix(codec, buf->data);
		return VA_STATUS_SUCCESS;
	case VASliceParameterBufferType:
		return hevc_store_slice_params(codec, buf);
	case VASliceDataBufferType:
		while (codec->slices_consumed < codec->nb_va_slices) {
			status = hevc_process_slice(ctx,
					&codec->va_slices[codec->slices_consumed],
					buf->data, v4l2r_buffer_bytes(buf));
			if (status != VA_STATUS_SUCCESS)
				return status;
			codec->slices_consumed++;
		}
		return VA_STATUS_SUCCESS;
	default:
		/* Ignore optional buffers such as subset parameters. */
		return VA_STATUS_SUCCESS;
	}
}

static VAStatus hevc_end_picture(struct v4l2r_context *ctx)
{
	struct hevc_context *codec = ctx->codec_priv;

	if (!codec->have_pic || !codec->num_slices)
		return VA_STATUS_ERROR_OPERATION_FAILED;

	return hevc_submit(ctx, true);
}

static const VAProfile hevc_profiles[] = {
	VAProfileHEVCMain,
	VAProfileHEVCMain10,
};

const struct v4l2r_codec v4l2r_codec_hevc = {
	.name = "hevc",
	.pixelformat = V4L2_PIX_FMT_HEVC_SLICE,
	.profiles = hevc_profiles,
	.nb_profiles = 2,
	.priv_size = sizeof(struct hevc_context),
	.init = hevc_init,
	.uninit = hevc_uninit,
	.begin_picture = hevc_begin_picture,
	.render_buffer = hevc_render_buffer,
	.end_picture = hevc_end_picture,
};

#endif /* HAVE_V4L2_CTRL_HEVC */
