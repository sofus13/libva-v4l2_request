/*
 * H.264 VA-API to V4L2 stateless translation.
 *
 * VA-API does not carry a handful of fields the V4L2 interface requires
 * (nal_ref_idc, idr_pic_id, the bit sizes of the picture order count and
 * dec_ref_pic_marking() syntax); those are recovered by re-parsing the
 * slice header from the bitstream, using the SPS/PPS state VA provides.
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

#if HAVE_V4L2_CTRL_H264

static const uint8_t annexb_start_code[3] = { 0x00, 0x00, 0x01 };

enum {
	SLICE_P = 0, SLICE_B, SLICE_I, SLICE_SP, SLICE_SI,
};

struct h264_slice_info {
	uint32_t nal_ref_idc;
	uint32_t nal_unit_type;
	bool idr;
	uint32_t idr_pic_id;
	uint32_t slice_type;
	uint32_t pic_parameter_set_id;
	uint32_t pic_order_cnt_lsb;
	int32_t delta_pic_order_cnt_bottom;
	int32_t delta_pic_order_cnt0;
	int32_t delta_pic_order_cnt1;
	uint32_t pic_order_cnt_bit_size;
	uint32_t dec_ref_pic_marking_bit_size;
	uint32_t header_bit_size;
	bool num_ref_idx_override;
	uint32_t redundant_pic_cnt;
	bool valid;
};

struct h264_context {
	int64_t decode_mode;
	int64_t start_code;

	VAPictureParameterBufferH264 va_pic;
	bool have_pic;

	struct v4l2_ctrl_h264_sps sps;
	struct v4l2_ctrl_h264_pps pps;
	struct v4l2_ctrl_h264_scaling_matrix scaling_matrix;
	struct v4l2_ctrl_h264_decode_params decode_params;
	struct v4l2_ctrl_h264_slice_params slice_params;
	struct v4l2_ctrl_h264_pred_weights pred_weights;
	bool pred_weights_required;

	VASliceParameterBufferH264 *va_slices;
	unsigned int nb_va_slices;
	unsigned int alloc_va_slices;
	unsigned int slices_consumed;

	bool staged;		/* a slice awaits submission */
	bool staged_first;
	unsigned int num_slices;
	/* VA-API does not carry the PPS default reference counts. They are
	 * learned from any slice that does not override num_ref_idx and kept
	 * here so they survive the per-picture PPS rebuild and can be re-applied
	 * every frame (the hardware uses them for non-overriding slices). */
	bool pps_defaults_known;
	uint8_t num_ref_idx_l0_default_minus1;
	uint8_t num_ref_idx_l1_default_minus1;
};

/*
 * Re-parse the slice header. The parse must track the exact syntax so the
 * bit offsets of the POC and dec_ref_pic_marking() sections come out
 * right; SPS/PPS values are taken from the VA picture parameters and the
 * actual reference counts from the VA slice parameters.
 */
static void h264_parse_slice_header(struct h264_context *codec,
				    const VASliceParameterBufferH264 *va_slice,
				    const uint8_t *data, size_t size,
				    struct h264_slice_info *info)
{
	const VAPictureParameterBufferH264 *pic = &codec->va_pic;
	unsigned int chroma_array_type = pic->seq_fields.bits.chroma_format_idc;
	bool field_pic = pic->pic_fields.bits.field_pic_flag;
	struct v4l2r_bits b;
	uint32_t slice_type;
	size_t mark;

	memset(info, 0, sizeof(*info));

	v4l2r_bits_init(&b, data, size, true);

	v4l2r_bits_bit(&b);				/* forbidden_zero_bit */
	info->nal_ref_idc = v4l2r_bits_read(&b, 2);
	info->nal_unit_type = v4l2r_bits_read(&b, 5);
	info->idr = info->nal_unit_type == 5;

	v4l2r_bits_ue(&b);				/* first_mb_in_slice */
	slice_type = v4l2r_bits_ue(&b);
	info->slice_type = slice_type % 5;
	info->pic_parameter_set_id = v4l2r_bits_ue(&b);

	v4l2r_bits_read(&b, pic->seq_fields.bits.log2_max_frame_num_minus4 + 4);

	if (!pic->seq_fields.bits.frame_mbs_only_flag) {
		if (v4l2r_bits_bit(&b))			/* field_pic_flag */
			v4l2r_bits_bit(&b);		/* bottom_field_flag */
	}

	if (info->idr)
		info->idr_pic_id = v4l2r_bits_ue(&b);

	mark = b.pos;
	if (pic->seq_fields.bits.pic_order_cnt_type == 0) {
		info->pic_order_cnt_lsb = v4l2r_bits_read(&b,
			pic->seq_fields.bits.log2_max_pic_order_cnt_lsb_minus4 + 4);
		if (pic->pic_fields.bits.pic_order_present_flag && !field_pic)
			info->delta_pic_order_cnt_bottom = v4l2r_bits_se(&b);
	} else if (pic->seq_fields.bits.pic_order_cnt_type == 1 &&
		   !pic->seq_fields.bits.delta_pic_order_always_zero_flag) {
		info->delta_pic_order_cnt0 = v4l2r_bits_se(&b);
		if (pic->pic_fields.bits.pic_order_present_flag && !field_pic)
			info->delta_pic_order_cnt1 = v4l2r_bits_se(&b);
	}
	info->pic_order_cnt_bit_size = b.pos - mark;

	if (pic->pic_fields.bits.redundant_pic_cnt_present_flag)
		info->redundant_pic_cnt = v4l2r_bits_ue(&b);

	if (info->slice_type == SLICE_B)
		v4l2r_bits_bit(&b);		/* direct_spatial_mv_pred_flag */

	if (info->slice_type == SLICE_P || info->slice_type == SLICE_SP ||
	    info->slice_type == SLICE_B) {
		info->num_ref_idx_override = v4l2r_bits_bit(&b);
		if (info->num_ref_idx_override) {
			v4l2r_bits_ue(&b);	/* num_ref_idx_l0_active_minus1 */
			if (info->slice_type == SLICE_B)
				v4l2r_bits_ue(&b);
		}
	}

	/* ref_pic_list_modification() */
	if (info->slice_type != SLICE_I && info->slice_type != SLICE_SI) {
		if (v4l2r_bits_bit(&b)) {
			uint32_t idc;
			do {
				idc = v4l2r_bits_ue(&b);
				if (idc == 0 || idc == 1 || idc == 2)
					v4l2r_bits_ue(&b);
			} while (idc != 3 && !b.error);
		}
	}
	if (info->slice_type == SLICE_B) {
		if (v4l2r_bits_bit(&b)) {
			uint32_t idc;
			do {
				idc = v4l2r_bits_ue(&b);
				if (idc == 0 || idc == 1 || idc == 2)
					v4l2r_bits_ue(&b);
			} while (idc != 3 && !b.error);
		}
	}

	/* pred_weight_table() */
	if ((pic->pic_fields.bits.weighted_pred_flag &&
	     (info->slice_type == SLICE_P || info->slice_type == SLICE_SP)) ||
	    (pic->pic_fields.bits.weighted_bipred_idc == 1 &&
	     info->slice_type == SLICE_B)) {
		unsigned int lists = info->slice_type == SLICE_B ? 2 : 1;

		v4l2r_bits_ue(&b);		/* luma_log2_weight_denom */
		if (chroma_array_type)
			v4l2r_bits_ue(&b);	/* chroma_log2_weight_denom */

		for (unsigned int list = 0; list < lists; list++) {
			unsigned int count = list ?
				va_slice->num_ref_idx_l1_active_minus1 + 1 :
				va_slice->num_ref_idx_l0_active_minus1 + 1;

			for (unsigned int i = 0; i < count && !b.error; i++) {
				if (v4l2r_bits_bit(&b)) {
					v4l2r_bits_se(&b);
					v4l2r_bits_se(&b);
				}
				if (chroma_array_type && v4l2r_bits_bit(&b)) {
					for (int j = 0; j < 4; j++)
						v4l2r_bits_se(&b);
				}
			}
		}
	}

	/* dec_ref_pic_marking() */
	mark = b.pos;
	if (info->nal_ref_idc) {
		if (info->idr) {
			v4l2r_bits_bit(&b);	/* no_output_of_prior_pics */
			v4l2r_bits_bit(&b);	/* long_term_reference_flag */
		} else if (v4l2r_bits_bit(&b)) {
			uint32_t op;
			do {
				op = v4l2r_bits_ue(&b);
				if (op == 1 || op == 3)
					v4l2r_bits_ue(&b);
				if (op == 2)
					v4l2r_bits_ue(&b);
				if (op == 3 || op == 6)
					v4l2r_bits_ue(&b);
				if (op == 4)
					v4l2r_bits_ue(&b);
			} while (op != 0 && !b.error);
		}
	}
	info->dec_ref_pic_marking_bit_size = b.pos - mark;

	if (pic->pic_fields.bits.entropy_coding_mode_flag &&
	    info->slice_type != SLICE_I && info->slice_type != SLICE_SI)
		v4l2r_bits_ue(&b);		/* cabac_init_idc */

	v4l2r_bits_se(&b);			/* slice_qp_delta */

	if (info->slice_type == SLICE_SP || info->slice_type == SLICE_SI) {
		if (info->slice_type == SLICE_SP)
			v4l2r_bits_bit(&b);	/* sp_for_switch_flag */
		v4l2r_bits_se(&b);		/* slice_qs_delta */
	}

	if (pic->pic_fields.bits.deblocking_filter_control_present_flag) {
		if (v4l2r_bits_ue(&b) != 1) {
			v4l2r_bits_se(&b);
			v4l2r_bits_se(&b);
		}
	}

	/* Offset in bits to slice_data() from the start of the slice NAL unit,
	 * i.e. including the NAL header byte. cedrus (and the V4L2 interface)
	 * count header_bit_size from bit 0 of the submitted slice buffer, which
	 * begins at the NAL header byte, so this must not exclude it. */
	info->header_bit_size = b.pos;
	info->valid = !b.error;
}

/* --- control fill helpers --- */

static void h264_profile_idc(VAProfile profile, uint8_t *profile_idc,
			     uint8_t *constraint_set_flags)
{
	switch (profile) {
	case VAProfileH264ConstrainedBaseline:
		*profile_idc = 66;
		*constraint_set_flags = 1 << 1;
		break;
	case VAProfileH264Main:
		*profile_idc = 77;
		*constraint_set_flags = 0;
		break;
	default:
		*profile_idc = 100;
		*constraint_set_flags = 0;
		break;
	}
}

static void h264_fill_sps_pps(struct v4l2r_context *ctx,
			      const VAPictureParameterBufferH264 *pic)
{
	struct h264_context *codec = ctx->codec_priv;
	struct v4l2_ctrl_h264_sps *sps = &codec->sps;
	struct v4l2_ctrl_h264_pps *pps = &codec->pps;

	*sps = (struct v4l2_ctrl_h264_sps) {
		.chroma_format_idc = pic->seq_fields.bits.chroma_format_idc,
		.bit_depth_luma_minus8 = pic->bit_depth_luma_minus8,
		.bit_depth_chroma_minus8 = pic->bit_depth_chroma_minus8,
		.log2_max_frame_num_minus4 =
			pic->seq_fields.bits.log2_max_frame_num_minus4,
		.pic_order_cnt_type = pic->seq_fields.bits.pic_order_cnt_type,
		.log2_max_pic_order_cnt_lsb_minus4 =
			pic->seq_fields.bits.log2_max_pic_order_cnt_lsb_minus4,
		.max_num_ref_frames = pic->num_ref_frames,
		.pic_width_in_mbs_minus1 = pic->picture_width_in_mbs_minus1,
		.pic_height_in_map_units_minus1 = pic->picture_height_in_mbs_minus1,
	};

	/* Map units are half the frame height when fields are coded. */
	if (!pic->seq_fields.bits.frame_mbs_only_flag)
		sps->pic_height_in_map_units_minus1 =
			(pic->picture_height_in_mbs_minus1 + 1) / 2 - 1;

	h264_profile_idc(ctx->profile, &sps->profile_idc,
			 &sps->constraint_set_flags);

	if (pic->seq_fields.bits.delta_pic_order_always_zero_flag)
		sps->flags |= V4L2_H264_SPS_FLAG_DELTA_PIC_ORDER_ALWAYS_ZERO;
	if (pic->seq_fields.bits.gaps_in_frame_num_value_allowed_flag)
		sps->flags |= V4L2_H264_SPS_FLAG_GAPS_IN_FRAME_NUM_VALUE_ALLOWED;
	if (pic->seq_fields.bits.frame_mbs_only_flag)
		sps->flags |= V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY;
	if (pic->seq_fields.bits.mb_adaptive_frame_field_flag)
		sps->flags |= V4L2_H264_SPS_FLAG_MB_ADAPTIVE_FRAME_FIELD;
	if (pic->seq_fields.bits.direct_8x8_inference_flag)
		sps->flags |= V4L2_H264_SPS_FLAG_DIRECT_8X8_INFERENCE;

	*pps = (struct v4l2_ctrl_h264_pps) {
		.pic_init_qp_minus26 = pic->pic_init_qp_minus26,
		.pic_init_qs_minus26 = pic->pic_init_qs_minus26,
		.chroma_qp_index_offset = pic->chroma_qp_index_offset,
		.second_chroma_qp_index_offset = pic->second_chroma_qp_index_offset,
		.weighted_bipred_idc = pic->pic_fields.bits.weighted_bipred_idc,
	};

	if (pic->pic_fields.bits.entropy_coding_mode_flag)
		pps->flags |= V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE;
	if (pic->pic_fields.bits.pic_order_present_flag)
		pps->flags |= V4L2_H264_PPS_FLAG_BOTTOM_FIELD_PIC_ORDER_IN_FRAME_PRESENT;
	if (pic->pic_fields.bits.weighted_pred_flag)
		pps->flags |= V4L2_H264_PPS_FLAG_WEIGHTED_PRED;
	if (pic->pic_fields.bits.deblocking_filter_control_present_flag)
		pps->flags |= V4L2_H264_PPS_FLAG_DEBLOCKING_FILTER_CONTROL_PRESENT;
	if (pic->pic_fields.bits.constrained_intra_pred_flag)
		pps->flags |= V4L2_H264_PPS_FLAG_CONSTRAINED_INTRA_PRED;
	if (pic->pic_fields.bits.redundant_pic_cnt_present_flag)
		pps->flags |= V4L2_H264_PPS_FLAG_REDUNDANT_PIC_CNT_PRESENT;
	if (pic->pic_fields.bits.transform_8x8_mode_flag)
		pps->flags |= V4L2_H264_PPS_FLAG_TRANSFORM_8X8_MODE;

	/* Restore the PPS default reference counts learned from a
	 * non-overriding slice; VA-API does not provide them, so without this
	 * they stay 0 and the hardware builds a wrong reference list for every
	 * slice that relies on the default. */
	if (codec->pps_defaults_known) {
		pps->num_ref_idx_l0_default_active_minus1 =
			codec->num_ref_idx_l0_default_minus1;
		pps->num_ref_idx_l1_default_active_minus1 =
			codec->num_ref_idx_l1_default_minus1;
	}

	/* VA-API clients always provide a scaling matrix. */
	pps->flags |= V4L2_H264_PPS_FLAG_SCALING_MATRIX_PRESENT;
}

static void h264_fill_dpb(struct v4l2r_context *ctx,
			  const VAPictureParameterBufferH264 *pic)
{
	struct h264_context *codec = ctx->codec_priv;
	struct v4l2_ctrl_h264_decode_params *decode = &codec->decode_params;
	unsigned int entries = 0;

	for (int i = 0; i < 16; i++) {
		const VAPictureH264 *ref = &pic->ReferenceFrames[i];
		struct v4l2_h264_dpb_entry *entry;

		if (ref->flags & VA_PICTURE_H264_INVALID)
			continue;
		if (ref->picture_id == VA_INVALID_SURFACE)
			continue;
		if (!(ref->flags & (VA_PICTURE_H264_SHORT_TERM_REFERENCE |
				    VA_PICTURE_H264_LONG_TERM_REFERENCE)))
			continue;

		entry = &decode->dpb[entries++];
		entry->reference_ts = v4l2r_surface_timestamp(ctx->drv,
							      ref->picture_id);
		entry->pic_num = ref->frame_idx;
		entry->frame_num = ref->frame_idx;
		entry->flags = V4L2_H264_DPB_ENTRY_FLAG_VALID |
			       V4L2_H264_DPB_ENTRY_FLAG_ACTIVE;

		if (ref->flags & VA_PICTURE_H264_LONG_TERM_REFERENCE)
			entry->flags |= V4L2_H264_DPB_ENTRY_FLAG_LONG_TERM;

		entry->fields = 0;
		if (ref->flags & VA_PICTURE_H264_TOP_FIELD)
			entry->fields |= V4L2_H264_TOP_FIELD_REF;
		if (ref->flags & VA_PICTURE_H264_BOTTOM_FIELD)
			entry->fields |= V4L2_H264_BOTTOM_FIELD_REF;
		if (!entry->fields)
			entry->fields = V4L2_H264_FRAME_REF;
		else
			entry->flags |= V4L2_H264_DPB_ENTRY_FLAG_FIELD;

		entry->top_field_order_cnt = ref->TopFieldOrderCnt;
		entry->bottom_field_order_cnt = ref->BottomFieldOrderCnt;
	}
}

static void h264_fill_ref_list(struct v4l2r_context *ctx,
			       struct v4l2_h264_reference *references,
			       const VAPictureH264 *va_list,
			       unsigned int count)
{
	struct h264_context *codec = ctx->codec_priv;
	struct v4l2_ctrl_h264_decode_params *decode = &codec->decode_params;

	for (unsigned int i = 0; i < count && i < 32; i++) {
		const VAPictureH264 *ref = &va_list[i];
		uint64_t timestamp;

		references[i].index = 0xff;
		references[i].fields = 0;

		if (ref->flags & VA_PICTURE_H264_INVALID)
			continue;
		if (ref->picture_id == VA_INVALID_SURFACE)
			continue;

		timestamp = v4l2r_surface_timestamp(ctx->drv, ref->picture_id);

		for (unsigned int j = 0; j < 16; j++) {
			const struct v4l2_h264_dpb_entry *entry = &decode->dpb[j];

			if ((entry->flags & V4L2_H264_DPB_ENTRY_FLAG_VALID) &&
			    entry->reference_ts == timestamp) {
				references[i].index = j;
				references[i].fields = V4L2_H264_FRAME_REF;
				if (ref->flags & VA_PICTURE_H264_TOP_FIELD)
					references[i].fields = V4L2_H264_TOP_FIELD_REF;
				else if (ref->flags & VA_PICTURE_H264_BOTTOM_FIELD)
					references[i].fields = V4L2_H264_BOTTOM_FIELD_REF;
				break;
			}
		}
	}
}

static void h264_fill_picture(struct v4l2r_context *ctx,
			      const VAPictureParameterBufferH264 *pic)
{
	struct h264_context *codec = ctx->codec_priv;
	struct v4l2_ctrl_h264_decode_params *decode = &codec->decode_params;

	codec->va_pic = *pic;
	codec->have_pic = true;

	h264_fill_sps_pps(ctx, pic);

	*decode = (struct v4l2_ctrl_h264_decode_params) {
		.frame_num = pic->frame_num,
		.top_field_order_cnt =
			(pic->CurrPic.flags & VA_PICTURE_H264_BOTTOM_FIELD) ?
			0 : pic->CurrPic.TopFieldOrderCnt,
		.bottom_field_order_cnt =
			(pic->CurrPic.flags & VA_PICTURE_H264_TOP_FIELD) ?
			0 : pic->CurrPic.BottomFieldOrderCnt,
	};

	if (pic->pic_fields.bits.field_pic_flag)
		decode->flags |= V4L2_H264_DECODE_PARAM_FLAG_FIELD_PIC;
	if (pic->CurrPic.flags & VA_PICTURE_H264_BOTTOM_FIELD)
		decode->flags |= V4L2_H264_DECODE_PARAM_FLAG_BOTTOM_FIELD;

	h264_fill_dpb(ctx, pic);
}

static void h264_fill_scaling_matrix(struct h264_context *codec,
				     const VAIQMatrixBufferH264 *iq)
{
	memcpy(codec->scaling_matrix.scaling_list_4x4, iq->ScalingList4x4,
	       sizeof(codec->scaling_matrix.scaling_list_4x4));
	/* VA-API only carries the two 8x8 lists used with 4:2:0. */
	memcpy(codec->scaling_matrix.scaling_list_8x8[0], iq->ScalingList8x8[0],
	       sizeof(codec->scaling_matrix.scaling_list_8x8[0]));
	memcpy(codec->scaling_matrix.scaling_list_8x8[1], iq->ScalingList8x8[1],
	       sizeof(codec->scaling_matrix.scaling_list_8x8[1]));
}

static void h264_fill_slice_params(struct v4l2r_context *ctx,
				   const VASliceParameterBufferH264 *va_slice,
				   const struct h264_slice_info *info)
{
	struct h264_context *codec = ctx->codec_priv;
	struct v4l2_ctrl_h264_slice_params *slice = &codec->slice_params;
	struct v4l2_ctrl_h264_pred_weights *weights = &codec->pred_weights;

	*slice = (struct v4l2_ctrl_h264_slice_params) {
		/* On a parse failure fall back to the VA bit offset, which is
		 * already the offset to slice_data() from the start of the slice
		 * NAL unit (including the NAL header byte) — the same quantity
		 * header_bit_size carries. */
		.header_bit_size = info->valid ? info->header_bit_size :
				   va_slice->slice_data_bit_offset,
		.first_mb_in_slice = va_slice->first_mb_in_slice,
		.slice_type = va_slice->slice_type % 5,
		.redundant_pic_cnt = info->redundant_pic_cnt,
		.cabac_init_idc = va_slice->cabac_init_idc,
		.slice_qp_delta = va_slice->slice_qp_delta,
		.slice_qs_delta = 0,
		.disable_deblocking_filter_idc =
			va_slice->disable_deblocking_filter_idc,
		.slice_alpha_c0_offset_div2 = va_slice->slice_alpha_c0_offset_div2,
		.slice_beta_offset_div2 = va_slice->slice_beta_offset_div2,
		.num_ref_idx_l0_active_minus1 = va_slice->num_ref_idx_l0_active_minus1,
		.num_ref_idx_l1_active_minus1 = va_slice->num_ref_idx_l1_active_minus1,
	};

	if (va_slice->slice_type % 5 == SLICE_B &&
	    va_slice->direct_spatial_mv_pred_flag)
		slice->flags |= V4L2_H264_SLICE_FLAG_DIRECT_SPATIAL_MV_PRED;

	if (va_slice->slice_type % 5 != SLICE_I &&
	    va_slice->slice_type % 5 != SLICE_SI)
		h264_fill_ref_list(ctx, slice->ref_pic_list0,
				   va_slice->RefPicList0,
				   va_slice->num_ref_idx_l0_active_minus1 + 1);
	if (va_slice->slice_type % 5 == SLICE_B)
		h264_fill_ref_list(ctx, slice->ref_pic_list1,
				   va_slice->RefPicList1,
				   va_slice->num_ref_idx_l1_active_minus1 + 1);

	codec->pred_weights_required =
		V4L2_H264_CTRL_PRED_WEIGHTS_REQUIRED(&codec->pps, slice);

	if (codec->pred_weights_required) {
		memset(weights, 0, sizeof(*weights));
		weights->luma_log2_weight_denom = va_slice->luma_log2_weight_denom;
		weights->chroma_log2_weight_denom = va_slice->chroma_log2_weight_denom;

		for (int i = 0; i <= va_slice->num_ref_idx_l0_active_minus1 && i < 32; i++) {
			weights->weight_factors[0].luma_weight[i] =
				va_slice->luma_weight_l0[i];
			weights->weight_factors[0].luma_offset[i] =
				va_slice->luma_offset_l0[i];
			for (int j = 0; j < 2; j++) {
				weights->weight_factors[0].chroma_weight[i][j] =
					va_slice->chroma_weight_l0[i][j];
				weights->weight_factors[0].chroma_offset[i][j] =
					va_slice->chroma_offset_l0[i][j];
			}
		}

		for (int i = 0; i <= va_slice->num_ref_idx_l1_active_minus1 && i < 32; i++) {
			weights->weight_factors[1].luma_weight[i] =
				va_slice->luma_weight_l1[i];
			weights->weight_factors[1].luma_offset[i] =
				va_slice->luma_offset_l1[i];
			for (int j = 0; j < 2; j++) {
				weights->weight_factors[1].chroma_weight[i][j] =
					va_slice->chroma_weight_l1[i][j];
				weights->weight_factors[1].chroma_offset[i][j] =
					va_slice->chroma_offset_l1[i][j];
			}
		}
	}
}

/* --- request submission --- */

static VAStatus h264_submit(struct v4l2r_context *ctx, bool last_slice)
{
	struct h264_context *codec = ctx->codec_priv;
	struct v4l2_ext_control controls[6];
	unsigned int count = 0;

	controls[count++] = (struct v4l2_ext_control) {
		.id = V4L2_CID_STATELESS_H264_SPS,
		.ptr = &codec->sps,
		.size = sizeof(codec->sps),
	};
	controls[count++] = (struct v4l2_ext_control) {
		.id = V4L2_CID_STATELESS_H264_PPS,
		.ptr = &codec->pps,
		.size = sizeof(codec->pps),
	};
	controls[count++] = (struct v4l2_ext_control) {
		.id = V4L2_CID_STATELESS_H264_SCALING_MATRIX,
		.ptr = &codec->scaling_matrix,
		.size = sizeof(codec->scaling_matrix),
	};
	controls[count++] = (struct v4l2_ext_control) {
		.id = V4L2_CID_STATELESS_H264_DECODE_PARAMS,
		.ptr = &codec->decode_params,
		.size = sizeof(codec->decode_params),
	};

	if (codec->decode_mode == V4L2_STATELESS_H264_DECODE_MODE_SLICE_BASED) {
		controls[count++] = (struct v4l2_ext_control) {
			.id = V4L2_CID_STATELESS_H264_SLICE_PARAMS,
			.ptr = &codec->slice_params,
			.size = sizeof(codec->slice_params),
		};
		if (codec->pred_weights_required) {
			controls[count++] = (struct v4l2_ext_control) {
				.id = V4L2_CID_STATELESS_H264_PRED_WEIGHTS,
				.ptr = &codec->pred_weights,
				.size = sizeof(codec->pred_weights),
			};
		}

		return v4l2r_decode(ctx, controls, count,
				    codec->staged_first, last_slice);
	}

	return v4l2r_decode(ctx, controls, count, true, true);
}

static VAStatus h264_process_slice(struct v4l2r_context *ctx,
				   const VASliceParameterBufferH264 *va_slice,
				   const uint8_t *data, size_t data_size)
{
	struct h264_context *codec = ctx->codec_priv;
	const uint8_t *slice_data = data + va_slice->slice_data_offset;
	struct h264_slice_info info;
	VAStatus status;

	if (!codec->have_pic)
		return VA_STATUS_ERROR_OPERATION_FAILED;

	/* offset and size come from the client; reject a slice that would read
	 * or copy past the slice-data buffer before touching it. */
	if (va_slice->slice_data_offset > data_size ||
	    va_slice->slice_data_size > data_size - va_slice->slice_data_offset)
		return VA_STATUS_ERROR_INVALID_BUFFER;

	/* In slice mode every slice becomes its own request; flush the
	 * previously staged one before starting the next. */
	if (codec->decode_mode == V4L2_STATELESS_H264_DECODE_MODE_SLICE_BASED &&
	    codec->staged) {
		status = h264_submit(ctx, false);
		if (status != VA_STATUS_SUCCESS)
			return status;

		status = v4l2r_picture_next_output(ctx);
		if (status != VA_STATUS_SUCCESS)
			return status;

		codec->staged = false;
	}

	h264_parse_slice_header(codec, va_slice, slice_data,
				va_slice->slice_data_size, &info);

	if (codec->num_slices == 0) {
		struct v4l2_ctrl_h264_decode_params *decode =
			&codec->decode_params;

		codec->pps.pic_parameter_set_id = info.pic_parameter_set_id;

		if (info.valid) {
			decode->nal_ref_idc = info.nal_ref_idc;
			decode->idr_pic_id = info.idr_pic_id;
			decode->pic_order_cnt_lsb = info.pic_order_cnt_lsb;
			decode->delta_pic_order_cnt_bottom =
				info.delta_pic_order_cnt_bottom;
			decode->delta_pic_order_cnt0 = info.delta_pic_order_cnt0;
			decode->delta_pic_order_cnt1 = info.delta_pic_order_cnt1;
			decode->dec_ref_pic_marking_bit_size =
				info.dec_ref_pic_marking_bit_size;
			decode->pic_order_cnt_bit_size =
				info.pic_order_cnt_bit_size;

			if (info.idr)
				decode->flags |= V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC;
#if defined(V4L2_H264_DECODE_PARAM_FLAG_PFRAME)
			if (info.slice_type == SLICE_P ||
			    info.slice_type == SLICE_SP)
				decode->flags |= V4L2_H264_DECODE_PARAM_FLAG_PFRAME;
#endif
#if defined(V4L2_H264_DECODE_PARAM_FLAG_BFRAME)
			if (info.slice_type == SLICE_B)
				decode->flags |= V4L2_H264_DECODE_PARAM_FLAG_BFRAME;
#endif
		} else {
			v4l2r_log("H.264 slice header parse failed, "
				  "decode parameters will be incomplete\n");
		}
	}

	/*
	 * Learn the PPS reference count defaults from any non-IDR P/B slice that
	 * does not override them: such a slice's active counts are exactly the
	 * PPS defaults. VA-API does not provide the defaults and I/IDR slices
	 * carry 0. This runs for every slice (not just the first) so a picture
	 * whose first slice overrides the counts but whose later slices rely on
	 * the defaults still teaches them; keep updating so a mid-stream PPS
	 * change is tracked, and apply to the current PPS which was already
	 * built. L1 is meaningful only in B slices, so learn it there.
	 */
	if (info.valid && !info.num_ref_idx_override) {
		if (info.slice_type == SLICE_P || info.slice_type == SLICE_B) {
			codec->num_ref_idx_l0_default_minus1 =
				va_slice->num_ref_idx_l0_active_minus1;
			codec->pps_defaults_known = true;
			codec->pps.num_ref_idx_l0_default_active_minus1 =
				codec->num_ref_idx_l0_default_minus1;
		}
		if (info.slice_type == SLICE_B) {
			codec->num_ref_idx_l1_default_minus1 =
				va_slice->num_ref_idx_l1_active_minus1;
			codec->pps.num_ref_idx_l1_default_active_minus1 =
				codec->num_ref_idx_l1_default_minus1;
		}
	}

	if (codec->start_code == V4L2_STATELESS_H264_START_CODE_ANNEX_B) {
		status = v4l2r_append_output(ctx, annexb_start_code, 3);
		if (status != VA_STATUS_SUCCESS)
			return status;
	}

	status = v4l2r_append_output(ctx, slice_data, va_slice->slice_data_size);
	if (status != VA_STATUS_SUCCESS)
		return status;

	if (codec->decode_mode == V4L2_STATELESS_H264_DECODE_MODE_SLICE_BASED) {
		h264_fill_slice_params(ctx, va_slice, &info);
		codec->staged = true;
		codec->staged_first = codec->num_slices == 0;
	}

	codec->num_slices++;
	return VA_STATUS_SUCCESS;
}

/* --- codec ops --- */

static VAStatus h264_init(struct v4l2r_context *ctx)
{
	struct h264_context *codec = ctx->codec_priv;
	struct v4l2_ext_control controls[2];
	int ret;

	ret = v4l2r_query_control_default(ctx,
			V4L2_CID_STATELESS_H264_DECODE_MODE, &codec->decode_mode);
	if (ret < 0 ||
	    (codec->decode_mode != V4L2_STATELESS_H264_DECODE_MODE_SLICE_BASED &&
	     codec->decode_mode != V4L2_STATELESS_H264_DECODE_MODE_FRAME_BASED))
		return VA_STATUS_ERROR_OPERATION_FAILED;

	ret = v4l2r_query_control_default(ctx,
			V4L2_CID_STATELESS_H264_START_CODE, &codec->start_code);
	if (ret < 0 ||
	    (codec->start_code != V4L2_STATELESS_H264_START_CODE_NONE &&
	     codec->start_code != V4L2_STATELESS_H264_START_CODE_ANNEX_B))
		return VA_STATUS_ERROR_OPERATION_FAILED;

	controls[0] = (struct v4l2_ext_control) {
		.id = V4L2_CID_STATELESS_H264_DECODE_MODE,
		.value = codec->decode_mode,
	};
	controls[1] = (struct v4l2_ext_control) {
		.id = V4L2_CID_STATELESS_H264_START_CODE,
		.value = codec->start_code,
	};

	if (v4l2r_set_controls(ctx, -1, controls, 2) < 0)
		return VA_STATUS_ERROR_OPERATION_FAILED;

	return VA_STATUS_SUCCESS;
}

static void h264_uninit(struct v4l2r_context *ctx)
{
	struct h264_context *codec = ctx->codec_priv;

	free(codec->va_slices);
	codec->va_slices = NULL;
	codec->alloc_va_slices = 0;
}

static VAStatus h264_begin_picture(struct v4l2r_context *ctx)
{
	struct h264_context *codec = ctx->codec_priv;

	codec->have_pic = false;
	codec->nb_va_slices = 0;
	codec->slices_consumed = 0;
	codec->staged = false;
	codec->staged_first = false;
	codec->num_slices = 0;
	memset(&codec->decode_params, 0, sizeof(codec->decode_params));

	return VA_STATUS_SUCCESS;
}

static VAStatus h264_store_slice_params(struct h264_context *codec,
					struct v4l2r_buffer *buf)
{
	const VASliceParameterBufferH264 *elements = buf->data;
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

static VAStatus h264_render_buffer(struct v4l2r_context *ctx,
				   struct v4l2r_buffer *buf)
{
	struct h264_context *codec = ctx->codec_priv;
	VAStatus status;

	switch (buf->type) {
	case VAPictureParameterBufferType:
		if (v4l2r_buffer_bytes(buf) < sizeof(VAPictureParameterBufferH264))
			return VA_STATUS_ERROR_INVALID_BUFFER;
		h264_fill_picture(ctx, buf->data);
		return VA_STATUS_SUCCESS;
	case VAIQMatrixBufferType:
		if (v4l2r_buffer_bytes(buf) < sizeof(VAIQMatrixBufferH264))
			return VA_STATUS_ERROR_INVALID_BUFFER;
		h264_fill_scaling_matrix(codec, buf->data);
		return VA_STATUS_SUCCESS;
	case VASliceParameterBufferType:
		return h264_store_slice_params(codec, buf);
	case VASliceDataBufferType:
		while (codec->slices_consumed < codec->nb_va_slices) {
			status = h264_process_slice(ctx,
					&codec->va_slices[codec->slices_consumed],
					buf->data, v4l2r_buffer_bytes(buf));
			if (status != VA_STATUS_SUCCESS)
				return status;
			codec->slices_consumed++;
		}
		return VA_STATUS_SUCCESS;
	default:
		return VA_STATUS_ERROR_UNSUPPORTED_BUFFERTYPE;
	}
}

static VAStatus h264_end_picture(struct v4l2r_context *ctx)
{
	struct h264_context *codec = ctx->codec_priv;

	if (!codec->have_pic || !codec->num_slices)
		return VA_STATUS_ERROR_OPERATION_FAILED;

	return h264_submit(ctx, true);
}

static const VAProfile h264_profiles[] = {
	VAProfileH264ConstrainedBaseline,
	VAProfileH264Main,
	VAProfileH264High,
};

const struct v4l2r_codec v4l2r_codec_h264 = {
	.name = "h264",
	.pixelformat = V4L2_PIX_FMT_H264_SLICE,
	.profiles = h264_profiles,
	.nb_profiles = 3,
	.priv_size = sizeof(struct h264_context),
	.init = h264_init,
	.uninit = h264_uninit,
	.begin_picture = h264_begin_picture,
	.render_buffer = h264_render_buffer,
	.end_picture = h264_end_picture,
};

#endif /* HAVE_V4L2_CTRL_H264 */
