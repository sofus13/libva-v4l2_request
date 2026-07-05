/*
 * VP8 VA-API to V4L2 stateless translation.
 *
 * The V4L2 VP8 interface wants the segment and quantiser values as they
 * appear in the bitstream; VA-API hands out fully resolved per-segment
 * values instead, so those are passed on in absolute mode with the
 * global deltas reconstructed from segment 0.
 *
 * Copyright (C) 2026 Ondrej Jirman <megi@xff.cz>
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <string.h>

#include <linux/videodev2.h>
#include <linux/v4l2-controls.h>

#include "v4l2_request.h"

#include <va/va_dec_vp8.h>

#if HAVE_V4L2_CTRL_VP8

struct vp8_context {
	struct v4l2_ctrl_vp8_frame frame;
	bool have_picture;
	bool keyframe;
};

static VAStatus vp8_begin_picture(struct v4l2r_context *ctx)
{
	struct vp8_context *codec = ctx->codec_priv;

	memset(codec, 0, sizeof(*codec));

	return VA_STATUS_SUCCESS;
}

static void vp8_fill_frame(struct v4l2r_context *ctx,
			   const VAPictureParameterBufferVP8 *va_pic)
{
	struct vp8_context *codec = ctx->codec_priv;
	struct v4l2_ctrl_vp8_frame *frame = &codec->frame;

	codec->keyframe = !va_pic->pic_fields.bits.key_frame;

	*frame = (struct v4l2_ctrl_vp8_frame) {
		.lf = {
			.sharpness_level = va_pic->pic_fields.bits.sharpness_level,
			.level = va_pic->loop_filter_level[0],
		},

		.width = va_pic->frame_width,
		.height = va_pic->frame_height,

		.horizontal_scale = 0,
		.vertical_scale = 0,

		.version = va_pic->pic_fields.bits.version,
		.prob_skip_false = va_pic->prob_skip_false,
		.prob_intra = va_pic->prob_intra,
		.prob_last = va_pic->prob_last,
		.prob_gf = va_pic->prob_gf,

		.coder_state = {
			.range = va_pic->bool_coder_ctx.range,
			.value = va_pic->bool_coder_ctx.value,
			.bit_count = va_pic->bool_coder_ctx.count,
		},
	};

	/*
	 * VA-API resolved the per-segment values already; report them in
	 * absolute mode (no DELTA_VALUE_MODE flag).
	 */
	for (int i = 0; i < 4; i++) {
		frame->segment.lf_update[i] = va_pic->loop_filter_level[i];
		frame->lf.ref_frm_delta[i] = va_pic->loop_filter_deltas_ref_frame[i];
		frame->lf.mb_mode_delta[i] = va_pic->loop_filter_deltas_mode[i];
	}

	for (int i = 0; i < 3; i++)
		frame->segment.segment_probs[i] = va_pic->mb_segment_tree_probs[i];

	if (va_pic->pic_fields.bits.segmentation_enabled) {
		frame->segment.flags |= V4L2_VP8_SEGMENT_FLAG_ENABLED;
		if (va_pic->pic_fields.bits.update_mb_segmentation_map)
			frame->segment.flags |= V4L2_VP8_SEGMENT_FLAG_UPDATE_MAP;
		if (va_pic->pic_fields.bits.update_segment_feature_data)
			frame->segment.flags |= V4L2_VP8_SEGMENT_FLAG_UPDATE_FEATURE_DATA;
	}

	if (va_pic->pic_fields.bits.loop_filter_adj_enable)
		frame->lf.flags |= V4L2_VP8_LF_ADJ_ENABLE;

	if (va_pic->pic_fields.bits.mode_ref_lf_delta_update)
		frame->lf.flags |= V4L2_VP8_LF_DELTA_UPDATE;

	if (va_pic->pic_fields.bits.filter_type)
		frame->lf.flags |= V4L2_VP8_LF_FILTER_TYPE_SIMPLE;

	if (codec->keyframe) {
		/* Fixed key frame mode probabilities per the VP8 spec;
		 * VA-API clients pass the inter frame state instead. */
		static const uint8_t keyframe_y_mode_probs[4] = {
			145, 156, 163, 128
		};
		static const uint8_t keyframe_uv_mode_probs[3] = {
			142, 114, 183
		};

		memcpy(frame->entropy.y_mode_probs, keyframe_y_mode_probs, 4);
		memcpy(frame->entropy.uv_mode_probs, keyframe_uv_mode_probs, 3);
	} else {
		memcpy(frame->entropy.y_mode_probs, va_pic->y_mode_probs, 4);
		memcpy(frame->entropy.uv_mode_probs, va_pic->uv_mode_probs, 3);
	}
	memcpy(frame->entropy.mv_probs, va_pic->mv_probs, sizeof(frame->entropy.mv_probs));

	frame->last_frame_ts =
		v4l2r_surface_timestamp(ctx->drv, va_pic->last_ref_frame);
	frame->golden_frame_ts =
		v4l2r_surface_timestamp(ctx->drv, va_pic->golden_ref_frame);
	frame->alt_frame_ts =
		v4l2r_surface_timestamp(ctx->drv, va_pic->alt_ref_frame);

	if (codec->keyframe)
		frame->flags |= V4L2_VP8_FRAME_FLAG_KEY_FRAME;

	/* VA-API only decodes shown frames. */
	frame->flags |= V4L2_VP8_FRAME_FLAG_SHOW_FRAME;

	if (va_pic->pic_fields.bits.mb_no_coeff_skip)
		frame->flags |= V4L2_VP8_FRAME_FLAG_MB_NO_SKIP_COEFF;

	if (va_pic->pic_fields.bits.sign_bias_golden)
		frame->flags |= V4L2_VP8_FRAME_FLAG_SIGN_BIAS_GOLDEN;

	if (va_pic->pic_fields.bits.sign_bias_alternate)
		frame->flags |= V4L2_VP8_FRAME_FLAG_SIGN_BIAS_ALT;

	codec->have_picture = true;
}

static void vp8_fill_quant(struct vp8_context *codec,
			   const VAIQMatrixBufferVP8 *va_iq)
{
	struct v4l2_ctrl_vp8_frame *frame = &codec->frame;
	const uint16_t (*q)[6] = va_iq->quantization_index;

	frame->quant.y_ac_qi = q[0][0];
	frame->quant.y_dc_delta = (int16_t)q[0][1] - (int16_t)q[0][0];
	frame->quant.y2_dc_delta = (int16_t)q[0][2] - (int16_t)q[0][0];
	frame->quant.y2_ac_delta = (int16_t)q[0][3] - (int16_t)q[0][0];
	frame->quant.uv_dc_delta = (int16_t)q[0][4] - (int16_t)q[0][0];
	frame->quant.uv_ac_delta = (int16_t)q[0][5] - (int16_t)q[0][0];

	for (int i = 0; i < 4; i++)
		frame->segment.quant_update[i] = q[i][0];
}

static void vp8_fill_probs(struct vp8_context *codec,
			   const VAProbabilityDataBufferVP8 *va_probs)
{
	memcpy(codec->frame.entropy.coeff_probs, va_probs->dct_coeff_probs,
	       sizeof(codec->frame.entropy.coeff_probs));
}

static VAStatus vp8_render_slice(struct v4l2r_context *ctx,
				 struct v4l2r_buffer *buf,
				 const VASliceParameterBufferVP8 *va_slice)
{
	struct vp8_context *codec = ctx->codec_priv;
	struct v4l2_ctrl_vp8_frame *frame = &codec->frame;
	/* A VP8 frame has 1 control plus 1, 2, 4 or 8 DCT partitions; clamp the
	 * client-supplied count to the [1, 9] the partition_size[] array and the
	 * dct_part_sizes[] control field can hold. */
	unsigned int nparts = va_slice->num_of_partitions;

	if (nparts < 1)
		nparts = 1;
	else if (nparts > 9)
		nparts = 9;

	/*
	 * The V4L2 control wants the full size of the first (control)
	 * partition, but VA-API reports partition_size[0] with the frame
	 * header bytes already subtracted off (see FFmpeg vaapi_vp8.c:
	 * header_partition_size - (macroblock_offset + 7) / 8). Add them
	 * back so the kernel finds the DCT partitions at the right offset.
	 */
	frame->first_part_header_bits = va_slice->macroblock_offset;
	frame->first_part_size = va_slice->partition_size[0] +
				 (va_slice->macroblock_offset + 7) / 8;
	frame->num_dct_parts = nparts - 1;

	for (unsigned int i = 0; i + 1 < nparts && i < 8; i++)
		frame->dct_part_sizes[i] = va_slice->partition_size[i + 1];

	(void)buf;
	return VA_STATUS_SUCCESS;
}

static VAStatus vp8_render_buffer(struct v4l2r_context *ctx,
				  struct v4l2r_buffer *buf)
{
	switch (buf->type) {
	case VAPictureParameterBufferType:
		if (v4l2r_buffer_bytes(buf) < sizeof(VAPictureParameterBufferVP8))
			return VA_STATUS_ERROR_INVALID_BUFFER;
		vp8_fill_frame(ctx, buf->data);
		return VA_STATUS_SUCCESS;
	case VAIQMatrixBufferType:
		if (v4l2r_buffer_bytes(buf) < sizeof(VAIQMatrixBufferVP8))
			return VA_STATUS_ERROR_INVALID_BUFFER;
		vp8_fill_quant(ctx->codec_priv, buf->data);
		return VA_STATUS_SUCCESS;
	case VAProbabilityBufferType:
		if (v4l2r_buffer_bytes(buf) < sizeof(VAProbabilityDataBufferVP8))
			return VA_STATUS_ERROR_INVALID_BUFFER;
		vp8_fill_probs(ctx->codec_priv, buf->data);
		return VA_STATUS_SUCCESS;
	case VASliceParameterBufferType:
		if (v4l2r_buffer_bytes(buf) < sizeof(VASliceParameterBufferVP8))
			return VA_STATUS_ERROR_INVALID_BUFFER;
		return vp8_render_slice(ctx, buf, buf->data);
	case VASliceDataBufferType: {
		/*
		 * VA-API strips the uncompressed data chunk (the 3-byte frame
		 * tag, plus the start code and dimensions - 10 bytes total - on
		 * key frames) and hands over only the partition data. The V4L2
		 * stateless VP8 interface, and the kernel decoders, expect the
		 * bitstream to start at that chunk: the first data partition
		 * lives at offset 3 (inter) or 10 (key). Rebuild it from the
		 * known frame parameters and prepend it before the partitions.
		 */
		struct vp8_context *codec = ctx->codec_priv;
		struct v4l2_ctrl_vp8_frame *frame = &codec->frame;
		uint8_t tag[10];
		size_t tag_len = 3;
		uint32_t hdr = (codec->keyframe ? 0u : 1u) |
			       ((uint32_t)frame->version << 1) |
			       (1u << 4) /* show_frame */ |
			       (frame->first_part_size << 5);
		VAStatus status;

		tag[0] = hdr & 0xff;
		tag[1] = (hdr >> 8) & 0xff;
		tag[2] = (hdr >> 16) & 0xff;
		if (codec->keyframe) {
			tag[3] = 0x9d;
			tag[4] = 0x01;
			tag[5] = 0x2a;
			tag[6] = frame->width & 0xff;
			tag[7] = (frame->width >> 8) & 0x3f;
			tag[8] = frame->height & 0xff;
			tag[9] = (frame->height >> 8) & 0x3f;
			tag_len = 10;
		}

		status = v4l2r_append_output(ctx, tag, tag_len);
		if (status != VA_STATUS_SUCCESS)
			return status;

		return v4l2r_append_output(ctx, buf->data,
					   (size_t)buf->element_size *
					   buf->nb_elements);
	}
	default:
		return VA_STATUS_ERROR_UNSUPPORTED_BUFFERTYPE;
	}
}

static VAStatus vp8_end_picture(struct v4l2r_context *ctx)
{
	struct vp8_context *codec = ctx->codec_priv;

	struct v4l2_ext_control controls[] = {
		{
			.id = V4L2_CID_STATELESS_VP8_FRAME,
			.ptr = &codec->frame,
			.size = sizeof(codec->frame),
		},
	};

	if (!codec->have_picture)
		return VA_STATUS_ERROR_OPERATION_FAILED;

	return v4l2r_decode(ctx, controls, 1, true, true);
}

static const VAProfile vp8_profiles[] = {
	VAProfileVP8Version0_3,
};

const struct v4l2r_codec v4l2r_codec_vp8 = {
	.name = "vp8",
	.pixelformat = V4L2_PIX_FMT_VP8_FRAME,
	.profiles = vp8_profiles,
	.nb_profiles = 1,
	.priv_size = sizeof(struct vp8_context),
	.begin_picture = vp8_begin_picture,
	.render_buffer = vp8_render_buffer,
	.end_picture = vp8_end_picture,
};

#endif /* HAVE_V4L2_CTRL_VP8 */
