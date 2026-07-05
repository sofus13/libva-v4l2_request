/*
 * MPEG-2 VA-API to V4L2 stateless translation.
 *
 * Copyright (C) 2026 Ondrej Jirman <megi@xff.cz>
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <string.h>

#include <linux/videodev2.h>
#include <linux/v4l2-controls.h>

#include "v4l2_request.h"

#if HAVE_V4L2_CTRL_MPEG2

struct mpeg2_context {
	struct v4l2_ctrl_mpeg2_sequence sequence;
	struct v4l2_ctrl_mpeg2_picture picture;
	struct v4l2_ctrl_mpeg2_quantisation quantisation;
	bool have_picture;
	bool quant_initialised;
};

/*
 * ISO/IEC 13818-2 default intra quantisation matrix in zigzag scan order,
 * which is the order both VA-API (VAIQMatrixBufferMPEG2) and the V4L2
 * control expect. This is the spec's default matrix run through the zigzag
 * scan; it is only used for clients that never load a matrix (FFmpeg and
 * GStreamer always load one every frame).
 */
static const uint8_t default_intra_matrix[64] = {
	 8, 16, 16, 19, 16, 19, 22, 22,
	22, 22, 22, 22, 26, 24, 26, 27,
	27, 27, 26, 26, 26, 26, 27, 27,
	27, 29, 29, 29, 34, 34, 34, 29,
	29, 29, 27, 27, 29, 29, 32, 32,
	34, 34, 37, 38, 37, 35, 35, 34,
	35, 38, 38, 40, 40, 40, 48, 48,
	46, 46, 56, 56, 58, 69, 69, 83
};

static void mpeg2_default_quantisation(struct v4l2_ctrl_mpeg2_quantisation *q)
{
	memcpy(q->intra_quantiser_matrix, default_intra_matrix, 64);
	memcpy(q->chroma_intra_quantiser_matrix, default_intra_matrix, 64);
	memset(q->non_intra_quantiser_matrix, 16, 64);
	memset(q->chroma_non_intra_quantiser_matrix, 16, 64);
}

static VAStatus mpeg2_begin_picture(struct v4l2r_context *ctx)
{
	struct mpeg2_context *codec = ctx->codec_priv;

	/* MPEG-2 quantisation matrices persist until reloaded or the sequence
	 * ends, so seed them with the defaults only once and let subsequent
	 * loads (mpeg2_fill_quantisation) update them; do not reset per frame. */
	if (!codec->quant_initialised) {
		mpeg2_default_quantisation(&codec->quantisation);
		codec->quant_initialised = true;
	}

	codec->have_picture = false;

	return VA_STATUS_SUCCESS;
}

static void mpeg2_fill_picture(struct v4l2r_context *ctx,
			       const VAPictureParameterBufferMPEG2 *va_pic)
{
	struct mpeg2_context *codec = ctx->codec_priv;

	codec->sequence = (struct v4l2_ctrl_mpeg2_sequence) {
		.horizontal_size = va_pic->horizontal_size,
		.vertical_size = va_pic->vertical_size,
		.vbv_buffer_size = ctx->pic.output ? ctx->pic.output->size : 0,
		.profile_and_level_indication = 0,
		.chroma_format = 1, /* 4:2:0, the only format VA-API decodes */
	};

	/*
	 * The sequence PROGRESSIVE flag reflects progressive_sequence from the
	 * sequence extension, which VA-API does not expose (only the per-picture
	 * progressive_frame bit, handled via V4L2_MPEG2_PIC_FLAG_PROGRESSIVE
	 * below). Driving the sequence flag from progressive_frame would flip it
	 * per frame and set it on exactly the frames where it is wrong, so leave
	 * it clear.
	 */

	codec->picture = (struct v4l2_ctrl_mpeg2_picture) {
		.picture_coding_type = va_pic->picture_coding_type,
		.f_code = {
			{
				(va_pic->f_code >> 12) & 0xf,
				(va_pic->f_code >>  8) & 0xf,
			},
			{
				(va_pic->f_code >>  4) & 0xf,
				(va_pic->f_code >>  0) & 0xf,
			},
		},
		.picture_structure =
			va_pic->picture_coding_extension.bits.picture_structure,
		.intra_dc_precision =
			va_pic->picture_coding_extension.bits.intra_dc_precision,
	};

	if (va_pic->picture_coding_extension.bits.top_field_first)
		codec->picture.flags |= V4L2_MPEG2_PIC_FLAG_TOP_FIELD_FIRST;

	if (va_pic->picture_coding_extension.bits.frame_pred_frame_dct)
		codec->picture.flags |= V4L2_MPEG2_PIC_FLAG_FRAME_PRED_DCT;

	if (va_pic->picture_coding_extension.bits.concealment_motion_vectors)
		codec->picture.flags |= V4L2_MPEG2_PIC_FLAG_CONCEALMENT_MV;

	if (va_pic->picture_coding_extension.bits.intra_vlc_format)
		codec->picture.flags |= V4L2_MPEG2_PIC_FLAG_INTRA_VLC;

	if (va_pic->picture_coding_extension.bits.q_scale_type)
		codec->picture.flags |= V4L2_MPEG2_PIC_FLAG_Q_SCALE_TYPE;

	if (va_pic->picture_coding_extension.bits.alternate_scan)
		codec->picture.flags |= V4L2_MPEG2_PIC_FLAG_ALT_SCAN;

	if (va_pic->picture_coding_extension.bits.repeat_first_field)
		codec->picture.flags |= V4L2_MPEG2_PIC_FLAG_REPEAT_FIRST;

	if (va_pic->picture_coding_extension.bits.progressive_frame)
		codec->picture.flags |= V4L2_MPEG2_PIC_FLAG_PROGRESSIVE;

	switch (va_pic->picture_coding_type) {
	case 3: /* B */
		codec->picture.backward_ref_ts =
			v4l2r_surface_timestamp(ctx->drv,
					va_pic->backward_reference_picture);
		/* fall-through */
	case 2: /* P */
		codec->picture.forward_ref_ts =
			v4l2r_surface_timestamp(ctx->drv,
					va_pic->forward_reference_picture);
	}

	codec->have_picture = true;
}

static void mpeg2_fill_quantisation(struct mpeg2_context *codec,
				    const VAIQMatrixBufferMPEG2 *va_iq)
{
	struct v4l2_ctrl_mpeg2_quantisation *q = &codec->quantisation;

	if (va_iq->load_intra_quantiser_matrix)
		memcpy(q->intra_quantiser_matrix,
		       va_iq->intra_quantiser_matrix, 64);

	if (va_iq->load_non_intra_quantiser_matrix)
		memcpy(q->non_intra_quantiser_matrix,
		       va_iq->non_intra_quantiser_matrix, 64);

	if (va_iq->load_chroma_intra_quantiser_matrix)
		memcpy(q->chroma_intra_quantiser_matrix,
		       va_iq->chroma_intra_quantiser_matrix, 64);
	else if (va_iq->load_intra_quantiser_matrix)
		memcpy(q->chroma_intra_quantiser_matrix,
		       va_iq->intra_quantiser_matrix, 64);

	if (va_iq->load_chroma_non_intra_quantiser_matrix)
		memcpy(q->chroma_non_intra_quantiser_matrix,
		       va_iq->chroma_non_intra_quantiser_matrix, 64);
	else if (va_iq->load_non_intra_quantiser_matrix)
		memcpy(q->chroma_non_intra_quantiser_matrix,
		       va_iq->non_intra_quantiser_matrix, 64);
}

static VAStatus mpeg2_render_buffer(struct v4l2r_context *ctx,
				    struct v4l2r_buffer *buf)
{
	struct mpeg2_context *codec = ctx->codec_priv;

	switch (buf->type) {
	case VAPictureParameterBufferType:
		if (v4l2r_buffer_bytes(buf) < sizeof(VAPictureParameterBufferMPEG2))
			return VA_STATUS_ERROR_INVALID_BUFFER;
		mpeg2_fill_picture(ctx, buf->data);
		return VA_STATUS_SUCCESS;
	case VAIQMatrixBufferType:
		if (v4l2r_buffer_bytes(buf) < sizeof(VAIQMatrixBufferMPEG2))
			return VA_STATUS_ERROR_INVALID_BUFFER;
		mpeg2_fill_quantisation(codec, buf->data);
		return VA_STATUS_SUCCESS;
	case VASliceParameterBufferType:
		/* The V4L2 MPEG-2 interface has no slice parameters. */
		return VA_STATUS_SUCCESS;
	case VASliceDataBufferType:
		return v4l2r_append_output(ctx, buf->data,
					   (size_t)buf->element_size *
					   buf->nb_elements);
	default:
		return VA_STATUS_ERROR_UNSUPPORTED_BUFFERTYPE;
	}
}

static VAStatus mpeg2_end_picture(struct v4l2r_context *ctx)
{
	struct mpeg2_context *codec = ctx->codec_priv;

	struct v4l2_ext_control controls[] = {
		{
			.id = V4L2_CID_STATELESS_MPEG2_SEQUENCE,
			.ptr = &codec->sequence,
			.size = sizeof(codec->sequence),
		},
		{
			.id = V4L2_CID_STATELESS_MPEG2_PICTURE,
			.ptr = &codec->picture,
			.size = sizeof(codec->picture),
		},
		{
			.id = V4L2_CID_STATELESS_MPEG2_QUANTISATION,
			.ptr = &codec->quantisation,
			.size = sizeof(codec->quantisation),
		},
	};

	if (!codec->have_picture)
		return VA_STATUS_ERROR_OPERATION_FAILED;

	return v4l2r_decode(ctx, controls, 3, true, true);
}

static const VAProfile mpeg2_profiles[] = {
	VAProfileMPEG2Simple,
	VAProfileMPEG2Main,
};

const struct v4l2r_codec v4l2r_codec_mpeg2 = {
	.name = "mpeg2",
	.pixelformat = V4L2_PIX_FMT_MPEG2_SLICE,
	.profiles = mpeg2_profiles,
	.nb_profiles = 2,
	.priv_size = sizeof(struct mpeg2_context),
	.begin_picture = mpeg2_begin_picture,
	.render_buffer = mpeg2_render_buffer,
	.end_picture = mpeg2_end_picture,
};

#endif /* HAVE_V4L2_CTRL_MPEG2 */
