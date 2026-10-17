/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Copyright (C) 2026 Ondrej Jirman <megi@xff.cz>
 */

#ifndef CODEC_H
#define CODEC_H

#include "v4l2_request.h"

/*
 * Codec backend: translates VA parameter buffers into V4L2 stateless
 * controls and drives the request submission through the decode engine.
 */
struct v4l2r_codec {
	const char *name;
	uint32_t pixelformat;
	const VAProfile *profiles;
	unsigned int nb_profiles;
	size_t priv_size;

	/* Called once after the OUTPUT format is set (device is chosen but
	 * not yet streaming); query decode mode and friends here. */
	VAStatus (*init)(struct v4l2r_context *ctx);
	void (*uninit)(struct v4l2r_context *ctx);
	VAStatus (*begin_picture)(struct v4l2r_context *ctx);
	VAStatus (*render_buffer)(struct v4l2r_context *ctx,
				  struct v4l2r_buffer *buf);
	VAStatus (*end_picture)(struct v4l2r_context *ctx);

	/* Optional: some codecs (AV1) hold the most recent frame back by one
	 * to reconstruct state that is only known once the next frame arrives.
	 * Called before a surface is read or synced so any decode still held
	 * for that surface is submitted first. A NULL target matches any held
	 * frame, flushing it unconditionally. */
	VAStatus (*flush)(struct v4l2r_context *ctx,
			  struct v4l2r_surface *target);
};

extern const struct v4l2r_codec v4l2r_codec_mpeg2;
extern const struct v4l2r_codec v4l2r_codec_h264;
extern const struct v4l2r_codec v4l2r_codec_hevc;
extern const struct v4l2r_codec v4l2r_codec_vp8;
extern const struct v4l2r_codec v4l2r_codec_vp9;
extern const struct v4l2r_codec v4l2r_codec_av1;

#endif /* CODEC_H */
