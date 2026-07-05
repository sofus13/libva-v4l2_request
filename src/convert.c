/*
 * Format converter chain: decoder-only CAPTURE formats (the packed 10-bit
 * NV15 produced by Rockchip decoders) are converted to NV12 with a plain
 * V4L2 mem2mem device (the Rockchip RGA), fully in hardware.
 *
 * The decoder's CAPTURE buffer is imported into the converter's OUTPUT
 * queue and the surface's standalone NV12 backing into its CAPTURE queue,
 * both as dma-bufs, so no pixel ever passes through the CPU. Conversions
 * are kicked as soon as a decode finishes (from the CAPTURE dequeue path)
 * and overlap subsequent decodes; readers wait for the result through
 * v4l2r_surface_ready().
 *
 * The same converter also backs the VAEntrypointVideoProc implementation
 * at the end of this file: surface-to-surface blits with rotation,
 * mirroring and scaling (VAProcPipelineParameterBuffer). When the source
 * surface sits behind a conversion chain, the blit reads the raw decoder
 * output directly, folding the 10-to-8-bit downconversion and the rotation
 * into a single pass.
 *
 * Copyright (C) 2026 Ondrej Jirman <megi@xff.cz>
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/videodev2.h>

#include "v4l2_request.h"

#define V4L2R_CONVERT_TIMEOUT_MS 2000

/* Dimension limits of the converter's crop/compose rectangles (the mainline
 * Rockchip RGA driver rejects anything smaller than 34 pixels and frames
 * larger than 8192). */
#define V4L2R_VPP_MIN_DIM 34
#define V4L2R_VPP_MAX_DIM 8192

VAStatus v4l2r_convert_setup(struct v4l2r_context *ctx)
{
	struct v4l2r_driver *drv = ctx->drv;
	struct v4l2r_convert *conv;
	struct v4l2_requestbuffers reqbufs;
	enum v4l2_buf_type type;
	uint32_t src_pixelformat = v4l2r_format_pixelformat(&ctx->capture_format);
	uint32_t width = v4l2r_format_width(&ctx->capture_format);
	uint32_t height = v4l2r_format_height(&ctx->capture_format);

	if (!v4l2r_converter_supports(drv, src_pixelformat))
		return VA_STATUS_ERROR_OPERATION_FAILED;

	/* Importing into the converter needs a single memory plane. */
	if (v4l2r_format_num_planes(&ctx->capture_format) != 1)
		return VA_STATUS_ERROR_OPERATION_FAILED;

	conv = calloc(1, sizeof(*conv));
	if (!conv)
		return VA_STATUS_ERROR_ALLOCATION_FAILED;

	conv->fd = open(drv->converter.video_path, O_RDWR | O_NONBLOCK);
	if (conv->fd < 0)
		goto fail;

	/* Source side mirrors the decoder's CAPTURE format exactly; both
	 * drivers derive the layout with v4l2_fill_pixfmt_mp(), so matching
	 * pixelformat and size means matching strides and offsets. */
	conv->output_format = (struct v4l2_format) {
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
	};
	conv->output_format.fmt.pix_mp.pixelformat = src_pixelformat;
	conv->output_format.fmt.pix_mp.width = width;
	conv->output_format.fmt.pix_mp.height = height;
	conv->output_format.fmt.pix_mp.num_planes = 1;
	if (ioctl(conv->fd, VIDIOC_S_FMT, &conv->output_format) < 0)
		goto fail;

	if (conv->output_format.fmt.pix_mp.pixelformat != src_pixelformat ||
	    conv->output_format.fmt.pix_mp.width != width ||
	    conv->output_format.fmt.pix_mp.height != height ||
	    v4l2r_format_bytesperline(&conv->output_format) !=
	    v4l2r_format_bytesperline(&ctx->capture_format)) {
		v4l2r_log("converter source layout mismatch for %.4s %ux%u\n",
			  (const char *)&src_pixelformat, width, height);
		goto fail;
	}

	conv->capture_format = (struct v4l2_format) {
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
	};
	conv->capture_format.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
	conv->capture_format.fmt.pix_mp.width = width;
	conv->capture_format.fmt.pix_mp.height = height;
	conv->capture_format.fmt.pix_mp.num_planes = 1;
	if (ioctl(conv->fd, VIDIOC_S_FMT, &conv->capture_format) < 0)
		goto fail;

	if (conv->capture_format.fmt.pix_mp.pixelformat != V4L2_PIX_FMT_NV12 ||
	    conv->capture_format.fmt.pix_mp.width != width ||
	    conv->capture_format.fmt.pix_mp.height != height ||
	    conv->capture_format.fmt.pix_mp.num_planes != 1) {
		v4l2r_log("converter cannot produce NV12 at %ux%u\n",
			  width, height);
		goto fail;
	}

	reqbufs = (struct v4l2_requestbuffers) {
		.count = V4L2R_CONVERT_SLOTS,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.memory = V4L2_MEMORY_DMABUF,
	};
	if (ioctl(conv->fd, VIDIOC_REQBUFS, &reqbufs) < 0 ||
	    reqbufs.count < V4L2R_CONVERT_SLOTS)
		goto fail;

	reqbufs = (struct v4l2_requestbuffers) {
		.count = V4L2R_CONVERT_SLOTS,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.memory = V4L2_MEMORY_DMABUF,
	};
	if (ioctl(conv->fd, VIDIOC_REQBUFS, &reqbufs) < 0 ||
	    reqbufs.count < V4L2R_CONVERT_SLOTS)
		goto fail;

	type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	if (ioctl(conv->fd, VIDIOC_STREAMON, &type) < 0)
		goto fail;
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	if (ioctl(conv->fd, VIDIOC_STREAMON, &type) < 0)
		goto fail;

	for (unsigned int i = 0; i < V4L2R_CONVERT_SLOTS; i++)
		conv->jobs[i].capture_index = -1;

	ctx->conv = conv;

	v4l2r_log("converting %.4s to NV12 via %s [%s]\n",
		  (const char *)&src_pixelformat, drv->converter.video_path,
		  drv->converter.card);

	return VA_STATUS_SUCCESS;

fail:
	if (conv->fd >= 0)
		close(conv->fd);
	free(conv);
	return VA_STATUS_ERROR_OPERATION_FAILED;
}

void v4l2r_convert_destroy(struct v4l2r_context *ctx)
{
	struct v4l2r_convert *conv = ctx->conv;

	if (!conv)
		return;

	/* Abandon in-flight jobs: their frames are never read again. */
	for (unsigned int i = 0; i < V4L2R_CONVERT_SLOTS; i++) {
		if ((conv->busy & (1u << i)) && conv->jobs[i].surface)
			conv->jobs[i].surface->convert_pending = false;
	}

	/* Closing the fd tears down the m2m context including any queued
	 * jobs; importer-side dma-buf references are dropped with it. */
	close(conv->fd);
	free(conv);
	ctx->conv = NULL;
}

/*
 * Reap finished conversions. Nonblocking unless wait is set, in which case
 * at least one in-flight job is waited for. Returns the number of reaped
 * jobs or a negative error. ctx->mutex must be held.
 */
static int convert_reap(struct v4l2r_context *ctx, bool wait)
{
	struct v4l2r_convert *conv = ctx->conv;
	struct pollfd pollfd = {
		.fd = conv->fd,
		.events = POLLIN,
	};
	int reaped = 0;

	while (conv->busy) {
		struct v4l2_plane planes[VIDEO_MAX_PLANES] = {0};
		struct v4l2_buffer buffer = {
			.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
			.memory = V4L2_MEMORY_DMABUF,
			.length = 1,
			.m.planes = planes,
		};

		if (ioctl(conv->fd, VIDIOC_DQBUF, &buffer) < 0) {
			if (errno != EAGAIN)
				return -errno;

			if (!wait || reaped)
				break;

			if (poll(&pollfd, 1, V4L2R_CONVERT_TIMEOUT_MS) <= 0)
				return -EIO;
			continue;
		}

		if (buffer.index < V4L2R_CONVERT_SLOTS &&
		    (conv->busy & (1u << buffer.index))) {
			unsigned int slot = buffer.index;

			if (conv->jobs[slot].surface)
				conv->jobs[slot].surface->convert_pending = false;
			conv->jobs[slot].surface = NULL;
			conv->jobs[slot].capture_index = -1;
			conv->busy &= ~(1u << slot);
			reaped++;
		}

		/* The paired source buffer completes with the job. */
		memset(planes, 0, sizeof(planes));
		buffer = (struct v4l2_buffer) {
			.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
			.memory = V4L2_MEMORY_DMABUF,
			.length = 1,
			.m.planes = planes,
		};
		if (ioctl(conv->fd, VIDIOC_DQBUF, &buffer) < 0) {
			/* The slot's source stays queued, so reusing the slot
			 * would fail anyway; disable the chain explicitly. */
			v4l2r_log("failed to dequeue converter source: %s\n",
				  strerror(errno));
			conv->failed = true;
		}
	}

	return reaped;
}

void v4l2r_convert_kick(struct v4l2r_context *ctx, int capture_index)
{
	struct v4l2r_convert *conv = ctx->conv;
	struct v4l2r_capture_buffer *capture;
	struct v4l2r_surface *surface;
	struct v4l2_plane planes[VIDEO_MAX_PLANES];
	struct v4l2_buffer buffer;
	unsigned int slot;

	if (!conv || conv->failed)
		return;
	if (capture_index < 0 || capture_index >= (int)ctx->nb_captures)
		return;

	capture = &ctx->captures[capture_index];
	surface = capture->surface;
	if (!surface)
		return;

	convert_reap(ctx, false);

	/* All slots busy: wait one out. */
	while (conv->busy & (1u << conv->next_slot)) {
		if (convert_reap(ctx, true) < 0) {
			v4l2r_log("format converter stalled\n");
			return;
		}
	}
	slot = conv->next_slot;
	conv->next_slot = (slot + 1) % V4L2R_CONVERT_SLOTS;

	if (v4l2r_surface_convert_backing(ctx->drv, surface) !=
	    VA_STATUS_SUCCESS) {
		if (!conv->failed)
			v4l2r_log("no conversion backing for surface, output will be wrong\n");
		conv->failed = true;
		return;
	}

	if (v4l2r_export_capture_dmabufs(ctx, capture, capture_index) < 0)
		return;

	/* For DMABUF the plane length must carry the dma-buf size: the
	 * kernel validates bytesused against it before resolving the fd
	 * (and the decoder's buffer is larger than the image anyway). */
	memset(planes, 0, sizeof(planes));
	planes[0].m.fd = capture->dmabuf_fd[0];
	planes[0].length = capture->plane_size[0];
	planes[0].bytesused =
		conv->output_format.fmt.pix_mp.plane_fmt[0].sizeimage;
	buffer = (struct v4l2_buffer) {
		.index = slot,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.memory = V4L2_MEMORY_DMABUF,
		.length = 1,
		.m.planes = planes,
	};
	if (ioctl(conv->fd, VIDIOC_QBUF, &buffer) < 0) {
		v4l2r_log("failed to queue converter source: %s\n",
			  strerror(errno));
		conv->failed = true;
		return;
	}

	memset(planes, 0, sizeof(planes));
	planes[0].m.fd = surface->backing->dmabuf_fd[0];
	planes[0].length = surface->backing->plane_size[0];
	buffer = (struct v4l2_buffer) {
		.index = slot,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.memory = V4L2_MEMORY_DMABUF,
		.length = 1,
		.m.planes = planes,
	};
	if (ioctl(conv->fd, VIDIOC_QBUF, &buffer) < 0) {
		v4l2r_log("failed to queue converter destination: %s\n",
			  strerror(errno));
		conv->failed = true;
		return;
	}

	conv->jobs[slot].capture_index = capture_index;
	conv->jobs[slot].surface = surface;
	conv->busy |= 1u << slot;
	surface->convert_pending = true;
}

void v4l2r_convert_drain_index(struct v4l2r_context *ctx, int capture_index)
{
	struct v4l2r_convert *conv = ctx->conv;
	bool in_flight = true;

	if (!conv)
		return;

	pthread_mutex_lock(&ctx->mutex);
	while (in_flight) {
		in_flight = false;
		for (unsigned int i = 0; i < V4L2R_CONVERT_SLOTS; i++) {
			if ((conv->busy & (1u << i)) &&
			    conv->jobs[i].capture_index == capture_index) {
				in_flight = true;
				break;
			}
		}

		if (in_flight && convert_reap(ctx, true) < 0)
			break;
	}
	pthread_mutex_unlock(&ctx->mutex);
}

VAStatus v4l2r_convert_wait(struct v4l2r_surface *surface)
{
	struct v4l2r_context *ctx = surface->ctx;
	VAStatus status = VA_STATUS_SUCCESS;

	if (!ctx || !ctx->conv)
		return VA_STATUS_SUCCESS;

	pthread_mutex_lock(&ctx->mutex);
	while (surface->convert_pending) {
		if (convert_reap(ctx, true) < 0) {
			v4l2r_log("failed waiting for format conversion\n");
			surface->convert_pending = false;
			status = VA_STATUS_ERROR_OPERATION_FAILED;
			break;
		}

		/* Nothing in flight can clear the flag anymore. */
		if (!ctx->conv->busy && surface->convert_pending) {
			surface->convert_pending = false;
			break;
		}
	}
	pthread_mutex_unlock(&ctx->mutex);

	return status;
}

/*
 * --- video processing (VAEntrypointVideoProc) ---
 *
 * Surface-to-surface blits on the format converter, driven through the
 * standard VA video processing pipeline: the source surface, its region,
 * the output region on the target surface, the rotation angle and the
 * mirroring directions all come from a VAProcPipelineParameterBuffer.
 * Rotation and mirroring map onto the converter's V4L2_CID_ROTATE/HFLIP/
 * VFLIP controls, the regions onto its crop/compose rectangles, and the
 * blit itself is one synchronous mem2mem job per vaEndPicture(), imported
 * dma-buf to imported dma-buf.
 */

VAStatus v4l2r_QueryVideoProcFilters(VADriverContextP va_ctx,
				     VAContextID context_id,
				     VAProcFilterType *filters,
				     unsigned int *num_filters)
{
	struct v4l2r_driver *drv = v4l2r_driver(va_ctx);

	(void)filters;

	if (!V4L2R_CONTEXT_GET(drv, context_id))
		return VA_STATUS_ERROR_INVALID_CONTEXT;

	/* Rotation and mirroring are pipeline parameters, not filters; no
	 * parametrized filters are offered. */
	*num_filters = 0;
	return VA_STATUS_SUCCESS;
}

VAStatus v4l2r_QueryVideoProcFilterCaps(VADriverContextP va_ctx,
					VAContextID context_id,
					VAProcFilterType type,
					void *filter_caps,
					unsigned int *num_filter_caps)
{
	struct v4l2r_driver *drv = v4l2r_driver(va_ctx);

	(void)type;
	(void)filter_caps;

	if (!V4L2R_CONTEXT_GET(drv, context_id))
		return VA_STATUS_ERROR_INVALID_CONTEXT;

	*num_filter_caps = 0;
	return VA_STATUS_ERROR_UNSUPPORTED_FILTER;
}

VAStatus v4l2r_QueryVideoProcPipelineCaps(VADriverContextP va_ctx,
					  VAContextID context_id,
					  VABufferID *filters,
					  unsigned int num_filters,
					  VAProcPipelineCaps *pipeline_caps)
{
	/* Arrays the caps point into must outlive the call. */
	static VAProcColorStandardType color_standards[] = {
		VAProcColorStandardBT601,
		VAProcColorStandardBT709,
	};
	static uint32_t pixel_formats[] = {
		VA_FOURCC_NV12,
	};
	struct v4l2r_driver *drv = v4l2r_driver(va_ctx);

	(void)filters;

	if (!V4L2R_CONTEXT_GET(drv, context_id))
		return VA_STATUS_ERROR_INVALID_CONTEXT;

	if (num_filters)
		return VA_STATUS_ERROR_UNSUPPORTED_FILTER;

	memset(pipeline_caps, 0, sizeof(*pipeline_caps));

	pipeline_caps->rotation_flags = (1 << VA_ROTATION_NONE) |
					(1 << VA_ROTATION_90) |
					(1 << VA_ROTATION_180) |
					(1 << VA_ROTATION_270);
	pipeline_caps->mirror_flags = VA_MIRROR_HORIZONTAL |
				      VA_MIRROR_VERTICAL;

	pipeline_caps->input_color_standards = color_standards;
	pipeline_caps->num_input_color_standards =
		sizeof(color_standards) / sizeof(color_standards[0]);
	pipeline_caps->output_color_standards = color_standards;
	pipeline_caps->num_output_color_standards =
		pipeline_caps->num_input_color_standards;

	pipeline_caps->input_pixel_format = pixel_formats;
	pipeline_caps->num_input_pixel_formats = 1;
	pipeline_caps->output_pixel_format = pixel_formats;
	pipeline_caps->num_output_pixel_formats = 1;

	pipeline_caps->min_input_width = V4L2R_VPP_MIN_DIM;
	pipeline_caps->min_input_height = V4L2R_VPP_MIN_DIM;
	pipeline_caps->max_input_width = V4L2R_VPP_MAX_DIM;
	pipeline_caps->max_input_height = V4L2R_VPP_MAX_DIM;
	pipeline_caps->min_output_width = V4L2R_VPP_MIN_DIM;
	pipeline_caps->min_output_height = V4L2R_VPP_MIN_DIM;
	pipeline_caps->max_output_width = V4L2R_VPP_MAX_DIM;
	pipeline_caps->max_output_height = V4L2R_VPP_MAX_DIM;

	return VA_STATUS_SUCCESS;
}

VAStatus v4l2r_vpp_create(struct v4l2r_context *ctx)
{
	if (!v4l2r_converter_available(ctx->drv))
		return VA_STATUS_ERROR_UNSUPPORTED_ENTRYPOINT;

	ctx->vpp = calloc(1, sizeof(*ctx->vpp));
	if (!ctx->vpp)
		return VA_STATUS_ERROR_ALLOCATION_FAILED;
	ctx->vpp->fd = -1;

	return VA_STATUS_SUCCESS;
}

static void vpp_teardown(struct v4l2r_vpp *vpp)
{
	if (vpp->fd >= 0) {
		close(vpp->fd);
		vpp->fd = -1;
	}
}

void v4l2r_vpp_destroy(struct v4l2r_context *ctx)
{
	if (!ctx->vpp)
		return;

	vpp_teardown(ctx->vpp);
	free(ctx->vpp);
	ctx->vpp = NULL;
}

VAStatus v4l2r_vpp_begin_picture(struct v4l2r_context *ctx,
				 struct v4l2r_surface *target)
{
	ctx->pic.target = target;
	ctx->in_picture = true;
	ctx->vpp->have_params = false;

	return VA_STATUS_SUCCESS;
}

VAStatus v4l2r_vpp_render_buffer(struct v4l2r_context *ctx,
				 struct v4l2r_buffer *buf)
{
	struct v4l2r_vpp *vpp = ctx->vpp;
	struct v4l2r_surface *target = ctx->pic.target;
	const VAProcPipelineParameterBuffer *params;
	struct v4l2r_surface *surface;

	if (buf->type != VAProcPipelineParameterBufferType)
		return VA_STATUS_ERROR_UNSUPPORTED_BUFFERTYPE;
	if (v4l2r_buffer_bytes(buf) < sizeof(*params))
		return VA_STATUS_ERROR_INVALID_BUFFER;
	params = buf->data;

	if (params->num_filters)
		return VA_STATUS_ERROR_UNSUPPORTED_FILTER;
	if (params->rotation_state > VA_ROTATION_270 ||
	    (params->mirror_state &
	     ~(uint32_t)(VA_MIRROR_HORIZONTAL | VA_MIRROR_VERTICAL)))
		return VA_STATUS_ERROR_INVALID_PARAMETER;

	surface = V4L2R_SURFACE_GET(ctx->drv, params->surface);
	if (!surface)
		return VA_STATUS_ERROR_INVALID_SURFACE;

	/* The region pointers are only valid during this call; keep copies
	 * (defaulted to the full surfaces) for vaEndPicture(). */
	vpp->src_surface = surface;
	if (params->surface_region)
		vpp->src_region = *params->surface_region;
	else
		vpp->src_region = (VARectangle) {
			0, 0, surface->width, surface->height,
		};
	if (params->output_region)
		vpp->dst_region = *params->output_region;
	else
		vpp->dst_region = (VARectangle) {
			0, 0, target->width, target->height,
		};
	vpp->rotation = params->rotation_state;
	vpp->mirror = params->mirror_state;
	vpp->have_params = true;

	return VA_STATUS_SUCCESS;
}

static bool vpp_region_valid(const VARectangle *region,
			     uint32_t width, uint32_t height)
{
	return region->x >= 0 && region->y >= 0 &&
	       region->width >= V4L2R_VPP_MIN_DIM &&
	       region->height >= V4L2R_VPP_MIN_DIM &&
	       (uint32_t)region->x + region->width <= width &&
	       (uint32_t)region->y + region->height <= height;
}

/*
 * Set one side of the converter to the buffer's layout. The converter
 * derives its line stride from the format width alone, so when the
 * buffer's stride differs (its allocator aligned the line up), the format
 * is re-set with the width that yields the buffer's stride - the active
 * area is confined by the crop/compose rectangle, not the format.
 */
static int vpp_set_format(int fd, enum v4l2_buf_type type,
			  uint32_t pixelformat, uint32_t width,
			  uint32_t height, uint32_t pitch,
			  struct v4l2_format *format)
{
	for (unsigned int attempt = 0; attempt < 2; attempt++) {
		*format = (struct v4l2_format) { .type = type };
		format->fmt.pix_mp.pixelformat = pixelformat;
		format->fmt.pix_mp.width = width;
		format->fmt.pix_mp.height = height;
		format->fmt.pix_mp.num_planes = 1;
		if (ioctl(fd, VIDIOC_S_FMT, format) < 0)
			return -errno;

		if (format->fmt.pix_mp.pixelformat != pixelformat ||
		    format->fmt.pix_mp.height != height ||
		    format->fmt.pix_mp.num_planes != 1)
			return -EINVAL;

		if (v4l2r_format_bytesperline(format) == pitch)
			return 0;

		if (attempt || !v4l2r_format_bytesperline(format))
			break;

		/* Stride scales linearly with the width for the packed
		 * single-plane YUV formats used here. */
		width = (uint32_t)((uint64_t)width * pitch /
				   v4l2r_format_bytesperline(format));
	}

	return -EINVAL;
}

static int vpp_set_control(int fd, uint32_t id, int32_t value)
{
	struct v4l2_control control = {
		.id = id,
		.value = value,
	};

	return ioctl(fd, VIDIOC_S_CTRL, &control) < 0 ? -errno : 0;
}

static int vpp_set_rect(int fd, enum v4l2_buf_type type, uint32_t target,
			const VARectangle *region)
{
	struct v4l2_selection selection = {
		.type = type,
		.target = target,
		.r = {
			.left = region->x,
			.top = region->y,
			.width = region->width,
			.height = region->height,
		},
	};

	return ioctl(fd, VIDIOC_S_SELECTION, &selection) < 0 ? -errno : 0;
}

/*
 * (Re)configure the converter for the source and destination buffer
 * layouts. Formats cannot change while buffers are allocated, so a layout
 * change tears the instance down and starts over; in practice a video
 * processing context sees one layout for its whole life.
 */
static VAStatus vpp_configure(struct v4l2r_context *ctx,
			      const struct v4l2r_frame_view *src,
			      const struct v4l2r_frame_view *dst)
{
	struct v4l2r_vpp *vpp = ctx->vpp;
	struct v4l2r_driver *drv = ctx->drv;
	struct v4l2_requestbuffers reqbufs;
	enum v4l2_buf_type type;

	if (vpp->fd >= 0 &&
	    vpp->src_pixelformat == src->pixelformat &&
	    vpp->src_width == src->width && vpp->src_height == src->height &&
	    vpp->src_pitch == src->pitch &&
	    vpp->dst_width == dst->width && vpp->dst_height == dst->height &&
	    vpp->dst_pitch == dst->pitch)
		return VA_STATUS_SUCCESS;

	vpp_teardown(vpp);

	vpp->fd = open(drv->converter.video_path, O_RDWR | O_NONBLOCK);
	if (vpp->fd < 0)
		return VA_STATUS_ERROR_OPERATION_FAILED;

	if (vpp_set_format(vpp->fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
			   src->pixelformat, src->width, src->height,
			   src->pitch, &vpp->output_format) < 0 ||
	    vpp_set_format(vpp->fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
			   V4L2_PIX_FMT_NV12, dst->width, dst->height,
			   dst->pitch, &vpp->capture_format) < 0) {
		v4l2r_log("converter cannot process %.4s %ux%u to NV12 %ux%u\n",
			  (const char *)&src->pixelformat, src->width,
			  src->height, dst->width, dst->height);
		goto fail;
	}

	/* The whole destination image must fit the imported dma-buf. */
	if (vpp->capture_format.fmt.pix_mp.plane_fmt[0].sizeimage >
	    dst->plane_size[0])
		goto fail;

	reqbufs = (struct v4l2_requestbuffers) {
		.count = 1,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.memory = V4L2_MEMORY_DMABUF,
	};
	if (ioctl(vpp->fd, VIDIOC_REQBUFS, &reqbufs) < 0 || !reqbufs.count)
		goto fail;

	reqbufs = (struct v4l2_requestbuffers) {
		.count = 1,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.memory = V4L2_MEMORY_DMABUF,
	};
	if (ioctl(vpp->fd, VIDIOC_REQBUFS, &reqbufs) < 0 || !reqbufs.count)
		goto fail;

	type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	if (ioctl(vpp->fd, VIDIOC_STREAMON, &type) < 0)
		goto fail;
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	if (ioctl(vpp->fd, VIDIOC_STREAMON, &type) < 0)
		goto fail;

	vpp->src_pixelformat = src->pixelformat;
	vpp->src_width = src->width;
	vpp->src_height = src->height;
	vpp->src_pitch = src->pitch;
	vpp->dst_width = dst->width;
	vpp->dst_height = dst->height;
	vpp->dst_pitch = dst->pitch;

	v4l2r_log("processing %.4s %ux%u to NV12 %ux%u via %s [%s]\n",
		  (const char *)&src->pixelformat, src->width, src->height,
		  dst->width, dst->height, drv->converter.video_path,
		  drv->converter.card);

	return VA_STATUS_SUCCESS;

fail:
	vpp_teardown(vpp);
	return VA_STATUS_ERROR_OPERATION_FAILED;
}

/* Queue the source and destination dma-bufs and wait the job out. */
static VAStatus vpp_run(struct v4l2r_vpp *vpp,
			const struct v4l2r_frame_view *src,
			const struct v4l2r_frame_view *dst)
{
	struct v4l2_plane planes[VIDEO_MAX_PLANES];
	struct v4l2_buffer buffer;
	struct pollfd pollfd = {
		.fd = vpp->fd,
		.events = POLLIN,
	};

	/* For DMABUF the plane length must carry the dma-buf size: the
	 * kernel validates bytesused against it before resolving the fd
	 * (and decoder buffers are larger than the image anyway). */
	memset(planes, 0, sizeof(planes));
	planes[0].m.fd = src->dmabuf_fd[0];
	planes[0].length = src->plane_size[0];
	planes[0].bytesused =
		vpp->output_format.fmt.pix_mp.plane_fmt[0].sizeimage;
	buffer = (struct v4l2_buffer) {
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.memory = V4L2_MEMORY_DMABUF,
		.length = 1,
		.m.planes = planes,
	};
	if (ioctl(vpp->fd, VIDIOC_QBUF, &buffer) < 0)
		return VA_STATUS_ERROR_OPERATION_FAILED;

	memset(planes, 0, sizeof(planes));
	planes[0].m.fd = dst->dmabuf_fd[0];
	planes[0].length = dst->plane_size[0];
	buffer = (struct v4l2_buffer) {
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.memory = V4L2_MEMORY_DMABUF,
		.length = 1,
		.m.planes = planes,
	};
	if (ioctl(vpp->fd, VIDIOC_QBUF, &buffer) < 0)
		return VA_STATUS_ERROR_OPERATION_FAILED;

	memset(planes, 0, sizeof(planes));
	buffer = (struct v4l2_buffer) {
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.memory = V4L2_MEMORY_DMABUF,
		.length = 1,
		.m.planes = planes,
	};
	while (ioctl(vpp->fd, VIDIOC_DQBUF, &buffer) < 0) {
		if (errno != EAGAIN ||
		    poll(&pollfd, 1, V4L2R_CONVERT_TIMEOUT_MS) <= 0)
			return VA_STATUS_ERROR_OPERATION_FAILED;
	}
	if (buffer.flags & V4L2_BUF_FLAG_ERROR)
		return VA_STATUS_ERROR_OPERATION_FAILED;

	memset(planes, 0, sizeof(planes));
	buffer = (struct v4l2_buffer) {
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.memory = V4L2_MEMORY_DMABUF,
		.length = 1,
		.m.planes = planes,
	};
	if (ioctl(vpp->fd, VIDIOC_DQBUF, &buffer) < 0)
		return VA_STATUS_ERROR_OPERATION_FAILED;

	return VA_STATUS_SUCCESS;
}

VAStatus v4l2r_vpp_end_picture(struct v4l2r_context *ctx)
{
	struct v4l2r_vpp *vpp = ctx->vpp;
	struct v4l2r_surface *src = vpp->src_surface;
	struct v4l2r_surface *dst = ctx->pic.target;
	struct v4l2r_frame_view src_view, dst_view;
	VAStatus status;

	if (!vpp->have_params || !dst || !src)
		return VA_STATUS_ERROR_INVALID_PARAMETER;

	/* The exported dma-buf carries no fence: wait for the source pixels
	 * (a held-back decode, the decode itself, a pending conversion). */
	status = v4l2r_surface_ready(src);
	if (status != VA_STATUS_SUCCESS)
		return status;

	/* Behind a conversion chain, read the raw decoder output (packed
	 * 10-bit NV15) instead of the converted backing: the rotation and
	 * the downconversion then happen in the same pass. Only without a
	 * horizontal crop offset - the converter addresses crop columns in
	 * whole bytes, which cannot land mid-byte in packed 10-bit layouts. */
	if (src->ctx && src->ctx->conv && src->capture_index >= 0 &&
	    vpp->src_region.x == 0)
		status = v4l2r_surface_capture_view(src, false, &src_view);
	else
		status = v4l2r_surface_view(ctx->drv, src, false, &src_view);
	if (status != VA_STATUS_SUCCESS)
		return status;

	if (src_view.nb_planes != 1 ||
	    !v4l2r_converter_supports(ctx->drv, src_view.pixelformat))
		return VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT;

	status = v4l2r_surface_view(ctx->drv, dst, false, &dst_view);
	if (status != VA_STATUS_SUCCESS)
		return status;

	if (dst_view.nb_planes != 1 ||
	    dst_view.pixelformat != V4L2_PIX_FMT_NV12)
		return VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT;

	/* The source region is in pre-rotation coordinates (the converter
	 * crops the source), the output region in destination coordinates
	 * (it composes into the destination) - both as VA-API defines them. */
	if (!vpp_region_valid(&vpp->src_region, src_view.width,
			      src_view.height) ||
	    !vpp_region_valid(&vpp->dst_region, dst_view.width,
			      dst_view.height))
		return VA_STATUS_ERROR_INVALID_PARAMETER;

	pthread_mutex_lock(&ctx->mutex);

	status = vpp_configure(ctx, &src_view, &dst_view);
	if (status != VA_STATUS_SUCCESS)
		goto done;

	/* VA rotation is clockwise, as is the converter's; VA mirroring is
	 * applied after the rotation, in output coordinates, matching the
	 * hardware's combined rotate+mirror addressing. */
	if (vpp_set_control(vpp->fd, V4L2_CID_ROTATE,
			    (int32_t)vpp->rotation * 90) < 0 ||
	    vpp_set_control(vpp->fd, V4L2_CID_HFLIP,
			    !!(vpp->mirror & VA_MIRROR_HORIZONTAL)) < 0 ||
	    vpp_set_control(vpp->fd, V4L2_CID_VFLIP,
			    !!(vpp->mirror & VA_MIRROR_VERTICAL)) < 0 ||
	    vpp_set_rect(vpp->fd, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
			 V4L2_SEL_TGT_CROP, &vpp->src_region) < 0 ||
	    vpp_set_rect(vpp->fd, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
			 V4L2_SEL_TGT_COMPOSE, &vpp->dst_region) < 0) {
		status = VA_STATUS_ERROR_OPERATION_FAILED;
		goto done;
	}

	status = vpp_run(vpp, &src_view, &dst_view);

done:
	if (status != VA_STATUS_SUCCESS) {
		v4l2r_log("video processing blit failed\n");
		/* Start from a clean converter instance next time. */
		vpp_teardown(vpp);
	}
	pthread_mutex_unlock(&ctx->mutex);

	return status;
}
