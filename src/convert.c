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

		if (format->fmt.pix_mp.plane_fmt[0].bytesperline == pitch)
			return 0;

		if (attempt || !format->fmt.pix_mp.plane_fmt[0].bytesperline)
			break;

		/* Stride scales linearly with the width for the packed
		 * single-plane YUV formats used here. */
		width = (uint32_t)((uint64_t)width * pitch /
				   format->fmt.pix_mp.plane_fmt[0].bytesperline);
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
