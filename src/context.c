/*
 * VA context lifecycle: decoder device selection, queue setup and
 * surface-to-CAPTURE-buffer binding.
 *
 * The CAPTURE side is configured lazily on the first decode, once the
 * codec controls (e.g. the SPS) are known, so the kernel driver can pick
 * a suitable frame format - mirroring the FFmpeg hwaccel probe order.
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
#include <sys/mman.h>
#include <unistd.h>

#include <linux/media.h>
#include <linux/videodev2.h>

#include "v4l2_request.h"

static int set_format(struct v4l2r_context *ctx, enum v4l2_buf_type type,
		      uint32_t pixelformat, uint32_t buffersize)
{
	struct v4l2_format format = {
		.type = type,
	};

	if (V4L2_TYPE_IS_MULTIPLANAR(type)) {
		format.fmt.pix_mp.width = ctx->picture_width;
		format.fmt.pix_mp.height = ctx->picture_height;
		format.fmt.pix_mp.pixelformat = pixelformat;
		format.fmt.pix_mp.plane_fmt[0].sizeimage = buffersize;
		format.fmt.pix_mp.num_planes = 1;
	} else {
		format.fmt.pix.width = ctx->picture_width;
		format.fmt.pix.height = ctx->picture_height;
		format.fmt.pix.pixelformat = pixelformat;
		format.fmt.pix.sizeimage = buffersize;
	}

	if (ioctl(ctx->video_fd, VIDIOC_S_FMT, &format) < 0)
		return -errno;

	return 0;
}

static int query_buffer_capabilities(struct v4l2r_context *ctx,
				     enum v4l2_buf_type type,
				     uint32_t *capabilities)
{
	struct v4l2_create_buffers buffers = {
		.count = 0,
		.memory = V4L2_MEMORY_MMAP,
		.format.type = type,
	};

	if (ioctl(ctx->video_fd, VIDIOC_CREATE_BUFS, &buffers) < 0)
		return -errno;

	*capabilities = buffers.capabilities;
	return 0;
}

static bool try_output_format(struct v4l2r_context *ctx, uint32_t pixelformat)
{
	struct v4l2_fmtdesc fmtdesc = {
		.type = ctx->output_format.type,
	};

	while (ioctl(ctx->video_fd, VIDIOC_ENUM_FMT, &fmtdesc) >= 0) {
		if (fmtdesc.pixelformat == pixelformat)
			return true;
		fmtdesc.index++;
	}

	return false;
}

static bool try_framesize(struct v4l2r_context *ctx, uint32_t pixelformat)
{
	struct v4l2_frmsizeenum frmsize = {
		.pixel_format = pixelformat,
	};

	if (ioctl(ctx->video_fd, VIDIOC_ENUM_FRAMESIZES, &frmsize) < 0)
		return errno == ENOTTY;

	do {
		if (frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE &&
		    ctx->picture_width == frmsize.discrete.width &&
		    ctx->picture_height == frmsize.discrete.height)
			return true;

		if ((frmsize.type == V4L2_FRMSIZE_TYPE_STEPWISE ||
		     frmsize.type == V4L2_FRMSIZE_TYPE_CONTINUOUS) &&
		    ctx->picture_width >= frmsize.stepwise.min_width &&
		    ctx->picture_height >= frmsize.stepwise.min_height &&
		    ctx->picture_width <= frmsize.stepwise.max_width &&
		    ctx->picture_height <= frmsize.stepwise.max_height)
			return true;

		frmsize.index++;
	} while (ioctl(ctx->video_fd, VIDIOC_ENUM_FRAMESIZES, &frmsize) >= 0);

	return false;
}

/*
 * Pick a CAPTURE format. Preference order:
 *  1. linear formats representable as VA images with matching bit depth,
 *     so display interop (dma-buf import) and readback work best,
 *  2. the driver preferred (default) format with matching bit depth,
 *  3. any known format with matching bit depth,
 *  4. any known format.
 */
static int select_capture_format(struct v4l2r_context *ctx)
{
	enum v4l2_buf_type type = ctx->capture_format.type;
	const struct v4l2r_format_info *info;
	struct v4l2_format format = {
		.type = type,
	};
	struct v4l2_fmtdesc fmtdesc = {
		.type = type,
	};
	uint32_t best = 0;
	int best_score = -1;

	while (ioctl(ctx->video_fd, VIDIOC_ENUM_FMT, &fmtdesc) >= 0) {
		info = v4l2r_format_by_pixelformat(fmtdesc.pixelformat);
		fmtdesc.index++;

		if (!info)
			continue;

		bool depth_ok = !ctx->bit_depth ||
				info->bit_depth == ctx->bit_depth;
		int score = 0;

		if (depth_ok)
			score += 2;
		if (info->linear && info->va_fourcc && depth_ok)
			score += 4;

		if (score > best_score) {
			best_score = score;
			best = info->pixelformat;
		}
	}

	/* Prefer the driver default over other non-linear candidates. */
	if (best_score < 6 &&
	    ioctl(ctx->video_fd, VIDIOC_G_FMT, &format) >= 0) {
		info = v4l2r_format_by_pixelformat(v4l2r_format_pixelformat(&format));
		if (info && (!ctx->bit_depth || info->bit_depth == ctx->bit_depth))
			best = info->pixelformat;
	}

	if (!best)
		return -EINVAL;

	return set_format(ctx, type, best, 0);
}

/* --- OUTPUT bitstream buffers with their media requests --- */

static void output_buffer_cleanup(struct v4l2r_context *ctx,
				  struct v4l2r_output_buffer *output)
{
	(void)ctx;

	if (output->request_fd >= 0) {
		close(output->request_fd);
		output->request_fd = -1;
	}

	if (output->addr) {
		munmap(output->addr, output->size);
		output->addr = NULL;
	}
}

/* Create and map one OUTPUT buffer; sizeimage overrides the size from the
 * queue format when nonzero (used to grow buffers mid-stream). */
static int output_buffer_setup(struct v4l2r_context *ctx,
			       struct v4l2r_output_buffer *output,
			       uint32_t sizeimage)
{
	struct v4l2_create_buffers buffers = {
		.count = 1,
		.memory = V4L2_MEMORY_MMAP,
		.format = ctx->output_format,
	};
	struct v4l2_plane planes[VIDEO_MAX_PLANES] = {0};
	struct v4l2_buffer buffer = {0};
	off_t offset;
	void *addr;

	if (sizeimage) {
		if (V4L2_TYPE_IS_MULTIPLANAR(buffers.format.type))
			buffers.format.fmt.pix_mp.plane_fmt[0].sizeimage =
				sizeimage;
		else
			buffers.format.fmt.pix.sizeimage = sizeimage;
	}

	if (ioctl(ctx->video_fd, VIDIOC_CREATE_BUFS, &buffers) < 0) {
		v4l2r_log("failed to create OUTPUT buffer: %s\n",
			  strerror(errno));
		return -errno;
	}

	/* The queued_output/queued_request bitmasks track buffers by index. */
	if (buffers.index >= 32)
		return -ENOSPC;

	buffer.type = ctx->output_format.type;
	buffer.index = buffers.index;
	if (V4L2_TYPE_IS_MULTIPLANAR(buffer.type)) {
		buffer.length = VIDEO_MAX_PLANES;
		buffer.m.planes = planes;
	}

	if (ioctl(ctx->video_fd, VIDIOC_QUERYBUF, &buffer) < 0) {
		v4l2r_log("failed to query OUTPUT buffer %u: %s\n",
			  buffers.index, strerror(errno));
		return -errno;
	}

	output->index = buffer.index;
	if (V4L2_TYPE_IS_MULTIPLANAR(buffer.type)) {
		output->size = buffer.m.planes[0].length;
		offset = buffer.m.planes[0].m.mem_offset;
	} else {
		output->size = buffer.length;
		offset = buffer.m.offset;
	}

	addr = mmap(NULL, output->size, PROT_READ | PROT_WRITE, MAP_SHARED,
		    ctx->video_fd, offset);
	if (addr == MAP_FAILED) {
		v4l2r_log("failed to map OUTPUT buffer %u: %s\n",
			  output->index, strerror(errno));
		return -errno;
	}
	output->addr = addr;
	output->bytesused = 0;

	if (ioctl(ctx->media_fd, MEDIA_IOC_REQUEST_ALLOC, &output->request_fd) < 0) {
		v4l2r_log("failed to allocate request: %s\n", strerror(errno));
		output->request_fd = -1;
		return -errno;
	}

	v4l2r_trace("allocated OUTPUT buffer #%u (%u bytes)\n",
		  output->index, output->size);

	return 0;
}

/*
 * Replace an OUTPUT buffer whose bitstream ran out of room with a larger
 * one, preserving the data staged so far. V4L2 cannot free individual
 * buffers, so the replaced one stays allocated (but idle) until the
 * context is destroyed; sizes double, so this happens at most a few times
 * per context and lets the initial allocation stay far below the raw
 * frame size.
 */
int v4l2r_output_buffer_grow(struct v4l2r_context *ctx,
			     struct v4l2r_output_buffer *output,
			     size_t min_size)
{
	struct v4l2r_output_buffer grown = { .request_fd = -1 };
	uint32_t size = output->size;
	int ret;

	while ((size_t)size < min_size) {
		if (size > UINT32_MAX / 2)
			return -ENOSPC;
		size *= 2;
	}

	ret = output_buffer_setup(ctx, &grown, size);
	if (ret < 0 || grown.size < min_size) {
		output_buffer_cleanup(ctx, &grown);
		return ret < 0 ? ret : -ENOSPC;
	}

	memcpy(grown.addr, output->addr, output->bytesused);
	grown.bytesused = output->bytesused;

	/* The replaced buffer is idle - next_output() waited for its dequeue
	 * before the picture started - but its request may still be marked in
	 * flight; forget it along with the buffer. */
	pthread_mutex_lock(&ctx->mutex);
	ctx->queued_request &= ~(1u << output->index);
	output_buffer_cleanup(ctx, output);
	*output = grown;
	pthread_mutex_unlock(&ctx->mutex);

	v4l2r_log("grew OUTPUT buffer to %u bytes\n", grown.size);

	return 0;
}

/* --- CAPTURE buffer pool --- */

static void capture_buffer_cleanup(struct v4l2r_context *ctx,
				   struct v4l2r_capture_buffer *capture)
{
	(void)ctx;

	for (unsigned int i = 0; i < VIDEO_MAX_PLANES; i++) {
		if (capture->map[i]) {
			munmap(capture->map[i], capture->plane_size[i]);
			capture->map[i] = NULL;
		}
		if (capture->dmabuf_fd[i] >= 0) {
			close(capture->dmabuf_fd[i]);
			capture->dmabuf_fd[i] = -1;
		}
	}

	if (capture->surface) {
		capture->surface->ctx = NULL;
		capture->surface->capture_index = -1;
		capture->surface = NULL;
	}
}

static void free_list_push(struct v4l2r_context *ctx, int index)
{
	unsigned int tail = (ctx->free_head + ctx->free_count) %
			    V4L2R_MAX_CAPTURE_BUFFERS;

	/* Balanced by owned<->free transitions, so free_count never exceeds
	 * nb_captures and the ring cannot overflow. */
	ctx->free_captures[tail] = (uint8_t)index;
	ctx->free_count++;
}

static int free_list_pop(struct v4l2r_context *ctx)
{
	int index;

	if (!ctx->free_count)
		return -1;

	index = ctx->free_captures[ctx->free_head];
	ctx->free_head = (ctx->free_head + 1) % V4L2R_MAX_CAPTURE_BUFFERS;
	ctx->free_count--;

	return index;
}

/* Per-CAPTURE-buffer size implied by the negotiated format, in bytes. */
static size_t capture_format_bytes(const struct v4l2_format *fmt)
{
	size_t bytes = 0;

	if (V4L2_TYPE_IS_MULTIPLANAR(fmt->type)) {
		for (unsigned int i = 0; i < fmt->fmt.pix_mp.num_planes; i++)
			bytes += fmt->fmt.pix_mp.plane_fmt[i].sizeimage;
	} else {
		bytes = fmt->fmt.pix.sizeimage;
	}

	return bytes;
}

/* Allocate one new CAPTURE buffer, returning its index (unbound). */
static int capture_buffer_new(struct v4l2r_context *ctx)
{
	struct v4l2_create_buffers buffers = {
		.count = 1,
		.memory = V4L2_MEMORY_MMAP,
		.format = ctx->capture_format,
	};
	struct v4l2_plane planes[VIDEO_MAX_PLANES] = {0};
	struct v4l2_buffer buffer = {0};
	struct v4l2r_capture_buffer *capture;
	size_t bufsize = capture_format_bytes(&ctx->capture_format);

	if (ctx->nb_captures >= V4L2R_MAX_CAPTURE_BUFFERS) {
		v4l2r_log("CAPTURE buffer limit reached (%u buffers, ~%llu MiB); "
			  "refusing to allocate more\n", ctx->nb_captures,
			  (unsigned long long)((uint64_t)ctx->nb_captures *
					       bufsize >> 20));
		return -ENOSPC;
	}

	if (ioctl(ctx->video_fd, VIDIOC_CREATE_BUFS, &buffers) < 0) {
		v4l2r_log("failed to allocate CAPTURE buffer #%u (%zu bytes; "
			  "already %u buffers ~%llu MiB allocated): %s\n",
			  ctx->nb_captures, bufsize, ctx->nb_captures,
			  (unsigned long long)((uint64_t)ctx->nb_captures *
					       bufsize >> 20),
			  strerror(errno));
		return -errno;
	}

	if (buffers.index >= V4L2R_MAX_CAPTURE_BUFFERS)
		return -ENOSPC;

	buffer.type = ctx->capture_format.type;
	buffer.index = buffers.index;
	if (V4L2_TYPE_IS_MULTIPLANAR(buffer.type)) {
		buffer.length = VIDEO_MAX_PLANES;
		buffer.m.planes = planes;
	}

	if (ioctl(ctx->video_fd, VIDIOC_QUERYBUF, &buffer) < 0) {
		v4l2r_log("failed to query CAPTURE buffer %u: %s\n",
			  buffers.index, strerror(errno));
		return -errno;
	}

	capture = &ctx->captures[buffer.index];
	memset(capture, 0, sizeof(*capture));

	if (V4L2_TYPE_IS_MULTIPLANAR(buffer.type)) {
		capture->nb_planes = ctx->capture_format.fmt.pix_mp.num_planes;
		for (unsigned int i = 0; i < capture->nb_planes; i++) {
			capture->plane_mem_offset[i] = buffer.m.planes[i].m.mem_offset;
			capture->plane_size[i] = buffer.m.planes[i].length;
		}
	} else {
		capture->nb_planes = 1;
		capture->plane_mem_offset[0] = buffer.m.offset;
		capture->plane_size[0] = buffer.length;
	}

	for (unsigned int i = 0; i < VIDEO_MAX_PLANES; i++)
		capture->dmabuf_fd[i] = -1;

	if (buffer.index >= ctx->nb_captures)
		ctx->nb_captures = buffer.index + 1;

	{
		size_t got = 0;
		for (unsigned int i = 0; i < capture->nb_planes; i++)
			got += capture->plane_size[i];
		v4l2r_trace("allocated CAPTURE buffer #%u (%zu bytes); "
			  "%u buffers, ~%llu MiB total\n", buffer.index, got,
			  ctx->nb_captures,
			  (unsigned long long)((uint64_t)ctx->nb_captures *
					       got >> 20));
	}

	return buffer.index;
}

/*
 * Wait for external consumers of the buffer's exported dma-buf(s) to finish
 * reading. POLLOUT on a dma-buf completes only once all fences on its
 * reservation object have signalled, which for an imported buffer includes the
 * read fence a GPU attaches while sampling it. Buffers that were never exported
 * carry fd == -1 and are skipped.
 */
static void capture_wait_readers(struct v4l2r_capture_buffer *capture)
{
	for (unsigned int i = 0; i < capture->nb_planes; i++) {
		struct pollfd pfd = {
			.fd = capture->dmabuf_fd[i],
			.events = POLLOUT,
		};

		if (capture->dmabuf_fd[i] < 0)
			continue;

		poll(&pfd, 1, V4L2R_POLL_TIMEOUT_MS);
	}
}

/*
 * Ensure the surface has a CAPTURE buffer and that the buffer is safe to
 * decode into now. A surface keeps the same buffer for its whole lifetime -
 * so vaExportSurfaceHandle() is stable per VASurfaceID - and the buffer is
 * only allocated (or an orphan from a destroyed surface recycled) on the
 * surface's first decode.
 *
 * Before a later decode overwrites the buffer, wait until its own previous
 * decode has finished and, crucially, until every frame that referenced its
 * previous contents has completed. The kernel matches reference frames purely
 * by CAPTURE buffer timestamp and offers no protection against overwriting
 * one still in use (dev-stateless-decoder.rst), so this wait is what prevents
 * reference corruption.
 */
static int capture_buffer_bind(struct v4l2r_context *ctx,
			       struct v4l2r_surface *surface)
{
	int index = surface->capture_index;

	/* Refresh the completion counter with anything already finished. */
	v4l2r_reap_capture(ctx);

	if (index < 0) {
		/* First decode into this surface: recycle a buffer orphaned by
		 * a destroyed surface, or grow the pool. */
		bool recycled = true;

		index = free_list_pop(ctx);
		if (index < 0) {
			recycled = false;
			index = capture_buffer_new(ctx);
			if (index < 0)
				return index;
		}

		ctx->captures[index].surface = surface;
		surface->ctx = ctx;
		surface->capture_index = index;

		v4l2r_trace("bind surface 0x%08x -> CAPTURE buffer #%d (%s)\n",
			  surface->id, index, recycled ? "recycled" : "new");
	}

	/* Do not overwrite the buffer while its own last decode is in flight,
	 * while any frame still references its current contents, or while
	 * the format converter is still reading it. */
	uint64_t t0 = v4l2r_now_ns();
	v4l2r_sync_capture(ctx, index);
	uint64_t t1 = v4l2r_now_ns();
	v4l2r_wait_completed(ctx, ctx->captures[index].last_ref_seq);
	uint64_t t2 = v4l2r_now_ns();
	v4l2r_convert_drain_index(ctx, index);

	/*
	 * The decoder writes into this buffer and the kernel offers no implicit
	 * synchronisation against an external consumer of the exported dma-buf.
	 * When a GPU is sampling it (e.g. mpv --vo=gpu importing the dma-buf as
	 * an EGL image), reuse would race the decode against the still in-flight
	 * read and tear the picture. POLLOUT on a dma-buf blocks until every
	 * fence on its reservation - including the GPU's read fence - signals, so
	 * wait for readers to finish before handing the buffer back to decode.
	 * No-op for a buffer never exported (fd < 0) or already idle.
	 */
	capture_wait_readers(&ctx->captures[index]);
	uint64_t t3 = v4l2r_now_ns();

	v4l2r_trace("bind wait (surface 0x%08x buf #%d): sync %.2f refwait %.2f "
		    "readers %.2f ms\n", surface->id, index,
		    (t1 - t0) / 1e6, (t2 - t1) / 1e6, (t3 - t2) / 1e6);

	return 0;
}

void v4l2r_context_release_capture(struct v4l2r_context *ctx, int index)
{
	if (index < 0 || index >= (int)ctx->nb_captures)
		return;

	/* Only owned buffers transition to free; ignore an already-free one. */
	if (!ctx->captures[index].surface)
		return;

	/* Orphan the buffer for later recycling. It keeps its last_ref_seq so
	 * a future decode that recycles it still waits out any frame that
	 * referenced its contents. */
	v4l2r_trace("release surface 0x%08x, orphan CAPTURE buffer #%d "
		  "(free list now %u)\n",
		  ctx->captures[index].surface->id, index, ctx->free_count + 1);
	ctx->captures[index].surface->capture_index = -1;
	ctx->captures[index].surface = NULL;
	free_list_push(ctx, index);
}

void v4l2r_flush_surface(struct v4l2r_surface *surface)
{
	if (surface && surface->ctx && surface->ctx->codec &&
	    surface->ctx->codec->flush)
		surface->ctx->codec->flush(surface->ctx, surface);
}

VAStatus v4l2r_context_bind_surface(struct v4l2r_context *ctx,
				    struct v4l2r_surface *surface)
{
	enum v4l2_buf_type type;
	bool starting = !ctx->streaming;
	int ret;

	if (surface->ctx && surface->ctx != ctx)
		return VA_STATUS_ERROR_SURFACE_BUSY;

	if (starting) {
		ret = select_capture_format(ctx);
		if (ret < 0) {
			v4l2r_log("failed to select a CAPTURE format: %s\n",
				  strerror(-ret));
			return VA_STATUS_ERROR_OPERATION_FAILED;
		}

		if (ioctl(ctx->video_fd, VIDIOC_G_FMT, &ctx->capture_format) < 0)
			return VA_STATUS_ERROR_OPERATION_FAILED;

		ret = query_buffer_capabilities(ctx, ctx->capture_format.type,
						&ctx->capture_capabilities);
		if (ret < 0)
			return VA_STATUS_ERROR_OPERATION_FAILED;

		/*
		 * When the decoder can only produce a format nothing can
		 * consume directly (packed 10-bit NV15), chain the hardware
		 * format converter behind it. Without a converter such a
		 * stream cannot be presented at all: fail the decode so the
		 * client falls back to software instead of getting frames
		 * it cannot display.
		 */
		{
			const struct v4l2r_format_info *info =
				v4l2r_format_by_pixelformat(
					v4l2r_format_pixelformat(&ctx->capture_format));

			if (info && !info->va_fourcc) {
				v4l2r_convert_setup(ctx);
				if (!ctx->conv) {
					v4l2r_log("no usable format converter for %.4s, "
						  "refusing hardware decoding\n",
						  (const char *)&info->pixelformat);
					return VA_STATUS_ERROR_OPERATION_FAILED;
				}
			}
		}

		type = ctx->output_format.type;
		if (ioctl(ctx->video_fd, VIDIOC_STREAMON, &type) < 0) {
			v4l2r_log("failed to start OUTPUT streaming: %s\n",
				  strerror(errno));
			return VA_STATUS_ERROR_OPERATION_FAILED;
		}
	}

	/* Bind (and, if reused, drain the references of) the surface's CAPTURE
	 * buffer for this frame's decode. */
	ret = capture_buffer_bind(ctx, surface);
	if (ret < 0)
		return VA_STATUS_ERROR_ALLOCATION_FAILED;

	/* The decode context provides the real storage now; drop any
	 * standalone backing from pre-decode export probing. With a
	 * conversion chain the backing IS the presented storage - keep it
	 * (v4l2r_surface_convert_backing replaces a mismatched one). */
	if (!ctx->conv)
		v4l2r_surface_free_backing(surface);

	if (starting) {
		type = ctx->capture_format.type;
		if (ioctl(ctx->video_fd, VIDIOC_STREAMON, &type) < 0) {
			v4l2r_log("failed to start CAPTURE streaming: %s\n",
				  strerror(errno));
			return VA_STATUS_ERROR_OPERATION_FAILED;
		}

		ctx->streaming = true;

		v4l2r_log("using CAPTURE format %.4s (%ux%u)\n",
			  (const char *)&(uint32_t){v4l2r_format_pixelformat(&ctx->capture_format)},
			  v4l2r_format_width(&ctx->capture_format),
			  v4l2r_format_height(&ctx->capture_format));
	}

	return VA_STATUS_SUCCESS;
}

/* --- VA entrypoints --- */

VAStatus v4l2r_CreateContext(VADriverContextP va_ctx, VAConfigID config_id,
			     int picture_width, int picture_height, int flag,
			     VASurfaceID *render_targets, int num_render_targets,
			     VAContextID *context_id)
{
	struct v4l2r_driver *drv = v4l2r_driver(va_ctx);
	const struct v4l2r_decoder *decoder = NULL;
	struct v4l2r_config *config;
	struct v4l2r_context *ctx;
	struct v4l2_capability capability = {0};
	unsigned int capabilities;
	uint32_t buffersize;
	VAContextID id;
	VAStatus status;
	int ret;

	(void)flag;

	config = V4L2R_CONFIG_GET(drv, config_id);
	if (!config)
		return VA_STATUS_ERROR_INVALID_CONFIG;

	pthread_mutex_lock(&drv->mutex);
	id = v4l2r_handles_alloc(&drv->contexts, sizeof(*ctx));
	ctx = V4L2R_CONTEXT(drv, id);
	pthread_mutex_unlock(&drv->mutex);
	if (!ctx)
		return VA_STATUS_ERROR_ALLOCATION_FAILED;

	ctx->drv = drv;
	ctx->config_id = config_id;
	ctx->profile = config->profile;
	ctx->codec = config->codec;
	ctx->picture_width = picture_width;
	ctx->picture_height = picture_height;
	ctx->bit_depth = v4l2r_profile_bit_depth(config->profile);
	ctx->video_fd = -1;
	ctx->media_fd = -1;
	pthread_mutex_init(&ctx->mutex, NULL);
	for (unsigned int i = 0; i < V4L2R_OUTPUT_BUFFERS; i++)
		ctx->output[i].request_fd = -1;

	/* Video processing contexts drive the format converter instead of a
	 * decoder: no codec, no decoder device, no queues. */
	if (!ctx->codec) {
		status = v4l2r_vpp_create(ctx);
		if (status != VA_STATUS_SUCCESS)
			goto fail;

		*context_id = id;
		return VA_STATUS_SUCCESS;
	}

	if (ctx->codec->priv_size) {
		ctx->codec_priv = calloc(1, ctx->codec->priv_size);
		if (!ctx->codec_priv) {
			status = VA_STATUS_ERROR_ALLOCATION_FAILED;
			goto fail;
		}
	}

	/* Find a decoder that takes this codec and open it. Probing stops at
	 * the first device where the whole OUTPUT setup succeeds. */
	status = VA_STATUS_ERROR_OPERATION_FAILED;
	for (unsigned int i = 0; i < drv->nb_decoders; i++) {
		decoder = &drv->decoders[i];

		ctx->video_fd = open(decoder->video_path, O_RDWR | O_NONBLOCK);
		if (ctx->video_fd < 0)
			continue;

		if (ioctl(ctx->video_fd, VIDIOC_QUERYCAP, &capability) < 0)
			goto next;

		capabilities = (capability.capabilities & V4L2_CAP_DEVICE_CAPS) ?
			       capability.device_caps : capability.capabilities;

		if (capabilities & V4L2_CAP_VIDEO_M2M_MPLANE) {
			ctx->output_format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
			ctx->capture_format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		} else if (capabilities & V4L2_CAP_VIDEO_M2M) {
			ctx->output_format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
			ctx->capture_format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		} else {
			goto next;
		}

		if (!try_output_format(ctx, ctx->codec->pixelformat))
			goto next;

		if (!try_framesize(ctx, ctx->codec->pixelformat))
			goto next;

		/* Initial bitstream buffer size: compressed frames rarely
		 * exceed a quarter of the raw size, and the buffers grow on
		 * demand (v4l2r_output_buffer_grow) - pre-booking the raw
		 * frame size would pin tens of megabytes of CMA per 4K
		 * context across the 4-buffer ring. */
		buffersize = ctx->picture_width * ctx->picture_height / 4;
		if (buffersize < 1024 * 1024)
			buffersize = 1024 * 1024;

		ret = set_format(ctx, ctx->output_format.type,
				 ctx->codec->pixelformat, buffersize);
		if (ret < 0)
			goto next;

		if (ioctl(ctx->video_fd, VIDIOC_G_FMT, &ctx->output_format) < 0)
			goto next;

		/* Query OUTPUT capabilities only now that the coded format is
		 * set: whether the queue supports requests, and crucially whether
		 * it supports holding the CAPTURE buffer across the slices of a
		 * frame (V4L2_BUF_CAP_SUPPORTS_M2M_HOLD_CAPTURE_BUF), which cedrus
		 * only reports once the OUTPUT format is a slice format. Querying
		 * before S_FMT misses it and breaks multi-slice frames (each slice
		 * would re-queue the same CAPTURE buffer). */
		ret = query_buffer_capabilities(ctx, ctx->output_format.type,
						&ctx->output_capabilities);
		if (ret < 0 ||
		    !(ctx->output_capabilities & V4L2_BUF_CAP_SUPPORTS_REQUESTS))
			goto next;

		ctx->media_fd = open(decoder->media_path, O_RDWR);
		if (ctx->media_fd < 0)
			goto next;

		/*
		 * Only accept this decoder if the codec can actually be driven on
		 * it. Some devices match the pixelformat but need controls we
		 * cannot supply (e.g. rkvdec2 HEVC, which requires the SPS RPS
		 * tables); their init rejects the device, so fall through and try
		 * the next candidate. On SoCs exposing several nodes for one codec
		 * (e.g. RK3399: rkvdec + hantro-G2 for HEVC) this lands on the one
		 * we can drive.
		 */
		if (ctx->codec->init) {
			status = ctx->codec->init(ctx);
			if (status != VA_STATUS_SUCCESS) {
				if (ctx->codec->uninit)
					ctx->codec->uninit(ctx);
				if (ctx->codec_priv)
					memset(ctx->codec_priv, 0,
					       ctx->codec->priv_size);
				goto next;
			}
		}

		break;

next:
		if (ctx->media_fd >= 0) {
			close(ctx->media_fd);
			ctx->media_fd = -1;
		}
		close(ctx->video_fd);
		ctx->video_fd = -1;
		decoder = NULL;
	}

	if (ctx->video_fd < 0 || !decoder) {
		v4l2r_log("no drivable decoder for %.4s at %dx%d\n",
			  (const char *)&ctx->codec->pixelformat,
			  picture_width, picture_height);
		goto fail;
	}

	v4l2r_log("decoding %s via %s [%s] (media %s)\n",
		  ctx->codec->name, decoder->video_path, decoder->card,
		  decoder->media_path);

	for (unsigned int i = 0; i < V4L2R_OUTPUT_BUFFERS; i++) {
		ret = output_buffer_setup(ctx, &ctx->output[i], 0);
		if (ret < 0) {
			status = VA_STATUS_ERROR_ALLOCATION_FAILED;
			goto fail;
		}
	}

	/* Bind the passed render targets up front when given; any other
	 * surface gets bound on first use. */
	for (int i = 0; i < num_render_targets; i++) {
		struct v4l2r_surface *surface;

		surface = V4L2R_SURFACE_GET(drv, render_targets[i]);
		if (!surface) {
			status = VA_STATUS_ERROR_INVALID_SURFACE;
			goto fail;
		}
		surface->ctx = ctx;
	}

	*context_id = id;
	return VA_STATUS_SUCCESS;

fail:
	v4l2r_DestroyContext(va_ctx, id);
	return status;
}

VAStatus v4l2r_DestroyContext(VADriverContextP va_ctx, VAContextID context_id)
{
	struct v4l2r_driver *drv = v4l2r_driver(va_ctx);
	struct v4l2r_context *ctx;
	enum v4l2_buf_type type;
	unsigned int iter = 0;
	struct v4l2r_surface *surface;

	ctx = V4L2R_CONTEXT_GET(drv, context_id);
	if (!ctx)
		return VA_STATUS_ERROR_INVALID_CONTEXT;

	if (ctx->video_fd >= 0 && ctx->streaming) {
		type = ctx->output_format.type;
		ioctl(ctx->video_fd, VIDIOC_STREAMOFF, &type);
		type = ctx->capture_format.type;
		ioctl(ctx->video_fd, VIDIOC_STREAMOFF, &type);
	}

	v4l2r_convert_destroy(ctx);
	v4l2r_vpp_destroy(ctx);

	for (unsigned int i = 0; i < V4L2R_OUTPUT_BUFFERS; i++)
		output_buffer_cleanup(ctx, &ctx->output[i]);

	for (unsigned int i = 0; i < ctx->nb_captures; i++)
		capture_buffer_cleanup(ctx, &ctx->captures[i]);

	/* Detach surfaces that were attached but never bound. */
	pthread_mutex_lock(&drv->mutex);
	while ((surface = v4l2r_handles_next(&drv->surfaces, &iter, NULL))) {
		if (surface->ctx == ctx) {
			surface->ctx = NULL;
			surface->capture_index = -1;
		}
	}
	pthread_mutex_unlock(&drv->mutex);

	if (ctx->video_fd >= 0)
		close(ctx->video_fd);
	if (ctx->media_fd >= 0)
		close(ctx->media_fd);

	if (ctx->codec && ctx->codec->uninit && ctx->codec_priv)
		ctx->codec->uninit(ctx);
	free(ctx->codec_priv);
	pthread_mutex_destroy(&ctx->mutex);

	pthread_mutex_lock(&drv->mutex);
	v4l2r_handles_free(&drv->contexts, context_id);
	pthread_mutex_unlock(&drv->mutex);

	return VA_STATUS_SUCCESS;
}

/* --- picture level entrypoints --- */

VAStatus v4l2r_BeginPicture(VADriverContextP va_ctx, VAContextID context_id,
			    VASurfaceID render_target)
{
	struct v4l2r_driver *drv = v4l2r_driver(va_ctx);
	struct v4l2r_context *ctx;
	struct v4l2r_surface *surface;
	VAStatus status;

	ctx = V4L2R_CONTEXT_GET(drv, context_id);
	surface = V4L2R_SURFACE_GET(drv, render_target);

	if (!ctx)
		return VA_STATUS_ERROR_INVALID_CONTEXT;
	if (!surface)
		return VA_STATUS_ERROR_INVALID_SURFACE;
	if (surface->ctx && surface->ctx != ctx)
		return VA_STATUS_ERROR_SURFACE_BUSY;

	if (ctx->vpp)
		return v4l2r_vpp_begin_picture(ctx, surface);

	status = v4l2r_picture_begin(ctx, surface);
	if (status != VA_STATUS_SUCCESS)
		return status;

	ctx->in_picture = true;

	if (ctx->codec->begin_picture)
		return ctx->codec->begin_picture(ctx);

	return VA_STATUS_SUCCESS;
}

VAStatus v4l2r_RenderPicture(VADriverContextP va_ctx, VAContextID context_id,
			     VABufferID *buffers, int num_buffers)
{
	struct v4l2r_driver *drv = v4l2r_driver(va_ctx);
	struct v4l2r_context *ctx;
	VAStatus status;

	ctx = V4L2R_CONTEXT_GET(drv, context_id);
	if (!ctx)
		return VA_STATUS_ERROR_INVALID_CONTEXT;
	if (!ctx->in_picture)
		return VA_STATUS_ERROR_OPERATION_FAILED;

	for (int i = 0; i < num_buffers; i++) {
		struct v4l2r_buffer *buffer;

		buffer = V4L2R_BUFFER_GET(drv, buffers[i]);
		if (!buffer)
			return VA_STATUS_ERROR_INVALID_BUFFER;

		status = ctx->vpp ? v4l2r_vpp_render_buffer(ctx, buffer) :
			 ctx->codec->render_buffer(ctx, buffer);
		if (status != VA_STATUS_SUCCESS)
			return status;
	}

	return VA_STATUS_SUCCESS;
}

VAStatus v4l2r_EndPicture(VADriverContextP va_ctx, VAContextID context_id)
{
	struct v4l2r_driver *drv = v4l2r_driver(va_ctx);
	struct v4l2r_context *ctx;
	VAStatus status;

	ctx = V4L2R_CONTEXT_GET(drv, context_id);
	if (!ctx)
		return VA_STATUS_ERROR_INVALID_CONTEXT;
	if (!ctx->in_picture)
		return VA_STATUS_ERROR_OPERATION_FAILED;

	status = ctx->vpp ? v4l2r_vpp_end_picture(ctx) :
		 ctx->codec->end_picture(ctx);

	ctx->in_picture = false;
	ctx->pic.output = NULL;
	ctx->pic.target = NULL;

	return status;
}
