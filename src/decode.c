/*
 * Decode engine: request submission, buffer queueing and synchronization.
 *
 * Ported from the FFmpeg v4l2-request hwaccel: a circular queue of
 * mmap()ed OUTPUT buffers each paired with a media request, per-frame
 * CAPTURE buffers referenced through timestamps, and slice batching via
 * V4L2_BUF_FLAG_M2M_HOLD_CAPTURE_BUF.
 *
 * Copyright (C) 2026 Ondrej Jirman <megi@xff.cz>
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <errno.h>
#include <poll.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <linux/media.h>
#include <linux/videodev2.h>

#include "v4l2_request.h"

/* Room kept at the tail of the OUTPUT buffer, zeroed before submission. */
#define V4L2R_BITSTREAM_PADDING	64

uint64_t v4l2r_surface_timestamp(struct v4l2r_driver *drv, VASurfaceID id)
{
	struct v4l2r_surface *surface = V4L2R_SURFACE_GET(drv, id);

	if (!surface || surface->capture_index < 0)
		return 0;

	/* Remember that the frame currently being assembled references this
	 * CAPTURE buffer, so its reuse can be gated on that frame completing. */
	if (surface->ctx && surface->ctx->in_picture &&
	    surface->capture_index < V4L2R_MAX_CAPTURE_BUFFERS)
		surface->ctx->pic.ref_mask |=
			UINT64_C(1) << surface->capture_index;

	return v4l2r_capture_index_timestamp(surface->capture_index);
}

int v4l2r_set_controls(struct v4l2r_context *ctx, int request_fd,
		       struct v4l2_ext_control *controls, unsigned int count)
{
	struct v4l2_ext_controls ext_controls = {
		.controls = controls,
		.count = count,
		.request_fd = request_fd,
		.which = (request_fd >= 0) ? V4L2_CTRL_WHICH_REQUEST_VAL : 0,
	};

	if (!controls || !count)
		return 0;

	if (ioctl(ctx->video_fd, VIDIOC_S_EXT_CTRLS, &ext_controls) < 0)
		return -errno;

	return 0;
}

int v4l2r_query_control(struct v4l2r_context *ctx,
			struct v4l2_query_ext_ctrl *control)
{
	if (ioctl(ctx->video_fd, VIDIOC_QUERY_EXT_CTRL, control) < 0)
		return -errno;

	return 0;
}

int v4l2r_query_control_default(struct v4l2r_context *ctx, uint32_t id,
				int64_t *value)
{
	struct v4l2_query_ext_ctrl control = {
		.id = id,
	};
	int ret;

	ret = v4l2r_query_control(ctx, &control);
	if (ret < 0)
		return ret;

	*value = control.default_value;
	return 0;
}

/* --- queue/dequeue primitives, ctx->mutex must be held --- */

static int queue_buffer(struct v4l2r_context *ctx, struct v4l2_buffer *buffer)
{
	struct v4l2_plane planes[VIDEO_MAX_PLANES] = {0};

	if (V4L2_TYPE_IS_MULTIPLANAR(buffer->type)) {
		planes[0].bytesused = buffer->bytesused;
		buffer->bytesused = 0;
		buffer->length = 1;
		buffer->m.planes = planes;
	}

	if (ioctl(ctx->video_fd, VIDIOC_QBUF, buffer) < 0)
		return -errno;

	if (V4L2_TYPE_IS_OUTPUT(buffer->type)) {
		ctx->queued_output |= 1u << buffer->index;
	} else {
		ctx->queued_capture |= UINT64_C(1) << buffer->index;
		/* Count decode submissions on the CAPTURE queue: one per frame,
		 * and completed only when the decode (and thus its reference
		 * reads) actually finishes, unlike the OUTPUT bitstream buffer
		 * which is released as soon as it is consumed. */
		ctx->submitted++;
	}

	return 0;
}

static int queue_capture_buffer(struct v4l2r_context *ctx, uint32_t index)
{
	struct v4l2_buffer buffer = {
		.index = index,
		.type = ctx->capture_format.type,
		.memory = V4L2_MEMORY_MMAP,
	};

	return queue_buffer(ctx, &buffer);
}

static int queue_output_buffer(struct v4l2r_context *ctx,
			       struct v4l2r_output_buffer *output,
			       uint32_t flags)
{
	struct v4l2_buffer buffer = {
		.index = output->index,
		.type = ctx->output_format.type,
		.memory = V4L2_MEMORY_MMAP,
		.timestamp = output->timestamp,
		.bytesused = output->bytesused,
		.request_fd = output->request_fd,
		.flags = V4L2_BUF_FLAG_REQUEST_FD | flags,
	};

	return queue_buffer(ctx, &buffer);
}

static int dequeue_buffer(struct v4l2r_context *ctx, enum v4l2_buf_type type)
{
	struct v4l2_plane planes[VIDEO_MAX_PLANES] = {0};
	struct v4l2_buffer buffer = {
		.type = type,
		.memory = V4L2_MEMORY_MMAP,
	};

	if (V4L2_TYPE_IS_MULTIPLANAR(type)) {
		buffer.length = 1;
		buffer.m.planes = planes;
	}

	if (ioctl(ctx->video_fd, VIDIOC_DQBUF, &buffer) < 0)
		return -errno;

	if (V4L2_TYPE_IS_OUTPUT(type)) {
		ctx->queued_output &= ~(1u << buffer.index);
	} else {
		ctx->queued_capture &= ~(UINT64_C(1) << buffer.index);
		ctx->completed++;
		if (buffer.index < ctx->nb_captures &&
		    ctx->captures[buffer.index].surface) {
			ctx->captures[buffer.index].surface->status =
				VASurfaceReady;
			/* Start the format conversion right away so it
			 * overlaps subsequent decodes. */
			if (ctx->conv)
				v4l2r_convert_kick(ctx, buffer.index);
		}
	}

	return 0;
}

static void dequeue_completed_buffers(struct v4l2r_context *ctx,
				      enum v4l2_buf_type type)
{
	while (!dequeue_buffer(ctx, type))
		continue;
}

static int wait_on_capture_locked(struct v4l2r_context *ctx, uint32_t index)
{
	struct pollfd pollfd = {
		.fd = ctx->video_fd,
		.events = POLLIN,
	};

	if (ctx->queued_capture)
		dequeue_completed_buffers(ctx, ctx->capture_format.type);

	while (ctx->queued_capture & (UINT64_C(1) << index)) {
		int ret = poll(&pollfd, 1, V4L2R_POLL_TIMEOUT_MS);
		if (ret <= 0)
			return -EIO;

		ret = dequeue_buffer(ctx, ctx->capture_format.type);
		if (ret < 0 && ret != -EAGAIN)
			return ret;
	}

	return 0;
}

VAStatus v4l2r_sync_capture(struct v4l2r_context *ctx, int capture_index)
{
	int ret;

	if (capture_index < 0 || !ctx->streaming)
		return VA_STATUS_SUCCESS;

	pthread_mutex_lock(&ctx->mutex);
	ret = wait_on_capture_locked(ctx, capture_index);
	pthread_mutex_unlock(&ctx->mutex);

	if (ret < 0) {
		v4l2r_log("failed waiting on CAPTURE buffer %d\n", capture_index);
		return VA_STATUS_ERROR_OPERATION_FAILED;
	}

	return VA_STATUS_SUCCESS;
}

void v4l2r_reap_capture(struct v4l2r_context *ctx)
{
	pthread_mutex_lock(&ctx->mutex);
	if (ctx->queued_capture)
		dequeue_completed_buffers(ctx, ctx->capture_format.type);
	pthread_mutex_unlock(&ctx->mutex);
}

static int wait_completed_locked(struct v4l2r_context *ctx, uint64_t target)
{
	struct pollfd pollfd = {
		.fd = ctx->video_fd,
		.events = POLLIN,
	};

	if (ctx->queued_capture)
		dequeue_completed_buffers(ctx, ctx->capture_format.type);

	/* Dequeue CAPTURE buffers in order until enough frames have completed.
	 * Completion is in submission order, so this drains exactly the frames
	 * submitted up to the target sequence. */
	while (ctx->completed < target && ctx->queued_capture) {
		int ret = poll(&pollfd, 1, V4L2R_POLL_TIMEOUT_MS);
		if (ret <= 0)
			return -EIO;

		ret = dequeue_buffer(ctx, ctx->capture_format.type);
		if (ret < 0 && ret != -EAGAIN)
			return ret;
	}

	return 0;
}

VAStatus v4l2r_wait_completed(struct v4l2r_context *ctx, uint64_t target)
{
	int ret;

	if (!ctx->streaming || ctx->completed >= target)
		return VA_STATUS_SUCCESS;

	pthread_mutex_lock(&ctx->mutex);
	ret = wait_completed_locked(ctx, target);
	pthread_mutex_unlock(&ctx->mutex);

	if (ret < 0) {
		v4l2r_log("failed waiting for %llu CAPTURE completions (at %llu)\n",
			  (unsigned long long)target,
			  (unsigned long long)ctx->completed);
		return VA_STATUS_ERROR_OPERATION_FAILED;
	}

	return VA_STATUS_SUCCESS;
}

/*
 * Reserve the next OUTPUT buffer from the circular queue, waiting for its
 * previous use to be dequeued first.
 */
static struct v4l2r_output_buffer *next_output(struct v4l2r_context *ctx)
{
	struct v4l2r_output_buffer *output;
	struct pollfd pollfd = {
		.fd = ctx->video_fd,
		.events = POLLOUT,
	};
	uint8_t index;

	pthread_mutex_lock(&ctx->mutex);

	index = ctx->next_output;
	output = &ctx->output[index];
	ctx->next_output = (index + 1) % V4L2R_OUTPUT_BUFFERS;

	if (ctx->queued_output)
		dequeue_completed_buffers(ctx, ctx->output_format.type);

	while (ctx->queued_output & (1u << output->index)) {
		int ret = poll(&pollfd, 1, V4L2R_POLL_TIMEOUT_MS);
		if (ret <= 0)
			goto fail;

		ret = dequeue_buffer(ctx, ctx->output_format.type);
		if (ret < 0 && ret != -EAGAIN)
			goto fail;
	}

	pthread_mutex_unlock(&ctx->mutex);

	output->bytesused = 0;
	return output;

fail:
	pthread_mutex_unlock(&ctx->mutex);
	v4l2r_log("failed waiting on OUTPUT buffer %u\n", output->index);
	return NULL;
}

/*
 * The OUTPUT buffer of the prior use of a request may be dequeued before
 * the request itself has completed (multi stage decoders); wait for the
 * request too before reusing its file descriptor.
 */
static int wait_on_request(struct v4l2r_context *ctx,
			   struct v4l2r_output_buffer *output)
{
	struct pollfd pollfd = {
		.fd = output->request_fd,
		.events = POLLPRI,
	};

	while (ctx->queued_request & (1u << output->index)) {
		int ret = poll(&pollfd, 1, V4L2R_POLL_TIMEOUT_MS);
		if (ret <= 0)
			break;

		if (pollfd.revents & (POLLPRI | POLLERR)) {
			ctx->queued_request &= ~(1u << output->index);
			break;
		}
	}

	if (ioctl(output->request_fd, MEDIA_REQUEST_IOC_REINIT) < 0) {
		v4l2r_log("failed to reinit request %d: %s\n",
			  output->request_fd, strerror(errno));
		return -errno;
	}

	ctx->queued_request &= ~(1u << output->index);

	return 0;
}

VAStatus v4l2r_picture_begin(struct v4l2r_context *ctx,
			     struct v4l2r_surface *target)
{
	struct v4l2r_output_buffer *output;

	output = next_output(ctx);
	if (!output)
		return VA_STATUS_ERROR_OPERATION_FAILED;

	/* The target's CAPTURE buffer is bound (and, if reused, drained of its
	 * references) at decode time, so there is nothing to sync here; only
	 * reserve the OUTPUT buffer and reset the per-frame reference set. */
	ctx->pic.output = output;
	ctx->pic.target = target;
	ctx->pic.ref_mask = 0;

	return VA_STATUS_SUCCESS;
}

VAStatus v4l2r_picture_next_output(struct v4l2r_context *ctx)
{
	struct v4l2r_output_buffer *output = next_output(ctx);

	if (!output)
		return VA_STATUS_ERROR_OPERATION_FAILED;

	ctx->pic.output = output;
	return VA_STATUS_SUCCESS;
}

VAStatus v4l2r_append_output(struct v4l2r_context *ctx, const void *data,
			     size_t size)
{
	struct v4l2r_output_buffer *output = ctx->pic.output;
	size_t needed;

	if (!output)
		return VA_STATUS_ERROR_OPERATION_FAILED;

	needed = (size_t)output->bytesused + size + V4L2R_BITSTREAM_PADDING;
	if (needed > output->size &&
	    v4l2r_output_buffer_grow(ctx, output, needed) < 0) {
		v4l2r_log("bitstream data (%zu bytes) overflows OUTPUT buffer %u (%u of %u used)\n",
			  size, output->index, output->bytesused, output->size);
		return VA_STATUS_ERROR_ALLOCATION_FAILED;
	}

	memcpy(output->addr + output->bytesused, data, size);
	output->bytesused += size;

	return VA_STATUS_SUCCESS;
}

static VAStatus queue_decode(struct v4l2r_context *ctx,
			     struct v4l2_ext_control *controls,
			     unsigned int count,
			     bool first_slice, bool last_slice)
{
	struct v4l2r_output_buffer *output = ctx->pic.output;
	struct v4l2r_surface *target = ctx->pic.target;
	uint32_t flags;
	int ret;

	if (!output || !target || target->capture_index < 0)
		return VA_STATUS_ERROR_OPERATION_FAILED;

	pthread_mutex_lock(&ctx->mutex);

	ret = wait_on_request(ctx, output);
	if (ret < 0)
		goto fail;

	if (ctx->queued_output)
		dequeue_completed_buffers(ctx, ctx->output_format.type);

	ret = v4l2r_set_controls(ctx, output->request_fd, controls, count);
	if (ret < 0) {
		v4l2r_log("failed to set %u control(s) for request %d: %s\n",
			  count, output->request_fd, strerror(-ret));
		goto fail;
	}

	/* Zero padding after the bitstream data, some decoders require it. */
	memset(output->addr + output->bytesused, 0, V4L2R_BITSTREAM_PADDING);

	/* The CAPTURE buffer index is the base for V4L2 frame references. */
	output->timestamp = (struct timeval) {
		.tv_sec = 0,
		.tv_usec = target->capture_index + 1,
	};

	flags = last_slice ? 0 : V4L2_BUF_FLAG_M2M_HOLD_CAPTURE_BUF;
	ret = queue_output_buffer(ctx, output, flags);
	if (ret < 0) {
		v4l2r_log("failed to queue OUTPUT buffer %u: %s\n",
			  output->index, strerror(-ret));
		goto fail;
	}

	if (first_slice) {
		/* Queue the specific CAPTURE buffer tied to the target
		 * surface so frames land where VA expects them. */
		ret = queue_capture_buffer(ctx, target->capture_index);
		if (ret < 0) {
			v4l2r_log("failed to queue CAPTURE buffer %d: %s\n",
				  target->capture_index, strerror(-ret));
			goto fail;
		}

		/* ctx->submitted is now this frame's sequence number. Mark every
		 * buffer it references as needed until this frame completes, so a
		 * later decode cannot overwrite a reference still in use. */
		for (unsigned int i = 0; i < ctx->nb_captures; i++) {
			if (ctx->pic.ref_mask & (UINT64_C(1) << i))
				ctx->captures[i].last_ref_seq = ctx->submitted;
		}
	}

	if (ioctl(output->request_fd, MEDIA_REQUEST_IOC_QUEUE) < 0) {
		ret = -errno;
		v4l2r_log("failed to queue request %d: %s\n",
			  output->request_fd, strerror(errno));
		/* The OUTPUT buffer stays bound to the still-idle request and
		 * will never be dequeued; the REINIT in wait_on_request()
		 * releases it when the ring comes back around, so waiting for
		 * a dequeue would only stall next_output(). */
		ctx->queued_output &= ~(1u << output->index);
		goto fail;
	}

	ctx->queued_request |= 1u << output->index;
	target->status = VASurfaceRendering;

	pthread_mutex_unlock(&ctx->mutex);
	return VA_STATUS_SUCCESS;

fail:
	pthread_mutex_unlock(&ctx->mutex);
	return VA_STATUS_ERROR_OPERATION_FAILED;
}

VAStatus v4l2r_decode(struct v4l2r_context *ctx,
		      struct v4l2_ext_control *controls, unsigned int count,
		      bool first_slice, bool last_slice)
{
	VAStatus status;

	/* First submission of the context configures the CAPTURE side using
	 * the codec controls that are now known (e.g. SPS bit depth). Set
	 * them without a request first so the driver can base its CAPTURE
	 * format decision on them. */
	if (!ctx->streaming) {
		int ret = v4l2r_set_controls(ctx, -1, controls, count);
		if (ret < 0)
			v4l2r_log("failed to set initial controls: %s\n",
				  strerror(-ret));
	}

	/* Bind a fresh CAPTURE buffer to the target once per frame (on its
	 * first slice); this also performs the one-time CAPTURE queue setup
	 * on the very first decode and syncs the picked buffer. */
	if (first_slice) {
		status = v4l2r_context_bind_surface(ctx, ctx->pic.target);
		if (status != VA_STATUS_SUCCESS)
			return status;
	}

	/* Without HOLD_CAPTURE_BUF support every slice must be submitted as
	 * a full frame. */
	if (!(ctx->output_capabilities & V4L2_BUF_CAP_SUPPORTS_M2M_HOLD_CAPTURE_BUF))
		return queue_decode(ctx, controls, count, true, true);

	return queue_decode(ctx, controls, count, first_slice, last_slice);
}
