/*
 * libva-v4l2request - a VA-API backend for V4L2 stateless (Request API)
 * video decoders.
 *
 * The overall decode flow follows the FFmpeg v4l2-request hwaccel design:
 * a small circular queue of mmap()ed OUTPUT buffers, each with its own
 * media request, carries the bitstream and codec controls.
 *
 * A VA surface is bound to one CAPTURE buffer on its first decode and keeps
 * it for the surface's whole lifetime, so vaExportSurfaceHandle() always
 * returns the same dma-buf for a given VASurfaceID - dma-buf consumers that
 * cache imports per surface id (e.g. mpv --vo=dmabuf-wayland) rely on that
 * stability. Frames reference each other through the buffer timestamp, which
 * encodes the CAPTURE buffer index. Because the kernel gives no protection
 * against overwriting a reference (see dev-stateless-decoder.rst: a decoded
 * buffer must not be reused as a decode target until every frame referencing
 * it has been dequeued), reuse of a surface's buffer for a new decode is
 * gated: the driver waits until all frames that referenced its previous
 * contents have completed before overwriting it.
 *
 * Copyright (C) 2026 Ondrej Jirman <megi@xff.cz>
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef V4L2_REQUEST_H
#define V4L2_REQUEST_H

#include <linux/videodev2.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#include <va/va.h>
#include <va/va_backend.h>
#include <va/va_vpp.h>

#include "config.h"

/* Monotonic timestamp in nanoseconds, for measuring how long waits block. */
static inline uint64_t v4l2r_now_ns(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000000ull + ts.tv_nsec;
}

#define V4L2R_MAX_PROFILES		32
#define V4L2R_MAX_ENTRYPOINTS		4
#define V4L2R_MAX_CONFIG_ATTRIBUTES	16
#define V4L2R_MAX_IMAGE_FORMATS		8
#define V4L2R_MAX_SUBPIC_FORMATS	1
#define V4L2R_MAX_DISPLAY_ATTRIBUTES	1
#define V4L2R_STR_VENDOR		"v4l2-request"

#define V4L2R_MAX_DECODERS		8
#define V4L2R_MAX_PIXELFORMATS		16

/* Number of OUTPUT (bitstream) buffers in the per-context circular queue. */
#define V4L2R_OUTPUT_BUFFERS		4
/* Largest CAPTURE buffer count we can track in the queued bitmask. With a
 * per-frame buffer pool the count reaches roughly the client's live surface
 * count plus the pipeline depth, so keep headroom up to the 64-bit bitmask. */
#define V4L2R_MAX_CAPTURE_BUFFERS	64

/* Upper bound on any single wait for the hardware (buffer/request/fence). */
#define V4L2R_POLL_TIMEOUT_MS		2000

/* In-flight job slots on the format converter, per decode context. */
#define V4L2R_CONVERT_SLOTS		4

#define V4L2R_ID_OFFSET_CONFIG		0x01000000
#define V4L2R_ID_OFFSET_CONTEXT		0x02000000
#define V4L2R_ID_OFFSET_SURFACE		0x04000000
#define V4L2R_ID_OFFSET_BUFFER		0x08000000
#define V4L2R_ID_OFFSET_IMAGE		0x10000000

struct v4l2r_driver;
struct v4l2r_context;
struct v4l2r_codec;

void v4l2r_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
void v4l2r_trace(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/*
 * One V4L2 stateless decoder found during device enumeration: the media
 * device it hangs off and the coded (OUTPUT queue) pixelformats it accepts.
 */
struct v4l2r_decoder {
	char media_path[256];
	char video_path[256];
	char card[32];
	uint32_t pixelformats[V4L2R_MAX_PIXELFORMATS];
	unsigned int nb_pixelformats;
	/* Whether this decoder accepts a 10-bit HEVC SPS. Not every HEVC-capable
	 * device can do 10-bit (e.g. Allwinner A64 lacks it), and the kernel only
	 * rejects it at decode time, so probe it up front and gate Main10. */
	bool hevc_10bit;
};

/*
 * A plain (non-request) V4L2 mem2mem format converter, e.g. the Rockchip
 * RGA. Used to turn decoder-only CAPTURE formats (the packed 10-bit NV15)
 * into NV12 that displays and clients can consume. pixelformats lists the
 * source (OUTPUT queue) formats the device can read.
 */
struct v4l2r_converter {
	char video_path[256];
	char card[32];
	uint32_t pixelformats[V4L2R_MAX_PIXELFORMATS];
	unsigned int nb_pixelformats;
};

/*
 * Trivial handle table: VA object ids are offset + slot index. Slots hold
 * malloc()ed objects, lookups are O(1). Protected by the driver mutex.
 */
struct v4l2r_handles {
	void **slots;
	unsigned int size;
	uint32_t id_offset;
};

int v4l2r_handles_init(struct v4l2r_handles *h, uint32_t id_offset);
void v4l2r_handles_destroy(struct v4l2r_handles *h);
/* Allocates a zeroed object of object_size, returns its id or VA_INVALID_ID. */
uint32_t v4l2r_handles_alloc(struct v4l2r_handles *h, size_t object_size);
void *v4l2r_handles_lookup(struct v4l2r_handles *h, uint32_t id);
void v4l2r_handles_free(struct v4l2r_handles *h, uint32_t id);
/* Iterate live objects; *iter starts at 0. Returns NULL at the end. */
void *v4l2r_handles_next(struct v4l2r_handles *h, unsigned int *iter,
			 uint32_t *id);

/* Grow a realloc()ed array to hold at least needed elements. Element count
 * capacity is kept in *alloc_elements; returns -1 on allocation failure. */
int v4l2r_array_reserve(void **array, unsigned int *alloc_elements,
			unsigned int needed, size_t element_size);

struct v4l2r_config {
	VAProfile profile;
	VAEntrypoint entrypoint;
	/* NULL for video processing configs (VAProfileNone/VideoProc). */
	const struct v4l2r_codec *codec;
	unsigned int rt_format;
	VAConfigAttrib attributes[V4L2R_MAX_CONFIG_ATTRIBUTES];
	int nb_attributes;
};

/*
 * Standalone dma-buf backing for surfaces that have not (yet) been bound
 * to a decode context. Allocated from a decoder CAPTURE queue on a
 * throwaway fd; the exported dma-bufs keep the memory alive after the fd
 * is closed. Needed for clients (e.g. mpv) that export and map bare test
 * surfaces while probing formats.
 */
struct v4l2r_surface_backing {
	uint32_t pixelformat;
	uint32_t width;
	uint32_t height;
	uint32_t pitch;
	unsigned int nb_planes;
	uint32_t plane_size[VIDEO_MAX_PLANES];
	int dmabuf_fd[VIDEO_MAX_PLANES];
	void *map[VIDEO_MAX_PLANES];
};

struct v4l2r_surface {
	struct v4l2r_context *ctx;	/* context the surface is bound to */
	VASurfaceID id;			/* own VA id, for logging/tracing */
	unsigned int width;
	unsigned int height;
	unsigned int rt_format;
	uint32_t fourcc;		/* requested pixel format, 0 = default */
	int capture_index;		/* CAPTURE buffer index, -1 if unbound */
	VASurfaceStatus status;
	struct v4l2r_surface_backing *backing;
	/* A finished decode still needs (or is undergoing) format conversion
	 * into the backing. Protected by ctx->mutex. */
	bool convert_pending;
	/* Codec scratch, e.g. the AV1 order hint of the decoded frame. */
	uint32_t codec_tag;
};

/*
 * Unified view of the memory behind a surface, whether it comes from a
 * context CAPTURE buffer or from standalone backing.
 */
struct v4l2r_frame_view {
	const struct v4l2r_format_info *info;
	uint32_t pixelformat;
	uint32_t width;
	uint32_t height;
	uint32_t pitch;
	unsigned int nb_planes;
	uint32_t plane_size[VIDEO_MAX_PLANES];
	int dmabuf_fd[VIDEO_MAX_PLANES];	/* borrowed, do not close */
	void *map[VIDEO_MAX_PLANES];		/* valid when maps requested */
};

/* Resolve the memory view of a surface, allocating standalone backing
 * for unbound surfaces on demand. */
VAStatus v4l2r_surface_view(struct v4l2r_driver *drv,
			    struct v4l2r_surface *surface, bool need_maps,
			    struct v4l2r_frame_view *view);
void v4l2r_surface_free_backing(struct v4l2r_surface *surface);

/* Per CAPTURE buffer state, lives in the context that allocated it. The
 * buffer holds the current decoded content of ->surface, or is free when
 * ->surface is NULL (its index then sits in the context free list). */
struct v4l2r_capture_buffer {
	struct v4l2r_surface *surface;
	unsigned int nb_planes;
	uint32_t plane_mem_offset[VIDEO_MAX_PLANES];
	uint32_t plane_size[VIDEO_MAX_PLANES];
	int dmabuf_fd[VIDEO_MAX_PLANES];	/* exported lazily, -1 if not */
	void *map[VIDEO_MAX_PLANES];		/* mmap()ed lazily, NULL if not */
	/* Submission count of the most recent frame that used this buffer as a
	 * reference. A new decode may not overwrite the buffer until
	 * ctx->completed reaches this value, i.e. until every frame that
	 * references its current contents has finished decoding. */
	uint64_t last_ref_seq;
};

struct v4l2r_output_buffer {
	uint32_t index;
	int request_fd;
	uint8_t *addr;
	uint32_t size;
	uint32_t bytesused;
	struct timeval timestamp;
};

/* State of the picture being assembled between Begin/EndPicture. */
struct v4l2r_picture {
	struct v4l2r_output_buffer *output;
	struct v4l2r_surface *target;
	/* Bitmask of CAPTURE buffer indices this frame references, accumulated
	 * as the codec resolves reference timestamps. Used to mark those
	 * buffers as needed until this frame completes. */
	uint64_t ref_mask;
};

struct v4l2r_buffer {
	VABufferType type;
	unsigned int element_size;
	unsigned int nb_elements;
	void *data;
	/* Set for images derived from a surface: data points into the
	 * CAPTURE buffer mapping and must not be freed. */
	bool derived;
};

struct v4l2r_image {
	VAImage image;
};

/* Payload byte size of a VA buffer (element_size * nb_elements). Client
 * supplied, so validate offsets/sizes against it before dereferencing. */
static inline size_t v4l2r_buffer_bytes(const struct v4l2r_buffer *buf)
{
	return (size_t)buf->element_size * buf->nb_elements;
}

struct v4l2r_context {
	struct v4l2r_driver *drv;
	VAConfigID config_id;
	VAProfile profile;
	const struct v4l2r_codec *codec;
	void *codec_priv;

	int video_fd;
	int media_fd;

	unsigned int picture_width;
	unsigned int picture_height;
	unsigned int bit_depth;		/* hint from the VA profile */

	struct v4l2_format output_format;
	struct v4l2_format capture_format;
	uint32_t output_capabilities;
	uint32_t capture_capabilities;
	bool streaming;			/* CAPTURE side fully configured */

	struct v4l2r_output_buffer output[V4L2R_OUTPUT_BUFFERS];
	uint8_t next_output;
	uint32_t queued_output;
	uint32_t queued_request;
	uint64_t queued_capture;
	/* Monotonic CAPTURE queue/dequeue counters (one per decoded frame).
	 * A CAPTURE dequeue signals the decode - and thus all of that frame's
	 * reference reads - has finished. Completion is in submission order on
	 * a stateless m2m decoder, so waiting for completed >= S waits for the
	 * first S submitted frames (and their reference reads) to finish. */
	uint64_t submitted;
	uint64_t completed;
	pthread_mutex_t mutex;

	struct v4l2r_capture_buffer captures[V4L2R_MAX_CAPTURE_BUFFERS];
	unsigned int nb_captures;
	/* FIFO of orphaned CAPTURE buffer indices left behind by destroyed
	 * surfaces (V4L2 cannot free individual buffers); reused when a new
	 * surface is bound for the first time. */
	uint8_t free_captures[V4L2R_MAX_CAPTURE_BUFFERS];
	unsigned int free_head;
	unsigned int free_count;

	struct v4l2r_picture pic;
	bool in_picture;

	/* Format conversion chain (NULL when the CAPTURE format is directly
	 * consumable); see convert.c. */
	struct v4l2r_convert *conv;

	/* Video processing state for VAEntrypointVideoProc contexts; such
	 * contexts have no codec, decoder device or queues. See convert.c. */
	struct v4l2r_vpp *vpp;
};

/*
 * Per-context state of the format converter chain: decoded frames are run
 * through a mem2mem converter (decoder CAPTURE dma-buf in, surface backing
 * dma-buf out) as soon as their decode finishes. All fields are protected
 * by ctx->mutex.
 */
struct v4l2r_convert {
	int fd;
	struct v4l2_format output_format;	/* source = decoder CAPTURE format */
	struct v4l2_format capture_format;	/* destination, NV12 */
	uint32_t busy;				/* bitmask of in-flight slots */
	unsigned int next_slot;
	struct {
		int capture_index;		/* decoder CAPTURE buffer read */
		struct v4l2r_surface *surface;	/* destination surface */
	} jobs[V4L2R_CONVERT_SLOTS];
	bool failed;			/* chain broke (layout/queue error), disabled */
};

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

/* Submit any decode a codec is holding back for this surface (no-op unless
 * the surface belongs to a context whose codec defers submission). */
void v4l2r_flush_surface(struct v4l2r_surface *surface);

struct v4l2r_driver {
	struct v4l2r_handles configs;
	struct v4l2r_handles contexts;
	struct v4l2r_handles surfaces;
	struct v4l2r_handles buffers;
	struct v4l2r_handles images;
	pthread_mutex_t mutex;

	struct v4l2r_decoder decoders[V4L2R_MAX_DECODERS];
	unsigned int nb_decoders;

	/* Converter detection is deferred until something can actually use
	 * one (see v4l2r_converter_available). */
	struct v4l2r_converter converter;
	bool converter_probed;
	bool has_converter;
};

static inline struct v4l2r_driver *v4l2r_driver(VADriverContextP va_ctx)
{
	return va_ctx->pDriverData;
}

#define V4L2R_CONFIG(drv, id) \
	((struct v4l2r_config *)v4l2r_handles_lookup(&(drv)->configs, id))
#define V4L2R_CONTEXT(drv, id) \
	((struct v4l2r_context *)v4l2r_handles_lookup(&(drv)->contexts, id))
#define V4L2R_SURFACE(drv, id) \
	((struct v4l2r_surface *)v4l2r_handles_lookup(&(drv)->surfaces, id))
#define V4L2R_BUFFER(drv, id) \
	((struct v4l2r_buffer *)v4l2r_handles_lookup(&(drv)->buffers, id))
#define V4L2R_IMAGE(drv, id) \
	((struct v4l2r_image *)v4l2r_handles_lookup(&(drv)->images, id))

/* Locked lookup for entrypoint prologues (drv->mutex must not be held);
 * the bare V4L2R_* macros above are for use under the driver mutex. */
static inline void *v4l2r_handles_get(struct v4l2r_driver *drv,
				      struct v4l2r_handles *h, uint32_t id)
{
	void *object;

	pthread_mutex_lock(&drv->mutex);
	object = v4l2r_handles_lookup(h, id);
	pthread_mutex_unlock(&drv->mutex);

	return object;
}

#define V4L2R_CONFIG_GET(drv, id) \
	((struct v4l2r_config *)v4l2r_handles_get(drv, &(drv)->configs, id))
#define V4L2R_CONTEXT_GET(drv, id) \
	((struct v4l2r_context *)v4l2r_handles_get(drv, &(drv)->contexts, id))
#define V4L2R_SURFACE_GET(drv, id) \
	((struct v4l2r_surface *)v4l2r_handles_get(drv, &(drv)->surfaces, id))
#define V4L2R_BUFFER_GET(drv, id) \
	((struct v4l2r_buffer *)v4l2r_handles_get(drv, &(drv)->buffers, id))
#define V4L2R_IMAGE_GET(drv, id) \
	((struct v4l2r_image *)v4l2r_handles_get(drv, &(drv)->images, id))

/* Accessors hiding the single/multi-planar v4l2_format split. */
static inline uint32_t v4l2r_format_pixelformat(const struct v4l2_format *format)
{
	return V4L2_TYPE_IS_MULTIPLANAR(format->type) ?
	       format->fmt.pix_mp.pixelformat : format->fmt.pix.pixelformat;
}

static inline uint32_t v4l2r_format_width(const struct v4l2_format *format)
{
	return V4L2_TYPE_IS_MULTIPLANAR(format->type) ?
	       format->fmt.pix_mp.width : format->fmt.pix.width;
}

static inline uint32_t v4l2r_format_height(const struct v4l2_format *format)
{
	return V4L2_TYPE_IS_MULTIPLANAR(format->type) ?
	       format->fmt.pix_mp.height : format->fmt.pix.height;
}

static inline uint32_t v4l2r_format_bytesperline(const struct v4l2_format *format)
{
	return V4L2_TYPE_IS_MULTIPLANAR(format->type) ?
	       format->fmt.pix_mp.plane_fmt[0].bytesperline :
	       format->fmt.pix.bytesperline;
}

static inline uint32_t v4l2r_format_num_planes(const struct v4l2_format *format)
{
	return V4L2_TYPE_IS_MULTIPLANAR(format->type) ?
	       format->fmt.pix_mp.num_planes : 1;
}

/*
 * CAPTURE pixelformats the driver knows how to hand out, together with the
 * DRM description used for dma-buf export. Mirrors the FFmpeg hwcontext
 * table.
 */
struct v4l2r_format_info {
	uint32_t pixelformat;
	uint32_t drm_format;
	uint64_t drm_modifier;
	uint32_t va_fourcc;		/* 0 when not representable as VAImage */
	unsigned int rt_format;
	unsigned int bit_depth;
	bool linear;			/* mappable for DeriveImage/GetImage */
};

const struct v4l2r_format_info *v4l2r_format_by_pixelformat(uint32_t pixelformat);
const struct v4l2r_format_info *v4l2r_capture_format_info(struct v4l2r_context *ctx);

/* --- decode engine (decode.c) --- */

/*
 * Frame referencing: the CAPTURE buffer index is turned into a buffer
 * timestamp, matching v4l2_timeval_to_ns() on the kernel side.
 */
static inline uint64_t v4l2r_capture_index_timestamp(int index)
{
	return ((uint64_t)index + 1) * 1000;
}

/* Timestamp of the CAPTURE buffer a reference surface was decoded into.
 * Returns 0 when the surface is invalid or was never decoded. */
uint64_t v4l2r_surface_timestamp(struct v4l2r_driver *drv, VASurfaceID id);

int v4l2r_set_controls(struct v4l2r_context *ctx, int request_fd,
		       struct v4l2_ext_control *controls, unsigned int count);
int v4l2r_query_control(struct v4l2r_context *ctx,
			struct v4l2_query_ext_ctrl *control);
int v4l2r_query_control_default(struct v4l2r_context *ctx, uint32_t id,
				int64_t *value);

/* Reserve the next OUTPUT buffer of the circular queue for the current
 * picture, waiting for it to be dequeued if it is still in flight. */
VAStatus v4l2r_picture_begin(struct v4l2r_context *ctx,
			     struct v4l2r_surface *target);
/* Move to a fresh OUTPUT buffer, used between per-slice requests. */
VAStatus v4l2r_picture_next_output(struct v4l2r_context *ctx);

/* Append bitstream data to the current OUTPUT buffer, growing it when the
 * staged bitstream outgrows the allocation. */
VAStatus v4l2r_append_output(struct v4l2r_context *ctx, const void *data,
			     size_t size);

/* Replace an OUTPUT buffer with a larger one, preserving its content
 * (context.c). Used when a frame's bitstream overflows the buffer. */
int v4l2r_output_buffer_grow(struct v4l2r_context *ctx,
			     struct v4l2r_output_buffer *output,
			     size_t min_size);

/* Submit the current request. When the codec runs slice based decoding,
 * first/last describe the slice position within the frame and the CAPTURE
 * buffer is held between slices. */
VAStatus v4l2r_decode(struct v4l2r_context *ctx,
		      struct v4l2_ext_control *controls, unsigned int count,
		      bool first_slice, bool last_slice);

/* Block until the CAPTURE buffer of the surface has been dequeued. */
VAStatus v4l2r_sync_capture(struct v4l2r_context *ctx, int capture_index);

/* Block until at least the first target frames have completed decoding, i.e.
 * until ctx->completed >= target. Used to drain the frames that reference a
 * buffer before it is overwritten by a new decode. */
VAStatus v4l2r_wait_completed(struct v4l2r_context *ctx, uint64_t target);

/* Non-blocking dequeue sweep, used by vaQuerySurfaceStatus. */
void v4l2r_reap_capture(struct v4l2r_context *ctx);

/* --- context management (context.c) --- */

/* Bind the surface's CAPTURE buffer for the frame about to be decoded
 * (allocating it on the surface's first decode), performing the one-time
 * CAPTURE queue setup on the first call, and waiting until the buffer is no
 * longer needed as a reference before it is reused. */
VAStatus v4l2r_context_bind_surface(struct v4l2r_context *ctx,
				    struct v4l2r_surface *surface);

/* Return a destroyed surface's CAPTURE buffer to the orphan free list. */
void v4l2r_context_release_capture(struct v4l2r_context *ctx, int index);

/* --- format converter chain (convert.c) --- */

/* Whether a usable format converter exists, probing the device nodes on
 * the first call. Only invoked when a converter could actually matter
 * (10-bit configs, undisplayable CAPTURE formats), so plain 8-bit clients
 * never pay for the scan. */
bool v4l2r_converter_available(struct v4l2r_driver *drv);

/* Whether the detected converter can read this pixelformat. */
bool v4l2r_converter_supports(struct v4l2r_driver *drv, uint32_t pixelformat);

/* Set up the conversion chain for the context's selected CAPTURE format.
 * Called once from the CAPTURE setup path; on failure the context decodes
 * without conversion (and 10-bit output stays NV15). */
VAStatus v4l2r_convert_setup(struct v4l2r_context *ctx);
void v4l2r_convert_destroy(struct v4l2r_context *ctx);

/* Queue the conversion of a just-decoded CAPTURE buffer into its surface's
 * backing. Called with ctx->mutex held, right after the CAPTURE dequeue. */
void v4l2r_convert_kick(struct v4l2r_context *ctx, int capture_index);

/* Wait until no in-flight conversion reads this CAPTURE buffer anymore,
 * before the buffer is overwritten by a new decode. Takes ctx->mutex. */
void v4l2r_convert_drain_index(struct v4l2r_context *ctx, int capture_index);

/* Wait for the surface's pending conversion to finish. Takes ctx->mutex. */
VAStatus v4l2r_convert_wait(struct v4l2r_surface *surface);

/* --- video processing (convert.c) --- */

/* Per-context state of a VAEntrypointVideoProc context: rotation, mirroring
 * and scaling blits between surfaces, run on the format converter. */
struct v4l2r_vpp {
	int fd;				/* configured converter, -1 until first blit */
	struct v4l2_format output_format;	/* source */
	struct v4l2_format capture_format;	/* destination, NV12 */

	/* Geometry the converter is currently configured for; a change in
	 * any of these forces a reconfiguration (V4L2 formats cannot change
	 * while buffers are allocated). */
	uint32_t src_pixelformat;
	uint32_t src_width, src_height, src_pitch;
	uint32_t dst_width, dst_height, dst_pitch;

	/* Pipeline parameters of the picture being assembled. */
	struct v4l2r_surface *src_surface;
	VARectangle src_region;
	VARectangle dst_region;
	uint32_t rotation;		/* VA_ROTATION_* */
	uint32_t mirror;		/* VA_MIRROR_* mask */
	bool have_params;
};

VAStatus v4l2r_vpp_create(struct v4l2r_context *ctx);
void v4l2r_vpp_destroy(struct v4l2r_context *ctx);

VAStatus v4l2r_vpp_begin_picture(struct v4l2r_context *ctx,
				 struct v4l2r_surface *target);
VAStatus v4l2r_vpp_render_buffer(struct v4l2r_context *ctx,
				 struct v4l2r_buffer *buf);
VAStatus v4l2r_vpp_end_picture(struct v4l2r_context *ctx);

VAStatus v4l2r_QueryVideoProcFilters(VADriverContextP va_ctx,
				     VAContextID context_id,
				     VAProcFilterType *filters,
				     unsigned int *num_filters);
VAStatus v4l2r_QueryVideoProcFilterCaps(VADriverContextP va_ctx,
					VAContextID context_id,
					VAProcFilterType type,
					void *filter_caps,
					unsigned int *num_filter_caps);
VAStatus v4l2r_QueryVideoProcPipelineCaps(VADriverContextP va_ctx,
					  VAContextID context_id,
					  VABufferID *filters,
					  unsigned int num_filters,
					  VAProcPipelineCaps *pipeline_caps);

/* --- surface helpers (surface.c) --- */

/* Make a surface's pixels available for reading: submit a held-back decode,
 * wait for the decode itself and for any pending format conversion. */
VAStatus v4l2r_surface_ready(struct v4l2r_surface *surface);

/* View of a bound surface's decode CAPTURE buffer itself, even when a
 * conversion chain makes v4l2r_surface_view() present the converted backing
 * instead. Lets the video processing blit read the raw decoder output (e.g.
 * packed 10-bit NV15) and fold the downconversion into the same pass. */
VAStatus v4l2r_surface_capture_view(struct v4l2r_surface *surface,
				    bool need_maps,
				    struct v4l2r_frame_view *view);

/* Export the dma-buf fds of a context CAPTURE buffer (idempotent). */
int v4l2r_export_capture_dmabufs(struct v4l2r_context *ctx,
				 struct v4l2r_capture_buffer *capture,
				 int capture_index);

/* Ensure the surface has standalone backing matching the converter's
 * destination format (NV12 at the converter CAPTURE layout). */
VAStatus v4l2r_surface_convert_backing(struct v4l2r_driver *drv,
				       struct v4l2r_surface *surface);

/* --- codec table --- */

extern const struct v4l2r_codec v4l2r_codec_mpeg2;
extern const struct v4l2r_codec v4l2r_codec_h264;
extern const struct v4l2r_codec v4l2r_codec_hevc;
extern const struct v4l2r_codec v4l2r_codec_vp8;
extern const struct v4l2r_codec v4l2r_codec_vp9;
extern const struct v4l2r_codec v4l2r_codec_av1;

const struct v4l2r_codec *v4l2r_codec_for_profile(VAProfile profile);
const struct v4l2r_codec **v4l2r_codec_list(unsigned int *count);
unsigned int v4l2r_profile_bit_depth(VAProfile profile);
unsigned int v4l2r_profile_rt_format(VAProfile profile);

/* --- VA entrypoints --- */

VAStatus v4l2r_Terminate(VADriverContextP va_ctx);
VAStatus v4l2r_QueryConfigProfiles(VADriverContextP va_ctx, VAProfile *profiles,
				   int *num_profiles);
VAStatus v4l2r_QueryConfigEntrypoints(VADriverContextP va_ctx, VAProfile profile,
				      VAEntrypoint *entrypoints,
				      int *num_entrypoints);
VAStatus v4l2r_QueryConfigAttributes(VADriverContextP va_ctx,
				     VAConfigID config_id, VAProfile *profile,
				     VAEntrypoint *entrypoint,
				     VAConfigAttrib *attrib_list,
				     int *num_attribs);
VAStatus v4l2r_CreateConfig(VADriverContextP va_ctx, VAProfile profile,
			    VAEntrypoint entrypoint, VAConfigAttrib *attrib_list,
			    int num_attribs, VAConfigID *config_id);
VAStatus v4l2r_DestroyConfig(VADriverContextP va_ctx, VAConfigID config_id);
VAStatus v4l2r_GetConfigAttributes(VADriverContextP va_ctx, VAProfile profile,
				   VAEntrypoint entrypoint,
				   VAConfigAttrib *attrib_list, int num_attribs);

VAStatus v4l2r_CreateSurfaces(VADriverContextP va_ctx, int width, int height,
			      int format, int num_surfaces,
			      VASurfaceID *surfaces);
VAStatus v4l2r_CreateSurfaces2(VADriverContextP va_ctx, unsigned int format,
			       unsigned int width, unsigned int height,
			       VASurfaceID *surfaces, unsigned int num_surfaces,
			       VASurfaceAttrib *attrib_list,
			       unsigned int num_attribs);
VAStatus v4l2r_DestroySurfaces(VADriverContextP va_ctx, VASurfaceID *surface_list,
			       int num_surfaces);
VAStatus v4l2r_SyncSurface(VADriverContextP va_ctx, VASurfaceID render_target);
VAStatus v4l2r_QuerySurfaceStatus(VADriverContextP va_ctx,
				  VASurfaceID render_target,
				  VASurfaceStatus *status);
VAStatus v4l2r_QuerySurfaceAttributes(VADriverContextP va_ctx, VAConfigID config,
				      VASurfaceAttrib *attrib_list,
				      unsigned int *num_attribs);
VAStatus v4l2r_ExportSurfaceHandle(VADriverContextP va_ctx, VASurfaceID surface_id,
				   uint32_t mem_type, uint32_t flags,
				   void *descriptor);
VAStatus v4l2r_PutSurface(VADriverContextP va_ctx, VASurfaceID surface,
			  void *draw, short srcx, short srcy,
			  unsigned short srcw, unsigned short srch,
			  short destx, short desty,
			  unsigned short destw, unsigned short desth,
			  VARectangle *cliprects, unsigned int number_cliprects,
			  unsigned int flags);
VAStatus v4l2r_LockSurface(VADriverContextP va_ctx, VASurfaceID surface,
			   unsigned int *fourcc, unsigned int *luma_stride,
			   unsigned int *chroma_u_stride,
			   unsigned int *chroma_v_stride,
			   unsigned int *luma_offset,
			   unsigned int *chroma_u_offset,
			   unsigned int *chroma_v_offset,
			   unsigned int *buffer_name, void **buffer);
VAStatus v4l2r_UnlockSurface(VADriverContextP va_ctx, VASurfaceID surface);

VAStatus v4l2r_CreateContext(VADriverContextP va_ctx, VAConfigID config_id,
			     int picture_width, int picture_height, int flag,
			     VASurfaceID *render_targets, int num_render_targets,
			     VAContextID *context_id);
VAStatus v4l2r_DestroyContext(VADriverContextP va_ctx, VAContextID context_id);

VAStatus v4l2r_CreateBuffer(VADriverContextP va_ctx, VAContextID context_id,
			    VABufferType type, unsigned int size,
			    unsigned int num_elements, void *data,
			    VABufferID *buf_id);
VAStatus v4l2r_BufferSetNumElements(VADriverContextP va_ctx, VABufferID buf_id,
				    unsigned int num_elements);
VAStatus v4l2r_MapBuffer(VADriverContextP va_ctx, VABufferID buf_id,
			 void **pbuf);
VAStatus v4l2r_UnmapBuffer(VADriverContextP va_ctx, VABufferID buf_id);
VAStatus v4l2r_DestroyBuffer(VADriverContextP va_ctx, VABufferID buf_id);
VAStatus v4l2r_BufferInfo(VADriverContextP va_ctx, VABufferID buf_id,
			  VABufferType *type, unsigned int *size,
			  unsigned int *num_elements);
VAStatus v4l2r_AcquireBufferHandle(VADriverContextP va_ctx, VABufferID buf_id,
				   VABufferInfo *buf_info);
VAStatus v4l2r_ReleaseBufferHandle(VADriverContextP va_ctx, VABufferID buf_id);

VAStatus v4l2r_BeginPicture(VADriverContextP va_ctx, VAContextID context_id,
			    VASurfaceID render_target);
VAStatus v4l2r_RenderPicture(VADriverContextP va_ctx, VAContextID context_id,
			     VABufferID *buffers, int num_buffers);
VAStatus v4l2r_EndPicture(VADriverContextP va_ctx, VAContextID context_id);

VAStatus v4l2r_QueryImageFormats(VADriverContextP va_ctx, VAImageFormat *formats,
				 int *num_formats);
VAStatus v4l2r_CreateImage(VADriverContextP va_ctx, VAImageFormat *format,
			   int width, int height, VAImage *image);
VAStatus v4l2r_DeriveImage(VADriverContextP va_ctx, VASurfaceID surface_id,
			   VAImage *image);
VAStatus v4l2r_DestroyImage(VADriverContextP va_ctx, VAImageID image_id);
VAStatus v4l2r_SetImagePalette(VADriverContextP va_ctx, VAImageID image,
			       unsigned char *palette);
VAStatus v4l2r_GetImage(VADriverContextP va_ctx, VASurfaceID surface_id,
			int x, int y, unsigned int width, unsigned int height,
			VAImageID image_id);
VAStatus v4l2r_PutImage(VADriverContextP va_ctx, VASurfaceID surface,
			VAImageID image, int src_x, int src_y,
			unsigned int src_width, unsigned int src_height,
			int dest_x, int dest_y, unsigned int dest_width,
			unsigned int dest_height);

VAStatus v4l2r_QuerySubpictureFormats(VADriverContextP va_ctx,
				      VAImageFormat *format_list,
				      unsigned int *flags,
				      unsigned int *num_formats);
VAStatus v4l2r_CreateSubpicture(VADriverContextP va_ctx, VAImageID image,
				VASubpictureID *subpicture);
VAStatus v4l2r_DestroySubpicture(VADriverContextP va_ctx,
				 VASubpictureID subpicture);
VAStatus v4l2r_SetSubpictureImage(VADriverContextP va_ctx,
				  VASubpictureID subpicture, VAImageID image);
VAStatus v4l2r_SetSubpictureChromakey(VADriverContextP va_ctx,
				      VASubpictureID subpicture,
				      unsigned int chromakey_min,
				      unsigned int chromakey_max,
				      unsigned int chromakey_mask);
VAStatus v4l2r_SetSubpictureGlobalAlpha(VADriverContextP va_ctx,
					VASubpictureID subpicture,
					float global_alpha);
VAStatus v4l2r_AssociateSubpicture(VADriverContextP va_ctx,
				   VASubpictureID subpicture,
				   VASurfaceID *target_surfaces,
				   int num_surfaces, short src_x, short src_y,
				   unsigned short src_width,
				   unsigned short src_height, short dest_x,
				   short dest_y, unsigned short dest_width,
				   unsigned short dest_height,
				   unsigned int flags);
VAStatus v4l2r_DeassociateSubpicture(VADriverContextP va_ctx,
				     VASubpictureID subpicture,
				     VASurfaceID *target_surfaces,
				     int num_surfaces);

VAStatus v4l2r_QueryDisplayAttributes(VADriverContextP va_ctx,
				      VADisplayAttribute *attr_list,
				      int *num_attributes);
VAStatus v4l2r_GetDisplayAttributes(VADriverContextP va_ctx,
				    VADisplayAttribute *attr_list,
				    int num_attributes);
VAStatus v4l2r_SetDisplayAttributes(VADriverContextP va_ctx,
				    VADisplayAttribute *attr_list,
				    int num_attributes);

#endif /* V4L2_REQUEST_H */
