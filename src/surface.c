/*
 * Surfaces: creation, synchronization and zero-copy dma-buf export.
 *
 * A surface is a thin handle; the backing CAPTURE buffer is allocated by
 * the context the surface is first decoded with. Export hands out the
 * V4L2 buffer planes as DRM PRIME file descriptors without any copy.
 *
 * Copyright (C) 2026 Ondrej Jirman <megi@xff.cz>
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <linux/videodev2.h>

#include <drm_fourcc.h>
#include <va/va_drmcommon.h>

#include "v4l2_request.h"
#include "codec/codec.h"

static void surface_unmap(struct v4l2r_surface *surface)
{
	surface->info = NULL;
	surface->height = 0;
	surface->width = 0;
	surface->pitch = 0;

	for (unsigned int i = 0; i < VIDEO_MAX_PLANES; i++) {
		if (surface->map[i])
			munmap(surface->map[i], surface->plane_size[i]);
		if (surface->dmabuf_fd[i] >= 0)
			close(surface->dmabuf_fd[i]);
	}

	surface->nb_planes = 0;
	memset(surface->dmabuf_fd, -1,
	       VIDEO_MAX_PLANES * sizeof(*surface->dmabuf_fd));
	memset(surface->map, 0,
	       VIDEO_MAX_PLANES * sizeof(*surface->map));
	memset(surface->plane_size, 0,
	       VIDEO_MAX_PLANES * sizeof(*surface->plane_size));
}

VAStatus v4l2r_CreateSurfaces2(VADriverContextP va_ctx, unsigned int format,
			       unsigned int width, unsigned int height,
			       VASurfaceID *surfaces, unsigned int num_surfaces,
			       VASurfaceAttrib *attrib_list,
			       unsigned int num_attribs)
{
	struct v4l2r_driver *drv = v4l2r_driver(va_ctx);
	uint32_t fourcc = 0;
	unsigned int i;

	for (i = 0; i < num_attribs; i++) {
		if (attrib_list[i].type == VASurfaceAttribPixelFormat &&
		    (attrib_list[i].flags & VA_SURFACE_ATTRIB_SETTABLE)) {
			fourcc = attrib_list[i].value.value.i;
			if (fourcc != VA_FOURCC_NV12 && fourcc != VA_FOURCC_P010)
				return VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT;
		}
	}

	for (i = 0; i < num_surfaces; i++) {
		struct v4l2r_surface *surface;
		VASurfaceID id;

		pthread_mutex_lock(&drv->mutex);
		id = v4l2r_handles_alloc(&drv->surfaces, sizeof(*surface));
		surface = V4L2R_SURFACE(drv, id);
		pthread_mutex_unlock(&drv->mutex);
		if (!surface)
			goto fail;

		surface->id = id;
		surface->va.width = width;
		surface->va.height = height;
		surface->va.rt_format = format;
		surface->va.fourcc = fourcc;
		surface->capture_index = -1;
		surface->status = VASurfaceReady;

		memset(surface->dmabuf_fd, -1,
		       VIDEO_MAX_PLANES * sizeof(*surface->dmabuf_fd));
		surface_unmap(surface);

		surfaces[i] = id;
	}

	return VA_STATUS_SUCCESS;

fail:
	pthread_mutex_lock(&drv->mutex);
	while (i--)
		v4l2r_handles_free(&drv->surfaces, surfaces[i]);
	pthread_mutex_unlock(&drv->mutex);
	return VA_STATUS_ERROR_ALLOCATION_FAILED;
}

VAStatus v4l2r_CreateSurfaces(VADriverContextP va_ctx, int width, int height,
			      int format, int num_surfaces,
			      VASurfaceID *surfaces)
{
	return v4l2r_CreateSurfaces2(va_ctx, format, width, height, surfaces,
				     num_surfaces, NULL, 0);
}

VAStatus v4l2r_DestroySurfaces(VADriverContextP va_ctx, VASurfaceID *surface_list,
			       int num_surfaces)
{
	struct v4l2r_driver *drv = v4l2r_driver(va_ctx);

	for (int i = 0; i < num_surfaces; i++) {
		struct v4l2r_surface *surface;

		surface = V4L2R_SURFACE_GET(drv, surface_list[i]);
		if (!surface)
			return VA_STATUS_ERROR_INVALID_SURFACE;

		v4l2r_surface_ready(surface);
		if (surface->ctx && surface->capture_index >= 0)
			v4l2r_context_release_capture(surface->ctx,
						      surface->capture_index);

		surface_unmap(surface);

		pthread_mutex_lock(&drv->mutex);
		v4l2r_handles_free(&drv->surfaces, surface_list[i]);
		pthread_mutex_unlock(&drv->mutex);
	}

	return VA_STATUS_SUCCESS;
}

VAStatus v4l2r_SyncSurface(VADriverContextP va_ctx, VASurfaceID render_target)
{
	struct v4l2r_driver *drv = v4l2r_driver(va_ctx);
	struct v4l2r_surface *surface;

	surface = V4L2R_SURFACE_GET(drv, render_target);
	if (!surface)
		return VA_STATUS_ERROR_INVALID_SURFACE;

	return v4l2r_surface_ready(surface);
}

VAStatus v4l2r_QuerySurfaceStatus(VADriverContextP va_ctx,
				  VASurfaceID render_target,
				  VASurfaceStatus *status)
{
	struct v4l2r_driver *drv = v4l2r_driver(va_ctx);
	struct v4l2r_surface *surface;

	surface = V4L2R_SURFACE_GET(drv, render_target);
	if (!surface)
		return VA_STATUS_ERROR_INVALID_SURFACE;

	/* A frame held back for reordering reports rendering until submitted. */
	v4l2r_flush_surface(surface);

	if (surface->ctx && surface->capture_index >= 0 &&
	    surface->status == VASurfaceRendering)
		v4l2r_reap_capture(surface->ctx);

	*status = surface->status;

	return VA_STATUS_SUCCESS;
}

VAStatus v4l2r_QuerySurfaceAttributes(VADriverContextP va_ctx, VAConfigID config,
				      VASurfaceAttrib *attrib_list,
				      unsigned int *num_attribs)
{
	struct v4l2r_driver *drv = v4l2r_driver(va_ctx);
	struct v4l2r_config *cfg;
	unsigned int i = 0;

	cfg = V4L2R_CONFIG_GET(drv, config);
	if (!cfg)
		return VA_STATUS_ERROR_INVALID_CONFIG;

	if (!attrib_list) {
		*num_attribs = 8;
		return VA_STATUS_SUCCESS;
	}

	if (*num_attribs < 8)
		return VA_STATUS_ERROR_MAX_NUM_EXCEEDED;

	attrib_list[i].type = VASurfaceAttribPixelFormat;
	attrib_list[i].flags = VA_SURFACE_ATTRIB_GETTABLE | VA_SURFACE_ATTRIB_SETTABLE;
	attrib_list[i].value.type = VAGenericValueTypeInteger;
	attrib_list[i].value.value.i = VA_FOURCC_NV12;
	i++;

	attrib_list[i].type = VASurfaceAttribPixelFormat;
	attrib_list[i].flags = VA_SURFACE_ATTRIB_GETTABLE | VA_SURFACE_ATTRIB_SETTABLE;
	attrib_list[i].value.type = VAGenericValueTypeInteger;
	attrib_list[i].value.value.i = VA_FOURCC_P010;
	i++;

	attrib_list[i].type = VASurfaceAttribMemoryType;
	attrib_list[i].flags = VA_SURFACE_ATTRIB_GETTABLE | VA_SURFACE_ATTRIB_SETTABLE;
	attrib_list[i].value.type = VAGenericValueTypeInteger;
	attrib_list[i].value.value.i = VA_SURFACE_ATTRIB_MEM_TYPE_VA |
				       VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2;
	i++;

	attrib_list[i].type = VASurfaceAttribMinWidth;
	attrib_list[i].flags = VA_SURFACE_ATTRIB_GETTABLE;
	attrib_list[i].value.type = VAGenericValueTypeInteger;
	attrib_list[i].value.value.i = 1;
	i++;

	attrib_list[i].type = VASurfaceAttribMinHeight;
	attrib_list[i].flags = VA_SURFACE_ATTRIB_GETTABLE;
	attrib_list[i].value.type = VAGenericValueTypeInteger;
	attrib_list[i].value.value.i = 1;
	i++;

	attrib_list[i].type = VASurfaceAttribMaxWidth;
	attrib_list[i].flags = VA_SURFACE_ATTRIB_GETTABLE;
	attrib_list[i].value.type = VAGenericValueTypeInteger;
	attrib_list[i].value.value.i = 65536;
	i++;

	attrib_list[i].type = VASurfaceAttribMaxHeight;
	attrib_list[i].flags = VA_SURFACE_ATTRIB_GETTABLE;
	attrib_list[i].value.type = VAGenericValueTypeInteger;
	attrib_list[i].value.value.i = 65536;
	i++;

	*num_attribs = i;
	return VA_STATUS_SUCCESS;
}

/*
 * Make a surface's pixels available for reading: submit a decode still held
 * back for reordering, wait for the decode itself, then for any pending
 * format conversion into the backing.
 */
VAStatus v4l2r_surface_ready(struct v4l2r_surface *surface)
{
	VAStatus status;

	v4l2r_flush_surface(surface);

	if (!surface->ctx || surface->capture_index < 0)
		return VA_STATUS_SUCCESS;

	uint64_t t0 = v4l2r_now_ns();
	status = v4l2r_sync_capture(surface->ctx, surface->capture_index);
	v4l2r_trace("export sync (surface 0x%08x buf #%d): %.2f ms\n",
		    surface->id, surface->capture_index,
		    (v4l2r_now_ns() - t0) / 1e6);

	return status;
}

static int try_format(int fd, struct v4l2_format *out,
		      struct v4l2_format *cap)
{
	struct v4l2_format o = *out, c = *cap;
	uint32_t width = c.fmt.pix_mp.width;
	uint32_t height = c.fmt.pix_mp.height;
	uint32_t pixelformat = c.fmt.pix_mp.pixelformat;

	if (ioctl(fd, VIDIOC_S_FMT, &o) < 0)
		return 1;

	if (ioctl(fd, VIDIOC_S_FMT, &c) < 0)
		return 1;

	if (c.fmt.pix_mp.pixelformat != pixelformat ||
	    c.fmt.pix_mp.width < width ||
	    c.fmt.pix_mp.height < height)
		return 1;

	*out = o;
	*cap = c;

	return 0;
}

static int surface_alloc(struct v4l2r_decoder *dec,
			 struct v4l2r_surface *surface,
			 struct v4l2_format *ctx_cap,
			 uint32_t pixfmt)
{
	struct v4l2_capability capability = {0};
	struct v4l2_create_buffers buffers;
	struct v4l2_plane planes[VIDEO_MAX_PLANES] = {0};
	struct v4l2_buffer buffer = {0};
	struct v4l2_format out = {
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.fmt.pix_mp = {
			.num_planes = 1,
			.width = surface->va.width,
			.height = surface->va.height,
			.pixelformat = dec->pixelformats[0],
		}
	};
	struct v4l2_format cap = {
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.fmt.pix_mp = {
			.num_planes = 1,
			.width = surface->va.width,
			.height = surface->va.height,
			.pixelformat = pixfmt,
		}
	};
	unsigned int capabilities;
	bool mplane;
	int fd, ret = 1;

	fd = open(dec->video_path, O_RDWR | O_CLOEXEC);
	if (fd < 0)
		return fd;

	if (ioctl(fd, VIDIOC_QUERYCAP, &capability) < 0)
		goto done;

	capabilities = (capability.capabilities & V4L2_CAP_DEVICE_CAPS) ?
		       capability.device_caps : capability.capabilities;
	mplane = capabilities & V4L2_CAP_VIDEO_M2M_MPLANE;

	if (!mplane)
		goto done;

	if (!ctx_cap) {
		if (try_format(fd, &out, &cap))
			goto done;

		/* make sure we have enough space */
		cap.fmt.pix_mp.plane_fmt[0].sizeimage +=
			cap.fmt.pix_mp.plane_fmt[0].sizeimage / 2;
	} else {
		cap = *ctx_cap;
	}

	surface->width = cap.fmt.pix_mp.width;
	surface->height = cap.fmt.pix_mp.height;
	surface->pitch = cap.fmt.pix_mp.plane_fmt[0].bytesperline;
	surface->nb_planes = cap.fmt.pix_mp.num_planes;

	buffers = (struct v4l2_create_buffers) {
		.count = 1,
		.memory = V4L2_MEMORY_MMAP,
		.format = cap,
	};

	if (ioctl(fd, VIDIOC_CREATE_BUFS, &buffers) < 0)
		goto done;

	buffer.type = cap.type;
	buffer.index = buffers.index;
	buffer.length = VIDEO_MAX_PLANES;
	buffer.m.planes = planes;

	if (ioctl(fd, VIDIOC_QUERYBUF, &buffer) < 0)
		goto done;

	surface->nb_planes = buffer.length;
	for (unsigned int i = 0; i < buffer.length; i++)
		surface->plane_size[i] = buffer.m.planes[i].length;

	for (unsigned int i = 0; i < surface->nb_planes; i++) {
		struct v4l2_exportbuffer exportbuffer = {
			.type = cap.type,
			.index = buffer.index,
			.plane = i,
			.flags = O_RDWR | O_CLOEXEC,
		};

		if (ioctl(fd, VIDIOC_EXPBUF, &exportbuffer) < 0)
			goto done;

		surface->dmabuf_fd[i] = exportbuffer.fd;
	}

	ret = 0;
	surface->pixelformat = pixfmt;

	{
		unsigned long got = 0;
		for (unsigned int i = 0; i < surface->nb_planes; i++)
			got += surface->plane_size[i];

		v4l2r_trace("allocated SURFACE 0x%x%s %.4s %.4s (%zu bytes, ~%llu MiB)\n",
			    surface->id, ctx_cap ? "" : " (no ctx)",
			    (const char *)&(uint32_t){pixfmt},
			    (const char *)&(uint32_t){out.fmt.pix_mp.pixelformat},
			    got, (unsigned long long)((uint64_t)got >> 20));
	}

done:
	if (ret)
		surface_unmap(surface);

	close(fd);
	return ret;
}

static int surface_allocate(struct v4l2r_driver *drv,
			    struct v4l2r_surface *surface)
{
	struct v4l2r_decoder *dec;
	uint32_t pixfmt;

	if (surface->ctx && surface->capture_index >= 0) {
		struct v4l2r_context *ctx = surface->ctx;
		struct v4l2_format *cap = &ctx->capture_format;

		dec = (struct v4l2r_decoder *)surface->ctx->decoder;
		if (surface_alloc(dec, surface, cap,
				  cap->fmt.pix_mp.pixelformat))
			return 1;
	} else {
		if (surface->va.fourcc == VA_FOURCC_P010)
			pixfmt = V4L2_PIX_FMT_P010;
		else
			pixfmt = V4L2_PIX_FMT_NV12;

		for (unsigned int i = 0; i < drv->nb_decoders; i++) {
			dec = &drv->decoders[i];
			if (surface_alloc(dec, surface, NULL, pixfmt))
				continue;
			else
				goto done;
		}

		return 1;
	}

done:
	surface->info = v4l2r_format_by_pixelformat(surface->pixelformat);
	if (!surface->info)
		return 1;

	return 0;
}

VAStatus v4l2r_surface_prepare(struct v4l2r_driver *drv,
			    struct v4l2r_surface *surface, bool need_maps)
{
	if (!surface->nb_planes) {
		if (surface_allocate(drv, surface))
			return VA_STATUS_ERROR_OPERATION_FAILED;
	}

	if (!need_maps)
		return VA_STATUS_SUCCESS;

	for (unsigned int i = 0; i < surface->nb_planes; i++) {
		if (surface->map[i])
			continue;
		void *addr = mmap(NULL, surface->plane_size[i],
				  PROT_READ | PROT_WRITE, MAP_SHARED,
				  surface->dmabuf_fd[i], 0);
		if (addr == MAP_FAILED)
			return VA_STATUS_ERROR_OPERATION_FAILED;
		surface->map[i] = addr;
	}

	return VA_STATUS_SUCCESS;
}

/*
 * Describe the surface memory layout for DRM consumers, following the
 * layout rules of the FFmpeg v4l2-request hwcontext.
 */
static VAStatus fill_prime_descriptor(const struct v4l2r_surface *view,
				      uint32_t flags,
				      VADRMPRIMESurfaceDescriptor *desc)
{
	const struct v4l2r_format_info *info = view->info;
	uint32_t width = view->width;
	uint32_t height = view->height;
	uint32_t pitch = view->pitch;
	uint32_t pixelformat __attribute__((unused)) = view->pixelformat;
	uint64_t modifier;
	uint32_t offset1, pitch1;
	bool two_planes;

	modifier = info->drm_modifier;

	memset(desc, 0, sizeof(*desc));
	desc->fourcc = info->drm_format;
	desc->width = width;
	desc->height = height;
	desc->num_objects = view->nb_planes;

	for (unsigned int i = 0; i < view->nb_planes; i++) {
		int fd = dup(view->dmabuf_fd[i]);
		if (fd < 0) {
			for (unsigned int j = 0; j < i; j++)
				close(desc->objects[j].fd);
			return VA_STATUS_ERROR_OPERATION_FAILED;
		}
		desc->objects[i].fd = fd;
		desc->objects[i].size = view->plane_size[i];
		desc->objects[i].drm_format_modifier = modifier;
	}

	/* AFBC formats expose a single plane, everything else has separate
	 * luma and chroma planes. */
	two_planes = (modifier >> 56) != DRM_FORMAT_MOD_VENDOR_ARM;

	offset1 = pitch * height;
	pitch1 = pitch;

#if defined(V4L2_PIX_FMT_NV12MT_COL128) && defined(V4L2_PIX_FMT_NV12MT_10_COL128)
	if (pixelformat == V4L2_PIX_FMT_NV12MT_COL128 ||
	    pixelformat == V4L2_PIX_FMT_NV12MT_10_COL128) {
		/* Raspberry Pi SAND tiling: pitch carries the column height. */
		pitch = height;
		pitch1 = pitch / 2;
		offset1 = 0;
	}
#endif
#if defined(V4L2_PIX_FMT_NV12_COL128) && defined(V4L2_PIX_FMT_NV12_10_COL128)
	if (pixelformat == V4L2_PIX_FMT_NV12_COL128 ||
	    pixelformat == V4L2_PIX_FMT_NV12_10_COL128) {
		uint64_t sand_modifier =
			DRM_FORMAT_MOD_BROADCOM_SAND128_COL_HEIGHT(pitch);
		for (unsigned int i = 0; i < desc->num_objects; i++)
			desc->objects[i].drm_format_modifier = sand_modifier;
		offset1 = 128 * height;
		pitch = width;
		if (pixelformat == V4L2_PIX_FMT_NV12_10_COL128)
			pitch *= 2;
		pitch1 = pitch;
	}
#endif

	if ((flags & VA_EXPORT_SURFACE_SEPARATE_LAYERS) && two_planes &&
	    view->nb_planes == 1 &&
	    (info->drm_format == DRM_FORMAT_NV12 ||
	     info->drm_format == DRM_FORMAT_P010) &&
	    modifier == DRM_FORMAT_MOD_LINEAR) {
		bool p010 = info->drm_format == DRM_FORMAT_P010;

		desc->num_layers = 2;
		desc->layers[0].drm_format = p010 ? DRM_FORMAT_R16 : DRM_FORMAT_R8;
		desc->layers[0].num_planes = 1;
		desc->layers[0].object_index[0] = 0;
		desc->layers[0].offset[0] = 0;
		desc->layers[0].pitch[0] = pitch;
		desc->layers[1].drm_format = p010 ? DRM_FORMAT_GR1616 : DRM_FORMAT_GR88;
		desc->layers[1].num_planes = 1;
		desc->layers[1].object_index[0] = 0;
		desc->layers[1].offset[0] = offset1;
		desc->layers[1].pitch[0] = pitch1;

		return VA_STATUS_SUCCESS;
	}

	desc->num_layers = 1;
	desc->layers[0].drm_format = info->drm_format;
	desc->layers[0].num_planes = two_planes ? 2 : 1;
	desc->layers[0].object_index[0] = 0;
	desc->layers[0].offset[0] = 0;
	desc->layers[0].pitch[0] = pitch;
	if (two_planes) {
		desc->layers[0].object_index[1] = view->nb_planes > 1 ? 1 : 0;
		desc->layers[0].offset[1] = view->nb_planes > 1 ? 0 : offset1;
		desc->layers[0].pitch[1] = pitch1;
	}

	return VA_STATUS_SUCCESS;
}

VAStatus v4l2r_ExportSurfaceHandle(VADriverContextP va_ctx, VASurfaceID surface_id,
				   uint32_t mem_type, uint32_t flags,
				   void *descriptor)
{
	struct v4l2r_driver *drv = v4l2r_driver(va_ctx);
	struct v4l2r_surface *surface;
	VAStatus status;

	if (mem_type != VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2)
		return VA_STATUS_ERROR_UNSUPPORTED_MEMORY_TYPE;

	surface = V4L2R_SURFACE_GET(drv, surface_id);
	if (!surface)
		return VA_STATUS_ERROR_INVALID_SURFACE;

	/* vaExportSurfaceHandle() does no synchronization, and the exported
	 * dma-buf carries no fence. Wait for the decode (and any format
	 * conversion) to finish so a consumer that does not vaSyncSurface()
	 * itself (e.g. --vo=dmabuf-wayland) cannot read a partially decoded
	 * frame. No-op for unbound or already-idle surfaces. */
	v4l2r_surface_ready(surface);

	status = v4l2r_surface_prepare(drv, surface, false);
	if (status != VA_STATUS_SUCCESS)
		return status;

	return fill_prime_descriptor(surface, flags, descriptor);
}

/* --- unsupported display path --- */

VAStatus v4l2r_PutSurface(VADriverContextP va_ctx, VASurfaceID surface,
			  void *draw, short srcx, short srcy,
			  unsigned short srcw, unsigned short srch,
			  short destx, short desty,
			  unsigned short destw, unsigned short desth,
			  VARectangle *cliprects, unsigned int number_cliprects,
			  unsigned int flags)
{
	(void)va_ctx; (void)surface; (void)draw; (void)srcx; (void)srcy;
	(void)srcw; (void)srch; (void)destx; (void)desty; (void)destw;
	(void)desth; (void)cliprects; (void)number_cliprects; (void)flags;
	return VA_STATUS_ERROR_UNIMPLEMENTED;
}

VAStatus v4l2r_LockSurface(VADriverContextP va_ctx, VASurfaceID surface,
			   unsigned int *fourcc, unsigned int *luma_stride,
			   unsigned int *chroma_u_stride,
			   unsigned int *chroma_v_stride,
			   unsigned int *luma_offset,
			   unsigned int *chroma_u_offset,
			   unsigned int *chroma_v_offset,
			   unsigned int *buffer_name, void **buffer)
{
	(void)va_ctx; (void)surface; (void)fourcc; (void)luma_stride;
	(void)chroma_u_stride; (void)chroma_v_stride; (void)luma_offset;
	(void)chroma_u_offset; (void)chroma_v_offset; (void)buffer_name;
	(void)buffer;
	return VA_STATUS_ERROR_UNIMPLEMENTED;
}

VAStatus v4l2r_UnlockSurface(VADriverContextP va_ctx, VASurfaceID surface)
{
	(void)va_ctx; (void)surface;
	return VA_STATUS_ERROR_UNIMPLEMENTED;
}
