/*
 * Driver entry point, device enumeration and VAConfig handling.
 *
 * Copyright (C) 2026 Ondrej Jirman <megi@xff.cz>
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <fcntl.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <unistd.h>

#include <linux/media.h>
#include <linux/videodev2.h>
#include <linux/v4l2-controls.h>

#include <drm_fourcc.h>

#include "v4l2_request.h"

/* Needs the va.h types pulled in by v4l2_request.h. */
#include <va/va_backend_vpp.h>

void v4l2r_log(const char *fmt, ...)
{
	va_list args;

	fprintf(stderr, "libva-v4l2request: ");
	va_start(args, fmt);
	vfprintf(stderr, fmt, args);
	va_end(args);
}

/* --- codec and format tables --- */

static const struct v4l2r_codec *const codecs[] = {
#if HAVE_V4L2_CTRL_MPEG2
	&v4l2r_codec_mpeg2,
#endif
#if HAVE_V4L2_CTRL_H264
	&v4l2r_codec_h264,
#endif
#if HAVE_V4L2_CTRL_HEVC
	&v4l2r_codec_hevc,
#endif
#if HAVE_V4L2_CTRL_VP8
	&v4l2r_codec_vp8,
#endif
#if HAVE_V4L2_CTRL_VP9
	&v4l2r_codec_vp9,
#endif
#if HAVE_V4L2_CTRL_AV1
	&v4l2r_codec_av1,
#endif
};

const struct v4l2r_codec **v4l2r_codec_list(unsigned int *count)
{
	*count = sizeof(codecs) / sizeof(codecs[0]);
	return (const struct v4l2r_codec **)codecs;
}

const struct v4l2r_codec *v4l2r_codec_for_profile(VAProfile profile)
{
	for (unsigned int i = 0; i < sizeof(codecs) / sizeof(codecs[0]); i++) {
		for (unsigned int j = 0; j < codecs[i]->nb_profiles; j++) {
			if (codecs[i]->profiles[j] == profile)
				return codecs[i];
		}
	}

	return NULL;
}

unsigned int v4l2r_profile_bit_depth(VAProfile profile)
{
	switch (profile) {
	case VAProfileHEVCMain10:
	case VAProfileVP9Profile2:
		return 10;
	default:
		return 8;
	}
}

unsigned int v4l2r_profile_rt_format(VAProfile profile)
{
	switch (profile) {
	case VAProfileHEVCMain10:
	case VAProfileVP9Profile2:
		return VA_RT_FORMAT_YUV420_10;
	case VAProfileAV1Profile0:
		return VA_RT_FORMAT_YUV420 | VA_RT_FORMAT_YUV420_10;
	default:
		return VA_RT_FORMAT_YUV420;
	}
}

static const struct v4l2r_format_info format_infos[] = {
	{ V4L2_PIX_FMT_NV12, DRM_FORMAT_NV12, DRM_FORMAT_MOD_LINEAR,
	  VA_FOURCC_NV12, VA_RT_FORMAT_YUV420, 8, true },
#if defined(V4L2_PIX_FMT_NV12_32L32)
	{ V4L2_PIX_FMT_NV12_32L32, DRM_FORMAT_NV12, DRM_FORMAT_MOD_ALLWINNER_TILED,
	  0, VA_RT_FORMAT_YUV420, 8, false },
#endif
#if defined(V4L2_PIX_FMT_NV15) && defined(DRM_FORMAT_NV15)
	{ V4L2_PIX_FMT_NV15, DRM_FORMAT_NV15, DRM_FORMAT_MOD_LINEAR,
	  0, VA_RT_FORMAT_YUV420_10, 10, false },
#endif
	{ V4L2_PIX_FMT_NV16, DRM_FORMAT_NV16, DRM_FORMAT_MOD_LINEAR,
	  0, VA_RT_FORMAT_YUV422, 8, true },
#if defined(V4L2_PIX_FMT_NV20) && defined(DRM_FORMAT_NV20)
	{ V4L2_PIX_FMT_NV20, DRM_FORMAT_NV20, DRM_FORMAT_MOD_LINEAR,
	  0, VA_RT_FORMAT_YUV422_10, 10, false },
#endif
#if defined(V4L2_PIX_FMT_P010) && defined(DRM_FORMAT_P010)
	{ V4L2_PIX_FMT_P010, DRM_FORMAT_P010, DRM_FORMAT_MOD_LINEAR,
	  VA_FOURCC_P010, VA_RT_FORMAT_YUV420_10, 10, true },
#endif
#if defined(V4L2_PIX_FMT_NV12MT_COL128) && defined(V4L2_PIX_FMT_NV12MT_10_COL128)
	{ V4L2_PIX_FMT_NV12MT_COL128, DRM_FORMAT_NV12, DRM_FORMAT_MOD_BROADCOM_SAND128,
	  0, VA_RT_FORMAT_YUV420, 8, false },
#if defined(DRM_FORMAT_P030)
	{ V4L2_PIX_FMT_NV12MT_10_COL128, DRM_FORMAT_P030, DRM_FORMAT_MOD_BROADCOM_SAND128,
	  0, VA_RT_FORMAT_YUV420_10, 10, false },
#endif
#endif
#if defined(V4L2_PIX_FMT_NV12_COL128) && defined(V4L2_PIX_FMT_NV12_10_COL128)
	{ V4L2_PIX_FMT_NV12_COL128, DRM_FORMAT_NV12, DRM_FORMAT_MOD_BROADCOM_SAND128,
	  0, VA_RT_FORMAT_YUV420, 8, false },
#if defined(DRM_FORMAT_P030)
	{ V4L2_PIX_FMT_NV12_10_COL128, DRM_FORMAT_P030, DRM_FORMAT_MOD_BROADCOM_SAND128,
	  0, VA_RT_FORMAT_YUV420_10, 10, false },
#endif
#endif
#if defined(V4L2_PIX_FMT_YUV420_8_AFBC_16X16_SPLIT)
	{ V4L2_PIX_FMT_YUV420_8_AFBC_16X16_SPLIT, DRM_FORMAT_YUV420_8BIT,
	  DRM_FORMAT_MOD_ARM_AFBC(AFBC_FORMAT_MOD_BLOCK_SIZE_16x16 |
				  AFBC_FORMAT_MOD_SPARSE | AFBC_FORMAT_MOD_SPLIT),
	  0, VA_RT_FORMAT_YUV420, 8, false },
#endif
#if defined(V4L2_PIX_FMT_YUV420_10_AFBC_16X16_SPLIT)
	{ V4L2_PIX_FMT_YUV420_10_AFBC_16X16_SPLIT, DRM_FORMAT_YUV420_10BIT,
	  DRM_FORMAT_MOD_ARM_AFBC(AFBC_FORMAT_MOD_BLOCK_SIZE_16x16 |
				  AFBC_FORMAT_MOD_SPARSE | AFBC_FORMAT_MOD_SPLIT),
	  0, VA_RT_FORMAT_YUV420_10, 10, false },
#endif
};

const struct v4l2r_format_info *v4l2r_format_by_pixelformat(uint32_t pixelformat)
{
	for (unsigned int i = 0; i < sizeof(format_infos) / sizeof(format_infos[0]); i++) {
		if (format_infos[i].pixelformat == pixelformat)
			return &format_infos[i];
	}

	return NULL;
}

const struct v4l2r_format_info *v4l2r_capture_format_info(struct v4l2r_context *ctx)
{
	if (!ctx->streaming)
		return NULL;

	return v4l2r_format_by_pixelformat(
		v4l2r_format_pixelformat(&ctx->capture_format));
}

/* --- device enumeration --- */

/* Resolve a character device number to its /dev node via sysfs, avoiding a
 * hard libudev dependency. */
static int devnode_from_devnum(uint32_t major, uint32_t minor, char *path,
			       size_t path_size)
{
	char sys_path[64], line[300];
	FILE *file;
	int ret = -1;

	snprintf(sys_path, sizeof(sys_path), "/sys/dev/char/%u:%u/uevent",
		 major, minor);
	file = fopen(sys_path, "r");
	if (!file)
		return -1;

	while (fgets(line, sizeof(line), file)) {
		if (!strncmp(line, "DEVNAME=", 8)) {
			char *name = line + 8;
			name[strcspn(name, "\n")] = '\0';
			if (strlen(name) + 6 > path_size)
				break;
			snprintf(path, path_size, "/dev/%.250s", name);
			ret = 0;
			break;
		}
	}

	fclose(file);
	return ret;
}

/*
 * Ask the device whether it can decode 10-bit HEVC. There is no V4L2 query for
 * this and the raw CAPTURE formats do not distinguish 8- from 10-bit, so mirror
 * what the decode path does: select the HEVC coded format on the OUTPUT queue,
 * then set a 10-bit HEVC SPS with VIDIOC_S_EXT_CTRLS (no request). Devices that
 * cannot do 10-bit reject the bit depth in try_ctrl with EINVAL (e.g. cedrus on
 * the A64); capable devices accept it.
 *
 * Selecting the coded format first matters: drivers dispatch control validation
 * per selected codec, and some (rkvdec) additionally reject an SPS whose picture
 * size exceeds the current OUTPUT format — so the SPS is sized to the dimensions
 * the device just accepted for the coded format.
 */
static bool video_device_probe_hevc_10bit(int fd, uint32_t output_type)
{
#if HAVE_V4L2_CTRL_HEVC
	bool mplane = V4L2_TYPE_IS_MULTIPLANAR(output_type);
	struct v4l2_format format = { .type = output_type };
	struct v4l2_ctrl_hevc_sps sps = {
		.chroma_format_idc = 1,
		.bit_depth_luma_minus8 = 2,
		.bit_depth_chroma_minus8 = 2,
		.log2_max_pic_order_cnt_lsb_minus4 = 4,
		.log2_diff_max_min_luma_coding_block_size = 3,
		.log2_diff_max_min_luma_transform_block_size = 3,
		.sps_max_dec_pic_buffering_minus1 = 4,
	};
	struct v4l2_ext_control control = {
		.id = V4L2_CID_STATELESS_HEVC_SPS,
		.ptr = &sps,
		.size = sizeof(sps),
	};
	struct v4l2_ext_controls controls = {
		.count = 1,
		.controls = &control,
	};

	if (mplane) {
		format.fmt.pix_mp.width = 1920;
		format.fmt.pix_mp.height = 1088;
		format.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_HEVC_SLICE;
		format.fmt.pix_mp.num_planes = 1;
	} else {
		format.fmt.pix.width = 1920;
		format.fmt.pix.height = 1088;
		format.fmt.pix.pixelformat = V4L2_PIX_FMT_HEVC_SLICE;
	}

	if (ioctl(fd, VIDIOC_S_FMT, &format) < 0)
		return false;

	/* Size the SPS to what the device accepted; some drivers reject an SPS
	 * larger than the current OUTPUT format. */
	if (mplane) {
		sps.pic_width_in_luma_samples = format.fmt.pix_mp.width;
		sps.pic_height_in_luma_samples = format.fmt.pix_mp.height;
	} else {
		sps.pic_width_in_luma_samples = format.fmt.pix.width;
		sps.pic_height_in_luma_samples = format.fmt.pix.height;
	}

	return ioctl(fd, VIDIOC_S_EXT_CTRLS, &controls) == 0;
#else
	(void)fd;
	(void)output_type;
	return false;
#endif
}

static bool video_device_is_request_decoder(const char *path,
					    struct v4l2r_decoder *decoder)
{
	struct v4l2_capability capability = {0};
	struct v4l2_create_buffers buffers = {0};
	struct v4l2_fmtdesc fmtdesc = {0};
	unsigned int capabilities;
	uint32_t output_type;
	int fd;

	fd = open(path, O_RDWR | O_NONBLOCK);
	if (fd < 0)
		return false;

	if (ioctl(fd, VIDIOC_QUERYCAP, &capability) < 0)
		goto fail;

	capabilities = (capability.capabilities & V4L2_CAP_DEVICE_CAPS) ?
		       capability.device_caps : capability.capabilities;

	if (!(capabilities & V4L2_CAP_STREAMING))
		goto fail;

	snprintf(decoder->card, sizeof(decoder->card), "%s", capability.card);

	if (capabilities & V4L2_CAP_VIDEO_M2M_MPLANE)
		output_type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	else if (capabilities & V4L2_CAP_VIDEO_M2M)
		output_type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	else
		goto fail;

	buffers.memory = V4L2_MEMORY_MMAP;
	buffers.format.type = output_type;
	if (ioctl(fd, VIDIOC_CREATE_BUFS, &buffers) < 0)
		goto fail;

	if (!(buffers.capabilities & V4L2_BUF_CAP_SUPPORTS_REQUESTS))
		goto fail;

	/* Collect the coded formats this decoder accepts. */
	decoder->nb_pixelformats = 0;
	fmtdesc.type = output_type;
	while (ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) >= 0 &&
	       decoder->nb_pixelformats < V4L2R_MAX_PIXELFORMATS) {
		decoder->pixelformats[decoder->nb_pixelformats++] =
			fmtdesc.pixelformat;
		fmtdesc.index++;
	}

	decoder->hevc_10bit = false;
	for (unsigned int i = 0; i < decoder->nb_pixelformats; i++) {
		if (decoder->pixelformats[i] == V4L2_PIX_FMT_HEVC_SLICE) {
			decoder->hevc_10bit =
				video_device_probe_hevc_10bit(fd, output_type);
			break;
		}
	}

	close(fd);
	return decoder->nb_pixelformats > 0;

fail:
	close(fd);
	return false;
}

static void probe_media_device(struct v4l2r_driver *drv, const char *media_path)
{
	struct media_v2_topology topology = {0};
	struct media_v2_interface *interfaces = NULL;
	int media_fd;

	if (drv->nb_decoders >= V4L2R_MAX_DECODERS)
		return;

	media_fd = open(media_path, O_RDWR);
	if (media_fd < 0)
		return;

	if (ioctl(media_fd, MEDIA_IOC_G_TOPOLOGY, &topology) < 0 ||
	    !topology.num_interfaces)
		goto done;

	interfaces = calloc(topology.num_interfaces, sizeof(*interfaces));
	if (!interfaces)
		goto done;

	topology.ptr_interfaces = (uintptr_t)interfaces;
	if (ioctl(media_fd, MEDIA_IOC_G_TOPOLOGY, &topology) < 0)
		goto done;

	for (unsigned int i = 0; i < topology.num_interfaces; i++) {
		struct v4l2r_decoder *decoder = &drv->decoders[drv->nb_decoders];
		char video_path[256];

		if (interfaces[i].intf_type != MEDIA_INTF_T_V4L_VIDEO)
			continue;

		if (devnode_from_devnum(interfaces[i].devnode.major,
					interfaces[i].devnode.minor,
					video_path, sizeof(video_path)) < 0)
			continue;

		if (!video_device_is_request_decoder(video_path, decoder))
			continue;

		snprintf(decoder->media_path, sizeof(decoder->media_path),
			 "%s", media_path);
		snprintf(decoder->video_path, sizeof(decoder->video_path),
			 "%s", video_path);
		drv->nb_decoders++;

		if (drv->nb_decoders >= V4L2R_MAX_DECODERS)
			break;
	}

done:
	free(interfaces);
	close(media_fd);
}

static void enumerate_decoders(struct v4l2r_driver *drv)
{
	const char *env_media = getenv("LIBVA_V4L2_REQUEST_MEDIA_PATH");

	if (env_media) {
		probe_media_device(drv, env_media);
		return;
	}

	for (int i = 0; i < 64 && drv->nb_decoders < V4L2R_MAX_DECODERS; i++) {
		char media_path[32];

		snprintf(media_path, sizeof(media_path), "/dev/media%d", i);
		if (access(media_path, F_OK) < 0)
			continue;

		probe_media_device(drv, media_path);
	}
}

/*
 * A usable format converter is a plain (non-request) mem2mem device that
 * can read at least one decoder-only CAPTURE format and write NV12, e.g.
 * the Rockchip RGA reading packed 10-bit NV15.
 */
static bool video_device_is_converter(const char *path,
				      struct v4l2r_converter *conv)
{
	struct v4l2_capability capability = {0};
	struct v4l2_fmtdesc fmtdesc = {0};
	unsigned int capabilities;
	bool nv12_capture = false;
	bool src_convertible = false;
	int fd;

	fd = open(path, O_RDWR | O_NONBLOCK);
	if (fd < 0)
		return false;

	if (ioctl(fd, VIDIOC_QUERYCAP, &capability) < 0)
		goto fail;

	capabilities = (capability.capabilities & V4L2_CAP_DEVICE_CAPS) ?
		       capability.device_caps : capability.capabilities;

	if (!(capabilities & V4L2_CAP_STREAMING) ||
	    !(capabilities & V4L2_CAP_VIDEO_M2M_MPLANE))
		goto fail;

	snprintf(conv->card, sizeof(conv->card), "%s", capability.card);

	/* Collect the source formats the converter reads. */
	conv->nb_pixelformats = 0;
	fmtdesc.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	while (ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) >= 0 &&
	       conv->nb_pixelformats < V4L2R_MAX_PIXELFORMATS) {
		const struct v4l2r_format_info *info =
			v4l2r_format_by_pixelformat(fmtdesc.pixelformat);

		conv->pixelformats[conv->nb_pixelformats++] =
			fmtdesc.pixelformat;

		/* Only useful if it reads a format we cannot hand out. */
		if (info && !info->va_fourcc)
			src_convertible = true;

		fmtdesc.index++;
	}

	memset(&fmtdesc, 0, sizeof(fmtdesc));
	fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	while (ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) >= 0) {
		if (fmtdesc.pixelformat == V4L2_PIX_FMT_NV12) {
			nv12_capture = true;
			break;
		}
		fmtdesc.index++;
	}

	close(fd);
	return src_convertible && nv12_capture;

fail:
	close(fd);
	return false;
}

/* Called lazily via v4l2r_converter_available(), with drv->mutex held. */
static void enumerate_converters(struct v4l2r_driver *drv)
{
	if (getenv("V4L2R_NO_CONVERT"))
		return;

	for (int i = 0; i < 64; i++) {
		char video_path[32];
		bool is_decoder = false;

		snprintf(video_path, sizeof(video_path), "/dev/video%d", i);
		if (access(video_path, F_OK) < 0)
			continue;

		for (unsigned int j = 0; j < drv->nb_decoders; j++) {
			if (!strcmp(drv->decoders[j].video_path, video_path)) {
				is_decoder = true;
				break;
			}
		}
		if (is_decoder)
			continue;

		if (!video_device_is_converter(video_path, &drv->converter))
			continue;

		snprintf(drv->converter.video_path,
			 sizeof(drv->converter.video_path), "%s", video_path);
		drv->has_converter = true;
		v4l2r_log("detected format converter %s [%s]\n",
			  video_path, drv->converter.card);
		return;
	}
}

/*
 * Probe for a converter on first use. Scanning the video device nodes
 * costs a few dozen opens/ioctls, so it is deferred until something can
 * actually use the result: plain 8-bit playback and vainfo never trigger
 * it.
 */
bool v4l2r_converter_available(struct v4l2r_driver *drv)
{
	pthread_mutex_lock(&drv->mutex);
	if (!drv->converter_probed) {
		enumerate_converters(drv);
		drv->converter_probed = true;
	}
	pthread_mutex_unlock(&drv->mutex);

	return drv->has_converter;
}

bool v4l2r_converter_supports(struct v4l2r_driver *drv, uint32_t pixelformat)
{
	if (!v4l2r_converter_available(drv))
		return false;

	for (unsigned int i = 0; i < drv->converter.nb_pixelformats; i++) {
		if (drv->converter.pixelformats[i] == pixelformat)
			return true;
	}

	return false;
}

static bool driver_supports_pixelformat(struct v4l2r_driver *drv,
					uint32_t pixelformat)
{
	for (unsigned int i = 0; i < drv->nb_decoders; i++) {
		for (unsigned int j = 0; j < drv->decoders[i].nb_pixelformats; j++) {
			if (drv->decoders[i].pixelformats[j] == pixelformat)
				return true;
		}
	}

	return false;
}

/* Whether some decoder can decode 10-bit HEVC. */
static bool driver_supports_hevc_10bit(struct v4l2r_driver *drv)
{
	for (unsigned int i = 0; i < drv->nb_decoders; i++) {
		if (drv->decoders[i].hevc_10bit)
			return true;
	}

	return false;
}

/*
 * Whether a decode profile is usable: some decoder must accept the codec's
 * coded format, and for 10-bit profiles the hardware must actually support the
 * higher bit depth (advertising Main10 on an 8-bit-only decoder would make
 * clients pick VA-API and then fail every frame at decode time).
 */
static bool driver_supports_profile(struct v4l2r_driver *drv, VAProfile profile)
{
	const struct v4l2r_codec *codec = v4l2r_codec_for_profile(profile);

	if (!codec || !driver_supports_pixelformat(drv, codec->pixelformat))
		return false;

	if (profile == VAProfileHEVCMain10 && !driver_supports_hevc_10bit(drv))
		return false;

	return true;
}

/* --- config --- */

VAStatus v4l2r_QueryConfigProfiles(VADriverContextP va_ctx, VAProfile *profiles,
				   int *num_profiles)
{
	struct v4l2r_driver *drv = v4l2r_driver(va_ctx);
	unsigned int nb_codecs;
	const struct v4l2r_codec **list = v4l2r_codec_list(&nb_codecs);
	int count = 0;

	pthread_mutex_lock(&drv->mutex);

	for (unsigned int i = 0; i < nb_codecs; i++) {
		if (!driver_supports_pixelformat(drv, list[i]->pixelformat))
			continue;

		for (unsigned int j = 0; j < list[i]->nb_profiles; j++) {
			if (!driver_supports_profile(drv, list[i]->profiles[j]))
				continue;
			if (count < V4L2R_MAX_PROFILES)
				profiles[count++] = list[i]->profiles[j];
		}
	}

	/* Video processing. Listed unconditionally so plain decode clients
	 * enumerating profiles never trigger the converter device scan; the
	 * entrypoint query performs the actual probe. */
	if (count < V4L2R_MAX_PROFILES)
		profiles[count++] = VAProfileNone;

	pthread_mutex_unlock(&drv->mutex);

	*num_profiles = count;
	return VA_STATUS_SUCCESS;
}

VAStatus v4l2r_QueryConfigEntrypoints(VADriverContextP va_ctx, VAProfile profile,
				      VAEntrypoint *entrypoints,
				      int *num_entrypoints)
{
	struct v4l2r_driver *drv = v4l2r_driver(va_ctx);
	const struct v4l2r_codec *codec = v4l2r_codec_for_profile(profile);

	if (profile == VAProfileNone) {
		*num_entrypoints = 0;
		if (v4l2r_converter_available(drv))
			entrypoints[(*num_entrypoints)++] =
				VAEntrypointVideoProc;
	} else if (codec && driver_supports_profile(drv, profile)) {
		entrypoints[0] = VAEntrypointVLD;
		*num_entrypoints = 1;
	} else {
		*num_entrypoints = 0;
	}

	return VA_STATUS_SUCCESS;
}

VAStatus v4l2r_CreateConfig(VADriverContextP va_ctx, VAProfile profile,
			    VAEntrypoint entrypoint, VAConfigAttrib *attrib_list,
			    int num_attribs, VAConfigID *config_id)
{
	struct v4l2r_driver *drv = v4l2r_driver(va_ctx);
	const struct v4l2r_codec *codec = NULL;
	struct v4l2r_config *config;
	VAConfigID id;

	if (profile == VAProfileNone) {
		/* Video processing (rotation/mirroring/scaling blits) runs
		 * on the format converter. */
		if (entrypoint != VAEntrypointVideoProc ||
		    !v4l2r_converter_available(drv))
			return VA_STATUS_ERROR_UNSUPPORTED_ENTRYPOINT;
	} else {
		codec = v4l2r_codec_for_profile(profile);
		if (!codec || !driver_supports_profile(drv, profile))
			return VA_STATUS_ERROR_UNSUPPORTED_PROFILE;

		if (entrypoint != VAEntrypointVLD)
			return VA_STATUS_ERROR_UNSUPPORTED_ENTRYPOINT;
	}

	pthread_mutex_lock(&drv->mutex);
	id = v4l2r_handles_alloc(&drv->configs, sizeof(*config));
	config = V4L2R_CONFIG(drv, id);
	pthread_mutex_unlock(&drv->mutex);
	if (!config)
		return VA_STATUS_ERROR_ALLOCATION_FAILED;

	config->profile = profile;
	config->entrypoint = entrypoint;
	config->codec = codec;
	config->rt_format = v4l2r_profile_rt_format(profile);

	config->attributes[0].type = VAConfigAttribRTFormat;
	config->attributes[0].value = config->rt_format;
	config->nb_attributes = 1;

	for (int i = 0; i < num_attribs; i++) {
		if (attrib_list[i].type == VAConfigAttribRTFormat) {
			if (!(attrib_list[i].value & config->rt_format)) {
				pthread_mutex_lock(&drv->mutex);
				v4l2r_handles_free(&drv->configs, id);
				pthread_mutex_unlock(&drv->mutex);
				return VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT;
			}
			continue;
		}

		if (config->nb_attributes < V4L2R_MAX_CONFIG_ATTRIBUTES)
			config->attributes[config->nb_attributes++] =
				attrib_list[i];
	}

	*config_id = id;
	return VA_STATUS_SUCCESS;
}

VAStatus v4l2r_DestroyConfig(VADriverContextP va_ctx, VAConfigID config_id)
{
	struct v4l2r_driver *drv = v4l2r_driver(va_ctx);
	VAStatus status = VA_STATUS_SUCCESS;

	pthread_mutex_lock(&drv->mutex);
	if (!V4L2R_CONFIG(drv, config_id))
		status = VA_STATUS_ERROR_INVALID_CONFIG;
	else
		v4l2r_handles_free(&drv->configs, config_id);
	pthread_mutex_unlock(&drv->mutex);

	return status;
}

VAStatus v4l2r_QueryConfigAttributes(VADriverContextP va_ctx,
				     VAConfigID config_id, VAProfile *profile,
				     VAEntrypoint *entrypoint,
				     VAConfigAttrib *attrib_list,
				     int *num_attribs)
{
	struct v4l2r_driver *drv = v4l2r_driver(va_ctx);
	struct v4l2r_config *config;

	pthread_mutex_lock(&drv->mutex);
	config = V4L2R_CONFIG(drv, config_id);
	if (!config) {
		pthread_mutex_unlock(&drv->mutex);
		return VA_STATUS_ERROR_INVALID_CONFIG;
	}

	if (profile)
		*profile = config->profile;
	if (entrypoint)
		*entrypoint = config->entrypoint;
	if (num_attribs)
		*num_attribs = config->nb_attributes;
	if (attrib_list) {
		for (int i = 0; i < config->nb_attributes; i++)
			attrib_list[i] = config->attributes[i];
	}
	pthread_mutex_unlock(&drv->mutex);

	return VA_STATUS_SUCCESS;
}

VAStatus v4l2r_GetConfigAttributes(VADriverContextP va_ctx, VAProfile profile,
				   VAEntrypoint entrypoint,
				   VAConfigAttrib *attrib_list, int num_attribs)
{
	(void)va_ctx;
	(void)entrypoint;

	for (int i = 0; i < num_attribs; i++) {
		switch (attrib_list[i].type) {
		case VAConfigAttribRTFormat:
			attrib_list[i].value = v4l2r_profile_rt_format(profile);
			break;
		default:
			attrib_list[i].value = VA_ATTRIB_NOT_SUPPORTED;
			break;
		}
	}

	return VA_STATUS_SUCCESS;
}

VAStatus v4l2r_QueryDisplayAttributes(VADriverContextP va_ctx,
				      VADisplayAttribute *attr_list,
				      int *num_attributes)
{
	(void)va_ctx;
	(void)attr_list;

	if (num_attributes)
		*num_attributes = 0;

	return VA_STATUS_SUCCESS;
}

VAStatus v4l2r_GetDisplayAttributes(VADriverContextP va_ctx,
				    VADisplayAttribute *attr_list,
				    int num_attributes)
{
	(void)va_ctx;
	(void)attr_list;
	(void)num_attributes;
	return VA_STATUS_ERROR_UNIMPLEMENTED;
}

VAStatus v4l2r_SetDisplayAttributes(VADriverContextP va_ctx,
				    VADisplayAttribute *attr_list,
				    int num_attributes)
{
	(void)va_ctx;
	(void)attr_list;
	(void)num_attributes;
	return VA_STATUS_ERROR_UNIMPLEMENTED;
}

/* --- driver init/terminate --- */

VAStatus v4l2r_Terminate(VADriverContextP va_ctx)
{
	struct v4l2r_driver *drv = v4l2r_driver(va_ctx);
	unsigned int iter;
	uint32_t id;

	/* Tear down leftover objects, contexts first so surfaces unbind. */
	iter = 0;
	while (v4l2r_handles_next(&drv->contexts, &iter, &id))
		v4l2r_DestroyContext(va_ctx, id);

	iter = 0;
	while (v4l2r_handles_next(&drv->images, &iter, &id))
		v4l2r_DestroyImage(va_ctx, id);

	iter = 0;
	while (v4l2r_handles_next(&drv->buffers, &iter, &id))
		v4l2r_DestroyBuffer(va_ctx, id);

	iter = 0;
	while (v4l2r_handles_next(&drv->surfaces, &iter, &id))
		v4l2r_DestroySurfaces(va_ctx, &id, 1);

	v4l2r_handles_destroy(&drv->configs);
	v4l2r_handles_destroy(&drv->contexts);
	v4l2r_handles_destroy(&drv->surfaces);
	v4l2r_handles_destroy(&drv->buffers);
	v4l2r_handles_destroy(&drv->images);

	pthread_mutex_destroy(&drv->mutex);

	free(drv);
	va_ctx->pDriverData = NULL;

	return VA_STATUS_SUCCESS;
}

/* libva looks up __vaDriverInit_<major>_<minor> when loading the module. */
#define V4L2R_INIT_NAME2(major, minor) __vaDriverInit_##major##_##minor
#define V4L2R_INIT_NAME1(major, minor) V4L2R_INIT_NAME2(major, minor)
#define V4L2R_DRIVER_INIT V4L2R_INIT_NAME1(VA_MAJOR_VERSION, VA_MINOR_VERSION)

VAStatus __attribute__((visibility("default")))
V4L2R_DRIVER_INIT(VADriverContextP va_ctx);

VAStatus V4L2R_DRIVER_INIT(VADriverContextP va_ctx)
{
	struct VADriverVTable *vtable = va_ctx->vtable;
	struct v4l2r_driver *drv;

	va_ctx->version_major = VA_MAJOR_VERSION;
	va_ctx->version_minor = VA_MINOR_VERSION;
	va_ctx->max_profiles = V4L2R_MAX_PROFILES;
	va_ctx->max_entrypoints = V4L2R_MAX_ENTRYPOINTS;
	va_ctx->max_attributes = V4L2R_MAX_CONFIG_ATTRIBUTES;
	va_ctx->max_image_formats = V4L2R_MAX_IMAGE_FORMATS;
	va_ctx->max_subpic_formats = V4L2R_MAX_SUBPIC_FORMATS;
	va_ctx->max_display_attributes = V4L2R_MAX_DISPLAY_ATTRIBUTES;
	va_ctx->str_vendor = V4L2R_STR_VENDOR;

	vtable->vaTerminate = v4l2r_Terminate;
	vtable->vaQueryConfigProfiles = v4l2r_QueryConfigProfiles;
	vtable->vaQueryConfigEntrypoints = v4l2r_QueryConfigEntrypoints;
	vtable->vaQueryConfigAttributes = v4l2r_QueryConfigAttributes;
	vtable->vaCreateConfig = v4l2r_CreateConfig;
	vtable->vaDestroyConfig = v4l2r_DestroyConfig;
	vtable->vaGetConfigAttributes = v4l2r_GetConfigAttributes;
	vtable->vaCreateSurfaces = v4l2r_CreateSurfaces;
	vtable->vaCreateSurfaces2 = v4l2r_CreateSurfaces2;
	vtable->vaDestroySurfaces = v4l2r_DestroySurfaces;
	vtable->vaExportSurfaceHandle = v4l2r_ExportSurfaceHandle;
	vtable->vaCreateContext = v4l2r_CreateContext;
	vtable->vaDestroyContext = v4l2r_DestroyContext;
	vtable->vaCreateBuffer = v4l2r_CreateBuffer;
	vtable->vaBufferSetNumElements = v4l2r_BufferSetNumElements;
	vtable->vaMapBuffer = v4l2r_MapBuffer;
	vtable->vaUnmapBuffer = v4l2r_UnmapBuffer;
	vtable->vaDestroyBuffer = v4l2r_DestroyBuffer;
	vtable->vaBufferInfo = v4l2r_BufferInfo;
	vtable->vaAcquireBufferHandle = v4l2r_AcquireBufferHandle;
	vtable->vaReleaseBufferHandle = v4l2r_ReleaseBufferHandle;
	vtable->vaBeginPicture = v4l2r_BeginPicture;
	vtable->vaRenderPicture = v4l2r_RenderPicture;
	vtable->vaEndPicture = v4l2r_EndPicture;
	vtable->vaSyncSurface = v4l2r_SyncSurface;
	vtable->vaQuerySurfaceAttributes = v4l2r_QuerySurfaceAttributes;
	vtable->vaQuerySurfaceStatus = v4l2r_QuerySurfaceStatus;
	vtable->vaPutSurface = v4l2r_PutSurface;
	vtable->vaLockSurface = v4l2r_LockSurface;
	vtable->vaUnlockSurface = v4l2r_UnlockSurface;
	vtable->vaQueryImageFormats = v4l2r_QueryImageFormats;
	vtable->vaCreateImage = v4l2r_CreateImage;
	vtable->vaDeriveImage = v4l2r_DeriveImage;
	vtable->vaDestroyImage = v4l2r_DestroyImage;
	vtable->vaSetImagePalette = v4l2r_SetImagePalette;
	vtable->vaGetImage = v4l2r_GetImage;
	vtable->vaPutImage = v4l2r_PutImage;
	vtable->vaQuerySubpictureFormats = v4l2r_QuerySubpictureFormats;
	vtable->vaCreateSubpicture = v4l2r_CreateSubpicture;
	vtable->vaDestroySubpicture = v4l2r_DestroySubpicture;
	vtable->vaSetSubpictureImage = v4l2r_SetSubpictureImage;
	vtable->vaSetSubpictureChromakey = v4l2r_SetSubpictureChromakey;
	vtable->vaSetSubpictureGlobalAlpha = v4l2r_SetSubpictureGlobalAlpha;
	vtable->vaAssociateSubpicture = v4l2r_AssociateSubpicture;
	vtable->vaDeassociateSubpicture = v4l2r_DeassociateSubpicture;
	vtable->vaQueryDisplayAttributes = v4l2r_QueryDisplayAttributes;
	vtable->vaGetDisplayAttributes = v4l2r_GetDisplayAttributes;
	vtable->vaSetDisplayAttributes = v4l2r_SetDisplayAttributes;

	if (va_ctx->vtable_vpp) {
		va_ctx->vtable_vpp->vaQueryVideoProcFilters =
			v4l2r_QueryVideoProcFilters;
		va_ctx->vtable_vpp->vaQueryVideoProcFilterCaps =
			v4l2r_QueryVideoProcFilterCaps;
		va_ctx->vtable_vpp->vaQueryVideoProcPipelineCaps =
			v4l2r_QueryVideoProcPipelineCaps;
	}

	drv = calloc(1, sizeof(*drv));
	if (!drv)
		return VA_STATUS_ERROR_ALLOCATION_FAILED;

	if (v4l2r_handles_init(&drv->configs, V4L2R_ID_OFFSET_CONFIG) < 0 ||
	    v4l2r_handles_init(&drv->contexts, V4L2R_ID_OFFSET_CONTEXT) < 0 ||
	    v4l2r_handles_init(&drv->surfaces, V4L2R_ID_OFFSET_SURFACE) < 0 ||
	    v4l2r_handles_init(&drv->buffers, V4L2R_ID_OFFSET_BUFFER) < 0 ||
	    v4l2r_handles_init(&drv->images, V4L2R_ID_OFFSET_IMAGE) < 0) {
		v4l2r_handles_destroy(&drv->configs);
		v4l2r_handles_destroy(&drv->contexts);
		v4l2r_handles_destroy(&drv->surfaces);
		v4l2r_handles_destroy(&drv->buffers);
		v4l2r_handles_destroy(&drv->images);
		free(drv);
		return VA_STATUS_ERROR_ALLOCATION_FAILED;
	}

	pthread_mutex_init(&drv->mutex, NULL);
	va_ctx->pDriverData = drv;

	enumerate_decoders(drv);
	if (!drv->nb_decoders) {
		v4l2r_log("no V4L2 Request API decoder found\n");
		v4l2r_Terminate(va_ctx);
		return VA_STATUS_ERROR_OPERATION_FAILED;
	}

	v4l2r_log("detected %u Request API decoder%s\n", drv->nb_decoders,
		  drv->nb_decoders == 1 ? "" : "s");

	return VA_STATUS_SUCCESS;
}
