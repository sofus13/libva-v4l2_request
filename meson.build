project('libva-v4l2_request', 'c',
        version: '1.1',
        license: 'GPL-3.0',
        default_options: [
          'c_std=gnu11',
          'buildtype=debugoptimized',
          'warning_level=2',
        ])

cc = meson.get_compiler('c')

libva_dep = dependency('libva', version: '>= 1.10.0')
libdrm_dep = dependency('libdrm')
threads_dep = dependency('threads')

# The driver talks straight to the kernel, make sure the uapi headers are
# recent enough for the stateless codec interfaces we support.
codec_checks = {
  'h264':  'V4L2_CID_STATELESS_H264_DECODE_MODE',
  'hevc':  'V4L2_CID_STATELESS_HEVC_SPS',
  'mpeg2': 'V4L2_CID_STATELESS_MPEG2_SEQUENCE',
  'vp8':   'V4L2_CID_STATELESS_VP8_FRAME',
  'vp9':   'V4L2_CID_STATELESS_VP9_FRAME',
  'av1':   'V4L2_CID_STATELESS_AV1_SEQUENCE',
}

conf = configuration_data()
foreach codec, ctrl : codec_checks
  have = cc.compiles('#include <linux/videodev2.h>\nint i = @0@;'.format(ctrl),
                     name: 'kernel supports stateless @0@'.format(codec))
  conf.set10('HAVE_V4L2_CTRL_' + codec.to_upper(), have)
endforeach

conf.set10('HAVE_HEVC_DECODE_PARAMS_NUM_DELTA_POCS',
           cc.has_member('struct v4l2_ctrl_hevc_decode_params',
                         'num_delta_pocs_of_ref_rps_idx',
                         prefix: '#include <linux/videodev2.h>'))

configure_file(output: 'config.h', configuration: conf)
root_inc = include_directories('.')

if get_option('driverdir') != ''
  driverdir = get_option('driverdir')
else
  driverdir = libva_dep.get_variable(pkgconfig: 'driverdir',
                                     default_value: get_option('prefix') / get_option('libdir') / 'dri')
endif

subdir('src')
