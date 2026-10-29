project('asahi-va-driver', 'c',
        version: '1.2',
        license: 'GPL-3.0',
        default_options: [
          'c_std=gnu11',
          'buildtype=release',
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
conf.set_quoted('DRIVER_VERSION', meson.project_version())
conf.set_quoted('DRIVER_NAME', meson.project_name())

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

sources = [
  'src/driver.c',
  'src/handles.c',
  'src/context.c',
  'src/convert.c',
  'src/decode.c',
  'src/surface.c',
  'src/buffer.c',
  'src/image.c',
  'src/codec/mpeg2.c',
  'src/codec/h264.c',
  'src/codec/hevc.c',
  'src/codec/vp8.c',
  'src/codec/vp9.c',
  'src/codec/av1.c',
]

shared_module('asahi_drv_video',
              sources,
              name_prefix: '',
              include_directories: root_inc,
              c_args: ['-fvisibility=hidden'],
              dependencies: [libva_dep, libdrm_dep, threads_dep],
              install: true,
              install_dir: driverdir)
