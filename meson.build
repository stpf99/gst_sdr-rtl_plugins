project('gst-sdr-plugins', 'c',
  version : '0.2.0',
  license : 'LGPL',
  meson_version : '>= 0.60',
  default_options : [
    'warning_level=2',
    'c_std=gnu11',
  ],
)

gst_req = '>= 1.20.0'

gst_dep      = dependency('gstreamer-1.0',      version : gst_req)
gst_base_dep = dependency('gstreamer-base-1.0', version : gst_req)
glib_dep     = dependency('glib-2.0')
gobject_dep  = dependency('gobject-2.0')

cc = meson.get_compiler('c')
m_dep = cc.find_library('m', required : false)

# Optional local USB RTL-SDR support (librtlsdr). Build without it if missing.
rtlsdr_dep = dependency('librtlsdr', required : false)
if not rtlsdr_dep.found()
  rtlsdr_dep = cc.find_library('rtlsdr', required : false)
endif

conf = configuration_data()
conf.set10('HAVE_RTLSDR', rtlsdr_dep.found())
configure_file(output : 'config.h', configuration : conf)

deps = [gst_dep, gst_base_dep, glib_dep, gobject_dep, m_dep]
if rtlsdr_dep.found()
  deps += rtlsdr_dep
endif

sources = [
  'src/plugin.c',
  'src/gst_sdr_src.c',
  'src/gst_sdr_demod.c',
  'src/gst_sdr_denoise.c',
]

gstsdrplugins = shared_library('gstsdrplugins',
  sources,
  dependencies : deps,
  include_directories : include_directories('src', '.'),
  install : true,
  install_dir : join_paths(get_option('libdir'), 'gstreamer-1.0'),
  gnu_symbol_visibility : 'hidden',
)

summary({
  'GStreamer' : gst_req,
  'librtlsdr' : rtlsdr_dep.found(),
  'Install'   : join_paths(get_option('prefix'), get_option('libdir'), 'gstreamer-1.0'),
}, section : 'Configuration')
