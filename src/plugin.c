#include "gst-sdr-plugins.h"

#define PACKAGE "gstsdrplugins"
#define VERSION "0.2.0"
#define LICENSE "LGPL"
#define ORIGIN "https://github.com/user/gst-sdr-plugins"
#define DESCRIPTION "GStreamer SDR source + AM/FM demodulator - SDR denoise (3 elements)"

static gboolean
plugin_init (GstPlugin * plugin)
{
  gboolean ret = TRUE;
  ret &= gst_element_register (plugin, "sdrsrc", GST_RANK_NONE, GST_TYPE_SDR_SRC);
  ret &= gst_element_register (plugin, "sdrdemod", GST_RANK_NONE, GST_TYPE_SDR_DEMOD);
  ret &= gst_element_register (plugin, "sdrdenoise", GST_RANK_NONE, GST_TYPE_SDR_DENOISE);
  return ret;
}

/* Library is built as libgstsdrplugins.so → symbol gst_plugin_sdrplugins_get_desc */
GST_PLUGIN_DEFINE (
    GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    sdrplugins,
    DESCRIPTION,
    plugin_init,
    VERSION,
    LICENSE,
    PACKAGE,
    ORIGIN
)
