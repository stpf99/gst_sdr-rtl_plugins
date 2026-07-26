#ifndef __GST_SDR_PLUGINS_H__
#define __GST_SDR_PLUGINS_H__

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <gst/gst.h>
#include <gst/base/gstbasetransform.h>
#include <gst/base/gstpushsrc.h>
#include <complex.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>

#include "config.h"

G_BEGIN_DECLS

#define GST_TYPE_SDR_SRC (gst_sdr_src_get_type())
#define GST_SDR_SRC(obj) (G_TYPE_CHECK_INSTANCE_CAST((obj), GST_TYPE_SDR_SRC, GstSdrSrc))

typedef struct _GstSdrSrc GstSdrSrc;
typedef struct _GstSdrSrcClass GstSdrSrcClass;

struct _GstSdrSrc {
  GstPushSrc parent;
  gchar *mode;
  gchar *host;
  guint16 port;
  guint32 frequency;
  guint32 sample_rate;
  gfloat gain;
  gint sock_fd;
  gboolean connected;
  guint8 *recv_buf;
  gsize recv_buf_size;
  guint8 pending_byte;
  gboolean has_pending_byte;
#if HAVE_RTLSDR
  void *rtl_dev;
#endif
};

struct _GstSdrSrcClass {
  GstPushSrcClass parent_class;
};

GType gst_sdr_src_get_type (void);

#define GST_TYPE_SDR_DEMOD (gst_sdr_demod_get_type())
#define GST_SDR_DEMOD(obj) (G_TYPE_CHECK_INSTANCE_CAST((obj), GST_TYPE_SDR_DEMOD, GstSdrDemod))

typedef enum {
  GST_SDR_MODE_FM = 0,
  GST_SDR_MODE_AM = 1,
} GstSdrMode;

typedef struct _GstSdrDemod GstSdrDemod;
typedef struct _GstSdrDemodClass GstSdrDemodClass;

struct _GstSdrDemod {
  GstBaseTransform parent;

  GstSdrMode mode;
  gboolean stereo;
  gfloat max_deviation;
  guint target_audio_rate;
  gfloat audio_cutoff;
  guint tau_us;
  gfloat freq_offset_hz;
  gfloat stereo_mix;

  guint sample_rate;
  guint out_rate;
  guint audio_rate;
  guint decim_factor;
  guint audio_decim;

  gfloat nco_phase;
  gfloat nco_delta;
  gfloat prev_theta;

  gfloat *fir_taps;
  gint n_taps;
  gfloat *fir_audio;
  gint n_taps_audio;

  gfloat *tail_i;
  gfloat *tail_q;
  gint tail_len;
  gint decim_phase;

  gfloat *tail_audio_l;
  gfloat *tail_audio_r;
  gint tail_audio_len;
  gint audio_phase;
  gint audio_phase_r;

  gfloat *scratch;
  gsize scratch_cap;
  gfloat *scratch2;
  gsize scratch2_cap;
  gfloat *disc_buf;
  gsize disc_buf_cap;
  gint last_audio_len;

  gfloat deemph_coeff;
  gfloat deemph_prev[2];

  gfloat pilot_phase;
  gfloat pilot_freq;
  gfloat pilot_integrator;
};

struct _GstSdrDemodClass {
  GstBaseTransformClass parent_class;
};

GType gst_sdr_demod_get_type (void);

#define GST_TYPE_SDR_DENOISE (gst_sdr_denoise_get_type())
#define GST_SDR_DENOISE(obj) (G_TYPE_CHECK_INSTANCE_CAST((obj), GST_TYPE_SDR_DENOISE, GstSdrDenoise))

#define SDR_DENOISE_FFT 2048
#define SDR_DENOISE_HOP 512
#define SDR_DENOISE_NFREQ (SDR_DENOISE_FFT / 2 + 1)

typedef struct _GstSdrDenoise GstSdrDenoise;
typedef struct _GstSdrDenoiseClass GstSdrDenoiseClass;

struct _GstSdrDenoise {
  GstBaseTransform parent;

  gboolean enabled;
  gfloat threshold_db;
  gfloat alpha_up;
  gfloat alpha_down;

  gint channels;
  guint rate;

  gfloat *hann;                          /* shared window, SDR_DENOISE_FFT */
  gfloat *hist[2];                       /* per-channel input history */
  gfloat *acc[2];                        /* per-channel OLA accumulator */
  gfloat *pending[2];                    /* per-channel leftover input < HOP */
  gint pending_n[2];
  gfloat *noise_floor[2];                /* per-channel, SDR_DENOISE_NFREQ */
  gboolean allocated;

  gfloat *deint[2];                      /* scratch: deinterleaved in/out per channel */
  gsize deint_cap;
  
  /* Bufor wyjściowy i jego pojemność */
  gfloat *out_queue[2];
  gint out_queue_n[2];
  gsize out_queue_cap[2];
};

struct _GstSdrDenoiseClass {
  GstBaseTransformClass parent_class;
};

GType gst_sdr_denoise_get_type (void);

G_END_DECLS

#endif
