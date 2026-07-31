#include "gst-sdr-plugins.h"

#if HAVE_RTLSDR
#include <rtl-sdr.h>
#endif

GST_DEBUG_CATEGORY_STATIC (gst_sdr_src_debug);
#define GST_CAT_DEFAULT gst_sdr_src_debug

enum {
  PROP_0,
  PROP_MODE,
  PROP_HOST,
  PROP_PORT,
  PROP_FREQUENCY,
  PROP_SAMPLE_RATE,
  PROP_GAIN,
  PROP_AUTO_GAIN,
  PROP_AUTO_GAIN_TARGET,
  PROP_LAST
};

#define DEFAULT_MODE "tcp"
#define DEFAULT_HOST "127.0.0.1"
#define DEFAULT_PORT 1234
#define DEFAULT_FREQ 100000000
#define DEFAULT_SRATE 1000000
#define DEFAULT_GAIN 0.0f
#define DEFAULT_AUTO_GAIN FALSE
/* Target mean IQ power in dBFS (0 dBFS = full-scale sine). RTL-SDR IQ
 * sits well under full scale in normal reception; -18 dBFS leaves
 * headroom against sudden strong-signal peaks while still using most
 * of the ADC's dynamic range. */
#define DEFAULT_AUTO_GAIN_TARGET -18.0f
#define AUTO_GAIN_MIN 0.9f
#define AUTO_GAIN_MAX 49.6f
#define AUTO_GAIN_STEP_DB 1.0f
/* Only re-evaluate every N buffers so the loop doesn't chase every
 * momentary blip (each buffer is tens of ms at typical sample rates --
 * this gives a AGC response time on the order of a few hundred ms). */
#define AUTO_GAIN_HOLD_BUFFERS 6

#define RTL_TCP_MAGIC "RTL0"

static GstStaticPadTemplate srctemplate = GST_STATIC_PAD_TEMPLATE ("src",
    GST_PAD_SRC,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS ("application/x-raw,"
        "format=F32LE,"
        "layout=interleaved,"
        "channels=2,"
        "rate=[8000,10000000]"));

G_DEFINE_TYPE (GstSdrSrc, gst_sdr_src, GST_TYPE_PUSH_SRC);

static void gst_sdr_src_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec);
static void gst_sdr_src_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec);
static void gst_sdr_src_finalize (GObject * object);

static gboolean gst_sdr_src_start (GstBaseSrc * src);
static gboolean gst_sdr_src_stop (GstBaseSrc * src);
static gboolean gst_sdr_src_unlock (GstBaseSrc * src);
static gboolean gst_sdr_src_unlock_stop (GstBaseSrc * src);
static GstCaps *gst_sdr_src_get_caps (GstBaseSrc * src, GstCaps * filter);
static gboolean gst_sdr_src_negotiate (GstBaseSrc * src);
static GstFlowReturn gst_sdr_src_create (GstPushSrc * src, GstBuffer ** buf);

/* ---- TCP helpers ------------------------------------------------------- */

static gboolean
gst_sdr_src_send_cmd (GstSdrSrc * sdr, guint8 op, guint32 val)
{
  guint8 cmd[5];
  ssize_t n;

  cmd[0] = op;
  cmd[1] = (val >> 24) & 0xFF;
  cmd[2] = (val >> 16) & 0xFF;
  cmd[3] = (val >> 8) & 0xFF;
  cmd[4] = val & 0xFF;

  n = send (sdr->sock_fd, cmd, sizeof (cmd), 0);
  if (n != (ssize_t) sizeof (cmd)) {
    GST_ERROR_OBJECT (sdr, "Failed to send tuner command 0x%02x (val=%u)", op, val);
    return FALSE;
  }
  return TRUE;
}

/* Push a (possibly new) manual gain value out to whichever backend is
 * active. Shared by PROP_GAIN's set_property and the auto-gain loop in
 * create(), so both paths stay consistent with each other. */
static void
gst_sdr_src_apply_gain (GstSdrSrc * sdr)
{
  if (sdr->connected && sdr->sock_fd >= 0) {
    if (sdr->gain > 0.0f) {
      gst_sdr_src_send_cmd (sdr, 0x03, 1);
      gst_sdr_src_send_cmd (sdr, 0x04, (guint) (sdr->gain * 10.0f + 0.5f));
    } else {
      gst_sdr_src_send_cmd (sdr, 0x03, 0);
      gst_sdr_src_send_cmd (sdr, 0x08, 1);
    }
  }
#if HAVE_RTLSDR
  else if (sdr->rtl_dev) {
    if (sdr->gain > 0.0f) {
      rtlsdr_set_tuner_gain_mode ((rtlsdr_dev_t *) sdr->rtl_dev, 1);
      rtlsdr_set_tuner_gain ((rtlsdr_dev_t *) sdr->rtl_dev, (int) (sdr->gain * 10.0f + 0.5f));
    } else {
      rtlsdr_set_tuner_gain_mode ((rtlsdr_dev_t *) sdr->rtl_dev, 0);
    }
  }
#endif
}

/* Software AGC: measures mean IQ power over the just-produced buffer,
 * tracks it with a slow EMA (so single strong pulses/fades don't jerk
 * the gain around), and every AUTO_GAIN_HOLD_BUFFERS buffers nudges
 * `gain` by one step towards auto_gain_target_db. Runs entirely inside
 * the source -- no external control loop needed, so it keeps working
 * the same way whether driven from gst-launch, the GTK4 GUI, or
 * anything else that just sets auto-gain=true. */
static void
gst_sdr_src_auto_gain_update (GstSdrSrc * sdr, const gfloat * iq, gsize n_pairs)
{
  gdouble sum = 0.0;
  gfloat power_db;
  gsize i;

  if (!sdr->auto_gain || n_pairs == 0)
    return;

  for (i = 0; i < n_pairs; i++) {
    gfloat ii = iq[i * 2 + 0], qq = iq[i * 2 + 1];
    sum += (gdouble) (ii * ii + qq * qq);
  }
  power_db = 10.0f * log10f ((gfloat) (sum / (gdouble) n_pairs) + 1e-12f);

  if (!sdr->power_ema_init) {
    sdr->power_ema_db = power_db;
    sdr->power_ema_init = TRUE;
  } else {
    sdr->power_ema_db = 0.9f * sdr->power_ema_db + 0.1f * power_db;
  }

  if (++sdr->auto_gain_hold < AUTO_GAIN_HOLD_BUFFERS)
    return;
  sdr->auto_gain_hold = 0;

  if (sdr->power_ema_db < sdr->auto_gain_target_db - 1.0f) {
    gfloat g = sdr->gain <= 0.0f ? AUTO_GAIN_MIN : sdr->gain + AUTO_GAIN_STEP_DB;
    if (g > AUTO_GAIN_MAX)
      g = AUTO_GAIN_MAX;
    if (g != sdr->gain) {
      sdr->gain = g;
      gst_sdr_src_apply_gain (sdr);
      GST_DEBUG_OBJECT (sdr, "auto-gain: power=%.1fdB -> gain up to %.1fdB", sdr->power_ema_db, g);
    }
  } else if (sdr->power_ema_db > sdr->auto_gain_target_db + 1.0f) {
    gfloat g = sdr->gain - AUTO_GAIN_STEP_DB;
    if (g < AUTO_GAIN_MIN)
      g = AUTO_GAIN_MIN;
    if (g != sdr->gain) {
      sdr->gain = g;
      gst_sdr_src_apply_gain (sdr);
      GST_DEBUG_OBJECT (sdr, "auto-gain: power=%.1fdB -> gain down to %.1fdB", sdr->power_ema_db, g);
    }
  }
}

static gboolean
gst_sdr_src_tcp_connect (GstSdrSrc * sdr)
{
  struct sockaddr_in addr;
  guint8 header[12];

  if (sdr->sock_fd >= 0)
    return TRUE;

  sdr->sock_fd = socket (AF_INET, SOCK_STREAM, 0);
  if (sdr->sock_fd < 0) {
    GST_ERROR_OBJECT (sdr, "Failed to create socket: %s", g_strerror (errno));
    return FALSE;
  }

  /* Default OS receive buffer is often small (commonly a few hundred KB).
   * Over a jittery link (WiFi to a DD-WRT router relaying rtl_tcp) that
   * gives almost no slack: if our consumer thread is a few tens of ms
   * late reading (scheduling, a GC-ish glib alloc, whatever), the kernel
   * buffer fills and TCP either throttles the sender or, worse, jitter
   * compounds into audible dropouts downstream. A few MB of headroom
   * costs nothing and absorbs bursts without touching the actual data
   * path. Best-effort: if the kernel clamps it lower, that's fine too. */
  {
    int rcvbuf = 4 * 1024 * 1024;
    setsockopt (sdr->sock_fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof (rcvbuf));
  }

  memset (&addr, 0, sizeof (addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons (sdr->port);

  if (inet_aton (sdr->host, &addr.sin_addr) == 0) {
    GST_ERROR_OBJECT (sdr, "Invalid host address: %s", sdr->host);
    close (sdr->sock_fd);
    sdr->sock_fd = -1;
    return FALSE;
  }

  if (connect (sdr->sock_fd, (struct sockaddr *) &addr, sizeof (addr)) < 0) {
    GST_ERROR_OBJECT (sdr, "Failed to connect to %s:%d: %s",
        sdr->host, sdr->port, g_strerror (errno));
    close (sdr->sock_fd);
    sdr->sock_fd = -1;
    return FALSE;
  }

  GST_INFO_OBJECT (sdr, "Connected to rtl_tcp %s:%d", sdr->host, sdr->port);

  if (recv (sdr->sock_fd, header, sizeof (header), MSG_WAITALL) !=
      (ssize_t) sizeof (header)) {
    GST_ERROR_OBJECT (sdr, "Failed to read RTL-TCP header");
    close (sdr->sock_fd);
    sdr->sock_fd = -1;
    return FALSE;
  }

  if (memcmp (header, RTL_TCP_MAGIC, 4) != 0) {
    GST_ERROR_OBJECT (sdr, "%s:%d is not an rtl_tcp server (bad magic)",
        sdr->host, sdr->port);
    close (sdr->sock_fd);
    sdr->sock_fd = -1;
    return FALSE;
  }

  /*
   * rtl_tcp command opcodes (rtl_tcp.c):
   *   0x01 SET_FREQ, 0x02 SET_SAMPLE_RATE, 0x03 SET_GAIN_MODE,
   *   0x04 SET_GAIN (tenths of dB), 0x08 SET_AGC_MODE,
   *   0x09 SET_DIRECT_SAMPLING, 0x0a SET_OFFSET_TUNING, 0x0e SET_BIAS_TEE
   *
   * Order matches working clients (e.g. gr-osmosdr / GQRX style):
   * gain mode → AGC → direct sampling OFF → offset OFF → bias tee OFF
   * → sample rate → freq → gain.
   * Direct sampling MUST be forced off: some servers leave it on from a
   * previous HF session, which yields pure noise on VHF/UHF (FC0012/R820T).
   */
  {
    gboolean manual = (sdr->gain > 0.0f);

    if (!gst_sdr_src_send_cmd (sdr, 0x03, manual ? 1 : 0) ||   /* gain mode */
        !gst_sdr_src_send_cmd (sdr, 0x08, manual ? 0 : 1) ||   /* AGC */
        !gst_sdr_src_send_cmd (sdr, 0x09, 0) ||                 /* direct sampling OFF */
        !gst_sdr_src_send_cmd (sdr, 0x0a, 0) ||                 /* offset tuning OFF */
        !gst_sdr_src_send_cmd (sdr, 0x0e, 0) ||                 /* bias tee OFF */
        !gst_sdr_src_send_cmd (sdr, 0x02, sdr->sample_rate) ||
        !gst_sdr_src_send_cmd (sdr, 0x01, sdr->frequency)) {
      close (sdr->sock_fd);
      sdr->sock_fd = -1;
      return FALSE;
    }

    if (manual) {
      /* rtl_tcp expects gain in tenths of a dB */
      if (!gst_sdr_src_send_cmd (sdr, 0x04, (guint) (sdr->gain * 10.0f + 0.5f))) {
        close (sdr->sock_fd);
        sdr->sock_fd = -1;
        return FALSE;
      }
    }
  }

  GST_INFO_OBJECT (sdr,
      "rtl_tcp init: freq=%u rate=%u gain=%.1f (manual=%d) direct_sampling=0",
      sdr->frequency, sdr->sample_rate, sdr->gain, (sdr->gain > 0.0f));

  sdr->connected = TRUE;
  sdr->has_pending_byte = FALSE;
  return TRUE;
}

static void
gst_sdr_src_tcp_disconnect (GstSdrSrc * sdr)
{
  if (sdr->sock_fd >= 0) {
    close (sdr->sock_fd);
    sdr->sock_fd = -1;
    sdr->connected = FALSE;
    sdr->has_pending_byte = FALSE;
    GST_DEBUG_OBJECT (sdr, "TCP disconnected");
  }
}

#if HAVE_RTLSDR
static gboolean
gst_sdr_src_usb_open (GstSdrSrc * sdr)
{
  int r;
  rtlsdr_dev_t *dev = NULL;

  r = rtlsdr_open (&dev, 0);
  if (r < 0 || !dev) {
    GST_ERROR_OBJECT (sdr, "rtlsdr_open failed (%d)", r);
    return FALSE;
  }

  rtlsdr_set_sample_rate (dev, sdr->sample_rate);
  rtlsdr_set_center_freq (dev, sdr->frequency);
  if (sdr->gain > 0.0f) {
    rtlsdr_set_tuner_gain_mode (dev, 1);
    rtlsdr_set_tuner_gain (dev, (int) (sdr->gain * 10));
  } else {
    rtlsdr_set_tuner_gain_mode (dev, 0);
  }
  rtlsdr_reset_buffer (dev);

  sdr->rtl_dev = dev;
  sdr->connected = TRUE;
  GST_INFO_OBJECT (sdr, "Opened local RTL-SDR @ %u Hz, rate %u",
      sdr->frequency, sdr->sample_rate);
  return TRUE;
}

static void
gst_sdr_src_usb_close (GstSdrSrc * sdr)
{
  if (sdr->rtl_dev) {
    rtlsdr_close ((rtlsdr_dev_t *) sdr->rtl_dev);
    sdr->rtl_dev = NULL;
    sdr->connected = FALSE;
  }
}
#endif

/* ---- Caps negotiation fix ----------------------------------------------
 *
 * GstBaseSrc::start may call set_caps() with the correct fixed rate, but
 * immediately afterwards start_complete() triggers negotiate() which, without
 * an overridden get_caps(), falls back to the pad template range and fixates
 * to the minimum (8000). That overwrites the real rate and desyncs every
 * downstream element's FIR / decimation math → pure noise.
 *
 * Fix: get_caps always advertises the exact sample-rate property value, and
 * negotiate() re-applies those fixed caps so nothing can regress them.
 */

static GstCaps *
gst_sdr_src_make_fixed_caps (GstSdrSrc * sdr)
{
  return gst_caps_new_simple ("application/x-raw",
      "format", G_TYPE_STRING, "F32LE",
      "layout", G_TYPE_STRING, "interleaved",
      "channels", G_TYPE_INT, 2,
      "rate", G_TYPE_INT, (gint) sdr->sample_rate,
      NULL);
}

static GstCaps *
gst_sdr_src_get_caps (GstBaseSrc * src, GstCaps * filter)
{
  GstSdrSrc *sdr = GST_SDR_SRC (src);
  GstCaps *caps = gst_sdr_src_make_fixed_caps (sdr);

  if (filter) {
    GstCaps *tmp = gst_caps_intersect_full (filter, caps, GST_CAPS_INTERSECT_FIRST);
    gst_caps_unref (caps);
    caps = tmp;
  }
  return caps;
}

static gboolean
gst_sdr_src_negotiate (GstBaseSrc * src)
{
  GstSdrSrc *sdr = GST_SDR_SRC (src);
  GstCaps *caps;
  gboolean ok;

  caps = gst_sdr_src_make_fixed_caps (sdr);
  ok = gst_base_src_set_caps (src, caps);
  gst_caps_unref (caps);

  if (!ok) {
    GST_ERROR_OBJECT (sdr, "Failed to fix caps to rate=%u", sdr->sample_rate);
    return FALSE;
  }
  GST_INFO_OBJECT (sdr, "Negotiated fixed IQ rate=%u Hz", sdr->sample_rate);
  return TRUE;
}

/* ---- GObject / element lifecycle --------------------------------------- */

static void
gst_sdr_src_class_init (GstSdrSrcClass * klass)
{
  GObjectClass *gobject_class = G_OBJECT_CLASS (klass);
  GstBaseSrcClass *gstbasesrc_class = GST_BASE_SRC_CLASS (klass);
  GstPushSrcClass *gstpushsrc_class = GST_PUSH_SRC_CLASS (klass);
  GstElementClass *gstelement_class = GST_ELEMENT_CLASS (klass);

  GST_DEBUG_CATEGORY_INIT (gst_sdr_src_debug, "sdrsrc", 0, "SDR IQ Source");

  gst_element_class_set_static_metadata (gstelement_class,
      "SDR IQ Source", "Source/Audio",
      "Receive IQ samples from rtl_tcp or local USB RTL-SDR",
      "Tomasz");

  gst_element_class_add_static_pad_template (gstelement_class, &srctemplate);

  gobject_class->set_property = gst_sdr_src_set_property;
  gobject_class->get_property = gst_sdr_src_get_property;
  gobject_class->finalize = gst_sdr_src_finalize;

  g_object_class_install_property (gobject_class, PROP_MODE,
      g_param_spec_string ("mode", "Mode",
          "Source mode: \"tcp\" (rtl_tcp) or \"usb\" (local librtlsdr)",
          DEFAULT_MODE, G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (gobject_class, PROP_HOST,
      g_param_spec_string ("host", "Host", "rtl_tcp host", DEFAULT_HOST,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (gobject_class, PROP_PORT,
      g_param_spec_uint ("port", "Port", "rtl_tcp port", 1, 65535, DEFAULT_PORT,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (gobject_class, PROP_FREQUENCY,
      g_param_spec_uint ("frequency", "Frequency", "Tuner frequency in Hz",
          24000000, 1862000000, DEFAULT_FREQ,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (gobject_class, PROP_SAMPLE_RATE,
      g_param_spec_uint ("sample-rate", "Sample Rate", "ADC sample rate in Hz",
          225001, 3200000, DEFAULT_SRATE,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (gobject_class, PROP_GAIN,
      g_param_spec_float ("gain", "Gain",
          "Tuner gain in dB (0 = hardware AGC). Overridden live while "
          "auto-gain=true, but keeps working exactly as before otherwise.",
          0.0f, 50.0f, DEFAULT_GAIN,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (gobject_class, PROP_AUTO_GAIN,
      g_param_spec_boolean ("auto-gain", "Auto Gain",
          "Software AGC: continuously adjust gain towards auto-gain-target-db "
          "(default off, existing manual 'gain' behaviour is unchanged)",
          DEFAULT_AUTO_GAIN, G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  g_object_class_install_property (gobject_class, PROP_AUTO_GAIN_TARGET,
      g_param_spec_float ("auto-gain-target-db", "Auto Gain Target",
          "Target mean IQ power in dBFS for the auto-gain loop",
          -60.0f, 0.0f, DEFAULT_AUTO_GAIN_TARGET,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  gstbasesrc_class->start = gst_sdr_src_start;
  gstbasesrc_class->stop = gst_sdr_src_stop;
  gstbasesrc_class->unlock = gst_sdr_src_unlock;
  gstbasesrc_class->unlock_stop = gst_sdr_src_unlock_stop;
  gstbasesrc_class->get_caps = gst_sdr_src_get_caps;
  gstbasesrc_class->negotiate = gst_sdr_src_negotiate;
  gstpushsrc_class->create = gst_sdr_src_create;
}

static void
gst_sdr_src_init (GstSdrSrc * sdr)
{
  sdr->mode = g_strdup (DEFAULT_MODE);
  sdr->host = g_strdup (DEFAULT_HOST);
  sdr->port = DEFAULT_PORT;
  sdr->frequency = DEFAULT_FREQ;
  sdr->sample_rate = DEFAULT_SRATE;
  sdr->gain = DEFAULT_GAIN;
  sdr->sock_fd = -1;
  sdr->connected = FALSE;
  sdr->recv_buf_size = 65536;
  sdr->recv_buf = g_malloc (sdr->recv_buf_size);
  sdr->has_pending_byte = FALSE;
  sdr->pending_byte = 0;
  sdr->flushing = FALSE;
  sdr->auto_gain = DEFAULT_AUTO_GAIN;
  sdr->auto_gain_target_db = DEFAULT_AUTO_GAIN_TARGET;
  sdr->power_ema_db = 0.0f;
  sdr->power_ema_init = FALSE;
  sdr->auto_gain_hold = 0;
#if HAVE_RTLSDR
  sdr->rtl_dev = NULL;
#endif

  gst_base_src_set_live (GST_BASE_SRC (sdr), TRUE);
  gst_base_src_set_format (GST_BASE_SRC (sdr), GST_FORMAT_TIME);
  gst_base_src_set_do_timestamp (GST_BASE_SRC (sdr), TRUE);
}

static void
gst_sdr_src_finalize (GObject * object)
{
  GstSdrSrc *sdr = GST_SDR_SRC (object);
  gst_sdr_src_tcp_disconnect (sdr);
#if HAVE_RTLSDR
  gst_sdr_src_usb_close (sdr);
#endif
  g_free (sdr->mode);
  g_free (sdr->host);
  g_free (sdr->recv_buf);
  G_OBJECT_CLASS (gst_sdr_src_parent_class)->finalize (object);
}

static gboolean
gst_sdr_src_start (GstBaseSrc * src)
{
  GstSdrSrc *sdr = GST_SDR_SRC (src);
  gboolean ok;

  if (g_strcmp0 (sdr->mode, "usb") == 0) {
#if HAVE_RTLSDR
    ok = gst_sdr_src_usb_open (sdr);
#else
    GST_ERROR_OBJECT (sdr, "Built without librtlsdr; mode=usb unavailable");
    return FALSE;
#endif
  } else {
    ok = gst_sdr_src_tcp_connect (sdr);
  }

  if (!ok)
    return FALSE;

  /* Pin caps before negotiate() runs from start_complete(). */
  return gst_sdr_src_negotiate (src);
}

static gboolean
gst_sdr_src_stop (GstBaseSrc * src)
{
  GstSdrSrc *sdr = GST_SDR_SRC (src);
  gst_sdr_src_tcp_disconnect (sdr);
#if HAVE_RTLSDR
  gst_sdr_src_usb_close (sdr);
#endif
  return TRUE;
}

/* ---- Interrupting blocking I/O on state change --------------------------
 *
 * gst_sdr_src_create() below blocks in recv()/rtlsdr_read_sync() with no
 * timeout. For a live=TRUE source, GstBaseSrc relies on unlock() to break
 * that blocking call when the pipeline is asked to PAUSE (e.g. Ctrl+C in
 * gst-launch, or a client app tearing down to retune). Without it, the
 * PLAYING->PAUSED transition hangs until data happens to arrive, and the
 * process often ends up force-killed instead of closing the TCP connection
 * cleanly. On a remote rtl_tcp server (e.g. one running on a router) that
 * can leave a half-torn-down "phantom" client holding the tuner at the old
 * frequency for a while, so the *next* gst-launch invocation's SET_FREQ
 * either arrives late or is served stale samples from before the retune -
 * symptom: you ask for frequency X but keep hearing whatever was tuned
 * previously. shutdown() on the fd wakes the blocked recv() immediately
 * with a clean return, letting stop()/tcp_disconnect() close() the socket
 * right away instead of leaving it to linger.
 */

static gboolean
gst_sdr_src_unlock (GstBaseSrc * src)
{
  GstSdrSrc *sdr = GST_SDR_SRC (src);

  sdr->flushing = TRUE;
  if (sdr->sock_fd >= 0)
    shutdown (sdr->sock_fd, SHUT_RDWR);
  return TRUE;
}

static gboolean
gst_sdr_src_unlock_stop (GstBaseSrc * src)
{
  GstSdrSrc *sdr = GST_SDR_SRC (src);

  sdr->flushing = FALSE;
  return TRUE;
}

static GstFlowReturn
gst_sdr_src_create (GstPushSrc * src, GstBuffer ** buf)
{
  GstSdrSrc *sdr = GST_SDR_SRC (src);
  GstMapInfo map;
  gfloat *out;
  gsize n_iq_pairs, i;

  if (!sdr->connected)
    return GST_FLOW_ERROR;

  if (sdr->flushing)
    return GST_FLOW_FLUSHING;

#if HAVE_RTLSDR
  if (sdr->rtl_dev) {
    /* USB path: read a fixed-size chunk of uint8 IQ */
    int n_read;
    gsize want = sdr->recv_buf_size;

    n_read = rtlsdr_read_sync ((rtlsdr_dev_t *) sdr->rtl_dev,
        sdr->recv_buf, (int) want, NULL);
    if (n_read <= 0)
      return sdr->flushing ? GST_FLOW_FLUSHING : GST_FLOW_EOS;

    n_iq_pairs = (gsize) n_read / 2;
    *buf = gst_buffer_new_allocate (NULL, n_iq_pairs * 2 * sizeof (gfloat), NULL);
    gst_buffer_map (*buf, &map, GST_MAP_WRITE);
    out = (gfloat *) map.data;

    for (i = 0; i < n_iq_pairs; i++) {
      out[i * 2 + 0] = ((gfloat) sdr->recv_buf[i * 2 + 0] - 127.5f) / 127.5f;
      out[i * 2 + 1] = ((gfloat) sdr->recv_buf[i * 2 + 1] - 127.5f) / 127.5f;
    }
    gst_sdr_src_auto_gain_update (sdr, out, n_iq_pairs);
    gst_buffer_unmap (*buf, &map);
    return GST_FLOW_OK;
  }
#endif

  /* TCP path */
  {
    gssize n_read;
    guint8 *in;
    gsize total_bytes;

    n_read = recv (sdr->sock_fd, sdr->recv_buf, sdr->recv_buf_size, 0);
    if (n_read <= 0)
      return sdr->flushing ? GST_FLOW_FLUSHING : GST_FLOW_EOS;

    in = sdr->recv_buf;
    total_bytes = (gsize) n_read + (sdr->has_pending_byte ? 1 : 0);
    n_iq_pairs = total_bytes / 2;

    *buf = gst_buffer_new_allocate (NULL, n_iq_pairs * 2 * sizeof (gfloat), NULL);
    gst_buffer_map (*buf, &map, GST_MAP_WRITE);
    out = (gfloat *) map.data;

    for (i = 0; i < n_iq_pairs; i++) {
      guint8 byte_i, byte_q;
      if (sdr->has_pending_byte && i == 0) {
        byte_i = sdr->pending_byte;
        byte_q = in[0];
      } else {
        gsize base = sdr->has_pending_byte ? (i * 2 - 1) : (i * 2);
        byte_i = in[base];
        byte_q = in[base + 1];
      }
      out[i * 2 + 0] = ((gfloat) byte_i - 127.5f) / 127.5f;
      out[i * 2 + 1] = ((gfloat) byte_q - 127.5f) / 127.5f;
    }

    sdr->has_pending_byte = FALSE;
    if (total_bytes % 2 != 0) {
      sdr->pending_byte = in[n_read - 1];
      sdr->has_pending_byte = TRUE;
    }

    gst_sdr_src_auto_gain_update (sdr, out, n_iq_pairs);
    gst_buffer_unmap (*buf, &map);
    return GST_FLOW_OK;
  }
}

static void
gst_sdr_src_set_property (GObject * object, guint prop_id,
    const GValue * value, GParamSpec * pspec)
{
  GstSdrSrc *sdr = GST_SDR_SRC (object);

  switch (prop_id) {
    case PROP_MODE:
      g_free (sdr->mode);
      sdr->mode = g_value_dup_string (value);
      break;
    case PROP_HOST:
      g_free (sdr->host);
      sdr->host = g_value_dup_string (value);
      break;
    case PROP_PORT:
      sdr->port = g_value_get_uint (value);
      break;
    case PROP_FREQUENCY:
      sdr->frequency = g_value_get_uint (value);
      if (sdr->connected && sdr->sock_fd >= 0)
        gst_sdr_src_send_cmd (sdr, 0x01, sdr->frequency);
#if HAVE_RTLSDR
      else if (sdr->rtl_dev)
        rtlsdr_set_center_freq ((rtlsdr_dev_t *) sdr->rtl_dev, sdr->frequency);
#endif
      break;
    case PROP_SAMPLE_RATE:
      sdr->sample_rate = g_value_get_uint (value);
      if (sdr->connected && sdr->sock_fd >= 0)
        gst_sdr_src_send_cmd (sdr, 0x02, sdr->sample_rate);
#if HAVE_RTLSDR
      else if (sdr->rtl_dev)
        rtlsdr_set_sample_rate ((rtlsdr_dev_t *) sdr->rtl_dev, sdr->sample_rate);
#endif
      break;
    case PROP_GAIN:
      sdr->gain = g_value_get_float (value);
      gst_sdr_src_apply_gain (sdr);
      break;
    case PROP_AUTO_GAIN:
      sdr->auto_gain = g_value_get_boolean (value);
      sdr->power_ema_init = FALSE;
      sdr->auto_gain_hold = 0;
      break;
    case PROP_AUTO_GAIN_TARGET:
      sdr->auto_gain_target_db = g_value_get_float (value);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}

static void
gst_sdr_src_get_property (GObject * object, guint prop_id,
    GValue * value, GParamSpec * pspec)
{
  GstSdrSrc *sdr = GST_SDR_SRC (object);

  switch (prop_id) {
    case PROP_MODE:
      g_value_set_string (value, sdr->mode);
      break;
    case PROP_HOST:
      g_value_set_string (value, sdr->host);
      break;
    case PROP_PORT:
      g_value_set_uint (value, sdr->port);
      break;
    case PROP_FREQUENCY:
      g_value_set_uint (value, sdr->frequency);
      break;
    case PROP_SAMPLE_RATE:
      g_value_set_uint (value, sdr->sample_rate);
      break;
    case PROP_GAIN:
      g_value_set_float (value, sdr->gain);
      break;
    case PROP_AUTO_GAIN:
      g_value_set_boolean (value, sdr->auto_gain);
      break;
    case PROP_AUTO_GAIN_TARGET:
      g_value_set_float (value, sdr->auto_gain_target_db);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
      break;
  }
}
