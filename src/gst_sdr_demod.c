/*
 * sdrdemod — AM/FM demodulator for GStreamer
 *
 * FM path follows williamyang98/FM-Radio (Broadcast_FM_Demod):
 *   IQ → [optional NCO] → FIR+decim → FM disc (Δarg) → composite
 *   composite → L+R LPF
 *   composite → pilot PLL @ 19 kHz → ×2 → L−R LPF
 *   L = (L+R)+(L−R), R = (L+R)−(L−R) → de-emphasis → audio
 *
 * AM path: magnitude + DC block.
 */
#include "gst-sdr-plugins.h"

GST_DEBUG_CATEGORY_STATIC (gst_sdr_demod_debug);
#define GST_CAT_DEFAULT gst_sdr_demod_debug

enum {
  PROP_0, PROP_MODE, PROP_STEREO, PROP_MAX_DEVIATION, PROP_AUDIO_RATE,
  PROP_AUDIO_CUTOFF, PROP_TAU, PROP_FREQ_OFFSET, PROP_STEREO_MIX,
  PROP_IF_BANDWIDTH, PROP_AUTO_BANDWIDTH, PROP_LAST
};

#define DEFAULT_MODE GST_SDR_MODE_FM
#define DEFAULT_STEREO FALSE
#define DEFAULT_MAX_DEVIATION 75000.0f
#define DEFAULT_AUDIO_RATE 48000
#define DEFAULT_AUDIO_CUTOFF 15000.0f
#define DEFAULT_TAU 75
#define DEFAULT_FREQ_OFFSET 0.0f
#define DEFAULT_STEREO_MIX 1.0f
#define FIR_TAPS_IF 63
#define FIR_TAPS_AUDIO 63
#define PILOT_FREQ_HZ 19000.0f

/* --if-bandwidth / --auto-bandwidth: IF filter cutoff can now be set
 * explicitly (if-bandwidth, 0 = keep the original automatic formula
 * below) or hunted live for best S/N (auto-bandwidth). Both are opt-in;
 * with both left at their defaults, configure() computes exactly the
 * same cutoff as before this change. */
#define DEFAULT_IF_BANDWIDTH 0.0f      /* 0 = legacy automatic cutoff */
#define DEFAULT_AUTO_BANDWIDTH FALSE
#define AUTO_BW_TARGET_LOW_DB 10.0f    /* narrow further when S/N proxy drops below this */
#define AUTO_BW_TARGET_HIGH_DB 18.0f   /* widen back out once S/N proxy clears this */
#define AUTO_BW_STEP_FRAC 0.06f        /* +/-6% of range per adjustment step */
#define AUTO_BW_HOLD_BUFFERS 40        /* how many transform() calls between adjustments */

static GstStaticPadTemplate sinktemplate = GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK, GST_PAD_ALWAYS,
    GST_STATIC_CAPS ("application/x-raw,format=F32LE,layout=interleaved,"
        "channels=2,rate=[8000,10000000]"));

static GstStaticPadTemplate srctemplate = GST_STATIC_PAD_TEMPLATE ("src",
    GST_PAD_SRC, GST_PAD_ALWAYS,
    GST_STATIC_CAPS ("audio/x-raw,format=F32LE,layout=interleaved,"
        "channels={1,2},rate=[8000,192000]"));

G_DEFINE_TYPE (GstSdrDemod, gst_sdr_demod, GST_TYPE_BASE_TRANSFORM);

static void gst_sdr_demod_set_property (GObject * o, guint id, const GValue * v, GParamSpec * p);
static void gst_sdr_demod_get_property (GObject * o, guint id, GValue * v, GParamSpec * p);
static void gst_sdr_demod_finalize (GObject * o);
static gboolean gst_sdr_demod_set_caps (GstBaseTransform * t, GstCaps * in, GstCaps * out);
static GstFlowReturn gst_sdr_demod_transform (GstBaseTransform * t, GstBuffer * in, GstBuffer * out);
static gboolean gst_sdr_demod_transform_size (GstBaseTransform * t, GstPadDirection dir,
    GstCaps * caps, gsize size, GstCaps * ocaps, gsize * osize);
static GstCaps *gst_sdr_demod_transform_caps (GstBaseTransform * t, GstPadDirection dir,
    GstCaps * caps, GstCaps * filter);

static void
design_lowpass (gfloat * taps, gint n, gdouble cutoff_hz, gdouble fs)
{
  gint m = n - 1, i;
  gdouble fc = cutoff_hz / fs, sum = 0.0;
  for (i = 0; i <= m; i++) {
    gdouble nn = i - m / 2.0;
    gdouble sinc = (nn == 0.0) ? 2.0 * fc : sin (2.0 * M_PI * fc * nn) / (M_PI * nn);
    gdouble w = 0.5 - 0.5 * cos (2.0 * M_PI * i / (gdouble) m);
    taps[i] = (gfloat) (sinc * w);
    sum += taps[i];
  }
  if (sum != 0.0)
    for (i = 0; i <= m; i++)
      taps[i] /= (gfloat) sum;
}

static gint
fir_decimate (const gfloat * taps, gint n_taps, gint factor,
    gfloat * tail, gint * phase, const gfloat * in, gint in_len,
    gfloat * out, gfloat * scratch)
{
  gint i, j, out_len = 0;
  gint need = (n_taps - 1) + in_len;
  memcpy (scratch, tail, (n_taps - 1) * sizeof (gfloat));
  memcpy (scratch + (n_taps - 1), in, in_len * sizeof (gfloat));
  for (i = *phase; i <= in_len - 1; i += factor) {
    gfloat acc = 0.0f;
    for (j = 0; j < n_taps; j++)
      acc += scratch[i + j] * taps[j];
    out[out_len++] = acc;
  }
  {
    gint last = *phase;
    while (last + factor <= in_len - 1)
      last += factor;
    *phase = (last + factor) - in_len;
    if (*phase < 0)
      *phase = 0;
  }
  memcpy (tail, scratch + (need - (n_taps - 1)), (n_taps - 1) * sizeof (gfloat));
  return out_len;
}

static gint
fir_decimate_cplx (const gfloat * taps, gint n_taps, gint factor,
    gfloat * tail_i, gfloat * tail_q, gint * phase,
    const gfloat * in_iq, gint n_iq, gfloat * out_i, gfloat * out_q, gfloat * scratch)
{
  gint i, j, out_len = 0;
  gint tl = n_taps - 1;
  gint need = tl + n_iq;
  gfloat *bi = scratch, *bq = scratch + need;
  memcpy (bi, tail_i, tl * sizeof (gfloat));
  memcpy (bq, tail_q, tl * sizeof (gfloat));
  for (i = 0; i < n_iq; i++) {
    bi[tl + i] = in_iq[i * 2 + 0];
    bq[tl + i] = in_iq[i * 2 + 1];
  }
  for (i = *phase; i <= n_iq - 1; i += factor) {
    gfloat ai = 0.0f, aq = 0.0f;
    for (j = 0; j < n_taps; j++) {
      ai += bi[i + j] * taps[j];
      aq += bq[i + j] * taps[j];
    }
    out_i[out_len] = ai;
    out_q[out_len] = aq;
    out_len++;
  }
  {
    gint last = *phase;
    while (last + factor <= n_iq - 1)
      last += factor;
    *phase = (last + factor) - n_iq;
    if (*phase < 0)
      *phase = 0;
  }
  memcpy (tail_i, bi + (need - tl), tl * sizeof (gfloat));
  memcpy (tail_q, bq + (need - tl), tl * sizeof (gfloat));
  return out_len;
}

static guint
calc_if_decim (guint in_rate, gboolean stereo, gfloat max_deviation_hz)
{
  /* The FM discriminator (fm_disc) runs on the IQ *after* this decimation,
   * on atan2()-based phase deltas. If the IF rate isn't comfortably above
   * the peak instantaneous deviation, delta_theta = 2*pi*dev/Fs exceeds
   * +-pi every cycle and wraps (phase aliasing), which sounds like pure
   * noise/distortion rather than recovered audio -- it is NOT a filter
   * design problem, it's the discriminator itself losing the plot. Keep
   * >=4x max_deviation of headroom (comfortably under the +-pi limit; std
   * broadcast FM peaks near 75kHz, 4x gives real margin), floor'd to the
   * old bandwidth-only minimums so AM (max_deviation_hz==0) and narrowband
   * FM keep their previous behaviour. */
  guint bw_min = stereo ? 200000 : 40000;
  guint min_if = bw_min;
  guint dd;

  if (max_deviation_hz > 0.0f) {
    guint disc_min = (guint) (max_deviation_hz * 4.0f);
    if (disc_min > min_if)
      min_if = disc_min;
  }

  dd = in_rate / min_if;
  if (dd < 1)
    dd = 1;
  while (dd > 1 && (in_rate / dd) < min_if)
    dd--;
  return dd;
}

/* Legacy cutoff = exactly what this element always computed before
 * if-bandwidth/auto-bandwidth existed. min_cutoff = the narrowest we'll
 * ever let the filter go, whether by manual if-bandwidth or by the
 * auto-bandwidth hunt -- narrow enough to meaningfully reject adjacent
 * noise, wide enough that the FM discriminator/pilot PLL still has the
 * signal they need (stereo needs the 38 kHz subcarrier + guard, mono
 * just needs the audio band + guard). */
static void
calc_bw_bounds (GstSdrDemod * d, gdouble * out_min, gdouble * out_legacy)
{
  gdouble legacy, minc;
  if (d->mode == GST_SDR_MODE_FM) {
    legacy = MIN ((gdouble) (d->max_deviation + (d->stereo ? 40000.0f : 12000.0f)),
        0.45 * (gdouble) d->sample_rate);
    minc = d->stereo
        ? MIN ((gdouble) (d->max_deviation * 1.15f + 38000.0f), legacy)
        : MIN ((gdouble) (d->max_deviation * 1.15f + 3000.0f), legacy);
  } else {
    legacy = MIN ((gdouble) d->audio_cutoff, 0.45 * (gdouble) d->sample_rate);
    minc = MIN ((gdouble) d->audio_cutoff * 1.1, legacy);
  }
  if (minc < 1000.0)
    minc = MIN (1000.0, legacy);
  *out_min = minc;
  *out_legacy = legacy;
}

/* Resolves if-bandwidth/auto-bandwidth against the current bounds into
 * one concrete cutoff, and remembers it in bw_current_hz (also serves
 * as the auto-bandwidth hunt's running value and as the readback for
 * the if-bandwidth property). Shared by configure() (full reconfigure,
 * e.g. on caps change) and reconfigure_if_filter() (bandwidth-only live
 * retune, used by the property setters and the auto-bandwidth hunt). */
static gdouble
calc_effective_cutoff (GstSdrDemod * d)
{
  gdouble bw_min, bw_legacy, cutoff;
  calc_bw_bounds (d, &bw_min, &bw_legacy);
  if (d->auto_bandwidth) {
    if (d->bw_current_hz <= 0.0f)
      d->bw_current_hz = (gfloat) bw_legacy;
    cutoff = CLAMP ((gdouble) d->bw_current_hz, bw_min, bw_legacy);
  } else if (d->if_bandwidth_hz > 0.0f) {
    cutoff = CLAMP ((gdouble) d->if_bandwidth_hz, bw_min, bw_legacy);
  } else {
    cutoff = bw_legacy;
  }
  d->bw_current_hz = (gfloat) cutoff;
  return cutoff;
}

/* Bandwidth-only live retune: rebuilds just the IF lowpass taps and
 * resets its tail (a small, bounded transient), without touching NCO
 * phase, pilot PLL, de-emphasis state or decimation factors the way a
 * full configure() would. Used whenever only the cutoff changed. */
static void
reconfigure_if_filter (GstSdrDemod * d)
{
  gdouble cutoff = calc_effective_cutoff (d);
  if (!d->fir_taps || d->n_taps <= 0)
    return;
  design_lowpass (d->fir_taps, d->n_taps, cutoff, (gdouble) d->sample_rate);
  if (d->tail_i && d->tail_len > 0) {
    memset (d->tail_i, 0, d->tail_len * sizeof (gfloat));
    memset (d->tail_q, 0, d->tail_len * sizeof (gfloat));
  }
}

static void
configure (GstSdrDemod * d)
{
  gdouble cutoff;
  guint if_decim;

  if (d->sample_rate < 8000)
    d->sample_rate = 48000;

  if_decim = calc_if_decim (d->sample_rate, d->stereo && d->mode == GST_SDR_MODE_FM,
      d->mode == GST_SDR_MODE_FM ? d->max_deviation : 0.0f);
  d->decim_factor = if_decim;
  d->out_rate = d->sample_rate / if_decim;
  if (d->out_rate < 1)
    d->out_rate = d->sample_rate;

  {
    guint ad = d->out_rate / d->target_audio_rate;
    if (ad < 1)
      ad = 1;
    while (ad > 1 && (d->out_rate / ad) < (guint) (2 * d->audio_cutoff))
      ad--;
    d->audio_decim = ad;
    d->audio_rate = d->out_rate / ad;
  }

  d->n_taps = FIR_TAPS_IF;
  g_free (d->fir_taps);
  d->fir_taps = g_new0 (gfloat, d->n_taps);

  cutoff = calc_effective_cutoff (d);
  design_lowpass (d->fir_taps, d->n_taps, cutoff, (gdouble) d->sample_rate);

  d->n_taps_audio = FIR_TAPS_AUDIO;
  g_free (d->fir_audio);
  d->fir_audio = g_new0 (gfloat, d->n_taps_audio);
  design_lowpass (d->fir_audio, d->n_taps_audio,
      MIN ((gdouble) d->audio_cutoff, 0.45 * (gdouble) d->out_rate),
      (gdouble) d->out_rate);

  g_free (d->tail_i);
  g_free (d->tail_q);
  d->tail_len = d->n_taps - 1;
  d->tail_i = g_new0 (gfloat, d->tail_len);
  d->tail_q = g_new0 (gfloat, d->tail_len);
  d->decim_phase = 0;

  g_free (d->tail_audio_l);
  g_free (d->tail_audio_r);
  d->tail_audio_len = d->n_taps_audio - 1;
  d->tail_audio_l = g_new0 (gfloat, d->tail_audio_len);
  d->tail_audio_r = g_new0 (gfloat, d->tail_audio_len);
  d->audio_phase = 0;
  d->audio_phase_r = 0;

  d->nco_phase = 0.0f;
  d->nco_delta = -2.0f * (gfloat) M_PI * d->freq_offset_hz / (gfloat) d->sample_rate;

  if (d->tau_us > 0 && d->audio_rate > 0) {
    gfloat tau = d->tau_us / 1e6f;
    d->deemph_coeff = 1.0f / (1.0f + tau * (gfloat) d->audio_rate);
  } else {
    d->deemph_coeff = 1.0f;
  }
  d->deemph_prev[0] = d->deemph_prev[1] = 0.0f;
  d->prev_theta = 0.0f;
  d->pilot_phase = 0.0f;
  d->pilot_freq = PILOT_FREQ_HZ;

  GST_INFO_OBJECT (d, "cfg mode=%s stereo=%d IQ=%u IF=%u audio=%u offset=%.0f tau=%u",
      d->mode == GST_SDR_MODE_FM ? "fm" : "am", d->stereo,
      d->sample_rate, d->out_rate, d->audio_rate, d->freq_offset_hz, d->tau_us);
}

/* Yang FM_Demod::Process — Δarg / (2π Fd / Fs) */
static void
fm_disc (GstSdrDemod * d, const gfloat * ii, const gfloat * qq, gint n, gfloat * out)
{
  gfloat Fs = (gfloat) d->out_rate;
  gfloat Wd = d->max_deviation * 2.0f * (gfloat) M_PI;
  /* Yang uses *0.5 so |y| stays in a playable range (without it → hard clip/buzz) */
  gfloat A = (Fs / Wd) * 0.5f;
  gfloat prev = d->prev_theta;
  gint i;
  for (i = 0; i < n; i++) {
    gfloat theta = atan2f (qq[i], ii[i]);
    gfloat delta = theta - prev;
    if (delta > (gfloat) M_PI)
      delta -= 2.0f * (gfloat) M_PI;
    else if (delta < -(gfloat) M_PI)
      delta += 2.0f * (gfloat) M_PI;
    out[i] = delta * A;
    prev = theta;
  }
  d->prev_theta = prev;
}

/* Pilot PLL + L±R mix (Yang LockOntoPilot / ExtractComponents / MixAudio) */
static void
stereo_process (GstSdrDemod * d, const gfloat * composite, gint n, gfloat * L, gfloat * R)
{
  gfloat fs = (gfloat) d->out_rate;
  gfloat two_pi = 2.0f * (gfloat) M_PI;
  gfloat wn = two_pi * 50.0f;
  gfloat alpha = 2.0f * wn / fs;
  gfloat beta = wn * wn / fs;
  gfloat phase = d->pilot_phase;
  gfloat freq = d->pilot_freq;
  gfloat *lpr = d->scratch;
  gfloat *lmr = d->scratch + n;
  gint i;

  for (i = 0; i < n; i++) {
    gfloat x = composite[i];
    gfloat err = x * sinf (phase);
    freq += beta * err;
    if (freq > 21000.0f)
      freq = 21000.0f;
    if (freq < 17000.0f)
      freq = 17000.0f;
    phase += two_pi * freq / fs + alpha * err;
    if (phase > two_pi)
      phase -= two_pi;
    else if (phase < 0.0f)
      phase += two_pi;
    lpr[i] = x;
    lmr[i] = x * (2.0f * cosf (2.0f * phase));
  }
  d->pilot_phase = phase;
  d->pilot_freq = freq;

  {
    /* L's own fir_decimate call aliases out==scratch (both d->scratch2 at
     * offset 0), which is safe: the decimated write index is always <=
     * the current read-window start, so nothing needed later gets
     * clobbered before it's read (classic in-place decimation).
     *
     * R must NOT reuse d->scratch2 as its internal tail++input workspace
     * though: that memcpy unconditionally overwrites scratch2[0..tl+n),
     * which is exactly where L's just-computed output lives (L occupies
     * scratch2[0..nL), and nL is always << tl+n). That clobbered L
     * before the mix loop below ever read it -- L was garbage on every
     * single stereo buffer. d->disc_buf is dead by this point (fm_disc
     * already consumed di/dq into `composite`), so use it as R's
     * private workspace instead of colliding with L's storage. */
    gint nL = fir_decimate (d->fir_audio, d->n_taps_audio, d->audio_decim,
        d->tail_audio_l, &d->audio_phase, lpr, n, L, d->scratch2);
    gint nR = fir_decimate (d->fir_audio, d->n_taps_audio, d->audio_decim,
        d->tail_audio_r, &d->audio_phase_r, lmr, n, R, d->disc_buf);
    gint m = MIN (nL, nR);
    gfloat k = d->stereo_mix;
    for (i = 0; i < m; i++) {
      gfloat a = L[i], b = R[i];
      L[i] = a + k * b;
      R[i] = a - k * b;
    }
    d->last_audio_len = m;
  }
}

static void
apply_deemph (GstSdrDemod * d, gfloat * x, gint n, gint ch)
{
  gfloat a = d->deemph_coeff, y = d->deemph_prev[ch];
  gint i;
  if (a >= 0.999f)
    return;
  for (i = 0; i < n; i++) {
    y = a * x[i] + (1.0f - a) * y;
    x[i] = y;
  }
  d->deemph_prev[ch] = y;
}

static void
gst_sdr_demod_class_init (GstSdrDemodClass * klass)
{
  GObjectClass *go = G_OBJECT_CLASS (klass);
  GstBaseTransformClass *bt = GST_BASE_TRANSFORM_CLASS (klass);
  GstElementClass *el = GST_ELEMENT_CLASS (klass);

  GST_DEBUG_CATEGORY_INIT (gst_sdr_demod_debug, "sdrdemod", 0, "SDR AM/FM Demod");

  gst_element_class_set_static_metadata (el,
      "SDR AM/FM Demodulator", "Filter/Audio",
      "Demodulate IQ to mono/stereo audio (FM pilot PLL stereo, AM envelope)",
      "Tomasz / algorithm from williamyang98/FM-Radio");

  gst_element_class_add_static_pad_template (el, &sinktemplate);
  gst_element_class_add_static_pad_template (el, &srctemplate);

  go->set_property = gst_sdr_demod_set_property;
  go->get_property = gst_sdr_demod_get_property;
  go->finalize = gst_sdr_demod_finalize;

  g_object_class_install_property (go, PROP_MODE,
      g_param_spec_string ("mode", "Mode", "\"fm\" or \"am\"", "fm",
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));
  g_object_class_install_property (go, PROP_STEREO,
      g_param_spec_boolean ("stereo", "Stereo", "FM stereo via pilot PLL",
          DEFAULT_STEREO, G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));
  g_object_class_install_property (go, PROP_MAX_DEVIATION,
      g_param_spec_float ("max-deviation", "Max Deviation", "FM deviation Hz",
          1000.0f, 200000.0f, DEFAULT_MAX_DEVIATION,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));
  g_object_class_install_property (go, PROP_AUDIO_RATE,
      g_param_spec_uint ("audio-rate", "Audio Rate", "Target audio rate",
          8000, 192000, DEFAULT_AUDIO_RATE,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));
  g_object_class_install_property (go, PROP_AUDIO_CUTOFF,
      g_param_spec_float ("audio-cutoff", "Audio Cutoff", "Audio LPF Hz",
          1000.0f, 20000.0f, DEFAULT_AUDIO_CUTOFF,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));
  g_object_class_install_property (go, PROP_TAU,
      g_param_spec_uint ("tau", "De-emphasis τ", "µs (0=off)", 0, 1000, DEFAULT_TAU,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));
  g_object_class_install_property (go, PROP_FREQ_OFFSET,
      g_param_spec_float ("freq-offset", "Freq Offset", "NCO shift Hz",
          -500000.0f, 500000.0f, DEFAULT_FREQ_OFFSET,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));
  g_object_class_install_property (go, PROP_STEREO_MIX,
      g_param_spec_float ("stereo-mix", "Stereo Mix", "L-R factor 0..1",
          0.0f, 1.0f, DEFAULT_STEREO_MIX,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));
  g_object_class_install_property (go, PROP_IF_BANDWIDTH,
      g_param_spec_float ("if-bandwidth", "IF Bandwidth",
          "Manual IF filter cutoff in Hz (0 = automatic, same as before "
          "this property existed). Narrower = less adjacent noise, "
          "weaker sensitivity to strong deviation peaks.",
          0.0f, 200000.0f, DEFAULT_IF_BANDWIDTH,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));
  g_object_class_install_property (go, PROP_AUTO_BANDWIDTH,
      g_param_spec_boolean ("auto-bandwidth", "Auto Bandwidth",
          "Continuously hunt the narrowest IF bandwidth that still gives "
          "good S/N on the current frequency (FM only; ignored for AM). "
          "Overrides if-bandwidth while enabled.",
          DEFAULT_AUTO_BANDWIDTH, G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  bt->set_caps = gst_sdr_demod_set_caps;
  bt->transform = gst_sdr_demod_transform;
  bt->transform_size = gst_sdr_demod_transform_size;
  bt->transform_caps = gst_sdr_demod_transform_caps;
  bt->passthrough_on_same_caps = FALSE;
}

static void
gst_sdr_demod_init (GstSdrDemod * d)
{
  d->mode = DEFAULT_MODE;
  d->stereo = DEFAULT_STEREO;
  d->max_deviation = DEFAULT_MAX_DEVIATION;
  d->target_audio_rate = DEFAULT_AUDIO_RATE;
  d->audio_cutoff = DEFAULT_AUDIO_CUTOFF;
  d->tau_us = DEFAULT_TAU;
  d->freq_offset_hz = DEFAULT_FREQ_OFFSET;
  d->stereo_mix = DEFAULT_STEREO_MIX;
  d->sample_rate = d->out_rate = d->audio_rate = 48000;
  d->decim_factor = d->audio_decim = 1;
  d->fir_taps = d->fir_audio = NULL;
  d->n_taps = d->n_taps_audio = 0;
  d->tail_i = d->tail_q = d->tail_audio_l = d->tail_audio_r = NULL;
  d->tail_len = d->tail_audio_len = 0;
  d->decim_phase = d->audio_phase = d->audio_phase_r = 0;
  d->scratch = d->scratch2 = d->disc_buf = NULL;
  d->scratch_cap = d->scratch2_cap = d->disc_buf_cap = 0;
  d->last_audio_len = 0;
  d->deemph_coeff = 1.0f;
  d->deemph_prev[0] = d->deemph_prev[1] = 0.0f;
  d->prev_theta = 0.0f;
  d->nco_phase = d->nco_delta = 0.0f;
  d->pilot_phase = 0.0f;
  d->pilot_freq = PILOT_FREQ_HZ;

  g_mutex_init (&d->cfg_lock);
  d->if_bandwidth_hz = DEFAULT_IF_BANDWIDTH;
  d->bw_current_hz = 0.0f;
  d->auto_bandwidth = DEFAULT_AUTO_BANDWIDTH;
  d->snr_lowband_pow = 0.0f;
  d->snr_highband_pow = 0.0f;
  d->snr_lpf_state = 0.0f;
  d->snr_db_ema = AUTO_BW_TARGET_HIGH_DB;
  d->auto_bw_hold = 0;
  d->auto_bw_dir = 0;
}

static void
gst_sdr_demod_finalize (GObject * o)
{
  GstSdrDemod *d = GST_SDR_DEMOD (o);
  g_mutex_clear (&d->cfg_lock);
  g_free (d->fir_taps);
  g_free (d->fir_audio);
  g_free (d->tail_i);
  g_free (d->tail_q);
  g_free (d->tail_audio_l);
  g_free (d->tail_audio_r);
  g_free (d->scratch);
  g_free (d->scratch2);
  g_free (d->disc_buf);
  G_OBJECT_CLASS (gst_sdr_demod_parent_class)->finalize (o);
}

static gboolean
gst_sdr_demod_set_caps (GstBaseTransform * t, GstCaps * incaps, GstCaps * outcaps)
{
  GstSdrDemod *d = GST_SDR_DEMOD (t);
  gint rate = 0;
  GstStructure *s = gst_caps_get_structure (incaps, 0);
  if (!gst_structure_get_int (s, "rate", &rate) || rate <= 0)
    return FALSE;
  d->sample_rate = (guint) rate;
  g_mutex_lock (&d->cfg_lock);
  configure (d);
  g_mutex_unlock (&d->cfg_lock);
  return TRUE;
}

static gboolean
gst_sdr_demod_transform_size (GstBaseTransform * t, GstPadDirection dir,
    GstCaps * caps, gsize size, GstCaps * ocaps, gsize * osize)
{
  GstSdrDemod *d = GST_SDR_DEMOD (t);
  guint decim = d->decim_factor * d->audio_decim;
  gint ch = (d->stereo && d->mode == GST_SDR_MODE_FM) ? 2 : 1;
  if (decim < 1)
    decim = 1;
  (void) caps;
  (void) ocaps;
  if (dir == GST_PAD_SINK) {
    gsize n_iq = size / (2 * sizeof (gfloat));
    *osize = (n_iq / decim + 16) * ch * sizeof (gfloat);
  } else {
    gsize n = size / (ch * sizeof (gfloat));
    *osize = (n * decim + 64) * 2 * sizeof (gfloat);
  }
  return TRUE;
}

static GstCaps *
gst_sdr_demod_transform_caps (GstBaseTransform * t, GstPadDirection dir,
    GstCaps * caps, GstCaps * filter)
{
  GstSdrDemod *d = GST_SDR_DEMOD (t);
  GstCaps *ret = gst_caps_new_empty ();
  guint i;
  gint ch = (d->stereo && d->mode == GST_SDR_MODE_FM) ? 2 : 1;

  for (i = 0; i < gst_caps_get_size (caps); i++) {
    GstStructure *s = gst_caps_get_structure (caps, i);
    GstStructure *ns;
    const GValue *rv = gst_structure_get_value (s, "rate");
    const GValue *fv = gst_structure_get_value (s, "format");
    const GValue *lv = gst_structure_get_value (s, "layout");

    if (dir == GST_PAD_SINK) {
      ns = gst_structure_new_empty ("audio/x-raw");
      gst_structure_set (ns, "channels", G_TYPE_INT, ch, NULL);
      if (rv && G_VALUE_HOLDS_INT (rv)) {
        gint in_rate = g_value_get_int (rv);
        guint ifd = calc_if_decim ((guint) in_rate, d->stereo && d->mode == GST_SDR_MODE_FM,
            d->mode == GST_SDR_MODE_FM ? d->max_deviation : 0.0f);
        guint if_rate = in_rate / ifd;
        guint ad = if_rate / d->target_audio_rate;
        if (ad < 1)
          ad = 1;
        gst_structure_set (ns, "rate", G_TYPE_INT, (gint) (if_rate / ad), NULL);
      } else if (rv) {
        gst_structure_set_value (ns, "rate", rv);
      }
    } else {
      ns = gst_structure_new_empty ("application/x-raw");
      gst_structure_set (ns, "channels", G_TYPE_INT, 2, NULL);
    }
    if (fv)
      gst_structure_set_value (ns, "format", fv);
    if (lv)
      gst_structure_set_value (ns, "layout", lv);
    ret = gst_caps_merge_structure (ret, ns);
  }
  if (filter) {
    GstCaps *ix = gst_caps_intersect_full (filter, ret, GST_CAPS_INTERSECT_FIRST);
    gst_caps_unref (ret);
    ret = ix;
  }
  return ret;
}

/* S/N proxy for the auto-bandwidth hunt: split the FM composite signal
 * (post-discriminator, pre audio-filter) into an in-band part via a
 * one-pole LPF at roughly the audio/pilot edge, and a residual "out of
 * band" part treated as a noise proxy. This is not a calibrated SNR
 * measurement -- it's a cheap, monotonic-enough proxy: as adjacent-
 * channel/thermal noise increases, out-of-band energy rises faster than
 * in-band energy, so the ratio still tracks "is narrowing helping".
 * Runs every buffer (cheap, O(n)); the actual bandwidth adjustment is
 * rate-limited by AUTO_BW_HOLD_BUFFERS so it converges slowly instead
 * of hunting every buffer. Caller already holds d->cfg_lock. */
static void
demod_snr_measure_and_hunt (GstSdrDemod * d, const gfloat * composite, gint n)
{
  gfloat fs, band_hz, rc, alpha, lp;
  gdouble low_sum = 0.0, high_sum = 0.0;
  gint i;

  if (n <= 0 || d->mode != GST_SDR_MODE_FM || d->out_rate == 0)
    return;

  fs = (gfloat) d->out_rate;
  band_hz = d->stereo ? 53000.0f : d->audio_cutoff;
  if (band_hz > 0.45f * fs)
    band_hz = 0.45f * fs;
  rc = 1.0f / (2.0f * (gfloat) M_PI * band_hz);
  alpha = (1.0f / fs) / (rc + 1.0f / fs);
  lp = d->snr_lpf_state;

  for (i = 0; i < n; i++) {
    gfloat x = composite[i];
    gfloat hf;
    lp += alpha * (x - lp);
    hf = x - lp;
    low_sum += (gdouble) (lp * lp);
    high_sum += (gdouble) (hf * hf);
  }
  d->snr_lpf_state = lp;

  {
    gfloat low_pow = (gfloat) (low_sum / n);
    gfloat high_pow = (gfloat) (high_sum / n);
    gfloat snr_db;
    d->snr_lowband_pow = 0.95f * d->snr_lowband_pow + 0.05f * low_pow;
    d->snr_highband_pow = 0.95f * d->snr_highband_pow + 0.05f * high_pow;
    snr_db = 10.0f * log10f ((d->snr_lowband_pow + 1e-12f) / (d->snr_highband_pow + 1e-9f));
    d->snr_db_ema = 0.8f * d->snr_db_ema + 0.2f * snr_db;
  }

  if (!d->auto_bandwidth)
    return;
  if (++d->auto_bw_hold < AUTO_BW_HOLD_BUFFERS)
    return;
  d->auto_bw_hold = 0;

  {
    gdouble bw_min, bw_legacy;
    gfloat range, step, cur, next;
    calc_bw_bounds (d, &bw_min, &bw_legacy);
    range = (gfloat) (bw_legacy - bw_min);
    if (range <= 0.0f)
      return;
    step = range * AUTO_BW_STEP_FRAC;
    cur = d->bw_current_hz > 0.0f ? d->bw_current_hz : (gfloat) bw_legacy;
    next = cur;

    if (d->snr_db_ema < AUTO_BW_TARGET_LOW_DB && cur > (gfloat) bw_min + 1.0f) {
      next = cur - step;
      if (next < (gfloat) bw_min)
        next = (gfloat) bw_min;
      d->auto_bw_dir = -1;
    } else if (d->snr_db_ema > AUTO_BW_TARGET_HIGH_DB && cur < (gfloat) bw_legacy - 1.0f) {
      next = cur + step;
      if (next > (gfloat) bw_legacy)
        next = (gfloat) bw_legacy;
      d->auto_bw_dir = 1;
    } else {
      d->auto_bw_dir = 0;
    }

    if (fabsf (next - cur) > 1.0f) {
      d->bw_current_hz = next;
      reconfigure_if_filter (d);
      GST_DEBUG_OBJECT (d, "auto-bandwidth: snr=%.1fdB bw %.0f -> %.0f Hz",
          d->snr_db_ema, cur, next);
    }
  }
}

static GstFlowReturn
gst_sdr_demod_transform (GstBaseTransform * t, GstBuffer * inbuf, GstBuffer * outbuf)
{
  GstSdrDemod *d = GST_SDR_DEMOD (t);
  GstMapInfo inmap, outmap;
  gfloat *in, *out;
  gint n_iq, i, n_if, n_aud = 0;
  gint ch = (d->stereo && d->mode == GST_SDR_MODE_FM) ? 2 : 1;

  if (!gst_buffer_map (inbuf, &inmap, GST_MAP_READ) ||
      !gst_buffer_map (outbuf, &outmap, GST_MAP_WRITE))
    return GST_FLOW_ERROR;

  in = (gfloat *) inmap.data;
  out = (gfloat *) outmap.data;
  n_iq = (gint) (inmap.size / (2 * sizeof (gfloat)));

  g_mutex_lock (&d->cfg_lock);

  {
    gsize need = (gsize) n_iq * 4 + 256;
    if (need > d->scratch_cap) {
      d->scratch = g_realloc (d->scratch, need * sizeof (gfloat));
      d->scratch_cap = need;
    }
    if (need > d->scratch2_cap) {
      d->scratch2 = g_realloc (d->scratch2, need * sizeof (gfloat));
      d->scratch2_cap = need;
    }
    if ((gsize) n_iq > d->disc_buf_cap) {
      /* di/dq each need up to (n_iq/decim_factor + 8) floats, and the
       * worst case is decim_factor == 1 (no IF decimation -- happens
       * whenever the input rate isn't comfortably above the computed
       * minimum IF rate, e.g. stereo FM at a 250 kHz IQ rate), where
       * each channel needs (n_iq + 8). Undersizing this by even the
       * fixed "+8" padding corrupts the heap (di/dq write straight
       * past the allocation into adjacent chunks), so size for that
       * worst case rather than the common (decimated) one. */
      gsize cap = (gsize) n_iq + 8;
      d->disc_buf = g_realloc (d->disc_buf, cap * 2 * sizeof (gfloat));
      d->disc_buf_cap = n_iq;
    }
  }

  /* NCO */
  {
    gfloat phase = d->nco_phase, delta = d->nco_delta;
    for (i = 0; i < n_iq; i++) {
      gfloat c = cosf (phase), s = sinf (phase);
      gfloat ii = in[i * 2], qq = in[i * 2 + 1];
      d->scratch[i * 2] = ii * c - qq * s;
      d->scratch[i * 2 + 1] = ii * s + qq * c;
      phase += delta;
      if (phase > (gfloat) M_PI)
        phase -= 2.0f * (gfloat) M_PI;
      else if (phase < -(gfloat) M_PI)
        phase += 2.0f * (gfloat) M_PI;
    }
    d->nco_phase = phase;
  }

  {
    gfloat *di = d->disc_buf;
    gfloat *dq = d->disc_buf + (n_iq / d->decim_factor + 8);
    n_if = fir_decimate_cplx (d->fir_taps, d->n_taps, d->decim_factor,
        d->tail_i, d->tail_q, &d->decim_phase,
        d->scratch, n_iq, di, dq, d->scratch2);

    if (d->mode == GST_SDR_MODE_FM) {
      gfloat *composite = d->scratch;
      fm_disc (d, di, dq, n_if, composite);
      demod_snr_measure_and_hunt (d, composite, n_if);

      if (d->stereo) {
        gfloat *L = d->scratch2;
        gfloat *R = d->scratch2 + (n_if / d->audio_decim + 8);
        stereo_process (d, composite, n_if, L, R);
        n_aud = d->last_audio_len;
        apply_deemph (d, L, n_aud, 0);
        apply_deemph (d, R, n_aud, 1);
        for (i = 0; i < n_aud; i++) {
          out[i * 2] = L[i];
          out[i * 2 + 1] = R[i];
        }
      } else {
        n_aud = fir_decimate (d->fir_audio, d->n_taps_audio, d->audio_decim,
            d->tail_audio_l, &d->audio_phase, composite, n_if, out, d->scratch2);
        apply_deemph (d, out, n_aud, 0);
      }
    } else {
      gfloat *env = d->scratch;
      for (i = 0; i < n_if; i++)
        env[i] = sqrtf (di[i] * di[i] + dq[i] * dq[i]);
      n_aud = fir_decimate (d->fir_audio, d->n_taps_audio, d->audio_decim,
          d->tail_audio_l, &d->audio_phase, env, n_if, out, d->scratch2);
      {
        gfloat a = expf (-2.0f * (gfloat) M_PI * 20.0f / (gfloat) d->audio_rate);
        gfloat px = d->deemph_prev[0], py = d->deemph_prev[1];
        for (i = 0; i < n_aud; i++) {
          gfloat x = out[i];
          gfloat y = x - px + a * py;
          out[i] = y;
          px = x;
          py = y;
        }
        d->deemph_prev[0] = px;
        d->deemph_prev[1] = py;
      }
    }
  }

  {
    gint nn = n_aud * ch;
    for (i = 0; i < nn; i++) {
      if (out[i] > 1.0f)
        out[i] = 1.0f;
      else if (out[i] < -1.0f)
        out[i] = -1.0f;
    }
  }

  g_mutex_unlock (&d->cfg_lock);

  gst_buffer_unmap (inbuf, &inmap);
  gst_buffer_unmap (outbuf, &outmap);
  gst_buffer_set_size (outbuf, n_aud * ch * sizeof (gfloat));
  return GST_FLOW_OK;
}

static void
gst_sdr_demod_set_property (GObject * o, guint id, const GValue * v, GParamSpec * p)
{
  GstSdrDemod *d = GST_SDR_DEMOD (o);
  switch (id) {
    case PROP_MODE:
      d->mode = (g_strcmp0 (g_value_get_string (v), "am") == 0) ? GST_SDR_MODE_AM : GST_SDR_MODE_FM;
      break;
    case PROP_STEREO:
      d->stereo = g_value_get_boolean (v);
      break;
    case PROP_MAX_DEVIATION:
      d->max_deviation = g_value_get_float (v);
      break;
    case PROP_AUDIO_RATE:
      d->target_audio_rate = g_value_get_uint (v);
      break;
    case PROP_AUDIO_CUTOFF:
      d->audio_cutoff = g_value_get_float (v);
      break;
    case PROP_TAU:
      d->tau_us = g_value_get_uint (v);
      break;
    case PROP_FREQ_OFFSET:
      d->freq_offset_hz = g_value_get_float (v);
      break;
    case PROP_STEREO_MIX:
      d->stereo_mix = g_value_get_float (v);
      break;
    case PROP_IF_BANDWIDTH:
      g_mutex_lock (&d->cfg_lock);
      d->if_bandwidth_hz = g_value_get_float (v);
      if (!d->auto_bandwidth)
        reconfigure_if_filter (d);
      g_mutex_unlock (&d->cfg_lock);
      break;
    case PROP_AUTO_BANDWIDTH:
      g_mutex_lock (&d->cfg_lock);
      d->auto_bandwidth = g_value_get_boolean (v);
      d->auto_bw_hold = 0;
      d->auto_bw_dir = 0;
      d->snr_db_ema = AUTO_BW_TARGET_HIGH_DB;
      if (!d->auto_bandwidth)
        d->bw_current_hz = 0.0f;      /* fall back to legacy/manual cutoff */
      reconfigure_if_filter (d);
      g_mutex_unlock (&d->cfg_lock);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (o, id, p);
  }
}

static void
gst_sdr_demod_get_property (GObject * o, guint id, GValue * v, GParamSpec * p)
{
  GstSdrDemod *d = GST_SDR_DEMOD (o);
  switch (id) {
    case PROP_MODE:
      g_value_set_string (v, d->mode == GST_SDR_MODE_AM ? "am" : "fm");
      break;
    case PROP_STEREO:
      g_value_set_boolean (v, d->stereo);
      break;
    case PROP_MAX_DEVIATION:
      g_value_set_float (v, d->max_deviation);
      break;
    case PROP_AUDIO_RATE:
      g_value_set_uint (v, d->target_audio_rate);
      break;
    case PROP_AUDIO_CUTOFF:
      g_value_set_float (v, d->audio_cutoff);
      break;
    case PROP_TAU:
      g_value_set_uint (v, d->tau_us);
      break;
    case PROP_FREQ_OFFSET:
      g_value_set_float (v, d->freq_offset_hz);
      break;
    case PROP_STEREO_MIX:
      g_value_set_float (v, d->stereo_mix);
      break;
    case PROP_IF_BANDWIDTH:
      /* Reads back the *effective* cutoff (bw_current_hz) once configured,
       * so a GUI can show what auto-bandwidth actually landed on. Before
       * the first configure() it reports the raw requested value. */
      g_value_set_float (v, d->bw_current_hz > 0.0f ? d->bw_current_hz : d->if_bandwidth_hz);
      break;
    case PROP_AUTO_BANDWIDTH:
      g_value_set_boolean (v, d->auto_bandwidth);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (o, id, p);
  }
}
