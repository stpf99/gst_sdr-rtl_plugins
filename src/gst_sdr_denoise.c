/*
 * sdrdenoise — anomaly-based spectral noise reduction for demodulated audio.
 *
 * Direct port of the reference Python RecursiveNoiseFloorEstimator
 * (sdr/fm_demod.py): asymmetric-hysteresis noise-floor tracking per FFT
 * bin, suppressing only bins that spike above the tracked floor rather
 * than attacking the whole signal. 2048-point FFT, 512-sample hop (75%
 * overlap), Hann window applied both before the forward FFT and after
 * the inverse FFT (Hann-squared OLA).
 *
 * Runs downstream of sdrdemod, on the already-decimated audio. Because
 * the STFT needs whole 512-sample hops to produce a block, this element
 * buffers a little internally (adds ~FFT_SIZE/rate of latency, e.g.
 * ~43ms at 48kHz) and may emit fewer samples than it was handed on any
 * given buffer -- the shortfall is made up on the next one. That's why
 * transform_size only gives an upper bound and transform() trims the
 * output buffer to whatever was actually ready, same pattern used in
 * sdrdemod's decimators.
 */
#include "gst-sdr-plugins.h"

GST_DEBUG_CATEGORY_STATIC (gst_sdr_denoise_debug);
#define GST_CAT_DEFAULT gst_sdr_denoise_debug

enum {
  PROP_0, PROP_ENABLED, PROP_THRESHOLD, PROP_ALPHA_UP, PROP_ALPHA_DOWN,
  PROP_INTERPOLATE, PROP_AUTO_INTERPOLATE, PROP_INTERP_STRENGTH, PROP_LAST
};

#define DEFAULT_ENABLED TRUE
#define DEFAULT_THRESHOLD 8.0f
#define DEFAULT_ALPHA_UP 0.01f
#define DEFAULT_ALPHA_DOWN 0.0001f
/* --interpolate / --auto-interpolate: a light adaptive smoothing pass
 * (one-pole lowpass) applied after denoising. Off by default -- with
 * both left at their defaults this element behaves exactly as before. */
#define DEFAULT_INTERPOLATE FALSE
#define DEFAULT_AUTO_INTERPOLATE FALSE
#define DEFAULT_INTERP_STRENGTH 0.5f

static GstStaticPadTemplate sinktemplate = GST_STATIC_PAD_TEMPLATE ("sink",
    GST_PAD_SINK, GST_PAD_ALWAYS,
    GST_STATIC_CAPS ("audio/x-raw,format=F32LE,layout=interleaved,"
        "channels={1,2},rate=[8000,192000]"));

static GstStaticPadTemplate srctemplate = GST_STATIC_PAD_TEMPLATE ("src",
    GST_PAD_SRC, GST_PAD_ALWAYS,
    GST_STATIC_CAPS ("audio/x-raw,format=F32LE,layout=interleaved,"
        "channels={1,2},rate=[8000,192000]"));

G_DEFINE_TYPE (GstSdrDenoise, gst_sdr_denoise, GST_TYPE_BASE_TRANSFORM);

static void gst_sdr_denoise_set_property (GObject * o, guint id, const GValue * v, GParamSpec * p);
static void gst_sdr_denoise_get_property (GObject * o, guint id, GValue * v, GParamSpec * p);
static void gst_sdr_denoise_finalize (GObject * o);
static gboolean gst_sdr_denoise_set_caps (GstBaseTransform * t, GstCaps * in, GstCaps * out);
static GstFlowReturn gst_sdr_denoise_transform (GstBaseTransform * t, GstBuffer * in, GstBuffer * out);
static gboolean gst_sdr_denoise_transform_size (GstBaseTransform * t, GstPadDirection dir,
    GstCaps * caps, gsize size, GstCaps * ocaps, gsize * osize);

/* ---- tiny in-place radix-2 complex FFT (n must be a power of 2) ------ */

static void
fft_bit_reverse (gfloat * re, gfloat * im, gint n)
{
  gint i, j = 0, k;
  for (i = 0; i < n - 1; i++) {
    if (i < j) {
      gfloat t;
      t = re[i]; re[i] = re[j]; re[j] = t;
      t = im[i]; im[i] = im[j]; im[j] = t;
    }
    k = n >> 1;
    while (k <= j) { j -= k; k >>= 1; }
    j += k;
  }
}

static void
fft_radix2 (gfloat * re, gfloat * im, gint n, gboolean invert)
{
  gint len;
  fft_bit_reverse (re, im, n);
  for (len = 2; len <= n; len <<= 1) {
    gdouble ang = 2.0 * M_PI / len * (invert ? 1.0 : -1.0);
    gfloat wr = (gfloat) cos (ang), wi = (gfloat) sin (ang);
    gint i;
    for (i = 0; i < n; i += len) {
      gfloat cwr = 1.0f, cwi = 0.0f;
      gint k, half = len / 2;
      for (k = 0; k < half; k++) {
        gfloat ur = re[i + k], ui = im[i + k];
        gfloat vr = re[i + k + half] * cwr - im[i + k + half] * cwi;
        gfloat vi = re[i + k + half] * cwi + im[i + k + half] * cwr;
        gfloat nwr, nwi;
        re[i + k] = ur + vr;
        im[i + k] = ui + vi;
        re[i + k + half] = ur - vr;
        im[i + k + half] = ui - vi;
        nwr = cwr * wr - cwi * wi;
        nwi = cwr * wi + cwi * wr;
        cwr = nwr; cwi = nwi;
      }
    }
  }
  if (invert) {
    gint i;
    for (i = 0; i < n; i++) { re[i] /= n; im[i] /= n; }
  }
}

/* ---- per-channel state ------------------------------------------------ */

static void
denoise_alloc (GstSdrDenoise * d)
{
  gint ch, i;

  if (d->allocated)
    return;

  d->hann = g_new0 (gfloat, SDR_DENOISE_FFT);
  for (i = 0; i < SDR_DENOISE_FFT; i++)
    d->hann[i] = 0.5f - 0.5f * cosf (2.0f * (gfloat) M_PI * i / (SDR_DENOISE_FFT - 1));

  for (ch = 0; ch < 2; ch++) {
    d->hist[ch] = g_new0 (gfloat, SDR_DENOISE_FFT);
    d->acc[ch] = g_new0 (gfloat, SDR_DENOISE_FFT);
    d->pending[ch] = g_new0 (gfloat, SDR_DENOISE_HOP);
    d->pending_n[ch] = 0;
    d->noise_floor[ch] = g_new (gfloat, SDR_DENOISE_NFREQ);
    for (i = 0; i < SDR_DENOISE_NFREQ; i++)
      d->noise_floor[ch][i] = 1e-6f;
      
    /* Alokacja bufora wyjściowego z bezpieczną pojemnością początkową */
    d->out_queue_cap[ch] = SDR_DENOISE_FFT * 2;
    d->out_queue[ch] = g_new0 (gfloat, d->out_queue_cap[ch]);
    d->out_queue_n[ch] = 0;
  }
  d->allocated = TRUE;
}

/* Runs one FFT/IFFT hop for channel ch: hist[] must already hold the
 * current FFT_SIZE-sample analysis window (newest HOP samples at the
 * tail). Adds the filtered, re-windowed hop into acc[] but does not
 * shift/emit -- caller does that, since hist/acc bookkeeping is shared
 * with the input-buffering logic in gst_sdr_denoise_transform(). */
static void
denoise_run_hop (GstSdrDenoise * d, gint ch)
{
  gfloat re[SDR_DENOISE_FFT], im[SDR_DENOISE_FFT];
  gfloat mag[SDR_DENOISE_NFREQ], phase[SDR_DENOISE_NFREQ];
  gfloat *hist = d->hist[ch];
  gfloat *acc = d->acc[ch];
  gfloat *nf = d->noise_floor[ch];
  gfloat threshold = d->threshold_db;
  gint i;

  for (i = 0; i < SDR_DENOISE_FFT; i++) {
    re[i] = hist[i] * d->hann[i];
    im[i] = 0.0f;
  }
  fft_radix2 (re, im, SDR_DENOISE_FFT, FALSE);

  for (i = 0; i < SDR_DENOISE_NFREQ; i++) {
    mag[i] = sqrtf (re[i] * re[i] + im[i] * im[i]);
    phase[i] = atan2f (im[i], re[i]);
  }

  /* Asymmetric noise-floor tracking: rise slowly (alpha_up), fall to a
   * dip almost instantly (alpha_down) -- mirrors the Python estimator.
   * (Python also runs a size-5 median filter across frequency on the
   * dB floor each update; omitted here to keep the port tractable --
   * revisit if the bin-to-bin floor proves too jittery in practice.) */
  for (i = 0; i < SDR_DENOISE_NFREQ; i++) {
    if (mag[i] < nf[i])
      nf[i] = (1.0f - d->alpha_down) * nf[i] + d->alpha_down * mag[i];
    else
      nf[i] = (1.0f - d->alpha_up) * nf[i] + d->alpha_up * mag[i];
  }

  for (i = 0; i < SDR_DENOISE_NFREQ; i++) {
    gfloat ratio_db = 10.0f * log10f (mag[i] / (nf[i] + 1e-10f) + 1e-10f);
    if (ratio_db > threshold) {
      gfloat strength = (ratio_db - threshold) / 20.0f;
      gfloat suppression = 1.0f - strength;
      if (suppression < 0.0f)
        suppression = 0.0f;
      mag[i] *= suppression;
    }
  }

  for (i = 0; i < SDR_DENOISE_NFREQ; i++) {
    re[i] = mag[i] * cosf (phase[i]);
    im[i] = mag[i] * sinf (phase[i]);
  }
  /* Rebuild the negative-frequency half via conjugate symmetry (real input). */
  for (i = 1; i < SDR_DENOISE_FFT - SDR_DENOISE_NFREQ + 1; i++) {
    re[SDR_DENOISE_FFT - i] = re[i];
    im[SDR_DENOISE_FFT - i] = -im[i];
  }

  fft_radix2 (re, im, SDR_DENOISE_FFT, TRUE);

  for (i = 0; i < SDR_DENOISE_FFT; i++)
    acc[i] += re[i] * d->hann[i];
}

/* Feeds `n` new samples for channel ch through the hop-buffered OLA
 * pipeline, writing however many finalized output samples are ready
 * into out (caller-sized for the worst case: n). Returns the count
 * actually written. */
static gint
denoise_process_channel (GstSdrDenoise * d, gint ch, const gfloat * in, gint n, gfloat * out)
{
  gint i, out_n = 0;
  gfloat *pend = d->pending[ch];

  if (!d->enabled) {
    memcpy (out, in, n * sizeof (gfloat));
    return n;
  }

  for (i = 0; i < n; i++) {
    pend[d->pending_n[ch]++] = in[i];
    if (d->pending_n[ch] < SDR_DENOISE_HOP)
      continue;

    /* Hop complete: slide the analysis window, run the FFT/OLA hop,
     * emit the now-finalized front of the accumulator, then slide the
     * accumulator itself by one hop. */
    memmove (d->hist[ch], d->hist[ch] + SDR_DENOISE_HOP,
        (SDR_DENOISE_FFT - SDR_DENOISE_HOP) * sizeof (gfloat));
    memcpy (d->hist[ch] + (SDR_DENOISE_FFT - SDR_DENOISE_HOP), pend,
        SDR_DENOISE_HOP * sizeof (gfloat));

    denoise_run_hop (d, ch);

    /* Zapewnienie odpowiedniej pojemności kolejki wyjściowej */
    if (d->out_queue_n[ch] + SDR_DENOISE_HOP > d->out_queue_cap[ch]) {
      d->out_queue_cap[ch] = d->out_queue_n[ch] + SDR_DENOISE_HOP;
      d->out_queue[ch] = g_realloc (d->out_queue[ch], d->out_queue_cap[ch] * sizeof (gfloat));
    }

    /* Odkładamy wynik OLA do bufora wyjściowego zamiast od razu do 'out' */
    memcpy (d->out_queue[ch] + d->out_queue_n[ch], d->acc[ch], SDR_DENOISE_HOP * sizeof (gfloat));
    d->out_queue_n[ch] += SDR_DENOISE_HOP;

    memmove (d->acc[ch], d->acc[ch] + SDR_DENOISE_HOP,
        (SDR_DENOISE_FFT - SDR_DENOISE_HOP) * sizeof (gfloat));
    memset (d->acc[ch] + (SDR_DENOISE_FFT - SDR_DENOISE_HOP), 0,
        SDR_DENOISE_HOP * sizeof (gfloat));

    d->pending_n[ch] = 0;
  }

  /* Kopiujemy z bufora wyjściowego do 'out', ale nie więcej niż 'n' próbek.
     Ponieważ out_n musi być <= n, nigdy nie przekroczymy bufora GStreamera. */
  out_n = MIN (n, d->out_queue_n[ch]);
  if (out_n > 0) {
    memcpy (out, d->out_queue[ch], out_n * sizeof (gfloat));
    /* Przesuwamy resztę w buforze wyjściowym na początek */
    memmove (d->out_queue[ch], d->out_queue[ch] + out_n,
        (d->out_queue_n[ch] - out_n) * sizeof (gfloat));
    d->out_queue_n[ch] -= out_n;
  }

  return out_n;
}

/* Adaptive smoothing pass, applied per channel after denoising (or
 * even with denoising disabled -- it's independent). In manual mode
 * (`interpolate`) the smoothing amount is fixed at `interp-strength`.
 * In `auto-interpolate` mode it's derived every buffer from a cheap
 * time-domain noise proxy: EMA of squared first-difference (~HF/noise
 * content) versus EMA of squared sample value (~signal level). This is
 * intentionally simple (a one-pole smoother, not a polyphase resampler)
 * so it's cheap enough to run unconditionally and has no failure modes
 * -- it trades a little top-end for fewer audible artifacts once the
 * IF bandwidth has been narrowed down for a weak signal. */
static void
denoise_apply_interp (GstSdrDenoise * d, gint ch, gfloat * buf, gint n)
{
  gfloat state = d->interp_state[ch];
  gfloat hf_ema = d->hf_energy_ema[ch];
  gfloat lf_ema = d->lf_energy_ema[ch];
  gfloat prev = d->prev_sample[ch];
  gint i;

  if (n <= 0)
    return;

  if (!d->interpolate && !d->auto_interpolate) {
    d->prev_sample[ch] = buf[n - 1];
    return;
  }

  for (i = 0; i < n; i++) {
    gfloat x = buf[i];
    gfloat d1 = x - prev;
    lf_ema = 0.99f * lf_ema + 0.01f * (x * x);
    hf_ema = 0.99f * hf_ema + 0.01f * (d1 * d1);
    prev = x;
  }

  {
    gfloat strength, coeff;
    if (d->auto_interpolate) {
      gfloat noise_db = 10.0f * log10f ((hf_ema + 1e-12f) / (lf_ema + 1e-9f));
      /* Calibration: roughly -40dB (clean, HF << signal) -> strength 0,
       * -10dB (choppy/noisy) -> strength 1. Tune against real signals. */
      strength = (noise_db + 40.0f) / 30.0f;
      strength = CLAMP (strength, 0.0f, 1.0f);
    } else {
      strength = d->interp_strength;
    }
    coeff = 1.0f - 0.9f * strength; /* 1.0 = passthrough, 0.1 = heavy smoothing */

    for (i = 0; i < n; i++) {
      state += coeff * (buf[i] - state);
      buf[i] = state;
    }
  }

  d->interp_state[ch] = state;
  d->hf_energy_ema[ch] = hf_ema;
  d->lf_energy_ema[ch] = lf_ema;
  d->prev_sample[ch] = prev;
}

static void
gst_sdr_denoise_class_init (GstSdrDenoiseClass * klass)
{
  GObjectClass *go = G_OBJECT_CLASS (klass);
  GstBaseTransformClass *bt = GST_BASE_TRANSFORM_CLASS (klass);
  GstElementClass *el = GST_ELEMENT_CLASS (klass);

  GST_DEBUG_CATEGORY_INIT (gst_sdr_denoise_debug, "sdrdenoise", 0, "SDR Anomaly Noise Reduction");

  gst_element_class_set_static_metadata (el,
      "SDR Anomaly-based Noise Reduction", "Filter/Audio",
      "Spectral noise reduction that suppresses only anomalies above a "
      "tracked per-bin noise floor (port of RecursiveNoiseFloorEstimator)",
      "Tomasz");

  gst_element_class_add_static_pad_template (el, &sinktemplate);
  gst_element_class_add_static_pad_template (el, &srctemplate);

  go->set_property = gst_sdr_denoise_set_property;
  go->get_property = gst_sdr_denoise_get_property;
  go->finalize = gst_sdr_denoise_finalize;

  g_object_class_install_property (go, PROP_ENABLED,
      g_param_spec_boolean ("enabled", "Enabled", "Enable noise reduction",
          DEFAULT_ENABLED, G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));
  g_object_class_install_property (go, PROP_THRESHOLD,
      g_param_spec_float ("threshold-db", "Threshold dB",
          "Anomaly threshold above the tracked noise floor, in dB", 1.0f, 40.0f,
          DEFAULT_THRESHOLD, G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));
  g_object_class_install_property (go, PROP_ALPHA_UP,
      g_param_spec_float ("alpha-up", "Alpha Up",
          "Noise floor rise rate (per hop) when the signal is above the floor",
          0.0001f, 1.0f, DEFAULT_ALPHA_UP, G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));
  g_object_class_install_property (go, PROP_ALPHA_DOWN,
      g_param_spec_float ("alpha-down", "Alpha Down",
          "Noise floor fall rate (per hop) when the signal dips below the floor",
          0.00001f, 1.0f, DEFAULT_ALPHA_DOWN, G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));
  g_object_class_install_property (go, PROP_INTERPOLATE,
      g_param_spec_boolean ("interpolate", "Interpolate",
          "Apply a light adaptive smoothing pass after denoising, "
          "strength fixed at interp-strength (default off, unchanged "
          "behaviour otherwise)",
          DEFAULT_INTERPOLATE, G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));
  g_object_class_install_property (go, PROP_AUTO_INTERPOLATE,
      g_param_spec_boolean ("auto-interpolate", "Auto Interpolate",
          "Derive the smoothing strength automatically from a HF/LF noise "
          "proxy each buffer, instead of using a fixed interp-strength",
          DEFAULT_AUTO_INTERPOLATE, G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));
  g_object_class_install_property (go, PROP_INTERP_STRENGTH,
      g_param_spec_float ("interp-strength", "Interpolation Strength",
          "Manual smoothing amount 0 (bypass) .. 1 (heavy) used when "
          "interpolate=true and auto-interpolate=false",
          0.0f, 1.0f, DEFAULT_INTERP_STRENGTH,
          G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS));

  bt->set_caps = gst_sdr_denoise_set_caps;
  bt->transform = gst_sdr_denoise_transform;
  bt->transform_size = gst_sdr_denoise_transform_size;
  bt->passthrough_on_same_caps = FALSE;
}

static void
gst_sdr_denoise_init (GstSdrDenoise * d)
{
  d->enabled = DEFAULT_ENABLED;
  d->threshold_db = DEFAULT_THRESHOLD;
  d->alpha_up = DEFAULT_ALPHA_UP;
  d->alpha_down = DEFAULT_ALPHA_DOWN;
  d->channels = 1;
  d->rate = 48000;
  d->hann = NULL;
  d->hist[0] = d->hist[1] = NULL;
  d->acc[0] = d->acc[1] = NULL;
  d->pending[0] = d->pending[1] = NULL;
  d->pending_n[0] = d->pending_n[1] = 0;
  d->noise_floor[0] = d->noise_floor[1] = NULL;
  d->allocated = FALSE;
  d->deint[0] = d->deint[1] = NULL;
  d->deint_cap = 0;
  
  /* Inicjalizacja bufora wyjściowego */
  d->out_queue[0] = d->out_queue[1] = NULL;
  d->out_queue_n[0] = d->out_queue_n[1] = 0;
  d->out_queue_cap[0] = d->out_queue_cap[1] = 0;

  d->interpolate = DEFAULT_INTERPOLATE;
  d->auto_interpolate = DEFAULT_AUTO_INTERPOLATE;
  d->interp_strength = DEFAULT_INTERP_STRENGTH;
  d->interp_state[0] = d->interp_state[1] = 0.0f;
  d->hf_energy_ema[0] = d->hf_energy_ema[1] = 0.0f;
  d->lf_energy_ema[0] = d->lf_energy_ema[1] = 1e-6f;
  d->prev_sample[0] = d->prev_sample[1] = 0.0f;
}

static void
gst_sdr_denoise_finalize (GObject * o)
{
  GstSdrDenoise *d = GST_SDR_DENOISE (o);
  gint ch;
  g_free (d->hann);
  for (ch = 0; ch < 2; ch++) {
    g_free (d->hist[ch]);
    g_free (d->acc[ch]);
    g_free (d->pending[ch]);
    g_free (d->noise_floor[ch]);
    g_free (d->deint[ch]);
    
    /* Zwalnianie bufora wyjściowego */
    g_free (d->out_queue[ch]);
  }
  G_OBJECT_CLASS (gst_sdr_denoise_parent_class)->finalize (o);
}

static gboolean
gst_sdr_denoise_set_caps (GstBaseTransform * t, GstCaps * incaps, GstCaps * outcaps)
{
  GstSdrDenoise *d = GST_SDR_DENOISE (t);
  GstStructure *s = gst_caps_get_structure (incaps, 0);
  gint rate = 48000, channels = 1;

  (void) outcaps;
  gst_structure_get_int (s, "rate", &rate);
  gst_structure_get_int (s, "channels", &channels);

  d->rate = (guint) rate;
  d->channels = channels;
  denoise_alloc (d);

  GST_INFO_OBJECT (d, "sdrdenoise: rate=%u channels=%d threshold=%.1fdB",
      d->rate, d->channels, d->threshold_db);
  return TRUE;
}

static gboolean
gst_sdr_denoise_transform_size (GstBaseTransform * t, GstPadDirection dir,
    GstCaps * caps, gsize size, GstCaps * ocaps, gsize * osize)
{
  (void) t; (void) dir; (void) caps; (void) ocaps;
  /* 1:1 passthrough size upper bound -- this stage only ever emits <=
   * as many samples as it was handed (hop-buffering delays some, never
   * invents extra), so input size is always a safe allocation bound. */
  *osize = size;
  return TRUE;
}

static GstFlowReturn
gst_sdr_denoise_transform (GstBaseTransform * t, GstBuffer * inbuf, GstBuffer * outbuf)
{
  GstSdrDenoise *d = GST_SDR_DENOISE (t);
  GstMapInfo inmap, outmap;
  gfloat *in, *out;
  gint n_frames, ch, i, out_n = 0;

  if (!gst_buffer_map (inbuf, &inmap, GST_MAP_READ) ||
      !gst_buffer_map (outbuf, &outmap, GST_MAP_WRITE))
    return GST_FLOW_ERROR;

  in = (gfloat *) inmap.data;
  out = (gfloat *) outmap.data;
  n_frames = (gint) (inmap.size / (d->channels * sizeof (gfloat)));

  if ((gsize) n_frames > d->deint_cap) {
    for (ch = 0; ch < d->channels; ch++)
      d->deint[ch] = g_realloc (d->deint[ch], n_frames * sizeof (gfloat));
    d->deint_cap = n_frames;
  }

  if (d->channels == 1) {
    out_n = denoise_process_channel (d, 0, in, n_frames, out);
    denoise_apply_interp (d, 0, out, out_n);
  } else {
    gfloat *outL = d->deint[0];
    gfloat *outR = d->deint[1];
    /* Używamy tymczasowych tablic do deinterleave, żeby nie nadpisać outL/outR przed przetworzeniem */
    gfloat *tmpL = g_alloca (n_frames * sizeof (gfloat));
    gfloat *tmpR = g_alloca (n_frames * sizeof (gfloat));
    gint nL, nR, m;

    for (i = 0; i < n_frames; i++) {
      tmpL[i] = in[i * 2 + 0];
      tmpR[i] = in[i * 2 + 1];
    }
    nL = denoise_process_channel (d, 0, tmpL, n_frames, outL);
    nR = denoise_process_channel (d, 1, tmpR, n_frames, outR);
    denoise_apply_interp (d, 0, outL, nL);
    denoise_apply_interp (d, 1, outR, nR);
    m = MIN (nL, nR);
    for (i = 0; i < m; i++) {
      out[i * 2 + 0] = outL[i];
      out[i * 2 + 1] = outR[i];
    }
    out_n = m;
  }

  gst_buffer_unmap (inbuf, &inmap);
  gst_buffer_unmap (outbuf, &outmap);
  gst_buffer_set_size (outbuf, (gsize) out_n * d->channels * sizeof (gfloat));
  return GST_FLOW_OK;
}

static void
gst_sdr_denoise_set_property (GObject * o, guint id, const GValue * v, GParamSpec * p)
{
  GstSdrDenoise *d = GST_SDR_DENOISE (o);
  switch (id) {
    case PROP_ENABLED: d->enabled = g_value_get_boolean (v); break;
    case PROP_THRESHOLD: d->threshold_db = g_value_get_float (v); break;
    case PROP_ALPHA_UP: d->alpha_up = g_value_get_float (v); break;
    case PROP_ALPHA_DOWN: d->alpha_down = g_value_get_float (v); break;
    case PROP_INTERPOLATE: d->interpolate = g_value_get_boolean (v); break;
    case PROP_AUTO_INTERPOLATE: d->auto_interpolate = g_value_get_boolean (v); break;
    case PROP_INTERP_STRENGTH: d->interp_strength = g_value_get_float (v); break;
    default: G_OBJECT_WARN_INVALID_PROPERTY_ID (o, id, p);
  }
}

static void
gst_sdr_denoise_get_property (GObject * o, guint id, GValue * v, GParamSpec * p)
{
  GstSdrDenoise *d = GST_SDR_DENOISE (o);
  switch (id) {
    case PROP_ENABLED: g_value_set_boolean (v, d->enabled); break;
    case PROP_THRESHOLD: g_value_set_float (v, d->threshold_db); break;
    case PROP_ALPHA_UP: g_value_set_float (v, d->alpha_up); break;
    case PROP_ALPHA_DOWN: g_value_set_float (v, d->alpha_down); break;
    case PROP_INTERPOLATE: g_value_set_boolean (v, d->interpolate); break;
    case PROP_AUTO_INTERPOLATE: g_value_set_boolean (v, d->auto_interpolate); break;
    case PROP_INTERP_STRENGTH: g_value_set_float (v, d->interp_strength); break;
    default: G_OBJECT_WARN_INVALID_PROPERTY_ID (o, id, p);
  }
}
