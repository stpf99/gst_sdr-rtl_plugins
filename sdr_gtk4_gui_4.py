#!/usr/bin/env python3
"""
sdr_gtk4_gui.py  (v3)

GTK4 control panel + widmo/wodospad (osobno kanał I / kanał Q, czarno-białe,
2-kolorowa normalizacja) + bufor 5s do trybu auto-korekty pasma IF.

Zmiany wzgledem v2:
  * Widmo rozbite na DWA osobne wodospady: "Kanał I (lewy)" i "Kanał Q (prawy)"
    - kazdy liczony z realnej (nie zespolonej) FFT samej skladowej I albo Q,
      wiec pokazuja dwa niezalezne widma tego samego zakresu.
    - renderowane WYLACZNIE czarno-bialo: sygnal powyzej progu = bialy piksel,
      ponizej progu (szum) = czarny piksel. Prog = mediana wiersza (podloga
      szumu) + margines w dB (regulowany suwakiem).
  * Wykrywanie szczytu (peak) i jego szerokosci pasma liczone jest OSOBNO,
    na pelnej zespolonej FFT (I+jQ) - bo tylko zespolone widmo poprawnie
    rozroznia gore/dol pasma. Nie jest wyswietlane wprost, ale steruje:
      - markerem szczytu rysowanym na obu wodospadach,
      - buforem 5-sekundowym (kolejka z timestampami) usredniajacym pozycje/
        szerokosc/poziom szczytu,
      - trybem "Auto-korekta": gdy wlaczony, co ~0.5s aplikuje ZYWO (bez
        restartu pipeline'u) usrednione if-bandwidth/freq-offset wyliczone
        z bufora, wiec efekt jest widoczny gladko w ciagu ~5s gdy sygnal
        sie zmienia, a nie skacze klatka po klatce.
      - jesli zaznaczysz pasmo przeciagnieciem, auto-korekta szuka szczytu
        TYLKO w zaznaczonym zakresie (namierzanie konkretnej stacji);
        bez zaznaczenia szuka najsilniejszego sygnalu w calym pasmie.

Wymaga: python3-gi, gir1.2-gtk-4.0, gir1.2-adw-1, gir1.2-gst-plugins-base-1.0,
        python3-numpy, oraz zbudowanego pluginu gst-sdr-plugins.
"""

import re
import sys
import time
import signal
from collections import deque

import gi

gi.require_version("Gtk", "4.0")
gi.require_version("Adw", "1")
gi.require_version("Gst", "1.0")
gi.require_version("GdkPixbuf", "2.0")

from gi.repository import Gtk, Adw, Gst, GLib, GdkPixbuf, Gdk  # noqa: E402

import numpy as np  # noqa: E402

Gst.init(None)

FFT_SIZE = 1024
DISP_BINS = 300
IQ_PROCESS_EVERY_N = 3          # przerzedzanie buforow IQ, zeby nie zamulic GUI
PEAK_BUFFER_SECONDS = 5.0       # dlugosc bufora usredniajacego dla auto-korekty
AUTO_APPLY_MIN_INTERVAL = 0.5   # min. odstep miedzy kolejnymi zywymi zmianami property
DC_GUARD_BINS = 6               # ile binow wokol DC ignorowac przy szukaniu szczytu
DEFAULT_THRESHOLD_MARGIN_DB = 6.0


# ==========================================================================
# Parsowanie czestotliwosci: "92.1M", "92100k", "92100000", "92,1MHz" itd.
# ==========================================================================
_FREQ_RE = re.compile(
    r'^\s*([0-9]*\.?[0-9]+)\s*(GHZ|MHZ|KHZ|HZ|G|M|K)?\s*$', re.IGNORECASE)

_FREQ_MULT = {
    "HZ": 1.0, "": 1.0,
    "K": 1e3, "KHZ": 1e3,
    "M": 1e6, "MHZ": 1e6,
    "G": 1e9, "GHZ": 1e9,
}


def parse_freq(text):
    m = _FREQ_RE.match(text.strip().replace(",", "."))
    if not m:
        raise ValueError(
            f"Nie rozumiem częstotliwości: {text!r}. "
            f"Użyj np. 92.1M, 92100k albo 92100000")
    val = float(m.group(1))
    unit = (m.group(2) or "").upper()
    return int(round(val * _FREQ_MULT[unit]))


def fmt_freq(hz):
    if hz >= 1e6:
        return f"{hz / 1e6:.3f} MHz"
    if hz >= 1e3:
        return f"{hz / 1e3:.1f} kHz"
    return f"{hz:.0f} Hz"


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ==========================================================================
# Widget widma/wodospadu - MONOCHROMATYCZNY (bialy sygnal / czarne tlo)
# ==========================================================================
class WaterfallView(Gtk.DrawingArea):
    """
    Wodospad czarno-bialy zasilany przez push_row(mag_db).
    Prog wykrywania = mediana wiersza + threshold_margin_db.
    Klikniecie -> on_tune(freq_hz). Przeciagniecie -> on_band_select(f_lo, f_hi).
    set_peak(freq_hz | None) rysuje/czysci znacznik szczytu (czerwona kreska).
    """

    def __init__(self, disp_bins=300, hist=140, label=""):
        super().__init__()
        self.disp_bins = disp_bins
        self.hist = hist
        self.label = label
        self.threshold_margin_db = DEFAULT_THRESHOLD_MARGIN_DB

        self.set_content_width(600)
        self.set_content_height(150)
        self.set_hexpand(True)

        self.img = np.zeros((hist, disp_bins, 3), dtype=np.uint8)
        self.center_freq = 92_000_000
        self.span = 250_000

        self.on_tune = None
        self.on_band_select = None
        self.freq_label_cb = None

        self._peak_freq = None
        self._widget_width = 600
        self._drag_start_x = None
        self._drag_cur_x = None

        self.set_draw_func(self._on_draw)

        click = Gtk.GestureClick()
        click.connect("released", self._on_release)
        self.add_controller(click)

        drag = Gtk.GestureDrag()
        drag.connect("drag-begin", self._on_drag_begin)
        drag.connect("drag-update", self._on_drag_update)
        drag.connect("drag-end", self._on_drag_end)
        self.add_controller(drag)

        motion = Gtk.EventControllerMotion()
        motion.connect("motion", self._on_motion)
        self.add_controller(motion)

    # -- dane -------------------------------------------------------------

    def set_axis(self, center_freq, span):
        self.center_freq = center_freq
        self.span = span

    def set_peak(self, freq_hz):
        self._peak_freq = freq_hz
        self.queue_draw()

    def push_row(self, mag_db):
        """Renderuje wiersz WYLACZNIE w 2 kolorach: bialy (sygnal) / czarny (szum)."""
        noise_floor = float(np.median(mag_db))
        threshold = noise_floor + self.threshold_margin_db
        mask = mag_db > threshold
        row = np.zeros((self.disp_bins, 3), dtype=np.uint8)
        row[mask] = (255, 255, 255)
        self.img = np.roll(self.img, 1, axis=0)
        self.img[0] = row
        self.queue_draw()

    # -- rysowanie ----------------------------------------------------------

    def _on_draw(self, area, cr, width, height):
        self._widget_width = width
        data = self.img.tobytes()
        rowstride = self.disp_bins * 3
        pixbuf = GdkPixbuf.Pixbuf.new_from_bytes(
            GLib.Bytes.new(data), GdkPixbuf.Colorspace.RGB, False, 8,
            self.disp_bins, self.hist, rowstride)
        cr.save()
        cr.scale(width / self.disp_bins, height / self.hist)
        Gdk.cairo_set_source_pixbuf(cr, pixbuf, 0, 0)
        cr.paint()
        cr.restore()

        if self._drag_start_x is not None and self._drag_cur_x is not None:
            x0 = min(self._drag_start_x, self._drag_cur_x)
            x1 = max(self._drag_start_x, self._drag_cur_x)
            cr.set_source_rgba(0.2, 0.6, 1.0, 0.25)
            cr.rectangle(x0, 0, x1 - x0, height)
            cr.fill()
            cr.set_source_rgba(0.2, 0.6, 1.0, 0.9)
            cr.rectangle(x0, 0, x1 - x0, height)
            cr.set_line_width(1.5)
            cr.stroke()

        # linia srodka
        cr.set_source_rgba(0.4, 0.4, 0.4, 0.8)
        cr.move_to(width / 2, 0)
        cr.line_to(width / 2, height)
        cr.set_line_width(1.0)
        cr.stroke()

        # znacznik szczytu (czerwona kreska u gory) - dziala jako "biezaco
        # zaznaczane szczytowe wychylenie"
        if self._peak_freq is not None:
            x = self._freq_to_x(self._peak_freq)
            if 0 <= x <= width:
                cr.set_source_rgba(1.0, 0.2, 0.2, 0.9)
                cr.move_to(x, 0)
                cr.line_to(x, height * 0.18)
                cr.set_line_width(2.0)
                cr.stroke()

        if self.label:
            cr.set_source_rgba(0.85, 0.85, 0.85, 0.9)
            cr.move_to(6, 14)
            cr.show_text(self.label)

    # -- mapowanie pixel <-> Hz --------------------------------------------

    def _x_to_freq(self, x):
        frac = x / max(self._widget_width, 1)
        return self.center_freq - self.span / 2.0 + frac * self.span

    def _freq_to_x(self, freq):
        frac = (freq - (self.center_freq - self.span / 2.0)) / self.span
        return frac * self._widget_width

    # -- gesty ---------------------------------------------------------------

    def _on_release(self, gesture, n_press, x, y):
        if self._drag_start_x is None:
            freq = self._x_to_freq(x)
            if self.on_tune:
                self.on_tune(freq)

    def _on_drag_begin(self, gesture, start_x, start_y):
        self._drag_start_x = start_x
        self._drag_cur_x = start_x

    def _on_drag_update(self, gesture, off_x, off_y):
        if self._drag_start_x is not None:
            self._drag_cur_x = self._drag_start_x + off_x
            self.queue_draw()

    def _on_drag_end(self, gesture, off_x, off_y):
        if self._drag_start_x is None:
            return
        x0 = self._drag_start_x
        x1 = self._drag_start_x + off_x
        if abs(x1 - x0) < 4:
            self._drag_start_x = None
            self._drag_cur_x = None
            freq = self._x_to_freq(x0)
            if self.on_tune:
                self.on_tune(freq)
            self.queue_draw()
            return
        f0 = self._x_to_freq(min(x0, x1))
        f1 = self._x_to_freq(max(x0, x1))
        self._drag_start_x = None
        self._drag_cur_x = None
        self.queue_draw()
        if self.on_band_select:
            self.on_band_select(f0, f1)

    def _on_motion(self, ctrl, x, y):
        if self.freq_label_cb:
            self.freq_label_cb(self._x_to_freq(x))


# ==========================================================================
# Pomocnicze widgety property
# ==========================================================================
class PropSlider(Gtk.Box):
    def __init__(self, element, prop_name, label, lo, hi, step, digits=2,
                 value=None, suffix=""):
        super().__init__(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        self.element = element
        self.prop_name = prop_name
        self.suffix = suffix

        lbl = Gtk.Label(label=label, xalign=0)
        lbl.set_size_request(140, -1)
        self.append(lbl)

        if value is None:
            try:
                value = element.get_property(prop_name)
            except Exception:
                value = lo

        adj = Gtk.Adjustment(value=value, lower=lo, upper=hi,
                              step_increment=step, page_increment=step * 10)
        self.scale = Gtk.Scale(orientation=Gtk.Orientation.HORIZONTAL,
                                adjustment=adj, digits=digits, hexpand=True)
        self.scale.set_draw_value(False)
        self.scale.connect("value-changed", self._on_changed)
        self.append(self.scale)

        self.value_lbl = Gtk.Label(label=self._fmt(value), width_chars=12)
        self.append(self.value_lbl)

    def _fmt(self, v):
        return f"{v:.2f}{self.suffix}"

    def _on_changed(self, scale):
        v = scale.get_value()
        self.value_lbl.set_label(self._fmt(v))
        try:
            self.element.set_property(self.prop_name, v)
        except Exception as e:
            print(f"[PropSlider] set_property({self.prop_name}) failed: {e}",
                  file=sys.stderr)

    def set_sensitive_live(self, sensitive):
        self.scale.set_sensitive(sensitive)

    def set_value_quiet(self, v):
        self.scale.handler_block_by_func(self._on_changed)
        self.scale.set_value(v)
        self.value_lbl.set_label(self._fmt(v))
        self.scale.handler_unblock_by_func(self._on_changed)

    def set_value_live(self, v):
        self.scale.set_value(v)


class PropSwitch(Gtk.Box):
    def __init__(self, element, prop_name, label, on_toggle=None):
        super().__init__(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        self.element = element
        self.prop_name = prop_name
        self.on_toggle = on_toggle

        lbl = Gtk.Label(label=label, xalign=0, hexpand=True)
        self.append(lbl)

        try:
            initial = element.get_property(prop_name)
        except Exception:
            initial = False

        self.switch = Gtk.Switch(active=initial, valign=Gtk.Align.CENTER)
        self.switch.connect("state-set", self._on_toggle)
        self.append(self.switch)

    def _on_toggle(self, switch, state):
        try:
            self.element.set_property(self.prop_name, state)
        except Exception as e:
            print(f"[PropSwitch] set_property({self.prop_name}) failed: {e}",
                  file=sys.stderr)
        if self.on_toggle:
            self.on_toggle(state)
        return False

    def get_active(self):
        return self.switch.get_active()

    def set_active_live(self, v):
        self.switch.set_active(v)


class SimpleSlider(Gtk.Box):
    """Suwak NIE spiety z GStreamer property - do parametrow czysto Pythonowych
    (np. margines progu wykrywania sygnalu)."""

    def __init__(self, label, lo, hi, step, value, digits=1, suffix="",
                 on_change=None):
        super().__init__(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        self.suffix = suffix
        self.on_change = on_change

        lbl = Gtk.Label(label=label, xalign=0)
        lbl.set_size_request(180, -1)
        self.append(lbl)

        adj = Gtk.Adjustment(value=value, lower=lo, upper=hi,
                              step_increment=step, page_increment=step * 5)
        self.scale = Gtk.Scale(orientation=Gtk.Orientation.HORIZONTAL,
                                adjustment=adj, digits=digits, hexpand=True)
        self.scale.set_draw_value(False)
        self.scale.connect("value-changed", self._on_changed)
        self.append(self.scale)

        self.value_lbl = Gtk.Label(label=f"{value:.{digits}f}{suffix}", width_chars=10)
        self.append(self.value_lbl)

    def _on_changed(self, scale):
        v = scale.get_value()
        self.value_lbl.set_label(f"{v:.1f}{self.suffix}")
        if self.on_change:
            self.on_change(v)


# ==========================================================================
# Glowne okno
# ==========================================================================
class SdrWindow(Adw.ApplicationWindow):
    def __init__(self, app, host, port, frequency, sample_rate):
        super().__init__(application=app, title="SDR FM Control")
        self.set_default_size(780, 1060)

        self.pipeline = None
        self.sdrsrc = None
        self.sdrdemod = None
        self.sdrdenoise = None
        self.iqsink = None

        self._init_host = host
        self._init_port = port
        self._init_freq = frequency
        self._sample_rate = sample_rate
        self._iq_buf_counter = 0
        self._closing = False

        self._selection_freqs = None      # (f_lo, f_hi) z recznego zaznaczenia
        self._pending_suggestion = None
        self._peak_buffer = deque()       # (t, freq_offset, bw_hz, level_db, floor_db)
        self._last_auto_apply = 0.0
        self._current_stereo = True

        self._build_pipeline(host, port, frequency, sample_rate, stereo=True)
        self._build_ui()
        self._start()

        self.connect("close-request", self._on_close_request)
        self._install_signal_handlers()

    # ---------------- Pipeline ---------------------------------------------

    def _build_pipeline(self, host, port, frequency, sample_rate, stereo=True):
        stereo_str = "true" if stereo else "false"
        self._current_stereo = stereo
        desc = (
            f'sdrsrc name=src mode=tcp host={host} port={port} '
            f'frequency={frequency} sample-rate={sample_rate} gain=4.0 '
            f'auto-gain=false auto-gain-target-db=-18.0 '
            f'! tee name=t '
            f'  t. ! queue name=q1 max-size-buffers=0 max-size-bytes=0 '
            f'         max-size-time=200000000 leaky=downstream '
            f'       ! sdrdemod name=demod mode=fm stereo={stereo_str} max-deviation=75000 '
            f'         audio-rate=48000 audio-cutoff=15000 tau=50 freq-offset=0 '
            f'         if-bandwidth=0 auto-bandwidth=false '
            f'       ! audioconvert ! audioresample '
            f'       ! audio/x-raw,rate=48000 '
            f'       ! sdrdenoise name=denoise enabled=true threshold-db=8 '
            f'         alpha-up=0.01 alpha-down=0.0001 interpolate=false '
            f'         auto-interpolate=false interp-strength=0.5 '
            f'       ! queue name=q2 max-size-buffers=0 max-size-bytes=0 '
            f'         max-size-time=1000000000 '
            f'       ! autoaudiosink sync=false '
            f'  t. ! queue name=qiq leaky=downstream max-size-buffers=2 '
            f'         max-size-bytes=0 max-size-time=0 '
            f'       ! appsink name=iqsink emit-signals=true sync=false '
            f'         max-buffers=2 drop=true'
        )
        self.pipeline = Gst.parse_launch(desc)
        self.sdrsrc = self.pipeline.get_by_name("src")
        self.sdrdemod = self.pipeline.get_by_name("demod")
        self.sdrdenoise = self.pipeline.get_by_name("denoise")
        self.iqsink = self.pipeline.get_by_name("iqsink")
        self.iqsink.connect("new-sample", self._on_new_iq_sample)

        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message::error", self._on_bus_error)
        bus.connect("message::warning", self._on_bus_warning)
        bus.connect("message::eos", self._on_bus_eos)

    def _start(self):
        self.pipeline.set_state(Gst.State.PLAYING)

    # -- IQ -> FFT (watek streamingowy GStreamera!) --------------------------

    def _on_new_iq_sample(self, sink):
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.OK

        self._iq_buf_counter += 1
        if self._iq_buf_counter % IQ_PROCESS_EVERY_N != 0:
            return Gst.FlowReturn.OK

        buf = sample.get_buffer()
        ok, mapinfo = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.OK
        try:
            raw = np.frombuffer(mapinfo.data, dtype=np.float32)
        finally:
            buf.unmap(mapinfo)

        n_complex = len(raw) // 2
        if n_complex < FFT_SIZE:
            return Gst.FlowReturn.OK

        iq = raw[: FFT_SIZE * 2].reshape(-1, 2)
        i_ch = iq[:, 0]
        q_ch = iq[:, 1]
        win = np.hanning(FFT_SIZE)

        # -- widma monochromatyczne, osobno I i Q (na wyswietlanie) --
        spec_i = np.fft.fftshift(np.fft.fft(i_ch * win))
        spec_q = np.fft.fftshift(np.fft.fft(q_ch * win))
        mag_i = 20.0 * np.log10(np.abs(spec_i) + 1e-9)
        mag_q = 20.0 * np.log10(np.abs(spec_q) + 1e-9)
        disp_i = self._bin_down(mag_i)
        disp_q = self._bin_down(mag_q)

        # -- pelne zespolone widmo (I+jQ), tylko do wykrywania szczytu --
        cplx = i_ch + 1j * q_ch
        spec_c = np.fft.fftshift(np.fft.fft(cplx * win))
        mag_c = 20.0 * np.log10(np.abs(spec_c) + 1e-9)

        center = self._current_freq()
        bin_hz = self._sample_rate / FFT_SIZE
        peak = self._find_peak(mag_c, center, bin_hz)

        GLib.idle_add(self._on_spectrum_frame, disp_i, disp_q, peak)
        return Gst.FlowReturn.OK

    @staticmethod
    def _bin_down(mag_db):
        if FFT_SIZE % DISP_BINS == 0:
            return mag_db.reshape(DISP_BINS, FFT_SIZE // DISP_BINS).mean(axis=1)
        return np.interp(np.linspace(0, FFT_SIZE - 1, DISP_BINS),
                          np.arange(FFT_SIZE), mag_db)

    def _find_peak(self, mag_c, center, bin_hz):
        """Szuka najsilniejszego szczytu (w calym pasmie albo tylko w recznie
        zaznaczonym zakresie) i estymuje jego szerokosc. Zwraca dict albo None."""
        n = len(mag_c)
        lo_bin, hi_bin = 0, n - 1

        if self._selection_freqs:
            f_lo, f_hi = self._selection_freqs
            lo_bin = int(clamp((f_lo - (center - self._sample_rate / 2)) / bin_hz, 0, n - 1))
            hi_bin = int(clamp((f_hi - (center - self._sample_rate / 2)) / bin_hz, 0, n - 1))
            if hi_bin <= lo_bin:
                lo_bin, hi_bin = 0, n - 1

        search = mag_c[lo_bin:hi_bin + 1].copy()
        # wytnij DC guard jesli jest w zasiegu przeszukiwania
        dc_bin = n // 2
        if lo_bin <= dc_bin <= hi_bin:
            local_dc = dc_bin - lo_bin
            g0 = max(0, local_dc - DC_GUARD_BINS)
            g1 = min(len(search), local_dc + DC_GUARD_BINS + 1)
            search[g0:g1] = -999.0

        if len(search) == 0 or np.all(search <= -998.0):
            return None

        peak_local = int(np.argmax(search))
        peak_bin = lo_bin + peak_local
        peak_level = float(mag_c[peak_bin])
        noise_floor = float(np.median(mag_c[lo_bin:hi_bin + 1]))

        margin = DEFAULT_THRESHOLD_MARGIN_DB
        left = peak_bin
        while left > lo_bin and mag_c[left - 1] > noise_floor + margin:
            left -= 1
        right = peak_bin
        while right < hi_bin and mag_c[right + 1] > noise_floor + margin:
            right += 1

        bw_hz = (right - left) * bin_hz
        freq_offset = (peak_bin - n / 2) * bin_hz
        return {
            "freq_offset": freq_offset,
            "bw_hz": bw_hz,
            "level": peak_level,
            "floor": noise_floor,
        }

    def _on_spectrum_frame(self, disp_i, disp_q, peak):
        if self._closing:
            return False
        center = self._current_freq()
        self.wf_left.set_axis(center, self._sample_rate)
        self.wf_right.set_axis(center, self._sample_rate)
        self.wf_left.push_row(disp_i)
        self.wf_right.push_row(disp_q)

        now = time.monotonic()
        if peak is not None:
            self._peak_buffer.append(
                (now, peak["freq_offset"], peak["bw_hz"], peak["level"], peak["floor"]))
        while self._peak_buffer and now - self._peak_buffer[0][0] > PEAK_BUFFER_SECONDS:
            self._peak_buffer.popleft()

        self._update_auto_correct(center, now)
        return False

    def _current_freq(self):
        try:
            return self.sdrsrc.get_property("frequency")
        except Exception:
            return self._init_freq

    # -- bufor 5s + auto-korekta ---------------------------------------------

    def _update_auto_correct(self, center, now):
        if not self._peak_buffer:
            self.auto_status_lbl.set_label("Bufor pusty — brak wykrytego sygnału")
            self.wf_left.set_peak(None)
            self.wf_right.set_peak(None)
            return

        arr = np.array([(o, bw, lv, fl) for (_, o, bw, lv, fl) in self._peak_buffer])
        avg_offset = float(arr[:, 0].mean())
        avg_bw = float(arr[:, 1].mean())
        avg_level = float(arr[:, 2].mean())
        avg_floor = float(arr[:, 3].mean())
        snr = avg_level - avg_floor

        peak_freq = center + avg_offset
        self.wf_left.set_peak(peak_freq)
        self.wf_right.set_peak(peak_freq)

        if_bw_suggest = clamp(avg_bw, 0.0, 200000.0)
        freq_off_suggest = clamp(avg_offset, -100000.0, 100000.0)

        scope = "zaznaczonym paśmie" if self._selection_freqs else "całym paśmie"
        self.auto_status_lbl.set_label(
            f"Bufor {len(self._peak_buffer)} próbek (~{PEAK_BUFFER_SECONDS:.0f}s), szukam w {scope}\n"
            f"Szczyt: {fmt_freq(peak_freq)}  |  S/N ≈ {snr:.1f} dB  |  "
            f"szacowane pasmo: {fmt_freq(avg_bw)}\n"
            f"Sugestia (uśredniona): if-bandwidth={fmt_freq(if_bw_suggest)}, "
            f"freq-offset={freq_off_suggest:+.0f} Hz")

        if self.auto_correct_switch.get_active():
            if now - self._last_auto_apply >= AUTO_APPLY_MIN_INTERVAL:
                if self.auto_bw_switch.get_active():
                    self.auto_bw_switch.set_active_live(False)
                self.if_bw_slider.set_value_live(if_bw_suggest)
                self.freq_offset_slider.set_value_live(freq_off_suggest)
                self._last_auto_apply = now

    # -- bus ------------------------------------------------------------------

    def _on_bus_error(self, bus, msg):
        err, dbg = msg.parse_error()
        print(f"[GST ERROR] {err} ({dbg})", file=sys.stderr)
        self.status_lbl.set_label(f"Błąd: {err}")

    def _on_bus_warning(self, bus, msg):
        w, dbg = msg.parse_warning()
        print(f"[GST WARN] {w} ({dbg})", file=sys.stderr)

    def _on_bus_eos(self, bus, msg):
        self.status_lbl.set_label("Koniec strumienia (EOS)")

    # -- reconnect (tylko host/port/mode) --------------------------------------

    def _reconnect(self, host, port, frequency, stereo=None):
        if stereo is None:
            stereo = self._current_stereo
        self.status_lbl.set_label("Przełączam źródło...")
        self._graceful_pipeline_stop()
        self._build_pipeline(host, port, frequency, self._sample_rate, stereo=stereo)
        self._rebind_prop_widgets()
        self._start()
        self.status_lbl.set_label("Połączono")

    def _restart_for_stereo(self, stereo):
        """
        UWAGA (diagnoza buga + true stereo): w gst_sdr_demod.c property
        'stereo' zmienia liczbe kanalow wyjsciowych liczona na zywo w
        transform_caps()/transform_size(), ale set_property() dla
        PROP_STEREO nie wywoluje gst_base_transform_reconfigure_src() ani
        nie bierze locka - wiec zwykly g_object_set(demod, "stereo", ...)
        w trakcie PLAYING rozjezdza juz wynegocjowane caps z faktycznym
        rozmiarem bufora i zatyka strumien. Dopoki C nie dostanie fixa
        (patrz odpowiedz w czacie), tutaj celowo robimy PELNY restart
        pipeline'u zamiast live set_property.

        Od tej wersji pipeline NIE wymusza juz channels=1 przed sdrdenoise
        (ktory i tak w pelni obsluguje stereo - deinterleave/process/
        reinterleave, patrz gst_sdr_denoise_transform()), wiec przelaczenie
        stereo teraz faktycznie zmienia liczbe kanalow AZ DO autoaudiosink -
        to jest prawdziwe stereo, nie tylko wewnetrzny downmix.
        """
        freq = self._current_freq()
        self._reconnect(self._init_host if not hasattr(self, "host_entry")
                         else self.host_entry.get_text().strip(),
                         int(self.port_entry.get_text().strip())
                         if hasattr(self, "port_entry") else self._init_port,
                         int(freq), stereo=stereo)

    # ---------------- UI -------------------------------------------------------

    def _build_ui(self):
        toolbar = Adw.ToolbarView()
        header = Adw.HeaderBar()
        toolbar.add_top_bar(header)

        outer = Gtk.ScrolledWindow(vexpand=True)
        content = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=14,
                           margin_top=14, margin_bottom=14,
                           margin_start=14, margin_end=14)
        outer.set_child(content)
        toolbar.set_content(outer)
        self.set_content(toolbar)

        self.status_lbl = Gtk.Label(label="Start...", xalign=0)
        self.status_lbl.add_css_class("dim-label")
        content.append(self.status_lbl)

        content.append(self._build_connection_group())
        content.append(self._build_waterfall_group())
        content.append(self._build_autocorrect_group())
        content.append(self._build_tuning_group())
        content.append(self._build_gain_group())
        content.append(self._build_bandwidth_group())
        content.append(self._build_denoise_group())
        content.append(self._build_transport_group())

    def _group(self, title):
        frame = Gtk.Frame(label=title)
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8,
                       margin_top=8, margin_bottom=8,
                       margin_start=8, margin_end=8)
        frame.set_child(box)
        return frame, box

    # -- Polaczenie -----------------------------------------------------------

    def _build_connection_group(self):
        frame, box = self._group("Źródło (rtl_tcp) — zmiana wymaga restartu")
        row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        self.host_entry = Gtk.Entry(text=self._init_host, hexpand=True)
        self.port_entry = Gtk.Entry(text=str(self._init_port), width_chars=6)
        apply_btn = Gtk.Button(label="Zastosuj połączenie")
        apply_btn.connect("clicked", self._on_apply_connection)
        row.append(Gtk.Label(label="host:"))
        row.append(self.host_entry)
        row.append(Gtk.Label(label="port:"))
        row.append(self.port_entry)
        row.append(apply_btn)
        box.append(row)
        return frame

    def _on_apply_connection(self, btn):
        host = self.host_entry.get_text().strip()
        try:
            port = int(self.port_entry.get_text().strip())
        except ValueError:
            self.status_lbl.set_label("Nieprawidłowy port")
            return
        freq = self._current_freq()
        self._reconnect(host, port, int(freq))

    # -- Widmo/wodospad (I / Q, czarno-biale) ------------------------------------

    def _build_waterfall_group(self):
        frame, box = self._group("Widmo / wodospad — kanał I (lewy) i Q (prawy)")

        self.wf_left = WaterfallView(disp_bins=DISP_BINS, hist=140, label="I (lewy)")
        self.wf_right = WaterfallView(disp_bins=DISP_BINS, hist=140, label="Q (prawy)")
        for wf in (self.wf_left, self.wf_right):
            wf.set_axis(self._init_freq, self._sample_rate)
            wf.on_tune = self._on_waterfall_tune
            wf.on_band_select = self._on_waterfall_band_select
            wf.freq_label_cb = self._on_waterfall_hover
            box.append(wf)

        self.hover_freq_lbl = Gtk.Label(label="—", xalign=0)
        self.hover_freq_lbl.add_css_class("dim-label")
        box.append(self.hover_freq_lbl)

        hint = Gtk.Label(
            label="Klik = strojenie na wskazaną częstotliwość. Przeciągnij = "
                  "zaznacz pasmo stacji (ogranicza szukanie szczytu dla "
                  "auto-korekty). Biały = sygnał powyżej progu, czarny = szum.",
            xalign=0, wrap=True)
        hint.add_css_class("dim-label")
        box.append(hint)

        threshold_slider = SimpleSlider(
            "Próg wykrywania (nad podłogą szumu)", lo=1.0, hi=20.0, step=0.5,
            value=DEFAULT_THRESHOLD_MARGIN_DB, digits=1, suffix=" dB",
            on_change=self._on_threshold_changed)
        box.append(threshold_slider)

        clear_btn = Gtk.Button(label="Wyczyść zaznaczenie pasma")
        clear_btn.connect("clicked", self._on_clear_selection)
        box.append(clear_btn)

        self.suggest_frame = Gtk.Frame(label="Sugestie dla zaznaczonego pasma (jednorazowo)")
        sbox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6,
                        margin_top=8, margin_bottom=8, margin_start=8, margin_end=8)
        self.suggest_frame.set_child(sbox)
        self.suggest_lbl = Gtk.Label(label="Zaznacz pasmo na wodospadzie...",
                                      xalign=0, wrap=True)
        sbox.append(self.suggest_lbl)
        self.suggest_apply_btn = Gtk.Button(label="Zastosuj sugestię (jednorazowo)")
        self.suggest_apply_btn.set_sensitive(False)
        self.suggest_apply_btn.connect("clicked", self._on_apply_suggestion)
        sbox.append(self.suggest_apply_btn)
        box.append(self.suggest_frame)

        return frame

    def _on_threshold_changed(self, v):
        self.wf_left.threshold_margin_db = v
        self.wf_right.threshold_margin_db = v

    def _on_clear_selection(self, btn):
        self._selection_freqs = None
        self.suggest_lbl.set_label("Zaznacz pasmo na wodospadzie...")
        self.suggest_apply_btn.set_sensitive(False)
        self.status_lbl.set_label("Wyczyszczono zaznaczenie — szukam w całym paśmie")

    def _on_waterfall_hover(self, freq):
        self.hover_freq_lbl.set_label(f"Kursor: {fmt_freq(freq)}")

    def _on_waterfall_tune(self, freq_hz):
        freq_hz = int(round(freq_hz))
        self.sdrsrc.set_property("frequency", freq_hz)
        self.sdrdemod.set_property("freq-offset", 0.0)
        self.freq_slider.set_value_quiet(freq_hz)
        self.freq_offset_slider.set_value_quiet(0.0)
        self.freq_entry.set_text(fmt_freq(freq_hz))
        self._selection_freqs = None  # nowe strojenie -> reset zaznaczenia
        self.status_lbl.set_label(f"Nastrojono: {fmt_freq(freq_hz)}")

    def _on_waterfall_band_select(self, f_lo, f_hi):
        self._selection_freqs = (f_lo, f_hi)
        center_freq = self._current_freq()
        bandwidth = abs(f_hi - f_lo)
        band_center = (f_lo + f_hi) / 2.0
        freq_offset = band_center - center_freq

        if_bw_suggest = clamp(bandwidth, 0.0, 200000.0)
        freq_offset_suggest = clamp(freq_offset, -100000.0, 100000.0)

        self._pending_suggestion = (if_bw_suggest, freq_offset_suggest)
        self.suggest_lbl.set_label(
            f"Zaznaczone pasmo: {fmt_freq(f_lo)} – {fmt_freq(f_hi)} "
            f"(szerokość ≈ {fmt_freq(bandwidth)})\n"
            f"Sugerowane if-bandwidth: {fmt_freq(if_bw_suggest)}\n"
            f"Sugerowany freq-offset (NCO): {freq_offset_suggest:+.0f} Hz\n"
            f"Auto-korekta (jeśli włączona) będzie teraz szukać szczytu "
            f"tylko w tym zakresie.")
        self.suggest_apply_btn.set_sensitive(True)

    def _on_apply_suggestion(self, btn):
        if not self._pending_suggestion:
            return
        if_bw, freq_off = self._pending_suggestion
        self.auto_bw_switch.set_active_live(False)
        self.if_bw_slider.set_value_live(if_bw)
        self.freq_offset_slider.set_value_live(freq_off)
        self.status_lbl.set_label("Zastosowano sugestię pasma")

    # -- Auto-korekta (bufor 5s) --------------------------------------------------

    def _build_autocorrect_group(self):
        frame, box = self._group("Auto-korekta (bufor ~5s, na żywo)")

        # Celowo NIE uzywamy PropSwitch tutaj: to nie jest property GStreamer,
        # tylko flaga sterujaca petla w _update_auto_correct(), wiec to zwykly
        # Gtk.Switch bez wlasnego set_property().
        row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        lbl = Gtk.Label(label="Auto-korekta if-bandwidth / freq-offset", xalign=0, hexpand=True)
        self.auto_correct_switch = Gtk.Switch(active=False, valign=Gtk.Align.CENTER)
        row.append(lbl)
        row.append(self.auto_correct_switch)
        box.append(row)

        self.auto_status_lbl = Gtk.Label(label="Bufor pusty — brak wykrytego sygnału",
                                          xalign=0, wrap=True)
        self.auto_status_lbl.add_css_class("dim-label")
        box.append(self.auto_status_lbl)

        hint = Gtk.Label(
            label="Gdy włączone: co ~0.5s aplikuje na żywo if-bandwidth i "
                  "freq-offset uśrednione z ostatnich ~5s wykrytego szczytu, "
                  "więc reaguje płynnie na wahania mocy/szczegółów sygnału "
                  "zamiast skakać klatka po klatce. Wyłącza przy tym "
                  "auto-bandwidth (żeby nie konkurowały).",
            xalign=0, wrap=True)
        hint.add_css_class("dim-label")
        box.append(hint)

        return frame

    # -- Strojenie ---------------------------------------------------------------

    def _build_tuning_group(self):
        frame, box = self._group("Strojenie (na żywo)")

        entry_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        entry_row.append(Gtk.Label(label="Częstotliwość:"))
        self.freq_entry = Gtk.Entry(text=fmt_freq(self._init_freq), hexpand=True)
        self.freq_entry.connect("activate", self._on_freq_entry_activate)
        entry_row.append(self.freq_entry)
        set_btn = Gtk.Button(label="Ustaw")
        set_btn.connect("clicked", self._on_freq_entry_activate)
        entry_row.append(set_btn)
        box.append(entry_row)

        hint = Gtk.Label(
            label="Akceptowane formaty: 92.1M / 92100k / 92100000 / 92,1MHz",
            xalign=0)
        hint.add_css_class("dim-label")
        box.append(hint)

        self.freq_slider = PropSlider(
            self.sdrsrc, "frequency", "Częstotliwość (suwak)",
            lo=24_000_000, hi=1_766_000_000, step=1000, digits=0,
            value=self._init_freq, suffix=" Hz")
        self.freq_slider.scale.connect("value-changed", self._on_freq_slider_moved)
        box.append(self.freq_slider)

        self.freq_offset_slider = PropSlider(
            self.sdrdemod, "freq-offset", "Offset NCO",
            lo=-100000, hi=100000, step=100, digits=0, suffix=" Hz")
        box.append(self.freq_offset_slider)

        stereo_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        stereo_lbl = Gtk.Label(label="Stereo (przełączenie = krótki restart ~1s)",
                                xalign=0, hexpand=True)
        self.stereo_switch = Gtk.Switch(active=self._current_stereo,
                                         valign=Gtk.Align.CENTER)
        self.stereo_switch.connect("state-set", self._on_stereo_switch_toggled)
        stereo_row.append(stereo_lbl)
        stereo_row.append(self.stereo_switch)
        box.append(stereo_row)

        stereo_hint = Gtk.Label(
            label="Nie jest to live-property (patrz wyjaśnienie w czacie: "
                  "'stereo' zmienia liczbę kanałów wyjściowych demodulatora, "
                  "a C-owy set_property nie wymusza renegocjacji capsów, "
                  "więc live-toggle zatyka pipeline). Tu robimy pełny, "
                  "bezpieczny restart zamiast tego.",
            xalign=0, wrap=True)
        stereo_hint.add_css_class("dim-label")
        box.append(stereo_hint)

        return frame

    def _on_stereo_switch_toggled(self, switch, state):
        self._restart_for_stereo(state)
        return False

    def _on_freq_entry_activate(self, widget):
        text = self.freq_entry.get_text()
        try:
            hz = parse_freq(text)
        except ValueError as e:
            self.status_lbl.set_label(str(e))
            return
        lo, hi = 24_000_000, 1_766_000_000
        if not (lo <= hz <= hi):
            self.status_lbl.set_label(
                f"Poza zakresem tunera ({fmt_freq(lo)}–{fmt_freq(hi)})")
            return
        self.sdrsrc.set_property("frequency", hz)
        self.freq_slider.set_value_quiet(hz)
        self.freq_entry.set_text(fmt_freq(hz))
        self._selection_freqs = None
        self.status_lbl.set_label(f"Nastrojono: {fmt_freq(hz)}")

    def _on_freq_slider_moved(self, scale):
        self.freq_entry.set_text(fmt_freq(scale.get_value()))

    # -- Gain / AGC ----------------------------------------------------------

    def _build_gain_group(self):
        frame, box = self._group("Gain / AGC")

        self.gain_slider = PropSlider(
            self.sdrsrc, "gain", "Gain ręczny", lo=0.0, hi=50.0, step=0.1,
            digits=1, suffix=" dB")
        box.append(self.gain_slider)

        self.auto_gain_switch = PropSwitch(
            self.sdrsrc, "auto-gain", "Auto Gain (AGC)",
            on_toggle=self._on_auto_gain_toggle)
        box.append(self.auto_gain_switch)

        self.auto_gain_target_slider = PropSlider(
            self.sdrsrc, "auto-gain-target-db", "Cel AGC",
            lo=-40.0, hi=0.0, step=0.5, digits=1, suffix=" dBFS")
        box.append(self.auto_gain_target_slider)

        self._on_auto_gain_toggle(self.auto_gain_switch.get_active())
        return frame

    def _on_auto_gain_toggle(self, active):
        self.gain_slider.set_sensitive_live(not active)
        self.auto_gain_target_slider.set_sensitive_live(active)

    # -- IF Bandwidth --------------------------------------------------------

    def _build_bandwidth_group(self):
        frame, box = self._group("Pasmo IF / dostrajanie S/N")

        self.if_bw_slider = PropSlider(
            self.sdrdemod, "if-bandwidth", "IF Bandwidth (0=auto z deviation)",
            lo=0.0, hi=200000.0, step=500.0, digits=0, suffix=" Hz")
        box.append(self.if_bw_slider)

        self.auto_bw_switch = PropSwitch(
            self.sdrdemod, "auto-bandwidth", "Auto-Bandwidth C (S/N hunt w C, tylko FM)",
            on_toggle=self._on_auto_bw_toggle)
        box.append(self.auto_bw_switch)

        hint = Gtk.Label(
            label="To jest OSOBNY mechanizm od 'Auto-korekty' w sekcji widma "
                  "(ten C-owy dziala na sygnale zdemodulowanym; auto-korekta "
                  "wyzej dziala na widmie IQ z wodospadu). Nie wlaczaj obu "
                  "naraz zeby nie konkurowaly o if-bandwidth.",
            xalign=0, wrap=True)
        hint.add_css_class("dim-label")
        box.append(hint)

        self._on_auto_bw_toggle(self.auto_bw_switch.get_active())
        return frame

    def _on_auto_bw_toggle(self, active):
        self.if_bw_slider.set_sensitive_live(not active)

    # -- Denoise ---------------------------------------------------------------

    def _build_denoise_group(self):
        frame, box = self._group("Redukcja szumu i interpolacja")

        self.denoise_switch = PropSwitch(self.sdrdenoise, "enabled", "Denoise włączony")
        box.append(self.denoise_switch)

        self.threshold_slider = PropSlider(
            self.sdrdenoise, "threshold-db", "Próg", lo=-40.0, hi=20.0,
            step=0.5, digits=1, suffix=" dB")
        box.append(self.threshold_slider)

        self.alpha_up_slider = PropSlider(
            self.sdrdenoise, "alpha-up", "Alpha Up", lo=0.0001, hi=1.0,
            step=0.001, digits=4)
        box.append(self.alpha_up_slider)

        self.alpha_down_slider = PropSlider(
            self.sdrdenoise, "alpha-down", "Alpha Down", lo=0.00001, hi=0.1,
            step=0.0001, digits=5)
        box.append(self.alpha_down_slider)

        box.append(Gtk.Separator(orientation=Gtk.Orientation.HORIZONTAL))

        self.interp_switch = PropSwitch(
            self.sdrdenoise, "interpolate", "Interpolacja (wygładzanie)",
            on_toggle=self._on_interp_toggle)
        box.append(self.interp_switch)

        self.auto_interp_switch = PropSwitch(
            self.sdrdenoise, "auto-interpolate", "Auto-Interpolacja",
            on_toggle=self._on_auto_interp_toggle)
        box.append(self.auto_interp_switch)

        self.interp_strength_slider = PropSlider(
            self.sdrdenoise, "interp-strength", "Siła interpolacji (ręczna)",
            lo=0.0, hi=1.0, step=0.01, digits=2)
        box.append(self.interp_strength_slider)

        self._on_interp_toggle(self.interp_switch.get_active())
        self._on_auto_interp_toggle(self.auto_interp_switch.get_active())
        return frame

    def _on_interp_toggle(self, active):
        self.auto_interp_switch.switch.set_sensitive(active)
        self.interp_strength_slider.set_sensitive_live(
            active and not self.auto_interp_switch.get_active())

    def _on_auto_interp_toggle(self, active):
        self.interp_strength_slider.set_sensitive_live(
            self.interp_switch.get_active() and not active)

    # -- Transport -------------------------------------------------------------

    def _build_transport_group(self):
        frame, box = self._group("Transport")
        row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)

        play_btn = Gtk.Button(label="Play")
        play_btn.connect("clicked", lambda b: self._set_state(Gst.State.PLAYING))
        pause_btn = Gtk.Button(label="Pause")
        pause_btn.connect("clicked", lambda b: self._set_state(Gst.State.PAUSED))
        stop_btn = Gtk.Button(label="Stop")
        stop_btn.connect("clicked", lambda b: self._set_state(Gst.State.NULL))

        row.append(play_btn)
        row.append(pause_btn)
        row.append(stop_btn)
        box.append(row)
        return frame

    def _set_state(self, state):
        self.pipeline.set_state(state)
        self.status_lbl.set_label(f"Stan: {state.value_nick}")

    # -- rebind po reconnect ------------------------------------------------

    def _rebind_prop_widgets(self):
        self.freq_slider.element = self.sdrsrc
        self.freq_offset_slider.element = self.sdrdemod
        self.gain_slider.element = self.sdrsrc
        self.auto_gain_switch.element = self.sdrsrc
        self.auto_gain_target_slider.element = self.sdrsrc
        self.if_bw_slider.element = self.sdrdemod
        self.auto_bw_switch.element = self.sdrdemod
        self.denoise_switch.element = self.sdrdenoise
        self.threshold_slider.element = self.sdrdenoise
        self.alpha_up_slider.element = self.sdrdenoise
        self.alpha_down_slider.element = self.sdrdenoise
        self.interp_switch.element = self.sdrdenoise
        self.auto_interp_switch.element = self.sdrdenoise
        self.interp_strength_slider.element = self.sdrdenoise

        for w, prop in [
            (self.freq_slider, "frequency"),
            (self.freq_offset_slider, "freq-offset"),
            (self.gain_slider, "gain"),
            (self.auto_gain_target_slider, "auto-gain-target-db"),
            (self.if_bw_slider, "if-bandwidth"),
            (self.threshold_slider, "threshold-db"),
            (self.alpha_up_slider, "alpha-up"),
            (self.alpha_down_slider, "alpha-down"),
            (self.interp_strength_slider, "interp-strength"),
        ]:
            w.element.set_property(prop, w.scale.get_value())

        for w, prop in [
            (self.auto_gain_switch, "auto-gain"),
            (self.auto_bw_switch, "auto-bandwidth"),
            (self.denoise_switch, "enabled"),
            (self.interp_switch, "interpolate"),
            (self.auto_interp_switch, "auto-interpolate"),
        ]:
            w.element.set_property(prop, w.get_active())

    # ============================================================
    # Prawidlowe zamkniecie sesji GST + TCP
    # ============================================================

    def _graceful_pipeline_stop(self):
        if self.pipeline is None:
            return
        self.pipeline.set_state(Gst.State.NULL)
        self.pipeline.get_state(Gst.CLOCK_TIME_NONE)
        self.pipeline = None

    def _on_close_request(self, *_args):
        self._closing = True
        self._graceful_pipeline_stop()
        return False

    def _install_signal_handlers(self):
        for sig in (signal.SIGINT, signal.SIGTERM):
            GLib.unix_signal_add(GLib.PRIORITY_HIGH, sig, self._on_unix_signal, sig)

    def _on_unix_signal(self, sig):
        print(f"[sdr_gui] Sygnał {sig}, zamykam sesję GST i TCP...", file=sys.stderr)
        self._closing = True
        self._graceful_pipeline_stop()
        self.get_application().quit()
        return GLib.SOURCE_REMOVE


class SdrApp(Adw.Application):
    def __init__(self, host, port, frequency, sample_rate):
        super().__init__(application_id="pl.local.sdrfmgui")
        self._host = host
        self._port = port
        self._frequency = frequency
        self._sample_rate = sample_rate

    def do_activate(self):
        win = SdrWindow(self, self._host, self._port, self._frequency,
                         self._sample_rate)
        win.present()


def main():
    import argparse
    p = argparse.ArgumentParser(description="GTK4 GUI dla sdrsrc/sdrdemod/sdrdenoise")
    p.add_argument("--host", default="192.168.1.1")
    p.add_argument("--port", type=int, default=1234)
    p.add_argument("--frequency", default="92.0M",
                    help="np. 92.1M, 92100k, 92100000")
    p.add_argument("--sample-rate", type=int, default=250000)
    args = p.parse_args()

    try:
        freq = parse_freq(args.frequency)
    except ValueError as e:
        print(f"Błąd: {e}", file=sys.stderr)
        return 1

    app = SdrApp(args.host, args.port, freq, args.sample_rate)
    return app.run(None)


if __name__ == "__main__":
    sys.exit(main())
