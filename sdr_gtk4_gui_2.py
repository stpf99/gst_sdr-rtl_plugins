#!/usr/bin/env python3
"""
sdr_gtk4_gui.py  (v2)

GTK4 control panel + widmo/wodospad dla pipeline'u:
  sdrsrc ! tee ! [queue -> sdrdemod -> audioconvert -> audioresample -> sdrdenoise -> queue -> autoaudiosink]
                [queue -> appsink (surowe IQ, do FFT/wodospadu)]

Nowosci w v2:
  * Wpisywanie czestotliwosci w Hz/kHz/MHz recznie ("92.1M", "92100k", "92100000").
  * Widmo/wodospad z surowego IQ (appsink, bez zmian w C) z:
      - pojedynczym kliknieciem -> strojenie na wskazana stacje (retune sdrsrc.frequency, live)
      - przeciagnieciem (drag) -> zaznaczenie pasma -> GUI liczy sugerowane if-bandwidth
        i freq-offset (NCO), pokazuje w panelu "Sugestie" i aplikuje dopiero po kliknieciu
        "Zastosuj sugestie" (live, bez restartu pipeline'u)
  * Prawidlowe zamkniecie sesji: window close-request i SIGINT/SIGTERM robia
    pipeline.set_state(NULL) + synchroniczne get_state() zanim proces sie zakonczy,
    zeby gniazdo TCP zdazylo dostac shutdown()+close() z C (gst_sdr_src_stop).
    UWAGA: crash rtl_tcp z Twojego loga (assertion w threads_posix.h) dzieje sie
    WEWNATRZ procesu serwera rtl_tcp przy jego wlasnym signal-handlerze - to nie
    jest cos co da sie naprawic z klienta GStreamer. Czysty shutdown po naszej
    stronie zmniejsza ryzyko trafienia w ten wyscig, ale realna naprawa to
    zaktualizowanie binarki rtl_tcp (nowszy librtlsdr/rtl-sdr-blog fork).

Wymaga: python3-gi, gir1.2-gtk-4.0, gir1.2-adw-1, gir1.2-gst-plugins-base-1.0,
        python3-numpy, oraz zbudowanego pluginu gst-sdr-plugins.
"""

import re
import sys
import signal

import gi

gi.require_version("Gtk", "4.0")
gi.require_version("Adw", "1")
gi.require_version("Gst", "1.0")
gi.require_version("GdkPixbuf", "2.0")

from gi.repository import Gtk, Adw, Gst, GLib, GdkPixbuf, Gdk  # noqa: E402

import numpy as np  # noqa: E402

Gst.init(None)


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
    """Zwraca czestotliwosc w Hz (int) albo rzuca ValueError z czytelnym opisem."""
    m = _FREQ_RE.match(text.strip().replace(",", "."))
    if not m:
        raise ValueError(
            f"Nie rozumiem częstotliwości: {text!r}. "
            f"Użyj np. 92.1M, 92100k albo 92100000")
    val = float(m.group(1))
    unit = (m.group(2) or "").upper()
    return int(round(val * _FREQ_MULT[unit]))


def fmt_freq(hz):
    """Ladny zapis Hz -> np. '92.100 MHz'."""
    if hz >= 1e6:
        return f"{hz / 1e6:.3f} MHz"
    if hz >= 1e3:
        return f"{hz / 1e3:.1f} kHz"
    return f"{hz:.0f} Hz"


# ==========================================================================
# Prosty kolormap dB -> RGB (niebieski/cyan/zielony/zolty/czerwony)
# ==========================================================================
def db_to_rgb(norm):
    """norm w [0,1] -> (r,g,b) uint8."""
    norm = np.clip(norm, 0.0, 1.0)
    stops = np.array([
        [0.00, 8, 8, 40],
        [0.25, 20, 30, 140],
        [0.50, 20, 160, 160],
        [0.70, 230, 220, 40],
        [1.00, 230, 30, 30],
    ], dtype=float)
    r = np.interp(norm, stops[:, 0], stops[:, 1])
    g = np.interp(norm, stops[:, 0], stops[:, 2])
    b = np.interp(norm, stops[:, 0], stops[:, 3])
    return np.stack([r, g, b], axis=-1).astype(np.uint8)


# ==========================================================================
# Widget widma/wodospadu
# ==========================================================================
class WaterfallView(Gtk.DrawingArea):
    """
    Wodospad zasilany przez push_row(mag_db) (numpy 1D, dlugosc = disp_bins).
    Klikniecie -> on_tune(freq_hz).
    Przeciagniecie -> on_band_select(freq_lo_hz, freq_hi_hz).
    """

    def __init__(self, disp_bins=300, hist=180):
        super().__init__()
        self.disp_bins = disp_bins
        self.hist = hist
        self.set_content_width(600)
        self.set_content_height(220)
        self.set_hexpand(True)

        self.img = np.zeros((hist, disp_bins, 3), dtype=np.uint8)
        self.center_freq = 92_000_000
        self.span = 250_000  # = sample-rate sdrsrc

        self.on_tune = None
        self.on_band_select = None

        self.set_draw_func(self._on_draw)

        click = Gtk.GestureClick()
        click.connect("pressed", self._on_press)
        click.connect("released", self._on_release)
        self.add_controller(click)

        drag = Gtk.GestureDrag()
        drag.connect("drag-begin", self._on_drag_begin)
        drag.connect("drag-update", self._on_drag_update)
        drag.connect("drag-end", self._on_drag_end)
        self.add_controller(drag)

        self._drag_start_x = None
        self._drag_cur_x = None
        self._widget_width = 600

        motion = Gtk.EventControllerMotion()
        motion.connect("motion", self._on_motion)
        self.add_controller(motion)
        self._hover_freq = None
        self.freq_label_cb = None  # wywolywane z aktualna czestotliwoscia pod kursorem

    # -- dane -------------------------------------------------------------

    def set_axis(self, center_freq, span):
        self.center_freq = center_freq
        self.span = span

    def push_row(self, mag_db):
        """mag_db: numpy array dlugosci disp_bins, wartosci w dB (juz przycietych)."""
        lo, hi = np.percentile(mag_db, [5, 99])
        if hi - lo < 1e-6:
            hi = lo + 1.0
        norm = (mag_db - lo) / (hi - lo)
        rgb_row = db_to_rgb(norm)
        self.img = np.roll(self.img, 1, axis=0)
        self.img[0] = rgb_row
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

        # zaznaczenie przeciagania
        if self._drag_start_x is not None and self._drag_cur_x is not None:
            x0 = min(self._drag_start_x, self._drag_cur_x)
            x1 = max(self._drag_start_x, self._drag_cur_x)
            cr.set_source_rgba(1, 1, 1, 0.25)
            cr.rectangle(x0, 0, x1 - x0, height)
            cr.fill()
            cr.set_source_rgba(1, 1, 1, 0.8)
            cr.rectangle(x0, 0, x1 - x0, height)
            cr.set_line_width(1.5)
            cr.stroke()

        # linia srodka (czestotliwosc centralna)
        cr.set_source_rgba(1, 1, 1, 0.5)
        cr.move_to(width / 2, 0)
        cr.line_to(width / 2, height)
        cr.set_line_width(1.0)
        cr.stroke()

    # -- mapowanie pixel <-> Hz --------------------------------------------

    def _x_to_freq(self, x):
        frac = x / max(self._widget_width, 1)
        return self.center_freq - self.span / 2.0 + frac * self.span

    # -- gesty ---------------------------------------------------------------

    def _on_press(self, gesture, n_press, x, y):
        pass  # obsluga w GestureClick 'released' zeby odroznic od draga

    def _on_release(self, gesture, n_press, x, y):
        if self._drag_start_x is None:  # zwykly klik, bez draga
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
            # ruch za maly zeby traktowac jako zaznaczenie pasma - to byl zwykly klik
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
        self._hover_freq = self._x_to_freq(x)
        if self.freq_label_cb:
            self.freq_label_cb(self._hover_freq)


# ==========================================================================
# Pomocnicze widgety property (bez zmian logiki wzgledem v1)
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
        """Ustawia wartosc I odpala set_property (uzywane przez 'Zastosuj sugestie')."""
        self.scale.set_value(v)  # to samo w sobie odpali _on_changed


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


# ==========================================================================
# Glowne okno
# ==========================================================================
FFT_SIZE = 1024
DISP_BINS = 300
IQ_PROCESS_EVERY_N = 3  # przerzedzanie buforow IQ zeby nie zamulic GUI


class SdrWindow(Adw.ApplicationWindow):
    def __init__(self, app, host, port, frequency, sample_rate):
        super().__init__(application=app, title="SDR FM Control")
        self.set_default_size(760, 980)

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

        self._build_pipeline(host, port, frequency, sample_rate)
        self._build_ui()
        self._start()

        self.connect("close-request", self._on_close_request)
        self._install_signal_handlers()

    # ---------------- Pipeline ---------------------------------------------

    def _build_pipeline(self, host, port, frequency, sample_rate):
        desc = (
            f'sdrsrc name=src mode=tcp host={host} port={port} '
            f'frequency={frequency} sample-rate={sample_rate} gain=4.0 '
            f'auto-gain=false auto-gain-target-db=-18.0 '
            f'! tee name=t '
            f'  t. ! queue name=q1 max-size-buffers=0 max-size-bytes=0 '
            f'         max-size-time=200000000 leaky=downstream '
            f'       ! sdrdemod name=demod mode=fm stereo=true max-deviation=75000 '
            f'         audio-rate=48000 audio-cutoff=15000 tau=50 freq-offset=0 '
            f'         if-bandwidth=0 auto-bandwidth=false '
            f'       ! audioconvert ! audioresample '
            f'       ! audio/x-raw,rate=48000,channels=1 '
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

    # -- IQ -> FFT -> wodospad (callback leci w watku streamingowym GStreamera!) --

    def _on_new_iq_sample(self, sink):
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.OK

        self._iq_buf_counter += 1
        if self._iq_buf_counter % IQ_PROCESS_EVERY_N != 0:
            return Gst.FlowReturn.OK  # przerzedzenie, zeby nie zamulic GUI

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
        cplx = iq[:, 0] + 1j * iq[:, 1]
        windowed = cplx * np.hanning(FFT_SIZE)
        spec = np.fft.fftshift(np.fft.fft(windowed))
        mag_db = 20.0 * np.log10(np.abs(spec) + 1e-9)

        # przerobienie FFT_SIZE binow na DISP_BINS do wyswietlenia (usrednianie grup)
        disp = mag_db.reshape(DISP_BINS, FFT_SIZE // DISP_BINS).mean(axis=1) \
            if FFT_SIZE % DISP_BINS == 0 else \
            np.interp(np.linspace(0, FFT_SIZE - 1, DISP_BINS),
                      np.arange(FFT_SIZE), mag_db)

        GLib.idle_add(self._push_waterfall_row, disp)
        return Gst.FlowReturn.OK

    def _push_waterfall_row(self, disp_row):
        if self._closing:
            return False
        self.waterfall.set_axis(self._current_freq(), self._sample_rate)
        self.waterfall.push_row(disp_row)
        return False  # jednorazowy idle callback

    def _current_freq(self):
        try:
            return self.sdrsrc.get_property("frequency")
        except Exception:
            return self._init_freq

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

    def _reconnect(self, host, port, frequency):
        self.status_lbl.set_label("Przełączam źródło...")
        self._graceful_pipeline_stop()
        self._build_pipeline(host, port, frequency, self._sample_rate)
        self._rebind_prop_widgets()
        self._start()
        self.status_lbl.set_label("Połączono")

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

    # -- Widmo/wodospad ---------------------------------------------------------

    def _build_waterfall_group(self):
        frame, box = self._group("Widmo / wodospad")

        self.waterfall = WaterfallView(disp_bins=DISP_BINS, hist=180)
        self.waterfall.set_axis(self._init_freq, self._sample_rate)
        self.waterfall.on_tune = self._on_waterfall_tune
        self.waterfall.on_band_select = self._on_waterfall_band_select
        self.waterfall.freq_label_cb = self._on_waterfall_hover
        box.append(self.waterfall)

        self.hover_freq_lbl = Gtk.Label(label="—", xalign=0)
        self.hover_freq_lbl.add_css_class("dim-label")
        box.append(self.hover_freq_lbl)

        hint = Gtk.Label(
            label="Klik = strojenie na wskazaną częstotliwość. "
                  "Przeciągnij = zaznacz pasmo stacji -> zobacz sugestie niżej.",
            xalign=0, wrap=True)
        hint.add_css_class("dim-label")
        box.append(hint)

        # panel sugestii wypelniany po zaznaczeniu pasma na wodospadzie
        self.suggest_frame = Gtk.Frame(label="Sugestie dla zaznaczonego pasma")
        sbox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6,
                        margin_top=8, margin_bottom=8,
                        margin_start=8, margin_end=8)
        self.suggest_frame.set_child(sbox)
        self.suggest_lbl = Gtk.Label(label="Zaznacz pasmo na wodospadzie...",
                                      xalign=0, wrap=True)
        sbox.append(self.suggest_lbl)
        self.suggest_apply_btn = Gtk.Button(label="Zastosuj sugestie")
        self.suggest_apply_btn.set_sensitive(False)
        self.suggest_apply_btn.connect("clicked", self._on_apply_suggestion)
        sbox.append(self.suggest_apply_btn)
        box.append(self.suggest_frame)

        self._pending_suggestion = None
        return frame

    def _on_waterfall_hover(self, freq):
        self.hover_freq_lbl.set_label(f"Kursor: {fmt_freq(freq)}")

    def _on_waterfall_tune(self, freq_hz):
        freq_hz = int(round(freq_hz))
        self.sdrsrc.set_property("frequency", freq_hz)
        self.sdrdemod.set_property("freq-offset", 0.0)
        self.freq_slider.set_value_quiet(freq_hz)
        self.freq_offset_slider.set_value_quiet(0.0)
        self.freq_entry.set_text(fmt_freq(freq_hz))
        self.status_lbl.set_label(f"Nastrojono: {fmt_freq(freq_hz)}")

    def _on_waterfall_band_select(self, f_lo, f_hi):
        center_freq = self._current_freq()
        bandwidth = abs(f_hi - f_lo)
        band_center = (f_lo + f_hi) / 2.0
        freq_offset = band_center - center_freq

        # przytnij do zakresow property (potwierdzonych w C)
        if_bw_suggest = float(np.clip(bandwidth, 0.0, 200000.0))
        freq_offset_suggest = float(np.clip(freq_offset, -100000.0, 100000.0))

        self._pending_suggestion = (if_bw_suggest, freq_offset_suggest)
        self.suggest_lbl.set_label(
            f"Zaznaczone pasmo: {fmt_freq(f_lo)} – {fmt_freq(f_hi)} "
            f"(szerokość ≈ {fmt_freq(bandwidth)})\n"
            f"Sugerowane if-bandwidth: {fmt_freq(if_bw_suggest)}\n"
            f"Sugerowany freq-offset (NCO): {freq_offset_suggest:+.0f} Hz\n"
            f"(auto-bandwidth zostanie wyłączone, żeby ręczna wartość nie była "
            f"od razu nadpisana)")
        self.suggest_apply_btn.set_sensitive(True)

    def _on_apply_suggestion(self, btn):
        if not self._pending_suggestion:
            return
        if_bw, freq_off = self._pending_suggestion
        self.auto_bw_switch.set_active_live(False)
        self.if_bw_slider.set_value_live(if_bw)
        self.freq_offset_slider.set_value_live(freq_off)
        self.status_lbl.set_label("Zastosowano sugestię pasma")

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

        self.stereo_switch = PropSwitch(self.sdrdemod, "stereo", "Stereo")
        box.append(self.stereo_switch)

        return frame

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
        self.status_lbl.set_label(f"Nastrojono: {fmt_freq(hz)}")

    def _on_freq_slider_moved(self, scale):
        # suwak sam ustawia property (PropSlider._on_changed); tu tylko
        # synchronizujemy pole tekstowe zeby pokazywalo aktualna wartosc
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
            self.sdrdemod, "auto-bandwidth", "Auto-Bandwidth (S/N hunt, tylko FM)",
            on_toggle=self._on_auto_bw_toggle)
        box.append(self.auto_bw_switch)

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
        self.stereo_switch.element = self.sdrdemod
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
            (self.stereo_switch, "stereo"),
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
        """
        Przechodzi PLAYING -> NULL i CZEKA (get_state, blokujaco) na
        faktyczne zakonczenie przejscia, zanim cokolwiek innego sie stanie.
        Dzieki temu gst_sdr_src_stop() (shutdown(SHUT_RDWR)+close() na
        sock_fd) na pewno zdazy wykonac sie w calosci PRZED wyjsciem z
        procesu GUI - to minimalizuje ryzyko, ze serwer rtl_tcp dostanie
        polowiczne/nieoczekiwane zamkniecie socketu w trakcie obslugi
        naszej ostatniej komendy.
        """
        if self.pipeline is None:
            return
        self.pipeline.set_state(Gst.State.NULL)
        # blokujace czekanie (Gst.CLOCK_TIME_NONE = bez limitu) az stan
        # faktycznie osiagnie NULL - w tym momencie stop() w C juz sie wykonal
        self.pipeline.get_state(Gst.CLOCK_TIME_NONE)
        self.pipeline = None

    def _on_close_request(self, *_args):
        self._closing = True
        self._graceful_pipeline_stop()
        return False  # pozwol oknu sie zamknac

    def _install_signal_handlers(self):
        """Lapie Ctrl+C / SIGTERM w terminalu, zeby zrobic ten sam czysty
        shutdown zamiast pozwolic procesowi umrzec z gniazdem w locie."""
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
