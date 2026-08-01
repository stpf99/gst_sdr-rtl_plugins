#!/usr/bin/env python3
"""
sdr_gtk4_gui.py  (v6)

GTK4 control panel dla pipeline'u sdrsrc/sdrdemod/sdrdenoise + widmo I/Q +
auto-korekta 5s + true stereo + rozszerzony lancuch FX audio + presety +
zakladka Stacje.

ZMIANY v6 (wzgledem v4/v5):
  * FIX: auto-korekta potrafila wywolywac sluchalny "klik" przy skokach,
    bo (1) pojedynczy bin szumu byl traktowany jak realny szczyt, (2)
    korekta liczona byla ze zwyklej sredniej (wrazliwej na outliery) i
    (3) aplikowana byla NATYCHMIAST (skok wartosci). Naprawa: odrzucanie
    waskich (1-binowych) "szczytow" jako szumu (patrz MIN_PEAK_WIDTH_BINS
    w _find_peak), mediana zamiast sredniej w _update_auto_correct, oraz
    plynne dochodzenie do wartosci docelowej (AUTO_CORRECT_SLEW) zamiast
    set_value_live() na sztywno.
  * NOWE FX (przeniesione z CarbonFX): Tape (ATS-1 symulacja tasmy),
    Spatial FX (poszerzenie stereo + Haas + saturacja), Phantom Stereo
    (crossfeed/Haas z presetami). Kazdy to osobny multi-elementowy
    lancuch GStreamer (patrz FX_DEFS[...]["elements"]), wlaczany/wylaczany
    restartem (zmiana struktury grafu), parametry WEWNATRZ dzialaja live.
  * EQ rozszerzony o presety (Flat/Rock/Pop/Wokal/Bas/Podcast/Tre),
    suwak "tilt" (nachylenie charakterystyki) i tryb Auto (wybor
    dominujacego pasma z elementu GStreamer 'spectrum' i lagodna,
    wygladzana korekta EQ - zeby nie trzaskac przy zmianach).
  * NOWA zakladka "Stacje" (pierwsza w kolejnosci): lista stacji
    (JSON), panel info (nazwa/czest./aktywne funkcje), oraz
    jednoklatkowe widmo cz.-b. (cairo) z kropkami TYLKO na
    potwierdzonych szczytach (kilka sasiednich binow ponad szumem -
    pojedyncze odrzucane, bo sa nieczytelne dla audio). Ta sama funkcja
    filtrujaca szum zasila auto-korekte.
  * Reorganizacja sidebaru na bardziej ergonomiczna: Stacje, Zrodlo,
    Strojenie, Gain, Pasmo IF, Widmo, Auto-korekta, Denoise, Equalizer,
    FX Kreatywne (Tape/Spatial/Phantom), Dynamika (Kompresor/Echo),
    Presety, Transport. Lekki CSS + ikony w sidebarze.

UKLAD (v4): sidebar po lewej (Gtk.StackSidebar, pionowo) z kategoriami,
zawartosc kategorii obok (jak w zalaczonym pluginie GNOME sdr-radio@your-domain
prefs.js: Adw.PreferencesPage per kategoria, kazdy wiersz z tytulem-jako-
-etykieta + tooltip z zakresem/opisem).

KATEGORIE:
  Zrodlo          - host/port/sample-rate (restart)
  Widmo           - wodospad I/Q, prog wykrywania, zaznaczanie pasma
  Auto-korekta    - bufor 5s, wlacz/wylacz, telemetria
  Strojenie       - czestotliwosc (recznie kHz/MHz), NCO offset, stereo
  Gain / AGC      - gain reczny + auto-gain (C, live)
  Pasmo IF        - if-bandwidth reczne + auto-bandwidth (C, live)
  Denoise         - threshold/alpha/interpolacja (live)
  FX Audio        - opcjonalny equalizer-10bands / audiodynamic (kompresor) /
                    audioecho, wlaczane/wylaczane przez restart, parametry
                    W SRODKU live (bez restartu)
  Presety         - podglad aktualnego gst-launch, zapis/wczytanie/usuwanie
                    presetow (JSON w ~/.config/sdr_gtk4_gui/presets.json),
                    ustawienie "domyslnego" presetu wczytywanego przy starcie

ZASADA live vs restart (bez zmian wzgledem v3, tylko rozszerzona o FX):
  * ZYWO (bez restartu): gain, auto-gain(+target), if-bandwidth,
    auto-bandwidth, freq-offset, frequency, threshold-db, alpha-up/down,
    interpolate/auto-interpolate/interp-strength, oraz WSZYSTKIE parametry
    WEWNATRZ juz wlaczonego FX (band0..9 equalizera, ratio/threshold
    kompresora, delay/intensity/feedback echa) - te elementy nie zmieniaja
    liczby kanalow/formatu wiec live-set jest bezpieczny.
  * RESTART (pelny rebuild pipeline'u): host/port, stereo (bug w C - patrz
    komentarz przy _restart_for_stereo), WLACZENIE/WYLACZENIE danego FX
    (bo to zmiana STRUKTURY grafu, nie parametru).

Wymaga: python3-gi, gir1.2-gtk-4.0, gir1.2-adw-1, gir1.2-gst-plugins-base-1.0,
        python3-numpy, gst-plugins-good (equalizer-10bands, audiodynamic,
        audioecho), oraz zbudowanego pluginu gst-sdr-plugins.
"""

import re
import sys
import time
import json
import signal
from pathlib import Path
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
IQ_PROCESS_EVERY_N = 3
PEAK_BUFFER_SECONDS = 5.0
AUTO_APPLY_MIN_INTERVAL = 0.5
DC_GUARD_BINS = 6
DEFAULT_THRESHOLD_MARGIN_DB = 6.0

# -- anti-klik / anti-szum (v6) --------------------------------------------
# Szczyt wezszy niz tyle binow (wlacznie) ponad progiem jest uznawany za
# pojedynczy szum (nie realny sygnal) i odrzucany - nie trafia do bufora
# auto-korekty ani nie jest rysowany jako kropka na widmie stacji.
# 2 = odrzuca izolowane 1-binowe piki szumu, przepuszcza watle/waskie stacje.
MIN_PEAK_WIDTH_BINS = 2
# Margines dB uzywany TYLKO do pomiaru szerokosci szczytu (if-bandwidth).
# Nizszy niz DEFAULT_THRESHOLD_MARGIN_DB, zeby nie zawężac sugestii pasma
# do samego "rdzenia" sygnalu (wczesniej 6 dB = za rygorystycznie).
PEAK_WIDTH_MARGIN_DB = 3.0
# Zapas do sugestii if-bandwidth (FM: boczne wstegi / guard).
IF_BW_PAD = 1.25
# Frakcja dystansu do wartosci sugerowanej pokonywana przy KAZDEJ aplikacji
# auto-korekty (co AUTO_APPLY_MIN_INTERVAL). Mniejsza wartosc = lagodniej,
# ale wolniej dogania. 0.35 daje plynne dojscie w ~1-2s bez skoku/kliku.
AUTO_CORRECT_SLEW = 0.35
# Minimalny odstep miedzy dwoma zgloszonymi szczytami na widmie stacji
# (w binach wyswietlanego/zbinowanego widma), zeby nie dublowac kropek.
STATION_PEAK_MIN_SEP = 6

PRESETS_PATH = Path(GLib.get_user_config_dir()) / "sdr_gtk4_gui" / "presets.json"
STATIONS_PATH = Path(GLib.get_user_config_dir()) / "sdr_gtk4_gui" / "stations.json"
DEFAULT_PRESET_KEY = "__default__"

FX_ORDER = ("eq", "comp", "echo", "tape", "spatial", "phantom")

# Kazdy FX to lista elementow GStreamer (gst_elem, name_suffix, extra_props)
# laczonych w tej kolejnosci znakiem "!" - dziala jednakowo dla FX
# jednoelementowych (comp/echo/eq) i wieloelementowych lancuchow
# (tape/spatial/phantom, przeniesionych z CarbonFX). Nazwa elementu w
# pipeline to zawsze f"fx_{key}_{suffix}".
FX_DEFS = {
    "eq": {
        "label": "Equalizer 10-band (presety / tilt / auto)",
        "elements": [
            ("spectrum", "spectrum",
             "bands=10 threshold=-70 post-messages=true interval=200000000"),
            ("equalizer-10bands", "main", ""),
        ],
        "band_labels": ["29 Hz", "59 Hz", "119 Hz", "237 Hz", "474 Hz",
                         "947 Hz", "1.9 kHz", "3.8 kHz", "7.5 kHz", "15 kHz"],
    },
    "comp": {
        "label": "Kompresor dynamiki (audiodynamic)",
        "elements": [("audiodynamic", "main",
                      "mode=compressor characteristics=soft-knee")],
        "params": [
            ("ratio", "Ratio", 1.0, 8.0, 0.1, 1, ":1",
             "Stosunek kompresji - wiekszy = mocniej wyrownuje glosnosc"),
            ("threshold", "Threshold", 0.0, 1.0, 0.01, 2, "",
             "Prog powyzej ktorego zaczyna dzialac kompresja (0..1 liniowo)"),
        ],
    },
    "echo": {
        "label": "Echo / pogłos (audioecho)",
        "elements": [("audioecho", "main", "max-delay=500000000")],
        "params": [
            ("delay", "Delay", 1_000_000, 500_000_000, 1_000_000, 0, " ns",
             "Opoznienie echa w nanosekundach (120000000 ~ 120ms)"),
            ("intensity", "Intensity", 0.0, 1.0, 0.01, 2, "",
             "Sila efektu echa"),
            ("feedback", "Feedback", 0.0, 1.0, 0.01, 2, "",
             "Ile echa wraca do wejscia (wiecej = dluzej brzmi)"),
        ],
    },
    "tape": {
        "label": "🎛 ATS-1 Tape Simulation",
        "elements": [
            ("audiodynamic", "sat", "characteristics=soft-knee mode=compressor"),
            ("volume", "gain", ""),
            ("equalizer-3bands", "tone", ""),
        ],
    },
    "spatial": {
        "label": "🌌 Spatial FX (poszerzenie stereo)",
        "elements": [
            ("audiodynamic", "sat", "characteristics=hard-knee mode=compressor"),
            ("audioconvert", "conv1", ""),
            ("stereo", "sw", "stereo=1.0"),
            ("audioconvert", "conv2", ""),
            ("audioecho", "echo", "max-delay=500000000"),
        ],
    },
    "phantom": {
        "label": "👻 Phantom Stereo (crossfeed / Haas)",
        "elements": [
            ("audioconvert", "conv1", ""),
            ("stereo", "sw", "stereo=1.0"),
            ("audioconvert", "conv2", ""),
            ("audioecho", "echo", "max-delay=500000000"),
        ],
    },
}

EQ_PRESETS = {
    "Flat":         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    "Rock":         [4, 3, 1, -1, -2, 0, 1, 2, 3, 4],
    "Pop":          [-1, 0, 2, 3, 2, 0, -1, -1, 0, 1],
    "Wokal / Mowa": [-3, -2, -1, 1, 3, 3, 2, 1, 0, -1],
    "Bas Boost":    [6, 5, 3, 1, 0, 0, 0, 0, 0, 0],
    "Podcast Mono": [-4, -2, 0, 2, 3, 2, 1, 0, -1, -2],
    "Treble Boost": [0, 0, 0, 0, 0, 1, 2, 3, 4, 5],
}

# (width_slider 0-60, haas_ms 0-20, sat_slider 10-80) - jak w CarbonFX
SPATIAL_PRESETS = {
    "Flat":   (0, 0, 10),
    "Studio": (0, 8, 15),
    "Wide":   (0, 12, 20),
}

PHANTOM_PRESETS = {
    "Off":          {"mode": "off",       "width": 5,  "delay": 0,  "blend": 0,
                      "desc": "Brak przetwarzania - sygnal oryginalny"},
    "Słuchawki":    {"mode": "crossfeed", "width": 5,  "delay": 0,  "blend": 8,
                      "desc": "Subtelny crossfeed sluchawkowy"},
    "TV Wide":      {"mode": "haas",      "width": 12, "delay": 6,  "blend": 0,
                      "desc": "Delikatne rozszerzenie - Haas 6ms"},
    "Podcast Mono": {"mode": "haas",      "width": 10, "delay": 8,  "blend": 0,
                      "desc": "Mono naturalnie, zachowuje srodek"},
    "Koncert":      {"mode": "haas",      "width": 15, "delay": 12, "blend": 5,
                      "desc": "Lekka scena koncertowa"},
    "Studio":       {"mode": "crossfeed", "width": 5,  "delay": 2,  "blend": 5,
                      "desc": "Minimalne crossfeed, bez koloryzacji"},
}


# ==========================================================================
# Parsowanie / formatowanie czestotliwosci
# ==========================================================================
_FREQ_RE = re.compile(
    r'^\s*([0-9]*\.?[0-9]+)\s*(GHZ|MHZ|KHZ|HZ|G|M|K)?\s*$', re.IGNORECASE)
_FREQ_MULT = {"HZ": 1.0, "": 1.0, "K": 1e3, "KHZ": 1e3, "M": 1e6, "MHZ": 1e6,
              "G": 1e9, "GHZ": 1e9}


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


def range_hint(lo, hi, suffix="", extra=""):
    txt = f"Zakres: {lo:g}{suffix} – {hi:g}{suffix}"
    if extra:
        txt = f"{extra}\n{txt}"
    return txt


def find_validated_peaks(mag_db, margin_db=DEFAULT_THRESHOLD_MARGIN_DB,
                          min_width=MIN_PEAK_WIDTH_BINS,
                          min_sep=STATION_PEAK_MIN_SEP):
    """
    Zwraca liste indeksow binow uznanych za PRAWDZIWE szczyty sygnalu.

    Pojedynczy bin ponad progiem to zazwyczaj szum (dla audio nieczytelny
    i myllacy) - odrzucamy go, wymagajac zeby lokalne maksimum bylo
    otoczone co najmniej `min_width` sasiednimi binami rowniez ponad
    progiem. To samo kryterium chroni auto-korekte przed "klikami"
    (patrz MIN_PEAK_WIDTH_BINS) i zasila kropki na widmie stacji.
    """
    n = len(mag_db)
    if n < 3:
        return []
    floor = float(np.median(mag_db))
    thresh = floor + margin_db
    mask = mag_db > thresh
    peaks = []
    i = 1
    while i < n - 1:
        if mask[i] and mag_db[i] >= mag_db[i - 1] and mag_db[i] >= mag_db[i + 1]:
            left = i
            while left > 0 and mask[left - 1]:
                left -= 1
            right = i
            while right < n - 1 and mask[right + 1]:
                right += 1
            width = right - left + 1
            if width >= min_width and (not peaks or (i - peaks[-1]) >= min_sep):
                peaks.append(i)
            i = right + 1
        else:
            i += 1
    return peaks


# ==========================================================================
# Widget widma/wodospadu - MONOCHROMATYCZNY
# ==========================================================================
class WaterfallView(Gtk.DrawingArea):
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

    def set_axis(self, center_freq, span):
        self.center_freq = center_freq
        self.span = span

    def set_peak(self, freq_hz):
        self._peak_freq = freq_hz
        self.queue_draw()

    def push_row(self, mag_db):
        noise_floor = float(np.median(mag_db))
        threshold = noise_floor + self.threshold_margin_db
        mask = mag_db > threshold
        row = np.zeros((self.disp_bins, 3), dtype=np.uint8)
        row[mask] = (255, 255, 255)
        self.img = np.roll(self.img, 1, axis=0)
        self.img[0] = row
        self.queue_draw()

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

        cr.set_source_rgba(0.4, 0.4, 0.4, 0.8)
        cr.move_to(width / 2, 0)
        cr.line_to(width / 2, height)
        cr.set_line_width(1.0)
        cr.stroke()

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

    def _x_to_freq(self, x):
        frac = x / max(self._widget_width, 1)
        return self.center_freq - self.span / 2.0 + frac * self.span

    def _freq_to_x(self, freq):
        frac = (freq - (self.center_freq - self.span / 2.0)) / self.span
        return frac * self._widget_width

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
# Jednoklatkowe widmo cz.-b. (cairo) z kropkami TYLKO na potwierdzonych
# szczytach - dla zakladki "Stacje" (v6)
# ==========================================================================
class SpectrumSnapshotView(Gtk.DrawingArea):
    def __init__(self):
        super().__init__()
        self.set_content_width(700)
        self.set_content_height(200)
        self.set_hexpand(True)
        self._mag = None
        self._peaks = []
        self.set_draw_func(self._on_draw)

    def push_frame(self, mag_db, peaks):
        self._mag = mag_db
        self._peaks = peaks
        self.queue_draw()

    def _on_draw(self, area, cr, width, height):
        cr.set_source_rgb(0.0, 0.0, 0.0)
        cr.paint()

        if self._mag is None or len(self._mag) < 2:
            cr.set_source_rgba(0.5, 0.5, 0.5, 0.8)
            cr.move_to(10, height / 2)
            cr.show_text("Oczekiwanie na sygnał...")
            return

        mag = self._mag
        n = len(mag)
        lo = float(np.min(mag))
        hi = float(np.max(mag))
        rng = max(hi - lo, 1e-6)
        pad_top, pad_bot = height * 0.12, height * 0.08

        def y_of(v):
            return height - pad_bot - ((v - lo) / rng) * (height - pad_top - pad_bot)

        # linia widma - biala na czarnym
        cr.set_source_rgb(1.0, 1.0, 1.0)
        cr.set_line_width(1.3)
        for i, v in enumerate(mag):
            x = i / (n - 1) * width
            y = y_of(v)
            if i == 0:
                cr.move_to(x, y)
            else:
                cr.line_to(x, y)
        cr.stroke()

        # delikatne wypelnienie pod krzywa
        cr.line_to(width, height)
        cr.line_to(0, height)
        cr.close_path()
        cr.set_source_rgba(1.0, 1.0, 1.0, 0.08)
        cr.fill()

        # kropki tylko na potwierdzonych szczytach (nie pojedynczy szum)
        cr.set_source_rgb(1.0, 1.0, 1.0)
        for pk in self._peaks:
            if pk < 0 or pk >= n:
                continue
            x = pk / (n - 1) * width
            y = y_of(mag[pk])
            cr.arc(x, y - 7, 3.4, 0, 2 * 3.14159265)
            cr.fill()
            cr.arc(x, y - 7, 5.5, 0, 2 * 3.14159265)
            cr.set_source_rgba(1.0, 1.0, 1.0, 0.35)
            cr.set_line_width(1.0)
            cr.stroke()
            cr.set_source_rgb(1.0, 1.0, 1.0)


# ==========================================================================
# Pomocnicze widgety property (z opcjonalnym tooltipem/zakresem)
# ==========================================================================
class PropSlider(Gtk.Box):
    def __init__(self, element, prop_name, label, lo, hi, step, digits=2,
                 value=None, suffix="", hint=None):
        super().__init__(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        self.element = element
        self.prop_name = prop_name
        self.suffix = suffix

        lbl = Gtk.Label(label=label, xalign=0)
        lbl.set_size_request(160, -1)
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
        tip = hint or range_hint(lo, hi, suffix)
        self.scale.set_tooltip_text(tip)
        lbl.set_tooltip_text(tip)
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

    def get_value(self):
        return self.scale.get_value()


class PropSwitch(Gtk.Box):
    def __init__(self, element, prop_name, label, on_toggle=None, hint=None):
        super().__init__(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        self.element = element
        self.prop_name = prop_name
        self.on_toggle = on_toggle

        lbl = Gtk.Label(label=label, xalign=0, hexpand=True)
        if hint:
            lbl.set_tooltip_text(hint)
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
    """Suwak NIE spiety z GStreamer property (parametry czysto Pythonowe)."""

    def __init__(self, label, lo, hi, step, value, digits=1, suffix="",
                 on_change=None, hint=None):
        super().__init__(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        self.suffix = suffix
        self.on_change = on_change

        lbl = Gtk.Label(label=label, xalign=0)
        lbl.set_size_request(200, -1)
        tip = hint or range_hint(lo, hi, suffix)
        lbl.set_tooltip_text(tip)
        self.append(lbl)

        adj = Gtk.Adjustment(value=value, lower=lo, upper=hi,
                              step_increment=step, page_increment=step * 5)
        self.scale = Gtk.Scale(orientation=Gtk.Orientation.HORIZONTAL,
                                adjustment=adj, digits=digits, hexpand=True)
        self.scale.set_draw_value(False)
        self.scale.set_tooltip_text(tip)
        self.scale.connect("value-changed", self._on_changed)
        self.append(self.scale)

        self.value_lbl = Gtk.Label(label=f"{value:.{digits}f}{suffix}", width_chars=10)
        self.append(self.value_lbl)

    def _on_changed(self, scale):
        v = scale.get_value()
        self.value_lbl.set_label(f"{v:.2f}{self.suffix}")
        if self.on_change:
            self.on_change(v)

    def get_value(self):
        return self.scale.get_value()

    def set_value_quiet(self, v):
        self.scale.handler_block_by_func(self._on_changed)
        self.scale.set_value(v)
        self.value_lbl.set_label(f"{v:.2f}{self.suffix}")
        self.scale.handler_unblock_by_func(self._on_changed)


# ==========================================================================
# Presety - proste odczyt/zapis JSON na dysku
# ==========================================================================
class PresetStore:
    def __init__(self, path=PRESETS_PATH):
        self.path = path

    def load_all(self):
        try:
            return json.loads(self.path.read_text(encoding="utf-8"))
        except Exception:
            return {}

    def save_all(self, data):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text(json.dumps(data, indent=2, ensure_ascii=False),
                              encoding="utf-8")

    def save_one(self, name, state):
        data = self.load_all()
        data[name] = state
        self.save_all(data)

    def delete_one(self, name):
        data = self.load_all()
        data.pop(name, None)
        self.save_all(data)


# ==========================================================================
# Stacje - lista (nazwa, czestotliwosc) w JSON (v6)
# ==========================================================================
class StationStore:
    def __init__(self, path=STATIONS_PATH):
        self.path = path

    def load_all(self):
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
            return data if isinstance(data, list) else []
        except Exception:
            return []

    def save_all(self, data):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text(json.dumps(data, indent=2, ensure_ascii=False),
                              encoding="utf-8")

    def add(self, name, frequency):
        data = [s for s in self.load_all() if s.get("name") != name]
        data.append({"name": name, "frequency": int(frequency)})
        data.sort(key=lambda s: s["frequency"])
        self.save_all(data)

    def delete(self, name):
        data = [s for s in self.load_all() if s.get("name") != name]
        self.save_all(data)


# ==========================================================================
# Glowne okno
# ==========================================================================
class SdrWindow(Adw.ApplicationWindow):
    def __init__(self, app, host, port, frequency, sample_rate):
        super().__init__(application=app, title="SDR FM Control")
        self.set_default_size(1180, 900)

        self.pipeline = None
        self.sdrsrc = None
        self.sdrdemod = None
        self.sdrdenoise = None
        self.iqsink = None
        self.fx_elements = {}  # name -> Gst.Element, tylko dla wlaczonych FX

        self._init_host = host
        self._init_port = port
        self._init_freq = frequency
        self._sample_rate = sample_rate
        self._iq_buf_counter = 0
        self._closing = False

        self._selection_freqs = None
        self._pending_suggestion = None
        self._peak_buffer = deque()
        self._last_auto_apply = 0.0
        self._current_stereo = True

        self._fx_enabled = {k: False for k in FX_ORDER}
        # comp/echo: prop -> wartosc (bezposrednio na jedynym elemencie "main")
        self._fx_params = {"comp": {}, "echo": {}}
        # tape/spatial/phantom: wysokopoziomowe kontrolki -> mapowane przez
        # _map_tape/_map_spatial/_map_phantom na (suffix, prop) -> wartosc
        self._fx_ext_state = {
            "tape": {"drive": 50.0, "warmth": 30.0, "comp": 20.0},
            "spatial": {"width": 0.0, "haas": 0.0, "sat": 10.0},
            "phantom": {"mode": "off", "width": 5.0, "delay": 0.0, "blend": 0.0},
        }
        # eq: bazowe pasma (preset / reczna edycja) + tilt + tryb auto
        self._eq_state = {"base_bands": [0.0] * 10, "tilt": 0.0, "auto": False}
        self._eq_band_sliders = []
        self._fx_sensitive_widgets = {k: [] for k in FX_ORDER}
        self._fx_widgets = {"comp": {}, "echo": {}}  # prop -> SimpleSlider
        self._fx_enable_switches = {}

        self.presets = PresetStore()
        self.stations = StationStore()
        self._stations_cache = []

        self._build_pipeline(host, port, frequency, sample_rate, stereo=True)
        self._build_ui()
        self._start()

        self.connect("close-request", self._on_close_request)
        self._install_signal_handlers()

        self._maybe_load_default_preset()

    # ---------------- Pipeline ---------------------------------------------

    def _fx_segment_string(self):
        """
        Buduje fragment gst-launch dla wlaczonych FX, w stalej kolejnosci.
        Kazdy FX to jeden lub wiecej elementow (FX_DEFS[key]["elements"]),
        laczonych "!" - dziala jednakowo dla FX jednoelementowych
        (comp/echo/eq) i wieloelementowych lancuchow (tape/spatial/phantom).
        Nazwa kazdego elementu w pipeline: f"fx_{key}_{suffix}".
        """
        parts = []
        for key in FX_ORDER:
            if not self._fx_enabled.get(key):
                continue
            elem_strs = []
            for gst_elem, suffix, extra in FX_DEFS[key]["elements"]:
                name = f"fx_{key}_{suffix}"
                elem_strs.append(f'{gst_elem} name={name} {extra}'.strip())
            parts.append(" ! ".join(elem_strs) + " !")
        return " ".join(parts)

    def _build_pipeline(self, host, port, frequency, sample_rate, stereo=True):
        stereo_str = "true" if stereo else "false"
        self._current_stereo = stereo
        fx_segment = self._fx_segment_string()
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
            f'       ! {fx_segment} '
            f'       audioconvert ! '
            f'       queue name=q2 max-size-buffers=0 max-size-bytes=0 '
            f'         max-size-time=1000000000 '
            f'       ! autoaudiosink sync=false '
            f'  t. ! queue name=qiq leaky=downstream max-size-buffers=2 '
            f'         max-size-bytes=0 max-size-time=0 '
            f'       ! appsink name=iqsink emit-signals=true sync=false '
            f'         max-buffers=2 drop=true'
        )
        self._last_pipeline_desc = desc
        self.pipeline = Gst.parse_launch(desc)
        self.sdrsrc = self.pipeline.get_by_name("src")
        self.sdrdemod = self.pipeline.get_by_name("demod")
        self.sdrdenoise = self.pipeline.get_by_name("denoise")
        self.iqsink = self.pipeline.get_by_name("iqsink")
        self.iqsink.connect("new-sample", self._on_new_iq_sample)

        # fx_elements[key] = {suffix: Gst.Element, ...} - tylko dla wlaczonych FX
        self.fx_elements = {}
        for key in FX_ORDER:
            if self._fx_enabled.get(key):
                self.fx_elements[key] = {}
                for _gst_elem, suffix, _extra in FX_DEFS[key]["elements"]:
                    name = f"fx_{key}_{suffix}"
                    self.fx_elements[key][suffix] = self.pipeline.get_by_name(name)

        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message::error", self._on_bus_error)
        bus.connect("message::warning", self._on_bus_warning)
        bus.connect("message::eos", self._on_bus_eos)
        bus.connect("message::element", self._on_bus_element)

    def _start(self):
        self.pipeline.set_state(Gst.State.PLAYING)

    # -- IQ -> FFT (watek streamingowy) --------------------------------------

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

        spec_i = np.fft.fftshift(np.fft.fft(i_ch * win))
        spec_q = np.fft.fftshift(np.fft.fft(q_ch * win))
        mag_i = 20.0 * np.log10(np.abs(spec_i) + 1e-9)
        mag_q = 20.0 * np.log10(np.abs(spec_q) + 1e-9)
        disp_i = self._bin_down(mag_i)
        disp_q = self._bin_down(mag_q)

        cplx = i_ch + 1j * q_ch
        spec_c = np.fft.fftshift(np.fft.fft(cplx * win))
        mag_c = 20.0 * np.log10(np.abs(spec_c) + 1e-9)
        disp_c = self._bin_down(mag_c)  # dla widma "Stacje" (v6)

        center = self._current_freq()
        bin_hz = self._sample_rate / FFT_SIZE
        peak = self._find_peak(mag_c, center, bin_hz)

        GLib.idle_add(self._on_spectrum_frame, disp_i, disp_q, peak, disp_c)
        return Gst.FlowReturn.OK

    @staticmethod
    def _bin_down(mag_db):
        if FFT_SIZE % DISP_BINS == 0:
            return mag_db.reshape(DISP_BINS, FFT_SIZE // DISP_BINS).mean(axis=1)
        return np.interp(np.linspace(0, FFT_SIZE - 1, DISP_BINS),
                          np.arange(FFT_SIZE), mag_db)

    def _find_peak(self, mag_c, center, bin_hz):
        n = len(mag_c)
        lo_bin, hi_bin = 0, n - 1

        if self._selection_freqs:
            f_lo, f_hi = self._selection_freqs
            lo_bin = int(clamp((f_lo - (center - self._sample_rate / 2)) / bin_hz, 0, n - 1))
            hi_bin = int(clamp((f_hi - (center - self._sample_rate / 2)) / bin_hz, 0, n - 1))
            if hi_bin <= lo_bin:
                lo_bin, hi_bin = 0, n - 1

        search = mag_c[lo_bin:hi_bin + 1].copy()
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

        # Najpierw walidacja: szczyt musi byc wyraznie ponad szumem
        # (DEFAULT_THRESHOLD_MARGIN_DB). Izolowany 1-binowy pik = szum.
        det_margin = DEFAULT_THRESHOLD_MARGIN_DB
        if peak_level < noise_floor + det_margin:
            return None

        left = peak_bin
        while left > lo_bin and mag_c[left - 1] > noise_floor + det_margin:
            left -= 1
        right = peak_bin
        while right < hi_bin and mag_c[right + 1] > noise_floor + det_margin:
            right += 1

        # width_bins WLACZNIE (right - left + 1) — wczesniej bylo right-left,
        # wiec przy MIN_PEAK_WIDTH_BINS=3 realne 3-binowe szczyty odpadaly.
        width_bins = right - left + 1
        if width_bins < MIN_PEAK_WIDTH_BINS:
            return None

        # Pomiar szerokosci pasma z lagodniejszym marginesem, zeby if-bandwidth
        # nie bywal zawezany do samego rdzenia (za rygorystycznie przy 6 dB).
        w_margin = PEAK_WIDTH_MARGIN_DB
        w_left = peak_bin
        while w_left > lo_bin and mag_c[w_left - 1] > noise_floor + w_margin:
            w_left -= 1
        w_right = peak_bin
        while w_right < hi_bin and mag_c[w_right + 1] > noise_floor + w_margin:
            w_right += 1

        bw_hz = max(1, w_right - w_left + 1) * bin_hz * IF_BW_PAD
        freq_offset = (peak_bin - n / 2) * bin_hz
        return {"freq_offset": freq_offset, "bw_hz": bw_hz,
                "level": peak_level, "floor": noise_floor}

    def _on_spectrum_frame(self, disp_i, disp_q, peak, disp_c=None):
        if self._closing:
            return False
        center = self._current_freq()
        self.wf_left.set_axis(center, self._sample_rate)
        self.wf_right.set_axis(center, self._sample_rate)
        self.wf_left.push_row(disp_i)
        self.wf_right.push_row(disp_q)

        if disp_c is not None and hasattr(self, "station_spectrum"):
            margin = self.wf_left.threshold_margin_db
            peaks = find_validated_peaks(disp_c, margin_db=margin,
                                          min_width=2, min_sep=STATION_PEAK_MIN_SEP)
            self.station_spectrum.push_frame(disp_c, peaks)

        now = time.monotonic()
        if peak is not None:
            self._peak_buffer.append(
                (now, peak["freq_offset"], peak["bw_hz"], peak["level"], peak["floor"]))
        while self._peak_buffer and now - self._peak_buffer[0][0] > PEAK_BUFFER_SECONDS:
            self._peak_buffer.popleft()

        self._update_auto_correct(center, now)
        if hasattr(self, "station_info_lbl"):
            self._update_station_info()
        return False

    def _current_freq(self):
        try:
            return self.sdrsrc.get_property("frequency")
        except Exception:
            return self._init_freq

    def _update_auto_correct(self, center, now):
        if not self._peak_buffer:
            self.auto_status_lbl.set_label("Bufor pusty — brak wykrytego sygnału")
            self.wf_left.set_peak(None)
            self.wf_right.set_peak(None)
            return

        arr = np.array([(o, bw, lv, fl) for (_, o, bw, lv, fl) in self._peak_buffer])
        # v6 FIX: MEDIANA zamiast sredniej - odporna na pojedyncze outliery,
        # ktore mimo filtra szerokosci w _find_peak moga sie sporadycznie
        # zdarzyc (np. przy realnym, ale krotkotrwalym zaniku sygnalu).
        med_offset = float(np.median(arr[:, 0]))
        med_bw = float(np.median(arr[:, 1]))
        med_level = float(np.median(arr[:, 2]))
        med_floor = float(np.median(arr[:, 3]))
        snr = med_level - med_floor

        peak_freq = center + med_offset
        self.wf_left.set_peak(peak_freq)
        self.wf_right.set_peak(peak_freq)

        if_bw_suggest = clamp(med_bw, 0.0, 200000.0)
        freq_off_suggest = clamp(med_offset, -100000.0, 100000.0)

        scope = "zaznaczonym paśmie" if self._selection_freqs else "całym paśmie"
        self.auto_status_lbl.set_label(
            f"Bufor {len(self._peak_buffer)} próbek (~{PEAK_BUFFER_SECONDS:.0f}s), szukam w {scope}\n"
            f"Szczyt: {fmt_freq(peak_freq)}  |  S/N ≈ {snr:.1f} dB  |  "
            f"szacowane pasmo: {fmt_freq(med_bw)}\n"
            f"Sugestia (mediana): if-bandwidth={fmt_freq(if_bw_suggest)}, "
            f"freq-offset={freq_off_suggest:+.0f} Hz")

        if self.auto_correct_switch.get_active():
            if now - self._last_auto_apply >= AUTO_APPLY_MIN_INTERVAL:
                if self.auto_bw_switch.get_active():
                    self.auto_bw_switch.set_active_live(False)
                # v6 FIX: plynne dochodzenie (slew) zamiast skoku na sztywno
                # do wartosci sugerowanej - eliminuje sluchalny "klik" nawet
                # gdy sugestia sie realnie zmieni (np. przy przestrojeniu).
                cur_bw = self.if_bw_slider.get_value()
                cur_off = self.freq_offset_slider.get_value()
                new_bw = cur_bw + (if_bw_suggest - cur_bw) * AUTO_CORRECT_SLEW
                new_off = cur_off + (freq_off_suggest - cur_off) * AUTO_CORRECT_SLEW
                self.if_bw_slider.set_value_live(new_bw)
                self.freq_offset_slider.set_value_live(new_off)
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

    def _on_bus_element(self, bus, msg):
        """Wiadomosci z elementu 'spectrum' wewnatrz EQ - uzywane tylko
        gdy tryb Auto EQ jest wlaczony (wybor dominujacego pasma)."""
        s = msg.get_structure()
        if s is None or s.get_name() != "spectrum":
            return
        if not self._eq_state.get("auto") or not self._fx_enabled.get("eq"):
            return
        try:
            mag = list(s.get_value("magnitude"))
        except Exception:
            return
        if len(mag) < 10:
            return
        low = float(np.mean(mag[0:3]))
        mid = float(np.mean(mag[3:7]))
        high = float(np.mean(mag[7:10]))
        groups = {"low": low, "mid": mid, "high": high}
        dominant = max(groups, key=groups.get)  # najglosniejsze = najmniej ujemne dB

        idxs = {"low": (0, 3), "mid": (3, 7), "high": (7, 10)}
        cut, boost = -2.5, 1.2
        target = [0.0] * 10
        for g, (a, b) in idxs.items():
            val = cut if g == dominant else boost
            for i in range(a, b):
                target[i] = val

        base = self._eq_state["base_bands"]
        for i in range(10):
            base[i] += (target[i] - base[i]) * 0.15  # wygladzone dochodzenie
        if hasattr(self, "_eq_dominant_lbl"):
            names = {"low": "NISKIE", "mid": "ŚRODEK", "high": "WYSOKIE"}
            self._eq_dominant_lbl.set_label(
                f"Auto EQ: dominujące pasmo — {names[dominant]}")
        for i, sl in enumerate(self._eq_band_sliders):
            sl.set_value_quiet(base[i])
        self._apply_fx_live("eq")

    # -- FX: aplikacja live (generyczna dla wszystkich 6 typow) --------------

    @staticmethod
    def _map_tape(s):
        drive = s.get("drive", 50.0)
        warmth = s.get("warmth", 30.0)
        comp = s.get("comp", 20.0)
        return {
            ("sat", "threshold"): 0.9 - (drive / 100.0 * 0.8),
            ("gain", "volume"): 1.0 + (drive / 100.0 * 0.8),
            ("tone", "band0"): warmth / 100.0 * 4.0,
            ("tone", "band2"): -(warmth / 100.0 * 3.0),
            ("sat", "ratio"): 1.0 + (comp / 100.0 * 9.0),
        }

    @staticmethod
    def _map_spatial(s):
        width = s.get("width", 0.0)
        haas = s.get("haas", 0.0)
        sat = s.get("sat", 10.0)
        stereo = clamp(width * 0.0005, 0.0, 1.0)
        ns = max(1, int(haas * 1_000_000))
        intensity = 0.0 if haas == 0 else 0.35
        ratio = sat / 10.0
        threshold = 0.85 if ratio > 1.5 else 1.0
        return {
            ("sw", "stereo"): stereo,
            ("echo", "delay"): ns,
            ("echo", "intensity"): intensity,
            ("echo", "feedback"): 0.0,
            ("sat", "ratio"): ratio,
            ("sat", "threshold"): threshold,
        }

    @staticmethod
    def _map_phantom(s):
        mode = s.get("mode", "off")
        width = clamp(s.get("width", 5.0) / 100.0, 0.0, 1.0)
        delay = s.get("delay", 8.0)
        blend = s.get("blend", 5.0) / 100.0
        if mode == "off":
            return {("sw", "stereo"): 1.0, ("echo", "delay"): 1,
                    ("echo", "intensity"): 0.0, ("echo", "feedback"): 0.0}
        if mode == "haas":
            ns = max(1, int(delay * 1_000_000))
            intensity = 0.25 if delay > 0 else 0.0
            return {("sw", "stereo"): width, ("echo", "delay"): ns,
                    ("echo", "intensity"): intensity, ("echo", "feedback"): 0.0}
        # crossfeed
        intensity = clamp(blend, 0.0, 0.4)
        return {("sw", "stereo"): width, ("echo", "delay"): 1_000_000,
                ("echo", "intensity"): intensity, ("echo", "feedback"): 0.0}

    def _compute_eq_bands(self):
        base = self._eq_state["base_bands"]
        tilt = self._eq_state.get("tilt", 0.0)
        n = len(base)
        out = []
        for i in range(n):
            ramp = (i - (n - 1) / 2.0) / ((n - 1) / 2.0)
            out.append(clamp(base[i] + tilt * ramp, -24.0, 12.0))
        return out

    def _apply_fx_live(self, key):
        """Aplikuje aktualny stan kontrolek FX na zywe elementy GStreamer
        (bez restartu) - dziala i po pierwszym wlaczeniu, i po rebindzie
        po restarcie pipeline'u."""
        elems = self.fx_elements.get(key)
        if not elems:
            return
        if key == "eq":
            el = elems.get("main")
            if el is None:
                return
            for i, v in enumerate(self._compute_eq_bands()):
                try:
                    el.set_property(f"band{i}", v)
                except Exception as e:
                    print(f"[FX eq] band{i}={v}: {e}", file=sys.stderr)
        elif key in ("comp", "echo"):
            el = elems.get("main")
            if el is None:
                return
            for prop, val in self._fx_params.get(key, {}).items():
                try:
                    el.set_property(prop, val)
                except Exception as e:
                    print(f"[FX {key}] set_property({prop}) failed: {e}",
                          file=sys.stderr)
        elif key in ("tape", "spatial", "phantom"):
            mapper = {"tape": self._map_tape, "spatial": self._map_spatial,
                      "phantom": self._map_phantom}[key]
            mapping = mapper(self._fx_ext_state[key])
            for (suffix, prop), val in mapping.items():
                el = elems.get(suffix)
                if el is None:
                    continue
                try:
                    el.set_property(prop, val)
                except Exception as e:
                    print(f"[FX {key}] {suffix}.{prop}={val}: {e}", file=sys.stderr)

    # -- restart / rebuild ----------------------------------------------------

    def _full_rebuild(self, host=None, port=None, frequency=None, stereo=None):
        """
        Jedyne miejsce ktore robi pelny restart pipeline'u. Uzywane przez:
        zmiane host/port, przelaczenie stereo, wlaczenie/wylaczenie FX.
        Zachowuje wszystkie ZYWE wartosci (gain, if-bandwidth, denoise...)
        odczytujac je z aktualnych widgetow PO rebuildzie (patrz
        _rebind_prop_widgets), oraz parametry FX z self._fx_params.
        """
        host = host if host is not None else (
            self.host_entry.get_text().strip() if hasattr(self, "host_entry")
            else self._init_host)
        port = port if port is not None else (
            int(self.port_entry.get_text().strip()) if hasattr(self, "port_entry")
            else self._init_port)
        frequency = int(frequency if frequency is not None else self._current_freq())
        stereo = self._current_stereo if stereo is None else stereo

        self.status_lbl.set_label("Przebudowuję pipeline...")
        self._graceful_pipeline_stop()
        self._build_pipeline(host, port, frequency, self._sample_rate, stereo=stereo)
        self._rebind_prop_widgets()
        self._start()
        self._update_pipeline_preview()
        self.status_lbl.set_label("Gotowe")

    def _reconnect(self, host, port, frequency, stereo=None):
        self._full_rebuild(host=host, port=port, frequency=frequency, stereo=stereo)

    def _restart_for_stereo(self, stereo):
        """
        Diagnoza: 'stereo' w gst_sdr_demod.c zmienia liczbe kanalow liczona
        na zywo w transform_caps()/transform_size(), ale PROP_STEREO w
        set_property() nie wywoluje gst_base_transform_reconfigure_src() ani
        nie bierze locka - live g_object_set() w trakcie PLAYING rozjezdza
        wynegocjowane caps z faktycznym rozmiarem bufora i zatyka strumien.
        Dlatego tu robimy pelny restart. Pipeline NIE wymusza juz channels=1
        (patrz _build_pipeline), sdrdenoise w pelni obsluguje stereo, wiec to
        jest prawdziwe stereo az do autoaudiosink, nie downmix.
        """
        self._full_rebuild(stereo=stereo)

    # ---------------- UI: layout glowny (sidebar + stack) ----------------------

    def _build_ui(self):
        self._apply_css()

        toolbar = Adw.ToolbarView()
        header = Adw.HeaderBar()
        toolbar.add_top_bar(header)

        self.status_lbl = Gtk.Label(label="Start...", xalign=0, margin_start=8)
        self.status_lbl.add_css_class("dim-label")
        header.pack_start(self.status_lbl)

        root = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=0)
        toolbar.set_content(root)
        self.set_content(toolbar)

        self.stack = Gtk.Stack(hexpand=True, vexpand=True,
                                transition_type=Gtk.StackTransitionType.CROSSFADE)
        sidebar = Gtk.StackSidebar(stack=self.stack, vexpand=True)
        sidebar.set_size_request(200, -1)

        root.append(sidebar)
        root.append(Gtk.Separator(orientation=Gtk.Orientation.VERTICAL))
        root.append(self.stack)

        # Kolejnosc zoptymalizowana ergonomicznie: "Stacje" jako glowny
        # ekran startowy (wybor + info + widmo), potem konfiguracja zrodla
        # i strojenia, potem zaawansowane (widmo IQ/auto-korekta/denoise),
        # potem efekty pogrupowane logicznie, na koncu presety/transport.
        self._add_page("stacje", "Stacje", self._build_stations_group(),
                        icon="starred-symbolic")
        self._add_page("zrodlo", "Źródło", self._build_connection_group(),
                        icon="network-wireless-symbolic")
        self._add_page("strojenie", "Strojenie", self._build_tuning_group(),
                        icon="input-tablet-symbolic")
        self._add_page("gain", "Gain / AGC", self._build_gain_group(),
                        icon="audio-volume-high-symbolic")
        self._add_page("bandwidth", "Pasmo IF", self._build_bandwidth_group(),
                        icon="view-continuous-symbolic")
        self._add_page("widmo", "Widmo (wodospad)", self._build_waterfall_group(),
                        icon="utilities-system-monitor-symbolic")
        self._add_page("autokorekta", "Auto-korekta", self._build_autocorrect_group(),
                        icon="preferences-desktop-multimedia-symbolic")
        self._add_page("denoise", "Denoise", self._build_denoise_group(),
                        icon="edit-clear-all-symbolic")
        self._add_page("eq", "Equalizer", self._build_eq_group(),
                        icon="audio-x-generic-symbolic")
        self._add_page("fxcreative", "FX Kreatywne", self._build_fx_creative_group(),
                        icon="applications-graphics-symbolic")
        self._add_page("fxdynamics", "Dynamika / Echo", self._build_fx_dynamics_group(),
                        icon="media-playlist-repeat-symbolic")
        self._add_page("presety", "Presety", self._build_presets_group(),
                        icon="document-save-symbolic")
        self._add_page("transport", "Transport", self._build_transport_group(),
                        icon="media-playback-start-symbolic")

    def _apply_css(self):
        css = b"""
        frame > label { font-weight: bold; color: #7fb2ff; }
        .sdr-station-title { font-size: 16px; font-weight: bold; color: #eaeaea; }
        .sdr-dim { color: #9a9a9a; font-size: 11px; }
        switch:checked { background-color: #2e8b57; }
        """
        provider = Gtk.CssProvider()
        provider.load_from_data(css)
        Gtk.StyleContext.add_provider_for_display(
            Gdk.Display.get_default(), provider,
            Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION)

    def _add_page(self, name, title, widget, icon=None):
        scroller = Gtk.ScrolledWindow(vexpand=True, hexpand=True)
        scroller.set_child(widget)
        self.stack.add_titled(scroller, name, title)
        if icon:
            page = self.stack.get_page(scroller)
            try:
                page.set_icon_name(icon)
            except Exception:
                pass

    def _group(self, title):
        frame = Gtk.Frame(label=title)
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8,
                       margin_top=8, margin_bottom=8,
                       margin_start=8, margin_end=8)
        frame.set_child(box)
        return frame, box

    def _page_box(self):
        return Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=14,
                        margin_top=14, margin_bottom=14,
                        margin_start=14, margin_end=14)

    # -- Stacje (v6) ------------------------------------------------------------

    def _build_stations_group(self):
        page = self._page_box()

        frame_i, box_i = self._group("Aktualna stacja")
        self.station_info_lbl = Gtk.Label(xalign=0, wrap=True, label="—")
        self.station_info_lbl.add_css_class("sdr-station-title")
        box_i.append(self.station_info_lbl)
        page.append(frame_i)

        frame_s, box_s = self._group("Widmo (migawka, czarno-białe)")
        self.station_spectrum = SpectrumSnapshotView()
        box_s.append(self.station_spectrum)
        hint = Gtk.Label(
            label="Białe kropki oznaczają POTWIERDZONE szczyty (kilka "
                  "sąsiednich binów ponad podłogą szumu). Pojedyncze, "
                  "izolowane piki są odrzucane jako szum — nieczytelne dla "
                  "audio — i nie trafiają też do auto-korekty.",
            xalign=0, wrap=True)
        hint.add_css_class("sdr-dim")
        box_s.append(hint)
        page.append(frame_s)

        frame_l, box_l = self._group("Lista stacji")
        add_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        self.station_name_entry = Gtk.Entry(
            hexpand=True, placeholder_text="np. Radio Kraków 103.6")
        add_btn = Gtk.Button(label="➕ Dodaj bieżącą częstotliwość")
        add_btn.connect("clicked", self._on_add_station)
        add_row.append(self.station_name_entry)
        add_row.append(add_btn)
        box_l.append(add_row)

        self.stations_list = Gtk.ListBox()
        self.stations_list.set_selection_mode(Gtk.SelectionMode.NONE)
        box_l.append(self.stations_list)
        page.append(frame_l)

        self._reload_stations()
        return page

    def _reload_stations(self):
        self._stations_cache = self.stations.load_all()
        child = self.stations_list.get_first_child()
        while child is not None:
            nxt = child.get_next_sibling()
            self.stations_list.remove(child)
            child = nxt

        for s in self._stations_cache:
            row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8,
                           margin_top=4, margin_bottom=4,
                           margin_start=4, margin_end=4)
            row.append(Gtk.Label(
                label=f'{s["name"]}   ({fmt_freq(s["frequency"])})',
                xalign=0, hexpand=True))
            tune_btn = Gtk.Button(label="📻 Strojenie")
            tune_btn.connect("clicked", lambda b, f=s["frequency"]:
                              self._on_waterfall_tune(f))
            del_btn = Gtk.Button(label="🗑")
            del_btn.connect("clicked", lambda b, n=s["name"]:
                             self._on_delete_station(n))
            row.append(tune_btn)
            row.append(del_btn)
            self.stations_list.append(row)

        if hasattr(self, "station_info_lbl"):
            self._update_station_info()

    def _on_add_station(self, btn):
        name = self.station_name_entry.get_text().strip()
        if not name:
            self.status_lbl.set_label("Podaj nazwę stacji")
            return
        self.stations.add(name, self._current_freq())
        self.station_name_entry.set_text("")
        self._reload_stations()
        self.status_lbl.set_label(f"Dodano stację: {name}")

    def _on_delete_station(self, name):
        self.stations.delete(name)
        self._reload_stations()
        self.status_lbl.set_label(f"Usunięto stację: {name}")

    def _update_station_info(self):
        if not hasattr(self, "station_info_lbl"):
            return
        freq = self._current_freq()
        match = None
        for s in self._stations_cache:
            if abs(s["frequency"] - freq) <= 50_000:
                match = s["name"]
                break
        title = match if match else "Stacja niezapisana"

        try:
            denoise_on = bool(self.sdrdenoise.get_property("enabled"))
        except Exception:
            denoise_on = False
        stereo_txt = "stereo" if self._current_stereo else "mono"
        denoise_txt = "denoise wł." if denoise_on else "denoise wył."
        # auto_correct_switch powstaje dopiero na stronie "Auto-korekta",
        # a ta metoda jest wywolywana juz przy budowie "Stacje".
        if hasattr(self, "auto_correct_switch"):
            auto_txt = ("auto-korekta wł." if self.auto_correct_switch.get_active()
                        else "auto-korekta wył.")
        else:
            auto_txt = "auto-korekta wył."
        fx_names = {"eq": "EQ", "comp": "Kompresor", "echo": "Echo",
                    "tape": "Tape", "spatial": "Spatial", "phantom": "Phantom"}
        active_fx = [fx_names[k] for k in FX_ORDER if self._fx_enabled.get(k)]
        fx_txt = "FX: " + (", ".join(active_fx) if active_fx else "brak")

        self.station_info_lbl.set_label(
            f"{title} — {fmt_freq(freq)}\n"
            f"{stereo_txt} • {denoise_txt} • {auto_txt} • {fx_txt}")

    # -- Polaczenie -----------------------------------------------------------

    def _build_connection_group(self):
        page = self._page_box()
        frame, box = self._group("Źródło (rtl_tcp) — zmiana wymaga restartu")
        row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        self.host_entry = Gtk.Entry(text=self._init_host, hexpand=True,
                                     tooltip_text="Adres IP/hostname serwera rtl_tcp")
        self.port_entry = Gtk.Entry(text=str(self._init_port), width_chars=6,
                                     tooltip_text=range_hint(1, 65535))
        apply_btn = Gtk.Button(label="Zastosuj połączenie")
        apply_btn.connect("clicked", self._on_apply_connection)
        row.append(Gtk.Label(label="host:"))
        row.append(self.host_entry)
        row.append(Gtk.Label(label="port:"))
        row.append(self.port_entry)
        row.append(apply_btn)
        box.append(row)
        page.append(frame)
        return page

    def _on_apply_connection(self, btn):
        try:
            port = int(self.port_entry.get_text().strip())
        except ValueError:
            self.status_lbl.set_label("Nieprawidłowy port")
            return
        self._full_rebuild(host=self.host_entry.get_text().strip(), port=port)

    # -- Widmo ------------------------------------------------------------------

    def _build_waterfall_group(self):
        page = self._page_box()
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
            label="Klik = strojenie. Przeciągnij = zaznacz pasmo (ogranicza "
                  "szukanie szczytu dla auto-korekty). Biały = sygnał powyżej "
                  "progu, czarny = szum.",
            xalign=0, wrap=True)
        hint.add_css_class("dim-label")
        box.append(hint)

        threshold_slider = SimpleSlider(
            "Próg wykrywania (nad podłogą szumu)", lo=1.0, hi=20.0, step=0.5,
            value=DEFAULT_THRESHOLD_MARGIN_DB, digits=1, suffix=" dB",
            on_change=self._on_threshold_changed,
            hint="Margines w dB ponad medianę wiersza widma, powyżej którego "
                 "piksel jest biały (uznany za sygnał).")
        box.append(threshold_slider)

        clear_btn = Gtk.Button(label="Wyczyść zaznaczenie pasma")
        clear_btn.connect("clicked", self._on_clear_selection)
        box.append(clear_btn)
        page.append(frame)

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
        page.append(self.suggest_frame)

        return page

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
        self._selection_freqs = None
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
            f"Sugerowany freq-offset (NCO): {freq_offset_suggest:+.0f} Hz")
        self.suggest_apply_btn.set_sensitive(True)

    def _on_apply_suggestion(self, btn):
        if not self._pending_suggestion:
            return
        if_bw, freq_off = self._pending_suggestion
        self.auto_bw_switch.set_active_live(False)
        self.if_bw_slider.set_value_live(if_bw)
        self.freq_offset_slider.set_value_live(freq_off)
        self.status_lbl.set_label("Zastosowano sugestię pasma")

    # -- Auto-korekta -------------------------------------------------------------

    def _build_autocorrect_group(self):
        page = self._page_box()
        frame, box = self._group("Auto-korekta (bufor ~5s, na żywo)")

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
            label="Co ~0.5s aplikuje na żywo if-bandwidth i freq-offset "
                  "uśrednione z ostatnich ~5s wykrytego szczytu. Wyłącza przy "
                  "tym auto-bandwidth (żeby oba mechanizmy nie konkurowały).",
            xalign=0, wrap=True)
        hint.add_css_class("dim-label")
        box.append(hint)
        page.append(frame)
        return page

    # -- Strojenie ---------------------------------------------------------------

    def _build_tuning_group(self):
        page = self._page_box()
        frame, box = self._group("Strojenie (na żywo)")

        entry_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        entry_row.append(Gtk.Label(label="Częstotliwość:"))
        self.freq_entry = Gtk.Entry(
            text=fmt_freq(self._init_freq), hexpand=True,
            tooltip_text="Formaty: 92.1M / 92100k / 92100000 / 92,1MHz")
        self.freq_entry.connect("activate", self._on_freq_entry_activate)
        entry_row.append(self.freq_entry)
        set_btn = Gtk.Button(label="Ustaw")
        set_btn.connect("clicked", self._on_freq_entry_activate)
        entry_row.append(set_btn)
        box.append(entry_row)

        self.freq_slider = PropSlider(
            self.sdrsrc, "frequency", "Częstotliwość (suwak)",
            lo=24_000_000, hi=1_766_000_000, step=1000, digits=0,
            value=self._init_freq, suffix=" Hz",
            hint="Zakres tunera RTL-SDR: 24 MHz – 1766 MHz")
        self.freq_slider.scale.connect("value-changed", self._on_freq_slider_moved)
        box.append(self.freq_slider)

        self.freq_offset_slider = PropSlider(
            self.sdrdemod, "freq-offset", "Offset NCO",
            lo=-100000, hi=100000, step=100, digits=0, suffix=" Hz",
            hint="Przesunięcie cyfrowego oscylatora wewnątrz pasma IF, "
                 "bez zmiany częstotliwości LO tunera.")
        box.append(self.freq_offset_slider)
        page.append(frame)

        frame2, box2 = self._group("Stereo")
        stereo_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        stereo_lbl = Gtk.Label(label="Stereo (przełączenie = krótki restart ~1s)",
                                xalign=0, hexpand=True)
        self.stereo_switch = Gtk.Switch(active=self._current_stereo,
                                         valign=Gtk.Align.CENTER)
        self.stereo_switch.connect("state-set", self._on_stereo_switch_toggled)
        stereo_row.append(stereo_lbl)
        stereo_row.append(self.stereo_switch)
        box2.append(stereo_row)
        stereo_hint = Gtk.Label(
            label="Prawdziwe stereo (2 kanały) aż do autoaudiosink — nie "
                  "wewnętrzny downmix. Restart wymagany przez bug w C "
                  "(brak gst_base_transform_reconfigure_src przy zmianie "
                  "'stereo' w gst_sdr_demod.c) — patrz historia czatu.",
            xalign=0, wrap=True)
        stereo_hint.add_css_class("dim-label")
        box2.append(stereo_hint)
        page.append(frame2)
        return page

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
        page = self._page_box()
        frame, box = self._group("Gain / AGC")

        self.gain_slider = PropSlider(
            self.sdrsrc, "gain", "Gain ręczny", lo=0.0, hi=50.0, step=0.1,
            digits=1, suffix=" dB", hint="Wzmocnienie tunera RTL-SDR.")
        box.append(self.gain_slider)

        self.auto_gain_switch = PropSwitch(
            self.sdrsrc, "auto-gain", "Auto Gain (AGC)",
            on_toggle=self._on_auto_gain_toggle,
            hint="Automatyczna pętla AGC w C (gst_sdr_src.c) mierząca moc IQ.")
        box.append(self.auto_gain_switch)

        self.auto_gain_target_slider = PropSlider(
            self.sdrsrc, "auto-gain-target-db", "Cel AGC",
            lo=-40.0, hi=0.0, step=0.5, digits=1, suffix=" dBFS",
            hint="Docelowy poziom sygnału dla pętli AGC.")
        box.append(self.auto_gain_target_slider)

        self._on_auto_gain_toggle(self.auto_gain_switch.get_active())
        page.append(frame)
        return page

    def _on_auto_gain_toggle(self, active):
        self.gain_slider.set_sensitive_live(not active)
        self.auto_gain_target_slider.set_sensitive_live(active)

    # -- IF Bandwidth --------------------------------------------------------

    def _build_bandwidth_group(self):
        page = self._page_box()
        frame, box = self._group("Pasmo IF / dostrajanie S/N")

        self.if_bw_slider = PropSlider(
            self.sdrdemod, "if-bandwidth", "IF Bandwidth (0=auto z deviation)",
            lo=0.0, hi=200000.0, step=500.0, digits=0, suffix=" Hz",
            hint="Szerokość filtru IF przed dyskryminatorem FM. 0 = liczone "
                 "automatycznie z max-deviation (stare zachowanie).")
        box.append(self.if_bw_slider)

        self.auto_bw_switch = PropSwitch(
            self.sdrdemod, "auto-bandwidth", "Auto-Bandwidth C (S/N hunt w C)",
            on_toggle=self._on_auto_bw_toggle,
            hint="Mechanizm w C działający na zdemodulowanym sygnale. "
                 "Osobny od 'Auto-korekty' (zakładka Auto-korekta), która "
                 "działa na widmie IQ z wodospadu — nie włączaj obu naraz.")
        box.append(self.auto_bw_switch)

        self._on_auto_bw_toggle(self.auto_bw_switch.get_active())
        page.append(frame)
        return page

    def _on_auto_bw_toggle(self, active):
        self.if_bw_slider.set_sensitive_live(not active)

    # -- Denoise ---------------------------------------------------------------

    def _build_denoise_group(self):
        page = self._page_box()
        frame, box = self._group("Redukcja szumu i interpolacja")

        self.denoise_switch = PropSwitch(self.sdrdenoise, "enabled", "Denoise włączony")
        box.append(self.denoise_switch)

        self.threshold_slider = PropSlider(
            self.sdrdenoise, "threshold-db", "Próg", lo=-40.0, hi=20.0,
            step=0.5, digits=1, suffix=" dB",
            hint="Próg STFT poniżej którego bin jest tłumiony jako szum.")
        box.append(self.threshold_slider)

        self.alpha_up_slider = PropSlider(
            self.sdrdenoise, "alpha-up", "Alpha Up", lo=0.0001, hi=1.0,
            step=0.001, digits=4,
            hint="Szybkość narastania estymaty szumu (wyższa = szybciej "
                 "reaguje na wzrost szumu).")
        box.append(self.alpha_up_slider)

        self.alpha_down_slider = PropSlider(
            self.sdrdenoise, "alpha-down", "Alpha Down", lo=0.00001, hi=0.1,
            step=0.0001, digits=5,
            hint="Szybkość opadania estymaty szumu (niższa = wolniej "
                 "'zapomina' o szumie, stabilniejsze tłumienie).")
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
        page.append(frame)
        return page

    def _on_interp_toggle(self, active):
        self.auto_interp_switch.switch.set_sensitive(active)
        self.interp_strength_slider.set_sensitive_live(
            active and not self.auto_interp_switch.get_active())

    def _on_auto_interp_toggle(self, active):
        self.interp_strength_slider.set_sensitive_live(
            self.interp_switch.get_active() and not active)

    # -- FX Audio ----------------------------------------------------------------

    def _fx_enable_row(self, key, box):
        """Wspolny wiersz ENABLE dla kazdego FX - restart strukturalny."""
        row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        lbl = Gtk.Label(label="Włączony (restart)", xalign=0, hexpand=True)
        sw = Gtk.Switch(active=False, valign=Gtk.Align.CENTER)
        sw.connect("state-set", self._make_fx_toggle_handler(key))
        row.append(lbl)
        row.append(sw)
        box.append(row)
        self._fx_enable_switches[key] = sw

    def _make_fx_toggle_handler(self, key):
        def handler(switch, state):
            self._fx_enabled[key] = state
            for w in self._fx_sensitive_widgets.get(key, []):
                w.set_sensitive(state)
            self._full_rebuild()
            return False
        return handler

    def _make_fx_param_handler(self, key, prop):
        """comp/echo: prop bezposrednio na jedynym elemencie 'main'."""
        def handler(value):
            self._fx_params[key][prop] = value
            if self._fx_enabled.get(key):
                self._apply_fx_live(key)
        return handler

    # -- Equalizer (presety / tilt / auto) ------------------------------------

    def _build_eq_group(self):
        page = self._page_box()
        info = Gtk.Label(
            label="Włączenie/wyłączenie robi krótki restart (zmiana struktury "
                  "grafu). Presety, tilt, pasma i tryb Auto działają na żywo.",
            xalign=0, wrap=True)
        info.add_css_class("dim-label")
        page.append(info)

        frame, box = self._group(FX_DEFS["eq"]["label"])
        self._fx_enable_row("eq", box)

        preset_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        preset_row.append(Gtk.Label(label="Preset:"))
        self.eq_preset_combo = Gtk.ComboBoxText()
        for name in EQ_PRESETS:
            self.eq_preset_combo.append_text(name)
        self.eq_preset_combo.set_active(0)
        self.eq_preset_combo.connect("changed", self._on_eq_preset_changed)
        preset_row.append(self.eq_preset_combo)
        box.append(preset_row)
        self._fx_sensitive_widgets["eq"].append(self.eq_preset_combo)

        self.eq_tilt_slider = SimpleSlider(
            "Tilt (nachylenie charakterystyki)", lo=-12.0, hi=12.0, step=0.5,
            value=0.0, digits=1, suffix=" dB", on_change=self._on_eq_tilt_changed,
            hint="Dodaje liniową rampę: dół pasma w dół, góra pasma w górę "
                 "(lub odwrotnie dla wartości ujemnych).")
        box.append(self.eq_tilt_slider)
        self._fx_sensitive_widgets["eq"].append(self.eq_tilt_slider)

        auto_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        auto_lbl = Gtk.Label(
            label="Auto EQ (wybór dominującego pasma, żywa korekta)",
            xalign=0, hexpand=True)
        self.eq_auto_switch = Gtk.Switch(active=False, valign=Gtk.Align.CENTER)
        self.eq_auto_switch.connect("state-set", self._on_eq_auto_toggled)
        auto_row.append(auto_lbl)
        auto_row.append(self.eq_auto_switch)
        box.append(auto_row)

        self._eq_dominant_lbl = Gtk.Label(label="Auto EQ: nieaktywny",
                                           xalign=0, wrap=True)
        self._eq_dominant_lbl.add_css_class("sdr-dim")
        box.append(self._eq_dominant_lbl)

        box.append(Gtk.Separator(orientation=Gtk.Orientation.HORIZONTAL))
        self._eq_band_sliders = []
        for i, label in enumerate(FX_DEFS["eq"]["band_labels"]):
            slider = SimpleSlider(
                label, -24.0, 12.0, 0.5, 0.0, digits=1, suffix=" dB",
                on_change=self._make_eq_band_handler(i))
            box.append(slider)
            self._eq_band_sliders.append(slider)
            self._fx_sensitive_widgets["eq"].append(slider)

        page.append(frame)
        return page

    def _make_eq_band_handler(self, idx):
        def handler(value):
            self._eq_state["base_bands"][idx] = value
            if self._fx_enabled.get("eq"):
                self._apply_fx_live("eq")
        return handler

    def _on_eq_preset_changed(self, combo):
        name = combo.get_active_text()
        bands = EQ_PRESETS.get(name)
        if not bands:
            return
        self._eq_state["base_bands"] = [float(v) for v in bands]
        for i, sl in enumerate(self._eq_band_sliders):
            sl.set_value_quiet(bands[i])
        if self._fx_enabled.get("eq"):
            self._apply_fx_live("eq")

    def _on_eq_tilt_changed(self, value):
        self._eq_state["tilt"] = value
        if self._fx_enabled.get("eq"):
            self._apply_fx_live("eq")

    def _on_eq_auto_toggled(self, switch, state):
        self._eq_state["auto"] = state
        for sl in self._eq_band_sliders:
            sl.set_sensitive(not state and self._fx_enabled.get("eq"))
        self.eq_preset_combo.set_sensitive(not state and self._fx_enabled.get("eq"))
        self.eq_tilt_slider.set_sensitive(not state and self._fx_enabled.get("eq"))
        self._eq_dominant_lbl.set_label(
            "Auto EQ: analizuję widmo..." if state else "Auto EQ: nieaktywny")
        return False

    # -- FX Kreatywne: Tape / Spatial / Phantom -------------------------------

    def _build_fx_creative_group(self):
        page = self._page_box()
        info = Gtk.Label(
            label="Efekty przeniesione z CarbonFX. Włącz/wyłącz = krótki "
                  "restart, suwaki i presety wewnątrz działają na żywo.",
            xalign=0, wrap=True)
        info.add_css_class("dim-label")
        page.append(info)

        # -- Tape --
        frame_t, box_t = self._group(FX_DEFS["tape"]["label"])
        self._fx_enable_row("tape", box_t)
        s = self._fx_ext_state["tape"]
        for key, label, lo, hi in (("drive", "Drive", 0.0, 100.0),
                                    ("warmth", "Warmth", 0.0, 100.0),
                                    ("comp", "Comp", 0.0, 100.0)):
            slider = SimpleSlider(
                label, lo, hi, 1.0, s[key], digits=0,
                on_change=self._make_ext_fx_handler("tape", key))
            slider.set_sensitive(False)
            box_t.append(slider)
            self._fx_sensitive_widgets["tape"].append(slider)
        page.append(frame_t)

        # -- Spatial --
        frame_s, box_s = self._group(FX_DEFS["spatial"]["label"])
        self._fx_enable_row("spatial", box_s)
        preset_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        preset_row.append(Gtk.Label(label="Preset:"))
        sp_preset = Gtk.ComboBoxText()
        for name in SPATIAL_PRESETS:
            sp_preset.append_text(name)
        sp_preset.set_active(0)
        sp_preset.connect("changed", self._on_spatial_preset_changed)
        preset_row.append(sp_preset)
        box_s.append(preset_row)
        self._fx_sensitive_widgets["spatial"].append(sp_preset)
        self._spatial_preset_combo = sp_preset

        ss = self._fx_ext_state["spatial"]
        self._spatial_sliders = {}
        for key, label, lo, hi, val in (
                ("width", "Stereo Width (0-3%)", 0.0, 60.0, ss["width"]),
                ("haas", "Haas Delay", 0.0, 20.0, ss["haas"]),
                ("sat", "Sat Ratio", 10.0, 80.0, ss["sat"])):
            slider = SimpleSlider(
                label, lo, hi, 1.0, val, digits=0,
                on_change=self._make_ext_fx_handler("spatial", key))
            slider.set_sensitive(False)
            box_s.append(slider)
            self._fx_sensitive_widgets["spatial"].append(slider)
            self._spatial_sliders[key] = slider
        page.append(frame_s)

        # -- Phantom --
        frame_p, box_p = self._group(FX_DEFS["phantom"]["label"])
        self._fx_enable_row("phantom", box_p)
        pr = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        pr.append(Gtk.Label(label="Preset:"))
        ph_preset = Gtk.ComboBoxText()
        for name in PHANTOM_PRESETS:
            ph_preset.append_text(name)
        ph_preset.set_active(0)
        ph_preset.connect("changed", self._on_phantom_preset_changed)
        pr.append(ph_preset)
        box_p.append(pr)
        self._fx_sensitive_widgets["phantom"].append(ph_preset)
        self._phantom_preset_combo = ph_preset

        self._phantom_desc_lbl = Gtk.Label(
            label=PHANTOM_PRESETS["Off"]["desc"], xalign=0, wrap=True)
        self._phantom_desc_lbl.add_css_class("sdr-dim")
        box_p.append(self._phantom_desc_lbl)

        mr = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        mr.append(Gtk.Label(label="Tryb:"))
        ph_mode = Gtk.ComboBoxText()
        for m in ("off", "crossfeed", "haas"):
            ph_mode.append_text(m)
        ph_mode.set_active(0)
        ph_mode.connect("changed", self._on_phantom_mode_changed)
        mr.append(ph_mode)
        box_p.append(mr)
        self._fx_sensitive_widgets["phantom"].append(ph_mode)
        self._phantom_mode_combo = ph_mode

        ps = self._fx_ext_state["phantom"]
        self._phantom_sliders = {}
        for key, label, lo, hi, val in (
                ("width", "Stereo Width", 0.0, 100.0, ps["width"]),
                ("delay", "Haas Delay", 0.0, 20.0, ps["delay"]),
                ("blend", "Crossfeed Blend", 0.0, 30.0, ps["blend"])):
            slider = SimpleSlider(
                label, lo, hi, 1.0, val, digits=0,
                on_change=self._make_ext_fx_handler("phantom", key))
            slider.set_sensitive(False)
            box_p.append(slider)
            self._fx_sensitive_widgets["phantom"].append(slider)
            self._phantom_sliders[key] = slider
        page.append(frame_p)

        return page

    def _make_ext_fx_handler(self, key, ctrl):
        def handler(value):
            self._fx_ext_state[key][ctrl] = value
            if self._fx_enabled.get(key):
                self._apply_fx_live(key)
        return handler

    def _on_spatial_preset_changed(self, combo):
        name = combo.get_active_text()
        p = SPATIAL_PRESETS.get(name)
        if not p:
            return
        width, haas, sat = p
        self._fx_ext_state["spatial"].update(width=width, haas=haas, sat=sat)
        self._spatial_sliders["width"].set_value_quiet(width)
        self._spatial_sliders["haas"].set_value_quiet(haas)
        self._spatial_sliders["sat"].set_value_quiet(sat)
        if self._fx_enabled.get("spatial"):
            self._apply_fx_live("spatial")

    def _on_phantom_preset_changed(self, combo):
        name = combo.get_active_text()
        p = PHANTOM_PRESETS.get(name)
        if not p:
            return
        self._phantom_desc_lbl.set_label(p.get("desc", ""))
        self._fx_ext_state["phantom"].update(
            mode=p["mode"], width=p["width"], delay=p["delay"], blend=p["blend"])
        self._phantom_mode_combo.set_active(
            {"off": 0, "crossfeed": 1, "haas": 2}.get(p["mode"], 0))
        self._phantom_sliders["width"].set_value_quiet(p["width"])
        self._phantom_sliders["delay"].set_value_quiet(p["delay"])
        self._phantom_sliders["blend"].set_value_quiet(p["blend"])
        if self._fx_enabled.get("phantom"):
            self._apply_fx_live("phantom")

    def _on_phantom_mode_changed(self, combo):
        mode = combo.get_active_text()
        self._fx_ext_state["phantom"]["mode"] = mode
        if self._fx_enabled.get("phantom"):
            self._apply_fx_live("phantom")

    # -- Dynamika: Kompresor / Echo --------------------------------------------

    def _build_fx_dynamics_group(self):
        page = self._page_box()
        for key in ("comp", "echo"):
            d = FX_DEFS[key]
            frame, box = self._group(d["label"])
            self._fx_enable_row(key, box)
            defaults = {
                ("comp", "ratio"): 2.0, ("comp", "threshold"): 0.2,
                ("echo", "delay"): 120_000_000, ("echo", "intensity"): 0.3,
                ("echo", "feedback"): 0.2,
            }
            for prop, plabel, lo, hi, step, digits, suffix, hint in d["params"]:
                default = defaults.get((key, prop), (lo + hi) / 2.0)
                self._fx_params[key][prop] = default
                slider = SimpleSlider(
                    plabel, lo, hi, step, default, digits=digits, suffix=suffix,
                    on_change=self._make_fx_param_handler(key, prop), hint=hint)
                slider.set_sensitive(False)
                box.append(slider)
                self._fx_widgets[key][prop] = slider
                self._fx_sensitive_widgets[key].append(slider)
            page.append(frame)
        return page

    # -- Presety -----------------------------------------------------------------

    def _build_presets_group(self):
        page = self._page_box()

        frame, box = self._group("Aktualny pipeline (gst-launch)")
        self.pipeline_preview = Gtk.TextView(editable=False, cursor_visible=False,
                                              wrap_mode=Gtk.WrapMode.WORD_CHAR)
        self.pipeline_preview.add_css_class("monospace")
        pv_scroll = Gtk.ScrolledWindow(min_content_height=140)
        pv_scroll.set_child(self.pipeline_preview)
        box.append(pv_scroll)
        page.append(frame)
        self._update_pipeline_preview()

        frame2, box2 = self._group("Zapisz obecne ustawienia jako preset")
        row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
        self.preset_name_entry = Gtk.Entry(hexpand=True,
                                            placeholder_text="np. Radio Kraków 103.6")
        save_btn = Gtk.Button(label="Zapisz")
        save_btn.connect("clicked", self._on_save_preset)
        row.append(self.preset_name_entry)
        row.append(save_btn)
        box2.append(row)

        default_btn = Gtk.Button(label="Ustaw obecny stan jako domyślny przy starcie")
        default_btn.connect("clicked", self._on_save_default_preset)
        box2.append(default_btn)
        page.append(frame2)

        frame3, box3 = self._group("Zapisane presety")
        self.presets_list = Gtk.ListBox()
        self.presets_list.set_selection_mode(Gtk.SelectionMode.NONE)
        box3.append(self.presets_list)
        page.append(frame3)
        self._refresh_presets_list()

        return page

    def _update_pipeline_preview(self):
        if hasattr(self, "pipeline_preview"):
            buf = self.pipeline_preview.get_buffer()
            pretty = getattr(self, "_last_pipeline_desc", "").replace(" ! ", " !\n  ")
            buf.set_text(pretty)

    def _refresh_presets_list(self):
        child = self.presets_list.get_first_child()
        while child is not None:
            nxt = child.get_next_sibling()
            self.presets_list.remove(child)
            child = nxt

        data = self.presets.load_all()
        for name in sorted(data.keys()):
            if name == DEFAULT_PRESET_KEY:
                continue
            row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8,
                           margin_top=4, margin_bottom=4,
                           margin_start=4, margin_end=4)
            row.append(Gtk.Label(label=name, xalign=0, hexpand=True))
            load_btn = Gtk.Button(label="Wczytaj")
            load_btn.connect("clicked", lambda b, n=name: self._on_load_preset(n))
            del_btn = Gtk.Button(label="Usuń")
            del_btn.connect("clicked", lambda b, n=name: self._on_delete_preset(n))
            row.append(load_btn)
            row.append(del_btn)
            self.presets_list.append(row)

    def _collect_state(self):
        return {
            "host": self.host_entry.get_text().strip(),
            "port": self.port_entry.get_text().strip(),
            "frequency": self._current_freq(),
            "stereo": self._current_stereo,
            "gain": self.gain_slider.get_value(),
            "auto_gain": self.auto_gain_switch.get_active(),
            "auto_gain_target": self.auto_gain_target_slider.get_value(),
            "if_bandwidth": self.if_bw_slider.get_value(),
            "auto_bandwidth": self.auto_bw_switch.get_active(),
            "freq_offset": self.freq_offset_slider.get_value(),
            "threshold_db": self.threshold_slider.get_value(),
            "alpha_up": self.alpha_up_slider.get_value(),
            "alpha_down": self.alpha_down_slider.get_value(),
            "interpolate": self.interp_switch.get_active(),
            "auto_interpolate": self.auto_interp_switch.get_active(),
            "interp_strength": self.interp_strength_slider.get_value(),
            "auto_correct": self.auto_correct_switch.get_active(),
            "fx_enabled": dict(self._fx_enabled),
            "fx_params": {k: dict(v) for k, v in self._fx_params.items()},
            "fx_ext_state": {k: dict(v) for k, v in self._fx_ext_state.items()},
            "eq_state": {"base_bands": list(self._eq_state["base_bands"]),
                         "tilt": self._eq_state["tilt"],
                         "auto": self._eq_state["auto"]},
        }

    def _apply_state(self, s):
        self.host_entry.set_text(s.get("host", self._init_host))
        self.port_entry.set_text(str(s.get("port", self._init_port)))

        self._fx_enabled = dict(s.get("fx_enabled", self._fx_enabled))
        self._fx_params = {k: dict(v) for k, v in
                            s.get("fx_params", self._fx_params).items()}
        self._fx_ext_state = {k: dict(v) for k, v in
                               s.get("fx_ext_state", self._fx_ext_state).items()}
        eq_s = s.get("eq_state")
        if eq_s:
            self._eq_state["base_bands"] = list(eq_s.get(
                "base_bands", self._eq_state["base_bands"]))
            self._eq_state["tilt"] = eq_s.get("tilt", 0.0)
            self._eq_state["auto"] = eq_s.get("auto", False)

        for key in FX_ORDER:
            self._fx_enable_switches[key].set_active(self._fx_enabled.get(key, False))

        # comp/echo suwaki
        for key in ("comp", "echo"):
            for prop, slider in self._fx_widgets[key].items():
                if prop in self._fx_params.get(key, {}):
                    slider.set_value_quiet(self._fx_params[key][prop])
                    slider.set_sensitive(self._fx_enabled.get(key, False))

        # tape/spatial/phantom - odtworz suwaki z fx_ext_state (best-effort,
        # jesli widgety juz istnieja)
        if hasattr(self, "_spatial_sliders"):
            ss = self._fx_ext_state.get("spatial", {})
            for k, sl in self._spatial_sliders.items():
                if k in ss:
                    sl.set_value_quiet(ss[k])
                    sl.set_sensitive(self._fx_enabled.get("spatial", False))
        if hasattr(self, "_phantom_sliders"):
            ps = self._fx_ext_state.get("phantom", {})
            for k, sl in self._phantom_sliders.items():
                if k in ps:
                    sl.set_value_quiet(ps[k])
                    sl.set_sensitive(self._fx_enabled.get("phantom", False))
            if "mode" in ps:
                self._phantom_mode_combo.set_active(
                    {"off": 0, "crossfeed": 1, "haas": 2}.get(ps["mode"], 0))
                self._phantom_mode_combo.set_sensitive(
                    self._fx_enabled.get("phantom", False))

        # eq suwaki + tilt
        if self._eq_band_sliders:
            for i, sl in enumerate(self._eq_band_sliders):
                if i < len(self._eq_state["base_bands"]):
                    sl.set_value_quiet(self._eq_state["base_bands"][i])
                sl.set_sensitive(self._fx_enabled.get("eq", False)
                                  and not self._eq_state["auto"])
        if hasattr(self, "eq_tilt_slider"):
            self.eq_tilt_slider.set_value_quiet(self._eq_state["tilt"])
        if hasattr(self, "eq_auto_switch"):
            self.eq_auto_switch.set_active(self._eq_state["auto"])

        # restart raz, z docelowym stereo/host/port/freq/FX juz ustawionymi
        self._full_rebuild(
            host=s.get("host", self._init_host),
            port=int(s.get("port", self._init_port)),
            frequency=int(s.get("frequency", self._init_freq)),
            stereo=s.get("stereo", True))

        # zywe wartosci - po restarcie, na nowych elementach
        self.gain_slider.set_value_live(s.get("gain", self.gain_slider.get_value()))
        self.auto_gain_switch.set_active_live(s.get("auto_gain", False))
        self.auto_gain_target_slider.set_value_live(
            s.get("auto_gain_target", self.auto_gain_target_slider.get_value()))
        self.if_bw_slider.set_value_live(s.get("if_bandwidth", self.if_bw_slider.get_value()))
        self.auto_bw_switch.set_active_live(s.get("auto_bandwidth", False))
        self.freq_offset_slider.set_value_live(
            s.get("freq_offset", self.freq_offset_slider.get_value()))
        self.threshold_slider.set_value_live(
            s.get("threshold_db", self.threshold_slider.get_value()))
        self.alpha_up_slider.set_value_live(
            s.get("alpha_up", self.alpha_up_slider.get_value()))
        self.alpha_down_slider.set_value_live(
            s.get("alpha_down", self.alpha_down_slider.get_value()))
        self.interp_switch.set_active_live(s.get("interpolate", False))
        self.auto_interp_switch.set_active_live(s.get("auto_interpolate", False))
        self.interp_strength_slider.set_value_live(
            s.get("interp_strength", self.interp_strength_slider.get_value()))
        self.auto_correct_switch.set_active(s.get("auto_correct", False))
        self.freq_entry.set_text(fmt_freq(self._current_freq()))

    def _on_save_preset(self, btn):
        name = self.preset_name_entry.get_text().strip()
        if not name:
            self.status_lbl.set_label("Podaj nazwę presetu")
            return
        self.presets.save_one(name, self._collect_state())
        self._refresh_presets_list()
        self.status_lbl.set_label(f"Zapisano preset: {name}")

    def _on_save_default_preset(self, btn):
        self.presets.save_one(DEFAULT_PRESET_KEY, self._collect_state())
        self.status_lbl.set_label("Ustawiono jako domyślny przy starcie")

    def _on_load_preset(self, name):
        data = self.presets.load_all()
        s = data.get(name)
        if not s:
            return
        self._apply_state(s)
        self.status_lbl.set_label(f"Wczytano preset: {name}")

    def _on_delete_preset(self, name):
        self.presets.delete_one(name)
        self._refresh_presets_list()
        self.status_lbl.set_label(f"Usunięto preset: {name}")

    def _maybe_load_default_preset(self):
        data = self.presets.load_all()
        s = data.get(DEFAULT_PRESET_KEY)
        if s:
            GLib.idle_add(self._apply_state, s)

    # -- Transport -------------------------------------------------------------

    def _build_transport_group(self):
        page = self._page_box()
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
        page.append(frame)
        return page

    def _set_state(self, state):
        self.pipeline.set_state(state)
        self.status_lbl.set_label(f"Stan: {state.value_nick}")

    # -- rebind po rebuild ------------------------------------------------------

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

        self.stereo_switch.set_state(self._current_stereo)

        # FX: re-aplikuj parametry na swiezo utworzonych elementach
        # (generyczne dla wszystkich 6 typow FX - patrz _apply_fx_live)
        for key in FX_ORDER:
            if self._fx_enabled.get(key):
                self._apply_fx_live(key)

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
