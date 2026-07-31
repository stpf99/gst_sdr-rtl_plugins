#!/usr/bin/env python3
"""
sdr_gtk4_gui.py

GTK4 control panel dla pipeline'u:
  sdrsrc ! queue ! sdrdemod ! audioconvert ! audioresample ! sdrdenoise ! queue ! autoaudiosink

Filozofia sterowania:
  * Wszystkie suwaki/przełączniki auto-gain, if-bandwidth, auto-bandwidth,
    interpolate, auto-interpolate, interp-strength, threshold-db, alpha-up/down,
    stereo-mix, audio-cutoff, tau, freq-offset, max-deviation, frequency
    -> ustawiane NA ŻYWO przez g_object_set(), pipeline zostaje w stanie PLAYING.
    To bezpieczne, bo w Twoim C te property mają już lock (cfg_lock / send-w-locku
    w src) i przeliczają tylko taps/parametry, nie robią pełnego re-configure().

  * Jedyne co wymaga twardego restartu gałęzi źródła to zmiana mode/host/port
    w sdrsrc (bo to zmienia socket/urządzenie, nie da się tego "przeliczyć w locie").
    Dla tego przypadku GUI robi stop -> unref -> rebuild -> start całego pipeline'u,
    ale tylko na wyraźne kliknięcie "Zastosuj połączenie", nigdy przy zwykłych
    suwakach.

  * frequency w sdrsrc jest live-settable (jak gain) więc strojenie stacji
    też NIE restartuje pipeline'u.

Wymaga: python3-gi, gir1.2-gtk-4.0, gir1.2-gst-plugins-base-1.0,
        oraz zbudowanego i zainstalowanego pluginu gst-sdr-plugins (sdrsrc/sdrdemod/sdrdenoise).
"""

import sys
import gi

gi.require_version("Gtk", "4.0")
gi.require_version("Adw", "1")
gi.require_version("Gst", "1.0")

from gi.repository import Gtk, Adw, Gst, GLib, GObject  # noqa: E402

Gst.init(None)


# --------------------------------------------------------------------------
# Pomocnicze: pojedynczy wiersz "etykieta + suwak + wartość" spinający się
# bezpośrednio do property GStreamer elementu, bez przechodzenia przez
# jakikolwiek callback restartujący pipeline.
# --------------------------------------------------------------------------
class PropSlider(Gtk.Box):
    """Suwak który na każdą zmianę robi element.set_property(name, value)."""

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

        self.value_lbl = Gtk.Label(label=self._fmt(value), width_chars=10)
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
        """Ustawia GUI bez odpalania set_property (np. gdy auto-* przejmuje kontrolę)."""
        self.scale.handler_block_by_func(self._on_changed)
        self.scale.set_value(v)
        self.value_lbl.set_label(self._fmt(v))
        self.scale.handler_unblock_by_func(self._on_changed)


class PropSwitch(Gtk.Box):
    """Switch który na zmianę robi element.set_property(name, bool)."""

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
        return False  # pozwól domyślnemu handlerowi ustawić stan wizualny

    def get_active(self):
        return self.switch.get_active()


# --------------------------------------------------------------------------
# Główne okno
# --------------------------------------------------------------------------
class SdrWindow(Adw.ApplicationWindow):
    def __init__(self, app, host, port, frequency):
        super().__init__(application=app, title="SDR FM Control")
        self.set_default_size(560, 780)

        self.pipeline = None
        self.sdrsrc = None
        self.sdrdemod = None
        self.sdrdenoise = None
        self.bus_watch_id = None

        self._init_host = host
        self._init_port = port
        self._init_freq = frequency

        self._build_pipeline(host, port, frequency)
        self._build_ui()
        self._start()

    # ---------------- Pipeline (budowany raz, live-tuned przez GUI) --------

    def _build_pipeline(self, host, port, frequency):
        desc = (
            f'sdrsrc name=src mode=tcp host={host} port={port} '
            f'frequency={frequency} sample-rate=250000 gain=4.0 '
            f'auto-gain=false auto-gain-target-db=-18.0 '
            f'! queue name=q1 max-size-buffers=0 max-size-bytes=0 '
            f'max-size-time=200000000 leaky=downstream '
            f'! sdrdemod name=demod mode=fm stereo=true max-deviation=75000 '
            f'audio-rate=48000 audio-cutoff=15000 tau=50 freq-offset=0 '
            f'if-bandwidth=0 auto-bandwidth=false '
            f'! audioconvert ! audioresample '
            f'! audio/x-raw,rate=48000,channels=1 '
            f'! sdrdenoise name=denoise enabled=true threshold-db=8 '
            f'alpha-up=0.01 alpha-down=0.0001 interpolate=false '
            f'auto-interpolate=false interp-strength=0.5 '
            f'! queue name=q2 max-size-buffers=0 max-size-bytes=0 '
            f'max-size-time=1000000000 '
            f'! autoaudiosink sync=false'
        )
        self.pipeline = Gst.parse_launch(desc)
        self.sdrsrc = self.pipeline.get_by_name("src")
        self.sdrdemod = self.pipeline.get_by_name("demod")
        self.sdrdenoise = self.pipeline.get_by_name("denoise")

        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message::error", self._on_bus_error)
        bus.connect("message::warning", self._on_bus_warning)
        bus.connect("message::eos", self._on_bus_eos)

    def _start(self):
        self.pipeline.set_state(Gst.State.PLAYING)

    def _stop(self):
        self.pipeline.set_state(Gst.State.NULL)

    def _on_bus_error(self, bus, msg):
        err, dbg = msg.parse_error()
        print(f"[GST ERROR] {err} ({dbg})", file=sys.stderr)
        self.status_lbl.set_label(f"Błąd: {err}")

    def _on_bus_warning(self, bus, msg):
        w, dbg = msg.parse_warning()
        print(f"[GST WARN] {w} ({dbg})", file=sys.stderr)

    def _on_bus_eos(self, bus, msg):
        self.status_lbl.set_label("Koniec strumienia (EOS)")

    def _reconnect(self, host, port, frequency):
        """
        JEDYNE miejsce, gdzie robimy pełny restart pipeline'u - bo zmiana
        host/port/mode w sdrsrc zmienia socket/urządzenie i nie da się tego
        przeliczyć w locie tak jak gain/bandwidth/denoise.
        Wywoływane wyłącznie z przycisku "Zastosuj połączenie".
        """
        self.status_lbl.set_label("Przełączam źródło...")
        self._stop()
        self.pipeline = None
        self._build_pipeline(host, port, frequency)
        self._rebind_prop_widgets()
        self._start()
        self.status_lbl.set_label("Połączono")

    # ---------------- UI ----------------------------------------------------

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

    # -- Połączenie (host/port) - to jedyna sekcja, która restartuje pipeline

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

        hint = Gtk.Label(
            label="Wszystko poniżej (gain, bandwidth, denoise, strojenie) "
                  "działa na żywo bez restartu.",
            xalign=0, wrap=True)
        hint.add_css_class("dim-label")
        box.append(hint)

        return frame

    def _on_apply_connection(self, btn):
        host = self.host_entry.get_text().strip()
        try:
            port = int(self.port_entry.get_text().strip())
        except ValueError:
            self.status_lbl.set_label("Nieprawidłowy port")
            return
        freq = self.freq_slider.scale.get_value() if hasattr(self, "freq_slider") else self._init_freq
        self._reconnect(host, port, int(freq))

    # -- Strojenie częstotliwości (live, bo sdrsrc.frequency jest live-settable)

    def _build_tuning_group(self):
        frame, box = self._group("Strojenie (na żywo)")
        self.freq_slider = PropSlider(
            self.sdrsrc, "frequency", "Częstotliwość",
            lo=24_000_000, hi=1_766_000_000, step=1000, digits=0,
            value=self._init_freq, suffix=" Hz")
        box.append(self.freq_slider)

        self.freq_offset_slider = PropSlider(
            self.sdrdemod, "freq-offset", "Offset NCO",
            lo=-100000, hi=100000, step=100, digits=0, suffix=" Hz")
        box.append(self.freq_offset_slider)

        self.stereo_switch = PropSwitch(self.sdrdemod, "stereo", "Stereo")
        box.append(self.stereo_switch)

        return frame

    # -- Gain / AGC

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

        # gdy AGC włączone, ręczny suwak gain jest bez sensu (auto-loop go nadpisuje)
        self._on_auto_gain_toggle(self.auto_gain_switch.get_active())

        return frame

    def _on_auto_gain_toggle(self, active):
        self.gain_slider.set_sensitive_live(not active)
        self.auto_gain_target_slider.set_sensitive_live(active)

    # -- IF Bandwidth / S-N hunt

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

        hint = Gtk.Label(
            label="Auto-bandwidth nadpisuje if-bandwidth dopóki włączone.",
            xalign=0, wrap=True)
        hint.add_css_class("dim-label")
        box.append(hint)

        self._on_auto_bw_toggle(self.auto_bw_switch.get_active())

        return frame

    def _on_auto_bw_toggle(self, active):
        self.if_bw_slider.set_sensitive_live(not active)

    # -- Denoise / interpolacja

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

        sep = Gtk.Separator(orientation=Gtk.Orientation.HORIZONTAL)
        box.append(sep)

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

    # -- Transport (play/pause/stop) - stan pipeline'u, nie property elementów

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

    # -- Po _reconnect() trzeba podpiąć widgety pod nowe obiekty elementów,
    #    bo stare Gst.Element zostały zniszczone razem ze starym pipeline'em.

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

        # re-aplikuj aktualne wartości GUI na nowe elementy (bo defaulty
        # świeżo zbudowanego pipeline'u mogą różnić się od tego co user ustawił)
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


class SdrApp(Adw.Application):
    def __init__(self, host, port, frequency):
        super().__init__(application_id="pl.local.sdrfmgui")
        self._host = host
        self._port = port
        self._frequency = frequency

    def do_activate(self):
        win = SdrWindow(self, self._host, self._port, self._frequency)
        win.present()


def main():
    import argparse
    p = argparse.ArgumentParser(description="GTK4 GUI dla sdrsrc/sdrdemod/sdrdenoise")
    p.add_argument("--host", default="192.168.1.1")
    p.add_argument("--port", type=int, default=1234)
    p.add_argument("--frequency", type=int, default=92_000_000)
    args = p.parse_args()

    app = SdrApp(args.host, args.port, args.frequency)
    return app.run(None)


if __name__ == "__main__":
    sys.exit(main())
