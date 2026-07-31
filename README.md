gst-launch-1.0 -v sdrsrc mode=tcp host=192.168.1.1 port=1234 frequency=92000000 sample-rate=250000 gain=4.0 ! queue max-size-buffers=0 max-size-bytes=0 max-size-time=200000000 leaky=downstream ! sdrdemod mode=fm stereo=true max-deviation=75000 audio-rate=48000 audio-cutoff=15000 tau=50 freq-offset=0 ! audioconvert ! audioresample ! audio/x-raw,rate=48000,channels=1 ! sdrdenoise enabled=true threshold-db=8 alpha-up=0.01 alpha-down=0.0001 ! queue max-size-buffers=0 max-size-bytes=0 max-size-time=1000000000 ! autoaudiosink sync=false



meson setup builddir --prefix=/usr

cd builddir

ninja

sudo ninja install


❯ gst-inspect-1.0 | grep sdr

sdrplugins:  sdrdemod: SDR AM/FM Demodulator

sdrplugins:  sdrdenoise: SDR Anomaly-based Noise Reduction

sdrplugins:  sdrsrc: SDR IQ Source





gnome dir containing  gnome shell 50 extension applet for fm/am // ~/.local/share/gnome-shell/extensions/$:unzip_here/ and navigate in www browser to https://extensions.gnome.org/local/ to manage (setup rtl-sdr -tcp -local etc add stations)
