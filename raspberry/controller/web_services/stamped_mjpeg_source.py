#!/usr/bin/env python3
"""stamped_mjpeg_source.py — sorgente MJPEG *timbrata* per la misura glass-to-glass.

Cattura con picamera2, disegna sul frame il tempo di cattura (Pi epoch ms, low
24 bit) come riga di blocchi bianco/nero leggibili da canvas nel browser, poi fa
l'encode JPEG (simplejpeg) e scrive i JPEG concatenati su stdout. È usato da
https_server._handle_mjpeg_fullstack con `stamp=1`: il server estrae i frame via
marker SOI/EOI (come per rpicam-vid) e li impacchetta multipart, invariato.

Il timbro è applicato SUBITO dopo la cattura → la latenza misurata nel browser
(now - (timbro + K), con K = ponte d'orologio dal ws_pong) copre encode JPEG +
trasporto + decode + display. Esclude sensore/ISP (comune alle due pipeline).

Layout timbro (riga in alto a sinistra, BS px per blocco):
  blocchi 0..23 = bit LSB-first di round(time.time()*1000) & 0xFFFFFF
  blocco 24 = bianco (guard), blocco 25 = nero (guard)  -> verifica posizione.
Stesso schema riusato dal ramo H264 (Stadio 3).

Uso: python3 stamped_mjpeg_source.py WIDTH HEIGHT FPS NFRAMES [QUALITY]
"""
import signal
import sys
import time

import numpy as np
import simplejpeg
from picamera2 import Picamera2

BITS = 24
BS = 16          # lato blocco in px (16 -> robusto a JPEG/H264, validato 300/300)
GUARD = 2        # blocchi guardia: bianco, nero

_picam = None


def _draw_stamp(img, value):
    for i in range(BITS):
        img[0:BS, i * BS:(i + 1) * BS, :] = 255 if ((value >> i) & 1) else 0
    img[0:BS, BITS * BS:(BITS + 1) * BS, :] = 255          # guard bianco
    img[0:BS, (BITS + 1) * BS:(BITS + 2) * BS, :] = 0       # guard nero


def _cleanup(*_a):
    global _picam
    try:
        if _picam is not None:
            _picam.stop()
    except Exception:
        pass
    sys.exit(0)


def main():
    global _picam
    W = int(sys.argv[1]); H = int(sys.argv[2])
    FPS = float(sys.argv[3]); N = int(sys.argv[4])
    quality = int(sys.argv[5]) if len(sys.argv) > 5 else 85

    _picam = Picamera2()
    cfg = _picam.create_video_configuration(
        main={"size": (W, H), "format": "RGB888"},
        controls={"FrameRate": FPS},
    )
    _picam.configure(cfg)
    _picam.start()
    # Rilascia la camera anche se il server ci termina (client disconnesso):
    # senza stop() la CSI resta occupata e il restart di MediaMTX fallisce.
    signal.signal(signal.SIGTERM, _cleanup)
    signal.signal(signal.SIGINT, _cleanup)

    out = sys.stdout.buffer
    sent = 0
    try:
        while sent < N:
            arr = _picam.capture_array("main")
            if arr is None or arr.ndim != 3 or arr.shape[1] < (BITS + GUARD) * BS:
                continue
            ts = round(time.time() * 1000) & 0xFFFFFF
            _draw_stamp(arr, ts)
            jpg = simplejpeg.encode_jpeg(np.ascontiguousarray(arr), quality=quality, colorspace="BGR")
            try:
                out.write(jpg)
                out.flush()
            except (BrokenPipeError, OSError):
                break
            sent += 1
    finally:
        try:
            _picam.stop()
        except Exception:
            pass


if __name__ == "__main__":
    main()
