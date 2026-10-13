#!/usr/bin/env python3
"""stamp_lines.py — wall-clock timestamp for rpicam-vid's per-frame log lines.

rpicam-vid -v 2 prints "Viewfinder frame N" when frame N leaves libcamera (after ISP). Piping its
stderr through this script records the Pi wall time of that event per frame:

    rpicam-vid ... -v 2 -o /tmp/cam0.fifo 2> >(python3 stamp_lines.py > /tmp/frames.log)

Output lines: "<epoch_ms> <frame>". With the frame number burned into the image (annotate_cv
"%frame") and the phone clock row, capture-to-display latency = phone_ms + offset - epoch_ms(frame),
without the run-dependent bias of the "Video start timestamp" anchor.
"""
import sys
import time

for line in sys.stdin:
    if line.startswith("Viewfinder frame "):
        try:
            n = int(line.split()[2])
        except (IndexError, ValueError):
            continue
        sys.stdout.write("%d %d\n" % (int(time.time() * 1000), n))
        sys.stdout.flush()
