#!/usr/bin/env python3
"""measure_mjpeg_fullstack.py

Misura la pipeline MJPEG full-stack a parita' di condizioni con MediaMTX/WebRTC.

Differenza con measure_mjpeg_baseline.py (encoder isolato):
- measure_mjpeg_baseline.py:  rpicam-vid stdout, nessun trasporto applicativo
                              => misura SOLO il costo encoder (~6% CPU)
- measure_mjpeg_fullstack.py: HTTPS client -> Python multipart server -> rpicam-vid
                              => misura la pipeline OPERATIVA COMPLETA
                              => confronto apples-to-apples con MediaMTX/WebRTC

Lo script:
1. Apre una connessione HTTPS verso /api/mjpeg-fullstack del dashboard server
   (che internamente ferma MediaMTX e avvia rpicam-vid MJPEG)
2. Riceve lo stream multipart, conta i boundary e registra timestamp inter-frame
3. In parallelo campiona CPU/RAM/temp del Pi a 1 Hz
4. Stima la latenza come (inter-frame interval) * buffer_depth (9 per MJPEG TCP)
5. Restituisce JSON con statistiche complete

Usage:
  python3 measure_mjpeg_fullstack.py --profile lowlatency [--all]
  python3 measure_mjpeg_fullstack.py --all
"""

from __future__ import annotations

import argparse
import json
import socket
import ssl
import statistics
import sys
import threading
import time
import urllib.request

JPEG_SOI = b"\xff\xd8"
JPEG_EOI = b"\xff\xd9"
BOUNDARY = b"--jonny5frame"
BUFFER_DEPTH = 9.0  # profondita' tipica TCP+browser per MJPEG (Cap.10)

PROFILES = {
    "lowlatency":   (800,  450,  120),
    "zoomfriendly": (1280, 720,  60),
    "inspection":   (1920, 1080, 30),
    "maxres":       (3840, 2160, 14),
    "initial":      (1280, 720,  30),
}

# Path filesystem usati dal sampler.
_THERMAL = "/sys/class/thermal/thermal_zone0/temp"
_STAT    = "/proc/stat"
_MEMINFO = "/proc/meminfo"


class SystemLoadSampler(threading.Thread):
    def __init__(self, interval_s: float = 1.0):
        super().__init__(daemon=True)
        self.interval_s = interval_s
        self._stop = threading.Event()
        self.cpu_samples: list[float] = []
        self.ram_samples: list[float] = []
        self.temp_samples: list[float] = []

    def stop(self):
        self._stop.set()

    def _read_cpu(self):
        try:
            with open(_STAT) as f:
                line = f.readline()
            parts = line.split()
            if parts[0] != "cpu":
                return None
            vals = [int(x) for x in parts[1:]]
            idle = vals[3] + (vals[4] if len(vals) > 4 else 0)
            return sum(vals), idle
        except Exception:
            return None

    def _read_ram(self):
        try:
            mt = ma = None
            with open(_MEMINFO) as f:
                for line in f:
                    if line.startswith("MemTotal:"):
                        mt = int(line.split()[1])
                    elif line.startswith("MemAvailable:"):
                        ma = int(line.split()[1])
                    if mt is not None and ma is not None:
                        break
            return 100.0 * (mt - ma) / mt if mt and ma else None
        except Exception:
            return None

    def _read_temp(self):
        try:
            with open(_THERMAL) as f:
                return float(f.read().strip()) / 1000.0
        except Exception:
            return None

    def run(self):
        prev = self._read_cpu()
        if self._stop.wait(self.interval_s):
            return
        while not self._stop.is_set():
            cur = self._read_cpu()
            if prev and cur:
                d_total = cur[0] - prev[0]
                d_idle  = cur[1] - prev[1]
                if d_total > 0:
                    pct = 100.0 * (1.0 - d_idle / d_total)
                    self.cpu_samples.append(max(0.0, min(100.0, pct)))
            prev = cur
            r = self._read_ram()
            if r is not None: self.ram_samples.append(r)
            t = self._read_temp()
            if t is not None: self.temp_samples.append(t)
            if self._stop.wait(self.interval_s):
                break

    def aggregate(self) -> dict:
        def _agg(xs):
            if not xs:
                return {"mean": None, "min": None, "max": None, "n": 0}
            return {
                "mean": sum(xs) / len(xs),
                "min": min(xs),
                "max": max(xs),
                "n": len(xs),
            }
        return {
            "cpu_pct": _agg(self.cpu_samples),
            "ram_pct": _agg(self.ram_samples),
            "temp_c":  _agg(self.temp_samples),
        }


def measure_profile(profile_name: str, target_frames: int = 300) -> dict:
    if profile_name not in PROFILES:
        return {"status": "error", "reason": "unknown_profile"}
    width, height, fps = PROFILES[profile_name]

    url = (f"https://127.0.0.1:8443/api/mjpeg-fullstack"
           f"?width={width}&height={height}&fps={fps}"
           f"&target_frames={target_frames}&label={profile_name}")

    # TLS context senza verifica cert (self-signed)
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE

    # Sampler parallelo
    sampler = SystemLoadSampler(interval_s=1.0)
    sampler.start()
    t_start = time.monotonic()

    intervals_ms: list[float] = []
    last_t: float | None = None
    boundary_count = 0
    bytes_total = 0

    try:
        with urllib.request.urlopen(url, context=ctx, timeout=120) as resp:
            buf = b""
            search_window = max(4096, len(BOUNDARY) * 4)
            while True:
                chunk = resp.read(65536)
                if not chunk:
                    break
                bytes_total += len(chunk)
                buf += chunk
                pos = 0
                while True:
                    idx = buf.find(BOUNDARY, pos)
                    if idx < 0:
                        break
                    now_ms = (time.monotonic() - t_start) * 1000.0
                    if last_t is not None:
                        intervals_ms.append(now_ms - last_t)
                    last_t = now_ms
                    boundary_count += 1
                    pos = idx + len(BOUNDARY)
                # Mantieni solo gli ultimi byte per cross-chunk matching
                if len(buf) > search_window:
                    buf = buf[-len(BOUNDARY):]
    except Exception as e:
        sampler.stop()
        sampler.join(timeout=2.0)
        return {"status": "error", "reason": "stream_error", "detail": str(e),
                "profile": profile_name, "config": {"width": width, "height": height, "fps": fps}}

    sampler.stop()
    sampler.join(timeout=2.0)

    if len(intervals_ms) < 3:
        return {"status": "error", "reason": "too_few_frames", "n_intervals": len(intervals_ms),
                "profile": profile_name}

    # Stima latenza: inter-frame * buffer depth
    latencies = [iv * BUFFER_DEPTH for iv in intervals_ms]
    n = len(latencies)
    mean = statistics.fmean(latencies)
    mn = min(latencies)
    mx = max(latencies)
    std = statistics.pstdev(latencies) if n > 1 else 0.0

    return {
        "status": "ok",
        "profile": profile_name,
        "config": {"width": width, "height": height, "fps": fps},
        "n_frames": boundary_count,
        "n_intervals": n,
        "bytes_total": bytes_total,
        "elapsed_s": round(time.monotonic() - t_start, 2),
        "estimated_latency_ms": {
            "min": mn, "mean": mean, "max": mx, "std": std,
        },
        "buffer_depth_assumed": BUFFER_DEPTH,
        "system_load": sampler.aggregate(),
    }


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--profile", choices=list(PROFILES.keys()))
    p.add_argument("--all", action="store_true", help="esegui su tutti i profili")
    p.add_argument("--target-frames", type=int, default=300)
    args = p.parse_args()

    if not args.all and not args.profile:
        p.error("specificare --profile <name> oppure --all")

    profiles = list(PROFILES.keys()) if args.all else [args.profile]
    results: list[dict] = []
    for prof in profiles:
        print(f"\n=== {prof} ({PROFILES[prof][0]}x{PROFILES[prof][1]}@{PROFILES[prof][2]}) ===", file=sys.stderr)
        r = measure_profile(prof, target_frames=args.target_frames)
        results.append(r)
        if r.get("status") == "ok":
            lat = r["estimated_latency_ms"]
            cpu = r["system_load"]["cpu_pct"]
            temp = r["system_load"]["temp_c"]
            print(f"  -> latency mean={lat['mean']:.1f} ms (min {lat['min']:.0f}, max {lat['max']:.0f}), "
                  f"CPU mean={cpu['mean']:.1f}%, T={temp['mean']:.1f}°C, "
                  f"frames={r['n_frames']}, elapsed={r['elapsed_s']}s", file=sys.stderr)
        else:
            print(f"  -> ERROR: {r}", file=sys.stderr)
        # Pausa fra profili: lascia mediamtx ripartire e si stabilizzi
        if args.all:
            time.sleep(4.0)

    print(json.dumps(results, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
