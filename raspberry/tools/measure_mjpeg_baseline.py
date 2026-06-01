#!/usr/bin/env python3
"""measure_mjpeg_baseline.py

Riproduce sul Raspberry Pi la misura della pipeline video MJPEG baseline
descritta nel Capitolo 10 della tesi (sezione 10.1 e 10.2). Lo script:

1. Ferma temporaneamente il servizio jonny5-mediamtx (per liberare la camera);
2. Avvia rpicam-vid in modalita' MJPEG a 1280x720 @ 30 fps (configurazione
   storica della pipeline MJPEG baseline);
3. Conta i marker JPEG SOI (0xFFD8) sullo stdout binario di rpicam-vid e
   registra il timestamp di ogni frame;
4. Calcola statistiche inter-frame (media/min/max/std);
5. Stima la latenza video come (buffer_depth * inter-frame interval),
   coerentemente con la metodologia del Capitolo 10 sezione 10.1 della tesi
   (la profondita' di buffering tipica di MJPEG+TCP+browser e' ~9 frame);
6. Riavvia jonny5-mediamtx;
7. Stampa i risultati come JSON su stdout.

Nessun server HTTP esposto: la misura e' interamente locale, coerente con
quanto dichiarato nel Cap.10 ("script Python sviluppati per questo progetto.
Tali script acquisiscono i timestamp dei frame dal flusso video").

NON modifica file di configurazione del sistema: e' uno script di sola misura.
"""

from __future__ import annotations

import argparse
import json
import statistics
import subprocess
import sys
import threading
import time
from typing import Iterable


# Valori di default = profilo "baseline storico" del Cap.10 (1280x720@30).
# Possono essere sovrascritti da CLI per testare i profili equivalenti
# MediaMTX (low-latency, zoom-friendly, inspection, max-res).
DEFAULT_WIDTH = 1280
DEFAULT_HEIGHT = 720
DEFAULT_FRAMERATE = 30
DEFAULT_DURATION_S = 12
DEFAULT_TARGET_FRAMES = 300
BUFFER_DEPTH_MJPEG = 9.0  # profondita' tipica di buffering TCP+browser per MJPEG

JPEG_SOI = b"\xff\xd8"
MEDIAMTX_SERVICE = "jonny5-mediamtx.service"

# Path filesystem usati dal sampler carico computazionale.
_THERMAL_ZONE_PATH = "/sys/class/thermal/thermal_zone0/temp"
_PROC_STAT_PATH    = "/proc/stat"
_PROC_MEMINFO_PATH = "/proc/meminfo"


class SystemLoadSampler(threading.Thread):
    """Campiona CPU%/RAM%/temperatura a 1 Hz in un thread separato.

    Avviato prima della cattura MJPEG, fermato dopo. La stat CPU% si calcola
    come delta su due letture consecutive di /proc/stat (idle vs total jiffies).
    """

    def __init__(self, interval_s: float = 1.0) -> None:
        super().__init__(daemon=True)
        self.interval_s = max(0.5, min(3.0, interval_s))
        self._stop_evt = threading.Event()
        self.cpu_samples: list[float] = []
        self.ram_samples: list[float] = []
        self.temp_samples: list[float] = []

    def stop(self) -> None:
        self._stop_evt.set()

    def _read_cpu_jiffies(self) -> tuple[int, int] | None:
        try:
            with open(_PROC_STAT_PATH, "r") as f:
                line = f.readline()
            parts = line.split()
            if not parts or parts[0] != "cpu":
                return None
            vals = [int(x) for x in parts[1:]]
            if len(vals) < 5:
                return None
            idle = vals[3] + (vals[4] if len(vals) > 4 else 0)
            return sum(vals), idle
        except Exception:
            return None

    def _read_ram_pct(self) -> float | None:
        try:
            mt = ma = None
            with open(_PROC_MEMINFO_PATH, "r") as f:
                for line in f:
                    if line.startswith("MemTotal:"):
                        mt = int(line.split()[1])
                    elif line.startswith("MemAvailable:"):
                        ma = int(line.split()[1])
                    if mt is not None and ma is not None:
                        break
            if mt is None or ma is None or mt <= 0:
                return None
            return 100.0 * (mt - ma) / mt
        except Exception:
            return None

    def _read_temp_c(self) -> float | None:
        try:
            with open(_THERMAL_ZONE_PATH, "r") as f:
                return float(f.read().strip()) / 1000.0
        except Exception:
            return None

    def run(self) -> None:
        prev = self._read_cpu_jiffies()
        if self._stop_evt.wait(self.interval_s):
            return
        while not self._stop_evt.is_set():
            cur = self._read_cpu_jiffies()
            if prev is not None and cur is not None:
                d_total = cur[0] - prev[0]
                d_idle  = cur[1] - prev[1]
                if d_total > 0:
                    pct = 100.0 * (1.0 - d_idle / d_total)
                    self.cpu_samples.append(max(0.0, min(100.0, pct)))
            prev = cur
            ram = self._read_ram_pct()
            if ram is not None:
                self.ram_samples.append(ram)
            t = self._read_temp_c()
            if t is not None:
                self.temp_samples.append(t)
            if self._stop_evt.wait(self.interval_s):
                break

    def aggregate(self) -> dict:
        def _agg(xs: list[float]) -> dict:
            if not xs:
                return {"mean": None, "min": None, "max": None, "std": None, "n": 0}
            n = len(xs)
            mn = min(xs)
            mx = max(xs)
            mean = sum(xs) / n
            std = (sum((x - mean) ** 2 for x in xs) / n) ** 0.5 if n > 1 else 0.0
            return {"mean": mean, "min": mn, "max": mx, "std": std, "n": n}
        return {
            "cpu_pct": _agg(self.cpu_samples),
            "ram_pct": _agg(self.ram_samples),
            "temp_c":  _agg(self.temp_samples),
            "interval_s": self.interval_s,
        }


def _run(args: list[str], check: bool = True, capture: bool = False) -> subprocess.CompletedProcess:
    return subprocess.run(args, check=check,
                          stdout=subprocess.PIPE if capture else None,
                          stderr=subprocess.PIPE if capture else None,
                          text=True if capture else False)


def stop_mediamtx() -> None:
    """Stop jonny5-mediamtx via sudo (NOPASSWD configurato per l'utente)."""
    _run(["sudo", "-n", "/bin/systemctl", "stop", MEDIAMTX_SERVICE])


def start_mediamtx() -> None:
    _run(["sudo", "-n", "/bin/systemctl", "start", MEDIAMTX_SERVICE])


def capture_mjpeg_frames(target_n: int, duration_s: int,
                          width: int, height: int, framerate: int) -> list[float]:
    """Avvia rpicam-vid in modalita' MJPEG e raccoglie timestamp dei frame.

    Ritorna lista di timestamp (in secondi monotonic) di inizio di ogni frame
    JPEG identificato dal marker SOI sullo stdout.
    """
    args = [
        "rpicam-vid",
        "--codec", "mjpeg",
        "--width", str(width),
        "--height", str(height),
        "--framerate", str(framerate),
        "--timeout", str(duration_s * 1000),
        "--inline", "1",
        "--nopreview",
        "--output", "-",  # stdout
    ]
    proc = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
    timestamps: list[float] = []
    buf = b""
    deadline = time.monotonic() + duration_s + 2.0
    try:
        while time.monotonic() < deadline and len(timestamps) < target_n:
            chunk = proc.stdout.read(65536)
            if not chunk:
                break
            buf += chunk
            # Trova tutti i marker SOI (inizio JPEG)
            start = 0
            while True:
                idx = buf.find(JPEG_SOI, start)
                if idx < 0:
                    break
                timestamps.append(time.monotonic())
                start = idx + 2
                if len(timestamps) >= target_n:
                    break
            # Mantieni gli ultimi 2 byte per matching cross-buffer
            if len(buf) > 4:
                buf = buf[-2:]
    finally:
        try:
            proc.terminate()
            proc.wait(timeout=2.0)
        except Exception:
            try: proc.kill()
            except Exception: pass
    return timestamps


def stats_from_timestamps(timestamps: list[float],
                          width: int, height: int, framerate: int) -> dict:
    """Calcola inter-frame intervals (ms), stima latenza MJPEG (ms)."""
    if len(timestamps) < 3:
        return {"error": "too_few_frames", "n_frames_captured": len(timestamps)}
    diffs_ms = [(timestamps[i] - timestamps[i - 1]) * 1000.0 for i in range(1, len(timestamps))]
    n = len(diffs_ms)
    avg = statistics.fmean(diffs_ms)
    mn = min(diffs_ms)
    mx = max(diffs_ms)
    std = statistics.pstdev(diffs_ms) if n > 1 else 0.0
    latency_avg = avg * BUFFER_DEPTH_MJPEG
    latency_min = mn * BUFFER_DEPTH_MJPEG
    latency_max = mx * BUFFER_DEPTH_MJPEG
    latency_std = std * BUFFER_DEPTH_MJPEG
    return {
        "n_frames": n,
        "interframe_ms": {"min": mn, "mean": avg, "max": mx, "std": std},
        "estimated_latency_ms": {
            "min": latency_min, "mean": latency_avg, "max": latency_max, "std": latency_std,
        },
        "buffer_depth_assumed": BUFFER_DEPTH_MJPEG,
        "config": {
            "width": width, "height": height, "framerate": framerate,
            "codec": "mjpeg",
            "methodology": "inter-frame timing on rpicam-vid stdout; latency estimated as buffer_depth * mean(interframe)",
            "reference": "Cap.10 sez.10.1 della tesi (MJPEG baseline 1280x720@30, ~306 ms)",
        },
    }


def main() -> int:
    p = argparse.ArgumentParser(
        description="Misura latenza MJPEG baseline su Raspberry Pi (Cap.10).")
    p.add_argument("--width", type=int, default=DEFAULT_WIDTH)
    p.add_argument("--height", type=int, default=DEFAULT_HEIGHT)
    p.add_argument("--fps", type=int, default=DEFAULT_FRAMERATE)
    p.add_argument("--duration", type=int, default=0,
                   help="durata acquisizione in secondi; se 0 calcolata da target-frames/fps")
    p.add_argument("--target-frames", type=int, default=DEFAULT_TARGET_FRAMES)
    p.add_argument("--label", type=str, default="",
                   help="etichetta libera per il risultato (es. 'lowlatency')")
    args = p.parse_args()

    # Auto-calcola durata se non specificata: tempo necessario per target_frames
    # alla framerate richiesta, con margine 30% + 3s di buffer di stabilizzazione.
    if args.duration <= 0:
        args.duration = max(5, int(args.target_frames * 1.3 / max(1, args.fps)) + 3)

    t0 = time.monotonic()
    result: dict = {"status": "running", "phases": [], "label": args.label,
                    "target_frames": args.target_frames, "duration_s_calc": args.duration}

    try:
        result["phases"].append("stop_mediamtx")
        stop_mediamtx()
        # Piccola pausa per essere certi che il device camera sia rilasciato
        time.sleep(1.5)

        # Avvio sampler di carico computazionale (CPU/RAM/temp) durante
        # la cattura, in thread separato (1 Hz). Finalizzato in finally.
        sampler = SystemLoadSampler(interval_s=1.0)
        sampler.start()

        result["phases"].append("capture_mjpeg")
        ts = capture_mjpeg_frames(args.target_frames, args.duration,
                                   args.width, args.height, args.fps)
        result["raw_n_jpegs"] = len(ts)

        # Ferma il sampler ed aggrega le statistiche di carico.
        sampler.stop()
        sampler.join(timeout=2.0)
        result["system_load"] = sampler.aggregate()

        result["phases"].append("compute_stats")
        stats = stats_from_timestamps(ts, args.width, args.height, args.fps)
        result.update(stats)

        result["status"] = "ok"
    except Exception as e:
        result["status"] = "error"
        result["error"] = str(e)
        result["error_type"] = type(e).__name__
    finally:
        # Sempre riavvia mediamtx, anche in caso di errore
        result["phases"].append("restart_mediamtx")
        try:
            start_mediamtx()
        except Exception as e:
            result.setdefault("warnings", []).append(f"start_mediamtx_failed: {e}")
        result["elapsed_s"] = round(time.monotonic() - t0, 2)

    print(json.dumps(result, indent=2))
    return 0 if result.get("status") == "ok" else 1


if __name__ == "__main__":
    sys.exit(main())
