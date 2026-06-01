"""
Stima picchi vibrazionali osservabili dall'IMU (PSD Welch su proxy di velocità angolare da quaternione).

Uso: fase finale del self-test Home. Non è analisi modale certificata: solo candidate peaks
in banda limitata dal filtro di fusione (Madgwick).
"""

from __future__ import annotations

import asyncio
import math
import os
import time
from typing import Any

import numpy as np
from scipy import signal as scipy_signal

from controller.teleop import shared_state

_TELEMETRY_FILE = os.environ.get("J5VR_TELEMETRY_FILE", "/dev/shm/j5vr_telemetry.json")
_TELEMETRY_FRESH_S = 1.5

_MIN_SAMPLES = 48
_DEFAULT_CAPTURE_S = 12.0
_DEFAULT_TARGET_HZ = 50.0


def _quat_conj_wxyz(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)


def _quat_mul_wxyz(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    w0, x0, y0, z0 = a
    w1, x1, y1, z1 = b
    return np.array(
        [
            w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1,
            w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1,
            w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1,
            w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1,
        ],
        dtype=float,
    )


def _quat_normalize(q: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(q))
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / n


def _body_rate_from_quat_pair(q0_wxyz: np.ndarray, q1_wxyz: np.ndarray, dt: float) -> np.ndarray:
    if dt <= 1e-9:
        return np.zeros(3)
    q0 = _quat_normalize(np.asarray(q0_wxyz, dtype=float))
    q1 = _quat_normalize(np.asarray(q1_wxyz, dtype=float))
    qr = _quat_mul_wxyz(_quat_conj_wxyz(q0), q1)
    if qr[0] < 0.0:
        qr = -qr
    w, x, y, z = qr
    v = np.array([x, y, z], dtype=float)
    vn = float(np.linalg.norm(v))
    if vn < 1e-12:
        return np.zeros(3)
    angle = 2.0 * math.atan2(vn, w)
    axis = v / vn
    return axis * (angle / dt)


def _sample_rate_hz(t_s: np.ndarray) -> float:
    dt = np.diff(t_s)
    dt = dt[dt > 1e-9]
    if dt.size == 0:
        return 1.0
    return float(1.0 / np.median(dt))


def _omega_norm_series(t_s: np.ndarray, qw: np.ndarray, qx: np.ndarray, qy: np.ndarray, qz: np.ndarray) -> np.ndarray:
    n = len(t_s)
    wx = np.zeros(n)
    wy = np.zeros(n)
    wz = np.zeros(n)
    for i in range(1, n):
        dt = float(t_s[i] - t_s[i - 1])
        q0 = np.array([qw[i - 1], qx[i - 1], qy[i - 1], qz[i - 1]])
        q1 = np.array([qw[i], qx[i], qy[i], qz[i]])
        om = _body_rate_from_quat_pair(q0, q1, dt)
        wx[i], wy[i], wz[i] = om[0], om[1], om[2]
    return np.sqrt(wx * wx + wy * wy + wz * wz)


def _welch_psd(
    x: np.ndarray,
    fs: float,
    nperseg: int | None,
    *,
    welch_detrend: str | bool = "linear",
) -> tuple[np.ndarray, np.ndarray]:
    x = np.asarray(x, dtype=float)
    n = len(x)
    if n < 8:
        return np.array([]), np.array([])
    seg = nperseg if nperseg is not None else min(256, max(32, n // 4))
    seg = min(seg, n)
    f, pxx = scipy_signal.welch(x, fs=fs, nperseg=seg, noverlap=seg // 2, detrend=welch_detrend)
    return f, pxx


def _pick_peaks(
    f: np.ndarray,
    pxx: np.ndarray,
    max_peaks: int,
    min_hz: float,
    max_hz: float,
    prominence_frac: float,
) -> list[tuple[float, float]]:
    """Ritorna lista (hz, prominence) ordinata per prominence decrescente."""
    if f.size == 0:
        return []
    mask = (f >= min_hz) & (f <= max_hz)
    ff = f[mask]
    pp = pxx[mask]
    if ff.size < 3:
        return []
    prom = float(np.max(pp) * prominence_frac) if np.max(pp) > 0 else 0.0
    peaks, props = scipy_signal.find_peaks(pp, prominence=max(prom, 1e-20))
    order = np.argsort(props["prominences"])[::-1]
    out: list[tuple[float, float]] = []
    for i in order[:max_peaks]:
        idx = peaks[i]
        out.append((float(ff[idx]), float(props["prominences"][i])))
    return out


def top_vibration_peaks_hz(
    rows: list[dict[str, Any]],
    *,
    top_n: int = 3,
    min_hz: float = 0.25,
    prominence_frac: float = 0.06,
    detrend_omega_linear: bool = False,
    min_samples: int | None = None,
) -> tuple[list[float], str | None]:
    """
    Estrae fino a top_n frequenze (Hz) dal canale ||omega|| stimato da quaternione.
    Ordine prominence (Welch), poi dedupe per vicinanza; output ordinato per Hz crescente.
    Se detrend_omega_linear: rimuove rampa lenta su ||omega|| prima della PSD (Welch senza detrend interno).
    Ritorna (peaks, errore_opzionale).
    """
    ms = _MIN_SAMPLES if min_samples is None else int(min_samples)
    if len(rows) < ms:
        return [], f"too few samples ({len(rows)} < {ms})"
    t_s = np.array([float(r["t_s"]) for r in rows], dtype=float)
    qw = np.array([float(r["imu_q_w"]) for r in rows], dtype=float)
    qx = np.array([float(r["imu_q_x"]) for r in rows], dtype=float)
    qy = np.array([float(r["imu_q_y"]) for r in rows], dtype=float)
    qz = np.array([float(r["imu_q_z"]) for r in rows], dtype=float)
    fs = _sample_rate_hz(t_s)
    nyq = 0.49 * fs
    max_hz = min(28.0, nyq)
    if max_hz <= min_hz + 0.5:
        return [], "Nyquist too low for peak search"
    om = _omega_norm_series(t_s, qw, qx, qy, qz)
    if detrend_omega_linear:
        om = scipy_signal.detrend(om, type="linear")
        f, pxx = _welch_psd(om, fs=fs, nperseg=None, welch_detrend=False)
    else:
        f, pxx = _welch_psd(om, fs=fs, nperseg=None)
    raw = _pick_peaks(f, pxx, max_peaks=12, min_hz=min_hz, max_hz=max_hz, prominence_frac=prominence_frac)
    if not raw:
        return [], None
    raw.sort(key=lambda x: -x[1])
    picked: list[float] = []
    min_sep = 0.35
    for hz, _prom in raw:
        if not any(abs(hz - p) < min_sep for p in picked):
            picked.append(hz)
        if len(picked) >= top_n:
            break
    display = sorted(picked[:top_n])
    return [round(h, 2) for h in display], None


async def capture_imu_quat_samples(
    duration_s: float = _DEFAULT_CAPTURE_S,
    target_hz: float = _DEFAULT_TARGET_HZ,
) -> tuple[list[dict[str, float]], str | None]:
    """
    Campiona quaternione IMU da telemetria file (stesso percorso del bridge SPI).
    """
    start = time.monotonic()
    rows: list[dict[str, float]] = []
    interval = 1.0 / max(1.0, float(target_hz))
    deadline = start + float(duration_s)
    next_sample_at = start

    while time.monotonic() < deadline:
        now = time.monotonic()
        if now < next_sample_at:
            await asyncio.sleep(min(0.002, next_sample_at - now))
            continue
        next_sample_at += interval

        tel = shared_state.read_telemetry_from_file()
        if not isinstance(tel, dict):
            continue
        try:
            fresh = os.path.isfile(_TELEMETRY_FILE) and (time.time() - os.path.getmtime(_TELEMETRY_FILE)) <= _TELEMETRY_FRESH_S
        except OSError:
            fresh = False
        if not fresh or not bool(tel.get("imu_valid", False)):
            continue
        if not all(k in tel for k in ("imu_q_w", "imu_q_x", "imu_q_y", "imu_q_z")):
            continue
        rows.append(
            {
                "t_s": float(now - start),
                "imu_q_w": float(tel["imu_q_w"]),
                "imu_q_x": float(tel["imu_q_x"]),
                "imu_q_y": float(tel["imu_q_y"]),
                "imu_q_z": float(tel["imu_q_z"]),
            }
        )

    if len(rows) < _MIN_SAMPLES:
        return rows, f"insufficient IMU samples ({len(rows)}); check IMU valid / SPI telemetry"
    return rows, None


async def measure_vibration_peaks_hz(
    duration_s: float = _DEFAULT_CAPTURE_S,
    target_hz: float = _DEFAULT_TARGET_HZ,
    top_n: int = 3,
) -> tuple[list[float], str | None]:
    """Cattura + analisi; ritorna (lista Hz, messaggio errore se capture fallita)."""
    rows, err = await capture_imu_quat_samples(duration_s=duration_s, target_hz=target_hz)
    if err:
        return [], err
    peaks, perr = top_vibration_peaks_hz(rows, top_n=top_n)
    if perr:
        return [], perr
    return peaks, None
