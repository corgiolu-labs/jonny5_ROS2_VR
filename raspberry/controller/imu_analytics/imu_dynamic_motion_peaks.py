"""
Conversione campioni telemetry WS → serie temporale quaternione per PSD (transitori self-test).

L'analisi Welch / picchi resta in `controller.imu_analytics.imu_vibration_peaks.top_vibration_peaks_hz`.
"""

from __future__ import annotations

from typing import Any


def telemetry_msgs_phase_move(
    msgs: list[dict[str, Any]],
    *,
    base_t: float,
    t_send_mono: float,
    t_done_mono: float,
) -> list[dict[str, Any]]:
    """
    Segmento C_phase_move: campioni IMU con t assoluto in [t_send_mono, t_done_mono).
    Richiede _sample_t_s = _recv_mono - base_t (da collect_telemetry + _annotate_time).
    """
    out: list[dict[str, Any]] = []
    for m in msgs:
        if not bool(m.get("imu_valid", False)):
            continue
        if not all(k in m for k in ("imu_q_w", "imu_q_x", "imu_q_y", "imu_q_z")):
            continue
        ts = float(m.get("_sample_t_s", 0.0))
        t_abs = float(base_t) + ts
        if t_send_mono <= t_abs < t_done_mono:
            out.append(m)
    out.sort(key=lambda x: float(x.get("_sample_t_s", 0.0)))
    return out


def telemetry_msgs_to_quat_rows(msgs: list[dict[str, Any]]) -> list[dict[str, float]]:
    """Estrae t_s (relativo al primo campione valido) e quaternione wxyz da messaggi telemetry."""
    valid: list[dict[str, Any]] = []
    for m in msgs:
        if not bool(m.get("imu_valid", False)):
            continue
        if not all(k in m for k in ("imu_q_w", "imu_q_x", "imu_q_y", "imu_q_z")):
            continue
        valid.append(m)
    if not valid:
        return []
    t0 = float(valid[0].get("_sample_t_s", 0.0))
    rows: list[dict[str, float]] = []
    for m in valid:
        rows.append(
            {
                "t_s": float(m.get("_sample_t_s", 0.0)) - t0,
                "imu_q_w": float(m["imu_q_w"]),
                "imu_q_x": float(m["imu_q_x"]),
                "imu_q_y": float(m["imu_q_y"]),
                "imu_q_z": float(m["imu_q_z"]),
            }
        )
    return rows
