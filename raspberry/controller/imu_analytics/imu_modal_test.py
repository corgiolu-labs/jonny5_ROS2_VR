"""
Modal test semplificato (forced response): micro-step solo su polso (Y/P/R), rilascio, acquisizione IMU, PSD.

Non è analisi modale certificata: misura di risposta dinamica a eccitazione debole controllata.
Base, spalla e gomito non vengono mai modificati rispetto a HOME.
"""

from __future__ import annotations

import asyncio
import logging
from collections.abc import Awaitable, Callable

from controller.imu_analytics.imu_vibration_peaks import capture_imu_quat_samples, top_vibration_peaks_hz
from controller.uart import uart_manager
from controller.web_services import settings_manager

logger = logging.getLogger("imu_modal_test")

# Limiti virtuali (gradi) allineati ai default routing / firmware-safe per Y/P/R.
_VIRT_MIN = [10, 10, 10, 10, 60, 60]
_VIRT_MAX = [170, 170, 170, 170, 120, 120]

_WRIST = (
    ("yaw", 3),
    ("pitch", 4),
    ("roll", 5),
)


def _clamp_virtual(virt: list[int | float]) -> list[int]:
    out: list[int] = []
    for i in range(6):
        mn = _VIRT_MIN[i] if i < len(_VIRT_MIN) else 10
        mx = _VIRT_MAX[i] if i < len(_VIRT_MAX) else 170
        v = int(round(float(virt[i])))
        out.append(max(mn, min(mx, v)))
    return out


def _build_setpose_t_virtual(virtual_pose: list[int], duration_ms: int, planner: str = "RTR3") -> str:
    current_settings = settings_manager.load()
    offsets = current_settings.get("offsets", settings_manager.DEFAULTS["offsets"])
    dirs = current_settings.get("dirs", settings_manager.DEFAULTS.get("dirs", [1, 1, 1, 1, 1, 1]))
    physical = settings_manager.virtual_to_physical(virtual_pose, offsets, dirs)
    return "SETPOSE_T " + " ".join(str(v) for v in physical) + f" {int(max(20, duration_ms))} {planner}"


def _home_virtual_pose() -> list[int]:
    current_settings = settings_manager.load()
    home = current_settings.get("home", settings_manager.DEFAULTS["home"])
    return _clamp_virtual([int(round(float(x))) for x in home])


async def _uart_ok(cmd: str, timeout_s: float = 3.5) -> tuple[bool, str]:
    ok, resp = await uart_manager.send_uart_command(cmd, timeout_s=timeout_s)
    return ok, (resp or "").strip()


async def ensure_safe_enable_home(
    *,
    home_duration_ms: int = 3500,
    settle_s: float = 4.0,
    planner: str = "RTR3",
) -> tuple[bool, str]:
    """SAFE → ENABLE → SETPOSE_T HOME, poi attesa stabilizzazione."""
    ok, r = await _uart_ok("SAFE", timeout_s=1.5)
    if not ok:
        return False, f"SAFE failed: {r}"
    await asyncio.sleep(0.12)
    ok, r = await _uart_ok("ENABLE", timeout_s=1.5)
    if not ok:
        return False, f"ENABLE failed: {r}"
    await asyncio.sleep(0.12)
    home_v = _home_virtual_pose()
    cmd = _build_setpose_t_virtual(home_v, home_duration_ms, planner=planner)
    ok, r = await _uart_ok(cmd, timeout_s=max(4.0, home_duration_ms / 1000.0 + 2.0))
    if not ok:
        return False, f"HOME SETPOSE_T failed: {r}"
    await asyncio.sleep(max(3.0, float(settle_s)))
    return True, ""


async def _wait_motion_rough(duration_ms: int) -> None:
    await asyncio.sleep(max(0.25, duration_ms / 1000.0 + 0.35))


def aggregate_modal_peaks(axis_peak_lists: list[list[float]], top_n: int = 3, merge_hz: float = 0.55) -> list[float]:
    """
    Unisce picchi delle tre eccitazioni: cluster per vicinanza in Hz, priorità al cluster con più occorrenze.
    """
    all_f = sorted(round(x, 2) for sub in axis_peak_lists for x in sub)
    if not all_f:
        return []
    clusters: list[list[float]] = []
    for f in all_f:
        placed = False
        for c in clusters:
            med = sum(c) / len(c)
            if abs(f - med) <= merge_hz:
                c.append(f)
                placed = True
                break
        if not placed:
            clusters.append([f])
    clusters.sort(key=lambda c: (-len(c), sum(c) / len(c)))
    out: list[float] = []
    for c in clusters[:top_n]:
        out.append(round(sum(c) / len(c), 2))
    return sorted(out)


async def run_imu_modal_test(
    emit_status: Callable[[str], Awaitable[None]] | None = None,
    *,
    step_deg: float = 2.0,
    hold_s: float = 0.5,
    capture_s: float = 10.0,
    capture_hz: float = 50.0,
    move_deg_s: float = 15.0,
    return_deg_s: float = 18.0,
    settle_home_s: float = 4.0,
    peaks_per_axis: int = 5,
    planner: str = "RTR3",
) -> tuple[list[float], str]:
    """
    Esegue sequenza SAFE/ENABLE/HOME + micro-step Y/P/R + acquisizione; ritorna (picchi aggregati Hz, errore).
    """
    step = abs(float(step_deg))
    if step > 4.0:
        step = 4.0

    async def _emit(msg: str) -> None:
        if emit_status:
            await emit_status(msg)
        logger.info("[MODAL TEST] %s", msg)

    await _emit("Modal test: SAFE → ENABLE → HOME…")
    ok, err = await ensure_safe_enable_home(settle_s=settle_home_s, planner=planner)
    if not ok:
        return [], err

    home_v = _home_virtual_pose()

    move_ms = int(max(250, min(1200, 1000.0 * step / max(5.0, move_deg_s))))
    ret_ms = int(max(220, min(1000, 1000.0 * step / max(5.0, return_deg_s))))

    per_axis_peaks: list[list[float]] = []

    for name, idx in _WRIST:
        await _emit(f"Modal test: wrist {name} micro-step +{step:.1f}°…")
        target = list(home_v)
        target[idx] = int(round(float(target[idx]) + step))
        target = _clamp_virtual(target)
        if target[idx] == home_v[idx]:
            logger.warning("[MODAL TEST] clamp blocked %s excitation — skip axis", name)
            per_axis_peaks.append([])
            continue

        cmd_out = _build_setpose_t_virtual(target, move_ms, planner=planner)
        u_ok, u_r = await _uart_ok(cmd_out, timeout_s=4.0)
        if not u_ok:
            return [], f"SETPOSE_T {name} out failed: {u_r}"
        await _wait_motion_rough(move_ms)
        await asyncio.sleep(float(hold_s))

        cmd_home = _build_setpose_t_virtual(home_v, ret_ms, planner=planner)
        u_ok, u_r = await _uart_ok(cmd_home, timeout_s=4.0)
        if not u_ok:
            return [], f"SETPOSE_T {name} return HOME failed: {u_r}"
        await _wait_motion_rough(ret_ms)
        await asyncio.sleep(0.2)

        await _emit(f"Modal test: capturing IMU after {name} release (~{capture_s:.0f}s)…")
        rows, cap_err = await capture_imu_quat_samples(duration_s=capture_s, target_hz=capture_hz)
        if cap_err:
            logger.warning("[MODAL TEST] capture %s: %s", name, cap_err)
            per_axis_peaks.append([])
            continue
        peaks, perr = top_vibration_peaks_hz(rows, top_n=peaks_per_axis, min_hz=0.25, prominence_frac=0.05)
        if perr:
            logger.warning("[MODAL TEST] peaks %s: %s", name, perr)
            per_axis_peaks.append([])
        else:
            per_axis_peaks.append(peaks)

    merged = aggregate_modal_peaks(per_axis_peaks, top_n=3)
    if not merged and any(per_axis_peaks):
        flat = [x for sub in per_axis_peaks for x in sub]
        merged = sorted({round(x, 2) for x in flat})[:3]
    if not merged:
        return [], "no modal peaks (check IMU / UART)"
    return merged, ""
