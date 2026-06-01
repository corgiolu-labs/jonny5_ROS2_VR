#!/usr/bin/env python3
"""calibrate_world_bias.py

Ricalibra `config_runtime/imu/imu_world_bias.json` catturando ~20s di
quaternione del BNO085 mentre il robot e' in HOME meccanico.

Flusso:
1. Connessione al WS locale.
2. Verifica che il robot sia in HOME (giunti ~90 +/- 2 gradi).
3. Verifica che la telemetria IMU sia valida.
4. Acquisisce SAMPLES_TOTAL campioni a SAMPLE_RATE_HZ.
5. Calcola la media quaternionica robusta (sign-flip alignment).
6. Estrae la sola componente di yaw del quaternione medio -> nuovo
   `world_bias` puro su asse Z.
7. Backup del precedente `imu_world_bias.json` e scrittura del nuovo.

NON tocca firmware o servizi: il file e' letto on-demand dai tool di
validazione e dal compare FK-vs-IMU della pagina FK/IK.
"""

from __future__ import annotations

import asyncio
import json
import math
import shutil
import ssl
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import websockets
from scipy.spatial.transform import Rotation as R


WS_URL = "wss://127.0.0.1:8557"
HOME_ANGLE_DEG = 90.0
TOLL_HOME_DEG = 3.0
SAMPLE_RATE_HZ = 30.0
ACQUIRE_DURATION_S = 20.0
SAMPLES_TOTAL = int(SAMPLE_RATE_HZ * ACQUIRE_DURATION_S)

WB_PATH = Path("/home/jonny5/raspberry5/config_runtime/imu/imu_world_bias.json")


def normalize_quat(q):
    n = float(np.linalg.norm(q))
    return q / n if n > 0.0 else q


def mean_quaternion(quats):
    """Media quaternionica con sign-flip alignment al primo campione."""
    arr = np.array(quats, dtype=float)
    norms = np.linalg.norm(arr, axis=1, keepdims=True)
    arr = arr / np.where(norms == 0.0, 1.0, norms)
    ref = arr[0].copy()
    for i in range(1, len(arr)):
        if float(np.dot(arr[i], ref)) < 0.0:
            arr[i] = -arr[i]
    avg = arr.mean(axis=0)
    return normalize_quat(avg)


async def receive_for(ws, duration_s, on_msg):
    """Drena messaggi WS per duration_s secondi (chiama on_msg(dict))."""
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        timeout = max(0.05, deadline - time.monotonic())
        try:
            raw = await asyncio.wait_for(ws.recv(), timeout=timeout)
        except asyncio.TimeoutError:
            break
        except Exception:
            break
        try:
            msg = json.loads(raw)
        except Exception:
            continue
        if isinstance(msg, dict):
            on_msg(msg)


async def send_home(ws):
    """Invia comando UART HOME e attende stabilizzazione."""
    payload = json.dumps({"type": "uart", "cmd": "HOME"})
    await ws.send(payload)


HOME_SETTLE_S = 5.0  # tempo di stabilizzazione dopo HOME prima dell'acquisizione


def extract_quat_wxyz(msg):
    keys_wxyz = ("imu_q_w", "imu_q_x", "imu_q_y", "imu_q_z")
    q = [msg.get(k) for k in keys_wxyz]
    if all(v is not None for v in q):
        try:
            return [float(x) for x in q]
        except (TypeError, ValueError):
            return None
    return None


def extract_servo_angles(msg):
    keys = ("servo_deg_B", "servo_deg_S", "servo_deg_G",
            "servo_deg_Y", "servo_deg_P", "servo_deg_R")
    try:
        vals = [msg.get(k) for k in keys]
        if all(v is not None for v in vals):
            return [float(v) for v in vals]
    except (TypeError, ValueError):
        pass
    return None


async def main():
    print(f"=== world_bias calibration  @ {datetime.now().isoformat()} ===")
    print(f"  WS={WS_URL}  samples={SAMPLES_TOTAL}  duration={ACQUIRE_DURATION_S}s")

    state = {"angles": None, "imu_valid": False, "imu_quat": None}

    def on_status(msg):
        a = extract_servo_angles(msg)
        if a:
            state["angles"] = a
        q = extract_quat_wxyz(msg)
        if q is not None:
            state["imu_quat"] = q
            state["imu_valid"] = True

    ssl_ctx = ssl.create_default_context()
    ssl_ctx.check_hostname = False
    ssl_ctx.verify_mode = ssl.CERT_NONE
    async with websockets.connect(WS_URL, ssl=ssl_ctx, max_size=2 ** 20) as ws:
        # 2 secondi per leggere stato iniziale
        print("\n[1/4] Lettura stato iniziale per 2.0 s ...")
        await receive_for(ws, 2.0, on_status)
        if state["angles"] is None:
            print("  ERROR: nessun servo angles ricevuto dal WS")
            return 1
        print(f"  servo_deg (pre-HOME) = {[round(a, 2) for a in state['angles']]}")
        print(f"  imu_valid (pre-HOME) = {state['imu_valid']}")

        # Invia comando HOME via UART e attendi stabilizzazione
        print(f"\n[1b] Invio comando UART HOME + settle {HOME_SETTLE_S:.1f} s ...")
        await send_home(ws)
        await receive_for(ws, HOME_SETTLE_S, on_status)
        print(f"  servo_deg (post-HOME) = {[round(a, 2) for a in state['angles']]}")
        print(f"  imu_valid (post-HOME) = {state['imu_valid']}")
        if state["imu_quat"]:
            print(f"  imu_quat_wxyz         = {[round(v, 4) for v in state['imu_quat']]}")
        if not state["imu_valid"] or state["imu_quat"] is None:
            print("  ERROR: IMU non valida dopo HOME; verifica BNO085")
            return 3
        print("  OK: robot in HOME e IMU valida")

        # Acquisizione
        captured = []
        def on_capture(msg):
            q = extract_quat_wxyz(msg)
            if q is not None:
                captured.append(q)

        print(f"\n[2/4] Acquisizione {SAMPLES_TOTAL} campioni per {ACQUIRE_DURATION_S:.1f} s ...")
        t0 = time.monotonic()
        await receive_for(ws, ACQUIRE_DURATION_S, on_capture)
        elapsed = time.monotonic() - t0
        print(f"  catturati {len(captured)} campioni in {elapsed:.2f} s ({len(captured)/elapsed:.1f} Hz)")

        if len(captured) < 50:
            print("  ERROR: meno di 50 campioni validi; annullo.")
            return 4

    # Calcolo
    print("\n[3/4] Media quaternionica e estrazione yaw assoluto ...")
    avg_wxyz = mean_quaternion(captured)
    avg_xyzw = [avg_wxyz[1], avg_wxyz[2], avg_wxyz[3], avg_wxyz[0]]
    rot = R.from_quat(avg_xyzw)
    yaw_deg, pitch_deg, roll_deg = (float(x) for x in rot.as_euler("ZYX", degrees=True))
    print(f"  quat medio (wxyz) = {[round(v, 6) for v in avg_wxyz.tolist()]}")
    print(f"  RPY medio (deg)   = roll={roll_deg:.3f}  pitch={pitch_deg:.3f}  yaw={yaw_deg:.3f}")

    yaw_rad = math.radians(yaw_deg)
    wb_quat_wxyz = [
        math.cos(yaw_rad / 2.0),
        0.0,
        0.0,
        math.sin(yaw_rad / 2.0),
    ]
    print(f"  world_bias quat_wxyz = {[round(v, 6) for v in wb_quat_wxyz]}")

    # Salvataggio
    print("\n[4/4] Backup + scrittura imu_world_bias.json ...")
    if WB_PATH.exists():
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        bak = WB_PATH.with_name(f"{WB_PATH.stem}.bak_{ts}.json")
        shutil.copy2(WB_PATH, bak)
        print(f"  backup: {bak}")

    new_cfg = {
        "description": (
            "BNO085 Rotation-Vector world-frame yaw bias (magnetometer-dependent). "
            "Refresh when BNO085 yaw reference drifts. "
            "Validators only; not consumed by operational path."
        ),
        "quat_wxyz": wb_quat_wxyz,
        "rpy_deg": [0.0, 0.0, yaw_deg],
        "calibrated_at": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S"),
        "samples": len(captured),
        "duration_s": ACQUIRE_DURATION_S,
        "rate_hz_target": SAMPLE_RATE_HZ,
    }
    WB_PATH.write_text(json.dumps(new_cfg, indent=2))
    print(f"  saved: {WB_PATH}")
    print(f"\nOK -- world_bias ricalibrato.  yaw_deg = {yaw_deg:.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()) or 0)
