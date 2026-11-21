#!/usr/bin/env python3
"""j5mini — minimal arm controller for JONNY5 (Raspberry Pi or Luckfox Aura).

One process replaces ws-teleop + the SPI bridge (or the ROS 2 container) for teleoperation:

  Quest app / web viewer  --wss :8557 JSON (same messages as ws-teleop)-->  j5mini
  j5mini --SPI (J5 frames, 100 Hz, 64 B or 128 B)--> STM32      (setpoints, telemetry, STATUS)
  j5mini --UART (ASCII lines, #seq)-------------> STM32      (ENABLE/SAFE/HOME/..., config push)

What it does (and nothing else):
  - SPI master at 100 Hz: the newest intent as a J5VR payload (in HEAD/HYBRID/ASSIST the head quaternion and the
    ASSIST arm targets interpolated on the headset's timestamps, smoothing_ms behind); after 150 ms without intents
    a neutral payload (same mode and heartbeat, no input: deadman released). One STATUS
    request every 20 transfers (5 Hz) for the diagnostics (deadman/armed/E-STOP bits).
  - UART owner: commands from the clients with #seq replies; on BOOT_READY (or 1.5 s after
    start) pushes the configuration lines from the config file, in order, with retries;
    STATUS? every 2 s for the robot state; ESTOP / SETPOSE_DONE lines forwarded.
  - Demo: the pose sequence saved in j5mini.json (settings.demo), round and round until the dashboard stops it, the
    robot leaves IDLE (STOP / SAFE / E-STOP) or a step fails. settings.demo_style "smooth" (default, 20/11/2026):
    one continuous curve through the poses (PCHIP, no stops) plus a roll oscillation (settings.demo_roll_deg),
    streamed as SETPOSE_T_HR every 40 ms; "steps": one SETPOSE per pose, the next at SETPOSE_DONE.
  - On entering HEAD/HYBRID/ASSIST (mode 3/4/5) re-sends SET_VR_PARAMS (2.5 s throttle), as ws-teleop.
  - Telemetry to the clients at 20 Hz: robot_state, servo_deg_*, IMU quaternion, deadman/armed/E-STOP.
  - Optional clock set from the headset (time_sync), the robot has no RTC and no internet.
  - Camera controls of the Quest app (zoom, brightness, wide/tele, focus, autofocus) forwarded to
    raspberry/tools/cam_stream.py over UDP, as ws-teleop; get_settings (offsets/dirs); HTTPS on 443
    for the app's GET /api/controller-mappings and POST /api/refocus-cameras.
  - The dashboard (minimal/dashboard, served on the HTTPS port): get_config / save_config (offsets, dirs,
    joint limits, SET_VR_PARAMS and SET_PWM_CONFIG lines, PARK / VR poses, speed, profile -> j5mini.json,
    with a .bak), get_kinematics / save_kinematics (POE axes -> dashboard/kinematics.json, used by the ASSIST
    gimbal too), ws_ping; its WebSocket goes through the same HTTPS port (/ws, tunnelled to a plain
    listener on 127.0.0.1:ws.local_port) so the browser accepts one certificate only.

  - PARK / TELEOPPOSE from the clients become a SETPOSE to the saved poses (settings "park" / "vr", virtual
    degrees), as ws-teleop did (the firmware's own PARK/TELEOPPOSE go to one factory pose), then teleop_pose_ack.
  - ASSIST (mode 5) as the robot had it: the gimbal of the old Pi (assist_gimbal.py, needs numpy) computes the
    B/S/G targets and the wrist quaternion; "point at an object" of the vision service moves the arm there.

Not here (on purpose): dashboard pages of the old Pi, ROS 2, captive portal, demo/self-test.

  python3 j5mini.py --config j5mini.json             # real hardware (spidev + pyserial)
  python3 j5mini.py --sim                            # no robot: the real firmware simulated on this PC
                                                     # (firmware/stm32/tools/fw_sim, needs gcc), dashboard
                                                     # on http://localhost:8558 (--http-port)
"""
from __future__ import annotations

import argparse
import asyncio
import collections
import json
import math
import os
import pathlib
import ssl
import struct
import sys
import threading
import time
from typing import Callable

try:
    import websockets                      # the server (serve()) and the closed-connection check in Server.handler
    from websockets.exceptions import ConnectionClosed as _WS_CLOSED   # submodule: not a lazy attribute of the package
except ImportError:                        # the module still imports without the package (helpers, tests)
    websockets = None
    _WS_CLOSED = ()

# ------------------------------------------------------------------ J5 protocol (firmware layout)

J5_TELEMETRY, J5_TEST_ECHO, J5_STATUS, J5_J5VR, J5_J5IK = 0x01, 0x02, 0x03, 0x04, 0x05
DIAG_BITS = ("deadman", "input_active", "armed", "freeze", "guard", "estop_active", "estop_bypass")
SERVO_KEYS = ("servo_deg_B", "servo_deg_S", "servo_deg_G", "servo_deg_Y", "servo_deg_P", "servo_deg_R")


def build_frame(ftype: int, seq: int, payload: bytes, length: int) -> bytes:
    p = bytes(payload).ljust(54, b"\0")[:54]
    f = b"J5" + bytes([1, ftype]) + struct.pack(">H", seq & 0xFFFF) + bytes([64, 0]) + p + b"\0\0"
    return f.ljust(length, b"\0")


def find_frame_window(prev: bytes, rx: bytes):
    """Newest complete J5 frame in the last two transfers. The STM32's circular DMA does not resynchronise on CS:
    once armed in the middle of a transfer every reply comes out a few bytes late and straddles two transfers
    (on the robot, 26/10/2026: 3 bytes with 64-byte transfers). Returns (frame, offset in the window) or None."""
    w = prev + rx
    for o in range(len(w) - 64, -1, -1):          # newest first
        f = w[o:o + 64]
        if f[:2] == b"J5" and f[2] == 1 and f[6] == 64 and f[7] == 0 and f[3] in (1, 2, 3, 4, 5):
            return f, o
    return None


def _axis(v):
    if isinstance(v, bool):
        return None
    if isinstance(v, int):
        return max(-32768, min(32767, v))
    if isinstance(v, float) and math.isfinite(v):
        return max(-32768, min(32767, int(round(max(-1.0, min(1.0, v)) * 32767))))
    return None


def _u8(v):
    if isinstance(v, bool):
        return None
    if isinstance(v, int):
        return max(0, min(255, v))
    if isinstance(v, float) and math.isfinite(v):
        return int(round(max(0.0, min(1.0, v)) * 255))
    return None


def intent_to_payload(d: dict) -> tuple[bytes | None, str | None]:
    """ws-teleop validate_and_build_intent() + j5vr_frame.build_setpoint_frame(), without the Pi's
    int16 +-1 full-scale quirk (fixed upstream on branch fix/j5vr-int16-axis)."""
    mode = d.get("mode")
    if not isinstance(mode, int) or isinstance(mode, bool) or not 0 <= mode <= 5:
        return None, "mode must be int 0..5"
    ax = {k: _axis(d.get(k)) for k in ("joy_x", "joy_y", "pitch", "yaw")}
    if any(v is None for v in ax.values()):
        return None, "joy_x/joy_y/pitch/yaw missing or not numeric"
    inten = _u8(d.get("intensity"))
    if inten is None:
        return None, "intensity missing or not numeric"
    grip = d.get("grip", 0) if d.get("grip", 0) in (0, 1) else 0
    hb = d.get("heartbeat", 0) if isinstance(d.get("heartbeat", 0), int) else 0

    def q(k, dflt):
        v = d.get(k)
        return max(-1.0, min(1.0, float(v))) if isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) else dflt

    def btn(k):
        v = d.get(k, 0)
        return v if isinstance(v, int) and 0 <= v <= 65535 else 0

    quat = (q("quat_w", 1.0), q("quat_x", 0.0), q("quat_y", 0.0), q("quat_z", 0.0))
    if mode not in (3, 4, 5):
        quat = (1.0, 0.0, 0.0, 0.0)
    if mode in (3, 5):
        ax = dict.fromkeys(ax, 0)
    p = bytearray(54)
    p[0] = mode
    struct.pack_into(">hhhh", p, 1, ax["joy_x"], ax["joy_y"], ax["pitch"], ax["yaw"])
    p[9], p[10] = inten, 1 if grip == 1 else 0
    struct.pack_into(">H", p, 11, hb & 0xFFFF)
    struct.pack_into(">ffff", p, 16, *quat)
    struct.pack_into(">HH", p, 32, btn("buttons_left"), btn("buttons_right"))
    return bytes(p), None


def with_mode5_arm(p: bytes, arm) -> bytes:
    """ASSIST B/S/G targets in the J5VR payload (bytes 36-45, marker 'I', as the old Pi's j5vr_frame.py).
    arm = (physical deg [3] | None, grip_active, hold_active, target_id)."""
    deg, grip_active, hold_active, tid = arm
    if p[0] != 5 or deg is None:
        return p
    b = bytearray(p)
    b[36], b[37] = ord("I"), 1 | (2 if grip_active else 0) | (4 if hold_active else 0)
    struct.pack_into(">H", b, 38, int(tid) & 0xFFFF)
    struct.pack_into(">hhh", b, 40, *(max(-32768, min(32767, int(round(float(v) * 100)))) for v in deg[:3]))
    return bytes(b)


def slerp(a, b, u: float):
    """Quaternion (w, x, y, z) between a (u=0) and b (u=1), shortest arc."""
    dot = sum(x * y for x, y in zip(a, b))
    if dot < 0.0:
        b, dot = tuple(-v for v in b), -dot
    if dot > 0.9995:
        q = [x + (y - x) * u for x, y in zip(a, b)]
    else:
        th = math.acos(min(1.0, dot))
        s = math.sin(th)
        wa, wb = math.sin((1.0 - u) * th) / s, math.sin(u * th) / s
        q = [wa * x + wb * y for x, y in zip(a, b)]
    n = math.sqrt(sum(v * v for v in q)) or 1.0
    return tuple(v / n for v in q)


def neutral_payload(mode: int, hb: int) -> bytes:
    p = bytearray(54)
    p[0] = mode
    struct.pack_into(">H", p, 11, hb & 0xFFFF)
    struct.pack_into(">f", p, 16, 1.0)
    return bytes(p)


# ------------------------------------------------------------------ core (deterministic, 10 ms steps)

class Core:
    STATUS_EVERY = 20           # one STATUS request every 20 transfers = 5 Hz at 100 Hz
    STATE_POLL_S = 2.0
    UART_TIMEOUT_S = 1.5
    BOOT_FALLBACK_S = 1.5
    VR_PARAMS_THROTTLE_S = 2.5

    def __init__(self, cfg: dict, spi, uart, clock: Callable[[], float] = time.monotonic,
                 log: Callable[[str], None] = print):
        self.cfg, self.spi, self.uart, self.clock, self.log = cfg, spi, uart, clock, log
        self.len = int(cfg.get("spi", {}).get("transfer_len", 64))
        self.link_timeout = cfg.get("link_timeout_ms", 150) / 1000.0
        # IMUON / IMUOFF no longer exist in the firmware (IMU always on): skipped if an older config has them
        self.boot_lines = [l for l in cfg.get("boot_commands", []) if l not in ("IMUON", "IMUOFF")]
        self.cfg_for_init = cfg                        # SETPOSE_INIT at BOOT_READY: last_pose or PARK
        self.auto_park_s = float(cfg.get("settings", {}).get("auto_park_s", POSE_DEFAULTS["auto_park_s"]))
        self.idle_since: float | None = None           # headset in a teleop mode with the grips released since
        self.auto_park_done = False                    # one PARK per idle period (until the grips are pressed)
        self.last_pose: list[int] | None = None        # servo angles seen in telemetry (kept across a reset)
        self.init_check_pending = False                # fallback push done: is the firmware fresh (all servos at 90)?
        self.vr_params_line = next((l for l in self.boot_lines if l.startswith("SET_VR_PARAMS")), None)
        self.t0 = clock()
        # setpoint
        self.payload: bytes | None = None
        self.payload_t = -1e9
        # smoothing (26/10/2026, jerky ASSIST on the robot): the app sends 60 intents/s from a 90 Hz frame loop
        # (gaps of 11 and 22 ms) and Wi-Fi adds jitter (measured 8..28 ms, up to 45). The firmware follows every
        # new target within a few ms, so the arm moved in uneven steps. In modes 3/4/5 the continuous part (head
        # quaternion, ASSIST B/S/G targets) is replayed smoothing_ms behind, interpolated on the headset's own
        # timestamps; grips, buttons and sticks stay immediate (releasing the grips still stops at once).
        self.smooth_s = max(0.0, float(cfg.get("smoothing_ms", 30))) / 1000.0
        # horizon ("Orizzonte" in the app, every mode): the firmware levels the wrist roll from the joint angles
        # (HORIZON line with the joint directions); trim_deg = correction from the wrist IMU (horizon_trim_step)
        self.horizon_on = bool(cfg.get("horizon_lock", False))
        self.horizon_trim = 0.0
        self.horizon_dirs = [1 if d >= 0 else -1 for d in cfg.get("settings", {}).get("dirs", [1] * 6)]
        self.horizon_hist: list = []
        self.samples: collections.deque = collections.deque(maxlen=90)   # (t_headset, t_local, quat, arm | None)
        self.last_mode, self.last_hb = 0, 0
        self.tx_seq, self.transfers = 0, 0
        self.prev_rx = b""
        self.last_frame_key = None
        # uart
        self.rx_buf = b""
        self.uart_seq = 0
        self.pending: dict[int, tuple[str, float, Callable | None]] = {}
        self.boot_queue: list[str] = []
        self.boot_done = False
        self.boot_retries = 0
        self.next_state_poll = self.t0 + 1.0
        self.last_vr_params = -1e9
        # state for clients
        self.robot_state = "UNKNOWN"
        self.telemetry: dict = {}
        self.diag: dict = {}
        self.events: list[dict] = []        # unsolicited lines, uart responses (drained by the server)
        self.demo: dict | None = None       # running pose sequence (demo_start), see _demo_next
        self.track: Tracker | None = None   # object following (track_start), see _track_tick
        self.stats = dict(spi_tx=0, spi_rx_ok=0, spi_rx_bad=0, neutral=0, uart_tx=0, uart_rx=0, uart_timeouts=0,
                          spi_shift=0, smoothed=0)

    # ---------------- inputs ----------------
    def set_intent(self, d: dict, mode5_arm=None) -> str | None:
        p, err = intent_to_payload(d)
        if err:
            return err
        if mode5_arm is not None:
            p = with_mode5_arm(p, mode5_arm)
        mode = p[0]
        now = self.clock()
        ts = d.get("timestamp")
        ts = float(ts) if isinstance(ts, (int, float)) and not isinstance(ts, bool) and math.isfinite(ts) else now
        if mode != self.last_mode or (self.samples and ts <= self.samples[-1][0]):
            self.samples.clear()                    # other mode, or the headset clock went back (app restart)
        arm = struct.unpack_from(">hhh", p, 40) if p[36] == ord("I") else None
        self.samples.append((ts, now, struct.unpack_from(">ffff", p, 16), arm))
        if mode in (3, 4, 5) and mode != self.last_mode:
            self.resend_vr_params()
        self.payload, self.payload_t = p, self.clock()
        self.last_mode, self.last_hb = mode, struct.unpack_from(">H", p, 11)[0]
        return None

    def horizon_line(self) -> str:
        if not self.horizon_on:
            return "HORIZON 0"
        return "HORIZON 1 " + " ".join(str(d) for d in self.horizon_dirs) + f" {self.horizon_trim:.2f}"

    def set_horizon(self, on: bool):
        self.horizon_on = bool(on)
        if not on:
            self.horizon_trim = 0.0
        self.uart_send(self.horizon_line())

    def horizon_trim_step(self, up_tool):
        """Called at ~5 Hz with the vertical measured by the wrist IMU in the tool frame (None = no IMU): while the
        arm and the IMU are still, the image's horizontal axis (tool y) should be level; what is left is fed back
        into the firmware's roll (trim, deg). Base tilt, servo play, offsets: whatever the kinematics miss."""
        now = self.clock()
        # only while the firmware really drives the roll: a VR mode with the grips held and a live intent. In
        # PREPARAZIONE (mode 0) or with the grips released nothing moves, and the trim ran away to its cap
        # (26/10/2026 on the robot: 8 deg, then the camera entered ASSIST tilted by 8 deg for 15 s)
        applied = (self.horizon_on and self.last_mode in (2, 3, 4, 5) and self.diag.get("deadman") is True
                   and now - self.payload_t < 0.5)
        if not applied or up_tool is None:
            self.horizon_hist = []
            return
        pose = tuple(self.telemetry.get(k) for k in SERVO_KEYS)
        h = self.horizon_hist
        h.append((now, pose, tuple(up_tool)))
        while h and now - h[0][0] > 0.8:
            h.pop(0)
        still = (len(h) >= 4 and now - h[0][0] >= 0.6 and all(x[1] == pose for x in h)
                 and max(abs(a - b) for x in h for a, b in zip(x[2], up_tool)) < 0.01)
        if not still:
            return
        err = math.degrees(math.asin(max(-1.0, min(1.0, up_tool[1]))))     # + = image tilted, its left side up
        if abs(err) < 0.5:
            return
        trim = max(-8.0, min(8.0, self.horizon_trim - 0.5 * err))
        if abs(trim - self.horizon_trim) >= 0.2:
            self.horizon_trim = trim
            self.uart_send(self.horizon_line())
            self.horizon_hist = []                 # let the wrist settle before the next correction

    def resend_vr_params(self):
        now = self.clock()
        if self.vr_params_line and now - self.last_vr_params >= self.VR_PARAMS_THROTTLE_S:
            self.last_vr_params = now
            self.uart_send(self.vr_params_line)

    # ---------------- UART ----------------
    def uart_send(self, cmd: str, cb: Callable[[bool, str], None] | None = None) -> int:
        self.uart_seq = self.uart_seq % 99999 + 1
        seq = self.uart_seq
        self.uart.write(f"#{seq} {cmd}\n".encode())
        self.pending[seq] = (cmd, self.clock() + self.UART_TIMEOUT_S, cb)
        self.stats["uart_tx"] += 1
        return seq

    def _uart_poll(self):
        self.rx_buf += self.uart.read()
        while True:
            i = min([k for k in (self.rx_buf.find(b"\n"), self.rx_buf.find(b"\r")) if k >= 0], default=-1)
            if i < 0:
                break
            line, self.rx_buf = self.rx_buf[:i].decode(errors="replace").strip(), self.rx_buf[i + 1:]
            if line:
                self.stats["uart_rx"] += 1
                self._uart_line(line)
        now = self.clock()
        for seq, (cmd, deadline, cb) in list(self.pending.items()):
            if now > deadline:
                del self.pending[seq]
                self.stats["uart_timeouts"] += 1
                self._uart_done(cmd, False, "TIMEOUT", cb)

    def _uart_line(self, line: str):
        if line.startswith("#"):
            head, _, rest = line[1:].partition(" ")
            if head.isdigit() and int(head) in self.pending:
                cmd, _, cb = self.pending.pop(int(head))
                self._uart_done(cmd, not rest.startswith("ERR"), rest, cb)
                return
        if line.startswith("STATUS:"):
            self.robot_state = line[7:].strip()
            return
        if self.track and line.startswith("SETPOSE_DONE"):
            return                                    # end of one 40 ms segment of the follower
        if self.demo and line.startswith("SETPOSE_DONE") and self.demo.get("style") == "smooth":
            d = self.demo
            if d["run"]:
                return                                # the end of one 40 ms segment: not news for the clients
            if d["moving"]:                           # at the start of the curve: stream from now on
                d["run"], d["next"], d["last"] = True, self.clock(), self.clock()
        elif self.demo and line.startswith("SETPOSE_DONE") and self.demo["moving"]:
            self._demo_next()
        elif self.demo and line.startswith("ESTOP") and line != "ESTOP_CLEAR":
            self.demo_stop("fungo")
        self.events.append({"type": "uart_unsolicited", "line": line})
        if line == "BOOT_READY":
            self.log("[j5mini] STM32 BOOT_READY: pushing configuration")
            self._start_boot_push(booted=True)

    def _uart_done(self, cmd, ok, resp, cb):
        if resp.startswith("STATUS:"):
            self.robot_state = resp[7:].strip()
        if self.boot_queue and cmd == self.boot_queue[0]:
            if not ok and self.boot_retries < 2:        # TIMEOUT at a Pi reboot (UART not ready yet), SET_VR_PARAMS hiccups
                self.boot_retries += 1
            else:
                if not ok:
                    self.log(f"[j5mini] config line failed: {cmd} -> {resp}")
                self.boot_queue.pop(0)
                self.boot_retries = 0
            self._boot_next()
        if cb:
            cb(ok, resp)
        if ok and cmd.startswith("SETPOSE_T_HR ") and (self.demo and self.demo.get("run") or self.track):
            return                                    # 25 a second while the smooth demo / the follower runs
        self.events.append({"type": "uart_response", "cmd": cmd, "ok": ok, "response": resp})

    def _start_boot_push(self, booted: bool = False):
        self.boot_queue = list(self.boot_lines)
        if booted:                                    # the firmware just (re)started: tell it where the arm is
            self.boot_queue.append(init_pose_line(self.cfg_for_init, self.last_pose))
        else:                                         # no BOOT_READY seen: maybe it booted before us (_init_pose_check)
            self.init_check_pending = True
        if self.horizon_on:
            self.boot_queue.append(self.horizon_line())
        self.boot_retries = 0
        self.boot_done = True
        self._boot_next()

    def _boot_next(self):
        if self.boot_queue:
            self.uart_send(self.boot_queue[0])

    # ---------------- SPI ----------------
    def _spi_tick(self):
        now = self.clock()
        self.transfers += 1
        self.tx_seq = (self.tx_seq + 1) & 0xFFFF
        if self.transfers % self.STATUS_EVERY == 0:
            frame = build_frame(J5_STATUS, self.tx_seq, b"", self.len)
        else:
            if self.payload is not None and now - self.payload_t <= self.link_timeout:
                p = self.payload
                if self.smooth_s > 0 and p[0] in (3, 4, 5) and len(self.samples) >= 2:
                    p = self._smoothed(p, now)
            else:
                p = neutral_payload(self.last_mode, self.last_hb)
                self.stats["neutral"] += 1
            frame = build_frame(J5_J5VR, self.tx_seq, p, self.len)
        rx = self.spi.transfer(frame)
        self.stats["spi_tx"] += 1
        found = find_frame_window(self.prev_rx, rx)
        self.prev_rx = rx
        if found is None:
            self.stats["spi_rx_bad"] += 1
            return
        f, off = found
        key = (f[3], f[4], f[5])                   # the same frame seen again in the next window
        if key == self.last_frame_key:
            self.stats["spi_rx_bad"] += 1
            return
        self.last_frame_key = key
        self.stats["spi_rx_ok"] += 1
        self.stats["spi_shift"] = off % len(rx)    # 0 = aligned; else the slave's DMA phase in bytes
        pl = f[8:62]
        if f[3] == J5_TELEMETRY:
            self.telemetry = {
                "imu_valid": bool(pl[28]),
                "imu_q_w": struct.unpack_from(">f", pl, 29)[0], "imu_q_x": struct.unpack_from(">f", pl, 33)[0],
                "imu_q_y": struct.unpack_from(">f", pl, 37)[0], "imu_q_z": struct.unpack_from(">f", pl, 41)[0],
                **{k: pl[45 + i] for i, k in enumerate(SERVO_KEYS)},
                "rt_loop_period_us": struct.unpack_from(">H", f, 62)[0],
            }
        elif f[3] == J5_STATUS and pl[0] == 0:
            diag = struct.unpack_from(">H", pl, 50)[0]
            self.diag = {"heartbeat_echo": struct.unpack_from(">H", pl, 46)[0], "mode_echo": pl[48], "diag": diag,
                         **{name: bool(diag & (1 << b)) for b, name in enumerate(DIAG_BITS)}}

    def _smoothed(self, p: bytes, now: float) -> bytes:
        """The newest payload with the head quaternion (and the ASSIST arm targets) as they were smooth_s ago on
        the headset's clock. Headset -> local offset = the smallest one seen in the window (least delayed
        packet)."""
        s = self.samples
        off = min(tl - th for th, tl, _, _ in s)
        r = now - off - self.smooth_s
        if r <= s[0][0]:
            a = b = s[0]
        elif r >= s[-1][0]:
            a = b = s[-1]
        else:
            i = next(k for k in range(len(s) - 1) if s[k + 1][0] > r)
            a, b = s[i], s[i + 1]
        u = (r - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
        out = bytearray(p)
        struct.pack_into(">ffff", out, 16, *slerp(a[2], b[2], u))
        if p[36] == ord("I") and a[3] is not None and b[3] is not None:
            struct.pack_into(">hhh", out, 40, *(int(round(x + (y - x) * u)) for x, y in zip(a[3], b[3])))
        self.stats["smoothed"] += 1
        return bytes(out)

    # ---------------- step ----------------
    def step(self):
        """Call every 10 ms."""
        self._spi_tick()
        self._uart_poll()
        now = self.clock()
        self._init_pose_check()
        self._demo_tick(now)
        self._track_tick(now)
        self._auto_park_tick(now)
        if not self.boot_done and now - self.t0 >= self.BOOT_FALLBACK_S:
            self.log("[j5mini] no BOOT_READY seen: pushing configuration anyway")
            self._start_boot_push()
        if now >= self.next_state_poll:
            self.next_state_poll = now + self.STATE_POLL_S
            self.uart_send("STATUS?")

    def _init_pose_check(self):
        """At a cold power-on the STM32 boots in a second and the Pi in a minute: BOOT_READY is printed to nobody and
        the fallback push has no SETPOSE_INIT, so the firmware believed the arm at 90 on every joint and the first pose
        snapped from PARK (27/10/2026). A firmware reporting exactly 90 everywhere has never been told where the arm
        is: it gets the init pose (last_pose, or PARK) after the configuration lines."""
        if not self.init_check_pending:
            return
        pose = [self.telemetry.get(k) for k in SERVO_KEYS]
        if not all(isinstance(v, int) for v in pose):
            return                                    # no telemetry yet
        self.init_check_pending = False
        if pose == [90] * 6:
            self.boot_queue.append(init_pose_line(self.cfg_for_init, self.last_pose))
            if len(self.boot_queue) == 1:
                self._boot_next()
            self.log("[j5mini] firmware never positioned (all servos at 90): sending the init pose")

    # ---------------- demo: the saved pose sequence, looped (dashboard "Demo") ----------------
    DEMO_STEP_TIMEOUT_S = 15.0

    def demo_start(self, lines: list[str]) -> tuple[bool, str]:
        """SETPOSE lines one at a time, the next one at SETPOSE_DONE, round and round until demo_stop(), the robot
        leaves IDLE (STOP / SAFE / E-STOP), a refused line or a step without SETPOSE_DONE for 15 s."""
        if not lines:
            return False, "nessun passo"
        if self.robot_state != "IDLE":
            return False, f"robot in {self.robot_state}: serve IDLE"
        if self.track:
            return False, "sta seguendo un oggetto: fermalo prima"
        self.demo = {"lines": list(lines), "i": -1, "deadline": 0.0, "moving": False}
        self._demo_next()
        return True, ""

    def demo_start_smooth(self, path: "DemoPath") -> tuple[bool, str]:
        """The continuous demo: a SETPOSE to the start of the curve, then from its SETPOSE_DONE one SETPOSE_T_HR
        every DEMO_STREAM_S along it (each lasting DEMO_STREAM_S, so the next one starts where it ends). The
        playback speed ramps up from 0 at the start and back to 0 when the dashboard stops it."""
        if self.robot_state != "IDLE":
            return False, f"robot in {self.robot_state}: serve IDLE"
        if self.track:
            return False, "sta seguendo un oggetto: fermalo prima"
        self.demo = {"style": "smooth", "path": path, "lines": [None] * path.n, "i": 0, "moving": False,
                     "deadline": 0.0, "run": False, "tau": 0.0, "ramp": 0.0, "stopping": None, "next": 0.0,
                     "last": 0.0}
        d = self.demo

        def sent(ok, resp):
            if self.demo is not d:
                return
            if not ok:
                self.demo_stop(f"partenza rifiutata: {resp}")
                return
            d["moving"] = True
            d["deadline"] = self.clock() + self.DEMO_STEP_TIMEOUT_S
        self.uart_send(path.start_line([self.telemetry.get(k) for k in SERVO_KEYS]), sent)
        self.events.append({"type": "demo", "on": True, "step": 1, "n": path.n})
        return True, ""

    DEMO_STREAM_S = 0.04      # 25 segments a second: 0.1 deg of ripple from each segment's own S-curve at 30 deg/s
    DEMO_RAMP_S = 1.5         # playback speed 0 -> 1 at the start, 1 -> 0 at a dashboard stop

    def _demo_stream(self, now: float):
        d = self.demo
        if now < d["next"]:
            return
        dt = min(0.2, now - d["last"])
        d["last"] = now
        d["next"] = max(d["next"] + self.DEMO_STREAM_S, now + self.DEMO_STREAM_S * 0.5)
        d["ramp"] += (-dt if d["stopping"] else dt) / self.DEMO_RAMP_S
        if d["stopping"] and d["ramp"] <= 0.0:
            self._demo_end(d["stopping"])                # already still: nothing to hold
            return
        d["ramp"] = max(0.0, min(1.0, d["ramp"]))
        rate = d["ramp"] * d["ramp"] * (3.0 - 2.0 * d["ramp"])
        d["tau"] += rate * self.DEMO_STREAM_S
        seg = d["path"].segment(d["tau"])
        if seg != d["i"]:
            d["i"] = seg
            self.events.append({"type": "demo", "on": True, "step": seg + 1, "n": d["path"].n})

        def sent(ok, resp):
            if self.demo is d and not ok:
                self.demo_stop(f"traiettoria rifiutata: {resp}")
        self.uart_send(d["path"].stream_line(d["tau"], int(self.DEMO_STREAM_S * 1000)), sent)

    def demo_stop(self, reason: str, hold: bool = False):
        """hold: stop where the arm is (a 120 ms trajectory to the measured angles replaces the running one). The
        smooth demo slows down to a stop instead (DEMO_RAMP_S) and ends there."""
        d = self.demo
        if not d:
            return
        if hold and d.get("run"):
            d["stopping"] = d["stopping"] or reason
            return
        self._demo_end(reason, hold)

    def _demo_end(self, reason: str, hold: bool = False):
        d = self.demo
        self.demo = None
        if hold and d["moving"] and all(k in self.telemetry for k in SERVO_KEYS):
            self.uart_send("SETPOSE_T_HR " + " ".join(str(int(round(self.telemetry[k] * 10))) for k in SERVO_KEYS)
                           + " 120 RTR3")
        self.events.append({"type": "demo", "on": False, "reason": reason})
        self.log(f"[j5mini] demo stopped ({reason})")

    def demo_status(self) -> dict | None:
        d = self.demo
        if not d:
            return None
        return {"step": d["i"] + 1, "n": len(d["lines"]), "style": d.get("style", "steps")}

    def _demo_next(self):
        d = self.demo
        d["i"] = (d["i"] + 1) % len(d["lines"])
        d["moving"] = False
        line = d["lines"][d["i"]]

        def sent(ok, resp):                  # the firmware's reply to this step's SETPOSE
            if self.demo is not d:
                return
            if not ok:
                self.demo_stop(f"passo {d['i'] + 1} rifiutato: {resp}")
                return
            d["moving"] = True
            d["deadline"] = self.clock() + self.DEMO_STEP_TIMEOUT_S
        self.uart_send(line, sent)
        self.events.append({"type": "demo", "on": True, "step": d["i"] + 1, "n": len(d["lines"])})

    # ---------------- object following (dashboard "Segui", stereo head boxes) ----------------
    def track_start(self, box: dict, t_capture: float, kin: dict | None = None) -> tuple[bool, str]:
        if self.robot_state != "IDLE":
            return False, f"robot in {self.robot_state}: serve IDLE"
        if self.demo:
            return False, "demo in corso: fermala prima"
        pose = [self.telemetry.get(k) for k in SERVO_KEYS]
        if not all(isinstance(v, (int, float)) for v in pose):
            return False, "posa del braccio sconosciuta (nessuna telemetria)"
        s = settings_from_config(self.cfg_for_init)
        virt = [90.0 + (float(ph) - s["offsets"][j]) / (-1 if s["dirs"][j] < 0 else 1) for j, ph in enumerate(pose)]
        self.track = Tracker(self.cfg_for_init, virt, box, t_capture, self.clock(), kin=kin)
        self.events.append({"type": "track", "on": True, "cls": self.track.cls})
        self.log(f"[j5mini] follow class {self.track.cls} from box {box}")
        return True, ""

    def track_boxes(self, boxes: list, t_capture: float):
        if self.track:
            self.track.on_boxes(boxes, t_capture, self.clock())

    def track_stop(self, reason: str, hold: bool = False):
        t = self.track
        if not t:
            return
        self.track = None
        if hold:
            self.uart_send("SETPOSE_T_HR " + " ".join(str(int(round(v * 10))) for v in t.physical()) + " 120 RTR3")
        self.events.append({"type": "track", "on": False, "reason": reason})
        self.log(f"[j5mini] follow stopped ({reason}), {t.n_meas} detections used")

    def _track_tick(self, now: float):
        t = self.track
        if not t:
            return
        if self.robot_state != "IDLE":
            self.track_stop(f"robot in {self.robot_state}")
            return
        if self.diag.get("deadman") is True:
            self.track_stop("grilletti del visore premuti")
            return
        if t.lost_for(now) > t.p["give_up_s"]:
            self.track_stop(f"oggetto perso da {t.p['give_up_s']:.0f} s")
            return
        line = t.tick(now)
        if line:
            def sent(ok, resp):
                if self.track is t and not ok:
                    self.track_stop(f"traiettoria rifiutata: {resp}")
            self.uart_send(line, sent)

    # ---------------- auto-park: the arm parks itself when the operator leaves it idle ----------------
    def auto_park_in(self, now: float | None = None) -> int | None:
        """Seconds left before the automatic PARK, or None when the countdown is not running."""
        if self.idle_since is None or self.auto_park_done or self.auto_park_s <= 0:
            return None
        return max(0, int(round(self.auto_park_s - ((now if now is not None else self.clock()) - self.idle_since))))

    def _auto_park_tick(self, now: float):
        """Headset in a teleop mode (2-5) with the grips released for auto_park_s (user, 27/10/2026: "so when I switch
        it off it is already in position"): PARK, once per idle period. Needs IDLE, no demo, no trajectory in flight
        and the arm not already there; pressing the grips or leaving the modes resets everything."""
        live = self.payload is not None and (now - self.payload_t) < 1.0
        in_mode = live and self.last_mode in (2, 3, 4, 5)
        grips = self.diag.get("deadman") is True
        if not in_mode or grips:
            self.idle_since = None
            self.auto_park_done = False
            return
        if self.idle_since is None:
            self.idle_since = now
            return
        if self.auto_park_done or self.auto_park_s <= 0 or now - self.idle_since < self.auto_park_s:
            return
        if self.robot_state != "IDLE" or self.demo or self.track:
            return                                    # try again next tick, the countdown stays at zero
        park = pose_physical(self.cfg_for_init, self.cfg_for_init.get("settings", {}).get("park", POSE_DEFAULTS["park"]))
        pose = [self.telemetry.get(k) for k in SERVO_KEYS]
        self.auto_park_done = True
        if all(isinstance(v, int) for v in pose) and max(abs(a - b) for a, b in zip(pose, park)) <= 2:
            return                                    # already parked
        line = pose_setpose_line(self.cfg_for_init, "park")
        self.uart_send(line)
        self.events.append({"type": "auto_park", "after_s": int(self.auto_park_s), "line": line})
        self.log(f"[j5mini] auto-park after {self.auto_park_s:.0f} s without the grips: {line}")

    def _demo_tick(self, now: float):
        d = self.demo
        if not d:
            return
        if self.robot_state != "IDLE":
            self.demo_stop(f"robot in {self.robot_state}")
        elif d.get("run"):
            self._demo_stream(now)
        elif d["moving"] and now > d["deadline"]:
            self.demo_stop(f"passo {d['i'] + 1} senza SETPOSE_DONE entro {self.DEMO_STEP_TIMEOUT_S:.0f} s")

    def telemetry_message(self) -> dict:
        pose = [self.telemetry.get(k) for k in SERVO_KEYS]
        if all(isinstance(v, int) for v in pose) and pose != [90] * 6:    # [90]*6 = a firmware just reset, not a pose
            self.last_pose = pose
        intent_age = self.clock() - self.payload_t
        live = intent_age < 1.0 and self.payload is not None
        return {"type": "telemetry", "robot_state": self.robot_state, **self.telemetry,
                "deadman_active": self.diag.get("deadman"), "vr_armed": self.diag.get("armed"),
                "estop_active": self.diag.get("estop_active"), "estop_bypass": self.diag.get("estop_bypass"),
                "input_active": self.diag.get("input_active"), "vr_freeze": self.diag.get("freeze"),
                "vr_guard": self.diag.get("guard"),
                "teleop_mode": self.last_mode if live else -1,
                "intent_age_ms": int(intent_age * 1000) if self.payload is not None else None,
                # the headset's orientation as the app sends it (dashboard "Visore" card), the heartbeat and the
                # mode echoed by the firmware in its STATUS reply
                "vr_q": [round(v, 4) for v in self.samples[-1][2]] if live and self.samples else None,
                "vr_heartbeat": self.last_hb if live else None, "mode_echo": self.diag.get("mode_echo"),
                "demo": self.demo_status(), "auto_park_in": self.auto_park_in(), "link": dict(self.stats),
                "track": self.track.status(self.clock()) if self.track else None}


# ------------------------------------------------------------------ settings (dashboard)

JOINTS = ("B", "S", "G", "Y", "P", "R")


POSE_DEFAULTS = {            # the robot's j5_settings.json (virtual degrees, 90 = HOME)
    "park": [90, 50, 150, 90, 130, 90],
    "vr": [90, 110, 90, 90, 70, 90],
    "auto_park_s": 60,           # headset in a teleop mode, grips released this long -> PARK by itself (0 = never)
    "speed_pct": 40,             # SETPOSE speed, as the firmware's HOME (ws-teleop used vel_max 120 = 100 %: too
                                 # fast on the robot, 26/10/2026)
    "profile": "RTR5",
}
DEMO_DEFAULT = [{"angles": [90, 90, 90, 90, 90, 90], "speed_pct": 10, "profile": "RTR5"},
                {"angles": [90, 100, 95, 90, 75, 90], "speed_pct": 10, "profile": "RTR5"},
                {"angles": [70, 95, 100, 85, 80, 90], "speed_pct": 10, "profile": "RTR5"},
                {"angles": [62, 102, 118, 80, 95, 90], "speed_pct": 10, "profile": "RTR5"},
                {"angles": [90, 105, 120, 90, 95, 90], "speed_pct": 10, "profile": "RTR5"},
                {"angles": [118, 102, 118, 100, 95, 90], "speed_pct": 10, "profile": "RTR5"},
                {"angles": [110, 95, 100, 95, 80, 90], "speed_pct": 10, "profile": "RTR5"},
                {"angles": [90, 115, 150, 90, 85, 90], "speed_pct": 8, "profile": "RTR5"},
                {"angles": [90, 95, 100, 90, 70, 90], "speed_pct": 10, "profile": "RTR5"}]   # 27/10/2026: forward only, slow (camera always in front of the base, POE-checked)



def pose_physical(cfg: dict, virtual) -> list[int]:
    """A virtual pose -> physical servo degrees (offsets, dirs, [5, 175], joint limits)."""
    s = settings_from_config(cfg)
    phys = []
    for i in range(6):
        d = -1 if s["dirs"][i] < 0 else 1
        v = max(5, min(175, int(s["offsets"][i] + d * (float(virtual[i]) - 90))))
        lo, hi = s["limits"][i]
        phys.append(max(lo, min(hi, v)))
    return phys


def setpose_line(cfg: dict, virtual, speed_pct, profile) -> str:
    """A virtual pose -> "SETPOSE b s g y p r vel% profile"."""
    pct = max(1, min(100, int(speed_pct)))
    return "SETPOSE " + " ".join(str(v) for v in pose_physical(cfg, virtual)) + f" {pct} {profile}"


def init_pose_line(cfg: dict, physical=None) -> str:
    """SETPOSE_INIT: where the arm is when the firmware boots, so its first trajectory starts there instead of
    snapping the servos to HOME (27/10/2026). `physical` = the last pose seen before a reset; without one (cold
    start) the PARK pose, where the arm rests when it is switched off. Sent after the boot lines, at BOOT_READY
    only: a j5mini restart with the firmware running must not change what the firmware knows."""
    if physical is None:
        st = cfg.get("settings", {})
        physical = pose_physical(cfg, st.get("park", POSE_DEFAULTS["park"]))
    return "SETPOSE_INIT " + " ".join(str(int(v)) for v in physical)


def pose_setpose_line(cfg: dict, name: str) -> str:
    """ws-teleop's PARK / TELEOPPOSE: the saved virtual pose at the saved speed and profile."""
    st = cfg.get("settings", {})
    return setpose_line(cfg, st.get(name, POSE_DEFAULTS[name]), st.get("speed_pct", POSE_DEFAULTS["speed_pct"]),
                        st.get("profile", POSE_DEFAULTS["profile"]))


def demo_lines(cfg: dict) -> list[str]:
    """The demo steps of the settings as SETPOSE lines."""
    return [setpose_line(cfg, x["angles"], x.get("speed_pct", POSE_DEFAULTS["speed_pct"]),
                         x.get("profile", POSE_DEFAULTS["profile"]))
            for x in cfg.get("settings", {}).get("demo", DEMO_DEFAULT)]


DEMO_ROLL_DEFAULT = 20.0      # deg of roll oscillation in the smooth demo (cut to the roll joint's room, 2 deg margin)
DEMO_ROLL_PERIOD_S = 9.0


class DemoPath:
    """The smooth demo (20/11/2026): a closed curve through the demo poses (virtual degrees). Each stretch takes as
    long as the step's SETPOSE would (largest joint change / the firmware's speed for its speed_pct), so the pace
    is the old one without the stops. PCHIP per joint: never beyond the poses' own range (a joint that turns back
    at a pose slows to zero there, the others keep going). Roll: + roll_deg * sin(2 pi t / 9 s) on top."""

    MIN_SEG_S = 0.8

    def __init__(self, cfg: dict):
        st = cfg.get("settings", {})
        s = settings_from_config(cfg)
        self.offsets, self.limits = s["offsets"], s["limits"]
        self.dirs = [-1 if d < 0 else 1 for d in s["dirs"]]
        steps = st.get("demo", DEMO_DEFAULT)
        pts, speeds = [], []
        for x in steps:
            p = [float(v) for v in x["angles"]]
            if not pts or p != pts[-1]:
                pts.append(p)
                speeds.append(int(x.get("speed_pct", POSE_DEFAULTS["speed_pct"])))
        if len(pts) > 1 and pts[0] == pts[-1]:
            pts.pop(), speeds.pop()
        if len(pts) < 2:                              # one pose: hold it, roll only
            pts = [pts[0], list(pts[0])] if pts else [[90.0] * 6, [90.0] * 6]
            speeds = (speeds * 2)[:2] or [POSE_DEFAULTS["speed_pct"]] * 2
        self.pts, self.n = pts, len(pts)
        self.t = [0.0]
        for k in range(self.n):
            a, b = pts[k], pts[(k + 1) % self.n]
            v = 1.0 + max(1, min(100, speeds[(k + 1) % self.n])) / 100.0 * 119.0   # firmware SETPOSE deg/s
            self.t.append(self.t[-1] + max(self.MIN_SEG_S, max(abs(x - y) for x, y in zip(a, b)) / v))
        self.period = self.t[-1]
        self.m = [self._slopes(j) for j in range(6)]
        roll = float(st.get("demo_roll_deg", DEMO_ROLL_DEFAULT))
        lo, hi = self.limits[5]
        rmin, rmax = min(p[5] for p in pts), max(p[5] for p in pts)
        room = min(self._virtual(5, hi) - rmax, rmin - self._virtual(5, lo)) if self.dirs[5] > 0 else \
            min(self._virtual(5, lo) - rmax, rmin - self._virtual(5, hi))
        self.roll = max(0.0, min(roll, room - 2.0))

    def _virtual(self, j: int, phys: float) -> float:
        return 90.0 + (phys - self.offsets[j]) / self.dirs[j]

    def _slopes(self, j: int) -> list[float]:
        """PCHIP (Fritsch-Butland) tangents of joint j on the closed loop, deg/s."""
        n, h, d = self.n, [], []
        for k in range(n):
            h.append(self.t[k + 1] - self.t[k])
            d.append((self.pts[(k + 1) % n][j] - self.pts[k][j]) / h[k])
        m = []
        for k in range(n):
            d0, d1, h0, h1 = d[k - 1], d[k], h[k - 1], h[k]
            if d0 * d1 <= 0:
                m.append(0.0)
            else:
                w1, w2 = 2 * h1 + h0, h1 + 2 * h0
                m.append((w1 + w2) / (w1 / d0 + w2 / d1))
        return m

    def segment(self, tau: float) -> int:
        u = tau % self.period
        return next(k for k in range(self.n) if u < self.t[k + 1] or k == self.n - 1)

    def virtual(self, tau: float) -> list[float]:
        k = self.segment(tau)
        h = self.t[k + 1] - self.t[k]
        x = min(1.0, max(0.0, (tau % self.period - self.t[k]) / h))
        x2, x3 = x * x, x * x * x
        h00, h10, h01, h11 = 2 * x3 - 3 * x2 + 1, x3 - 2 * x2 + x, -2 * x3 + 3 * x2, x3 - x2
        a, b = self.pts[k], self.pts[(k + 1) % self.n]
        out = [h00 * a[j] + h10 * h * self.m[j][k] + h01 * b[j] + h11 * h * self.m[j][(k + 1) % self.n]
               for j in range(6)]
        out[5] += self.roll * math.sin(2 * math.pi * tau / DEMO_ROLL_PERIOD_S)
        return out

    def physical(self, tau: float) -> list[float]:
        out = []
        for j, v in enumerate(self.virtual(tau)):
            p = max(5.0, min(175.0, self.offsets[j] + self.dirs[j] * (v - 90.0)))
            out.append(max(self.limits[j][0], min(self.limits[j][1], p)))
        return out

    def start_line(self, current=None) -> str:
        """The way to the start of the curve, at the speed of a 20 % SETPOSE (24.8 deg/s) from `current` (servo
        degrees, telemetry), in tenths like the stream so the first segment does not twitch by a rounding."""
        target = self.physical(0.0)
        ms = 3000
        if current and len(current) == 6 and all(isinstance(v, (int, float)) for v in current):
            ms = max(300, int(max(abs(a - b) for a, b in zip(target, current)) / 24.8 * 1000))
        return "SETPOSE_T_HR " + " ".join(str(int(round(v * 10))) for v in target) + f" {ms} RTR5"

    def stream_line(self, tau: float, ms: int) -> str:
        return "SETPOSE_T_HR " + " ".join(str(int(round(v * 10))) for v in self.physical(tau)) + f" {ms} RTR3"


# ------------------------------------------------------------------ object following (dashboard "Segui")

TRACK_DEFAULTS = {
    "joints": [3, 4],              # pan = wrist yaw (Y), tilt = wrist pitch (P)
    "px_per_deg": [17.1, -17.1],   # how far (left-eye px) a still object moves in the image for +1 virtual deg of
                                   # each joint, sign included. Measured on the robot 21/11/2026 (park pose, camera
                                   # looking down): yaw +13.4 px/deg on x, pitch -12 px/deg on y, i.e. ~17 level
                                   # (f_px 981); the pitch servo has ~4 deg of backlash (5 deg steps: 12 px)
    "lag_s": 0.20,                 # servos behind the firmware's trajectory (IMU campaign 18/10/2026: 200-250 ms)
    "max_deg_s": 70.0,             # command speed limit per joint (35, then 50 on 22/11/2026: a person close to
                                   # the camera stepping sideways went up to ~80 deg/s and the arm fell behind)
    "max_acc_deg_s2": 300.0,
    "tau_s": 0.15,                 # time constant of the approach to the target direction
    "deadband_deg": 1.5,           # a still object within this of the command: no motion (no hunting)
    "alpha": 0.6, "beta": 0.4,     # alpha-beta filter of the object's direction (for gain_dt_s; alpha 0.5 until the
                                   # time-based gains of 22/11/2026, tuned at 6.7-15 Hz; first tuned 21/11/2026
                                   # with noisy, frame-cut boxes in test_j5mini TrackWorld: still object 44 -> ~6
                                   # direction reversals in 16 s, person bigger than the frame 14 -> 8 deg RMS)
    "max_dv_deg_s": 12.0,          # one detection changes the speed estimate by at most this (a box that jumps
                                   # or changes shape is not a sudden speed: the robot hunted with +-55 deg/s)
    "gain_dt_s": 0.15,             # alpha, beta, max_dv and the decays above are for detections this far apart;
                                   # other intervals rescale them (alpha compounds, beta x (dt/gain_dt)^beta_exp,
                                   # max_dv and the decay per second), so 15 Hz does not take in twice the noise
    "beta_exp": 1.5,
    "v_dead_deg_s": 5.0,           # speed below this is noise: the prediction uses |v| - v_dead
    "edge_px": 24,                 # a box side this close to the frame's edge is cut: the box centre is not the
                                   # object's centre on that axis (21/11/2026: person close to the camera). YOLO's
                                   # cut boxes stop short of the far edges: right side 1899-1908, bottom 1062-1066
    "size_memory_s": 3.0,          # how long the last uncut size on an axis is used to rebuild a cut box
    "max_predict_s": 0.6,
    "max_predict_deg": 10.0,       # the prediction never puts the target more than this ahead of the estimate:
                                   # at a reversal it overshot by 14 deg (46 deg/s x 0.3 s) and the arm swung
    "near_box_px": [700, 1300],    # a box this wide (person close: quick, jerky directions) gets less prediction,
    "near_lead_frac": 0.4,         # down to this fraction of the lead at the upper width
    "gate_px": 260,                # a box farther than this (+ the box size) from where the target should be: not it
    "base_joint": 0,               # the base turns too when the wrist yaw runs out of room (None: wrist only)
    "base_gain": 1.0,              # pan per base degree, in wrist yaw degrees, when there is no kinematics
                                   # (with kinematics.json it comes from the pose: 1 with shoulder + elbow at 90)
    "base_dead_deg": 15.0,         # the base stays put while the wrist yaw is within this of its centre
    "base_tau_s": 1.0,             # ... beyond it the base brings the wrist back towards the centre
    "base_max_deg_s": 30.0,        # the base carries the whole arm: slower than the wrist (20 until 22/11/2026)
    "base_max_acc_deg_s2": 60.0,
    "hold_after_s": 0.8,           # target not seen: the prediction stops, the arm holds
    "give_up_s": 8.0,              # ... and the follower ends
}


def _rodrigues(w, a: float) -> list[list[float]]:
    """Rotation by `a` rad about the unit axis w (3x3, rows)."""
    x, y, z = w
    c, s_, t = math.cos(a), math.sin(a), 1.0 - math.cos(a)
    return [[t * x * x + c, t * x * y - s_ * z, t * x * z + s_ * y],
            [t * x * y + s_ * z, t * y * y + c, t * y * z - s_ * x],
            [t * x * z - s_ * y, t * y * z + s_ * x, t * z * z + c]]


def _mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def camera_axes_coefficients(kin: dict, virtual: list) -> tuple[list[float], list[float]] | None:
    """How much each joint turns the camera about its own up axis (the image moves sideways) and its own side
    axis (the image moves up/down), per degree, at this pose (virtual degrees): the joint's POE axis seen from the
    camera frame. 1 for wrist yaw and base at HOME; the wrist yaw drops with the cosine of the camera's pitch
    (park pose: 0.77, measured 13.4 / 17.1 px per degree on 21/11/2026). None when kin is unusable."""
    try:
        axes = [[float(v) for v in row[:3]] for row in kin["S"]][:6]
        m = [[float(v) for v in row[:3]] for row in kin["M"][:3]]
        if len(axes) < 6 or len(m) < 3:
            return None
    except (KeyError, TypeError, ValueError, IndexError):
        return None
    r = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    spatial = []
    for i, w in enumerate(axes):
        spatial.append([sum(r[a][k] * w[k] for k in range(3)) for a in range(3)])
        r = _mat_mul(r, _rodrigues(w, math.radians(float(virtual[i]) - 90.0)))
    rc = _mat_mul(r, m)                                 # camera (end-effector) frame: x optical axis, z up
    up = [sum(rc[k][2] * w[k] for k in range(3)) for w in spatial]
    side = [sum(rc[k][1] * w[k] for k in range(3)) for w in spatial]
    return up, side


class Tracker:
    """Follow an object with the camera (21/11/2026, dashboard "Segui"). The YOLO boxes of the stereo head (left
    eye, ~5 Hz, kCtrlVideoMeta via the meta mirror) steer two joints, by default wrist yaw (pan) and wrist pitch
    (tilt), with one SETPOSE_T_HR every 40 ms like the smooth demo.

    A box is 100-200 ms old when it gets here and the servos are another ~200 ms behind the commands: a plain
    "move by the offset from the centre" loop over-corrects and swings. So each box becomes the object's ABSOLUTE
    direction in joint space: the joint where the camera was when the frame was taken (this follower's own command
    history, lag_s earlier) + the box's offset from the centre / px_per_deg. An alpha-beta filter estimates that
    direction and its speed; the command goes towards where the object will be when the servos get there (lag_s
    ahead), with speed and acceleration limits and a dead band for a still object. The other joints stay put.

    No box matching the target for hold_after_s: no more prediction (the arm stops on the last estimate); after
    give_up_s the follower ends (Core.track_stop). Pure logic: Core feeds boxes and time."""

    STREAM_S = 0.04
    W, H = 1920, 1080

    def __init__(self, cfg: dict, start_virtual, box: dict, t_capture: float, now: float, kin: dict | None = None):
        self.p = p = {**TRACK_DEFAULTS, **cfg.get("settings", {}).get("track", {})}
        s = settings_from_config(cfg)
        self.offsets, self.dirs = s["offsets"], [-1 if d < 0 else 1 for d in s["dirs"]]
        self.plimits = s["limits"]
        self.j = [int(x) for x in p["joints"]][:2]
        self.ppd = [float(v) if abs(float(v)) > 1e-3 else 17.1 for v in p["px_per_deg"]][:2]
        self.cmd = [float(v) for v in start_virtual]
        self.vlim = []
        for j in self.j:                               # the joint's room in virtual degrees
            lo, hi = (max(5, self.plimits[j][0]), min(175, self.plimits[j][1]))
            a, b = self._virtual(j, lo), self._virtual(j, hi)
            self.vlim.append((min(a, b), max(a, b)))
        bj = p.get("base_joint")
        self.bj = int(bj) if isinstance(bj, (int, float)) and int(bj) not in self.j and 0 <= int(bj) < 6 else None
        if self.bj is not None:
            lo, hi = (max(5, self.plimits[self.bj][0]), min(175, self.plimits[self.bj][1]))
            a, b = self._virtual(self.bj, lo), self._virtual(self.bj, hi)
            self.blim = (min(a, b), max(a, b))
        self.bvel, self.bmoving = 0.0, False
        self.kin = kin if kin and camera_axes_coefficients(kin, self.cmd) else None
        self._update_coefficients()
        self.vel = [0.0, 0.0]
        self.hist = collections.deque(maxlen=250)      # (t, [yaw, pitch, base]) commanded, 10 s at 25 Hz
        self.hist.append((now - 10.0, self._raw()))
        self.hist.append((now, self._raw()))
        self.cls = int(box["cls"])
        self.x: list[float] | None = None              # filtered object direction (virtual deg of the two joints)
        self.v = [0.0, 0.0]
        self.t_meas = t_capture
        self.seen = now
        self.box = dict(box)
        self.n_meas = 0
        self.moving = [False, False]
        self.next = now
        self.size = [None, None]                       # last uncut (extent, t) per axis, to rebuild cut boxes
        self.trust = [1.0, 1.0]                        # how the last box was used per axis (status/debug)
        self.measure(box, t_capture, now)

    def _virtual(self, j: int, phys: float) -> float:
        return 90.0 + (phys - self.offsets[j]) / self.dirs[j]

    def _raw(self) -> list[float]:
        return [self.cmd[self.j[0]], self.cmd[self.j[1]], self.cmd[self.bj] if self.bj is not None else 90.0]

    def _update_coefficients(self):
        """Pixel scale per axis (px_per_deg x how much the joint turns the camera at this pose) and the base's
        worth in wrist yaw degrees; from kinematics.json, or the plain px_per_deg / base_gain without it."""
        c = camera_axes_coefficients(self.kin, self.cmd) if self.kin else None
        if c:
            up, side = c
            yaw = up[self.j[0]] if abs(up[self.j[0]]) >= 0.3 else math.copysign(0.3, up[self.j[0]] or 1.0)
            tilt = side[self.j[1]] if abs(side[self.j[1]]) >= 0.3 else math.copysign(0.3, side[self.j[1]] or 1.0)
            self.scale = [yaw, tilt]
            self.kb = up[self.bj] / yaw if self.bj is not None else 0.0
        else:
            self.scale = [1.0, 1.0]
            self.kb = float(self.p["base_gain"]) if self.bj is not None else 0.0
        if self.bj is not None:
            ends = [w + self.kb * (b - 90.0) for w in self.vlim[0] for b in self.blim]
            self.plim = (min(ends), max(ends))
        else:
            self.plim = self.vlim[0]

    def _ppd(self, i: int) -> float:
        return self.ppd[i] * self.scale[i]

    def _axes(self, raw: list[float]) -> list[float]:
        """[pan, tilt] of a [yaw, pitch, base] pose: pan in wrist yaw degrees, the base counted at its worth."""
        return [raw[0] + self.kb * (raw[2] - 90.0), raw[1]]

    def physical(self) -> list[float]:
        out = []
        for j, v in enumerate(self.cmd):
            ph = max(5.0, min(175.0, self.offsets[j] + self.dirs[j] * (v - 90.0)))
            out.append(max(self.plimits[j][0], min(self.plimits[j][1], ph)))
        return out

    def pose_at(self, t: float) -> list[float]:
        """The camera's two joints at time t: the commands of lag_s before (the servos follow that far behind)."""
        t -= self.p["lag_s"]
        h = self.hist
        if t <= h[0][0]:
            return self._axes(h[0][1])
        prev = h[0]
        for e in h:
            if e[0] >= t:
                u = (t - prev[0]) / (e[0] - prev[0]) if e[0] > prev[0] else 1.0
                return self._axes([a + (b - a) * u for a, b in zip(prev[1], e[1])])
            prev = e
        return self._axes(h[-1][1])

    def _centre(self, b: dict) -> tuple[float, float]:
        return b["x"] + b["w"] / 2.0, b["y"] + b["h"] / 2.0

    def _axis(self, b: dict, i: int, t: float) -> tuple[float | None, float]:
        """The object's centre along axis i (0 = x, 1 = y) and how far to trust it:
        1 the box is whole on that axis; 0.5 one side is cut by the frame and the centre is rebuilt from the
        other side and the last uncut size; 0.25 one side cut and no size known (the box centre lies towards the
        inside of the frame: only a nudge); 0 both sides cut (the object is bigger than the frame: no measure)."""
        lo, ext = (b["x"], b["w"]) if i == 0 else (b["y"], b["h"])
        n, e = (self.W if i == 0 else self.H), self.p["edge_px"]
        cut_lo, cut_hi = lo <= e, lo + ext >= n - e
        if not cut_lo and not cut_hi:
            self.size[i] = (ext, t)
            return lo + ext / 2.0, 1.0
        if cut_lo and cut_hi:
            return None, 0.0
        known = self.size[i]
        if known and t - known[1] <= self.p["size_memory_s"] and known[0] > ext:
            return ((lo + ext - known[0] / 2.0) if cut_lo else (lo + known[0] / 2.0)), 0.5
        return lo + ext / 2.0, 0.25

    def expected_px(self, t: float) -> tuple[float, float]:
        """Where the target should be in a frame taken at t (box centre if no estimate yet)."""
        if self.x is None:
            return self._centre(self.box)
        pose = self.pose_at(t)
        dt = max(0.0, min(self.p["max_predict_s"], t - self.t_meas))
        a = [x + v * dt for x, v in zip(self.x, self.v)]
        return (self.W / 2 + self._ppd(0) * (pose[0] - a[0]), self.H / 2 + self._ppd(1) * (pose[1] - a[1]))

    def on_boxes(self, boxes: list, t_capture: float, now: float) -> bool:
        """One detection frame. True when the target was in it (and the estimate was updated)."""
        if t_capture <= self.t_meas:
            return False
        ex, ey = self.expected_px(t_capture)
        size = max(self.box["w"], self.box["h"])
        best, best_d = None, None
        for b in boxes:
            if int(b["cls"]) != self.cls:
                continue
            cx, cy = self._centre(b)
            d = math.hypot(cx - ex, cy - ey)
            if d <= self.p["gate_px"] + 0.75 * size and (best is None or d < best_d):
                best, best_d = b, d
        if best is None:
            return False
        self.measure(best, t_capture, now)
        return True

    def measure(self, b: dict, t: float, now: float):
        """One box of the target: update the direction estimate axis by axis, as far as the box can be trusted.
        A box that changed size a lot since the last one (turning, partly hidden, YOLO's box breathing) moves the
        position but not the speed; a cut side is handled by _axis."""
        p, pose = self.p, self.pose_at(t)
        area0 = max(1.0, self.box["w"] * self.box["h"])
        ratio = (b["w"] * b["h"]) / area0
        reshaped = self.x is not None and not (0.6 <= ratio <= 1.0 / 0.6)
        if self.x is None:
            self.x = [pose[0], pose[1]]
        dt = max(1e-3, t - self.t_meas)
        vmax = p["max_deg_s"] * 2
        first = self.n_meas == 0
        k = max(0.2, min(2.0, dt / p["gain_dt_s"]))    # this interval in units of the one the gains are for
        decay = 0.7 ** k
        for i in range(2):
            c, w = self._axis(b, i, t)
            self.trust[i] = w
            pred = self.x[i] + (0.0 if first else self.v[i] * dt)
            if c is None:                              # nothing to measure on this axis: coast, slowing down
                self.x[i] = pred
                self.v[i] *= decay
                continue
            a = pose[i] + ((self.W if i == 0 else self.H) / 2 - c) / self._ppd(i)
            if first:
                self.x[i] = a
                continue
            r = a - pred
            alpha = 1.0 - (1.0 - p["alpha"] * (1.0 if w >= 0.5 else 0.4)) ** k
            beta = p["beta"] * k ** p["beta_exp"] * (1.0 if w >= 1.0 else 0.5 if w >= 0.5 else 0.0) * \
                (0.0 if reshaped else 1.0)
            self.x[i] = pred + alpha * r
            if beta > 0:
                dvmax = p["max_dv_deg_s"] * k
                dv = max(-dvmax, min(dvmax, beta * r / dt))
                self.v[i] = max(-vmax, min(vmax, self.v[i] + dv))
            else:
                self.v[i] *= decay
        self.t_meas, self.seen, self.box = t, now, dict(b)
        self.n_meas += 1

    def _lead_frac(self) -> float:
        """Less prediction for a big box (a person close to the camera: big angles, quick reversals)."""
        lo, hi = self.p["near_box_px"]
        u = (self.box["w"] - lo) / max(1.0, hi - lo)
        return 1.0 - (1.0 - self.p["near_lead_frac"]) * max(0.0, min(1.0, u))

    def _v_eff(self, i: int) -> float:
        """The speed used for the prediction: the estimate minus its noise floor."""
        v, d = self.v[i], self.p["v_dead_deg_s"]
        return math.copysign(max(0.0, abs(v) - d), v)

    def lost_for(self, now: float) -> float:
        return now - self.seen

    def tick(self, now: float) -> str | None:
        """Every STREAM_S: the next SETPOSE_T_HR, or None when the arm stays where it is."""
        if now < self.next:
            return None
        self.next = max(self.next + self.STREAM_S, now + self.STREAM_S * 0.5)
        p, dt = self.p, self.STREAM_S
        fresh = self.lost_for(now) < p["hold_after_s"]
        if not fresh:
            self.v = [0.0, 0.0]                        # lost: hold on the last estimate
        moved = False
        self._update_coefficients()
        for i, j in enumerate(self.j):
            lead = max(0.0, min(p["max_predict_s"], now - self.t_meas + p["lag_s"])) * self._lead_frac()
            v = self._v_eff(i)
            ahead = max(-p["max_predict_deg"], min(p["max_predict_deg"], v * lead))
            want = self.x[i] + ahead
            lo, hi = self.plim if i == 0 else self.vlim[i]
            want = max(lo, min(hi, want))
            cur = self._axes(self._raw())[i]
            err = want - cur
            still = abs(v) < 1.0
            if still and abs(err) < p["deadband_deg"] * (0.3 if self.moving[i] else 1.0):
                self.moving[i] = False
                target_v = 0.0
            else:
                self.moving[i] = True
                target_v = v + err / p["tau_s"]
            target_v = max(-p["max_deg_s"], min(p["max_deg_s"], target_v))
            dv = max(-p["max_acc_deg_s2"] * dt, min(p["max_acc_deg_s2"] * dt, target_v - self.vel[i]))
            self.vel[i] += dv
            if not self.moving[i] and abs(self.vel[i]) < p["max_acc_deg_s2"] * dt:
                self.vel[i] = 0.0
            new = max(lo, min(hi, cur + self.vel[i] * dt))
            if new in (lo, hi):
                self.vel[i] = 0.0
            if i == 0 and self.bj is not None:
                moved |= self._split_pan(new, fresh, dt)
                continue
            if abs(new - self.cmd[j]) > 1e-4:
                moved = True
            self.cmd[j] = new
        self.hist.append((now, self._raw()))
        if not moved:
            return None
        return "SETPOSE_T_HR " + " ".join(str(int(round(v * 10))) for v in self.physical()) + \
            f" {int(self.STREAM_S * 1000)} RTR3"

    def _split_pan(self, pan: float, fresh: bool, dt: float) -> bool:
        """Share the pan between wrist yaw (fast, does the following) and base (slow): beyond base_dead_deg of
        wrist yaw from its centre the base turns to bring the wrist back, and the wrist gives back what the base
        adds, so the camera keeps pointing where the follower wants. True when a joint moved."""
        p, jy, jb = self.p, self.j[0], self.bj
        e = self.cmd[jy] - 90.0
        dead = p["base_dead_deg"] * (0.3 if self.bmoving else 1.0)
        target = 0.0
        if fresh and abs(e) > dead and abs(self.kb) > 0.2:
            target = (e - math.copysign(dead, e)) / self.kb / p["base_tau_s"]
            self.bmoving = abs(target) >= 0.5 or not self.bmoving   # no endless creeping at the end
            if not self.bmoving:
                target = 0.0
        else:
            self.bmoving = False
        target = max(-p["base_max_deg_s"], min(p["base_max_deg_s"], target))
        acc = p["base_max_acc_deg_s2"] * dt
        self.bvel += max(-acc, min(acc, target - self.bvel))
        if not self.bmoving and abs(self.bvel) < acc:
            self.bvel = 0.0
        lo, hi = self.blim
        b = max(lo, min(hi, self.cmd[jb] + self.bvel * dt))
        if b in (lo, hi):
            self.bvel = 0.0
        lo, hi = self.vlim[0]
        y = max(lo, min(hi, pan - self.kb * (b - 90.0)))
        moved = abs(b - self.cmd[jb]) > 1e-4 or abs(y - self.cmd[jy]) > 1e-4
        self.cmd[jb], self.cmd[jy] = b, y
        return moved

    def status(self, now: float) -> dict:
        err = None
        if self.x is not None:
            err = [round(self.x[i] - c, 1) for i, c in enumerate(self._axes(self._raw()))]
        lost = self.lost_for(now)
        return {"state": "segue" if lost < self.p["hold_after_s"] else "perso", "cls": self.cls,
                "box": [self.box["x"], self.box["y"], self.box["w"], self.box["h"]],
                "lost_ms": int(lost * 1000), "err_deg": err, "speed_deg_s": [round(v, 1) for v in self.v],
                "n": self.n_meas, "trust": list(self.trust),
                "base": round(self.cmd[self.bj], 1) if self.bj is not None else None,
                "scale": [round(v, 2) for v in self.scale], "base_gain": round(self.kb, 2)}


def track_pick(boxes: list, x: float | None = None, y: float | None = None) -> dict | None:
    """The box to follow: the smallest one containing (x, y) (default the crosshair, the image centre), else the
    nearest one to it within 200 px."""
    x = Tracker.W / 2 if x is None else float(x)
    y = Tracker.H / 2 if y is None else float(y)
    inside = [b for b in boxes if b["x"] <= x <= b["x"] + b["w"] and b["y"] <= y <= b["y"] + b["h"]]
    if inside:
        return min(inside, key=lambda b: b["w"] * b["h"])
    def gap(b):
        dx = max(b["x"] - x, 0, x - b["x"] - b["w"]); dy = max(b["y"] - y, 0, y - b["y"] - b["h"])
        return math.hypot(dx, dy)
    near = [b for b in boxes if gap(b) <= 200]
    return min(near, key=gap) if near else None


def settings_from_config(cfg: dict) -> dict:
    """What the dashboard edits, read back from the boot lines (the firmware's view) and cfg["settings"]."""
    lines = [l for l in cfg.get("boot_commands", []) if l not in ("IMUON", "IMUOFF")]   # older configs
    offsets = next(([int(v) for v in l.split()[1:7]] for l in lines if l.startswith("SET_OFFSETS ")), [90] * 6)
    limits = [[0, 180] for _ in range(6)]
    for l in lines:
        f = l.split()
        if f[0] == "SET_JOINT_LIMITS" and len(f) == 4 and f[1].isdigit() and int(f[1]) < 6:
            limits[int(f[1])] = [int(f[2]), int(f[3])]
    st = cfg.get("settings", {})
    dirs = list(st.get("dirs", [1] * 6))
    return {"offsets": offsets, "dirs": dirs, "limits": limits,
            "vr_params": next((l for l in lines if l.startswith("SET_VR_PARAMS ")), ""),
            "pwm_config": next((l for l in lines if l.startswith("SET_PWM_CONFIG ")), ""),
            "boot_commands": list(lines), **poses_from_settings(st)}


def poses_from_settings(st: dict) -> dict:
    """PARK / VR poses (virtual degrees), SETPOSE speed and profile used by pose_command(), the demo steps."""
    return {"park": list(st.get("park", POSE_DEFAULTS["park"])), "vr": list(st.get("vr", POSE_DEFAULTS["vr"])),
            "speed_pct": int(st.get("speed_pct", POSE_DEFAULTS["speed_pct"])),
            "profile": str(st.get("profile", POSE_DEFAULTS["profile"])),
            "auto_park_s": int(st.get("auto_park_s", POSE_DEFAULTS["auto_park_s"])),
            "demo": [dict(x) for x in st.get("demo", DEMO_DEFAULT)],
            "demo_style": str(st.get("demo_style", "smooth")),
            "demo_roll_deg": float(st.get("demo_roll_deg", DEMO_ROLL_DEFAULT))}


def poses_from_settings_msg(s: dict) -> tuple[dict | None, str | None]:
    """The optional pose fields of save_config, validated: (poses, None) or (None, error); {} when absent."""
    out = {}
    for name in ("park", "vr"):
        v = s.get(name)
        if v is None:
            continue
        if not (isinstance(v, list) and len(v) == 6 and
                all(isinstance(x, (int, float)) and not isinstance(x, bool) and 0 <= x <= 180 for x in v)):
            return None, f"{name}: 6 angoli virtuali 0..180"
        out[name] = [round(float(x), 1) for x in v]
    if s.get("speed_pct") is not None:
        pct = s["speed_pct"]
        if not (isinstance(pct, (int, float)) and not isinstance(pct, bool) and 1 <= pct <= 100):
            return None, "speed_pct: 1..100"
        out["speed_pct"] = int(pct)
    if s.get("profile") is not None:
        if s["profile"] not in ("RTR5", "RTR3", "BCB", "BB"):
            return None, "profile: RTR5, RTR3, BCB o BB"
        out["profile"] = s["profile"]
    if s.get("auto_park_s") is not None:
        v = s["auto_park_s"]
        if not (isinstance(v, (int, float)) and not isinstance(v, bool) and 0 <= v <= 3600):
            return None, "auto_park_s: 0..3600 (0 = mai)"
        out["auto_park_s"] = int(v)
    if s.get("demo") is not None:
        steps = s["demo"]
        if not isinstance(steps, list) or len(steps) > 40:
            return None, "demo: lista di al massimo 40 passi"
        out["demo"] = []
        for k, step in enumerate(steps):
            a = step.get("angles") if isinstance(step, dict) else None
            if not (isinstance(a, list) and len(a) == 6 and
                    all(isinstance(x, (int, float)) and not isinstance(x, bool) and 0 <= x <= 180 for x in a)):
                return None, f"demo passo {k + 1}: 6 angoli virtuali 0..180"
            pct, prof = step.get("speed_pct", POSE_DEFAULTS["speed_pct"]), step.get("profile", POSE_DEFAULTS["profile"])
            if not (isinstance(pct, (int, float)) and not isinstance(pct, bool) and 1 <= pct <= 100):
                return None, f"demo passo {k + 1}: velocità 1..100"
            if prof not in ("RTR5", "RTR3", "BCB", "BB"):
                return None, f"demo passo {k + 1}: profilo RTR5, RTR3, BCB o BB"
            out["demo"].append({"angles": [round(float(x), 1) for x in a], "speed_pct": int(pct), "profile": prof})
    if s.get("demo_style") is not None:
        if s["demo_style"] not in ("smooth", "steps"):
            return None, "demo_style: smooth o steps"
        out["demo_style"] = s["demo_style"]
    if s.get("demo_roll_deg") is not None:
        v = s["demo_roll_deg"]
        if not (isinstance(v, (int, float)) and not isinstance(v, bool) and 0 <= v <= 40):
            return None, "demo_roll_deg: 0..40"
        out["demo_roll_deg"] = round(float(v), 1)
    return out, None


def boot_lines_from_settings(s: dict) -> tuple[list[str] | None, str | None]:
    """Validated boot lines in the Pi's order (SET_OFFSETS, SET_VR_PARAMS, SET_JOINT_LIMITS x6,
    SET_PWM_CONFIG), or (None, error). No IMUON: the IMU is always on."""
    def ints(v, n, lo, hi):
        return (isinstance(v, list) and len(v) == n and
                all(isinstance(x, int) and not isinstance(x, bool) and lo <= x <= hi for x in v))
    if not ints(s.get("offsets"), 6, 0, 180):
        return None, "offsets: 6 interi 0..180"
    if not (isinstance(s.get("dirs"), list) and len(s["dirs"]) == 6 and all(d in (1, -1) for d in s["dirs"])):
        return None, "dirs: 6 valori 1 o -1"
    lim = s.get("limits")
    if not (isinstance(lim, list) and len(lim) == 6 and all(ints(x, 2, 0, 180) and x[0] < x[1] for x in lim)):
        return None, "limits: 6 coppie [min, max] intere 0..180 con min < max"
    lines = ["SET_OFFSETS " + " ".join(str(v) for v in s["offsets"])]
    for key, prefix in (("vr_params", "SET_VR_PARAMS "), ("pwm_config", "SET_PWM_CONFIG ")):
        v = s.get(key) or ""
        if v and (not v.startswith(prefix) or any(not (c.isalnum() or c in " .-_") for c in v) or len(v) > 400):
            return None, f"{key}: deve iniziare con '{prefix.strip()}' e contenere solo numeri"
        if v and key == "vr_params":
            lines.append(v)
    lines += [f"SET_JOINT_LIMITS {i} {a} {b}" for i, (a, b) in enumerate(lim)]
    if s.get("pwm_config"):
        lines.append(s["pwm_config"])
    return lines, None


def save_config_file(path: str, lines: list[str], dirs: list[int], poses: dict | None = None):
    """Rewrites only boot_commands and settings in the config file (the in-memory cfg has absolute paths),
    keeping the previous file as .bak. poses: park / vr / speed_pct / profile, only the keys given."""
    with open(path, encoding="utf-8") as f:
        disk = json.load(f)
    disk["boot_commands"] = lines
    disk.setdefault("settings", {})["offsets"] = [int(v) for v in lines[0].split()[1:7]]
    disk["settings"]["dirs"] = dirs
    disk["settings"].update(poses or {})
    try:
        os.replace(path, path + ".bak")
    except OSError:
        pass
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(disk, f, indent=1)
        f.write("\n")
    os.replace(tmp, path)


# ------------------------------------------------------------------ kinematics (POE, dashboard + ASSIST gimbal)

KIN_PATH = pathlib.Path(__file__).resolve().parent.parent / "dashboard" / "kinematics.json"


def load_kinematics(path=KIN_PATH) -> dict | None:
    try:
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def kinematics_from_msg(msg: dict) -> tuple[dict | None, str | None]:
    """The dashboard's POE editor: S (6 screw axes [w v], w unit), M (4x4, last row 0 0 0 1), draw_points (6 x 3),
    metres. Validated: (data, None) or (None, error)."""
    def nums(v, n):
        return (isinstance(v, list) and len(v) == n and
                all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in v))
    S = msg.get("S")
    if not (isinstance(S, list) and len(S) == 6 and all(nums(s, 6) for s in S)):
        return None, "S: 6 assi di vite da 6 numeri [wx wy wz vx vy vz]"
    for i, s in enumerate(S):
        if abs(math.sqrt(s[0] ** 2 + s[1] ** 2 + s[2] ** 2) - 1.0) > 1e-3:
            return None, f"S[{i}]: w deve essere unitario (|w| = 1)"
    M = msg.get("M")
    if not (isinstance(M, list) and len(M) == 4 and all(nums(r, 4) for r in M) and M[3] == [0, 0, 0, 1]):
        return None, "M: matrice 4x4 con ultima riga 0 0 0 1"
    dp = msg.get("draw_points")
    if not (isinstance(dp, list) and len(dp) == 6 and all(nums(x, 3) for x in dp)):
        return None, "draw_points: 6 punti [x y z]"
    f = lambda rows: [[float(x) for x in r] for r in rows]  # noqa: E731
    return {"S": f(S), "M": f(M), "draw_points": f(dp)}, None


def save_kinematics_file(path, data: dict):
    """Rewrites S, M and draw_points in kinematics.json (docs and joint names kept), previous file as .bak."""
    path = str(path)
    disk = load_kinematics(path) or {}
    disk.update(data)
    try:
        os.replace(path, path + ".bak")
    except OSError:
        pass
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(disk, f, indent=1)
        f.write("\n")
    os.replace(tmp, path)


# ------------------------------------------------------------------ cameras (tools/cam_stream.py)

class Cameras:
    """Camera controls of the native Quest app, as ws-teleop (ws_handlers_imu.py): JSON over UDP to the two
    capture processes (127.0.0.1:5600 + camera); zoom and brightness re-sent every second and the mode every
    3 s until cam0 reports it (a capture restart comes back at 1x, ev 0, tele). cam0 publishes its applied
    state in /dev/shm/jonny5_cam0_zoom.json; vr_zoom0/1 in the telemetry are the residual digital zoom."""
    PORT_BASE = 5600
    STATE_PATH = "/dev/shm/jonny5_cam0_zoom.json"
    # stereo head (jonny5-stereo-head sender/src/protocol.hpp): ControlHeader + payload on its control port
    J5_MAGIC, CTRL_EXPOSURE, CTRL_FOCUS, CTRL_COLLIM_CAL = 0x4A35, 14, 15, 16
    FOCUS_AUTO, FOCUS_ONESHOT, FOCUS_FIXED, FOCUS_HOLD = 0, 1, 2, 3

    def __init__(self, state_path: str = STATE_PATH, clock: Callable[[], float] = time.monotonic, sendto=None,
                 board_ctrl_port: int | None = None):
        """board_ctrl_port: the robot has the stereo head (config board.ctrl_port, 20/11/2026): focus, refocus,
        focus hold and brightness go to its sender as binary control datagrams; zoom and tele/wide are the
        headset's business there and are not forwarded."""
        self.state_path, self.clock = state_path, clock
        self._sock = None
        self._sendto = sendto or self._udp
        self.board_port = board_ctrl_port
        self.zoom = self.ev = self.mode = None
        self.zoom_sent = self.ev_sent = self.mode_sent = -1e9
        self.vr_zoom = [None, None]
        self.last_af = -1e9
        self.focus_hold = False

    def _board(self, typ: int, payload: bytes):
        hdr = struct.pack("<HBBI", self.J5_MAGIC, typ, 0, int(time.monotonic() * 1e6) & 0xFFFFFFFF)
        self._sendto(hdr + payload, self.board_port)

    def collim_calibrate(self, msg: dict) -> bool:
        """Dashboard "calibra a questa distanza" on the stereo head: kCtrlCollimCal to the sender, which pairs the
        distance with its recent locked dx, refits dx_inf / f_px, saves them and answers with kCtrlCollimCalResult
        (BoardMeta -> "stereo_calibration"). reset: forget the previous points (alone, or before this one)."""
        if not self.board_port:
            return False
        mm = msg.get("distance_mm", 0)
        reset = bool(msg.get("reset"))
        if not self._num(mm) or not (mm == 0 and reset or 100 <= mm <= 10000):
            return False
        self._board(self.CTRL_COLLIM_CAL, struct.pack("<HBB", int(mm), 1 if reset else 0, 0))
        return True

    def _board_focus(self, mode: int, dioptres: float = 0.0):
        self._board(self.CTRL_FOCUS, struct.pack("<BBH", mode, 0, int(round(max(0.0, min(16.0, dioptres)) * 256))))

    def _udp(self, data: bytes, port: int):
        import socket
        try:
            if self._sock is None:
                self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.sendto(data, ("127.0.0.1", port))
        except OSError:
            pass

    def _send(self, obj: dict, cams=(0, 1)):
        data = json.dumps(obj).encode()
        for cam in cams:
            self._sendto(data, self.PORT_BASE + cam)

    @staticmethod
    def _num(v) -> bool:
        return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)

    def on_message(self, msg: dict) -> bool:
        if self.board_port:
            return self._on_board_message(msg)
        typ = msg.get("type")
        if typ == "vr_zoom_state":
            for i in (0, 1):
                if self._num(msg.get(f"zoom{i}")):
                    self.vr_zoom[i] = float(msg[f"zoom{i}"])
            if msg.get("camera") and self._num(msg.get("zoom0")):
                self.zoom, self.zoom_sent = float(msg["zoom0"]), self.clock()
                self._send({"zoom": self.zoom})
        elif typ == "cam_exposure":
            if self._num(msg.get("ev")):
                self.ev, self.ev_sent = max(-2.0, min(2.0, float(msg["ev"]))), self.clock()
                self._send({"ev": self.ev})
        elif typ == "cam_mode":
            if msg.get("mode") in ("tele", "wide"):
                self.mode, self.mode_sent = msg["mode"], self.clock()
                self._send({"mode": self.mode})
        elif typ == "cam_autofocus":                 # one-shot AF on both cameras (dashboard / phone video test)
            self.autofocus()
        elif typ == "cam_focus":                     # cam0 only: cam1 follows the master (cam_stream --follow)
            v = "auto" if msg.get("auto") else msg.get("dioptres", "auto")
            if v == "auto" or self._num(v):
                self._send({"focus": v if v == "auto" else max(0.0, min(8.0, float(v)))}, cams=(0,))
        else:
            return False
        return True

    def _on_board_message(self, msg: dict) -> bool:
        typ = msg.get("type")
        if typ == "vr_zoom_state":
            for i in (0, 1):
                if self._num(msg.get(f"zoom{i}")):
                    self.vr_zoom[i] = float(msg[f"zoom{i}"])
        elif typ == "cam_mode":
            pass
        elif typ == "cam_exposure":
            if self._num(msg.get("ev")):
                self.ev = max(-2.0, min(2.0, float(msg["ev"])))
                self._board(self.CTRL_EXPOSURE, struct.pack("<hBB", int(round(self.ev * 256)), 0, 0))
        elif typ == "cam_autofocus":                 # dashboard Refocus: one search, then stereo focus (or held)
            self.autofocus()
        elif typ == "cam_focus":
            v = "auto" if msg.get("auto") else msg.get("dioptres", "auto")
            if v == "auto":
                self.focus_hold = False
                self._board_focus(self.FOCUS_AUTO)
            elif self._num(v):
                self.focus_hold = False
                self._board_focus(self.FOCUS_FIXED, float(v))
        elif typ == "cam_focus_hold":                # lock both lenses where they are / back to automatic
            self.focus_hold = bool(msg.get("on"))
            self._board_focus(self.FOCUS_HOLD if self.focus_hold else self.FOCUS_AUTO)
        else:
            return False
        return True

    def autofocus(self) -> bool:
        """One-shot AF on both cameras (POST /api/refocus-cameras); False within 1 s of the last one."""
        now = self.clock()
        if now - self.last_af < 1.0:
            return False
        self.last_af = now
        if self.board_port:
            self._board_focus(self.FOCUS_ONESHOT)
        else:
            self._send({"af": {"src": "button"}})
        return True

    def state(self):
        try:
            with open(self.state_path) as f:
                st = json.load(f)
            if time.time() - float(st.get("ts", 0)) < 3.0:
                return st
        except (OSError, ValueError, TypeError):
            pass
        return None

    def telemetry(self) -> dict:
        now, out = self.clock(), {}
        if self.board_port:
            out["cam_focus_hold"] = self.focus_hold
            if self.ev is not None:
                out["cam_ev"] = self.ev
            for i in (0, 1):
                if self.vr_zoom[i] is not None:
                    out[f"vr_zoom{i}"] = max(1.0, self.vr_zoom[i])
            return out
        st = self.state()
        cz = float(st.get("zoom", 1.0)) if st else None
        if cz is not None:
            out["cam_zoom"] = round(cz, 3)
        if self.zoom is not None and now - self.zoom_sent > 1.0:
            self.zoom_sent = now
            self._send({"zoom": self.zoom})
        if st:
            out.update(cam_mode=st.get("mode", "tele"), cam_hfov=st.get("hfov", 46.8), cam_fps=st.get("fps"))
            if self.mode and st.get("mode") != self.mode and now - self.mode_sent > 3.0:
                self.mode_sent = now
                self._send({"mode": self.mode})
        if self.ev is not None:
            out["cam_ev"] = self.ev
            if now - self.ev_sent > 1.0:
                self.ev_sent = now
                self._send({"ev": self.ev})
        div = cz if (cz is not None and self.zoom is not None) else 1.0
        for i in (0, 1):
            if self.vr_zoom[i] is not None:
                out[f"vr_zoom{i}"] = max(1.0, self.vr_zoom[i] / div)
        return out


# ------------------------------------------------------------------ hardware backends

class SpidevBackend:
    """SPI master, mode 0, 8 bits. Uses the spidev module when installed, otherwise the kernel's spidev ioctl
    directly (stdlib only: the stereo head's Debian image has neither spidev nor the headers to build it)."""

    _IOC_MESSAGE_1 = 0x40206B00   # SPI_IOC_MESSAGE(1) = _IOW('k', 0, struct spi_ioc_transfer[1]) (32 bytes)
    _IOC_WR_MODE = 0x40016B01
    _IOC_WR_BITS = 0x40016B03
    _IOC_WR_SPEED = 0x40046B04

    def __init__(self, device: str, speed_hz: int):
        self.speed = speed_hz
        try:
            import spidev
        except ImportError:
            spidev = None
        if spidev is not None:
            bus, dev = device.replace("/dev/spidev", "").split(".")
            self.s = spidev.SpiDev()
            self.s.open(int(bus), int(dev))
            self.s.mode, self.s.bits_per_word, self.s.max_speed_hz = 0, 8, speed_hz
            self.fd = None
            return
        import fcntl, struct
        self.s = None
        self.fd = os.open(device, os.O_RDWR)
        fcntl.ioctl(self.fd, self._IOC_WR_MODE, struct.pack("B", 0))
        fcntl.ioctl(self.fd, self._IOC_WR_BITS, struct.pack("B", 8))
        fcntl.ioctl(self.fd, self._IOC_WR_SPEED, struct.pack("I", speed_hz))

    def transfer(self, data: bytes) -> bytes:
        if self.s is not None:
            return bytes(self.s.xfer2(list(data)))
        import ctypes, fcntl, struct
        n = len(data)
        tx = ctypes.create_string_buffer(bytes(data), n)
        rx = ctypes.create_string_buffer(n)
        # struct spi_ioc_transfer: u64 tx_buf, u64 rx_buf, u32 len, u32 speed_hz, u16 delay_usecs,
        # u8 bits_per_word, u8 cs_change, u8 tx_nbits, u8 rx_nbits, u8 word_delay_usecs, u8 pad
        xfer = struct.pack("QQIIHBBBBBB", ctypes.addressof(tx), ctypes.addressof(rx), n, self.speed, 0, 8,
                           0, 0, 0, 0, 0)
        fcntl.ioctl(self.fd, self._IOC_MESSAGE_1, xfer)
        return rx.raw[:n]


class SerialBackend:
    """Non-blocking 8N1 serial port: pyserial when installed, otherwise termios (stdlib only)."""

    def __init__(self, device: str, baud: int):
        try:
            import serial
        except ImportError:
            serial = None
        if serial is not None:
            self.s = serial.Serial(device, baud, timeout=0)
            self.fd = None
            return
        import termios
        self.s = None
        self.fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        speed = getattr(termios, f"B{baud}")
        attr = termios.tcgetattr(self.fd)
        attr[0] = 0                                                     # iflag: raw
        attr[1] = 0                                                     # oflag: raw
        attr[2] = termios.CS8 | termios.CREAD | termios.CLOCAL          # cflag: 8N1, no modem lines
        attr[3] = 0                                                     # lflag: no echo, no canonical mode
        attr[4] = attr[5] = speed
        attr[6][termios.VMIN] = 0
        attr[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attr)
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def write(self, data: bytes):
        if self.s is not None:
            self.s.write(data)
            return
        view = memoryview(data)
        while view:
            try:
                n = os.write(self.fd, view)
            except BlockingIOError:
                time.sleep(0.001)
                continue
            view = view[n:]

    def read(self) -> bytes:
        if self.s is not None:
            return self.s.read(4096)
        try:
            return os.read(self.fd, 4096)
        except BlockingIOError:
            return b""


class SimRobot:
    """--sim: the real firmware compiled for this PC (firmware/stm32/tools/fw_sim) as SPI + UART backend.
    The control thread advances its clock by 10 ms after every step, so it runs in real time."""

    def __init__(self, transfer_len: int):
        sys.path.insert(0, str(SIM_DIR))
        import fw_sim
        out = fw_sim.BUILD / ("fw_sim_j5mini_sim" + fw_sim.LIB.suffix)
        if not out.exists() or out.stat().st_mtime < max(f.stat().st_mtime for f in fw_sim.SRC.rglob("*.[ch]")):
            fw_sim.build(padded128=transfer_len == 128, out=out)
        self.sim = fw_sim.Sim(out)

    def transfer(self, data: bytes) -> bytes:
        return self.sim.spi(data)

    def write(self, data: bytes):
        self.sim.lib.sim_uart_write(data)

    def read(self) -> bytes:
        return self.sim.uart_read().encode()

    def tick(self):
        self.sim.run(10)


HERE = pathlib.Path(__file__).resolve().parent
SIM_DIR = HERE.parents[1] / "firmware" / "stm32" / "tools" / "fw_sim"


# ------------------------------------------------------------------ WebSocket server (ws-teleop subset)

class Server:
    # forwarded as they are to the other clients, like ws-teleop; the optional services on the robot
    # (raspberry/tools/stereo_collimator.py, vision_service.py) are clients of this server too
    RELAY = ("cameras_refocus_triggered", "vr_session_state", "vr_zoom_command",
             "stereo_collimation", "stereo_calibrate", "vision_request", "vision_result",
             "voice_command")   # stereo head: tools/voice/j5_voice.py -> headset app (it executes them)

    def __init__(self, core: Core, lock: threading.Lock, cfg: dict, cams: Cameras | None = None, kin_path=KIN_PATH):
        self.core, self.lock, self.cfg = core, lock, cfg
        self.cams = cams or Cameras()
        self.clients = set()
        self.ignored = set()
        self.board = None           # BoardMeta when the stereo head's meta mirror is listened to
        self.kin_path = kin_path
        self.kin = load_kinematics(kin_path) or {}      # POE: the dashboard draws with it, the ASSIST gimbal solves with it
        self.assist = Assist.create(core, lock, cfg)
        if self.assist and self.kin.get("S"):
            with self.lock:
                self.assist.configure(self.kin["S"])

    # A client that stops reading (headset asleep, phone in the pocket) fills its socket; without a limit the
    # broadcast waited for it and the whole asyncio loop froze (27/10/2026: 6 minutes without telemetry, dashboard
    # and STOP, the Quest app paused with 87 KB queued). A send that takes longer than this drops the client.
    SEND_TIMEOUT_S = 0.5

    async def handler(self, ws):
        self.clients.add(ws)
        try:
            async for raw in ws:
                try:
                    msg = json.loads(raw)
                except ValueError:
                    continue
                await self.on_message(ws, msg)
        except _WS_CLOSED:
            pass                                     # dropped without a close frame: normal for apps and phones
        finally:
            self.clients.discard(ws)

    async def send_to(self, ws, data: str):
        """One client, bounded: a slow one is dropped instead of holding the others."""
        try:
            await asyncio.wait_for(ws.send(data), self.SEND_TIMEOUT_S)
        except (asyncio.TimeoutError, Exception) as e:
            if ws in self.clients:
                self.clients.discard(ws)
                why = "send blocked > %.1f s" % self.SEND_TIMEOUT_S if isinstance(e, asyncio.TimeoutError) else type(e).__name__
                self.core.log(f"[j5mini] client {getattr(ws, 'remote_address', '?')} dropped: {why}")
                try:
                    asyncio.get_running_loop().create_task(ws.close(code=1011, reason="slow client"))
                except RuntimeError:
                    pass

    async def send_all(self, data: str, but=None):
        await asyncio.gather(*(self.send_to(c, data) for c in list(self.clients) if c is not but))

    async def on_message(self, ws, msg: dict):
        typ = msg.get("type")
        if typ is None and "mode" in msg:
            with self.lock:
                arm = self.assist.step(msg) if self.assist and msg.get("mode") == 5 else None
                err = self.core.set_intent(msg, arm)
            if err:
                await ws.send(json.dumps({"type": "error", "error": err}))
            elif arm is not None:
                done = self.assist.pop_goal_done()
                if done:   # point at an object: the arm got there -> the vision service looks again from close up
                    await self.broadcast({"type": "vision_point", "ok": True, "done": True, "label": done.get("label"),
                                          "dist_after_mm": done.get("dist_after_mm")})
                    await self.broadcast({"type": "vision_request", "action": "analyze", "req": "after_point"})
            return
        if typ == "uart" and isinstance(msg.get("cmd"), str):
            cmd = msg["cmd"].strip()
            if self.assist and cmd == "HEADZERO":
                self.assist.headzero_external()
            if cmd in ("PARK", "TELEOPPOSE"):
                self.pose_command(cmd)
                return
            with self.lock:
                self.core.uart_send(cmd)
        elif typ == "set_vr_mode":
            if msg.get("mode") in (3, 4, 5):
                with self.lock:
                    self.core.resend_vr_params()
            await ws.send(json.dumps({"type": "ack", "set_vr_mode": msg.get("mode")}))
        elif typ == "imu_campaign":                  # record desired / commanded / firmware / IMU orientation
            res = {"on": False, "why": "ASSIST non disponibile"}
            if self.assist:
                with self.lock:
                    res = self.assist.g.set_campaign(bool(msg.get("on")), str(msg.get("label", "")))
            await ws.send(json.dumps({"type": "imu_campaign", **res}))
        elif typ == "demo":                          # dashboard "Demo": the saved pose sequence, looped
            with self.lock:
                if msg.get("on") and self.core.demo:
                    ok, err = False, "demo già in corso"
                elif msg.get("on") and self.cfg.get("settings", {}).get("demo_style", "smooth") == "smooth":
                    ok, err = self.core.demo_start_smooth(DemoPath(self.cfg))
                elif msg.get("on"):
                    ok, err = self.core.demo_start(demo_lines(self.cfg))
                else:
                    self.core.demo_stop("dashboard", hold=True)
                    ok, err = True, ""
            await ws.send(json.dumps({"type": "demo", "ok": ok, "on": bool(msg.get("on")) and ok, "error": err or None}))
        elif typ == "track":                         # dashboard "Segui": follow the object under (x, y)
            ok, err = self._track_message(msg)
            await ws.send(json.dumps({"type": "track", "ok": ok, "on": bool(msg.get("on")) and ok, "error": err or None}))
        elif typ == "horizon_lock":                  # "Orizzonte": the image stays level in every mode
            on = bool(msg.get("on"))
            with self.lock:
                self.core.set_horizon(on)
                if self.assist:
                    self.assist.set_horizon_lock(on)    # ASSIST also levels its own camera target
            await self.broadcast({"type": "horizon_lock", "on": on})
        elif typ == "time_sync":
            self.time_sync(msg.get("epoch_ms"))
        elif typ == "get_settings":
            await ws.send(json.dumps({"type": "settings", **self.cfg.get("settings", {})}))
        elif typ == "get_config":
            with self.lock:
                s = settings_from_config(self.cfg)
                s["horizon_lock"] = self.core.horizon_on
            await ws.send(json.dumps({"type": "config", **s, "sim": bool(self.cfg.get("_sim")),
                                      "saveable": bool(self.cfg.get("_path"))}))
        elif typ == "save_config":
            await ws.send(json.dumps(self.save_config(msg)))
        elif typ == "get_kinematics":
            await ws.send(json.dumps({"type": "kinematics", **{k: v for k, v in self.kin.items() if not k.startswith("_")},
                                      "saveable": bool(self.cfg.get("_path"))}))
        elif typ == "save_kinematics":
            await ws.send(json.dumps(self.save_kinematics(msg)))
        elif typ == "ws_ping":
            await ws.send(json.dumps({"type": "ws_pong", "id": msg.get("id"), "ts_client": msg.get("ts_client"),
                                      "ts_server": time.time()}))
        elif typ in self.RELAY:
            await self.send_all(json.dumps(msg), but=ws)
            if typ == "stereo_calibrate":
                self.cams.collim_calibrate(msg)          # stereo head: the sender calibrates (no-op on the Pi)
            if typ == "vision_request" and self.board is not None and self.board.n:
                # stereo head board: no vision_service, the answer comes from the NPU boxes + collimator
                res = self.board.vision_result(msg)
                self.core.log(f"[j5mini] vision_request {msg.get('action')} -> board: {len(res['objects'])} objects, "
                              f"target {res['target'] and res['target'].get('label_it')} {res['target'] and res['target'].get('dist_mm')} mm")
                await self.broadcast(res)
            if typ == "vision_result" and msg.get("aim"):
                # right trigger in ASSIST: the vision service measured the object under the crosshair, the gimbal
                # moves the arm to point cam0 at it (grips held); the outcome goes to every client
                tgt = msg.get("target") or {}
                with self.lock:
                    ok, info = (self.assist.request_point(tgt.get("cam_xyz_mm"), tgt.get("label_it")) if self.assist
                                else (False, {"why": "ASSIST non disponibile (numpy mancante)"}))
                self.core.log(f"[j5mini] point {tgt.get('label_it')}: ok={ok} {info}")
                await self.broadcast(dict({k: v for k, v in info.items() if k != "pose_v"}, type="vision_point",
                                          ok=ok, label=tgt.get("label_it")))
        elif self.cams.on_message(msg):
            pass
        elif typ not in self.ignored:
            self.ignored.add(typ)
            self.core.log(f"[j5mini] message type {typ!r} not handled by the minimal controller (ignored)")

    def _track_message(self, msg: dict) -> tuple[bool, str]:
        if not msg.get("on"):
            with self.lock:
                self.core.track_stop("dashboard", hold=True)
            return True, ""
        b = self.board
        if b is None:
            return False, "nessuna testa stereo (meta NPU) su questo controllore"
        fresh = time.monotonic() - b.boxes_at < 1.0
        x, y = msg.get("x"), msg.get("y")
        if x is not None and not (isinstance(x, (int, float)) and isinstance(y, (int, float))):
            return False, "coordinate non valide"
        box = track_pick(b.boxes if fresh else [], x, y)
        if box is None:
            return False, ("nessun oggetto riconosciuto lì" if fresh else "nessun oggetto riconosciuto dalla NPU")
        with self.lock:
            if self.core.track:
                self.core.track_stop("nuovo bersaglio")
            return self.core.track_start(box, b.boxes_tcap, kin=self.kin)

    def pose_command(self, cmd: str):
        """PARK / TELEOPPOSE as ws-teleop: SETPOSE to the saved pose; the reply goes out under the client's command,
        and on OK teleop_pose_ack (the Quest app waits for it, up to 5 s, before its next step)."""
        with self.lock:
            line = pose_setpose_line(self.cfg, "park" if cmd == "PARK" else "vr")
            core = self.core

            def done(ok, resp):                # control thread, lock held
                core.events.append({"type": "uart_response", "cmd": cmd, "ok": ok, "response": resp})
                if ok:
                    core.events.append({"type": "teleop_pose_ack", "teleop_pose_ack": True,
                                        "id": int(time.monotonic() * 1000)})
            core.uart_send(line, done)
        self.core.log(f"[j5mini] {cmd} -> {line}")

    async def broadcast(self, obj: dict):
        await self.send_all(json.dumps(obj))

    def save_config(self, msg: dict) -> dict:
        """Dashboard settings -> boot lines (validated) -> j5mini.json (+ .bak) and the running config. The
        firmware gets them at its next BOOT_READY, or now from the dashboard's "Applica" (UART lines)."""
        lines, err = boot_lines_from_settings(msg)
        if err:
            return {"type": "config_saved", "ok": False, "error": err}
        poses, err = poses_from_settings_msg(msg)
        if err:
            return {"type": "config_saved", "ok": False, "error": err}
        path = self.cfg.get("_path")
        try:
            if path:
                save_config_file(path, lines, list(msg["dirs"]), poses)
        except (OSError, ValueError) as e:
            return {"type": "config_saved", "ok": False, "error": str(e)}
        with self.lock:
            self.cfg["boot_commands"] = lines
            self.cfg.setdefault("settings", {})["offsets"] = list(msg["offsets"])
            self.cfg["settings"]["dirs"] = list(msg["dirs"])
            self.cfg["settings"].update(poses)
            self.core.auto_park_s = float(self.cfg["settings"].get("auto_park_s", POSE_DEFAULTS["auto_park_s"]))
            self.core.horizon_dirs = [1 if d >= 0 else -1 for d in msg["dirs"]]
            self.core.boot_lines = list(lines)
            self.core.vr_params_line = next((l for l in lines if l.startswith("SET_VR_PARAMS")), None)
            if self.assist:
                self.assist.configure(self.kin.get("S"))
        self.core.log(f"[j5mini] configuration saved ({'file' if path else 'memory only'})")
        return {"type": "config_saved", "ok": True, "saved_to_file": bool(path), "boot_commands": lines}

    def save_kinematics(self, msg: dict) -> dict:
        """POE editor of the dashboard (Impostazioni -> Cinematica): validated -> kinematics.json (+ .bak, only when
        the config is on disk, as save_config) and at once into the ASSIST gimbal (wrist IK, arm geometry)."""
        data, err = kinematics_from_msg(msg)
        if err:
            return {"type": "kinematics_saved", "ok": False, "error": err}
        path = self.cfg.get("_path")
        try:
            if path:
                save_kinematics_file(self.kin_path, data)
        except OSError as e:
            return {"type": "kinematics_saved", "ok": False, "error": str(e)}
        with self.lock:
            self.kin.update(data)
            if self.assist:
                self.assist.configure(data["S"])
        self.core.log(f"[j5mini] kinematics saved ({'file' if path else 'memory only'})")
        return {"type": "kinematics_saved", "ok": True, "saved_to_file": bool(path)}

    def time_sync(self, epoch_ms):
        if not self.cfg.get("set_clock") or not isinstance(epoch_ms, (int, float)):
            return
        if abs(time.time() - epoch_ms / 1000.0) > 2.0:
            import subprocess
            subprocess.run(["sudo", "-n", "date", "-u", "-s", f"@{epoch_ms / 1000.0:.3f}"], check=False)

    def write_telemetry_file(self, tm: dict):
        """Servo angles for vision_service.py (it reads the old Pi's /dev/shm/j5vr_telemetry.json)."""
        path = self.cfg.get("telemetry_file")
        if not path or not all(k in tm for k in SERVO_KEYS):
            return
        try:
            tmp = path + ".tmp"
            with open(tmp, "w") as f:
                json.dump({k: tm[k] for k in SERVO_KEYS}, f)
            os.replace(tmp, path)
        except OSError:
            pass

    async def broadcast_loop(self):
        n = 0
        while True:
            await asyncio.sleep(0.05)
            n += 1
            with self.lock:
                if n % 4 == 0:
                    self.core.horizon_trim_step(self.assist.imu_up_tool() if self.assist else None)
                msgs = [{**self.core.telemetry_message(), **self.cams.telemetry(), "clients": len(self.clients),
                         "horizon_lock": self.core.horizon_on, "horizon_trim": round(self.core.horizon_trim, 2)}] + \
                    self.core.events
                self.core.events = []
            self.write_telemetry_file(msgs[0])
            if self.clients:
                for m in msgs:
                    await self.send_all(json.dumps(m))


# ------------------------------------------------------------------ ASSIST (mode 5): the old Pi's gimbal

ASSIST_DEFAULTS = {          # the robot's routing_config.json (gimbalPosition / gimbalPrediction / gimbalImuTrim)
    "mode": "gimbal",
    "position": {"enable": True, "scale": 0.7},
    "prediction": {"enable": True, "lead_s": 0.2, "max_deg": 12.0, "yaw_lead_s": 0.12, "brake": True,
                   "base_tau_s": 0.0},          # 0: the wrist is solved on the intended arm, the arm command is
                                                # delayed instead (assist_gimbal ARM_DELAY_S, from the IMU campaign)
    "imu_trim": {"enable": True, "tau_s": 1.0, "deadband_deg": 0.7, "max_deg": 8.0},
}


class Assist:
    """Mode 5 like ws-teleop with "assistMode": "gimbal": per intent, B/S/G targets (physical, in the J5VR
    payload) and a synthetic head quaternion for the firmware's wrist pipeline. Grips released: nothing moves;
    pressed again: new references, no jump. Off with "assist": {"mode": "off"} (B/S/G then hold, as before)."""

    LIMIT_NAMES = ("base", "spalla", "gomito", "yaw", "pitch", "roll")

    def __init__(self, core: Core, lock: threading.Lock, cfg: dict, gimbal):
        self.core, self.lock, self.cfg, self.g = core, lock, cfg, gimbal
        a = cfg.get("assist", {})
        self.opts = {k: {**v, **a.get(k, {})} if isinstance(v, dict) else a.get(k, v)
                     for k, v in ASSIST_DEFAULTS.items()}
        self.uart = _UartBridge(core, lock)
        self.configure()
        gimbal.set_horizon_lock(bool(cfg.get("horizon_lock", False)))

    @classmethod
    def create(cls, core, lock, cfg):
        if cfg.get("assist", {}).get("mode", "gimbal") != "gimbal":
            return None
        try:
            import assist_gimbal
        except ImportError as e:            # numpy missing: mode 5 holds B/S/G as without ASSIST
            core.log(f"[j5mini] ASSIST gimbal not available ({e}): mode 5 moves the wrist only")
            return None
        return cls(core, lock, cfg, assist_gimbal)

    def configure(self, screws=None):
        s = settings_from_config(self.cfg)
        if screws is None:
            screws = (load_kinematics() or {}).get("S")
        self.limits = {n: {"min": lo, "max": hi} for n, (lo, hi) in zip(self.LIMIT_NAMES, s["limits"])}
        self.g.configure(s["offsets"], s["dirs"], screws=screws, telemetry=lambda: self.core.telemetry)

    def step(self, intent: dict):
        """Called with the lock held; rewrites intent quat_*; returns the mode-5 arm tuple for the payload."""
        tm = self.core.telemetry
        six = [float(tm[k]) for k in SERVO_KEYS] if all(k in tm for k in SERVO_KEYS) else None
        vr = (self.core.vr_params_line or "").split()[1:]
        if len(vr) < 19:
            return None
        return self.g.step(intent, physical_six=six, limits_src=self.limits, vr_params=vr, uart_manager=self.uart,
                           grip=intent.get("grip") == 1, pos_cfg=self.opts["position"],
                           pred_cfg=self.opts["prediction"], trim_cfg=self.opts["imu_trim"])

    def request_point(self, xyz, label):
        if not (isinstance(xyz, list) and len(xyz) == 3):
            return False, {"why": "distanza non misurata"}
        try:
            return self.g.request_point([float(v) for v in xyz], label)
        except Exception as e:               # noqa: BLE001 - a planning error must not stop the server
            return False, {"why": f"errore: {e}"}

    def pop_goal_done(self):
        return self.g.pop_goal_done()

    def set_horizon_lock(self, on: bool) -> bool:
        return self.g.set_horizon_lock(on)

    def imu_up_tool(self):
        """World vertical seen in the tool frame by the wrist IMU (mounting from imu_ee_mount.json), or None."""
        tm = self.core.telemetry
        if not tm.get("imu_valid"):
            return None
        m, _ = self.g._imu_mount()
        if m is None:
            return None
        r = self.g._quat_to_mat(tm["imu_q_w"], tm["imu_q_x"], tm["imu_q_y"], tm["imu_q_z"])
        return [float(v) for v in m @ (r.T @ self.g.np.array([0.0, 0.0, 1.0]))]

    def headzero_external(self):
        self.g.notify_headzero_external()


class _UartBridge:
    """uart_manager.send_uart_command() of the old Pi, for the gimbal's HEADZERO (sent from an asyncio task)."""

    def __init__(self, core: Core, lock: threading.Lock):
        self.core, self.lock = core, lock

    async def send_uart_command(self, cmd: str, timeout_s: float = 1.0):
        loop = asyncio.get_running_loop()
        fut = loop.create_future()

        def done(ok, resp):                   # control thread
            loop.call_soon_threadsafe(lambda: fut.done() or fut.set_result((ok, resp)))
        with self.lock:
            self.core.uart_send(cmd, done)
        try:
            return await asyncio.wait_for(fut, timeout_s + 0.5)
        except asyncio.TimeoutError:
            return False, "TIMEOUT"


WHEP_PATHS = ("cam0", "cam1")
STATIC_TYPES = {".html": "text/html; charset=utf-8", ".js": "text/javascript; charset=utf-8",
                ".css": "text/css; charset=utf-8", ".json": "application/json", ".svg": "image/svg+xml"}


def https_server(cfg: dict, cams: Cameras, lock: threading.Lock):
    """The two HTTPS calls of the Quest app (the old https_server.py on 443): GET /api/controller-mappings
    (button bindings; the app falls back to its built-in copy) and POST /api/refocus-cameras; every other GET
    serves the dashboard's static files (https.dashboard)."""
    import socket
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
    h_cfg, ws_cfg = cfg.get("https", {}), cfg.get("ws", {})

    class Handler(BaseHTTPRequestHandler):
        def _reply(self, code: int, body: bytes):
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split("?")[0]
            if path == "/ws" and "upgrade" in (self.headers.get("Connection") or "").lower():
                return self.tunnel_ws()
            if path == "/api/controller-mappings" and h_cfg.get("controller_mappings"):
                try:
                    with open(h_cfg["controller_mappings"], "rb") as f:
                        return self._reply(200, f.read())
                except OSError:
                    pass
            root = h_cfg.get("dashboard")
            if root and not path.startswith("/api/"):
                rel = "index.html" if path in ("/", "") else path.lstrip("/")
                full = os.path.realpath(os.path.join(root, rel))
                if full.startswith(os.path.realpath(root) + os.sep) and os.path.isfile(full):
                    with open(full, "rb") as f:
                        body = f.read()
                    self.send_response(200)
                    self.send_header("Content-Type", STATIC_TYPES.get(os.path.splitext(full)[1], "application/octet-stream"))
                    self.send_header("Cache-Control", "no-cache")
                    self.end_headers()
                    self.wfile.write(body)
                    return
            self._reply(404, b'{"ok":false}')

        def tunnel_ws(self):
            """The dashboard's WebSocket on this same port and origin (wss://robot/ws): one certificate to
            accept instead of two (443 and 8557, the phone's WebView included). The handshake and everything
            after it are copied byte by byte to the controller's local listener (ws.local_port, loopback);
            the apps keep their direct wss://robot:8557."""
            port = int(ws_cfg.get("local_port") or 0)
            try:
                up = socket.create_connection(("127.0.0.1", port), timeout=5) if port else None
            except OSError:
                up = None
            if up is None:
                return self._reply(503, b'{"ok":false,"error":"websocket non disponibile"}')
            head = f"GET {self.path} HTTP/1.1\r\n" + "".join(f"{k}: {v}\r\n" for k, v in self.headers.items()) + "\r\n"
            self.close_connection = True
            down = self.connection
            up.settimeout(None)
            down.settimeout(None)

            def pump(src, dst):
                try:
                    while True:
                        data = src.recv(65536)
                        if not data:
                            break
                        dst.sendall(data)
                except OSError:
                    pass
                finally:
                    for s in (src, dst):
                        try:
                            s.shutdown(socket.SHUT_RDWR)
                        except OSError:
                            pass

            try:
                up.sendall(head.encode("latin-1"))
                t = threading.Thread(target=pump, args=(up, down), daemon=True, name="j5mini-ws-tunnel")
                t.start()
                pump(down, up)
                t.join(2)
            finally:
                up.close()

        def do_POST(self):
            path = self.path.split("?")[0]
            if path.startswith("/whep/"):
                return self.whep(path[len("/whep/"):])
            if path != "/api/refocus-cameras":
                return self._reply(404, b'{"ok":false}')
            with lock:
                ok = cams.autofocus()
            if ok:
                self._reply(200, b'{"ok":true,"mode":"af"}')
            else:
                self._reply(429, b'{"ok":false,"error":"cooldown 1s"}')

        def whep(self, cam: str):
            """WebRTC (WHEP) for the dashboard's video test: the SDP offer goes to MediaMTX on the robot
            (https.whep, plain http on localhost) and its answer comes back over this HTTPS port, so an https
            page can use it; the video itself then flows over UDP 8189 between browser and MediaMTX.
            Only the camera paths, nothing else of MediaMTX."""
            import urllib.error
            import urllib.request
            if cam not in WHEP_PATHS:
                return self._reply(404, b'{"ok":false}')
            n = int(self.headers.get("Content-Length") or 0)
            if not 0 < n <= 65536:
                return self._reply(400, b'{"ok":false,"error":"offer"}')
            offer = self.rfile.read(n)
            req = urllib.request.Request(f"{h_cfg.get('whep', 'http://127.0.0.1:8889')}/{cam}/whep", data=offer,
                                         method="POST", headers={"Content-Type": "application/sdp"})
            try:
                with urllib.request.urlopen(req, timeout=5) as r:
                    code, body = r.status, r.read()
            except urllib.error.HTTPError as e:
                code, body = e.code, e.read()
            except OSError as e:
                return self._reply(502, json.dumps({"ok": False, "error": f"MediaMTX non raggiungibile: {e}"}).encode())
            self.send_response(code)
            self.send_header("Content-Type", "application/sdp" if code < 300 else "text/plain")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, fmt, *args):
            pass

    class Http(ThreadingHTTPServer):
        address_family = socket.AF_INET6 if ":" in h_cfg.get("host", "::") else socket.AF_INET

    srv = Http((h_cfg.get("host", "::"), int(h_cfg["port"])), Handler)
    if ws_cfg.get("cert") and ws_cfg.get("key"):
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ctx.load_cert_chain(ws_cfg["cert"], ws_cfg["key"])
        srv.socket = ctx.wrap_socket(srv.socket, server_side=True)
    return srv


def control_thread(core: Core, lock: threading.Lock, stop: threading.Event, period=0.01, sim=None):
    nxt = time.monotonic()
    while not stop.is_set():
        with lock:
            core.step()
            if sim is not None:
                sim.tick()
        nxt += period
        d = nxt - time.monotonic()
        if d > 0:
            time.sleep(d)
        else:
            nxt = time.monotonic()


class BoardMeta(asyncio.DatagramProtocol):
    """Stereo head board (jonny5-stereo-head sender, `--meta-mirror 127.0.0.1:<port>`): copies of the UDP meta
    datagrams it sends to the headset. Parsed here into dashboard messages:
      kCtrlVideoMeta (4): VideoMeta(frame_id u32, eye u8, count u8, infer_us u16) + DetBox(x,y,w,h u16, cls u8,
                           score u8)[count] -> {"type": "npu_detections", ...} (boxes in left-eye 1920x1080 px)
      kCtrlCollimation (13): dx_q4 i16, dy_q4 i16, dist_mm u16, score u8, flags u8 -> {"type": "stereo_collimation", ...}
      kCtrlCollimCalResult (17): dx_inf_q4 i16, f_px_q2 u16, distance_mm u16, dx_q4 i16, points u8, status u8
                           -> {"type": "stereo_calibration", ...} (answer to the dashboard's stereo_calibrate)
    Little-endian, header = magic 0x4A35 u16, type u8, version u8, ts_us u32 (sender/src/protocol.hpp)."""
    MAGIC, T_META, T_COLLIM, T_CAL = 0x4A35, 4, 13, 17
    CAL_STATUS = ("offset", "full", "reset", "no_lock", "fit_rejected", "bad_distance")

    COCO_IT = ["persona", "bicicletta", "auto", "moto", "aereo", "autobus", "treno", "camion", "barca", "semaforo",
               "idrante", "stop", "parchimetro", "panchina", "uccello", "gatto", "cane", "cavallo", "pecora", "mucca",
               "elefante", "orso", "zebra", "giraffa", "zaino", "ombrello", "borsa", "cravatta", "valigia", "frisbee",
               "sci", "snowboard", "palla", "aquilone", "mazza da baseball", "guanto da baseball", "skateboard",
               "tavola da surf", "racchetta", "bottiglia", "bicchiere", "tazza", "forchetta", "coltello", "cucchiaio",
               "ciotola", "banana", "mela", "panino", "arancia", "broccolo", "carota", "hot dog", "pizza", "ciambella",
               "torta", "sedia", "divano", "pianta", "letto", "tavolo", "water", "tv", "portatile", "mouse",
               "telecomando", "tastiera", "cellulare", "microonde", "forno", "tostapane", "lavandino", "frigorifero",
               "libro", "orologio", "vaso", "forbici", "orsacchiotto", "asciugacapelli", "spazzolino"]

    def __init__(self, srv: "Server", loop: asyncio.AbstractEventLoop):
        self.srv, self.loop, self.n, self.last_det = srv, loop, 0, 0.0
        self.boxes, self.boxes_at, self.boxes_tcap, self.infer_us = [], 0.0, 0.0, 0
        self.age_ms = None                     # last detection: capture -> here (the sender stamps the frame PTS)
        self.collim, self.collim_at = None, 0.0
        srv.board = self

    def vision_result(self, req: dict) -> dict:
        """Answer to the dashboard's / headset's `vision_request` built from the board's own data: the YOLO boxes
        (last 2 s) and the collimator (distance under the crosshair). Same shape as vision_service.py's result,
        without OCR ("texts" empty)."""
        now = time.monotonic()
        boxes = self.boxes if now - self.boxes_at < 2.0 else []
        col = self.collim if self.collim and now - self.collim_at < 2.0 else None
        W, H = 1920, 1080
        objs = []
        for b in boxes:
            name = self.COCO_IT[b["cls"]] if 0 <= b["cls"] < len(self.COCO_IT) else f"classe {b['cls']}"
            objs.append({"box": [b["x"], b["y"], b["x"] + b["w"], b["y"] + b["h"]], "label_it": name,
                         "label": name, "conf": b["score"], "cls": b["cls"]})
        # target = the box that contains the crosshair (image centre), else the nearest one to it
        cx, cy = W / 2, H / 2
        def dist2(o):
            x0, y0, x1, y1 = o["box"]
            if x0 <= cx <= x1 and y0 <= cy <= y1:
                return 0.0
            dx = max(x0 - cx, 0, cx - x1); dy = max(y0 - cy, 0, cy - y1)
            return dx * dx + dy * dy
        target = None
        if objs:
            t = min(objs, key=dist2)
            target = dict(t, target=True, center=[(t["box"][0] + t["box"][2]) / 2, (t["box"][1] + t["box"][3]) / 2])
        elif col and col.get("lock"):
            target = {"label_it": "oggetto", "label": "object", "center": [cx, cy]}
        if target is not None and col and col.get("lock"):
            target.update({"dist_mm": col.get("dist_mm"), "dx": col.get("dx"), "dy": col.get("dy")})
        ok = bool(objs) or bool(col and col.get("lock"))
        return {"type": "vision_result", "action": req.get("action", "analyze"), "req": req.get("req"), "ok": ok,
                "why": None if ok else "nessun oggetto riconosciuto (NPU) e collimatore non agganciato",
                "objects": objs, "target": target, "texts": [], "zoom": 1.0, "infer_us": self.infer_us,
                "src": "board", "ts": time.time()}

    def datagram_received(self, data: bytes, addr):
        if len(data) < 8 or struct.unpack_from("<H", data, 0)[0] != self.MAGIC:
            return
        typ = data[2]
        self.n += 1
        if self.n == 1:
            self.srv.core.log(f"[j5mini] stereo head meta from {addr[0]}:{addr[1]}")
        if typ == self.T_META and len(data) >= 16:
            frame_id, eye, count, infer_us = struct.unpack_from("<IBBH", data, 8)
            # capture time: the header's ts_us is the frame's PTS (CLOCK_MONOTONIC us, same clock as ours: the
            # sender runs on this board); an older sender stamps the send time -> + the inference only
            now = time.monotonic()
            age = (int(now * 1e6) - struct.unpack_from("<I", data, 4)[0]) & 0xFFFFFFFF
            t_cap = now - age / 1e6 if age < 2000000 else now - infer_us / 1e6 - 0.05
            boxes = []
            for i in range(min(count, 32)):
                o = 16 + i * 10
                if o + 10 > len(data):
                    break
                x, y, w, h, cls, score = struct.unpack_from("<HHHHBB", data, o)
                boxes.append({"x": x, "y": y, "w": w, "h": h, "cls": cls, "score": round(score / 255.0, 2)})
            self.age_ms = int((now - t_cap) * 1000)
            if boxes:
                self.boxes, self.boxes_at, self.boxes_tcap, self.infer_us = boxes, now, t_cap, infer_us
            if getattr(self.srv.core, 'track', None):
                with self.srv.lock:
                    self.srv.core.track_boxes(boxes, t_cap)
            if not boxes and time.monotonic() - self.last_det < 1.0:
                return            # the board's empty heartbeat: nothing new to draw
            self.last_det = time.monotonic()
            asyncio.ensure_future(self.srv.broadcast({"type": "npu_detections", "frame_id": frame_id, "eye": eye,
                                                       "infer_us": infer_us, "w": 1920, "h": 1080, "boxes": boxes,
                                                       "age_ms": self.age_ms}))
        elif typ == self.T_COLLIM and len(data) >= 16:
            dx, dy, dist, score, flags = struct.unpack_from("<hhHBB", data, 8)
            self.collim = {"type": "stereo_collimation", "lock": bool(flags & 1), "dx": dx / 16.0, "dy": dy / 16.0,
                           "score": round(score / 255.0, 2), "dist_mm": dist if dist else None, "src": "board"}
            self.collim_at = time.monotonic()
            asyncio.ensure_future(self.srv.broadcast(self.collim))
        elif typ == self.T_CAL and len(data) >= 18:
            dxi, fpx, mm, dx, n, st = struct.unpack_from("<hHHhBB", data, 8)
            status = self.CAL_STATUS[st] if st < len(self.CAL_STATUS) else str(st)
            msg = {"type": "stereo_calibration", "ok": status in ("offset", "full", "reset", "fit_rejected"),
                   "status": status, "dx_inf": dxi / 16.0, "f_px": fpx / 4.0, "distance_mm": mm,
                   "dx": dx / 16.0 if status not in ("no_lock", "reset", "bad_distance") else None, "points": n,
                   "src": "board"}
            self.srv.core.log(f"[j5mini] collimator calibration {mm} mm: {status}, dx_inf {msg['dx_inf']:.2f}, "
                              f"f_px {msg['f_px']:.1f}, {n} point(s)")
            asyncio.ensure_future(self.srv.broadcast(msg))


async def serve(core: Core, lock: threading.Lock, cfg: dict, cams: Cameras):
    if websockets is None:
        raise SystemExit("websockets non installato")
    ws_cfg = cfg.get("ws", {})
    ssl_ctx = None
    if ws_cfg.get("cert") and ws_cfg.get("key"):
        ssl_ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ssl_ctx.load_cert_chain(ws_cfg["cert"], ws_cfg["key"])
    srv = Server(core, lock, cfg, cams)
    meta_port = int(cfg.get("board", {}).get("meta_port", 5603))   # stereo head meta mirror (0 = off)
    if meta_port:
        loop = asyncio.get_running_loop()
        await loop.create_datagram_endpoint(lambda: BoardMeta(srv, loop), local_addr=("127.0.0.1", meta_port))
        core.log(f"[j5mini] stereo head meta listener on udp/127.0.0.1:{meta_port}")
    local = int(ws_cfg.get("local_port") or 0)        # plain ws on loopback for the HTTPS server's /ws tunnel
    async with websockets.serve(srv.handler, ws_cfg.get("host", "0.0.0.0"), ws_cfg.get("port", 8557), ssl=ssl_ctx,
                                ping_interval=5, ping_timeout=5):
        core.log(f"[j5mini] {'wss' if ssl_ctx else 'ws'}://0.0.0.0:{ws_cfg.get('port', 8557)} ready")
        if local:
            async with websockets.serve(srv.handler, "127.0.0.1", local, ping_interval=5, ping_timeout=5):
                core.log(f"[j5mini] dashboard WebSocket tunnel: /ws on the HTTPS port -> 127.0.0.1:{local}")
                await srv.broadcast_loop()
        else:
            await srv.broadcast_loop()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None, help="default: j5mini.json (with --sim: j5mini.example.json)")
    ap.add_argument("--sim", action="store_true", help="simulated firmware on this PC instead of SPI/UART")
    ap.add_argument("--http-port", type=int, default=8558, help="--sim: dashboard port (default 8558)")
    a = ap.parse_args()
    path = a.config or str(HERE / ("j5mini.example.json" if a.sim else "j5mini.json"))
    cfg = json.load(open(path, encoding="utf-8"))
    cfg["_path"] = None if (a.sim and a.config is None) else path   # --sim alone never rewrites the example
    base = os.path.dirname(os.path.abspath(path))              # relative paths in the config: from its folder
    for sect, key in (("ws", "cert"), ("ws", "key"), ("https", "controller_mappings"), ("https", "dashboard")):
        if cfg.get(sect, {}).get(key):
            cfg[sect][key] = os.path.join(base, cfg[sect][key])
    sim = None
    if a.sim:
        # plain ws/http on this PC (no certificate), dashboard on --http-port, no clock setting, no telemetry file
        cfg["_sim"] = True
        cfg["ws"] = {"host": cfg.get("ws", {}).get("host", "0.0.0.0"), "port": 8557,
                     "local_port": cfg.get("ws", {}).get("local_port", 8556)}
        cfg["https"] = {**cfg.get("https", {}), "host": "0.0.0.0", "port": a.http_port}
        cfg["set_clock"] = False
        cfg.pop("telemetry_file", None)
        sim = SimRobot(int(cfg.get("spi", {}).get("transfer_len", 64)))
        spi = uart = sim
        print(f"[j5mini] simulated robot: dashboard on http://localhost:{a.http_port}")
    else:
        spi = SpidevBackend(cfg["spi"]["device"], int(cfg["spi"].get("speed_hz", 1000000)))
        uart = SerialBackend(cfg["uart"]["device"], int(cfg["uart"].get("baud", 115200)))
    core = Core(cfg, spi, uart)
    cams = Cameras(board_ctrl_port=cfg.get("board", {}).get("ctrl_port"))
    lock, stop = threading.Lock(), threading.Event()
    threading.Thread(target=control_thread, args=(core, lock, stop), kwargs={"sim": sim}, daemon=True,
                     name="j5mini-100hz").start()
    if cfg.get("https", {}).get("port"):
        threading.Thread(target=https_server(cfg, cams, lock).serve_forever, daemon=True, name="j5mini-https").start()
    try:
        asyncio.run(serve(core, lock, cfg, cams))
    finally:
        stop.set()


if __name__ == "__main__":
    main()
