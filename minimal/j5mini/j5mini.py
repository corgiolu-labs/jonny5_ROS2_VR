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
  - Demo: the pose sequence saved in j5mini.json (settings.demo), one SETPOSE per step, the next at SETPOSE_DONE,
    round and round until the dashboard stops it, the robot leaves IDLE (STOP / SAFE / E-STOP) or a step fails.
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
        self.last_pose: list[int] | None = None        # servo angles seen in telemetry (kept across a reset)
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
        if self.demo and line.startswith("SETPOSE_DONE") and self.demo["moving"]:
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
        self.events.append({"type": "uart_response", "cmd": cmd, "ok": ok, "response": resp})

    def _start_boot_push(self, booted: bool = False):
        self.boot_queue = list(self.boot_lines)
        if booted:                                    # the firmware just (re)started: tell it where the arm is
            self.boot_queue.append(init_pose_line(self.cfg_for_init, self.last_pose))
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
        self._demo_tick(now)
        if not self.boot_done and now - self.t0 >= self.BOOT_FALLBACK_S:
            self.log("[j5mini] no BOOT_READY seen: pushing configuration anyway")
            self._start_boot_push()
        if now >= self.next_state_poll:
            self.next_state_poll = now + self.STATE_POLL_S
            self.uart_send("STATUS?")

    # ---------------- demo: the saved pose sequence, looped (dashboard "Demo") ----------------
    DEMO_STEP_TIMEOUT_S = 15.0

    def demo_start(self, lines: list[str]) -> tuple[bool, str]:
        """SETPOSE lines one at a time, the next one at SETPOSE_DONE, round and round until demo_stop(), the robot
        leaves IDLE (STOP / SAFE / E-STOP), a refused line or a step without SETPOSE_DONE for 15 s."""
        if not lines:
            return False, "nessun passo"
        if self.robot_state != "IDLE":
            return False, f"robot in {self.robot_state}: serve IDLE"
        self.demo = {"lines": list(lines), "i": -1, "deadline": 0.0, "moving": False}
        self._demo_next()
        return True, ""

    def demo_stop(self, reason: str, hold: bool = False):
        """hold: stop where the arm is (a 120 ms trajectory to the measured angles replaces the running one)."""
        d = self.demo
        if not d:
            return
        self.demo = None
        if hold and d["moving"] and all(k in self.telemetry for k in SERVO_KEYS):
            self.uart_send("SETPOSE_T_HR " + " ".join(str(int(round(self.telemetry[k] * 10))) for k in SERVO_KEYS)
                           + " 120 RTR3")
        self.events.append({"type": "demo", "on": False, "reason": reason})
        self.log(f"[j5mini] demo stopped ({reason})")

    def demo_status(self) -> dict | None:
        d = self.demo
        return {"step": d["i"] + 1, "n": len(d["lines"])} if d else None

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

    def _demo_tick(self, now: float):
        d = self.demo
        if not d:
            return
        if self.robot_state != "IDLE":
            self.demo_stop(f"robot in {self.robot_state}")
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
                "demo": self.demo_status(), "link": dict(self.stats)}


# ------------------------------------------------------------------ settings (dashboard)

JOINTS = ("B", "S", "G", "Y", "P", "R")


POSE_DEFAULTS = {            # the robot's j5_settings.json (virtual degrees, 90 = HOME)
    "park": [90, 50, 150, 90, 130, 90],
    "vr": [90, 110, 90, 90, 70, 90],
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
            "demo": [dict(x) for x in st.get("demo", DEMO_DEFAULT)]}


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

    def __init__(self, state_path: str = STATE_PATH, clock: Callable[[], float] = time.monotonic, sendto=None):
        self.state_path, self.clock = state_path, clock
        self._sock = None
        self._sendto = sendto or self._udp
        self.zoom = self.ev = self.mode = None
        self.zoom_sent = self.ev_sent = self.mode_sent = -1e9
        self.vr_zoom = [None, None]
        self.last_af = -1e9

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

    def autofocus(self) -> bool:
        """One-shot AF on both cameras (POST /api/refocus-cameras); False within 1 s of the last one."""
        now = self.clock()
        if now - self.last_af < 1.0:
            return False
        self.last_af = now
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
    def __init__(self, device: str, speed_hz: int):
        import spidev
        bus, dev = device.replace("/dev/spidev", "").split(".")
        self.s = spidev.SpiDev()
        self.s.open(int(bus), int(dev))
        self.s.mode, self.s.bits_per_word, self.s.max_speed_hz = 0, 8, speed_hz

    def transfer(self, data: bytes) -> bytes:
        return bytes(self.s.xfer2(list(data)))


class SerialBackend:
    def __init__(self, device: str, baud: int):
        import serial
        self.s = serial.Serial(device, baud, timeout=0)

    def write(self, data: bytes):
        self.s.write(data)

    def read(self) -> bytes:
        return self.s.read(4096)


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
             "stereo_collimation", "stereo_calibrate", "vision_request", "vision_result")

    def __init__(self, core: Core, lock: threading.Lock, cfg: dict, cams: Cameras | None = None, kin_path=KIN_PATH):
        self.core, self.lock, self.cfg = core, lock, cfg
        self.cams = cams or Cameras()
        self.clients = set()
        self.ignored = set()
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
                if msg.get("on"):
                    ok, err = self.core.demo_start(demo_lines(self.cfg))
                else:
                    self.core.demo_stop("dashboard", hold=True)
                    ok, err = True, ""
            await ws.send(json.dumps({"type": "demo", "ok": ok, "on": bool(msg.get("on")) and ok, "error": err or None}))
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


async def serve(core: Core, lock: threading.Lock, cfg: dict, cams: Cameras):
    if websockets is None:
        raise SystemExit("websockets non installato")
    ws_cfg = cfg.get("ws", {})
    ssl_ctx = None
    if ws_cfg.get("cert") and ws_cfg.get("key"):
        ssl_ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ssl_ctx.load_cert_chain(ws_cfg["cert"], ws_cfg["key"])
    srv = Server(core, lock, cfg, cams)
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
    cams = Cameras()
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
