#!/usr/bin/env python3
"""j5mini — minimal arm controller for JONNY5 (Raspberry Pi or Luckfox Aura).

One process replaces ws-teleop + the SPI bridge (or the ROS 2 container) for teleoperation:

  Quest app / web viewer  --wss :8557 JSON (same messages as ws-teleop)-->  j5mini
  j5mini --SPI (J5 frames, 100 Hz, 64 B or 128 B)--> STM32      (setpoints, telemetry, STATUS)
  j5mini --UART (ASCII lines, #seq)-------------> STM32      (ENABLE/SAFE/HOME/..., config push)

What it does (and nothing else):
  - SPI master at 100 Hz: the newest intent as a J5VR payload; after 150 ms without intents
    a neutral payload (same mode and heartbeat, no input: deadman released). One STATUS
    request every 20 transfers (5 Hz) for the diagnostics (deadman/armed/E-STOP bits).
  - UART owner: commands from the clients with #seq replies; on BOOT_READY (or 1.5 s after
    start) pushes the configuration lines from the config file, in order, with retries;
    STATUS? every 2 s for the robot state; ESTOP / SETPOSE_DONE lines forwarded.
  - On entering HEAD/HYBRID/ASSIST (mode 3/4/5) re-sends SET_VR_PARAMS (2.5 s throttle), as ws-teleop.
  - Telemetry to the clients at 20 Hz: robot_state, servo_deg_*, IMU quaternion, deadman/armed/E-STOP.
  - Optional clock set from the headset (time_sync), the robot has no RTC and no internet.
  - Camera controls of the Quest app (zoom, brightness, wide/tele, focus, autofocus) forwarded to
    raspberry/tools/cam_stream.py over UDP, as ws-teleop; get_settings (offsets/dirs); HTTPS on 443
    for the app's GET /api/controller-mappings and POST /api/refocus-cameras.

Not here (on purpose): ASSIST gimbal targets, IK/FK, dashboard pages, vision, collimator,
ROS 2, captive portal, demo/self-test. Mode 5 therefore holds B/S/G.

  python3 j5mini.py --config j5mini.json             # real hardware (spidev + pyserial)
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import ssl
import struct
import threading
import time
from typing import Callable

# ------------------------------------------------------------------ J5 protocol (firmware layout)

J5_TELEMETRY, J5_TEST_ECHO, J5_STATUS, J5_J5VR, J5_J5IK = 0x01, 0x02, 0x03, 0x04, 0x05
DIAG_BITS = ("deadman", "input_active", "armed", "freeze", "guard", "estop_active", "estop_bypass")
SERVO_KEYS = ("servo_deg_B", "servo_deg_S", "servo_deg_G", "servo_deg_Y", "servo_deg_P", "servo_deg_R")


def build_frame(ftype: int, seq: int, payload: bytes, length: int) -> bytes:
    p = bytes(payload).ljust(54, b"\0")[:54]
    f = b"J5" + bytes([1, ftype]) + struct.pack(">H", seq & 0xFFFF) + bytes([64, 0]) + p + b"\0\0"
    return f.ljust(length, b"\0")


def find_frame(rx: bytes):
    """Canonical 64-byte frame in a transfer (rule of the Pi's spi_transport_mode.py)."""
    def ok(o):
        f = rx[o:o + 64]
        return (len(f) == 64 and f[:2] == b"J5" and f[2] == 1 and f[6] == 64 and f[7] == 0
                and f[3] in (1, 2, 3, 4, 5))
    if len(rx) <= 64:
        return rx if ok(0) else None
    other = None
    for o in range(0, len(rx) - 63):
        if ok(o):
            if o in (0, 64):
                return rx[o:o + 64]
            other = other if other is not None else rx[o:o + 64]
    return other


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
        self.boot_lines = list(cfg.get("boot_commands", []))
        self.vr_params_line = next((l for l in self.boot_lines if l.startswith("SET_VR_PARAMS")), None)
        self.t0 = clock()
        # setpoint
        self.payload: bytes | None = None
        self.payload_t = -1e9
        self.last_mode, self.last_hb = 0, 0
        self.tx_seq, self.transfers = 0, 0
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
        self.stats = dict(spi_tx=0, spi_rx_ok=0, spi_rx_bad=0, neutral=0, uart_tx=0, uart_rx=0, uart_timeouts=0)

    # ---------------- inputs ----------------
    def set_intent(self, d: dict) -> str | None:
        p, err = intent_to_payload(d)
        if err:
            return err
        mode = p[0]
        if mode in (3, 4, 5) and mode != self.last_mode:
            self.resend_vr_params()
        self.payload, self.payload_t = p, self.clock()
        self.last_mode, self.last_hb = mode, struct.unpack_from(">H", p, 11)[0]
        return None

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
        self.events.append({"type": "uart_unsolicited", "line": line})
        if line == "BOOT_READY":
            self.log("[j5mini] STM32 BOOT_READY: pushing configuration")
            self._start_boot_push()

    def _uart_done(self, cmd, ok, resp, cb):
        if resp.startswith("STATUS:"):
            self.robot_state = resp[7:].strip()
        if self.boot_queue and cmd == self.boot_queue[0]:
            if not ok and cmd.startswith("SET_VR_PARAMS") and self.boot_retries < 2:
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

    def _start_boot_push(self):
        self.boot_queue = list(self.boot_lines)
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
            else:
                p = neutral_payload(self.last_mode, self.last_hb)
                self.stats["neutral"] += 1
            frame = build_frame(J5_J5VR, self.tx_seq, p, self.len)
        rx = self.spi.transfer(frame)
        self.stats["spi_tx"] += 1
        f = find_frame(rx)
        if f is None:
            self.stats["spi_rx_bad"] += 1
            return
        self.stats["spi_rx_ok"] += 1
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

    # ---------------- step ----------------
    def step(self):
        """Call every 10 ms."""
        self._spi_tick()
        self._uart_poll()
        now = self.clock()
        if not self.boot_done and now - self.t0 >= self.BOOT_FALLBACK_S:
            self.log("[j5mini] no BOOT_READY seen: pushing configuration anyway")
            self._start_boot_push()
        if now >= self.next_state_poll:
            self.next_state_poll = now + self.STATE_POLL_S
            self.uart_send("STATUS?")

    def telemetry_message(self) -> dict:
        intent_age = self.clock() - self.payload_t
        return {"type": "telemetry", "robot_state": self.robot_state, **self.telemetry,
                "deadman_active": self.diag.get("deadman"), "vr_armed": self.diag.get("armed"),
                "estop_active": self.diag.get("estop_active"), "estop_bypass": self.diag.get("estop_bypass"),
                "teleop_mode": self.last_mode if intent_age < 1.0 else -1,
                "intent_age_ms": int(intent_age * 1000) if self.payload is not None else None,
                "link": dict(self.stats)}


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


# ------------------------------------------------------------------ WebSocket server (ws-teleop subset)

class Server:
    # forwarded as they are to the other clients, like ws-teleop; the optional services on the robot
    # (raspberry/tools/stereo_collimator.py, vision_service.py) are clients of this server too
    RELAY = ("cameras_refocus_triggered", "vr_session_state", "vr_zoom_command",
             "stereo_collimation", "stereo_calibrate", "vision_request", "vision_result")

    def __init__(self, core: Core, lock: threading.Lock, cfg: dict, cams: Cameras | None = None):
        self.core, self.lock, self.cfg = core, lock, cfg
        self.cams = cams or Cameras()
        self.clients = set()
        self.ignored = set()

    async def handler(self, ws):
        self.clients.add(ws)
        try:
            async for raw in ws:
                try:
                    msg = json.loads(raw)
                except ValueError:
                    continue
                await self.on_message(ws, msg)
        finally:
            self.clients.discard(ws)

    async def on_message(self, ws, msg: dict):
        typ = msg.get("type")
        if typ is None and "mode" in msg:
            with self.lock:
                err = self.core.set_intent(msg)
            if err:
                await ws.send(json.dumps({"type": "error", "error": err}))
            return
        if typ == "uart" and isinstance(msg.get("cmd"), str):
            with self.lock:
                self.core.uart_send(msg["cmd"].strip())
        elif typ == "set_vr_mode":
            if msg.get("mode") in (3, 4, 5):
                with self.lock:
                    self.core.resend_vr_params()
            await ws.send(json.dumps({"type": "ack", "set_vr_mode": msg.get("mode")}))
        elif typ == "time_sync":
            self.time_sync(msg.get("epoch_ms"))
        elif typ == "get_settings":
            await ws.send(json.dumps({"type": "settings", **self.cfg.get("settings", {})}))
        elif typ in self.RELAY:
            data = json.dumps(msg)
            await asyncio.gather(*(c.send(data) for c in list(self.clients) if c is not ws), return_exceptions=True)
            if typ == "vision_result" and msg.get("aim"):
                # "point" moved the arm through ws-teleop's ASSIST gimbal, which j5mini does not have
                label = (msg.get("target") or {}).get("label_it")
                data = json.dumps({"type": "vision_point", "ok": False, "label": label,
                                   "why": "puntamento non disponibile (stack minimale)"})
                await asyncio.gather(*(c.send(data) for c in list(self.clients)), return_exceptions=True)
        elif self.cams.on_message(msg):
            pass
        elif typ not in self.ignored:
            self.ignored.add(typ)
            self.core.log(f"[j5mini] message type {typ!r} not handled by the minimal controller (ignored)")

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
        while True:
            await asyncio.sleep(0.05)
            with self.lock:
                msgs = [{**self.core.telemetry_message(), **self.cams.telemetry()}] + self.core.events
                self.core.events = []
            self.write_telemetry_file(msgs[0])
            if self.clients:
                for m in msgs:
                    data = json.dumps(m)
                    await asyncio.gather(*(c.send(data) for c in list(self.clients)), return_exceptions=True)


def https_server(cfg: dict, cams: Cameras, lock: threading.Lock):
    """The two HTTPS calls of the Quest app (the old https_server.py on 443): GET /api/controller-mappings
    (button bindings; the app falls back to its built-in copy) and POST /api/refocus-cameras."""
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
            if self.path.split("?")[0] == "/api/controller-mappings" and h_cfg.get("controller_mappings"):
                try:
                    with open(h_cfg["controller_mappings"], "rb") as f:
                        return self._reply(200, f.read())
                except OSError:
                    pass
            self._reply(404, b'{"ok":false}')

        def do_POST(self):
            if self.path.split("?")[0] != "/api/refocus-cameras":
                return self._reply(404, b'{"ok":false}')
            with lock:
                ok = cams.autofocus()
            if ok:
                self._reply(200, b'{"ok":true,"mode":"af"}')
            else:
                self._reply(429, b'{"ok":false,"error":"cooldown 1s"}')

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


def control_thread(core: Core, lock: threading.Lock, stop: threading.Event, period=0.01):
    nxt = time.monotonic()
    while not stop.is_set():
        with lock:
            core.step()
        nxt += period
        d = nxt - time.monotonic()
        if d > 0:
            time.sleep(d)
        else:
            nxt = time.monotonic()


async def serve(core: Core, lock: threading.Lock, cfg: dict, cams: Cameras):
    import websockets
    ws_cfg = cfg.get("ws", {})
    ssl_ctx = None
    if ws_cfg.get("cert") and ws_cfg.get("key"):
        ssl_ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ssl_ctx.load_cert_chain(ws_cfg["cert"], ws_cfg["key"])
    srv = Server(core, lock, cfg, cams)
    async with websockets.serve(srv.handler, ws_cfg.get("host", "0.0.0.0"), ws_cfg.get("port", 8557), ssl=ssl_ctx):
        core.log(f"[j5mini] {'wss' if ssl_ctx else 'ws'}://0.0.0.0:{ws_cfg.get('port', 8557)} ready")
        await srv.broadcast_loop()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default="j5mini.json")
    a = ap.parse_args()
    cfg = json.load(open(a.config, encoding="utf-8"))
    base = os.path.dirname(os.path.abspath(a.config))          # relative paths in the config: from its folder
    for sect, key in (("ws", "cert"), ("ws", "key"), ("https", "controller_mappings")):
        if cfg.get(sect, {}).get(key):
            cfg[sect][key] = os.path.join(base, cfg[sect][key])
    spi = SpidevBackend(cfg["spi"]["device"], int(cfg["spi"].get("speed_hz", 1000000)))
    uart = SerialBackend(cfg["uart"]["device"], int(cfg["uart"].get("baud", 115200)))
    core = Core(cfg, spi, uart)
    cams = Cameras()
    lock, stop = threading.Lock(), threading.Event()
    threading.Thread(target=control_thread, args=(core, lock, stop), daemon=True, name="j5mini-100hz").start()
    if cfg.get("https", {}).get("port"):
        threading.Thread(target=https_server(cfg, cams, lock).serve_forever, daemon=True, name="j5mini-https").start()
    try:
        asyncio.run(serve(core, lock, cfg, cams))
    finally:
        stop.set()


if __name__ == "__main__":
    main()
