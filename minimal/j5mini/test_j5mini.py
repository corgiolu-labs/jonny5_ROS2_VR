"""j5mini against the real firmware logic (firmware/stm32/tools/fw_sim), no hardware.

    python -m pytest minimal/j5mini -q
"""
import asyncio
import json
import pathlib
import struct
import sys
import threading
import time

import pytest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1] / "firmware/stm32/tools/fw_sim"))
import fw_sim  # noqa: E402
import j5mini  # noqa: E402

CFG = json.loads((HERE / "j5mini.example.json").read_text(encoding="utf-8"))
GRIP = 0x0002


@pytest.fixture(scope="session")
def lib():
    """Production firmware build: 64-byte SPI transfers."""
    return fw_sim.build(padded128=False, out=fw_sim.BUILD / ("fw_sim_j5mini64" + fw_sim.LIB.suffix))


@pytest.fixture(scope="session")
def lib128():
    return fw_sim.build(padded128=True, out=fw_sim.BUILD / ("fw_sim_j5mini" + fw_sim.LIB.suffix))


class SimSpi:
    def __init__(self, sim):
        self.sim = sim

    def transfer(self, data):
        return self.sim.spi(data)


class SimUart:
    def __init__(self, sim):
        self.sim = sim

    def write(self, data):
        self.sim.lib.sim_uart_write(data)

    def read(self):
        return self.sim.uart_read().encode()


class Rig:
    """j5mini core + simulated STM32, stepped together every 10 ms."""

    def __init__(self, lib, cfg=CFG, transfer_len=None):
        cfg = json.loads(json.dumps(cfg))
        if transfer_len:
            cfg["spi"]["transfer_len"] = transfer_len
        self.sim = fw_sim.Sim(lib)
        self.logs = []
        self.core = j5mini.Core(cfg, SimSpi(self.sim), SimUart(self.sim), clock=lambda: self.sim.now_ms() / 1000.0,
                                log=self.logs.append)
        self.hb = 0

    def run(self, ms, intent=None):
        for _ in range(ms // 10):
            if intent is not None:
                self.hb += 1
                assert self.core.set_intent(dict(intent, heartbeat=self.hb)) is None
            self.core.step()
            self.sim.run(10)

    def responses(self):
        out = [e for e in self.core.events if e["type"] == "uart_response"]
        return out

    def unsolicited(self):
        return [e["line"] for e in self.core.events if e["type"] == "uart_unsolicited"]


def intent(mode=2, **kw):
    d = dict(mode=mode, joy_x=0, joy_y=0, pitch=0, yaw=0, intensity=0, grip=0, buttons_left=0, buttons_right=0)
    d.update(kw)
    return d


def test_boot_ready_pushes_the_pi_config(lib):
    r = Rig(lib)
    r.run(1200)                                            # first STATUS? poll at 1.0 s
    assert "BOOT_READY" in r.unsolicited()
    cmds = [e["cmd"] for e in r.responses()]
    for line in CFG["boot_commands"]:
        assert line in cmds, line
    assert all(e["ok"] for e in r.responses() if e["cmd"] in CFG["boot_commands"])
    assert r.core.robot_state == "SAFE"
    assert r.sim.command("STATUS?") == ["STATUS:SAFE"]


def test_teleop_manual_and_neutral_after_silence(lib):
    r = Rig(lib)
    r.run(1000)
    r.run(300, intent())                                   # advancing heartbeat -> firmware arms IDLE
    assert r.sim.state() == "IDLE"
    seq = r.core.uart_send("HOME")
    r.run(4000, intent())
    assert "SETPOSE_DONE" in " ".join(r.unsolicited())
    offsets = [int(v) for v in CFG["boot_commands"][0].split()[1:]]
    assert [round(a) for a in r.sim.angles()] == offsets   # HOME = the offsets pushed at boot
    yaw0 = r.sim.angles()[3]
    r.run(500, intent(joy_x=30000, buttons_left=GRIP, buttons_right=GRIP))
    assert r.sim.angles()[3] > yaw0 + 5
    assert r.core.diag.get("deadman") is True              # from the 5 Hz STATUS requests
    n0 = r.core.stats["neutral"]
    r.run(500)                                             # headset silent
    assert r.core.stats["neutral"] - n0 >= 30              # neutral after 150 ms
    assert r.core.diag.get("deadman") is False
    pulses, _ = r.sim.pwm()
    assert pulses == [0] * 6                               # deadman released -> outputs off
    assert r.core.telemetry["servo_deg_Y"] == round(r.sim.angles()[3])


def test_app_preparation_mode0_never_moves_the_arm(lib):
    """The Quest app's PREPARAZIONE sends mode-0 intents: the firmware reaches IDLE but nothing moves, not even
    with the deadman and the sticks; the start gesture then switches to MANUAL with the grips still held and
    the sticks centred, and only a stick moves the arm."""
    r = Rig(lib)
    r.run(1000)
    r.run(300, intent(mode=0))
    assert r.sim.state() == "IDLE"
    a0 = r.sim.angles()
    r.run(1000, intent(mode=0, joy_x=30000, yaw=-30000, buttons_left=GRIP | 1, buttons_right=GRIP | 1))
    assert r.sim.angles() == a0
    r.run(500, intent(mode=2, buttons_left=GRIP, buttons_right=GRIP))       # gesture done: MANUAL, grips held
    assert max(abs(x - y) for x, y in zip(r.sim.angles(), a0)) < 0.5
    r.run(500, intent(mode=2, joy_x=30000, buttons_left=GRIP, buttons_right=GRIP))
    assert r.sim.angles()[3] > a0[3] + 5


def test_head_mode_entry_resends_vr_params(lib):
    r = Rig(lib)
    r.run(1000)
    r.run(200, intent(mode=3, buttons_left=GRIP, buttons_right=GRIP))
    vr = [e for e in r.responses() if e["cmd"].startswith("SET_VR_PARAMS")]
    assert len(vr) == 2                                    # boot + mode-3 entry (throttled 2.5 s)
    r.run(500, intent(mode=3, buttons_left=GRIP, buttons_right=GRIP, quat_w=0.966, quat_y=0.259))
    assert r.sim.state() == "IDLE"


def test_estop_bits_and_lines(lib):
    r = Rig(lib)
    r.run(1000)
    r.run(300, intent())
    r.sim.estop(True)
    r.run(300, intent())
    assert "ESTOP" in r.unsolicited()
    assert r.core.diag.get("estop_active") is True
    assert r.sim.state() == "STOPPED"
    seq = r.core.uart_send("ENABLE")
    r.run(100, intent())
    assert any(e["cmd"] == "ENABLE" and not e["ok"] and "ESTOP_ACTIVE" in e["response"] for e in r.responses())


def test_128_byte_transport_with_128_byte_firmware(lib128):
    r = Rig(lib128, transfer_len=128)
    r.run(1000)
    r.run(500, intent())
    assert r.sim.state() == "IDLE"
    assert r.core.stats["spi_rx_bad"] <= 1                 # only the very first reply (nothing queued yet)


def test_intent_validation_matches_ws_teleop():
    p, err = j5mini.intent_to_payload(intent(joy_x=1))
    assert err is None and struct.unpack_from(">h", p, 1)[0] == 1          # no +-1 full-scale quirk
    assert j5mini.intent_to_payload(intent(mode=6))[1]
    assert j5mini.intent_to_payload({"mode": 2})[1]
    p, _ = j5mini.intent_to_payload(intent(mode=2, joy_x=0.5, intensity=0.5, quat_w=0.5))
    assert struct.unpack_from(">h", p, 1)[0] == 16384 and p[9] == 128
    assert struct.unpack_from(">f", p, 16)[0] == 1.0                        # quaternion only in modes 3/4/5


def test_websocket_end_to_end(lib):
    """Real WebSocket client (like the Quest app) -> j5mini server -> simulated STM32."""
    websockets = pytest.importorskip("websockets")
    r = Rig(lib)
    lock, stop = threading.Lock(), threading.Event()

    def stepper():                                          # simulated firmware time follows real time
        while not stop.is_set():
            with lock:
                r.core.step()
                r.sim.run(10)
            time.sleep(0.01)

    cfg = json.loads(json.dumps(CFG))
    srv = j5mini.Server(r.core, lock, cfg)

    async def go():
        async with websockets.serve(srv.handler, "127.0.0.1", 0) as wss:      # free port from the OS
            port = next(iter(wss.sockets)).getsockname()[1]
            bt = asyncio.create_task(srv.broadcast_loop())
            async with websockets.connect(f"ws://127.0.0.1:{port}") as ws:
                hb = 0
                seen = {}
                t_end = time.time() + 3.0
                await ws.send(json.dumps({"type": "uart", "cmd": "STATUS?"}))
                await ws.send(json.dumps({"type": "get_settings"}))
                while time.time() < t_end:
                    hb += 1
                    await ws.send(json.dumps(intent(heartbeat=hb)))
                    try:
                        m = json.loads(await asyncio.wait_for(ws.recv(), 0.02))
                        seen.setdefault(m["type"], m)
                        if m["type"] == "uart_response":
                            seen.setdefault("resp:" + m["cmd"], m)
                    except asyncio.TimeoutError:
                        pass
                    await asyncio.sleep(0.016)
            bt.cancel()
            return seen

    th = threading.Thread(target=stepper, daemon=True)
    th.start()
    try:
        seen = asyncio.run(go())
    finally:
        stop.set()
        th.join(1)
    assert seen["telemetry"]["robot_state"] in ("SAFE", "IDLE", "UNKNOWN")
    assert "servo_deg_B" in seen["telemetry"]
    assert seen["resp:STATUS?"]["response"].startswith("STATUS:")
    assert r.sim.state() == "IDLE"                          # the intents armed the firmware
    assert seen["settings"]["offsets"] == CFG["settings"]["offsets"] and len(seen["settings"]["dirs"]) == 6


class FakeClock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def test_camera_controls_like_ws_teleop(tmp_path):
    sent, clk = [], FakeClock()
    state = tmp_path / "cam0.json"
    cams = j5mini.Cameras(str(state), clock=clk, sendto=lambda data, port: sent.append((port, json.loads(data))))
    assert cams.on_message({"type": "vr_zoom_state", "zoom0": 3.0, "zoom1": 3.0, "camera": True})
    assert sent == [(5600, {"zoom": 3.0}), (5601, {"zoom": 3.0})]
    sent.clear()
    cams.on_message({"type": "cam_exposure", "ev": 5})                 # clamped like ws-teleop
    cams.on_message({"type": "cam_focus", "dioptres": 2.5})            # cam0 only, cam1 follows
    cams.on_message({"type": "cam_focus", "auto": True})
    cams.on_message({"type": "cam_mode", "mode": "wide"})
    cams.on_message({"type": "cam_mode", "mode": "bogus"})
    assert sent == [(5600, {"ev": 2.0}), (5601, {"ev": 2.0}), (5600, {"focus": 2.5}), (5600, {"focus": "auto"}),
                    (5600, {"mode": "wide"}), (5601, {"mode": "wide"})]
    assert not cams.on_message({"type": "uart"})
    # cam0 reports 2x applied in tele: residual digital zoom 1.5, nothing re-sent within 1 s
    state.write_text(json.dumps({"zoom": 2.0, "mode": "tele", "hfov": 46.8, "fps": 90, "ts": time.time()}))
    sent.clear()
    tm = cams.telemetry()
    assert tm["cam_zoom"] == 2.0 and tm["vr_zoom0"] == 1.5 and tm["cam_ev"] == 2.0 and tm["cam_mode"] == "tele"
    assert sent == []
    clk.t += 3.5                                                        # zoom/ev every 1 s, mode after 3 s
    cams.telemetry()
    assert {(p, tuple(m)) for p, m in sent} == {(c, k) for c in (5600, 5601) for k in (("zoom",), ("ev",), ("mode",))}
    # autofocus: both cameras, 1 s cooldown
    sent.clear()
    assert cams.autofocus() and not cams.autofocus()
    assert sent == [(5600, {"af": {"src": "button"}}), (5601, {"af": {"src": "button"}})]
    clk.t += 1.1
    assert cams.autofocus()


def test_https_api_of_the_app():
    import urllib.request
    sent = []
    cams = j5mini.Cameras("/nonexistent", sendto=lambda data, port: sent.append(port))
    cfg = {"https": {"host": "127.0.0.1", "port": 0, "controller_mappings": str(HERE / "controller_mappings.json")}}
    srv = j5mini.https_server(cfg, cams, threading.Lock())
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{srv.server_address[1]}"
    body = urllib.request.urlopen(base + "/api/controller-mappings", timeout=2).read()
    assert json.loads(body) == json.loads((HERE / "controller_mappings.json").read_text(encoding="utf-8"))
    req = urllib.request.Request(base + "/api/refocus-cameras", data=b"", method="POST")
    assert json.loads(urllib.request.urlopen(req, timeout=2).read())["ok"] is True
    assert sent == [5600, 5601]
    with pytest.raises(urllib.error.HTTPError) as e:
        urllib.request.urlopen(urllib.request.Request(base + "/api/refocus-cameras", data=b"", method="POST"), timeout=2)
    assert e.value.code == 429


def test_relay_for_collimator_and_vision(lib, tmp_path):
    """The optional robot services are WebSocket clients: their messages reach the app and back, an "aim"
    result gets vision_point ok=false (no ASSIST gimbal), and the servo angles land in the telemetry file."""
    websockets = pytest.importorskip("websockets")
    r = Rig(lib)
    r.run(300)
    cfg = json.loads(json.dumps(CFG))
    cfg["telemetry_file"] = str(tmp_path / "j5vr_telemetry.json")
    srv = j5mini.Server(r.core, threading.Lock(), cfg, j5mini.Cameras("/nonexistent", sendto=lambda d, p: None))

    async def recv_type(ws, typ):
        while True:
            m = json.loads(await asyncio.wait_for(ws.recv(), 2))
            if m["type"] == typ:
                return m

    async def go():
        async with websockets.serve(srv.handler, "127.0.0.1", 0) as wss:
            url = "ws://127.0.0.1:%d" % next(iter(wss.sockets)).getsockname()[1]
            bt = asyncio.create_task(srv.broadcast_loop())
            async with websockets.connect(url) as app, websockets.connect(url) as svc:
                await asyncio.sleep(0.1)
                await svc.send(json.dumps({"type": "stereo_collimation", "dx": 12.5, "dy": -1.0, "lock": True}))
                col = await recv_type(app, "stereo_collimation")
                await app.send(json.dumps({"type": "vision_request", "action": "point", "req": 7}))
                req = await recv_type(svc, "vision_request")
                await svc.send(json.dumps({"type": "vision_result", "req": 7, "ok": True, "aim": True,
                                           "target": {"label_it": "tazza"}}))
                res = await recv_type(app, "vision_result")
                pt = await recv_type(app, "vision_point")
            bt.cancel()
            return col, req, res, pt

    col, req, res, pt = asyncio.run(go())
    assert col["dx"] == 12.5 and col["lock"] is True
    assert req["action"] == "point" and req["req"] == 7
    assert res["req"] == 7
    assert pt["ok"] is False and pt["label"] == "tazza" and "minimale" in pt["why"]
    tm = json.loads((tmp_path / "j5vr_telemetry.json").read_text())
    assert set(tm) == set(j5mini.SERVO_KEYS)
