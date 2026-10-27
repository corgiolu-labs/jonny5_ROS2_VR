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
    result without a measured distance gets vision_point ok=false, and the servo angles land in the telemetry
    file."""
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
    assert pt["ok"] is False and pt["label"] == "tazza" and pt["why"] == "distanza non misurata"
    tm = json.loads((tmp_path / "j5vr_telemetry.json").read_text())
    assert set(tm) == set(j5mini.SERVO_KEYS)


# ------------------------------------------------------------------ dashboard support

def test_settings_round_trip_rebuilds_the_boot_lines():
    s = j5mini.settings_from_config(CFG)
    assert s["offsets"] == CFG["settings"]["offsets"] and s["dirs"] == CFG["settings"]["dirs"]
    assert s["limits"][5] == [68, 128] and s["vr_params"].startswith("SET_VR_PARAMS ")
    lines, err = j5mini.boot_lines_from_settings(s)
    assert err is None and lines == CFG["boot_commands"]          # same lines, same order as the Pi's


@pytest.mark.parametrize("change, msg", [
    (dict(offsets=[90] * 5), "offsets"), (dict(dirs=[1, 0, 1, 1, 1, 1]), "dirs"),
    (dict(limits=[[10, 170]] * 5 + [[120, 60]]), "limits"), (dict(vr_params="SET_VR_PARAMS 1; rm -rf"), "vr_params"),
    (dict(pwm_config="SET_OFFSETS 1 2 3"), "pwm_config"),
])
def test_settings_validation(change, msg):
    s = {**j5mini.settings_from_config(CFG), **change}
    lines, err = j5mini.boot_lines_from_settings(s)
    assert lines is None and err.startswith(msg)


def test_save_config_keeps_paths_and_a_backup(tmp_path):
    path = tmp_path / "j5mini.json"
    path.write_text((HERE / "j5mini.example.json").read_text(encoding="utf-8"), encoding="utf-8")
    s = j5mini.settings_from_config(CFG)
    s["offsets"][0] = 91
    lines, _ = j5mini.boot_lines_from_settings(s)
    j5mini.save_config_file(str(path), lines, [1, 1, 1, 1, 1, 1])
    disk = json.loads(path.read_text(encoding="utf-8"))
    assert disk["boot_commands"][0].startswith("SET_OFFSETS 91 ")
    assert disk["settings"]["offsets"][0] == 91 and disk["settings"]["dirs"] == [1] * 6
    assert disk["ws"]["cert"] == CFG["ws"]["cert"]                 # relative paths untouched
    assert json.loads((tmp_path / "j5mini.json.bak").read_text(encoding="utf-8")) == CFG


def test_dashboard_messages_over_websocket(lib, tmp_path):
    websockets = pytest.importorskip("websockets")
    r = Rig(lib)
    r.run(300)
    path = tmp_path / "j5mini.json"
    path.write_text((HERE / "j5mini.example.json").read_text(encoding="utf-8"), encoding="utf-8")
    cfg = json.loads(json.dumps(CFG))
    cfg["_path"] = str(path)
    srv = j5mini.Server(r.core, threading.Lock(), cfg, j5mini.Cameras("/nonexistent", sendto=lambda d, p: None))

    async def go():
        async with websockets.serve(srv.handler, "127.0.0.1", 0) as wss:
            url = "ws://127.0.0.1:%d" % next(iter(wss.sockets)).getsockname()[1]
            async with websockets.connect(url) as ws:
                async def ask(o, typ):
                    await ws.send(json.dumps(o))
                    while True:
                        m = json.loads(await asyncio.wait_for(ws.recv(), 2))
                        if m["type"] == typ:
                            return m
                conf = await ask({"type": "get_config"}, "config")
                pong = await ask({"type": "ws_ping", "id": 7, "ts_client": 1.5}, "ws_pong")
                bad = await ask({**conf, "type": "save_config", "offsets": [1, 2]}, "config_saved")
                conf["limits"][0] = [40, 140]
                ok = await ask({**conf, "type": "save_config"}, "config_saved")
                return conf, pong, bad, ok

    conf, pong, bad, ok = asyncio.run(go())
    assert conf["offsets"] == CFG["settings"]["offsets"] and conf["saveable"] is True and conf["sim"] is False
    assert pong["id"] == 7 and pong["ts_client"] == 1.5
    assert bad["ok"] is False and "offsets" in bad["error"]
    assert ok["ok"] is True and "SET_JOINT_LIMITS 0 40 140" in ok["boot_commands"]
    assert "SET_JOINT_LIMITS 0 40 140" in json.loads(path.read_text(encoding="utf-8"))["boot_commands"]
    assert "SET_JOINT_LIMITS 0 40 140" in r.core.boot_lines        # the next BOOT_READY pushes it


def test_https_serves_the_dashboard_and_nothing_else():
    import urllib.error
    import urllib.request
    dash = HERE.parent / "dashboard"
    cfg = {"https": {"host": "127.0.0.1", "port": 0, "dashboard": str(dash)}}
    srv = j5mini.https_server(cfg, j5mini.Cameras("/nonexistent", sendto=lambda d, p: None), threading.Lock())
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{srv.server_address[1]}"
    page = urllib.request.urlopen(base + "/", timeout=2)
    assert page.headers["Content-Type"].startswith("text/html") and b"app.js" in page.read()
    js = urllib.request.urlopen(base + "/kin.js", timeout=2)
    assert js.headers["Content-Type"].startswith("text/javascript")
    for bad in ("/../j5mini/j5mini.py", "/%2e%2e/j5mini/j5mini.py", "/nope.html"):
        with pytest.raises(urllib.error.HTTPError) as e:
            urllib.request.urlopen(base + bad, timeout=2)
        assert e.value.code == 404
    srv.shutdown()


def test_sim_backend_boots_and_takes_the_configuration():
    """--sim: the same Core on the simulated firmware, as a student's PC runs it."""
    robot = j5mini.SimRobot(64)
    core = j5mini.Core(json.loads(json.dumps(CFG)), robot, robot, clock=lambda: robot.sim.now_ms() / 1000.0,
                       log=lambda s: None)
    for _ in range(150):
        core.step()
        robot.tick()
    assert all(e["ok"] for e in core.events if e["type"] == "uart_response" and e["cmd"].startswith("SET_"))
    core.uart_send("ENABLE")
    for _ in range(30):
        core.step()
        robot.tick()
    assert robot.sim.state() == "IDLE"


def test_whep_proxy_for_the_video_test():
    """POST /whep/cam0 -> MediaMTX's WHEP (here a fake), only the camera paths; 502 when MediaMTX is down."""
    import urllib.error
    import urllib.request
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
    seen = []

    class FakeMtx(BaseHTTPRequestHandler):
        def do_POST(self):
            body = self.rfile.read(int(self.headers["Content-Length"]))
            seen.append((self.path, self.headers["Content-Type"], body))
            self.send_response(201)
            self.send_header("Content-Type", "application/sdp")
            self.end_headers()
            self.wfile.write(b"v=0 answer")

        def log_message(self, *a):
            pass

    mtx = ThreadingHTTPServer(("127.0.0.1", 0), FakeMtx)
    threading.Thread(target=mtx.serve_forever, daemon=True).start()
    cams = j5mini.Cameras("/nonexistent", sendto=lambda d, p: None)
    srv = j5mini.https_server({"https": {"host": "127.0.0.1", "port": 0,
                                         "whep": f"http://127.0.0.1:{mtx.server_address[1]}"}}, cams, threading.Lock())
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{srv.server_address[1]}"

    def post(path, data=b"v=0 offer"):
        return urllib.request.urlopen(urllib.request.Request(base + path, data=data, method="POST",
                                                             headers={"Content-Type": "application/sdp"}), timeout=3)
    r = post("/whep/cam1")
    assert r.status == 201 and r.read() == b"v=0 answer" and r.headers["Content-Type"] == "application/sdp"
    assert seen == [("/cam1/whep", "application/sdp", b"v=0 offer")]
    for bad in ("/whep/cam_g2g", "/whep/../v3/config/global/get"):
        with pytest.raises(urllib.error.HTTPError) as e:
            post(bad)
        assert e.value.code == 404
    mtx.shutdown(); mtx.server_close()
    with pytest.raises(urllib.error.HTTPError) as e:
        post("/whep/cam0")
    assert e.value.code == 502 and b"MediaMTX" in e.value.read()
    srv.shutdown()


def test_cam_autofocus_message():
    sent = []
    cams = j5mini.Cameras("/nonexistent", sendto=lambda d, p: sent.append((p, json.loads(d))))
    assert cams.on_message({"type": "cam_autofocus"})
    assert sent == [(5600, {"af": {"src": "button"}}), (5601, {"af": {"src": "button"}})]


@pytest.mark.parametrize("phase", [0, 3, 37, 61, 63])
def test_link_survives_the_slave_dma_phase(lib, phase):
    """On the robot the slave DMA ran 3 bytes late (26/10/2026): every reply straddled two 64-byte transfers and
    j5mini, which looked for the frame at offset 0 only, saw no valid reply at all. It now scans the last two
    transfers for the newest complete frame."""
    r = Rig(lib)
    r.sim.spi_phase(phase)
    r.run(1000)
    r.run(500, intent())
    s = r.core.stats
    assert r.sim.state() == "IDLE"                                    # the firmware still reads our frames
    assert s["spi_rx_ok"] >= 0.9 * s["spi_tx"] - 3, s                # and we read its replies
    assert s["spi_shift"] == (64 - phase) % 64                        # phase 61 = the robot: frame at byte 3
    assert r.core.diag.get("estop_active") is False                   # STATUS replies decoded
    assert r.core.telemetry.get("servo_deg_B") is not None


# ------------------------------------------------------------------ ASSIST (mode 5, the old Pi's gimbal)

def quat_y(deg):
    """Head turned left by deg (XR frame: y up)."""
    import math
    h = math.radians(deg) / 2
    return {"quat_w": math.cos(h), "quat_x": 0.0, "quat_y": math.sin(h), "quat_z": 0.0}


def test_assist_moves_the_whole_arm_and_regrip_does_not_jump(lib):
    """26/10/2026 on the robot: mode 5 held B/S/G (no gimbal in j5mini). Now the head turn moves the base too, and
    grips released + head turned + grips pressed again = no jump, the arm goes on from where it is."""
    pytest.importorskip("numpy")
    import assist_gimbal
    assist_gimbal._state = assist_gimbal.GimbalState()
    r = Rig(lib)
    lock, stop = threading.Lock(), threading.Event()
    r.run(1000)
    r.run(300, intent())
    r.core.uart_send("HOME")
    r.run(3000, intent())
    home = r.sim.angles()

    def stepper():
        while not stop.is_set():
            with lock:
                r.core.step()
                r.sim.run(10)
            time.sleep(0.01)

    srv = j5mini.Server(r.core, lock, json.loads(json.dumps(CFG)), j5mini.Cameras("/nonexistent", sendto=lambda d, p: None))
    assert srv.assist is not None

    class Ws:
        async def send(self, data):
            pass

    async def stream(seconds, deg0, deg1, grip):
        n = int(seconds * 60)
        for i in range(n):
            r.hb += 1
            deg = deg0 + (deg1 - deg0) * (i + 1) / n
            g = GRIP if grip else 0
            await srv.on_message(Ws(), intent(mode=5, grip=1 if grip else 0, buttons_left=g, buttons_right=g,
                                              heartbeat=r.hb, **quat_y(deg)))
            await asyncio.sleep(1 / 60)

    async def go():
        await stream(1.0, 0, 0, True)                     # engage: HEADZERO, references
        await stream(2.0, 0, 40, True)                    # head left 40 deg
        await stream(1.0, 40, 40, True)
        turned = r.sim.angles()
        await stream(1.0, 40, 80, False)                  # grips released, the head turns on
        await stream(0.5, 80, 80, False)
        released = r.sim.angles()
        await stream(1.5, 80, 80, True)                   # grips pressed again with the head elsewhere
        regrip = r.sim.angles()
        await stream(1.5, 80, 60, True)                   # and it follows from here
        return turned, released, regrip, r.sim.angles()

    th = threading.Thread(target=stepper, daemon=True)
    th.start()
    try:
        turned, released, regrip, back = asyncio.run(go())
    finally:
        stop.set()
        th.join(1)
    assert abs(turned[0] - home[0]) > 5, (home, turned)                    # the base helped the wrist yaw
    assert max(abs(a - b) for a, b in zip(released, turned)) < 1.0         # released: nothing moves
    assert max(abs(a - b) for a, b in zip(regrip, released)) < 2.0, (released, regrip)   # no jump
    assert max(abs(a - b) for a, b in zip(back, regrip)) > 3.0             # steering again


def test_park_and_vr_pose_go_to_the_saved_poses_with_ack(lib):
    """26/10/2026 on the robot: PARK and VR pose both went to the firmware's factory teleop pose. ws-teleop sent a
    SETPOSE to the saved virtual poses (j5_settings park / vr) and teleop_pose_ack (the Quest app waits for it)."""
    cfg = json.loads(json.dumps(CFG))
    cfg["settings"]["dirs"] = [1, -1, 1, 1, -1, 1]                 # the robot's
    r = Rig(lib, cfg)
    r.run(1000)
    r.run(300, intent())
    srv = j5mini.Server(r.core, threading.Lock(), cfg, j5mini.Cameras("/nonexistent", sendto=lambda d, p: None))

    async def send(cmd):
        await srv.on_message(None, {"type": "uart", "cmd": cmd})

    out = {}
    for cmd in ("PARK", "TELEOPPOSE"):
        r.core.events = []
        asyncio.run(send(cmd))
        r.run(3000, intent())
        out[cmd] = ([round(a) for a in r.sim.angles()],
                    [e for e in r.core.events if e.get("cmd") == cmd], [e for e in r.core.events if e.get("teleop_pose_ack")])
    assert out["PARK"][0] == [88, 125, 143, 90, 50, 98]
    assert out["TELEOPPOSE"][0] == [88, 65, 83, 90, 110, 98]
    for cmd in out:
        assert out[cmd][1] and out[cmd][1][0]["ok"], cmd       # the reply keeps the client's command
        assert len(out[cmd][2]) == 1, cmd


def jerkiness(lib, smoothing_ms, seed=1):
    """HEAD mode, the head turning smoothly (sinusoid), intents as the Quest app sends them: 60/s from a 90 Hz
    frame loop (11 / 22 ms apart) plus Wi-Fi delay jitter, timestamped at sampling. Returns the spread of the
    moving wrist servo's velocity over 10 ms windows (relative), i.e. how uneven the motion is."""
    import math
    import random
    rnd = random.Random(seed)
    cfg = json.loads(json.dumps(CFG))
    cfg["smoothing_ms"] = smoothing_ms
    r = Rig(lib, cfg)
    r.run(1000)
    r.run(300, intent())
    r.core.uart_send("HOME")
    r.run(3000, intent())
    r.run(200, intent(mode=3, grip=1, buttons_left=GRIP, buttons_right=GRIP))
    t0 = r.sim.now_ms()
    sends, t, k = [], 0.0, 0
    while t < 3000:                                   # sampling times on the 90 Hz grid, 1 or 2 frames apart
        t += (1000 / 90) * (1 if k % 2 else 2)
        k += 1
        sends.append((t + 4 + rnd.uniform(0, 14), t))  # (arrival, sampled) in ms after t0
    sends.sort()
    angles, i, hb = [], 0, 0
    for ms in range(3000):
        while i < len(sends) and sends[i][0] <= ms:
            ts = sends[i][1]
            a = math.radians(15 * math.sin(2 * math.pi * ts / 2000.0)) / 2
            hb += 1
            assert r.core.set_intent(intent(mode=3, grip=1, buttons_left=GRIP, buttons_right=GRIP, heartbeat=hb,
                                            quat_w=math.cos(a), quat_x=0.0, quat_y=math.sin(a), quat_z=0.0,
                                            timestamp=(t0 + ts) / 1000.0)) is None
            i += 1
        if ms % 10 == 0:
            r.core.step()
        r.sim.run(1)
        angles.append(r.sim.angles())
    j = max(range(6), key=lambda n: max(a[n] for a in angles) - min(a[n] for a in angles))
    x = [a[j] for a in angles[500:]]
    v = [x[n + 10] - x[n] for n in range(0, len(x) - 10, 10)]
    dv = [b - a for a, b in zip(v, v[1:])]
    mean_abs_v = sum(abs(q) for q in v) / len(v)
    return (sum(q * q for q in dv) / len(dv)) ** 0.5 / mean_abs_v, max(x) - min(x)


def test_head_motion_is_smoothed_against_the_apps_irregular_intents(lib):
    """26/10/2026: small jerks in ASSIST on the robot. The same smooth head motion, irregular intents: with the
    interpolation the servo's 10 ms velocity varies much less."""
    raw, span_raw = jerkiness(lib, 0)
    smooth, span = jerkiness(lib, 30)
    assert span > 10 and span_raw > 10                  # the wrist really followed the head
    assert smooth < raw * 0.5, (raw, smooth)


def assist_camera_roll(lib, horizon):
    """ASSIST, head turned a bit and tilted sideways by 15 deg: roll of the commanded camera (deg from level)."""
    import math
    import numpy as np
    import assist_gimbal
    assist_gimbal._state = assist_gimbal.GimbalState()
    r = Rig(lib)
    lock, stop = threading.Lock(), threading.Event()
    r.run(1000)
    r.run(300, intent())
    r.core.uart_send("HOME")
    r.run(3000, intent())

    def stepper():
        while not stop.is_set():
            with lock:
                r.core.step()
                r.sim.run(10)
            time.sleep(0.01)

    srv = j5mini.Server(r.core, lock, json.loads(json.dumps(CFG)), j5mini.Cameras("/nonexistent", sendto=lambda d, p: None))

    class Ws:
        async def send(self, data):
            pass

    def head(yaw, roll):                        # XR frame: yaw about y (up), roll about z (back)
        hy, hr = math.radians(yaw) / 2, math.radians(roll) / 2
        qy = (math.cos(hy), 0.0, math.sin(hy), 0.0)
        qr = (math.cos(hr), 0.0, 0.0, math.sin(hr))
        a, b = qy, qr                           # q = qy * qr
        q = (a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
             a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
             a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
             a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0])
        return dict(zip(("quat_w", "quat_x", "quat_y", "quat_z"), q))

    async def stream(seconds, yaw0, yaw1, roll0, roll1):
        n = int(seconds * 60)
        for i in range(n):
            f = (i + 1) / n
            r.hb += 1
            await srv.on_message(Ws(), intent(mode=5, grip=1, buttons_left=GRIP, buttons_right=GRIP, heartbeat=r.hb,
                                              timestamp=time.monotonic(),
                                              **head(yaw0 + (yaw1 - yaw0) * f, roll0 + (roll1 - roll0) * f)))
            await asyncio.sleep(1 / 60)

    async def go():
        await srv.on_message(Ws(), {"type": "horizon_lock", "on": horizon})
        await stream(1.0, 0, 0, 0, 0)
        await stream(1.5, 0, 10, 0, 15)
        await stream(1.5, 10, 10, 15, 15)

    th = threading.Thread(target=stepper, daemon=True)
    th.start()
    try:
        asyncio.run(go())
    finally:
        stop.set()
        th.join(1)
    s = j5mini.settings_from_config(CFG)
    virt = assist_gimbal.settings_manager.physical_to_virtual(r.sim.angles(), s["offsets"], s["dirs"])
    rc = assist_gimbal.cam_rotation(np.radians(np.asarray(virt) - 90.0))
    return math.degrees(math.asin(float(rc[2, 1])))      # image horizontal axis (y) against the vertical


def test_horizon_lock_keeps_the_assist_image_level(lib):
    """26/10/2026, user: a button that keeps the camera roll level. Head tilted 15 deg: without the lock the camera
    tilts with it, with the lock it stays level."""
    pytest.importorskip("numpy")
    free = assist_camera_roll(lib, False)
    locked = assist_camera_roll(lib, True)
    assert abs(free) > 8, free
    assert abs(locked) < 2, locked


def test_horizon_in_every_mode_from_the_server(lib):
    """"Orizzonte" is not a mode: the server sends HORIZON to the firmware (joint directions) whatever the mode,
    the configuration push repeats it after an STM32 reset, and the telemetry reports it."""
    cfg = json.loads(json.dumps(CFG))
    cfg["settings"]["dirs"] = [1, -1, 1, 1, -1, 1]
    r = Rig(lib, cfg)
    r.run(1000)
    srv = j5mini.Server(r.core, threading.Lock(), cfg, j5mini.Cameras("/nonexistent", sendto=lambda d, p: None))

    class Ws:
        async def send(self, data):
            pass

    asyncio.run(srv.on_message(Ws(), {"type": "horizon_lock", "on": True}))
    r.run(200)
    resp = [e for e in r.core.events if e.get("cmd", "").startswith("HORIZON")]
    assert resp[-1]["cmd"] == "HORIZON 1 1 -1 1 1 -1 1 0.00" and resp[-1]["response"] == "OK HORIZON ON"
    assert r.core.telemetry_message() and r.core.horizon_on
    r.core._start_boot_push()
    assert r.core.boot_queue[-1] == "HORIZON 1 1 -1 1 1 -1 1 0.00"
    asyncio.run(srv.on_message(Ws(), {"type": "horizon_lock", "on": False}))
    r.run(200)
    assert [e for e in r.core.events if e.get("cmd") == "HORIZON 0"][-1]["response"] == "OK HORIZON OFF"


def test_horizon_trim_from_the_wrist_imu():
    """Arm and IMU still, image tilted (tool y has an upward component): the trim turns the roll back, in steps,
    and only while still."""
    import math
    sent = []
    core = j5mini.Core(json.loads(json.dumps(CFG)), None, None, clock=FakeClock(), log=lambda s: None)
    core.uart_send = lambda cmd, cb=None: sent.append(cmd)
    core.horizon_on = True
    core.telemetry = {k: 90 for k in j5mini.SERVO_KEYS}
    tilt = math.radians(3.0)
    up = (0.0, math.sin(tilt), math.cos(tilt))                 # image tilted by 3 deg, left side up

    def run(n):
        for _ in range(n):
            core.clock.t += 0.2
            core.payload_t = core.clock.t
            core.horizon_trim_step(up)
    # PREPARAZIONE (mode 0) / grips released: the firmware does not drive the roll -> no trim (it ran away before)
    core.last_mode, core.diag = 0, {"deadman": True}
    run(5)
    core.last_mode, core.diag = 5, {"deadman": False}
    run(5)
    assert sent == [] and core.horizon_trim == 0.0
    core.last_mode, core.diag = 5, {"deadman": True}
    run(5)
    assert sent and core.horizon_trim == pytest.approx(-1.5, abs=0.01)
    assert sent[-1].endswith(" -1.50")
    n = len(sent)
    for _ in range(5):                                         # the arm moves: no correction
        core.telemetry["servo_deg_Y"] += 1
        run(1)
    assert len(sent) == n


def point_end_wrist_jump():
    """26/10/2026, user: a small jerk at the end of "punta l'oggetto". With an IMU trim in effect the live IK after
    the motion differs from the planned pose: the wrist now blends into it over POINT_BLEND_S. Returns the largest
    per-intent wrist change (virtual deg)."""
    pytest.importorskip("numpy")
    import math
    import numpy as np
    import assist_gimbal as g
    g._state = g.GimbalState()
    g.configure([88, 85, 83, 90, 90, 98], [1, -1, 1, 1, -1, 1], telemetry=lambda: {"imu_valid": False})
    lim = {n: {"min": a, "max": b} for n, (a, b) in zip(j5mini.Assist.LIMIT_NAMES,
                                                       [[45, 135], [30, 145], [30, 145], [45, 135], [50, 145], [68, 128]])}
    vr = CFG["boot_commands"][1].split()[1:]

    class U:
        async def send_uart_command(self, c, timeout_s=1.0):
            return True, "OK HEADZERO"

    phys = [88.0, 65.0, 83.0, 90.0, 110.0, 98.0]           # the VR pose
    changes, requested = [], []                           # per intent: wrist change (deg), None during the motion

    async def go():
        nonlocal phys
        it = dict(mode=5, grip=1, quat_w=1.0, quat_x=0.0, quat_y=0.0, quat_z=0.0)
        for i in range(400):
            prev = g._state.last_full_v
            arm, *_ = g.step(dict(it), physical_six=phys, limits_src=lim, vr_params=vr, uart_manager=U(), grip=True,
                             pos_cfg={"enable": False}, pred_cfg={"enable": False}, trim_cfg={"enable": True})
            if g._state.phase == "track" and g._state.goal is None and i > 60 and not requested:
                g._state.trim = np.array([0.0, 0.0, math.radians(5.0)])     # 5 deg standing IMU correction
                ok, info = g.request_point([0.0, 0.0, 600.0], "tazza")
                assert ok, info
                requested.append(i)
            if arm is not None and g._state.last_full_v is not None:   # the servos follow the command at once
                phys = [float(v) for v in arm] + [g.settings_manager.current["offsets"][k] +
                                                 (1 if g.settings_manager.current["dirs"][k] >= 0 else -1) *
                                                 (g._state.last_full_v[k] - 90.0) for k in (3, 4, 5)]
            if prev is not None and g._state.last_full_v is not None:
                changes.append(None if g._state.goal is not None else
                               max(abs(a - b) for a, b in zip(prev[3:], g._state.last_full_v[3:])))
            await asyncio.sleep(0.016)

    asyncio.run(go())
    assert g._state.goal is None and g._state.goal_done is not None       # the motion ran and finished
    end = max(i for i, c in enumerate(changes) if c is None)               # last intent of the motion
    return max(c for c in changes[end + 1:end + 20] if c is not None)      # the 0.3 s after it


def test_point_at_object_ends_without_a_wrist_jump():
    assert point_end_wrist_jump() < 1.0                 # per-intent wrist change (deg) right after the motion;
                                                        # before: 3.4 (arm assist at full rate at once)


def test_dashboard_websocket_tunnel_on_the_https_port():
    """GET /ws with Upgrade on the HTTPS port reaches the controller's local listener byte by byte: the page
    talks to its own origin and the browser accepts one certificate (443), not two."""
    websockets = pytest.importorskip("websockets")
    robot = j5mini.SimRobot(64)
    core = j5mini.Core(json.loads(json.dumps(CFG)), robot, robot, log=lambda s: None)
    srv = j5mini.Server(core, threading.Lock(), json.loads(json.dumps(CFG)),
                        j5mini.Cameras("/nonexistent", sendto=lambda d, p: None))

    async def go():
        async with websockets.serve(srv.handler, "127.0.0.1", 0) as local:
            lport = next(iter(local.sockets)).getsockname()[1]
            http = j5mini.https_server({"https": {"host": "127.0.0.1", "port": 0}, "ws": {"local_port": lport}},
                                       srv.cams, threading.Lock())
            threading.Thread(target=http.serve_forever, daemon=True).start()
            try:
                async with websockets.connect(f"ws://127.0.0.1:{http.server_address[1]}/ws") as ws:
                    await ws.send(json.dumps({"type": "ws_ping", "id": 3}))
                    while True:
                        m = json.loads(await asyncio.wait_for(ws.recv(), 2))
                        if m["type"] == "ws_pong":
                            return m
            finally:
                http.shutdown()

    assert asyncio.run(go())["id"] == 3


def test_https_ws_without_a_local_listener_is_refused():
    srv = j5mini.https_server({"https": {"host": "127.0.0.1", "port": 0}}, j5mini.Cameras("/nonexistent", sendto=lambda d, p: None),
                              threading.Lock())
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    import http.client                      # urllib would force "Connection: close"
    c = http.client.HTTPConnection("127.0.0.1", srv.server_address[1], timeout=2)
    c.putrequest("GET", "/ws", skip_host=False)
    c.putheader("Connection", "Upgrade")
    c.putheader("Upgrade", "websocket")
    c.endheaders()
    assert c.getresponse().status == 503
    c.close()
    srv.shutdown()


def test_save_config_poses_speed_and_profile(tmp_path):
    """PARK / VR poses, SETPOSE speed and profile edited by the dashboard: validated, written to j5mini.json and
    used at once by pose_command()."""
    path = tmp_path / "j5mini.json"
    path.write_text((HERE / "j5mini.example.json").read_text(encoding="utf-8"), encoding="utf-8")
    cfg = json.loads(json.dumps(CFG))
    cfg["_path"] = str(path)
    robot = j5mini.SimRobot(64)
    core = j5mini.Core(cfg, robot, robot, log=lambda s: None)
    srv = j5mini.Server(core, threading.Lock(), cfg, j5mini.Cameras("/nonexistent", sendto=lambda d, p: None))
    s = j5mini.settings_from_config(cfg)
    assert s["park"] == CFG["settings"]["park"] and s["speed_pct"] == 40 and s["profile"] == "RTR5"
    for change, key in ((dict(speed_pct=0), "speed_pct"), (dict(profile="FAST"), "profile"), (dict(vr=[90] * 5), "vr"),
                        (dict(park=[90, 90, 90, 90, 90, 181]), "park")):
        bad = srv.save_config({**s, **change})
        assert bad["ok"] is False and bad["error"].startswith(key)
    ok = srv.save_config({**s, "park": [90, 60, 140, 90, 120, 90.5], "speed_pct": 25, "profile": "RTR3"})
    assert ok["ok"] is True
    disk = json.loads(path.read_text(encoding="utf-8"))["settings"]
    assert disk["park"] == [90, 60, 140, 90, 120, 90.5] and disk["speed_pct"] == 25 and disk["profile"] == "RTR3"
    assert disk["vr"] == CFG["settings"]["vr"]
    assert j5mini.pose_setpose_line(cfg, "park").endswith(" 25 RTR3")
    assert j5mini.settings_from_config(cfg)["profile"] == "RTR3"


def test_telemetry_carries_the_headset_pose():
    robot = j5mini.SimRobot(64)
    core = j5mini.Core(json.loads(json.dumps(CFG)), robot, robot, clock=lambda: robot.sim.now_ms() / 1000.0,
                       log=lambda s: None)
    assert core.telemetry_message()["vr_q"] is None
    assert core.set_intent(intent(mode=3, heartbeat=7, quat_w=0.9, quat_x=0.1, quat_y=0.2, quat_z=0.3, timestamp=1.0)) is None
    core.step()
    robot.tick()
    tm = core.telemetry_message()
    assert tm["vr_q"] == [0.9, 0.1, 0.2, 0.3] and tm["teleop_mode"] == 3 and tm["vr_heartbeat"] == 7


def test_kinematics_editor_validates_saves_and_reconfigures_the_gimbal(tmp_path):
    """Impostazioni -> Cinematica (POE): get_kinematics serves the file, save_kinematics validates, writes it with a
    .bak (config on disk) and hands the new screw axes to the ASSIST gimbal."""
    kin = tmp_path / "kinematics.json"
    kin.write_text(j5mini.KIN_PATH.read_text(encoding="utf-8"), encoding="utf-8")
    cfg = json.loads(json.dumps(CFG))
    cfg["_path"] = str(tmp_path / "j5mini.json")
    robot = j5mini.SimRobot(64)
    core = j5mini.Core(cfg, robot, robot, log=lambda s: None)
    srv = j5mini.Server(core, threading.Lock(), cfg, j5mini.Cameras("/nonexistent", sendto=lambda d, p: None), kin_path=kin)
    assert srv.kin["S"][1][3] == -0.1343 and "_doc" in srv.kin
    good = {k: srv.kin[k] for k in ("S", "M", "draw_points")}
    for change, key in (({"S": good["S"][:5]}, "S:"), ({"S": [[0, 0, 2, 0, 0, 0]] + good["S"][1:]}, "S[0]"),
                        ({"M": good["M"][:3] + [[0, 0, 0, 2]]}, "M:"), ({"draw_points": [[0, 0]] * 6}, "draw_points")):
        bad = srv.save_kinematics({**good, **change})
        assert bad["ok"] is False and bad["error"].startswith(key)
    new_s = json.loads(json.dumps(good["S"]))
    new_s[1][3] = -0.15
    ok = srv.save_kinematics({**good, "S": new_s})
    assert ok["ok"] is True and ok["saved_to_file"] is True
    disk = json.loads(kin.read_text(encoding="utf-8"))
    assert disk["S"][1][3] == -0.15 and disk["_doc"].startswith("JONNY5") and disk["joints"][0] == "Base"
    assert json.loads((tmp_path / "kinematics.json.bak").read_text(encoding="utf-8"))["S"][1][3] == -0.1343
    assert srv.kin["S"][1][3] == -0.15
    if srv.assist:                                                   # numpy present: the gimbal got the new axes
        assert srv.assist.g._glue["screws"][1][3] == -0.15 and srv.assist.g._ARM_GEOM is None
        assert abs(srv.assist.g._arm_geom()[0] - 0.15) < 1e-9       # shoulder height from the new screw


class FakeWs:
    def __init__(self):
        self.sent = []

    async def send(self, data):
        self.sent.append(json.loads(data))


def test_demo_loops_the_saved_sequence_and_stops(lib):
    """Dashboard "Demo" (27/10/2026, the old Pi's demo_steps): one SETPOSE per step, the next at SETPOSE_DONE,
    round and round; the dashboard stops it where it is, STOP / SAFE / E-STOP end it as well."""
    cfg = json.loads(json.dumps(CFG))
    cfg["settings"]["demo"] = [{"angles": [90, 90, 90, 90, 90, 90], "speed_pct": 60, "profile": "RTR5"},
                               {"angles": [100, 90, 90, 90, 90, 90], "speed_pct": 60, "profile": "RTR3"}]
    r = Rig(lib, cfg)
    r.run(1000)
    r.run(2200, intent())                                   # IDLE, and a STATUS? poll has seen it
    assert r.core.robot_state == "IDLE"
    srv = j5mini.Server(r.core, threading.Lock(), cfg, j5mini.Cameras("/nonexistent", sendto=lambda d, p: None))
    ws = FakeWs()

    def msg(m):
        asyncio.run(srv.on_message(ws, m))

    msg({"type": "demo", "on": True})
    assert ws.sent[-1] == {"type": "demo", "ok": True, "on": True, "error": None}
    r.run(6000, intent())
    steps = [e["step"] for e in r.core.events if e["type"] == "demo" and e.get("on")]
    assert steps[:4] == [1, 2, 1, 2], steps                 # it loops
    assert r.core.telemetry_message()["demo"] in ({"step": 1, "n": 2}, {"step": 2, "n": 2})
    sent = [e for e in r.core.events if e["type"] == "uart_response" and e["cmd"].startswith("SETPOSE ")]
    assert sent and all(e["ok"] for e in sent)
    assert sent[0]["cmd"] == "SETPOSE 88 85 83 90 90 98 60 RTR5"      # virtual 90 = the offsets
    assert sent[1]["cmd"] == "SETPOSE 98 85 83 90 90 98 60 RTR3"      # base +10 (dir +1)
    msg({"type": "demo", "on": False})
    r.run(500, intent())
    assert r.core.demo is None and r.core.telemetry_message()["demo"] is None
    ends = [e for e in r.core.events if e["type"] == "demo" and not e.get("on")]
    assert ends and ends[-1]["reason"] == "dashboard"
    hold = [e for e in r.core.events if e["type"] == "uart_response" and e["cmd"].startswith("SETPOSE_T_HR ")]
    assert hold and hold[-1]["ok"]                          # stopped where it was (120 ms hold trajectory)
    # STOP from any client ends it: the next STATUS? poll sees STOPPED
    r.core.events = []
    msg({"type": "demo", "on": True})
    r.run(300, intent())
    msg({"type": "uart", "cmd": "STOP"})
    r.run(2500, intent())
    assert r.core.demo is None
    assert [e for e in r.core.events if e["type"] == "demo" and not e.get("on")][-1]["reason"] == "robot in STOPPED"
    # not in IDLE: refused
    msg({"type": "demo", "on": True})
    assert ws.sent[-1]["ok"] is False and "IDLE" in ws.sent[-1]["error"]


def test_demo_steps_are_saved_with_the_config(tmp_path):
    cfg = json.loads(json.dumps(CFG))
    path = tmp_path / "j5mini.json"
    path.write_text(json.dumps(cfg), encoding="utf-8")
    cfg["_path"] = str(path)
    core = j5mini.Core(cfg, None, None, clock=lambda: 0.0, log=lambda m: None)
    srv = j5mini.Server(core, threading.Lock(), cfg, j5mini.Cameras("/nonexistent", sendto=lambda d, p: None))
    base = j5mini.settings_from_config(cfg)
    assert len(base["demo"]) == len(j5mini.DEMO_DEFAULT)
    steps = [{"angles": [95, 90, 90, 90, 90, 90], "speed_pct": 25, "profile": "BB"}]
    out = srv.save_config({**base, "demo": steps})
    assert out["ok"], out
    assert json.loads(path.read_text(encoding="utf-8"))["settings"]["demo"] == steps
    assert j5mini.demo_lines(cfg) == ["SETPOSE 93 85 83 90 90 98 25 BB"]
    bad = srv.save_config({**base, "demo": [{"angles": [1, 2, 3], "speed_pct": 25, "profile": "BB"}]})
    assert not bad["ok"] and "passo 1" in bad["error"]
    bad = srv.save_config({**base, "demo": [{"angles": [90] * 6, "speed_pct": 0, "profile": "BB"}]})
    assert not bad["ok"] and "velocità" in bad["error"]


class StuckWs:
    """A client whose socket never drains (the headset asleep): send() waits for ever."""
    def __init__(self):
        self.closed = False

    async def send(self, data):
        await asyncio.Event().wait()

    async def close(self, code=1000, reason=""):
        self.closed = True


def test_a_client_that_stops_reading_is_dropped_and_the_others_still_get_telemetry():
    """27/10/2026 on the robot: the Quest app paused with 87 KB queued on its socket and j5mini's broadcast waited
    for it for 6 minutes: no telemetry, no dashboard, no STOP. Now a send longer than 0.5 s drops that client."""
    cfg = json.loads(json.dumps(CFG))
    core = j5mini.Core(cfg, None, None, clock=lambda: 0.0, log=lambda m: None)
    srv = j5mini.Server(core, threading.Lock(), cfg, j5mini.Cameras("/nonexistent", sendto=lambda d, p: None))
    good, stuck = FakeWs(), StuckWs()
    srv.clients.update([good, stuck])

    async def run():
        t0 = time.monotonic()
        await srv.broadcast({"type": "telemetry", "n": 1})
        dt = time.monotonic() - t0
        await asyncio.sleep(0.05)                      # the close task
        await srv.broadcast({"type": "telemetry", "n": 2})
        return dt
    dt = asyncio.run(run())
    assert dt < 1.5, dt
    assert stuck not in srv.clients and good in srv.clients and stuck.closed
    assert [m["n"] for m in good.sent] == [1, 2]


def test_boot_push_ends_with_the_park_pose_as_init_pose(lib):
    """27/10/2026: at power-on the servos snapped to HOME. After the configuration j5mini declares the PARK pose
    (where the arm is left when switched off) with SETPOSE_INIT; HOME then starts from there as a trajectory."""
    cfg = json.loads(json.dumps(CFG))
    cfg["settings"]["dirs"] = [1, -1, 1, 1, -1, 1]
    r = Rig(lib, cfg)
    r.run(1500)
    cmds = [e["cmd"] for e in r.responses()]
    assert "SETPOSE_INIT 88 125 143 90 50 98" in cmds                # park [90,50,150,90,130,90] virtual -> physical
    assert cmds.index("SETPOSE_INIT 88 125 143 90 50 98") > cmds.index(CFG["boot_commands"][-1])
    assert all(e["ok"] for e in r.responses() if e["cmd"].startswith("SETPOSE_INIT"))
    assert r.sim.angles() == [88.0, 125.0, 143.0, 90.0, 50.0, 98.0] and r.sim.pwm()[0] == [0] * 6
    # a j5mini restart with the firmware already running (fallback push) must not touch what the firmware knows
    r.core.boot_queue = []
    r.core._start_boot_push(booted=False)
    assert not any(l.startswith("SETPOSE_INIT") for l in r.core.boot_queue)
    # a firmware reset mid-session: the last pose seen in telemetry, not PARK (the arm stayed about there)
    r.core.last_pose = [88, 85, 83, 90, 90, 98]
    r.core._start_boot_push(booted=True)
    assert "SETPOSE_INIT 88 85 83 90 90 98" in r.core.boot_queue

