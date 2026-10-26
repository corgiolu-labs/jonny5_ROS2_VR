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
    cfg["ws"] = {"host": "127.0.0.1", "port": 28557}
    srv = j5mini.Server(r.core, lock, cfg)

    async def go():
        async with websockets.serve(srv.handler, "127.0.0.1", 28557):
            bt = asyncio.create_task(srv.broadcast_loop())
            async with websockets.connect("ws://127.0.0.1:28557") as ws:
                hb = 0
                seen = {}
                t_end = time.time() + 3.0
                await ws.send(json.dumps({"type": "uart", "cmd": "STATUS?"}))
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
