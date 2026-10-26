"""Behaviour tests of the JONNY5 firmware on the PC simulator (real firmware C sources).

    cd firmware/stm32 && python -m pytest tools/fw_sim -q

1. Differential: one long scenario covering everything used in production runs on the
   minimal firmware AND on the original (merge 22b6a46 = flashed firmware + sub-degree,
   before any pruning), with 64-byte and 128-byte transfers; UART replies, SPI replies and PWM pulses must be identical.
   Set FW_SIM_ORIG_SRC to an extracted original firmware/stm32/src to enable it
   (tools/fw_sim/README.md); otherwise it is skipped.
2. Behaviour checks of the minimal firmware against the spec (docs in README).
3. SPI transfer length: 64 bytes is the production setting (prj.conf); the 128-byte option
   is still built and checked, with matched and mismatched masters.
"""
import os
import pathlib
import struct
import sys

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import fw_sim  # noqa: E402
from fw_sim import Sim, j5_frame, j5vr_payload, pad  # noqa: E402

GRIP = 0x0002          # buttons bit1 = grip; both hands = deadman
TRIG = 0x0001
BTN_A = 0x0010         # right A = slow speed (20 deg/s)


@pytest.fixture(scope="session")
def lib():
    """Production configuration: 64-byte SPI transfers (CONFIG_J5_CANONICAL_PADDED_128_MODE=n)."""
    return fw_sim.build(padded128=False, out=fw_sim.BUILD / ("fw_sim64" + fw_sim.LIB.suffix))


@pytest.fixture(scope="session")
def lib128():
    return fw_sim.build(padded128=True, out=fw_sim.BUILD / ("fw_sim" + fw_sim.LIB.suffix))


@pytest.fixture(scope="session")
def lib_orig():
    src = os.environ.get("FW_SIM_ORIG_SRC")
    if not src:
        pytest.skip("FW_SIM_ORIG_SRC not set (path of the original firmware/stm32/src)")
    libs = {}

    def get(padded128):
        if padded128 not in libs:
            name = "fw_sim_orig" + ("" if padded128 else "64") + fw_sim.LIB.suffix
            libs[padded128] = fw_sim.build(
                padded128=padded128, out=fw_sim.BUILD / name, src=pathlib.Path(src),
                extra_sources=["servo/pickplace.c", "servo/motion_planner.c", "core/hw_watchdog.c",
                               "spi/boundary_buffers.c", "spi/assist_v2_raw.c"],
                spi_frames_from=pathlib.Path(fw_sim.__file__).parent / "orig")   # frozen, with v2/J5IK
        return libs[padded128]
    return get


class Pi:
    """The master side as the Pi / board does it: one transfer every 10 ms (100 Hz)."""

    def __init__(self, sim, length=None, version=1):
        self.sim, self.len, self.ver = sim, length or sim.frame_len, version
        self.seq, self.hb = 0, 0
        self.replies = []

    def transfer(self, ftype, payload=b"", raw=None):
        self.seq = (self.seq + 1) & 0xFFFF
        f = raw if raw is not None else j5_frame(ftype, self.seq, payload, self.ver)
        r = self.sim.spi(pad(f, self.len) if len(f) < self.len else f[: self.len])
        self.replies.append(r)
        self.sim.run(10)
        return r

    def j5vr(self, ms, advance_hb=True, **kw):
        for _ in range(max(1, ms // 10)):
            if advance_hb:
                self.hb = (self.hb + 1) & 0xFFFF
            self.transfer(0x04, j5vr_payload(hb=self.hb, **kw))

    def status(self):
        return self.transfer(0x03)


def j5ik_payload(targets_cdeg, mode=6, flags=0x80, valid=1, hb=1, target_id=0):
    p = bytearray(54)
    p[0], p[1] = valid, flags
    struct.pack_into(">HH", p, 2, target_id, hb)
    p[6] = mode
    struct.pack_into(">6h", p, 8, *targets_cdeg)
    return bytes(p)


def quat_axis(axis, deg):
    import math
    h = math.radians(deg) / 2
    s = math.sin(h)
    return (math.cos(h), *(s if a == axis else 0.0 for a in "xyz"))


PI_BOOT_PUSH = [
    "#1 SET_OFFSETS 100 88 93 95 90 95",
    "#2 SET_VR_PARAMS 1 1 1 0.05 0.35 3 60 60 35 0.35 0.35 0.1 1 -1 1 1 0 2 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0",
    "#3 SET_JOINT_LIMITS 0 10 170", "#4 SET_JOINT_LIMITS 1 10 170", "#5 SET_JOINT_LIMITS 2 10 170",
    "#6 SET_JOINT_LIMITS 3 10 170", "#7 SET_JOINT_LIMITS 4 50 145", "#8 SET_JOINT_LIMITS 5 68 128",
    "#9 SET_PWM_CONFIG 50 500 2500 180 330 500 2500 180",
    "#11 HYBRID ENABLE",
]


def scenario(sim):
    """Everything the Pi / Quest / dashboard use, in one run. Returns a transcript."""
    log = []
    pi = Pi(sim)

    def snap(tag):
        pulses, periods = sim.pwm()
        log.append((tag, sim.now_ms(), sim.state(), tuple(pulses), tuple(periods), tuple(sim.angles())))

    def cmd(line, ms=30):
        out = sim.command(line, ms)
        log.append(("uart", line, tuple(out)))
        return out

    sim.imu(True, (1.0, 0.0, 0.0, 0.0))
    sim.run(50)
    log.append(("boot", sim.uart_read()))
    for line in PI_BOOT_PUSH:
        cmd(line)
    for line in ["STATUS?", "#42 STATUS?", "VERSION?", "SETPOSE 90 90 90 90 90", "SETPOSE 200 90 90 90 90 90 50 RTR5",
                 "SETPOSE 90 90 90 90 90 90 50 XYZ", "SET_OFFSETS 1 2 3", "SET_JOINT_LIMITS 4 150 60", "SET_PWM_CONFIG 50"]:
        cmd(line)
    # stream with advancing heartbeat -> automatic SAFE -> IDLE
    pi.j5vr(300, mode=2)
    snap("auto-idle")
    cmd("HOME")
    pi.j5vr(4000, mode=2)
    snap("home")
    # MANUAL: deadman + left stick X (YAW), then freeze, Y (PITCH), right stick (SPALLA/GOMITO), triggers (BASE), X/Y (ROLL)
    pi.j5vr(600, mode=2, joy_x=20000, btn_l=GRIP, btn_r=GRIP)
    snap("manual-yaw")
    pi.j5vr(400, mode=2, btn_l=GRIP, btn_r=GRIP)
    snap("manual-freeze")
    pi.j5vr(300, mode=2, joy_y=-25000, yaw=15000, pitch=-15000, btn_l=GRIP | TRIG, btn_r=GRIP | BTN_A)
    snap("manual-mix")
    pi.j5vr(300, mode=2, btn_l=GRIP | 0x0010, btn_r=GRIP, intensity=128)
    snap("manual-roll")
    pi.j5vr(200, mode=2, joy_x=30000)                       # no deadman: nothing moves
    snap("manual-nodeadman")
    # STATUS / TEST_ECHO / TELEMETRY requests, garbage
    for ft in (0x03, 0x02, 0x01, 0x07):
        pi.transfer(ft)
    pi.transfer(0, raw=b"\x55" * pi.len)
    pi.status()
    # HEAD
    pi.j5vr(100, mode=3, btn_l=GRIP, btn_r=GRIP)
    for axis, deg in (("y", 15), ("z", -20), ("x", 25), ("y", 0)):
        pi.j5vr(400, mode=3, quat=quat_axis(axis, deg), btn_l=GRIP, btn_r=GRIP)
        snap(f"head-{axis}{deg}")
    cmd("HEADZERO")
    pi.j5vr(300, mode=3, quat=quat_axis("y", 10), btn_l=GRIP, btn_r=GRIP)
    snap("head-after-zero")
    # HYBRID
    pi.j5vr(500, mode=4, quat=quat_axis("z", 12), yaw=20000, btn_l=GRIP, btn_r=GRIP | TRIG)
    snap("hybrid")
    # ASSIST with B/S/G targets
    pi.j5vr(800, mode=5, quat=quat_axis("x", -10), btn_l=GRIP, btn_r=GRIP, mode5_bsg_cdeg=(8000, 10000, 11000))
    snap("assist")
    # deadman released, neutral payload (what the board sends when the Quest is silent)
    pi.j5vr(300, mode=5, advance_hb=False)
    snap("neutral")
    # SETPOSE forms
    for line in ["SETPOSE 100 95 90 90 100 95 60 RTR3", "SETPOSE_T 95 90 95 92 98 96 800 BB",
                 "SETPOSE_T_HR 1005 905 955 905 1105 955 600 B-C-B", "SETPOSE_T_HR 1000 900 950 900 1100 950 10 RTR5",
                 "PARK", "TELEOPPOSE"]:
        cmd(line, 10)
        pi.j5vr(1500, mode=1)
        snap("pose:" + line.split()[0])
    cmd("RELAX_DIGITAL")
    snap("relax")
    # SPI silence -> SAFE after 500 ms
    sim.run(700)
    snap("spi-timeout")
    cmd("STATUS?")
    # E-STOP
    pi.j5vr(200, mode=2)
    sim.estop(True); sim.run(50)
    snap("estop")
    for line in ["ENABLE", "HOME", "SETPOSE_T 90 90 90 90 90 90 500 RTR5", "ESTOP?", "STATUS?"]:
        cmd(line)
    sim.estop(False); sim.run(50)
    for line in ["ESTOP?", "STATUS?", "SAFE"]:
        cmd(line)
    pi.j5vr(200, mode=2)
    snap("after-estop")
    sim.estop(True); sim.run(30)
    for line in ["ESTOP_BYPASS ON 1", "ESTOP?", "SAFE", "ENABLE", "ESTOP_BYPASS OFF", "ESTOP?"]:
        cmd(line)
    sim.estop(False); sim.run(30)
    cmd("SAFE")
    # mode 6 in a J5VR (the ROS 2 joint stream, removed): no automatic re-arm, outputs off even after ENABLE
    pi.j5vr(200, mode=6)
    snap("mode6-safe")
    cmd("ENABLE")
    pi.j5vr(300, mode=6, btn_l=GRIP, btn_r=GRIP, joy_x=20000)
    snap("mode6")
    pi.j5vr(200, mode=2)
    snap("leave-mode6")
    cmd("STOP")
    snap("stop")
    cmd("SAFE")
    log.append(("spi", tuple(r.hex() for r in pi.replies)))
    return log


def test_scenario_runs(lib):
    log = scenario(Sim(lib))
    replies = {e[1]: e[2] for e in log if e[0] == "uart"}
    assert replies["#2 SET_VR_PARAMS 1 1 1 0.05 0.35 3 60 60 35 0.35 0.35 0.1 1 -1 1 1 0 2 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0"] == ("#2 OK SET_VR_PARAMS",)
    states = {e[0]: e[2] for e in log if e[0] not in ("uart", "boot", "spi")}
    assert states["auto-idle"] == "IDLE" and states["spi-timeout"] == "SAFE"
    assert states["estop"] == "STOPPED" and states["stop"] == "STOPPED"


@pytest.mark.parametrize("padded128", [False, True], ids=["64B", "128B"])
def test_identical_to_original_firmware(padded128, lib, lib128, lib_orig):
    """The minimal firmware must answer and actuate exactly like the original one, at both lengths."""
    a = scenario(Sim(lib128 if padded128 else lib))
    b = scenario(Sim(lib_orig(padded128)))
    assert len(a) == len(b)
    for ea, eb in zip(a, b):
        assert ea == eb, f"differs at {ea[0]}: minimal={ea!r}\noriginal={eb!r}"


# ------------------------------------------------------------------ behaviour (minimal firmware)

def booted(lib):
    s = Sim(lib)
    s.run(50)
    assert s.uart_read().strip() == "BOOT_READY"
    return s


def test_boot_is_safe_with_outputs_off(lib):
    s = booted(lib)
    assert s.state() == "SAFE"
    pulses, periods = s.pwm()
    assert pulses == [0] * 6
    assert periods == [20000] * 4 + [3030, 3030]          # 50 Hz arm, 330 Hz wrist


def test_uart_seq_prefix_and_errors(lib):
    s = booted(lib)
    assert s.command("#7 STATUS?") == ["#7 STATUS:SAFE"]
    assert s.command("SET_OFFSETS 100 88 93 95 90 95") == ["OK SET_OFFSETS"]
    assert s.command("#3 SET_OFFSETS 1 2") == ["#3 ERR SET_OFFSETS_FMT"]
    assert s.command("FOO") == ["ERR UNKNOWN"]
    assert s.command("PP1 50") == ["ERR UNKNOWN"]            # pick&place removed (no hardware on the G474)
    assert s.command("HOME") == ["ERR BUSY"]                 # not IDLE
    assert s.command("HYBRID ENABLE") == ["OK HYBRID ENABLED"]


def test_reply_type_follows_request_and_lags_one_transfer(lib):
    s = booted(lib)
    pi = Pi(s)
    pi.j5vr(10, mode=2)                    # transfer 1: J5VR
    r1 = pi.status()                       # transfer 2: STATUS request, gets the reply to transfer 1
    r2 = pi.transfer(0x04, j5vr_payload(hb=99))   # gets the reply to the STATUS request
    assert r1[:2] == b"J5" and r1[3] == 0x01 and struct.unpack_from(">H", r1, 4)[0] == 1   # TELEMETRY, seq 1
    assert r2[:2] == b"J5" and r2[3] == 0x03 and struct.unpack_from(">H", r2, 4)[0] == 2   # STATUS, seq 2
    assert len(r1) == 64 and len(r2) == 64


def test_auto_idle_needs_advancing_heartbeat_and_spi_timeout_goes_safe(lib):
    s = booted(lib)
    pi = Pi(s)
    pi.j5vr(300, mode=2, advance_hb=False)        # heartbeat 0: stays SAFE
    assert s.state() == "SAFE"
    pi.j5vr(50, mode=2)
    assert s.state() == "IDLE"
    s.run(450)
    assert s.state() == "IDLE"
    s.run(200)                                    # > 500 ms without frames (checked every 100 ticks)
    assert s.state() == "SAFE"


def test_home_then_manual_yaw_with_deadman_only(lib):
    s = booted(lib)
    for line in PI_BOOT_PUSH:
        s.command(line)
    pi = Pi(s)
    pi.j5vr(100, mode=2)
    assert s.command("HOME", 10) == ["OK HOME"]
    pi.j5vr(4000, mode=2)
    out = s.uart_read() + "\n".join(s.command("STATUS?"))
    assert "SETPOSE_DONE" in out and "RELAX_DIGITAL pitch roll" in out
    assert s.angles() == [100.0, 88.0, 93.0, 95.0, 90.0, 95.0]   # offsets pushed by the Pi
    yaw0 = s.angles()[3]
    pi.j5vr(500, mode=2, joy_x=30000)                         # no deadman
    assert s.angles()[3] == yaw0
    pi.j5vr(500, mode=2, joy_x=30000, btn_l=GRIP, btn_r=GRIP)  # deadman held
    # wrist YAW capped by vel_digital = 35 deg/s (SET_VR_PARAMS f8, ints u9..u11 = 0)
    assert 16.5 <= s.angles()[3] - yaw0 <= 18.0
    pulses, _ = s.pwm()
    assert pulses[3] > 0


def test_estop_latches_stopped_and_blocks_motion_commands(lib):
    s = booted(lib)
    pi = Pi(s)
    pi.j5vr(100, mode=2)
    s.estop(True)
    s.run(30)
    assert s.state() == "STOPPED"
    assert "ESTOP" in s.uart_read()
    assert s.command("ENABLE") == ["ERR ESTOP_ACTIVE"]
    s.estop(False)
    s.run(30)
    assert "ESTOP_CLEAR" in s.uart_read()
    assert s.state() == "STOPPED"                 # no automatic resume
    assert s.command("SAFE") == ["OK SAFE"]
    pi.j5vr(50, mode=2)
    assert s.state() == "IDLE"


def test_ros2_frames_are_rejected(lib):
    """Protocol v2 (CRC) and J5IK were ROS 2 only and are gone: such frames are not valid frames
    (STATUS 0xEE in reply, they do not feed the SPI watchdog)."""
    s = booted(lib)
    pi = Pi(s)
    pi.j5vr(200, mode=2)
    assert s.state() == "IDLE"
    pi.ver = 2
    for _ in range(70):                                          # 700 ms of v2 frames only
        pi.transfer(0x04, j5vr_payload(mode=2, hb=pi.hb))
    assert s.state() == "SAFE"                                   # watchdog: no valid frame
    r = pi.replies[-1]
    assert r[3] == 0x03 and r[8] == 0xEE
    pi.ver = 1
    pi.transfer(0x05, j5ik_payload([12000, 9000, 9000, 9000, 9000, 9000]))
    r = pi.transfer(0x03)
    assert r[3] == 0x03 and r[8] == 0xEE                         # reply to the J5IK frame


def test_mode6_keeps_outputs_off_even_after_enable(lib):
    s = booted(lib)
    pi = Pi(s)
    pi.j5vr(300, mode=6)
    assert s.state() == "SAFE"                                   # mode 6 never re-arms by itself
    assert s.command("ENABLE") == ["OK ENABLE"]
    pi.j5vr(500, mode=6, btn_l=GRIP, btn_r=GRIP, joy_x=20000)
    assert list(s.pwm()[0]) == [0] * 6                           # servo outputs off


# ------------------------------------------------------------------ SPI transfer length

def valid_replies(replies):
    return sum(1 for r in replies if any(r[o:o + 2] == b"J5" and r[o + 6] == 64 for o in range(0, len(r) - 63)))


def test_128_byte_firmware_with_128_byte_master(lib128):
    s = booted(lib128)
    pi = Pi(s, 128)
    pi.j5vr(500, mode=2)
    assert s.state() == "IDLE"
    assert valid_replies(pi.replies[1:]) == len(pi.replies) - 1
    assert all(r[64:] == bytes(64) for r in pi.replies[1:])   # padded tail always zero


def test_64_byte_firmware_with_64_byte_master(lib):
    s = booted(lib)
    assert s.frame_len == 64
    pi = Pi(s, 64)
    pi.j5vr(500, mode=2)
    assert s.state() == "IDLE"
    assert valid_replies(pi.replies[1:]) == len(pi.replies) - 1
    r = pi.status()
    assert r[3] == 0x01 and len(r) == 64


def test_mismatched_length_master_64_on_128_firmware(lib128):
    """The slave DMA works in 128-byte halves: a 64-byte master gets a reply only every
    other transfer, half of the time the zero tail, and the J5VR in the second 64 bytes of
    each half is the one applied (the scan keeps the newest)."""
    s = booted(lib128)
    pi = Pi(s, 64)
    pi.j5vr(1000, mode=2)
    n_valid = valid_replies(pi.replies)
    assert n_valid <= len(pi.replies) // 2 + 1
    assert s.state() == "IDLE"          # commands still get through, telemetry rate halves


@pytest.mark.parametrize("mode", [3, 4])
def test_grips_pressed_again_take_the_new_head_as_reference(lib, mode):
    """Clutch (26/10/2026, user on the robot): turn the head with the grips released, press them again ->
    the arm must not jump to where the old head reference says; it moves from there with the new head."""
    s = booted(lib)
    for line in PI_BOOT_PUSH:
        s.command(line)
    pi = Pi(s)
    pi.j5vr(100, mode=2)
    assert s.command("HOME", 10) == ["OK HOME"]
    pi.j5vr(3000, mode=2)
    pi.j5vr(200, mode=mode, btn_l=GRIP, btn_r=GRIP)                       # engage, head straight
    pi.j5vr(1500, mode=mode, quat=quat_axis("y", 15), btn_l=GRIP, btn_r=GRIP)
    held = s.angles()
    assert max(abs(a - b) for a, b in zip(held, [100.0, 88.0, 93.0, 95.0, 90.0, 95.0])) > 5   # the wrist followed
    pi.j5vr(500, mode=mode, quat=quat_axis("y", 35))                      # grips released, head turns on
    pi.j5vr(1500, mode=mode, quat=quat_axis("y", 35), btn_l=GRIP, btn_r=GRIP)   # grips pressed again
    assert max(abs(a - b) for a, b in zip(s.angles(), held)) < 0.5, "jumped to the old head reference"
    pi.j5vr(1500, mode=mode, quat=quat_axis("y", 50), btn_l=GRIP, btn_r=GRIP)   # and it follows from here
    assert max(abs(a - b) for a, b in zip(s.angles(), held)) > 5


def motion_share_right_after_frames(lib, mode):
    """Smooth head motion, one J5VR frame every 10 ms (the Pi): share of each frame's motion that happens in its
    first 3 ms. Higher = the arm moves to every new target and then waits (steps), 0.3 = uniform."""
    import math
    s = booted(lib)
    for line in PI_BOOT_PUSH:
        s.command(line)
    pi = Pi(s)
    pi.j5vr(100, mode=2)
    s.command("HOME", 10)
    pi.j5vr(3000, mode=2)
    kw = dict(btn_l=GRIP, btn_r=GRIP)
    bsg = None
    pi.j5vr(300, mode=mode, **kw)
    blocks = []
    for k in range(150):
        t = k * 10
        a = math.radians(15 * math.sin(2 * math.pi * t / 2000.0)) / 2
        if mode == 5:
            bsg = tuple(int(100 * (v + 20 * math.sin(2 * math.pi * t / 2000.0))) for v in (100, 88, 93))
        pi.hb += 1
        pi.seq = (pi.seq + 1) & 0xFFFF
        s.spi(pad(j5_frame(0x04, pi.seq, j5vr_payload(mode=mode, hb=pi.hb, quat=(math.cos(a), 0.0, math.sin(a), 0.0),
                                                     mode5_bsg_cdeg=bsg, **kw)), s.frame_len))
        trace = []
        for _ in range(10):
            s.run(1)
            trace.append(s.angles())
        blocks.append(trace)
    shares = []
    for j in range(6):
        tot = early = 0.0
        prev = blocks[49][-1][j]
        for b in blocks[50:]:
            steps = [abs(b[0][j] - prev)] + [abs(b[n][j] - b[n - 1][j]) for n in range(1, 10)]
            tot += sum(steps)
            early += sum(steps[:3])
            prev = b[-1][j]
        if tot > 5.0:
            shares.append(early / tot)
    return shares


@pytest.mark.parametrize("mode", [3, 5])
def test_targets_are_interpolated_between_spi_frames(lib, mode):
    """26/10/2026, jerks in ASSIST on the robot: the RT loop followed each new frame within a few ms and then
    waited for the next one (steps every 10 ms). The head quaternion and the ASSIST B/S/G targets now glide
    from one frame to the next."""
    shares = motion_share_right_after_frames(lib, mode)
    assert shares, "nothing moved"
    assert max(shares) < 0.33, shares               # without it: 0.37-0.42; uniform = 0.30
