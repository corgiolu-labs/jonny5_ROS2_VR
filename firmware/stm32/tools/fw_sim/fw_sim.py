#!/usr/bin/env python3
"""Build and drive the JONNY5 firmware simulator (the real firmware C sources on a PC).

    python tools/fw_sim/fw_sim.py build            # -> tools/fw_sim/build/fw_sim.{dll,so}
    python -m pytest tools/fw_sim -q               # behaviour tests (test_fw_sim.py)

Compiled from src/: state machine, E-STOP, RT loop step, J5 protocol + SPI frame logic,
UART control plane, actuation, HEAD, SETPOSE, manual, quaternion utils, servo_control.
Not compiled (hardware only): hal_spi_slave.c (HAL + DMA), imu.c/bno085.c, main.c, fatal_diag.c.
"""
import ctypes
import os
import pathlib
import shutil
import struct
import subprocess
import sys
import tempfile
import itertools

HERE = pathlib.Path(__file__).resolve().parent
FW = HERE.parents[1]                       # firmware/stm32
SRC = FW / "src"
BUILD = HERE / "build"
LIB = BUILD / ("fw_sim.dll" if os.name == "nt" else "libfw_sim.so")

SOURCES = [
    "core/state_machine.c", "core/estop.c",
    "spi/j5_protocol.c", "spi/spi_frames.c",
    "uart/uart_control.c",
    "servo/servo_control.c", "servo/j5vr_actuation.c", "servo/j5vr_head.c",
    "servo/j5vr_setpose.c", "servo/j5vr_manual.c", "servo/j5vr_quat_utils.c",
]


def build(padded128=True, out=LIB, src=SRC, extra_sources=(), spi_frames_from=None):
    """Build the simulator library. `src` may be another firmware tree (e.g. the original
    firmware, for differential tests); its spi_frames.c can come from `spi_frames_from`."""
    BUILD.mkdir(exist_ok=True)
    out = pathlib.Path(out)
    cc = os.environ.get("CC", "gcc")
    srcs = []
    for s in list(SOURCES) + list(extra_sources):
        if s == "spi/spi_frames.c" and spi_frames_from is not None:
            srcs.append(str(pathlib.Path(spi_frames_from) / s))
        else:
            srcs.append(str(pathlib.Path(src) / s))
    rt = BUILD / f"sim_rt_{out.stem}.c"
    rt.write_text((HERE / "sim_rt.c").read_text(encoding="utf-8").replace(
        '#include "../../src/core/rt_loop.c"', f'#include "{(pathlib.Path(src) / "core/rt_loop.c").as_posix()}"'),
        encoding="utf-8")
    incs = ["-I", str(HERE / "include"), "-I", str(src)]
    if spi_frames_from is not None:
        incs += ["-I", str(spi_frames_from)]
    cmd = [cc, "-std=gnu11", "-O1", "-g", "-shared", "-fPIC", "-Wall", "-Wno-unused-function",
           "-Wno-unused-variable", "-Wno-unused-but-set-variable",
           f"-DCONFIG_J5_CANONICAL_PADDED_128_MODE={1 if padded128 else 0}",
           *incs, "-o", str(out)] + srcs + [str(HERE / "sim_hw.c"), str(rt), "-lm"]
    subprocess.run(cmd, check=True)
    return out


_counter = itertools.count()


class Sim:
    """One fresh firmware instance (each instance loads its own copy of the library,
    so the firmware's static state starts from zero)."""

    def __init__(self, lib_path=LIB, verbose=False):
        tmp = pathlib.Path(tempfile.gettempdir()) / f"fw_sim_{os.getpid()}_{next(_counter)}{lib_path.suffix}"
        shutil.copyfile(lib_path, tmp)
        self.lib = ctypes.CDLL(str(tmp))
        L = self.lib
        L.sim_spi_transfer.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_int]
        L.sim_uart_write.argtypes = [ctypes.c_char_p]
        L.sim_uart_read.argtypes = [ctypes.c_char_p, ctypes.c_int]
        L.sim_console_read.argtypes = [ctypes.c_char_p, ctypes.c_int]
        L.sim_set_imu.argtypes = [ctypes.c_int] + [ctypes.c_float] * 4
        L.sim_now_ms.restype = ctypes.c_int64
        L.sim_set_verbose(1 if verbose else 0)
        self.frame_len = L.sim_frame_len()
        L.sim_boot()

    # --- time / io ---
    def run(self, ms):
        self.lib.sim_run_ms(int(ms))

    def spi(self, mosi):
        out = ctypes.create_string_buffer(len(mosi))
        self.lib.sim_spi_transfer(bytes(mosi), out, len(mosi))
        return out.raw

    def uart(self, line):
        self.lib.sim_uart_write(line.encode() + b"\n")

    def uart_read(self):
        buf = ctypes.create_string_buffer(1 << 15)
        self.lib.sim_uart_read(buf, len(buf))
        return buf.value.decode(errors="replace")

    def command(self, line, ms=30):
        """Send one UART line, run the firmware, return the reply lines."""
        self.uart(line)
        self.run(ms)
        return [l for l in self.uart_read().split("\n") if l]

    def console(self):
        buf = ctypes.create_string_buffer(1 << 16)
        self.lib.sim_console_read(buf, len(buf))
        return buf.value.decode(errors="replace")

    # --- hardware ---
    def estop(self, pressed):
        self.lib.sim_set_gpio(6, 1 if pressed else 0)       # PC6: high = stop

    def imu(self, valid, q=(1.0, 0.0, 0.0, 0.0)):
        self.lib.sim_set_imu(1 if valid else 0, *q)

    def pwm(self):
        pulse = (ctypes.c_uint32 * 6)()
        period = (ctypes.c_uint32 * 6)()
        self.lib.sim_pwm(pulse, period)
        return list(pulse), list(period)

    def angles(self):
        a = (ctypes.c_float * 6)()
        self.lib.sim_servo_angles(a)
        return [round(x, 3) for x in a]

    def state(self):
        return ("SAFE", "IDLE", "STOPPED")[self.lib.sim_state()]

    def now_ms(self):
        return self.lib.sim_now_ms()


# ---------------------------------------------------------------- J5 frames (Pi side)

def crc16_ccitt_false(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def j5_frame(ftype, seq, payload=b"", version=1):
    """64-byte J5 frame as the Pi builds it (j5vr_frame.py / j5_protocol_v2.py)."""
    p = bytes(payload).ljust(54, b"\0")[:54]
    f = bytearray(b"J5" + bytes([version, ftype]) + struct.pack(">H", seq & 0xFFFF) + bytes([64, 1 if version == 2 else 0]) + p + b"\0\0")
    if version == 2:
        struct.pack_into(">H", f, 62, crc16_ccitt_false(f[:62]))
    return bytes(f)


def j5vr_payload(mode=2, joy_x=0, joy_y=0, pitch=0, yaw=0, intensity=0, grip=0, hb=0,
                 quat=(1.0, 0.0, 0.0, 0.0), btn_l=0, btn_r=0, mode5_bsg_cdeg=None):
    p = bytearray(54)
    p[0] = mode
    struct.pack_into(">hhhh", p, 1, joy_x, joy_y, pitch, yaw)
    p[9] = intensity
    p[10] = grip
    struct.pack_into(">H", p, 11, hb & 0xFFFF)
    struct.pack_into(">ffff", p, 16, *quat)
    struct.pack_into(">HH", p, 32, btn_l, btn_r)
    if mode5_bsg_cdeg is not None:
        p[36] = ord("I")
        p[37] = 1
        struct.pack_into(">hhh", p, 40, *mode5_bsg_cdeg)
    return bytes(p)


def pad(frame, length):
    return frame + bytes(length - len(frame))


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "build":
        print(build())
    else:
        print(__doc__)
