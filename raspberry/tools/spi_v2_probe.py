#!/usr/bin/env python3
"""
spi_v2_probe.py -- check the J5 SPI link (protocol v1 and v2) without ROS.

Polls the STM32 with TELEMETRY requests at a fixed rate and reports link
quality and firmware state. It sends no motion command: TELEMETRY polls do
not change the setpoint, and the firmware goes to SAFE after 500 ms without
J5VR/J5IK frames anyway.

Run it on the Pi with the ROS 2 stack and the legacy SPI service STOPPED
(only one process may own /dev/spidev0.0):

    python3 raspberry/tools/spi_v2_probe.py                 # v2, 128 B, 100 Hz, 10 s
    python3 raspberry/tools/spi_v2_probe.py --protocol 1    # v1 regression check
    python3 raspberry/tools/spi_v2_probe.py --transfer-len 64 --rate 200 --seconds 30
    python3 raspberry/tools/spi_v2_probe.py --selftest      # no hardware: codec loopback

Exit code 0 = pass (see --max-crc-rate / --max-gap-rate), 1 = fail, 2 = no link.
"""

from __future__ import annotations

import argparse
import collections
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from controller.spi_dataplane import j5_protocol_v2 as v2  # noqa: E402


class SpidevLink:
    def __init__(self, device: str, speed_hz: int) -> None:
        import spidev  # only needed on the Pi

        bus, dev = (int(x) for x in device.replace("/dev/spidev", "").split("."))
        self._spi = spidev.SpiDev()
        self._spi.open(bus, dev)
        self._spi.mode = 0
        self._spi.bits_per_word = 8
        self._spi.max_speed_hz = speed_hz

    def transfer(self, tx: bytes) -> bytes:
        return bytes(self._spi.xfer2(list(tx)))

    def close(self) -> None:
        self._spi.close()


class LoopbackLink:
    """--selftest: a fake v2 firmware (one-transfer reply lag like the real slave)."""

    def __init__(self) -> None:
        self._pending = bytes(64)
        self._tx_seq = 0
        self._t0 = time.monotonic()

    def transfer(self, tx: bytes) -> bytes:
        reply = self._pending
        seq = (tx[4] << 8) | tx[5]
        self._tx_seq = (self._tx_seq + 1) & 0xFFFF
        if tx[2] == 2:
            self._pending = v2.build_telemetry_v2(
                sequence=seq, stm_time_ms=int((time.monotonic() - self._t0) * 1000),
                stm_tx_seq=self._tx_seq, fsm_state=0, status_flags=v2.ST_IMU_VALID,
                mode=0, joint_cdeg=[10000, 8800, 9300, 9500, 9000, 9500],
                rt_loop_period_us=1000, rt_step_us=45,
                diag_flags=v2.DG_IMU_PRESENT | v2.DG_IMU_ENABLED | v2.DG_POSE_KNOWN,
            )
        else:
            f = bytearray(64)
            f[0:2], f[2], f[3], f[6] = b"J5", 1, 0x01, 64
            struct.pack_into(">H", f, 4, seq)
            f[8 + 28] = 1
            f[8 + 45:8 + 51] = bytes([100, 88, 93, 95, 90, 95])
            struct.pack_into(">H", f, 62, 1000)
            self._pending = bytes(f)
        return reply + bytes(len(tx) - 64)

    def close(self) -> None:
        pass


def find_frame(rx: bytes) -> bytes | None:
    """Canonical 64 B frame inside a 64/128 B transfer (v2 CRC-valid or v1 header)."""
    for off in [0, 64] + [o for o in range(1, len(rx) - 63) if o != 64]:
        f = rx[off:off + 64]
        if len(f) < 64 or f[0:2] != b"J5":
            continue
        if f[2] == 2 and v2.crc_ok(f):
            return f
        if f[2] == 1 and f[6] == 64 and f[7] == 0:
            return f
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", default="/dev/spidev0.0")
    ap.add_argument("--speed-hz", type=int, default=1_000_000)
    ap.add_argument("--transfer-len", type=int, choices=(64, 128), default=128,
                    help="128 = firmware CONFIG_J5_CANONICAL_PADDED_128_MODE (default build)")
    ap.add_argument("--protocol", type=int, choices=(1, 2), default=2)
    ap.add_argument("--rate", type=float, default=100.0, help="transfers per second")
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--max-crc-rate", type=float, default=0.001, help="fail above this CRC error ratio")
    ap.add_argument("--max-gap-rate", type=float, default=0.001, help="fail above this seq-gap ratio")
    ap.add_argument("--selftest", action="store_true", help="no hardware: in-process fake firmware")
    args = ap.parse_args()

    link = LoopbackLink() if args.selftest else SpidevLink(args.device, args.speed_hz)
    period = 1.0 / args.rate
    seq = 0
    stats = collections.Counter()
    fsm_hist = collections.Counter()
    last_tx_seq = None
    first = last = None
    periods = []
    t_next = time.monotonic()
    t_end = t_next + args.seconds
    try:
        while time.monotonic() < t_end:
            if args.protocol == 2:
                tx = v2.build_request_v2(v2.FRAME_TYPE_TELEMETRY, seq)
            else:
                f = bytearray(64)
                f[0:2], f[2], f[3], f[6] = b"J5", 1, 0x01, 64
                struct.pack_into(">H", f, 4, seq)
                tx = bytes(f)
            rx = link.transfer(tx + bytes(args.transfer_len - 64))
            seq = (seq + 1) & 0xFFFF
            stats["sent"] += 1
            frame = find_frame(rx)
            if frame is None:
                if any(rx[i:i + 3] == b"J5\x02" for i in range(len(rx) - 2)):
                    stats["crc_error_pi"] += 1
                else:
                    stats["no_frame"] += 1
            elif frame[2] == 2 and frame[3] == v2.FRAME_TYPE_TELEMETRY_V2:
                t = v2.parse_telemetry_v2(frame)
                stats["v2_telemetry"] += 1
                if last_tx_seq is not None and t["stm_tx_seq"] == last_tx_seq:
                    stats["repeated"] += 1
                last_tx_seq = t["stm_tx_seq"]
                fsm_hist[t["fsm_state_name"] + (" +ESTOP" if t["estop_active"] else "")] += 1
                periods.append(t["rt_loop_period_us"])
                first = first or t
                last = t
            elif frame[2] == 2:
                stats[f"v2_type_0x{frame[3]:02x}" + (f"_err_0x{frame[8]:02x}" if frame[3] == 3 else "")] += 1
            elif frame[3] == 0x01:
                stats["v1_telemetry"] += 1
                period_us = struct.unpack_from(">H", frame, 62)[0]
                periods.append(period_us)
                last = {"servo_deg": list(frame[8 + 45:8 + 51]), "imu_valid": bool(frame[8 + 28])}
            else:
                stats[f"v1_type_0x{frame[3]:02x}"] += 1
            t_next += period
            delay = t_next - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                t_next = time.monotonic()
    finally:
        link.close()

    sent = stats["sent"]
    print(f"transfers: {sent} in {args.seconds:.1f}s ({sent / args.seconds:.1f}/s), "
          f"protocol v{args.protocol}, {args.transfer_len} B")
    for k, v in sorted(stats.items()):
        if k != "sent":
            print(f"  {k:24s} {v:7d}  ({100.0 * v / max(sent, 1):.2f}%)")
    if periods:
        good = [p for p in periods if p]
        if good:
            print(f"  STM32 RT loop period    avg {sum(good) / len(good):.0f} us, max {max(good)} us")
    if args.protocol == 1:
        ok = stats["v1_telemetry"] > 0.9 * sent
        print("v1 regression:", "PASS" if ok else "FAIL", "| last:", last)
        return 0 if ok else (2 if not stats["v1_telemetry"] else 1)
    if not last:
        print("NO TELEMETRY_V2: wrong transfer length, v1-only firmware, or SPI wiring/owner problem")
        return 2
    print("  FSM states:", dict(fsm_hist))
    print(f"  STM32 counters: rx_seq_gaps={last['rx_seq_gaps'] - first['rx_seq_gaps']} "
          f"crc_errors={last['crc_errors_stm'] - first['crc_errors_stm']} "
          f"rt_overruns={last['rt_overruns'] - first['rt_overruns']} (delta over the run)")
    print(f"  last joints (deg): {[round(j / 100, 2) for j in last['joint_cdeg']]}  "
          f"imu_valid={last['imu_valid']} mode={last['mode']}")
    crc_rate = stats["crc_error_pi"] / max(sent, 1)
    gap_rate = (last["rx_seq_gaps"] - first["rx_seq_gaps"]) / max(sent, 1)
    ok = crc_rate <= args.max_crc_rate and gap_rate <= args.max_gap_rate and stats["v2_telemetry"] > 0.9 * sent
    print("RESULT:", "PASS" if ok else "FAIL",
          f"(crc {crc_rate:.4f} <= {args.max_crc_rate}, gaps {gap_rate:.4f} <= {args.max_gap_rate}, "
          f"telemetry {stats['v2_telemetry']}/{sent})")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
