#!/usr/bin/env python3
"""
j5_uart.py -- send UART control commands to the STM32 without the web stack.

Needed when the ROS 2 stacks own the robot (ws_server not running), e.g. to
ENABLE after SAFE, read STATUS?, or run HOME/PARK:

    python3 raspberry/tools/j5_uart.py STATUS?
    python3 raspberry/tools/j5_uart.py SAFE ENABLE STATUS?
    python3 raspberry/tools/j5_uart.py --listen 10      # print unsolicited lines (ESTOP, SETPOSE_DONE...)

Stop jonny5-ws-teleop.service first: it owns /dev/serial0 exclusively.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import serial


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("commands", nargs="*", help="e.g. STATUS? SAFE ENABLE HOME PARK STOP")
    ap.add_argument("--port", default=os.environ.get("SERIAL_DEV", "/dev/serial0"))
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--timeout", type=float, default=1.0, help="seconds to wait for each reply")
    ap.add_argument("--listen", type=float, default=0.0, help="then print unsolicited lines for N s")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    rc = 0
    try:
        ser.reset_input_buffer()
        for seq, cmd in enumerate(args.commands, start=1):
            ser.write(f"#{seq} {cmd}\n".encode())
            deadline = time.monotonic() + args.timeout
            reply = None
            while time.monotonic() < deadline and reply is None:
                line = ser.readline().decode(errors="replace").strip()
                if not line:
                    continue
                if line.startswith(f"#{seq} "):
                    reply = line.split(" ", 1)[1]
                else:
                    print(f"  (unsolicited) {line}")
            print(f"{cmd:>12} -> {reply if reply is not None else 'NO REPLY'}")
            if reply is None or reply.startswith("ERR"):
                rc = 1
        end = time.monotonic() + args.listen
        while time.monotonic() < end:
            line = ser.readline().decode(errors="replace").strip()
            if line:
                print(f"  {line}")
    finally:
        ser.close()
    return rc


if __name__ == "__main__":
    sys.exit(main())
