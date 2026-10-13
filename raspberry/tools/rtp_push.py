#!/usr/bin/env python3
"""rtp_push.py — Annex B H.264 on stdin -> RFC 6184 RTP over UDP, no server in between.

Direct video path for the headset: rpicam-vid writes encoded frames to a pipe, this script
packetizes each access unit (single NAL or FU-A fragments) and sends it straight to the
viewer app (jonny5-quest-viewer in "udp" mode), which listens on the given port.

Latency design:
  - no parsing delay: a frame is sent as soon as its bytes arrive on the pipe. rpicam-vid
    writes one frame per fflush (--flush); the reader coalesces the writes of one frame with a
    short quiet-time (COALESCE_S) instead of waiting for the next frame's start code.
  - access-unit boundaries are also detected from the bitstream (SPS/PPS/AUD, or a slice with
    first_mb_in_slice == 0 after another slice), so two frames in one read are still split.
  - SPS/PPS are cached and re-sent in front of every IDR frame, so the viewer can start at
    any keyframe even if the encoder does not repeat the headers (rpicam-vid --inline does).
  - RTP timestamp = send time on a 90 kHz clock; marker bit on the last packet of each frame.

Usage:  rpicam-vid ... --inline --flush -o - | rtp_push.py --host 10.42.0.10 --port 5000
"""
import argparse
import fcntl
import os
import select
import socket
import struct
import sys
import time

COALESCE_S = 0.0004          # quiet time that closes a frame read from the pipe (writer uses stdbuf -o0:
                             # one write per frame, so this only catches the rare split write)
MAX_COALESCE_ROUNDS = 8
F_SETPIPE_SZ = 1031          # Linux fcntl: enlarge the stdin pipe so a frame never blocks the writer


def iter_nals(buf: bytes):
    """Yield NAL unit payloads (without start code) found in buf, in order."""
    n = len(buf)
    i = 0
    starts = []
    while True:
        j = buf.find(b"\x00\x00\x01", i)
        if j < 0:
            break
        starts.append(j + 3)
        i = j + 3
    for k, s in enumerate(starts):
        e = n if k + 1 == len(starts) else starts[k + 1] - 3
        # strip the optional 4th zero byte of the next start code (00 00 00 01)
        while e > s and buf[e - 1] == 0 and k + 1 < len(starts):
            e -= 1
        if e > s:
            yield buf[s:e]


def is_new_au(nal: bytes, au_has_slice: bool) -> bool:
    t = nal[0] & 0x1F
    if t in (7, 8, 9):                  # SPS, PPS, AUD always open a new access unit
        return au_has_slice
    if t in (1, 5) and len(nal) > 1:    # slice with first_mb_in_slice == 0 (ue(v) '1' bit)
        return au_has_slice and (nal[1] & 0x80) != 0
    return False


class RtpSender:
    def __init__(self, host: str, port: int, pt: int, mtu: int, ssrc: int):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4 << 20)
        try:
            self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_TOS, 0xB8)   # DSCP EF: Wi-Fi voice queue
        except OSError:
            pass
        self.addr = (host, port)
        self.pt = pt
        self.payload_max = mtu - 12
        self.ssrc = ssrc
        self.seq = int.from_bytes(os.urandom(2), "big")
        self.packets = 0
        self.bytes = 0

    def _send(self, payload: bytes, ts: int, marker: bool):
        hdr = struct.pack("!BBHII", 0x80, self.pt | (0x80 if marker else 0), self.seq & 0xFFFF, ts, self.ssrc)
        self.sock.sendto(hdr + payload, self.addr)
        self.seq += 1
        self.packets += 1
        self.bytes += 12 + len(payload)

    def send_au(self, nals, ts: int):
        last = len(nals) - 1
        for idx, nal in enumerate(nals):
            is_last_nal = idx == last
            if len(nal) <= self.payload_max:
                self._send(nal, ts, is_last_nal)
                continue
            # FU-A fragmentation (RFC 6184 5.8)
            hdr = nal[0]
            fu_ind = bytes([(hdr & 0xE0) | 28])
            typ = hdr & 0x1F
            pos = 1
            first = True
            chunk = self.payload_max - 2
            while pos < len(nal):
                end = min(pos + chunk, len(nal))
                is_end = end == len(nal)
                fu_hdr = bytes([(0x80 if first else 0) | (0x40 if is_end else 0) | typ])
                self._send(fu_ind + fu_hdr + nal[pos:end], ts, is_last_nal and is_end)
                pos = end
                first = False


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", required=True, help="viewer IP (the headset / phone on the hotspot)")
    ap.add_argument("--port", type=int, required=True, help="viewer UDP port (5000 cam0, 5002 cam1)")
    ap.add_argument("--pt", type=int, default=96)
    ap.add_argument("--mtu", type=int, default=1400, help="max RTP packet size incl. 12-byte header")
    ap.add_argument("--ssrc", type=lambda v: int(v, 0), default=None)
    ap.add_argument("--stats", type=float, default=5.0, help="print stats every N s on stderr (0 = off)")
    a = ap.parse_args()

    fd = sys.stdin.fileno()
    try:
        fcntl.fcntl(fd, F_SETPIPE_SZ, 1 << 20)
    except OSError:
        pass
    os.set_blocking(fd, True)

    sender = RtpSender(a.host, a.port, a.pt, a.mtu, a.ssrc if a.ssrc is not None else int.from_bytes(os.urandom(4), "big"))
    sps = pps = None
    frames = 0
    splits = 0
    tails = 0
    t_stats = time.monotonic()
    pending_nonvcl = []   # SPS/PPS/SEI seen before the next slice

    def emit(nals):
        nonlocal frames
        if not nals:
            return
        first_vcl = next((n for n in nals if (n[0] & 0x1F) in (1, 5)), None)
        if first_vcl is not None and (first_vcl[0] & 0x1F) == 5 and sps is not None and pps is not None \
                and not any((n[0] & 0x1F) == 7 for n in nals):
            nals = [sps, pps] + nals
        ts = int(time.monotonic() * 90000) & 0xFFFFFFFF
        sender.send_au(nals, ts)
        frames += 1

    while True:
        data = os.read(fd, 1 << 20)
        if not data:
            break
        rounds = 0
        while rounds < MAX_COALESCE_ROUNDS and select.select([fd], [], [], COALESCE_S)[0]:
            more = os.read(fd, 1 << 20)
            if not more:
                break
            data += more
            rounds += 1

        if not (data.startswith(b"\x00\x00\x01") or data.startswith(b"\x00\x00\x00\x01")):
            tails += 1   # continuation of a NAL already sent: previous frame went out truncated
        au = list(pending_nonvcl)
        pending_nonvcl = []
        au_has_slice = any((n[0] & 0x1F) in (1, 5) for n in au)
        for nal in iter_nals(data):
            t = nal[0] & 0x1F
            if t == 7:
                sps = nal
            elif t == 8:
                pps = nal
            if is_new_au(nal, au_has_slice):
                emit(au)
                splits += 1
                au = []
                au_has_slice = False
            au.append(nal)
            if t in (1, 5):
                au_has_slice = True
        if au_has_slice:
            emit(au)
        else:
            pending_nonvcl = au   # headers only (e.g. SPS/PPS write split from the IDR): wait for the slice

        if a.stats and time.monotonic() - t_stats >= a.stats:
            dt = time.monotonic() - t_stats
            sys.stderr.write("rtp_push %s:%d  %.1f fps  %d pkt/s  %.0f kbit/s  au-splits %d  split-tails %d\n" % (
                a.host, a.port, frames / dt, sender.packets / dt, sender.bytes * 8 / dt / 1000, splits, tails))
            sys.stderr.flush()
            frames = 0
            sender.packets = 0
            sender.bytes = 0
            t_stats = time.monotonic()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
