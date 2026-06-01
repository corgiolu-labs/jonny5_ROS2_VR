"""Mock SPI worker for hardware-free dry-run of the native JONNY5 SPI driver.

It mimics the public surface of ``controller.spi_dataplane.spi_worker.SPIWorker``
(``is_open``, ``open()``, ``close()``, ``transfer()``, ``_frame_len``) and returns
synthetic but *protocol-valid* J5 TELEMETRY (0x01) frames so that the real
``J5VRSPIBridge`` RX parser produces realistic telemetry. No ``spidev`` required.

The synthetic frame layout matches exactly what ``J5VRSPIBridge.send_setpoint_once``
parses (see ``j5vr_spi_bridge.py``):

    frame[0:2]   = b"J5"
    frame[2]     = 0x01  protocol version
    frame[3]     = 0x01  frame_type = TELEMETRY
    frame[4:6]   = sequence (BE)
    frame[6]     = 64    payload_len
    frame[7]     = 0x00  flags
    payload (frame[8:62], 54 B):
        [28]      imu_valid (1)
        [29:33]   imu_q_w  float32 BE
        [33:37]   imu_q_x
        [37:41]   imu_q_y
        [41:45]   imu_q_z
        [45:51]   servo deg B,S,G,Y,P,R (uint8, 0..180)
        [51:54]   imu_sample_counter (24-bit BE)
    frame[62:64] = rt_loop_period_us (BE uint16)
"""

from __future__ import annotations

import math
import struct


def build_telemetry_frame(
    *,
    quat: tuple[float, float, float, float],
    servo_deg: tuple[int, int, int, int, int, int],
    sample_counter: int,
    sequence: int = 0,
    rt_loop_period_us: int = 1000,
) -> bytes:
    """Assemble a protocol-valid 64-byte J5 TELEMETRY frame."""
    frame = bytearray(64)
    frame[0] = 0x4A  # 'J'
    frame[1] = 0x35  # '5'
    frame[2] = 0x01  # protocol version
    frame[3] = 0x01  # frame_type = TELEMETRY
    struct.pack_into(">H", frame, 4, sequence & 0xFFFF)
    frame[6] = 64
    frame[7] = 0x00

    payload = bytearray(54)
    payload[28] = 1  # imu_valid
    w, x, y, z = quat
    struct.pack_into(">f", payload, 29, float(w))
    struct.pack_into(">f", payload, 33, float(x))
    struct.pack_into(">f", payload, 37, float(y))
    struct.pack_into(">f", payload, 41, float(z))
    for i, deg in enumerate(servo_deg):
        payload[45 + i] = max(0, min(180, int(deg)))
    sc = int(sample_counter) & 0xFFFFFF
    payload[51] = (sc >> 16) & 0xFF
    payload[52] = (sc >> 8) & 0xFF
    payload[53] = sc & 0xFF

    frame[8:62] = payload
    struct.pack_into(">H", frame, 62, int(rt_loop_period_us) & 0xFFFF)
    return bytes(frame)


def build_status_frame(
    *,
    diag_mask: int,
    mode: int,
    heartbeat: int,
    sequence: int = 0,
) -> bytes:
    """Assemble a protocol-valid 64-byte J5 STATUS (0x03) frame.

    Mirrors the firmware ``j5vr_fill_tx_telemetry`` layout (payload offsets):
      46-47 heartbeat (BE), 48 mode, 50-51 diag_mask (BE):
      bit0=deadman, 1=input_active, 2=armed, 3=freeze, 4=guard_seen.
    """
    frame = bytearray(64)
    frame[0] = 0x4A  # 'J'
    frame[1] = 0x35  # '5'
    frame[2] = 0x01  # protocol version
    frame[3] = 0x03  # frame_type = STATUS
    struct.pack_into(">H", frame, 4, sequence & 0xFFFF)
    frame[6] = 64
    frame[7] = 0x00

    payload = bytearray(54)
    payload[46] = (int(heartbeat) >> 8) & 0xFF
    payload[47] = int(heartbeat) & 0xFF
    payload[48] = int(mode) & 0xFF
    payload[50] = (int(diag_mask) >> 8) & 0xFF
    payload[51] = int(diag_mask) & 0xFF
    frame[8:62] = payload
    return bytes(frame)


class MockSpiWorker:
    """Drop-in replacement for ``SPIWorker`` that fabricates telemetry frames.

    Responds to a J5VR/J5IK setpoint (or any non-status frame) with a TELEMETRY
    (0x01) frame, and to a STATUS request (0x03) with a STATUS (0x03) frame
    carrying a synthetic diag mask — mirroring the real firmware dispatch.
    """

    def __init__(self, *, rt_loop_period_us: int = 1000) -> None:
        self._is_open = False
        self._frame_len = 64
        self._rt_loop_period_us = int(rt_loop_period_us)
        self._tick = 0
        # Synthetic firmware diag: deadman+input+armed set (bits 0,1,2).
        self._diag_mask = 0x0007

    # --- SPIWorker-compatible surface -------------------------------------
    def open(self) -> None:
        self._is_open = True

    def close(self) -> None:
        self._is_open = False

    @property
    def is_open(self) -> bool:
        return self._is_open

    def transfer(self, tx: bytes) -> bytes:
        """Return a synthetic RX frame matching the firmware dispatch.

        A STATUS request (tx frame_type 0x03) yields a STATUS frame with the
        synthetic diag; anything else yields a TELEMETRY frame.
        """
        tx_type = tx[3] if len(tx) > 3 else 0
        if tx_type == 0x03:
            self._tick += 1
            return build_status_frame(
                diag_mask=self._diag_mask,
                mode=2,
                heartbeat=self._tick & 0xFFFF,
                sequence=(tx[4] << 8 | tx[5]) if len(tx) >= 6 else self._tick,
            )
        t = self._tick / 50.0
        quat_yaw = 0.25 * math.sin(t * 0.5)
        half = quat_yaw / 2.0
        quat = (math.cos(half), 0.0, 0.0, math.sin(half))
        servo = (
            int(90 + 20 * math.sin(t * 0.6)),
            int(90 + 15 * math.sin(t * 0.7 + 0.5)),
            int(90 + 12 * math.sin(t * 0.8 + 1.0)),
            int(90 + 10 * math.sin(t * 1.2)),
            int(90 + 8 * math.sin(t * 1.1 + 0.3)),
            int(90 + 7 * math.sin(t * 1.4 + 0.8)),
        )
        seq = tx[4] << 8 | tx[5] if len(tx) >= 6 else self._tick
        frame = build_telemetry_frame(
            quat=quat,
            servo_deg=servo,
            sample_counter=self._tick,
            sequence=seq,
            rt_loop_period_us=self._rt_loop_period_us,
        )
        self._tick += 1
        return frame

    def __enter__(self) -> "MockSpiWorker":
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> bool:
        self.close()
        return False
