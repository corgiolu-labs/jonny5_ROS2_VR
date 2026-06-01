"""
spi_dataplane — SPI DATA PLANE 1.0

SPI1 = DATA PLANE realtime tra Raspberry Pi e STM32.

Moduli implementati:
- j5vr_frame: Frame J5VR (64 byte) con layout identico a j5_frame_t C
- spi_worker: Worker SPI per accesso a /dev/spidev0.0
- j5vr_spi_bridge: Bridge logico tra shared_state e SPI DATA PLANE

Questa cartella è pensata per essere autonoma e deployment-ready: può essere
copiata integralmente sul Raspberry Pi come sottosistema dedicato al data plane
SPI (teleoperazione continua + telemetria).

Architettura JONNY5 v1.1.1 — SPI DATA PLANE 1.0
"""

from .j5vr_frame import (
    J5VRFrame,
    J5VRPayload,
    build_setpoint_frame,
    J5_PROTOCOL_FRAME_SIZE,
    J5_FRAME_TYPE_J5VR,
    J5_FRAME_TYPE_TELEMETRY,
)

from .spi_worker import SPIWorker

from .j5vr_spi_bridge import J5VRSPIBridge

__all__ = [
    "J5VRFrame",
    "J5VRPayload",
    "build_setpoint_frame",
    "SPIWorker",
    "J5VRSPIBridge",
    "J5_PROTOCOL_FRAME_SIZE",
    "J5_FRAME_TYPE_J5VR",
    "J5_FRAME_TYPE_TELEMETRY",
]
