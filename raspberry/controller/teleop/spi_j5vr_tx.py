#!/usr/bin/env python3
"""
NOTE [RPI-SAFE-REFACTOR-PHASE1]
- Modulo analizzato, nessuna modifica funzionale.
- Marcatura delle funzioni CORE / UTILITY / DIAGNOSTIC / LEGACY.
- Obiettivo: documentazione interna per futura FASE 2.

SPI J5VR TX — Invio frame J5VR verso STM32 via SPI DATA PLANE.

Non cambia il comportamento esterno (service/systemd): continua a leggere l'intent
da `shared_state.py` (file IPC) e a trasmettere frame 64 B su `/dev/spidev0.0`.

Implementazione rifattorizzata per usare i moduli in `spi_dataplane/`:
- `SPIWorker` (accesso SPI reale)
- `J5VRSPIBridge` (bridge logico: state -> frame -> transfer)

Nota: eseguire con PYTHONPATH alla root (raspberry5 sul Pi):
  PYTHONPATH=/home/jonny5/raspberry5 python3 controller/teleop/spi_j5vr_tx.py

Nota tecnica:
- questo sender esporta frame utili verso STM32 a circa 100 Hz;
- il loop real-time STM32 resta separatamente a 1 kHz.
"""

import logging
import sys
import time

from controller.spi_dataplane.spi_worker import SPIWorker
from controller.spi_dataplane.j5vr_spi_bridge import J5VRSPIBridge
from controller.teleop import shared_state

# Periodo invio 100 Hz — double-buffer SPI, bilancia latenza e overhead CPU.
SEND_PERIOD = 1.0 / 100.0

# Log minimale: una riga ogni N frame (circa 1 riga ogni 0.5 s a 100 Hz)
LOG_EVERY_N = 50

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("spi_j5vr_tx")


def run_spi_loop():
    """Loop di invio: legge latest intent, costruisce frame, invia SPI, logga RX (minimale)."""
    frame_count = 0

    try:
        with SPIWorker(device="/dev/spidev0.0", mode=0, max_speed_hz=1_000_000) as spi:
            bridge = J5VRSPIBridge(spi_worker=spi, state_provider=shared_state)

            while True:
                rx = bridge.send_setpoint_once()
                frame_count += 1

                if frame_count % LOG_EVERY_N == 0 and rx:
                    header_ok = rx[0:2] == b"J5"
                    frame_type_rx = rx[3] if len(rx) > 3 else 0
                    logger.info(
                        "SPI: tx_frames=%d seq=%d rx_header_J5=%s rx_frame_type=0x%02x",
                        frame_count,
                        bridge.sequence_counter,
                        header_ok,
                        frame_type_rx,
                    )

                time.sleep(SEND_PERIOD)
    except KeyboardInterrupt:
        logger.info("Interruzione utente")
    except Exception as e:
        logger.error("Errore fatale SPI J5VR TX: %s", e, exc_info=True)
        raise


if __name__ == "__main__":
    try:
        run_spi_loop()
    except Exception:
        sys.exit(1)
