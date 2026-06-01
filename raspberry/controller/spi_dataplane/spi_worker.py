"""
NOTE [RPI-SAFE-REFACTOR-PHASE1]
- Modulo analizzato, nessuna modifica funzionale.
- Marcatura delle funzioni CORE / UTILITY / DIAGNOSTIC / LEGACY.
- Obiettivo: documentazione interna per futura FASE 2.

spi_worker.py -- Worker SPI per accesso a /dev/spidev0.0 (SPI1 su Raspberry Pi).

Incapsula configurazione spidev e fornisce API semplice per trasferimenti SPI.
Allineato alla configurazione usata nei test SPI (mode=0, 1 MHz, 8 bit).

Architettura JONNY5 v1.1.1 -- SPI DATA PLANE 1.0
"""

import logging
import os
import time
from typing import Optional

try:
    import spidev
except ImportError:
    spidev = None

logger = logging.getLogger(__name__)

# Cache env var at import time — avoid os.environ lookup on every SPI transfer
_POST_XFER_DELAY_MS_ENV = os.environ.get("J5_SPI_POST_XFER_DELAY_MS", "")


class SPIWorker:
    """
    Worker SPI che incapsula connessione e trasferimenti verso /dev/spidev0.0.

    Configurazione allineata ai test SPI:
    - mode = 0 (CPOL=0, CPHA=0)
    - max_speed_hz = 1_000_000 (1 MHz)
    - bits_per_word = 8
    """

    def __init__(
        self,
        device: str = "/dev/spidev0.0",
        mode: int = 0,
        max_speed_hz: int = 1_000_000,
        bits_per_word: int = 8,
    ):
        """
        Inizializza worker SPI.

        Args:
            device: Path device SPI (default: /dev/spidev0.0)
            mode: SPI mode (default: 0)
            max_speed_hz: Velocita massima Hz (default: 1 MHz)
            bits_per_word: Bit per parola (default: 8)
        """
        self.device = device
        self.mode = mode
        self.max_speed_hz = max_speed_hz
        self.bits_per_word = bits_per_word
        self._spi: Optional[spidev.SpiDev] = None
        self._is_open = False
        self._frame_len = 64

    def open(self) -> None:
        """
        Apre il device SPI e configura parametri.

        Raises:
            RuntimeError: Se spidev non e installato o il device non esiste
            OSError: Se l'apertura del device fallisce
        """
        if spidev is None:
            raise RuntimeError("spidev non installato: pip install spidev")

        if not os.path.exists(self.device):
            raise RuntimeError(f"Device SPI non trovato: {self.device}")

        parts = self.device.replace("/dev/spidev", "").strip().split(".")
        if len(parts) != 2:
            raise ValueError(f"Path device SPI invalido: {self.device} (atteso formato /dev/spidevX.Y)")

        bus = int(parts[0])
        dev = int(parts[1])

        self._spi = spidev.SpiDev()
        self._spi.open(bus, dev)
        self._spi.mode = self.mode
        self._spi.max_speed_hz = self.max_speed_hz
        self._spi.bits_per_word = self.bits_per_word

        self._is_open = True
        try:
            from .spi_transport_mode import (
                log_transport_banner,
                spi_transport_frame_length,
            )

            self._frame_len = int(spi_transport_frame_length())
            log_transport_banner(self._frame_len)
        except Exception:
            self._frame_len = 64

        logger.info(
            "SPI aperto: %s (bus=%d, dev=%d, mode=%d, speed=%d Hz, bits=%d, frame_len=%d)",
            self.device,
            bus,
            dev,
            self.mode,
            self.max_speed_hz,
            self.bits_per_word,
            self._frame_len,
        )

    def close(self) -> None:
        """Chiude il device SPI."""
        if self._spi is not None:
            try:
                self._spi.close()
            except Exception as e:
                logger.warning("Errore durante chiusura SPI: %s", e)
            finally:
                self._spi = None
                self._is_open = False

    def transfer(self, tx: bytes) -> bytes:
        """
        Esegue un transfer full-duplex SPI.

        Args:
            tx: Buffer TX della dimensione di transport frame attiva

        Returns:
            bytes: Buffer RX ricevuto (stessa dimensione di tx)

        Raises:
            RuntimeError: Se SPI non e aperto
            ValueError: Se tx non e della dimensione corretta
        """
        if not self._is_open or self._spi is None:
            raise RuntimeError("SPI non aperto: chiamare open() prima di transfer()")

        if len(tx) != self._frame_len:
            raise ValueError(
                f"Buffer TX deve essere {self._frame_len} byte (ricevuti {len(tx)})"
            )

        rx_list = self._spi.xfer2(list(tx))
        rx = bytes(rx_list)

        # I transfer a 128 byte richiedono una breve pausa post-xfer sul Pi
        # per evitare di rileggere la stessa telemetria dal buffer slave.
        if self._frame_len == 128:
            delay_ms = float(_POST_XFER_DELAY_MS_ENV) if _POST_XFER_DELAY_MS_ENV else 20.0
            time.sleep(delay_ms / 1000.0)

        if len(rx) != self._frame_len:
            from .spi_transport_mode import spi_transport_mode_name

            raise RuntimeError(
                "SPI RX length mismatch: attesi %d B (mode=%s), ricevuti %d -- "
                "possibile accoppiamento errato firmware 64/128 vs Raspberry"
                % (
                    self._frame_len,
                    spi_transport_mode_name(),
                    len(rx),
                )
            )

        return rx

    def __enter__(self):
        """Context manager: apre SPI."""
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager: chiude SPI."""
        self.close()
        return False

    @property
    def is_open(self) -> bool:
        """Verifica se SPI e aperto."""
        return self._is_open
