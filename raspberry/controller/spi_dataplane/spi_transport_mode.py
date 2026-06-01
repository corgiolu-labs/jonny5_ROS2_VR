"""
spi_transport_mode.py -- Selezione del solo envelope di trasporto SPI.

Policy di progetto:
- legacy_64: frame canonico J5 da 64 byte
- canonical_padded_128: stessi 64 byte canonici + tail 64..127 zero/ignored
"""

from __future__ import annotations

import logging
import os

J5_SPI_FRAME_LEN_LEGACY = 64
J5_SPI_FRAME_LEN_128 = 128


def _env_on(name: str) -> bool:
    v = os.environ.get(name, "").strip().lower()
    return v in ("1", "true", "yes", "on")


def canonical_128_pi_enabled() -> bool:
    return _env_on("J5_SPI_CANONICAL_128")


def spi_transport_mode_name() -> str:
    if canonical_128_pi_enabled():
        return "canonical_padded_128"
    return "legacy_64"


def spi_transport_frame_length() -> int:
    mode = spi_transport_mode_name()
    if mode == "legacy_64":
        return J5_SPI_FRAME_LEN_LEGACY
    return J5_SPI_FRAME_LEN_128


def _is_valid_canonical_j5_frame_at(rx: bytes, off: int) -> bool:
    if off < 0 or off + J5_SPI_FRAME_LEN_LEGACY > len(rx):
        return False
    frame = rx[off : off + J5_SPI_FRAME_LEN_LEGACY]
    if frame[0:2] != b"J5":
        return False
    if frame[2] != 0x01:
        return False
    if frame[6] != J5_SPI_FRAME_LEN_LEGACY:
        return False
    if frame[7] != 0x00:
        return False
    if frame[3] not in (0x01, 0x02, 0x03, 0x04, 0x05):
        return False
    return True


def extract_canonical_frame64_from_transport_rx(rx: bytes) -> bytes:
    """
    Estrae il frame canonico 64B da un trasferimento SPI.

    - 64B legacy: ritorna il buffer invariato
    - 128B canonical padded: cerca il primo frame J5 valido dentro la finestra
    """
    if len(rx) <= J5_SPI_FRAME_LEN_LEGACY:
        return rx

    candidates: list[tuple[int, bytes]] = []
    for off in range(0, len(rx) - J5_SPI_FRAME_LEN_LEGACY + 1):
        if _is_valid_canonical_j5_frame_at(rx, off):
            candidates.append((off, rx[off : off + J5_SPI_FRAME_LEN_LEGACY]))

    if not candidates:
        return rx[:J5_SPI_FRAME_LEN_LEGACY]

    candidates.sort(key=lambda item: (0 if item[0] in (0, 64) else 1, item[0]))
    return candidates[0][1]


def verify_spi_worker_frame_len(worker: object) -> None:
    want = spi_transport_frame_length()
    got = int(getattr(worker, "_frame_len", -1))
    if got != want:
        raise RuntimeError(
            "SPI incoerente: mode=%s richiede transfer %d B, ma SPIWorker ha frame_len=%d "
            "(riaprire SPI dopo export env)"
            % (spi_transport_mode_name(), want, got)
        )


def log_transport_banner(frame_len: int) -> None:
    exp = spi_transport_frame_length()
    logging.getLogger(__name__).info(
        "SPI PREFLIGHT: mode=%s -> transfer_len=%d B (atteso %d) %s",
        spi_transport_mode_name(),
        frame_len,
        exp,
        "OK" if frame_len == exp else "INCOERENTE",
    )
    if frame_len != exp:
        raise RuntimeError(
            "SPI PREFLIGHT fallito: frame_len=%d ma policy richiede %d B"
            % (frame_len, exp)
        )
