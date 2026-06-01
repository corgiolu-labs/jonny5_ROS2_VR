"""
NOTE [RPI-SAFE-REFACTOR-PHASE2]
- Modulo analizzato, nessuna modifica funzionale.
- Responsabilità documentate come CORE / UTILITY / DIAGNOSTIC / LEGACY.

ws_core.py — Utility comuni e costanti globali del WebSocket server.

Contiene:
  - costanti temporali condivise da tutti i moduli ws_handlers_*  (CORE/UTILITY)
  - set ``clients`` (websocket connessi, acceduto da tutti i loop) (CORE PATH)
  - helper numerici: _is_finite, _is_finite_tuple, _clamp_float           (UTILITY)
  - helper WebSocket: _ws_safe_send                                      (UTILITY)

[RPi-0.5] Creato da ws_server.py. Nessuna modifica comportamentale.
[RPi-0.6] Import ordinati PEP8; docstring modulo aggiornata.
[RPi-0.7] Aggiunti commenti avviso su clients (no lock, safe asyncio).
"""

# stdlib
import asyncio
import math

# ---------------------------------------------------------------------------
# Costanti temporali (secondi)
# ---------------------------------------------------------------------------
_STATUS_BOOT_DELAY_S   = 1.0     # attesa boot STM32 prima del primo poll STATUS?
_STATUS_POLL_PERIOD_S  = 2.0     # periodo polling STATUS?
_IMU_DEBUG_SLEEP_S     = 0.010   # sleep nel loop imu_debug_loop
_FEEDBACK_LOOP_SLEEP_S = 0.050   # sleep nel loop feedback e head-heartbeat

# ---------------------------------------------------------------------------
# Stato condiviso a livello di processo
#
# NOTA [RPi-0.7]: `clients` è un set Python modificato da handle_client
# (add/discard) e letto da imu_debug_loop, feedback_loop e ws_handlers_uart.
# Tutti operano nello stesso event loop asyncio (single-thread cooperativo),
# quindi l'accesso non richiede lock.
# I loop che iterano su clients usano list(clients) come snapshot per evitare
# "Set changed size during iteration" in caso di disconnessione concorrente.
# Se in futuro si aggiungono thread separati che accedono a clients,
# sarà necessario sostituire con un asyncio.Lock o threading.Lock.
# ---------------------------------------------------------------------------

# Set dei websocket connessi (acceduto da feedback_loop, imu_debug_loop, ecc.)
clients: set = set()

# ---------------------------------------------------------------------------
# Helper numerici
# ---------------------------------------------------------------------------

def _is_finite(x) -> bool:
    return isinstance(x, (int, float)) and math.isfinite(x)


def _is_finite_tuple(*vals) -> bool:
    """
    [RPi-0.4] Ritorna True solo se tutti i valori passati sono finiti.
    Consolidamento del pattern: if not math.isfinite(x) or not math.isfinite(y) ...
    """
    return all(_is_finite(v) for v in vals)


def _clamp_float(x, lo, hi):
    if not _is_finite(x):
        return None
    return max(lo, min(hi, float(x)))


# ---------------------------------------------------------------------------
# Helper WebSocket
# ---------------------------------------------------------------------------

async def _ws_safe_send(ws, data: str) -> None:
    """
    [RPi-0.4] Invia un messaggio WS ignorando le eccezioni di connessione.
    Identico al pattern try/except pass usato in tutto handle_client.
    """
    try:
        await ws.send(data)
    except Exception:
        pass
