"""
NOTE [RPI-SAFE-REFACTOR-PHASE1]
- Modulo analizzato, nessuna modifica funzionale.
- Marcatura delle funzioni CORE / UTILITY / DIAGNOSTIC / LEGACY.
- Obiettivo: documentazione interna per futura FASE 2.

UART Manager — industrial-grade control plane.
Un solo owner della seriale, coda FIFO, pattern request/response deterministico.
API asincrona: await uart_manager.send_uart_command(cmd, timeout_s=0.5) -> (ok, response).

Architettura a due thread:
  - _writer_thread : invia comandi, attende risposta #seq dal reader via coda interna.
  - _reader_thread : legge la porta in modo continuo, smista righe:
        * iniziano con "#seq " → coda _rx_lines per il writer
        * non iniziano con "#"  → callback unsolicited (SETPOSE_DONE ecc.)
        * iniziano con "#" ma seq diverso → log DISCARD
"""
import asyncio
import logging
import os
import queue
import threading
import time

try:
    import serial
except ImportError:
    serial = None

logger = logging.getLogger("uart_manager")

BAUD = 115200

# Margine aggiunto al timeout asyncio (wait_for) oltre al timeout_s del comando.
# Lascia al writer thread il tempo di rilevare il timeout internamente
# prima che asyncio lo annulli dall'esterno.
_UART_TIMEOUT_MARGIN_S = 1.0


def _get_port():
    return os.environ.get("SERIAL_DEV", "/dev/serial0")


class UARTManager:
    """
    Writer thread: invia comandi dalla coda FIFO, attende risposta via _rx_lines.
    Reader thread: legge continuamente dalla porta, smista righe con o senza #seq.
    La porta seriale è condivisa ma serializzata: il writer tiene il lock durante
    TX; il reader legge senza lock (pyserial è thread-safe per RX/TX separati).
    """

    def __init__(self):
        self._request_queue = queue.Queue()
        self._rx_lines      = queue.Queue()   # righe "#seq payload" per il writer
        self._ser           = None
        self._lock          = threading.Lock()
        self._writer_thread = None
        self._reader_thread = None
        self._started       = False
        self._seq_counter   = 0  # usato solo dal writer thread; wrap 1..65535
        # Callback per righe non-solicitate (es. SETPOSE_DONE): callable(line: str)
        self._unsolicited_callback = None
        self._comm_fail_streak = 0
        self._recover_every_failures = 3

    def set_unsolicited_callback(self, cb):
        """Registra un callable(line: str) per le righe UART non-solicitate."""
        self._unsolicited_callback = cb

    def is_available(self) -> bool:
        """True se pyserial è installato e la porta esiste."""
        if serial is None:
            return False
        return os.path.exists(_get_port())

    def _ensure_worker_started(self):
        with self._lock:
            if self._started:
                return
            self._started = True
            self._writer_thread = threading.Thread(target=self._writer, daemon=True, name="uart-writer")
            self._reader_thread = threading.Thread(target=self._reader, daemon=True, name="uart-reader")
            self._writer_thread.start()
            self._reader_thread.start()
            logger.info("[UART] writer + reader thread avviati")

    # -----------------------------------------------------------------------
    # Reader thread — legge continuamente, smista righe
    # -----------------------------------------------------------------------
    def _reader(self):
        """Thread dedicato alla lettura continua della porta seriale."""
        while True:
            try:
                if self._ser is None or not self._ser.is_open:
                    time.sleep(0.1)
                    continue
                raw = self._ser.readline()
                if not raw:
                    continue
                try:
                    text = raw.decode("ascii").strip()
                    logger.debug("[UART RX] %r", text)
                except UnicodeDecodeError as de:
                    text = raw.decode("ascii", errors="ignore").strip()
                    logger.warning("[UART RX DECODE] fallback %r err=%s", text, de)

                if not text:
                    continue

                if text.startswith("#"):
                    # Risposta a un comando sequenziato — la mette nella coda del writer
                    logger.debug("[UART RX SEQ] %r", text[:100])
                    self._rx_lines.put(text)
                else:
                    # Riga non-solicitata (SETPOSE_DONE, BOOT_READY, log firmware, ecc.)
                    logger.debug("[UART UNSOLICITED] %r", text[:100])
                    cb = self._unsolicited_callback
                    if cb is not None:
                        try:
                            cb(text)
                        except Exception as cb_err:
                            logger.warning("[UART UNSOLICITED CB] errore: %s", cb_err)
            except Exception as e:
                logger.warning("[UART reader] errore: %s", e)
                time.sleep(0.2)

    def _clear_rx_queue(self):
        """Svuota la coda RX interna (righe #seq) in modo non bloccante."""
        while not self._rx_lines.empty():
            try:
                self._rx_lines.get_nowait()
            except queue.Empty:
                break

    def _recover_serial_link(self, reason: str):
        """
        Recovery locale del link UART senza restart servizio:
        - chiude la seriale corrente (se aperta)
        - svuota coda RX interna da eventuali frame stale
        """
        logger.warning("[UART RECOVERY] trigger reason=%s", reason)
        try:
            if self._ser is not None and self._ser.is_open:
                self._ser.close()
        except Exception as e:
            logger.warning("[UART RECOVERY] close error: %s", e)
        self._ser = None
        self._clear_rx_queue()

    # -----------------------------------------------------------------------
    # Writer thread — invia comandi dalla coda FIFO
    # -----------------------------------------------------------------------
    def _writer(self):
        while True:
            try:
                item = self._request_queue.get()
                if item is None:
                    break
                cmd, timeout_s, loop, fut = item
                ok, response = self._do_one(cmd, timeout_s)
                try:
                    loop.call_soon_threadsafe(fut.set_result, (ok, response))
                except Exception as e:
                    logger.warning("[UART] call_soon_threadsafe: %s", e)
            except Exception as e:
                logger.exception("[UART] writer: %s", e)
                try:
                    loop.call_soon_threadsafe(fut.set_result, (False, str(e)))
                except Exception:
                    pass

    def _next_seq(self) -> int:
        """Prossimo sequence number (solo writer thread)."""
        self._seq_counter = (self._seq_counter % 65535) + 1
        return self._seq_counter

    def _open_serial(self):
        if self._ser is not None and self._ser.is_open:
            return True
        if serial is None:
            return False
        try:
            port = _get_port()
            # timeout breve: il reader fa readline() — non deve bloccare per sempre
            # exclusive=True evita aperture concorrenti della UART (es. pytest + ws_server).
            # Se non supportato dalla piattaforma/driver, fallback compatibile senza exclusive.
            try:
                self._ser = serial.Serial(port, BAUD, timeout=0.1, exclusive=True)
            except TypeError:
                self._ser = serial.Serial(port, BAUD, timeout=0.1)
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()
            logger.info("[UART] seriale aperta %s %d baud", port, BAUD)
            return True
        except Exception as e:
            logger.warning("[UART] apertura %s fallita: %s", _get_port(), e)
            self._ser = None
            return False

    def _do_one(self, cmd: str, timeout_s: float):
        """
        Invia un comando e attende la risposta #seq dalla coda _rx_lines
        (popolata dal reader thread). Non chiama più readline() direttamente.
        """
        cmd_clean = (cmd if cmd.endswith("\n") else cmd + "\n").strip()
        seq = self._next_seq()
        to_send = f"#{seq} {cmd_clean}\n"
        t0 = time.time()
        deadline = t0 + timeout_s

        if not self._open_serial():
            logger.warning("[UART DIAG] OPEN_ERROR per cmd=%s", cmd_clean)
            self._comm_fail_streak += 1
            if self._comm_fail_streak >= self._recover_every_failures:
                self._recover_serial_link("open_error_streak")
                self._comm_fail_streak = 0
            return False, "OPEN_ERROR"

        try:
            # Svuota eventuali righe rimaste nella coda rx (stale da comandi precedenti)
            while not self._rx_lines.empty():
                stale = self._rx_lines.get_nowait()
                logger.info("[UART RX STALE] scartato: %r", stale[:80])

            logger.debug("[UART TX] cmd=%s seq=%d frame=%r", cmd_clean, seq, to_send)
            self._ser.write(to_send.encode("ascii"))
            self._ser.flush()

            wanted_prefix = f"#{seq} "
            response_payload = None

            while time.monotonic() < deadline:
                remaining = deadline - time.monotonic()
                try:
                    text = self._rx_lines.get(timeout=min(0.1, remaining))
                except queue.Empty:
                    continue

                if text.startswith(wanted_prefix):
                    response_payload = text[len(wanted_prefix):].strip()
                    logger.info("[UART RX MATCH] seq=%d payload=%r", seq, response_payload)
                    break
                else:
                    # Seq diverso: scarta (uart_manager è serializzato, non c'è pipeline)
                    logger.info("[UART RX DISCARD] wanted_seq=%d got=%r", seq, text[:80])

            elapsed_ms = (time.time() - t0) * 1000
            if response_payload is None:
                logger.warning("[UART TIMEOUT] cmd=%s seq=%d after=%.0f ms timeout_s=%.2f",
                               cmd_clean, seq, elapsed_ms, timeout_s)
                self._comm_fail_streak += 1
                if self._comm_fail_streak >= self._recover_every_failures:
                    self._recover_serial_link("timeout_streak")
                    self._comm_fail_streak = 0
                return False, "TIMEOUT"

            ok = response_payload.startswith("OK ") or response_payload.startswith("STATUS:")
            if not ok and not response_payload.startswith("ERR "):
                ok = False
            logger.info("[UART] cmd=%s seq=%d response=%s %.0f ms",
                        cmd_clean, seq, response_payload[:60], elapsed_ms)
            self._comm_fail_streak = 0
            return ok, response_payload

        except Exception as e:
            logger.warning("[UART] cmd=%s errore: %s", cmd_clean, e)
            self._comm_fail_streak += 1
            self._recover_serial_link("exception")
            return False, str(e)

    async def send_uart_command(self, cmd: str, timeout_s: float = 0.5) -> tuple[bool, str]:
        """
        Invia un comando UART (ENABLE, STOP, STATUS?, IMUON, IMUOFF, ecc.).
        Ritorna (ok, response). ok=False per timeout, errore apertura, o eccezione.
        """
        self._ensure_worker_started()
        loop = asyncio.get_running_loop()
        fut = loop.create_future()
        self._request_queue.put((cmd.strip(), timeout_s, loop, fut))
        try:
            return await asyncio.wait_for(fut, timeout=timeout_s + _UART_TIMEOUT_MARGIN_S)
        except asyncio.TimeoutError:
            logger.warning("[UART] send_uart_command timeout (await) cmd=%s", cmd)
            return False, "TIMEOUT"


# Singleton usato da ws_server
uart_manager = UARTManager()
