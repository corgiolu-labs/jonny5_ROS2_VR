"""
NOTE [RPI-SAFE-REFACTOR-PHASE1]
- Modulo analizzato, nessuna modifica funzionale.
- Marcatura delle funzioni CORE / UTILITY / DIAGNOSTIC / LEGACY.
- Obiettivo: documentazione interna per futura FASE 2.

j5vr_spi_bridge.py — Bridge logico tra shared_state e SPI DATA PLANE.

Riceve dati da shared_state (file IPC), costruisce frame J5VR, invia su SPI1.
Allineato al protocollo definito in src/spi/j5_protocol.h.

Architettura JONNY5 v1.1.1 — SPI DATA PLANE 1.0
"""

import math
import struct
import logging
import time
from types import ModuleType
from typing import Optional, Callable, Union, Any


from .j5vr_frame import build_setpoint_frame, J5_FRAME_TYPE_TELEMETRY
from .spi_worker import SPIWorker

logger = logging.getLogger(__name__)

# Throttle log SPI-RX TELEMETRY: loggare solo 1 ogni LOG_EVERY_N frame (evita spam a 100 Hz)
LOG_EVERY_N = 20

# Finestra di grazia per "heartbeat" telemetria: quando il firmware risponde con un
# frame J5 valido ma non-TELEMETRY (es. STATUS 0x03) entro TELEMETRY_GRACE_S secondi
# dall'ultimo 0x01, rigeneriamo il file telemetria con gli ultimi valori IMU noti in
# modo che shared_state.is_telemetry_fresh() non flippi a False per brevi buchi dello
# slave SPI. Previene IMU pill flicker osservato in campo senza mascherare guasti reali
# (oltre la finestra il file torna correttamente stale).
TELEMETRY_GRACE_S = 0.5


def _clamp_deg_int(x: Any) -> Optional[int]:
    try:
        v = float(x)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(v):
        return None
    out = int(round(v))
    if out < 0:
        out = 0
    elif out > 180:
        out = 180
    return out



class J5VRSPIBridge:
    """
    Bridge logico tra shared_state e SPI DATA PLANE.
    
    Responsabilità:
    - Leggere stato da shared_state provider
    - Costruire frame J5VR (64 byte)
    - Inviare frame su SPI1 verso STM32
    - Gestire sequenza e errori
    """
    
    def __init__(
        self,
        spi_worker: SPIWorker,
        state_provider: Union[Callable[[], Optional[dict]], ModuleType, Any],
    ):
        """
        Inizializza bridge SPI.
        
        Args:
            spi_worker: Worker SPI già configurato
            state_provider: Funzione che ritorna dict con stato (o None se non disponibile)
        """
        self.spi_worker = spi_worker
        self.state_provider = state_provider
        self.sequence_counter = 0
        self._frame_count = 0
        self._error_count = 0
        self._teleoppose_ack_prev = False
        self._teleoppose_ack_id = 0
        self._spi_send_log_count = 0
        self._rx_diag_prev = None
        self._imu_prev_sample_counter = None
        self._imu_prev_sample_t = None
        # Cache telemetria: usata per mtime-heartbeat in grace window e per preservare
        # imu_valid quando arriva lo stesso sample IMU (payload[28] transitorio).
        self._last_telemetry_out: Optional[dict] = None
        self._last_telemetry_wall_t: Optional[float] = None
        self._last_imu_valid: Optional[bool] = None

        # Allineamento: se esiste già un feedback file con id, continuiamo da lì.
        try:
            fb_reader = getattr(self.state_provider, "read_feedback_from_file", None)
            if callable(fb_reader):
                fb = fb_reader()
                if isinstance(fb, dict):
                    self._teleoppose_ack_id = int(fb.get("id", 0) or 0) & 0xFFFFFFFF
        except Exception:
            self._teleoppose_ack_id = 0


    def _pad_tx_frame(self, frame_64: bytes) -> bytes:
        fl = int(getattr(self.spi_worker, "_frame_len", 64))
        if fl == 64:
            return frame_64
        if len(frame_64) == fl:
            return frame_64
        if len(frame_64) == 64:
            return frame_64 + b"\x00" * (fl - 64)
        raise ValueError(f"TX {len(frame_64)} byte incompatibile con frame_len={fl}")

    # CORE PATH: lettura stato intent dal provider (callable/shared_state)
    def _read_state(self) -> Optional[dict]:
        """
        Legge lo stato corrente dal provider.

        Supporta:
        - callable che ritorna Optional[dict]
        - modulo/oggetto stile shared_state con metodo read_intent_from_file()
        """
        sp = self.state_provider
        if callable(sp):
            return sp()
        read_fn = getattr(sp, "read_intent_from_file", None)
        if callable(read_fn):
            return read_fn()
        raise TypeError(
            "state_provider deve essere una callable oppure un oggetto/modulo "
            "con metodo read_intent_from_file()"
        )


    # CORE PATH: costruzione frame J5VR + transfer SPI + update telemetria
    def send_setpoint_once(self) -> Optional[bytes]:
        """
        Invia un singolo frame setpoint su SPI.
        
        Processo:
        1. Legge stato dal provider
        2. Costruisce frame J5VR
        3. Invia su SPI
        4. Ritorna risposta RX (se disponibile)
        
        Returns:
            bytes: Risposta RX (64 byte) o None se errore
            
        Raises:
            RuntimeError: Se SPI non è aperto
        """
        from .spi_transport_mode import verify_spi_worker_frame_len

        verify_spi_worker_frame_len(self.spi_worker)

        if not self.spi_worker.is_open:
            raise RuntimeError("SPI non aperto: aprire spi_worker prima di inviare")
        
        # Leggi stato dal provider
        state = self._read_state()
        if state is None:
            # Nessun stato disponibile: crea frame vuoto (IDLE)
            state = {}
        else:
            # Log quando riceviamo dati VR (ogni 50 frame = ~1 secondo a 50 Hz)
            if self._frame_count % 50 == 0:
                logger.info(
                    "VR data: mode=%s joy_x=%.2f joy_y=%.2f pitch=%.2f yaw=%.2f intensity=%.2f quat=[%.3f,%.3f,%.3f,%.3f] buttons_L=0x%04x buttons_R=0x%04x",
                    state.get("mode", "N/A"),
                    state.get("joy_x", 0.0),
                    state.get("joy_y", 0.0),
                    state.get("pitch", 0.0),
                    state.get("yaw", 0.0),
                    state.get("intensity", 0.0),
                    state.get("quat_w", 1.0),
                    state.get("quat_x", 0.0),
                    state.get("quat_y", 0.0),
                    state.get("quat_z", 0.0),
                    state.get("buttons_left", 0) & 0xFFFF,
                    state.get("buttons_right", 0) & 0xFFFF,
                )
        
        try:
            # Mode 5 (HEAD ASSIST): frame J5VR — target B/S/G nell'estensione byte 36-45 (marker 'I').
            sc0 = self.sequence_counter
            frame = build_setpoint_frame(state, sequence_counter=sc0)
            frame_bytes = frame.to_bytes()
            try:
                p = frame_bytes[8:62]
                fp = 2166136261
                for b in p:
                    fp ^= int(b)
                    fp = (fp * 16777619) & 0xFFFFFFFF
                logger.info(
                    "[SPI_TX_TIMELINE] t=%.6f seq=%d jx=%d jy=%d bl=0x%04x br=0x%04x fp=%08x",
                    time.monotonic(),
                    sc0,
                    int(getattr(frame.payload, "joy_x", 0)),
                    int(getattr(frame.payload, "joy_y", 0)),
                    int(getattr(frame.payload, "buttons_left", 0)) & 0xFFFF,
                    int(getattr(frame.payload, "buttons_right", 0)) & 0xFFFF,
                    fp,
                )
            except Exception:
                pass

            # Invarianti: un frame J5VR valido deve essere sempre lungo 64 byte.
            # La logica sottostante gestisce comunque errori di dimensione
            # restituendo None; questa assert serve solo come guardrail in sviluppo
            # e non cambia il comportamento per i casi validi.
            assert len(frame_bytes) == 64, "[SPI][ASSERT] frame_bytes deve essere lungo 64 byte"

            # Log esplicito quando inviamo un comando di movimento (diagnostica "robot non si muove")
            mode_val = state.get("mode", 0)
            mode_code = mode_val if isinstance(mode_val, int) else (1 if str(mode_val) == "RELATIVE_MOVE" else 0)
            jx, jy = state.get("joy_x", 0), state.get("joy_y", 0)
            pt, yw = state.get("pitch", 0), state.get("yaw", 0)
            if mode_code == 1 and (jx != 0 or jy != 0 or pt != 0 or yw != 0):
                # Payload inizia dopo header 8 byte; primi 11 byte payload = mode, joy_x, joy_y, pitch, yaw, intensity, grip
                pl = frame_bytes[8:8+54] if len(frame_bytes) >= 62 else b""
                logger.info(
                    ">>> INVIO MOVIMENTO mode=1 joy_x=%s joy_y=%s pitch=%s yaw=%s grip=%s | payload[0:12]=%s",
                    jx, jy, pt, yw, state.get("grip"), pl[:12].hex() if pl else "n/a",
                )

            # Log dei pulsanti realmente inviati su SPI (post-normalizzazione) e deadman atteso lato STM32.
            # Nota: il firmware usa bit1 come GRIP per ciascuna mano.
            if self._frame_count % 50 == 0:
                bl = int(getattr(frame.payload, "buttons_left", 0)) & 0xFFFF
                br = int(getattr(frame.payload, "buttons_right", 0)) & 0xFFFF
                grip_l = (bl & (1 << 1)) != 0
                grip_r = (br & (1 << 1)) != 0
                logger.info(
                    "[SPI-TX] buttons(sent) L=0x%04x R=0x%04x gripL(bit1)=%d gripR(bit1)=%d deadman=%d",
                    bl, br, int(grip_l), int(grip_r), int(grip_l and grip_r),
                )
            # Log diagnostico 4 livelli: quaternione inviato su SPI verso STM32 (solo mode=2, ogni 5° frame)
            if state.get("mode") == 2:
                self._spi_send_log_count += 1
                if self._spi_send_log_count % 5 == 0:
                    logger.info(
                        "[SPI-SEND] qvis=(%.3f %.3f %.3f %.3f)",
                        frame.payload.quat_w, frame.payload.quat_x,
                        frame.payload.quat_y, frame.payload.quat_z,
                    )

            # Diagnostica comandi sporadici (no-spam): logga TELEOPPOSE se presente.
            try:
                if state.get("cmd") == "TELEOPPOSE" and (self._frame_count % 10 == 0):
                    p40 = frame_bytes[8 + 40]
                    p41 = frame_bytes[8 + 41]
                    logger.info("[SPI] TELEOPPOSE encoded (payload[40]=0x%02x payload[41]=0x%02x)", p40, p41)
            except Exception:
                pass
            
            # Verifica dimensione
            if len(frame_bytes) != 64:
                logger.error("Frame size errato: %d byte (attesi 64)", len(frame_bytes))
                self._error_count += 1
                return None
            
            # Invia su SPI: il frame semantico resta canonico 64B, l'eventuale 128B
            # e solo padding di trasporto.
            txb = self._pad_tx_frame(frame_bytes)
            rx_bytes = self.spi_worker.transfer(txb)
            from .spi_transport_mode import extract_canonical_frame64_from_transport_rx

            rx_legacy = (
                extract_canonical_frame64_from_transport_rx(rx_bytes)
                if rx_bytes and len(rx_bytes) >= 64
                else rx_bytes
            )

            # RX parsing: TELEOPPOSE ACK (payload 52-53) e telemetria IMU (payload 28-44)
            try:
                ack = False
                if rx_legacy and len(rx_legacy) >= 64 and rx_legacy[0:2] == b"J5":
                    frame_type_rx = rx_legacy[3]
                    payload = rx_legacy[8:62]
                    # Invariante: il payload deve essere sempre 54 byte (8..61).
                    assert len(payload) == 54, "[SPI][ASSERT] payload J5VR/TELEMETRY deve essere lungo 54 byte"
                    # Diagnostica STM32 -> RPI (ultimi 8 byte payload): hb, mode, grip, diag_mask, ack
                    try:
                        hb = ((payload[46] << 8) | payload[47]) & 0xFFFF
                        m = int(payload[48]) & 0xFF
                        g = int(payload[49]) & 0xFF
                        diag = ((payload[50] << 8) | payload[51]) & 0xFFFF
                        d_deadman = 1 if (diag & (1 << 0)) else 0
                        d_input = 1 if (diag & (1 << 1)) else 0
                        d_armed = 1 if (diag & (1 << 2)) else 0
                        d_freeze = 1 if (diag & (1 << 3)) else 0
                        d_guard = 1 if (diag & (1 << 4)) else 0
                        prev = self._rx_diag_prev
                        cur = (hb, m, g, diag, frame_type_rx)
                        if prev != cur and (self._frame_count % 5 == 0):
                            logger.info(
                                "[SPI-RX][DIAG] hb=%d mode=%d grip=%d diag=0x%04x deadman=%d input=%d armed=%d freeze=%d guard=%d rx_type=0x%02x",
                                hb, m, g, diag, d_deadman, d_input, d_armed, d_freeze, d_guard, frame_type_rx,
                            )
                        self._rx_diag_prev = cur
                    except Exception:
                        pass
                    # ACK TELEOPPOSE valido solo su frame STATUS:
                    # nei frame TELEMETRY i byte 52..53 sono usati per imu_sample_counter
                    # e non devono mai generare teleop_pose_ack.
                    if frame_type_rx == 0x03 and len(payload) > 53:
                        p52 = payload[52]
                        p53 = payload[53]
                        if p52 == ord("T") and p53 == 1:
                            ack = True
                    # TELEMETRY (0x01): estrai imu_valid, quaternione e angoli servo (gradi) per debug UI
                    if frame_type_rx == J5_FRAME_TYPE_TELEMETRY and len(payload) >= 51:
                        imu_valid_raw = bool(payload[28] != 0)
                        imu_valid = imu_valid_raw
                        try:
                            imu_accel_x = struct.unpack_from(">f", payload, 0)[0]
                            imu_accel_y = struct.unpack_from(">f", payload, 4)[0]
                            imu_accel_z = struct.unpack_from(">f", payload, 8)[0]
                            imu_gyro_x = struct.unpack_from(">f", payload, 12)[0]
                            imu_gyro_y = struct.unpack_from(">f", payload, 16)[0]
                            imu_gyro_z = struct.unpack_from(">f", payload, 20)[0]
                        except Exception:
                            imu_accel_x, imu_accel_y, imu_accel_z = 0.0, 0.0, 0.0
                            imu_gyro_x, imu_gyro_y, imu_gyro_z = 0.0, 0.0, 0.0
                        try:
                            imu_q_w = struct.unpack_from(">f", payload, 29)[0]
                            imu_q_x = struct.unpack_from(">f", payload, 33)[0]
                            imu_q_y = struct.unpack_from(">f", payload, 37)[0]
                            imu_q_z = struct.unpack_from(">f", payload, 41)[0]
                        except Exception:
                            imu_q_w, imu_q_x, imu_q_y, imu_q_z = 1.0, 0.0, 0.0, 0.0
                        try:
                            imu_temp = struct.unpack_from(">f", payload, 24)[0]
                        except Exception:
                            imu_temp = None
                        # Offset 45-50: B, S, G, Y, P, R (gradi 0-180)
                        servo_B = int(payload[45]) if len(payload) > 45 else 0
                        servo_S = int(payload[46]) if len(payload) > 46 else 0
                        servo_G = int(payload[47]) if len(payload) > 47 else 0
                        servo_Y = int(payload[48]) if len(payload) > 48 else 0
                        servo_P = int(payload[49]) if len(payload) > 49 else 0
                        servo_R = int(payload[50]) if len(payload) > 50 else 0
                        imu_sample_counter = (
                            ((int(payload[51]) & 0xFF) << 16)
                            | ((int(payload[52]) & 0xFF) << 8)
                            | (int(payload[53]) & 0xFF)
                        ) if len(payload) > 53 else 0
                        # Periodo runtime del RT loop dello STM32, dai 2 byte
                        # reserved del frame canonical (offset assoluto 62-63),
                        # uint16 BE in microsecondi (EWMA filtrato lato firmware).
                        # 0 = warm-up o firmware senza supporto. Backward compat
                        # garantita dai parser legacy che ignoravano questi byte.
                        rt_loop_period_us = None
                        rt_loop_hz_est = None
                        if len(rx_legacy) >= 64:
                            raw_period = (int(rx_legacy[62]) << 8) | int(rx_legacy[63])
                            if 100 <= raw_period <= 60000:
                                rt_loop_period_us = raw_period
                                rt_loop_hz_est = 1_000_000.0 / raw_period
                        now_mono = time.monotonic()
                        imu_sample_delta = None
                        imu_rate_hz_est = None
                        imu_repeated = False
                        imu_jump = 0
                        if self._imu_prev_sample_counter is not None and self._imu_prev_sample_t is not None:
                            imu_sample_delta = (imu_sample_counter - self._imu_prev_sample_counter) & 0xFFFFFF
                            dt_s = now_mono - self._imu_prev_sample_t
                            if dt_s > 1e-6:
                                imu_rate_hz_est = imu_sample_delta / dt_s
                            imu_repeated = (imu_sample_delta == 0)
                            if imu_sample_delta > 1:
                                imu_jump = int(imu_sample_delta - 1)
                        # Se e' lo STESSO sample IMU (counter invariato) e in precedenza
                        # lo avevamo gia' validato, conserva imu_valid=True. Evita flicker
                        # dovuto a race sul DMA slave che azzera transitoriamente payload[28]
                        # mentre il sample sottostante non e' cambiato.
                        if imu_repeated and self._last_imu_valid is True and not imu_valid_raw:
                            imu_valid = True
                        self._imu_prev_sample_counter = imu_sample_counter
                        self._imu_prev_sample_t = now_mono
                        self._last_imu_valid = imu_valid
                        if self._frame_count % LOG_EVERY_N == 0:
                            logger.info(
                                "[SPI-RX TELEMETRY] imu_valid=%s raw_byte=%d sample=%d d_sample=%s rate=%.1fHz rep=%s jump=%d | "
                                "quat=(w=%.3f, x=%.3f, y=%.3f, z=%.3f) | "
                                "servo B/S/G/Y/P/R = %d/%d/%d/%d/%d/%d",
                                imu_valid,
                                payload[28],
                                imu_sample_counter,
                                str(imu_sample_delta),
                                float(imu_rate_hz_est or 0.0),
                                imu_repeated,
                                imu_jump,
                                imu_q_w, imu_q_x, imu_q_y, imu_q_z,
                                servo_B, servo_S, servo_G, servo_Y, servo_P, servo_R,
                            )
                        telemetry_writer = getattr(self.state_provider, "write_telemetry_to_file", None)
                        if callable(telemetry_writer):
                            out = {
                                "imu_valid": imu_valid,
                                "imu_accel_x": imu_accel_x,
                                "imu_accel_y": imu_accel_y,
                                "imu_accel_z": imu_accel_z,
                                "imu_gyro_x": imu_gyro_x,
                                "imu_gyro_y": imu_gyro_y,
                                "imu_gyro_z": imu_gyro_z,
                                "imu_q_w": imu_q_w,
                                "imu_q_x": imu_q_x,
                                "imu_q_y": imu_q_y,
                                "imu_q_z": imu_q_z,
                                "servo_deg_B": servo_B,
                                "servo_deg_S": servo_S,
                                "servo_deg_G": servo_G,
                                "servo_deg_Y": servo_Y,
                                "servo_deg_P": servo_P,
                                "servo_deg_R": servo_R,
                                "assist_state": None,
                                "deadman_active": None,
                                "movement_allowed": None,
                                "fault_code": None,
                                "flags_echo": None,
                                "rx_classification": None,
                                "assist_proto_version": None,
                                "frame_type": int(J5_FRAME_TYPE_TELEMETRY),
                                "packet_index": self._frame_count,
                                "wire_source": "legacy_0x01",
                                "imu_stale": (not bool(imu_valid)),
                                "imu_sample_counter": imu_sample_counter,
                            }
                            if imu_sample_delta is not None:
                                out["imu_sample_delta"] = int(imu_sample_delta)
                                out["imu_sample_repeated"] = bool(imu_repeated)
                                out["imu_sample_jump"] = int(imu_jump)
                            if imu_rate_hz_est is not None:
                                out["imu_rate_hz_est"] = float(imu_rate_hz_est)
                            if imu_temp is not None:
                                out["imu_temp"] = imu_temp
                            if rt_loop_period_us is not None:
                                out["rt_loop_period_us"] = int(rt_loop_period_us)
                                out["rt_loop_hz_est"] = float(rt_loop_hz_est)
                            telemetry_writer(out)
                            # Aggiorna cache per mtime-heartbeat in grace window.
                            self._last_telemetry_out = out
                            self._last_telemetry_wall_t = time.monotonic()
                    elif (
                        frame_type_rx != J5_FRAME_TYPE_TELEMETRY
                        and self._last_telemetry_out is not None
                        and self._last_telemetry_wall_t is not None
                        and (time.monotonic() - self._last_telemetry_wall_t) <= TELEMETRY_GRACE_S
                    ):
                        # Frame J5 valido ma non-TELEMETRY (es. STATUS 0x03): rigenera il
                        # file telemetria con gli ultimi valori per mantenere fresco il
                        # mtime (is_telemetry_fresh) ed evitare IMU pill flicker su brevi
                        # buchi dello slave. Non flippa imu_valid e marca l'evento come
                        # heartbeat per eventuale diagnostica downstream.
                        telemetry_writer_hb = getattr(self.state_provider, "write_telemetry_to_file", None)
                        if callable(telemetry_writer_hb):
                            hb = dict(self._last_telemetry_out)
                            hb["packet_index"] = self._frame_count
                            hb["telemetry_heartbeat"] = True
                            hb["heartbeat_rx_frame_type"] = int(frame_type_rx)
                            telemetry_writer_hb(hb)
                if ack and not self._teleoppose_ack_prev:
                    self._teleoppose_ack_id = (self._teleoppose_ack_id + 1) & 0xFFFFFFFF
                    logger.info("[SPI] TELEOPPOSE ACK from STM32 (id=%d)", self._teleoppose_ack_id)
                    fb_writer = getattr(self.state_provider, "write_feedback_to_file", None)
                    if callable(fb_writer):
                        fb_writer({"teleop_pose_ack": True, "id": int(self._teleoppose_ack_id)})
                self._teleoppose_ack_prev = ack
            except Exception:
                self._teleoppose_ack_prev = False
            
            # Incrementa contatore sequenza
            self.sequence_counter = (sc0 + 1) & 0xFFFF
            self._frame_count += 1
            
            # Log periodico (ogni 1000 frame ≈ 10 s a ~100 Hz)
            if self._frame_count % 1000 == 0:
                logger.debug(
                    "Frame inviati: %d, sequenza: %d, errori: %d",
                    self._frame_count, self.sequence_counter, self._error_count
                )
            
            return rx_legacy
            
        except Exception as e:
            logger.error("Errore durante invio frame SPI: %s", e, exc_info=True)
            self._error_count += 1
            return None
