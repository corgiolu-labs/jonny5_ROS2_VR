"""
j5vr_frame.py — Frame J5VR (64 byte) per SPI DATA PLANE.

Implementazione Python allineata esattamente a src/spi/j5_protocol.h (lato STM32).
Frame fisso 64 byte con layout identico alla struct C j5_frame_t.

Architettura JONNY5 v1.1.1 — SPI DATA PLANE 1.0
"""

import struct
from dataclasses import dataclass
from typing import Optional

# Costanti allineate a j5_protocol.h
J5_PROTOCOL_FRAME_SIZE = 64
J5_FRAME_TYPE_TELEMETRY = 0x01
J5_FRAME_TYPE_J5VR = 0x04

# Header riconoscibile
J5_HEADER_BYTE0 = ord('J')  # 0x4A
J5_HEADER_BYTE1 = ord('5')  # 0x35
J5_PROTOCOL_VERSION = 1


def _norm_float_to_i16(x) -> int:
    """
    Accetta int16 già scalato oppure float normalizzato [-1..1].
    Identico alla vecchia nested to_i16() in build_setpoint_frame().
    Se arriva un int piccolo (-1..1), lo considera normalizzato per compatibilità.
    """
    if isinstance(x, int):
        if -1 <= x <= 1:
            v = float(x)
            return max(-32768, min(32767, int(round(v * 32767))))
        return max(-32768, min(32767, int(x)))
    try:
        v = float(x)
    except Exception:
        v = 0.0
    v = max(-1.0, min(1.0, v))
    return max(-32768, min(32767, int(round(v * 32767))))


def _norm_float_to_u8_intensity(x) -> int:
    """
    Accetta uint8 [0..255] oppure float [0..1].
    Identico alla vecchia nested to_u8_intensity() in build_setpoint_frame().
    """
    if isinstance(x, int):
        return max(0, min(255, int(x)))
    try:
        v = float(x)
    except Exception:
        v = 0.0
    v = max(0.0, min(1.0, v))
    return max(0, min(255, int(round(v * 255))))


def _angle_deg_to_cdeg_i16(x) -> int:
    """
    Accetta angolo in gradi fisici [0..180] e lo converte in centi-gradi int16.
    Usato per l'estensione mode=5 (target B/S/G) nel frame J5VR.
    """
    try:
        v = float(x)
    except Exception:
        v = 90.0
    v = max(0.0, min(180.0, v))
    return max(-32768, min(32767, int(round(v * 100.0))))


@dataclass
class J5VRPayload:
    """
    Payload J5VR (54 byte) — layout identico a struct j5vr_state in j5_protocol.h.
    
    Layout byte-per-byte (big-endian per valori multi-byte):
    - offset 0: mode (uint8)
    - offset 1-2: joy_x (int16 big-endian)
    - offset 3-4: joy_y (int16 big-endian)
    - offset 5-6: pitch (int16 big-endian)
    - offset 7-8: yaw (int16 big-endian)
    - offset 9: intensity (uint8)
    - offset 10: grip (uint8)
    - offset 11-12: vr_heartbeat (uint16 big-endian)
    - offset 13: priority (uint8)
    - offset 14-15: safe_mask (uint16 big-endian)
    - offset 16-31: quaternioni orientamento visore (4 float32 big-endian, 16 byte)
    - offset 32-33: buttons_left (uint16 big-endian)
    - offset 34-35: buttons_right (uint16 big-endian)
    - offset 36-45: estensioni mode-specific
      - mode=5 + marker 'I': target braccio B/S/G (HEAD ASSIST)
      - mode!=5: CAMCTRL / TELEOPPOSE legacy
    - offset 46-53: diagnostica TX (8 byte)
    """
    mode: int = 0                    # uint8
    joy_x: int = 0                   # int16 (-32768..32767)
    joy_y: int = 0                   # int16 (-32768..32767)
    pitch: int = 0                   # int16 (-32768..32767)
    yaw: int = 0                     # int16 (-32768..32767)
    intensity: int = 0               # uint8 (0..255)
    grip: int = 0                    # uint8 (0 o 1)
    vr_heartbeat: int = 0            # uint16 (0..65535)
    priority: int = 0                # uint8
    safe_mask: int = 0x0000          # uint16
    quat_w: float = 1.0              # float32 (quaternione W)
    quat_x: float = 0.0              # float32 (quaternione X)
    quat_y: float = 0.0              # float32 (quaternione Y)
    quat_z: float = 0.0              # float32 (quaternione Z)
    buttons_left: int = 0            # uint16 (bitmask pulsanti controller sinistro)
    buttons_right: int = 0           # uint16 (bitmask pulsanti controller destro)
    # Estensione v1.1.5 (senza cambiare dimensione frame): CAMCTRL nei byte riservati 36-45
    # - payload[36] = 'C' (marker)
    # - payload[37] = cmd_id (1=focus, 2=zoom, 3=conv)
    # - payload[38..39] = delta int16 BE
    camctrl_cmd: int = 0             # uint8 (0=none, 1..3 come sopra)
    camctrl_delta: int = 0           # int16 (delta)
    # Estensione mode=5 (B/S/G assist) nei byte 36-45:
    # - payload[36] = 'I'
    # - payload[37] = flags (bit0=valid, bit1=grip_active, bit2=hold_active)
    # - payload[38..39] = target_id uint16 BE
    # - payload[40..41] = base  int16 BE centi-gradi fisici
    # - payload[42..43] = spalla int16 BE centi-gradi fisici
    # - payload[44..45] = gomito int16 BE centi-gradi fisici
    mode5_arm_valid: int = 0
    mode5_control_flags: int = 0
    mode5_target_id: int = 0
    mode5_base_deg: float = 90.0
    mode5_spalla_deg: float = 90.0
    mode5_gomito_deg: float = 90.0

    def to_bytes(self) -> bytes:
        """Serializza payload in 54 byte (big-endian)."""
        payload = bytearray(54)
        
        # Clamp valori per sicurezza
        mode = max(0, min(255, self.mode))
        joy_x = max(-32768, min(32767, self.joy_x))
        joy_y = max(-32768, min(32767, self.joy_y))
        pitch = max(-32768, min(32767, self.pitch))
        yaw = max(-32768, min(32767, self.yaw))
        intensity = max(0, min(255, self.intensity))
        grip = 1 if self.grip else 0
        vr_heartbeat = max(0, min(65535, self.vr_heartbeat)) & 0xFFFF
        priority = max(0, min(255, self.priority))
        safe_mask = max(0, min(65535, self.safe_mask)) & 0xFFFF
        
        # Serializzazione (big-endian come in C)
        payload[0] = mode
        struct.pack_into(">h", payload, 1, joy_x)   # int16 big-endian
        struct.pack_into(">h", payload, 3, joy_y)
        struct.pack_into(">h", payload, 5, pitch)
        struct.pack_into(">h", payload, 7, yaw)
        payload[9] = intensity
        payload[10] = grip
        struct.pack_into(">H", payload, 11, vr_heartbeat)  # uint16 big-endian
        payload[13] = priority
        struct.pack_into(">H", payload, 14, safe_mask)
        
        # Offset 16-31: quaternioni orientamento visore (4 float32 big-endian)
        struct.pack_into(">f", payload, 16, float(self.quat_w))
        struct.pack_into(">f", payload, 20, float(self.quat_x))
        struct.pack_into(">f", payload, 24, float(self.quat_y))
        struct.pack_into(">f", payload, 28, float(self.quat_z))
        
        # Offset 32-35: pulsanti joystick (2 uint16 big-endian)
        buttons_left = max(0, min(65535, self.buttons_left)) & 0xFFFF
        buttons_right = max(0, min(65535, self.buttons_right)) & 0xFFFF
        struct.pack_into(">H", payload, 32, buttons_left)
        struct.pack_into(">H", payload, 34, buttons_right)
        
        # Offset 36-45: estensioni mode-specific senza cambiare dimensione frame.
        if mode == 5 and int(self.mode5_arm_valid or 0) == 1:
            flags = int(self.mode5_control_flags or 0) & 0xFF
            flags |= 1 << 0
            payload[36] = ord("I")
            payload[37] = flags
            struct.pack_into(">H", payload, 38, int(self.mode5_target_id or 0) & 0xFFFF)
            struct.pack_into(">h", payload, 40, _angle_deg_to_cdeg_i16(self.mode5_base_deg))
            struct.pack_into(">h", payload, 42, _angle_deg_to_cdeg_i16(self.mode5_spalla_deg))
            struct.pack_into(">h", payload, 44, _angle_deg_to_cdeg_i16(self.mode5_gomito_deg))
        else:
            cmd = int(self.camctrl_cmd or 0)
            if cmd in (1, 2, 3):
                delta = max(-32768, min(32767, int(self.camctrl_delta or 0)))
                payload[36] = ord("C")
                payload[37] = cmd & 0xFF
                struct.pack_into(">h", payload, 38, delta)
                # payload[40..45] resta zero

        # Offset 46-53: diagnostica TX (gestita dal firmware)
        
        return bytes(payload)

class J5VRFrame:
    """
    Frame J5VR completo (64 byte) — layout identico a j5_frame_t in j5_protocol.h.
    
    Struttura (packed, 64 byte esatti):
    - header[2]: 'J' '5' (0x4A 0x35)
    - protocol_version: 1
    - frame_type: J5_FRAME_TYPE_J5VR (0x04)
    - sequence_counter: uint16 big-endian
    - payload_len: 64
    - flags: 0
    - payload[54]: dati J5VR
    - reserved[2]: 0
    """
    
    def __init__(
        self,
        payload: Optional[J5VRPayload] = None,
        sequence_counter: int = 0,
        frame_type: int = J5_FRAME_TYPE_J5VR,
    ):
        """
        Crea un frame J5VR.
        
        Args:
            payload: Payload J5VR (se None, crea payload vuoto)
            sequence_counter: Contatore sequenza (0..65535)
            frame_type: Tipo frame (default J5_FRAME_TYPE_J5VR = 0x04)
        """
        self.header = (J5_HEADER_BYTE0, J5_HEADER_BYTE1)
        self.protocol_version = J5_PROTOCOL_VERSION
        self.frame_type = frame_type
        self.sequence_counter = sequence_counter & 0xFFFF
        self.payload_len = J5_PROTOCOL_FRAME_SIZE
        self.flags = 0
        self.payload = payload if payload is not None else J5VRPayload()
        self.reserved = (0, 0)

    def to_bytes(self) -> bytes:
        """
        Serializza frame in 64 byte esatti (layout identico a j5_frame_t C).
        
        Returns:
            bytes: Frame completo di 64 byte
        """
        frame = bytearray(J5_PROTOCOL_FRAME_SIZE)
        
        # Header: 'J' '5'
        frame[0] = self.header[0]
        frame[1] = self.header[1]
        
        # protocol_version
        frame[2] = self.protocol_version
        
        # frame_type
        frame[3] = self.frame_type
        
        # sequence_counter (big-endian, come in C con __builtin_bswap16)
        frame[4] = (self.sequence_counter >> 8) & 0xFF
        frame[5] = self.sequence_counter & 0xFF
        
        # payload_len
        frame[6] = self.payload_len
        
        # flags
        frame[7] = self.flags
        
        # payload (54 byte)
        payload_bytes = self.payload.to_bytes()
        frame[8:62] = payload_bytes
        
        # reserved (2 byte)
        frame[62] = self.reserved[0]
        frame[63] = self.reserved[1]
        
        return bytes(frame)

def build_setpoint_frame(shared_state_data: dict, sequence_counter: int = 0) -> J5VRFrame:
    """
    Costruisce un frame J5VR a partire dai dati di shared_state.
    
    Args:
        shared_state_data: Dict con chiavi: mode, joy_x, joy_y, pitch, yaw, intensity, grip, heartbeat
        sequence_counter: Contatore sequenza
        
    Returns:
        J5VRFrame: Frame pronto per invio SPI
    """
    # Mode:
    # - v1.1.5: int (0=IDLE, 1=MANUAL, 2=HEAD, 3=HYBRID)
    # - legacy: string ("IDLE"/"RELATIVE_MOVE"/"ABSOLUTE_POSE")
    MODE_TO_CODE_LEGACY = {"IDLE": 0, "RELATIVE_MOVE": 1, "ABSOLUTE_POSE": 2}

    mode_raw = shared_state_data.get("mode", 0)
    if isinstance(mode_raw, int):
        mode_code = max(0, min(255, int(mode_raw)))
    else:
        # Legacy stringhe ("IDLE"/"RELATIVE_MOVE") oppure stringa numerica "1" (evita mode 0 per errore)
        s = str(mode_raw).strip()
        if s.isdigit():
            mode_code = max(0, min(255, int(s)))
        else:
            mode_code = MODE_TO_CODE_LEGACY.get(s, 0)
    
    # Quaternioni (default identità se non presenti)
    quat_w = float(shared_state_data.get("quat_w", 1.0))
    quat_x = float(shared_state_data.get("quat_x", 0.0))
    quat_y = float(shared_state_data.get("quat_y", 0.0))
    quat_z = float(shared_state_data.get("quat_z", 0.0))
    # HEAD/HYBRID/IK MODE richiedono i quaternioni del visore sul data plane.
    if mode_code not in (3, 4, 5):
        quat_w, quat_x, quat_y, quat_z = 1.0, 0.0, 0.0, 0.0
    
    # Pulsanti (default 0 se non presenti)
    buttons_left = int(shared_state_data.get("buttons_left", 0)) & 0xFFFF
    buttons_right = int(shared_state_data.get("buttons_right", 0)) & 0xFFFF

    # CAMCTRL (opzionale): dict {"cmd": "...", "delta": int} oppure stringa "CAMCTRL,cmd,delta"
    camctrl_cmd = 0
    camctrl_delta = 0
    camctrl = shared_state_data.get("camctrl")
    if isinstance(camctrl, dict):
        cmd = camctrl.get("cmd")
        delta = camctrl.get("delta")
        if cmd in ("focus", "zoom", "conv"):
            camctrl_cmd = {"focus": 1, "zoom": 2, "conv": 3}[cmd]
            if isinstance(delta, int):
                camctrl_delta = max(-32768, min(32767, delta))
    else:
        payload_txt = shared_state_data.get("camctrl_payload")
        if isinstance(payload_txt, str) and payload_txt.startswith("CAMCTRL,"):
            try:
                _p = payload_txt.strip().split(",")
                if len(_p) >= 3 and _p[1] in ("focus", "zoom", "conv"):
                    camctrl_cmd = {"focus": 1, "zoom": 2, "conv": 3}[_p[1]]
                    camctrl_delta = max(-32768, min(32767, int(_p[2])))
            except Exception:
                camctrl_cmd = 0
                camctrl_delta = 0

    mode5_arm = dict(shared_state_data.get("mode5_arm") or {})
    mode5_physical = list(mode5_arm.get("physical_deg") or [90, 90, 90])
    while len(mode5_physical) < 3:
        mode5_physical.append(90)
    mode5_control_flags = 0
    if bool(mode5_arm.get("grip_active", False)):
        mode5_control_flags |= 1 << 1
    if bool(mode5_arm.get("hold_active", False)):
        mode5_control_flags |= 1 << 2

    # mode=3 (HEAD): braccio escluso, solo polso via quaternioni → joystick azzerati.
    # mode=5 (ASSIST): B/S/G via estensione 'I', nessun joystick legacy.
    _zero_joy = mode_code in (3, 5)
    joy_x_v = 0 if _zero_joy else shared_state_data.get("joy_x", 0)
    joy_y_v = 0 if _zero_joy else shared_state_data.get("joy_y", 0)
    pitch_v = 0 if _zero_joy else shared_state_data.get("pitch", 0)
    yaw_v   = 0 if _zero_joy else shared_state_data.get("yaw", 0)

    payload = J5VRPayload(
        mode=mode_code,
        joy_x=_norm_float_to_i16(joy_x_v),
        joy_y=_norm_float_to_i16(joy_y_v),
        pitch=_norm_float_to_i16(pitch_v),
        yaw=_norm_float_to_i16(yaw_v),
        intensity=_norm_float_to_u8_intensity(shared_state_data.get("intensity", 0)),
        grip=1 if shared_state_data.get("grip") == 1 else 0,
        vr_heartbeat=int(shared_state_data.get("heartbeat", 0)) & 0xFFFF,
        priority=0,
        safe_mask=0x0000,
        quat_w=quat_w,
        quat_x=quat_x,
        quat_y=quat_y,
        quat_z=quat_z,
        buttons_left=buttons_left,
        buttons_right=buttons_right,
        camctrl_cmd=camctrl_cmd,
        camctrl_delta=camctrl_delta,
        mode5_arm_valid=1 if mode_code == 5 and bool(mode5_arm.get("valid")) else 0,
        mode5_control_flags=mode5_control_flags,
        mode5_target_id=int(mode5_arm.get("target_id", 0)) & 0xFFFF,
        mode5_base_deg=mode5_physical[0],
        mode5_spalla_deg=mode5_physical[1],
        mode5_gomito_deg=mode5_physical[2],
    )
    
    return J5VRFrame(payload=payload, sequence_counter=sequence_counter, frame_type=J5_FRAME_TYPE_J5VR)
