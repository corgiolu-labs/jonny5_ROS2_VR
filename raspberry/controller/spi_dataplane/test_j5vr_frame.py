#!/usr/bin/env python3
"""
test_j5vr_frame.py — regressione offline (niente robot, niente SPI) per j5vr_frame.py.

Bug 2026-10-24: _norm_float_to_i16() trattava un int in -1..1 come float normalizzato, quindi un asse int16
già scalato di +1/-1 (stick a 0,003 % dal centro, uscita normale di validate_and_build_intent) diventava
+32767/-32767 nel frame J5VR: un comando a fondo scala. Trovato confrontando il payload con l'app Quest
nativa (jonny5-quest-viewer, stream/.../j5/J5ArmPayloads.kt, tools/j5vr_vectors.py).

    cd raspberry && python -m unittest controller.spi_dataplane.test_j5vr_frame -v
    (oppure: python raspberry/controller/spi_dataplane/test_j5vr_frame.py)
"""
import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from controller.spi_dataplane import j5vr_frame as jf  # noqa: E402


def axes(payload: bytes):
    return struct.unpack_from(">hhhh", payload, 1)   # joy_x, joy_y, pitch, yaw


class NormFloatToI16(unittest.TestCase):
    def test_small_ints_stay_ints(self):
        self.assertEqual(jf._norm_float_to_i16(1), 1)
        self.assertEqual(jf._norm_float_to_i16(-1), -1)
        self.assertEqual(jf._norm_float_to_i16(0), 0)

    def test_ints_are_clamped_not_scaled(self):
        self.assertEqual(jf._norm_float_to_i16(12000), 12000)
        self.assertEqual(jf._norm_float_to_i16(40000), 32767)
        self.assertEqual(jf._norm_float_to_i16(-40000), -32768)

    def test_floats_are_normalised(self):
        self.assertEqual(jf._norm_float_to_i16(0.5), 16384)      # round(16383.5) = 16384 (half-even)
        self.assertEqual(jf._norm_float_to_i16(-0.25), -8192)    # round(-8191.75)
        self.assertEqual(jf._norm_float_to_i16(1.0), 32767)
        self.assertEqual(jf._norm_float_to_i16(2.0), 32767)
        self.assertEqual(jf._norm_float_to_i16("x"), 0)


class SetpointFrame(unittest.TestCase):
    def frame(self, **kw):
        state = {"mode": 2, "joy_x": 0, "joy_y": 0, "pitch": 0, "yaw": 0, "intensity": 0, "grip": 1, "heartbeat": 7}
        state.update(kw)
        return jf.build_setpoint_frame(state).to_bytes()

    def test_axis_of_one_is_not_full_scale(self):
        f = self.frame(joy_x=1, joy_y=-1, pitch=1, yaw=-1)
        self.assertEqual(axes(f[8:62]), (1, -1, 1, -1))
        self.assertEqual(f[9:13], bytes.fromhex("0001ffff"))           # payload[1..4] = joy_x, joy_y BE

    def test_regular_values_unchanged(self):
        self.assertEqual(axes(self.frame(joy_x=12000, joy_y=-32768, pitch=32767, yaw=0)[8:62]),
                         (12000, -32768, 32767, 0))

    def test_full_pi_path_from_intent_json(self):
        """The Quest's JSON -> ws_handlers_intent.validate_and_build_intent -> build_setpoint_frame."""
        try:
            from controller.web_services import ws_handlers_intent as wi
        except Exception as e:  # pragma: no cover - optional deps of the web services
            self.skipTest(f"ws_handlers_intent not importable here: {e}")
        intent, err = wi.validate_and_build_intent(
            {"mode": 2, "joy_x": 1, "joy_y": -1, "pitch": 0.5, "yaw": 0, "intensity": 3, "grip": 1, "heartbeat": 9})
        self.assertIsNone(err)
        p = jf.build_setpoint_frame(intent).payload.to_bytes()
        self.assertEqual(axes(p), (1, -1, 16384, 0))


if __name__ == "__main__":
    unittest.main(verbosity=2)
