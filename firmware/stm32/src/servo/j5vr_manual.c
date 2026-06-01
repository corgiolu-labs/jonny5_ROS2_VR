/*
 * j5vr_manual.c — Modalità MANUAL VR (mode 1) — helper puri
 *
 * Implementazione delle funzioni di conversione input joystick/int16.
 * Nessuna dipendenza da variabili globali di actuation.
 * Dipendenze minime: j5vr_manual.h, math.h.
 *
 * JONNY5-4.0 — Step 3.2 refactor (2026-02-25)
 */

#include "servo/j5vr_manual.h"
#include <math.h>

float j5vr_int16_to_normalized(int16_t val)
{
    if (val >= 0)
    {
        return (float)val / 32767.0f;
    }
    else
    {
        return (float)val / 32768.0f;
    }
}

float j5vr_int16_to_normalized_dz(int16_t val, float deadzone)
{
    float normalized = j5vr_int16_to_normalized(val);
    if (fabsf(normalized) < deadzone)
    {
        return 0.0f;
    }
    float sign = normalized >= 0.0f ? 1.0f : -1.0f;
    float abs_val = fabsf(normalized);
    return sign * ((abs_val - deadzone) / (1.0f - deadzone));
}

uint8_t j5vr_normalized_to_angle(float normalized, uint8_t center, uint8_t range)
{
    float angle_float = center + (normalized * range);
    if (angle_float < 0.0f) angle_float = 0.0f;
    if (angle_float > 180.0f) angle_float = 180.0f;
    return (uint8_t)roundf(angle_float);
}
