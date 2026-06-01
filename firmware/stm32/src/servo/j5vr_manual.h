/*
 * j5vr_manual.h — Modalità MANUAL VR (mode 1) — helper puri
 *
 * Contiene le funzioni di conversione input joystick/int16 estratte da
 * j5vr_actuation.c. Queste funzioni sono stateless e non dipendono da
 * nessuna variabile globale del modulo actuation.
 *
 * Le funzioni che accedono a stato condiviso tra moduli (safe_increment,
 * update_max_velocity_from_buttons, j5vr_apply_setpoint_incremental)
 * rimangono in j5vr_actuation.c/.h poiché condividono variabili globali
 * con i moduli HEAD e SETPOSE.
 *
 * JONNY5-4.0 — Step 3.2 refactor (2026-02-25)
 */

#ifndef J5VR_MANUAL_H
#define J5VR_MANUAL_H

#include <stdint.h>
#include <math.h>

/* -----------------------------------------------------------------------
 * Conversione input joystick
 * ----------------------------------------------------------------------- */

/**
 * Converte int16 [-32768, 32767] a float normalizzato [-1.0, 1.0].
 * Asimmetrico: positivo ÷ 32767, negativo ÷ 32768.
 */
float j5vr_int16_to_normalized(int16_t val);

/**
 * Versione con dead zone: i valori entro ±deadzone vengono azzerati.
 * Il residuo viene rescalato linearmente su [0, 1] per eliminare
 * il gradino alla soglia.
 */
float j5vr_int16_to_normalized_dz(int16_t val, float deadzone);

/**
 * Converte float normalizzato [-1.0, 1.0] in angolo uint8
 * nell'intervallo [center-range, center+range], clampato a [0, 180].
 */
uint8_t j5vr_normalized_to_angle(float normalized, uint8_t center, uint8_t range);

#endif /* J5VR_MANUAL_H */
