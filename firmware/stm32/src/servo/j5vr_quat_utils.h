/*
 * j5vr_quat_utils.h — Utility matematica quaternioni JONNY5-4.0
 *
 * Libreria pura di algebra quaternioni estratta da j5vr_actuation.c.
 * Indipendente da desired_positions, g_head_* e SETPOSE.
 * Include minimi: math.h + tipi SPI/IMU per i costruttori from_*.
 *
 * JONNY5-4.0 — Step 3.1 refactor (2026-02-25)
 */

#ifndef J5VR_QUAT_UTILS_H
#define J5VR_QUAT_UTILS_H

#include <math.h>

/* Forward declaration dei tipi sorgente (strutture definite altrove).
 * Evita include circolari: questo header non tira tutto j5_protocol.h
 * né imu.h nelle unità che includono solo j5vr_quat_utils.h. */
struct j5vr_state;
struct imu_quat;

/* -----------------------------------------------------------------------
 * Tipo quaternione
 * ----------------------------------------------------------------------- */
typedef struct {
    float w, x, y, z;
} quat_t;

/* -----------------------------------------------------------------------
 * Costruttori da tipi del sistema
 * ----------------------------------------------------------------------- */

/** Costruisce quat_t dal quaternione del visore VR (j5vr_state). */
quat_t quat_from_j5vr(const struct j5vr_state *s);

/** Costruisce quat_t dal quaternione IMU (imu_quat). */
quat_t quat_from_imu(const struct imu_quat *iq);

/* -----------------------------------------------------------------------
 * Operazioni algebriche
 * ----------------------------------------------------------------------- */

/** Normalizza q; ritorna quaternione identità se norma ~0. */
quat_t quat_normalize(quat_t q);

/** Prodotto di Hamilton: a ⊗ b. */
quat_t quat_multiply(quat_t a, quat_t b);

/** Coniugato (inverso per quaternioni unitari): q* = (w, -x, -y, -z). */
quat_t quat_conjugate(quat_t q);

/* -----------------------------------------------------------------------
 * Conversione a angoli di Eulero
 * ----------------------------------------------------------------------- */

/**
 * Converte quaternione (w,x,y,z) in yaw/pitch/roll (gradi).
 * Convenzione: ZYX intrinsec (Tait-Bryan).
 * Puntatori nulli ignorati (output opzionale per asse).
 */
void quat_to_ypr_deg(float w, float x, float y, float z,
                     float *yaw_deg, float *pitch_deg, float *roll_deg);

/**
 * Converte il quaternione errore in componenti del vettore di rotazione
 * locale espresse come yaw/pitch/roll (gradi) in ordine Z/Y/X.
 *
 * A differenza della decomposizione Euler, queste componenti sono piu'
 * adatte al controllo per piccoli/medi errori perche' riducono il coupling
 * tra assi introdotto dalla parametrizzazione ZYX.
 */
void quat_to_rotvec_ypr_deg(float w, float x, float y, float z,
                            float *yaw_deg, float *pitch_deg, float *roll_deg);

/**
 * Estrae dal quaternione errore la twist firmata attorno agli assi locali
 * Z/Y/X, espressa come yaw/pitch/roll (gradi).
 *
 * Rispetto alla semplice conversione in Euler o alle componenti del
 * rotation-vector, questa stima privilegia il contenuto di rotazione
 * attorno a ciascun asse del polso, riducendo l'accoppiamento percepito
 * in HEAD tracking.
 */
void quat_to_twist_ypr_deg(float w, float x, float y, float z,
                           float *yaw_deg, float *pitch_deg, float *roll_deg);

#endif /* J5VR_QUAT_UTILS_H */
