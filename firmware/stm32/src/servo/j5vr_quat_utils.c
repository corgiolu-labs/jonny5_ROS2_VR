/*
 * j5vr_quat_utils.c — Utility matematica quaternioni JONNY5-4.0
 *
 * Libreria pura di algebra quaternioni estratta da j5vr_actuation.c.
 * Dipendenze minime: math.h, j5vr_quat_utils.h,
 *   spi/j5_protocol.h (per struct j5vr_state),
 *   imu/imu.h         (per struct imu_quat).
 *
 * JONNY5-4.0 — Step 3.1 refactor (2026-02-25)
 */

#include "servo/j5vr_quat_utils.h"
#include "spi/j5_protocol.h"
#include "imu/imu.h"
#include <math.h>

/* -----------------------------------------------------------------------
 * Costruttori da tipi del sistema
 * ----------------------------------------------------------------------- */

quat_t quat_from_j5vr(const struct j5vr_state *s)
{
    quat_t q = { 1.0f, 0.0f, 0.0f, 0.0f };
    if (s)
    {
        q.w = s->quat_w;
        q.x = s->quat_x;
        q.y = s->quat_y;
        q.z = s->quat_z;
    }
    return q;
}

/* -----------------------------------------------------------------------
 * Operazioni algebriche
 * ----------------------------------------------------------------------- */

quat_t quat_normalize(quat_t q)
{
    float n = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (n > 1e-6f)
    {
        q.w /= n;
        q.x /= n;
        q.y /= n;
        q.z /= n;
    }
    else
    {
        q.w = 1.0f;
        q.x = 0.0f;
        q.y = 0.0f;
        q.z = 0.0f;
    }
    return q;
}

quat_t quat_multiply(quat_t a, quat_t b)
{
    quat_t r;
    r.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
    r.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
    r.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
    r.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
    return r;
}

quat_t quat_conjugate(quat_t q)
{
    quat_t r;
    r.w = q.w;
    r.x = -q.x;
    r.y = -q.y;
    r.z = -q.z;
    return r;
}

void quat_to_twist_ypr_deg(float w, float x, float y, float z,
                           float *yaw_deg, float *pitch_deg, float *roll_deg)
{
    quat_t q = quat_normalize((quat_t){ w, x, y, z });

    /* q e -q rappresentano la stessa rotazione: scegliamo la rappresentazione
     * con parte scalare positiva per minimizzare la twist firmata estratta. */
    if (q.w < 0.0f)
    {
        q.w = -q.w;
        q.x = -q.x;
        q.y = -q.y;
        q.z = -q.z;
    }

    const float rad2deg = 57.2957795f;
    const float w_clamped = (q.w > 1.0f) ? 1.0f : (q.w < -1.0f ? -1.0f : q.w);

    /* Twist attorno a X/Y/Z locale:
     * si proietta la parte vettoriale sull'asse desiderato e si ricostruisce
     * il quaternione di twist corrispondente. */
    const float px = q.x;
    const float py = q.y;
    const float pz = q.z;

    const float ax = fabsf(px);
    const float ay = fabsf(py);
    const float az = fabsf(pz);

    const float twist_x = (ax > 1e-6f) ? copysignf(2.0f * atan2f(ax, w_clamped), px) : 0.0f;
    const float twist_y = (ay > 1e-6f) ? copysignf(2.0f * atan2f(ay, w_clamped), py) : 0.0f;
    const float twist_z = (az > 1e-6f) ? copysignf(2.0f * atan2f(az, w_clamped), pz) : 0.0f;

    if (yaw_deg)   { *yaw_deg   = twist_z * rad2deg; }
    if (pitch_deg) { *pitch_deg = twist_y * rad2deg; }
    if (roll_deg)  { *roll_deg  = twist_x * rad2deg; }
}
