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

quat_t quat_from_imu(const struct imu_quat *iq)
{
    quat_t q = { 1.0f, 0.0f, 0.0f, 0.0f };
    if (iq)
    {
        q.w = iq->w;
        q.x = iq->x;
        q.y = iq->y;
        q.z = iq->z;
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

/* -----------------------------------------------------------------------
 * Conversione a angoli di Eulero
 * ----------------------------------------------------------------------- */

void quat_to_ypr_deg(float w, float x, float y, float z,
                     float *yaw_deg, float *pitch_deg, float *roll_deg)
{
    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    float roll = atan2f(sinr_cosp, cosr_cosp);

    float sinp = 2.0f * (w * y - z * x);
    float pitch;
    if (fabsf(sinp) >= 1.0f)
        pitch = copysignf(1.57079632679f, sinp);
    else
        pitch = asinf(sinp);

    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    float yaw = atan2f(siny_cosp, cosy_cosp);

    const float rad2deg = 57.2957795f;
    if (yaw_deg)   { *yaw_deg   = yaw   * rad2deg; }
    if (pitch_deg) { *pitch_deg = pitch * rad2deg; }
    if (roll_deg)  { *roll_deg  = roll  * rad2deg; }
}

void quat_to_rotvec_ypr_deg(float w, float x, float y, float z,
                            float *yaw_deg, float *pitch_deg, float *roll_deg)
{
    quat_t q = quat_normalize((quat_t){ w, x, y, z });

    /* q e -q rappresentano la stessa rotazione: scegliamo l'arco piu' corto. */
    if (q.w < 0.0f)
    {
        q.w = -q.w;
        q.x = -q.x;
        q.y = -q.y;
        q.z = -q.z;
    }

    const float vnorm = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z);
    float rx = 0.0f, ry = 0.0f, rz = 0.0f;

    if (vnorm > 1e-6f)
    {
        const float angle = 2.0f * atan2f(vnorm, q.w);
        const float scale = angle / vnorm;
        rx = q.x * scale;
        ry = q.y * scale;
        rz = q.z * scale;
    }
    else
    {
        /* Approssimazione small-angle: rotvec ~= 2 * v. */
        rx = 2.0f * q.x;
        ry = 2.0f * q.y;
        rz = 2.0f * q.z;
    }

    const float rad2deg = 57.2957795f;
    if (yaw_deg)   { *yaw_deg   = rz * rad2deg; }
    if (pitch_deg) { *pitch_deg = ry * rad2deg; }
    if (roll_deg)  { *roll_deg  = rx * rad2deg; }
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
