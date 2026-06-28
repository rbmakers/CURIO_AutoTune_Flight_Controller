/*
    CURIO Mahony AHRS 姿態估計模組 — 實作檔
    從 CURIO_AutoTune_Dual_PID.ino 拆分而來
    製作 : 火箭鳥創客倉庫
*/

#include "mahony.h"
#include <math.h>

// ================================================================
//  內部濾波器狀態 (對外不可見，僅透過下方 API 存取)
// ================================================================
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float eIntX = 0.0f, eIntY = 0.0f, eIntZ = 0.0f;
static float roll_deg = 0.0f, pitch_deg = 0.0f;

void mahonyUpdate(float ax, float ay, float az, float gx, float gy, float gz, float dt) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        if (MAHONY_KI > 0.0f) {
            eIntX += halfex * MAHONY_KI * dt;
            eIntY += halfey * MAHONY_KI * dt;
            eIntZ += halfez * MAHONY_KI * dt;
            gx += eIntX; gy += eIntY;
            gz += eIntZ;
        }
        gx += halfex * MAHONY_KP;
        gy += halfey * MAHONY_KP;
        gz += halfez * MAHONY_KP;
    }

    gx *= (0.5f * dt);
    gy *= (0.5f * dt); gz *= (0.5f * dt);
    float qa = q0, qb = q1, qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);
    recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;

    roll_deg  = atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2) * 57.29578f;
    pitch_deg = asinf(-2.0f * (q1 * q3 - q0 * q2)) * 57.29578f;
}

float mahonyGetRoll() {
    return roll_deg;
}

float mahonyGetPitch() {
    return pitch_deg;
}

void mahonyReset() {
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    eIntX = 0.0f; eIntY = 0.0f; eIntZ = 0.0f;
    roll_deg = 0.0f; pitch_deg = 0.0f;
}
