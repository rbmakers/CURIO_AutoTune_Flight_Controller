#ifndef __CURIO_MAHONY_H__
#define __CURIO_MAHONY_H__

#include <Arduino.h>

// ================================================================
//  CURIO Mahony AHRS 姿態估計模組
//  從 CURIO_AutoTune_Dual_PID.ino 拆分而來
//  製作 : 火箭鳥創客倉庫
// ================================================================

// --- 濾波器增益 (可依機體響應調整) ---
#define MAHONY_KP 2.0f
#define MAHONY_KI 0.005f

// ----------------------------------------------------------------
// 對外 API
// ----------------------------------------------------------------

// 用一次 IMU 取樣更新姿態四元數估計
//   ax, ay, az : 加速度計讀值 (任意一致單位即可，函式內部會自行正規化)
//   gx, gy, gz : 陀螺儀讀值，務必是 rad/s (呼叫前自行做 deg->rad 轉換)
//   dt         : 與上次呼叫的時間間隔 (秒)
void mahonyUpdate(float ax, float ay, float az, float gx, float gy, float gz, float dt);

// 取得目前估計姿態 (單位: 度)
float mahonyGetRoll();
float mahonyGetPitch();

// 重置四元數與積分誤差為水平姿態 (一般不需要主動呼叫，保留供未來擴充)
void mahonyReset();

#endif
