// ================================================================
//  CURIO_AutoTune_Dual_PID.ino
//  雙環串接 (Angle外環+Rate內環) 自調校飛行控制韌體
//  硬體平台：CURIO (RP2350 / RP2040)，火箭鳥創客倉庫
//  模組拆分：
//    elrs.h     / elrs.cpp     — ELRS/CRSF 接收機
//    autotune.h / autotune.cpp — 繼電器回授自動調校 + Flash 增益持久化
//    mahony.h   / mahony.cpp   — Mahony AHRS 姿態估計
// ================================================================

#include <Arduino.h>
#include <Wire.h>
#include "BMI088.h"
#include "elrs.h"
#include "autotune.h"
#include "mahony.h"

// ================================================================
// 🛠️ 手動函式原型宣告 (Function Prototypes)
//    (型態定義已移至 autotune.h，此處僅保留主程式自身函式，
//     維持防止 Arduino-cli 自動原型產生器解析順序問題之慣例)
// ================================================================
void calibrateSensors();
float stickToTarget(unsigned long us, float max_value);
int throttleToBase(unsigned long us);
float computeAxisOutput(AxisState &axis, float measured_angle, float measured_rate, bool run_outer, float outer_dt, float inner_dt, unsigned long now_ms);

// ==========================================
// 🛠️ 硬體接腳與參數定義
// ==========================================
const int M1_PIN = 0;   // 右前 (CW)
const int M2_PIN = 11;  // 右後 (CCW)
const int M3_PIN = 14;  // 左後 (CW)
const int M4_PIN = 28;  // 左前 (CCW)

const int LED_A = 7;    // 解鎖狀態 (常亮=已解鎖)
const int LED_B = 8;    // 狀態指示燈 (藍)
const int LED_C = 9;    // 狀態指示燈 (紅)

const int I2C0_SDA = 20;
const int I2C0_SCL = 25;
#define BMI088_ACC_ADDR  0x18
#define BMI088_GYRO_ADDR 0x69

const int PWM_FREQ = 20000;
const int PWM_RANGE = 255;
const int MOTOR_MAX_LIMIT = 216;   // 85% 物理電流防護限制
const int MOTOR_IDLE_PWM  = 15;    // 解鎖時最低馬達轉速

const int THROTTLE_HEADROOM_RESERVE = 36;
const int THROTTLE_MAX_MAPPED = MOTOR_MAX_LIMIT - THROTTLE_HEADROOM_RESERVE; // = 180

// ==========================================
// 🕹️ 飛行安全參數
//    (RC_US_MID 加上 extern 使其具有外部連結性，
//     autotune.cpp 的 autotunePreconditionsOk() 需要 extern 參照此值)
// ==========================================
extern const unsigned long RC_US_MIN = 1000, RC_US_MID = 1500, RC_US_MAX = 2000;
const unsigned long ARM_SWITCH_THRESHOLD_US = 1500;
const unsigned long ARM_THROTTLE_MAX_US     = 1100;
const unsigned long FAILSAFE_TIMEOUT_MS     = 300;   

const float SEVERE_TILT_CUTOFF_DEG = 75.0f;

const float MAX_STICK_ANGLE    = 30.0f;  
const float MAX_STICK_YAWRATE  = 180.0f;
const unsigned long STICK_DEADBAND_US = 15;

const unsigned long EMERGENCY_SWITCH_THRESHOLD_US = 1500;
const float EMERGENCY_DESCENT_RATE_PWM_PER_S = 35.0f; 
const int   EMERGENCY_LAND_PWM_THRESHOLD = MOTOR_IDLE_PWM + 5;
const unsigned long EMERGENCY_LAND_CONFIRM_MS = 1000; 

// ==========================================
// 📐 姿態估計 (BMI088 讀值 + Mahony Filter，見 mahony.cpp)
// ==========================================
BMI088 bmi088(Wire, BMI088_ACC_ADDR, BMI088_GYRO_ADDR);
float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
float ax_offset = 0, ay_offset = 0, az_offset = 0;
float gx_offset = 0, gy_offset = 0, gz_offset = 0;
unsigned long lastUpdate = 0;
float roll = 0.0f, pitch = 0.0f;   // 每個 loop tick 由 mahonyGetRoll()/mahonyGetPitch() 更新快取

float target_yaw_rate = 0.0f; 
float pid_p_yaw = 1.5f;
unsigned long last_pid_time = 0;
bool is_armed = false;

// ==========================================
// 🎛️ 雙環串接 PID 變數宣告 (AxisState/SystemMode 型態定義見 autotune.h)
// ==========================================
const int ANGLE_LOOP_DIVIDER = 5;
const float MAX_TARGET_RATE  = 250.0f; 

int angle_loop_counter = 0;
unsigned long last_angle_loop_time = 0;

AxisState roll_axis;
AxisState pitch_axis;

SystemMode current_mode = MODE_NORMAL;
bool autotune_switch_prev = false;
bool autotune_request_pending = false;
unsigned long tune_done_flash_until = 0;

float emergency_descend_base = 0.0f;
unsigned long emergency_descend_start_ms = 0;
unsigned long emergency_low_throttle_since_ms = 0;
int last_motor_base = 0; 

// ==========================================
// 🕒 感測器校正
// ==========================================
void calibrateSensors() {
    digitalWrite(LED_C, HIGH);
    Serial.println("正在校正感測器，請保持機體靜止水平...");
    long samples = 500;
    for (int i = 0; i < samples; i++) {
        bmi088.getAcceleration(&ax, &ay, &az);
        bmi088.getGyroscope(&gx, &gy, &gz);
        ax_offset += ax; ay_offset += ay; az_offset += (az - 9.80665f);
        gx_offset += gx; gy_offset += gy;
        gz_offset += gz;
        delay(4);
    }
    ax_offset /= samples; ay_offset /= samples; az_offset /= samples;
    gx_offset /= samples; gy_offset /= samples; gz_offset /= samples;
    digitalWrite(LED_C, LOW);
    Serial.println("✅ 校正完成！");
}

// ==========================================
// 🕹️ 搖桿 / 油門 映射函數
// ==========================================
float stickToTarget(unsigned long us, float max_value) {
    long centered = (long)us - (long)RC_US_MID;
    if (abs(centered) < (long)STICK_DEADBAND_US) centered = 0;
    float normalized = constrain((float)centered / 500.0f, -1.0f, 1.0f);
    return normalized * max_value;
}

int throttleToBase(unsigned long us) {
    long mapped = map((long)us, (long)RC_US_MIN, (long)RC_US_MAX, 0, THROTTLE_MAX_MAPPED);
    return (int)constrain(mapped, 0L, (long)THROTTLE_MAX_MAPPED);
}

// ------------------------------------------
// 雙環 PID 輸出計算 (一般飛行 / 自動調校共用)
// 調校模式下會呼叫 autotune.cpp 提供的 autotuneRelayStep() 取代對應迴路
// ------------------------------------------
float computeAxisOutput(AxisState &axis, float measured_angle, float measured_rate,
                         bool run_outer, float outer_dt, float inner_dt, unsigned long now_ms) {

    if (axis.mode == AXIS_TUNE_RATE) {
        return autotuneRelayStep(axis.rt_rate, measured_rate, 0.0f, now_ms,
                          RELAY_AMPLITUDE_RATE, RELAY_HYSTERESIS_RATE);
    }

    if (axis.mode == AXIS_TUNE_ANGLE) {
        if (run_outer) {
            axis.target_rate = autotuneRelayStep(axis.rt_angle, measured_angle, axis.target_angle, now_ms,
                                          RELAY_AMPLITUDE_ANGLE, RELAY_HYSTERESIS_ANGLE);
        }
        float err_rate = axis.target_rate - measured_rate;
        axis.int_rate += err_rate * inner_dt;
        axis.int_rate = constrain(axis.int_rate, -50.0f, 50.0f);
        float deriv_rate = (err_rate - axis.err_prev_rate) / inner_dt;
        float output = (axis.pid_p_rate * err_rate) + (axis.pid_i_rate * axis.int_rate) + (axis.pid_d_rate * deriv_rate);
        axis.err_prev_rate = err_rate;
        return output;
    }

    if (run_outer) {
        float err_angle = axis.target_angle - measured_angle;
        axis.int_angle += err_angle * outer_dt;
        axis.int_angle = constrain(axis.int_angle, -50.0f, 50.0f);
        float deriv_angle = (err_angle - axis.err_prev_angle) / outer_dt;
        float new_target_rate = (axis.pid_p_angle * err_angle) + (axis.pid_i_angle * axis.int_angle) + (axis.pid_d_angle * deriv_angle);
        axis.err_prev_angle = err_angle;
        axis.target_rate = constrain(new_target_rate, -MAX_TARGET_RATE, MAX_TARGET_RATE);
    }

    float err_rate = axis.target_rate - measured_rate;
    axis.int_rate += err_rate * inner_dt;
    axis.int_rate = constrain(axis.int_rate, -50.0f, 50.0f);
    float deriv_rate = (err_rate - axis.err_prev_rate) / inner_dt;
    float output = (axis.pid_p_rate * err_rate) + (axis.pid_i_rate * axis.int_rate) + (axis.pid_d_rate * deriv_rate);
    axis.err_prev_rate = err_rate;
    return output;
}

// ==========================================
// 🚀 Arduino 核心 Setup & Loop
// ==========================================
void setup() {
    Serial.begin(115200);
    delay(200);

    // 開機先讀 Flash：若先前已完成過自動調校，直接覆蓋下方 AxisState 預設增益 (見 autotune.cpp)
    autotuneLoadGains();

    pinMode(M1_PIN, OUTPUT); pinMode(M2_PIN, OUTPUT);
    pinMode(M3_PIN, OUTPUT); pinMode(M4_PIN, OUTPUT);
    pinMode(LED_A, OUTPUT);  pinMode(LED_B, OUTPUT); pinMode(LED_C, OUTPUT);

    analogWriteFreq(PWM_FREQ);
    analogWriteRange(PWM_RANGE);
    analogWrite(M1_PIN, 0); analogWrite(M2_PIN, 0);
    analogWrite(M3_PIN, 0); analogWrite(M4_PIN, 0);

    elrsInit();   // ELRS/CRSF 接收機初始化 (見 elrs.cpp)

    Wire.setSDA(I2C0_SDA);
    Wire.setSCL(I2C0_SCL);
    Wire.begin();
    while (!bmi088.isConnection()) {
        Serial.println("❌ BMI088 連線失敗，檢查硬體中...");
        digitalWrite(LED_C, HIGH); delay(500); digitalWrite(LED_C, LOW); delay(500);
    }
    bmi088.initialize();

    calibrateSensors();

    if (RELAY_AMPLITUDE_RATE > (float)MOTOR_MAX_LIMIT / 2.0f) {
        Serial.println("⚠️ 警告: RELAY_AMPLITUDE_RATE 設定過大，請重新確認！");
    }

    Serial.println("==========================================");
    Serial.println(" 飛控就緒");
    Serial.println(" 解鎖   : CH5開 + 油門最低");
    Serial.println(" 自動調校: 起飛懸停後 CH6(AUX1)開 (需搖桿置中+油門在懸停區間)");
    Serial.println(" 緊急緩降: 任何狀態下 CH7(AUX2)開，扳回可取消");
    Serial.println(" 地面測試 Serial 指令: t/r/a/x");
    Serial.println("==========================================");

    lastUpdate = micros();
    last_pid_time = millis();
    last_angle_loop_time = millis();
    digitalWrite(LED_A, LOW);
}

void loop() {
    // 0. 讀取 ELRS/CRSF (見 elrs.cpp)
    elrsUpdate();

    // 0b. 地面測試用 Serial 指令
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        if (cmd == 'x' || cmd == 'X') {
            abortAutotune("使用者手動中止 (Serial)");
        } else if (current_mode == MODE_NORMAL && is_armed) {
            bool ok = autotunePreconditionsOk(elrsGetChannel(CH_ROLL), elrsGetChannel(CH_PITCH), elrsGetChannel(CH_THROTTLE));
            if (!ok) {
                Serial.println("⚠️ 搖桿未置中或油門不在懸停區間，拒絕開始自動調校");
            } else {
                if (cmd == 't' || cmd == 'T') startFullAutotune();
                else if (cmd == 'r' || cmd == 'R') startRateOnlyAutotune();
                else if (cmd == 'a' || cmd == 'A') startAngleOnlyAutotune();
            }
        }
    }

    // 1. IMU 讀取與姿態估計 (Mahony，見 mahony.cpp)
    bmi088.getAcceleration(&ax, &ay, &az);
    bmi088.getGyroscope(&gx, &gy, &gz);
    ax -= ax_offset; ay -= ay_offset; az -= az_offset;
    gx -= gx_offset; gy -= gy_offset;
    gz -= gz_offset;

    unsigned long now = micros();
    float dt = (now - lastUpdate) / 1000000.0f;
    lastUpdate = now;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.002f;

    float gx_rad = gx * 0.0174533f;
    float gy_rad = gy * 0.0174533f;
    float gz_rad = gz * 0.0174533f;
    mahonyUpdate(ax, ay, az, gx_rad, gy_rad, gz_rad, dt);
    roll  = mahonyGetRoll();
    pitch = mahonyGetPitch();

    // 2. 核心控制迴路 (內環固定頻率：500Hz / 2ms)
    unsigned long currentTime = millis();
    float pid_dt = (currentTime - last_pid_time) / 1000.0f;

    if (pid_dt >= 0.002f) {
        last_pid_time = currentTime;
        unsigned long roll_us     = elrsGetChannel(CH_ROLL);
        unsigned long pitch_us    = elrsGetChannel(CH_PITCH);
        unsigned long yaw_us      = elrsGetChannel(CH_YAW);
        unsigned long throttle_us = elrsGetChannel(CH_THROTTLE);
        bool armSwitchOn       = elrsGetChannel(CH_ARM)               > ARM_SWITCH_THRESHOLD_US;
        bool autotuneSwitchOn  = elrsGetChannel(CH_AUTOTUNE)          > AUTOTUNE_SWITCH_THRESHOLD_US;
        bool emergencySwitchOn = elrsGetChannel(CH_EMERGENCY_DESCEND) > EMERGENCY_SWITCH_THRESHOLD_US;
        bool linkOk = elrsLinkOk(FAILSAFE_TIMEOUT_MS);

        // --- 失聯保護 (Failsafe) ---
        if (!linkOk) {
            if (is_armed) {
                is_armed = false;
                if (current_mode != MODE_NORMAL) abortAutotune("ELRS 失聯 (Failsafe)");
            }
        } else {
            // --- 解鎖 / 上鎖互鎖 ---
            if (!is_armed && armSwitchOn && throttle_us < ARM_THROTTLE_MAX_US) {
                is_armed = true;
                resetAxisIntegrators(roll_axis);
                resetAxisIntegrators(pitch_axis);
            } else if (is_armed && !armSwitchOn) {
                is_armed = false;
                if (current_mode != MODE_NORMAL) abortAutotune("解鎖開關關閉");
            }
        }
        digitalWrite(LED_A, is_armed ? HIGH : LOW);

        // --- 嚴重傾斜安全斷電 ---
        if (is_armed && (abs(roll) > SEVERE_TILT_CUTOFF_DEG || abs(pitch) > SEVERE_TILT_CUTOFF_DEG)) {
            is_armed = false;
            if (current_mode != MODE_NORMAL) abortAutotune("傾斜角度超過安全門檻");
        }

        // --- 外環(Angle Loop)分頻判斷 ---
        angle_loop_counter++;
        bool run_outer = false;
        float outer_dt = 0.0f;
        if (angle_loop_counter >= ANGLE_LOOP_DIVIDER) {
            angle_loop_counter = 0;
            run_outer = true;
            outer_dt = (currentTime - last_angle_loop_time) / 1000.0f;
            last_angle_loop_time = currentTime;
            if (outer_dt <= 0.0f || outer_dt > 0.5f) outer_dt = ANGLE_LOOP_DIVIDER * 0.002f;
        }

        float output_roll = 0, output_pitch = 0, output_yaw = 0;
        int motor_base = 0;

        if (is_armed && linkOk) {
            // --- 自動調校開關 edge-trigger 偵測 ---
            bool autotune_rising_edge = autotuneSwitchOn && !autotune_switch_prev;
            autotune_switch_prev = autotuneSwitchOn;
            if (autotune_rising_edge) autotune_request_pending = true;
            if (!autotuneSwitchOn) autotune_request_pending = false;

            // --- 緊急自動減速緩降 ---
            if (emergencySwitchOn && current_mode != MODE_EMERGENCY_DESCEND) {
                if (current_mode != MODE_NORMAL) abortAutotune("緊急自動減速緩降已觸發，調校中止");
                current_mode = MODE_EMERGENCY_DESCEND;
                emergency_descend_base = (float)last_motor_base;
                emergency_descend_start_ms = currentTime;
                emergency_low_throttle_since_ms = 0;
                autotune_request_pending = false;
                Serial.println("🛑 緊急自動減速緩降啟動！");
            } else if (!emergencySwitchOn && current_mode == MODE_EMERGENCY_DESCEND) {
                current_mode = MODE_NORMAL;
                Serial.println("緊急緩降已取消，恢復遙控操作");
            }

            if (current_mode == MODE_EMERGENCY_DESCEND) {
                roll_axis.target_angle  = 0.0f;
                pitch_axis.target_angle = 0.0f;
                target_yaw_rate = 0.0f;

                float err_yaw_rate = target_yaw_rate - gz;
                output_yaw = pid_p_yaw * err_yaw_rate;
                output_roll  = computeAxisOutput(roll_axis,  roll,  gx, run_outer, outer_dt, pid_dt, currentTime);
                output_pitch = computeAxisOutput(pitch_axis, pitch, gy, run_outer, outer_dt, pid_dt, currentTime);

                float elapsed_s = (currentTime - emergency_descend_start_ms) / 1000.0f;
                float ramped = emergency_descend_base - EMERGENCY_DESCENT_RATE_PWM_PER_S * elapsed_s;
                motor_base = (int)constrain(ramped, 0.0f, (float)MOTOR_MAX_LIMIT);
                if (motor_base <= EMERGENCY_LAND_PWM_THRESHOLD) {
                    if (emergency_low_throttle_since_ms == 0) emergency_low_throttle_since_ms = currentTime;
                    if (currentTime - emergency_low_throttle_since_ms > EMERGENCY_LAND_CONFIRM_MS) {
                        is_armed = false;
                        current_mode = MODE_NORMAL;
                        Serial.println("✅ 緊急緩降完成，已自動上鎖");
                    }
                } else {
                    emergency_low_throttle_since_ms = 0;
                }

            } else {
                // --- 正常飛行 / 自動調校 ---
                roll_axis.target_angle  = stickToTarget(roll_us,  MAX_STICK_ANGLE);
                pitch_axis.target_angle = stickToTarget(pitch_us, MAX_STICK_ANGLE);
                target_yaw_rate         = stickToTarget(yaw_us,   MAX_STICK_YAWRATE);
                if (current_mode == MODE_NORMAL) {
                    if (autotune_request_pending && autotunePreconditionsOk(roll_us, pitch_us, throttle_us)) {
                        startFullAutotune();
                        autotune_request_pending = false; 
                    }
                } else {
                    if (!autotuneSwitchOn || !autotunePreconditionsOk(roll_us, pitch_us, throttle_us)) {
                        abortAutotune("飛行中安全前提不再滿足 (開關/搖桿/油門)");
                    }
                }

                float err_yaw_rate = target_yaw_rate - gz;
                output_yaw = pid_p_yaw * err_yaw_rate;
                output_roll  = computeAxisOutput(roll_axis,  roll,  gx, run_outer, outer_dt, pid_dt, currentTime);
                output_pitch = computeAxisOutput(pitch_axis, pitch, gy, run_outer, outer_dt, pid_dt, currentTime);

                // --- 檢查目前調校階段是否完成/逾時，並自動推進或中止 (見 autotune.cpp) ---
                // --- 四階段全部完成時，autotuneUpdate() 內部會自動呼叫 autotuneSaveGains() 寫入 Flash ---
                autotuneUpdate();

                motor_base = max(throttleToBase(throttle_us), MOTOR_IDLE_PWM);
            }

            last_motor_base = motor_base;
        }

        // 3. 馬達混控 (X型混控架構)
        int m1 = motor_base - output_roll + output_pitch + output_yaw;
        int m2 = motor_base - output_roll - output_pitch - output_yaw;
        int m3 = motor_base + output_roll - output_pitch + output_yaw;
        int m4 = motor_base + output_roll + output_pitch - output_yaw;
        if (is_armed && linkOk) {
            m1 = constrain(m1, 0, MOTOR_MAX_LIMIT);
            m2 = constrain(m2, 0, MOTOR_MAX_LIMIT);
            m3 = constrain(m3, 0, MOTOR_MAX_LIMIT);
            m4 = constrain(m4, 0, MOTOR_MAX_LIMIT);
        } else {
            m1 = m2 = m3 = m4 = 0;
        }

        analogWrite(M1_PIN, m1);
        analogWrite(M2_PIN, m2);
        analogWrite(M3_PIN, m3);
        analogWrite(M4_PIN, m4);
    }

    // 4. LED 狀態燈號 (10Hz 統一更新)
    static unsigned long led_timer = 0;
    if (millis() - led_timer > 100) {
        led_timer = millis();
        bool linkOkNow = elrsLinkOk(FAILSAFE_TIMEOUT_MS);

        if (!linkOkNow) {
            digitalWrite(LED_C, HIGH);
            digitalWrite(LED_B, LOW);
        } else if (current_mode == MODE_EMERGENCY_DESCEND) {
            static bool toggle = false;
            toggle = !toggle;
            digitalWrite(LED_B, toggle ? HIGH : LOW);
            digitalWrite(LED_C, toggle ? LOW : HIGH);
        } else if (current_mode != MODE_NORMAL) {
            digitalWrite(LED_C, !digitalRead(LED_C));
            digitalWrite(LED_B, LOW);
        } else if (millis() < tune_done_flash_until) {
            digitalWrite(LED_B, HIGH);
            digitalWrite(LED_C, LOW);
        } else if (is_armed) {
            digitalWrite(LED_C, LOW);
            static byte blink_cnt = 0;
            blink_cnt++;
            if (blink_cnt % 5 == 0) digitalWrite(LED_B, !digitalRead(LED_B));
        } else {
            digitalWrite(LED_B, LOW);
            digitalWrite(LED_C, LOW);
        }
    }

    // 5. 低頻地面除錯輸出 (10Hz)
    static unsigned long last_debug = 0;
    if (millis() - last_debug > 100) {
        last_debug = millis();
        Serial.print("ARM:");
        Serial.print(is_armed ? 1 : 0);
        Serial.print(" LINK:"); Serial.print(elrsLinkOk(FAILSAFE_TIMEOUT_MS) ? 1 : 0);
        Serial.print(" MODE:"); Serial.print(autotuneModeName(current_mode));
        Serial.print(" R:"); Serial.print(roll, 1);
        Serial.print(" P:"); Serial.print(pitch, 1);
        Serial.print(" THR:"); Serial.print(elrsGetChannel(CH_THROTTLE));
        Serial.print(" AUX1:"); Serial.print(elrsGetChannel(CH_AUTOTUNE));
        Serial.print(" AUX2:"); Serial.print(elrsGetChannel(CH_EMERGENCY_DESCEND));
        Serial.print(" CRC_ERR:"); Serial.println(elrsGetCrcErrorCount());
    }
}
