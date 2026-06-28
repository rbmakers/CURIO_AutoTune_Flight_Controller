/*
    CURIO 自動調校 (繼電器回授 Åström–Hägglund 法) 模組 — 實作檔
    從 CURIO_AutoTune_Dual_PID.ino 拆分而來
    Flash 增益持久化使用 arduino-pico 核心的 EEPROM.h (flash 模擬 EEPROM)
    製作 : 火箭鳥創客倉庫
*/

#include "autotune.h"
#include "mahony.h"
#include <EEPROM.h>

// ================================================================
//  與主程式共享的飛行狀態 (實際定義於主 .ino，此處僅 extern 參照)
// ================================================================
extern AxisState      roll_axis;
extern AxisState      pitch_axis;
extern SystemMode     current_mode;
extern float          gx, gy;                 // 原始角速度 (deg/s)，供 Rate Loop 調校重置用
extern unsigned long  tune_done_flash_until;  // 調校完成後 LED 閃爍提示的到期時間
extern const unsigned long RC_US_MID;         // 搖桿中點 (us)，定義於主 .ino

// ================================================================
//  調校鏈狀態 (內部私有，外部不需也不應直接存取)
// ================================================================
enum ChainMode {
    CHAIN_NONE,
    CHAIN_RATE_ONLY,
    CHAIN_FULL,
    CHAIN_ANGLE_ONLY
};
static ChainMode autotune_chain_mode = CHAIN_NONE;

// ================================================================
//  Flash 增益持久化 — 內部私有資料結構與輔助函式
// ================================================================
#define AUTOTUNE_EEPROM_SIZE  64
#define AUTOTUNE_EEPROM_ADDR  0
#define AUTOTUNE_GAIN_MAGIC   0x43555231UL   // "CUR1"；若日後改動結構記得更新版本碼

struct AutotuneGainRecord {
    uint32_t magic;
    float roll_pid_p_rate,   roll_pid_i_rate,   roll_pid_d_rate;
    float pitch_pid_p_rate,  pitch_pid_i_rate,  pitch_pid_d_rate;
    float roll_pid_p_angle,  roll_pid_i_angle,  roll_pid_d_angle;
    float pitch_pid_p_angle, pitch_pid_i_angle, pitch_pid_d_angle;
    uint32_t checksum;   // 簡易加總校驗，僅用於偵測空白/毀損 Flash，非加密用途
};

static uint32_t autotuneChecksum(const AutotuneGainRecord &rec) {
    const uint8_t *p = (const uint8_t*)&rec;
    size_t len = sizeof(AutotuneGainRecord) - sizeof(rec.checksum);
    uint32_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum = (sum * 31u) + p[i];
    }
    return sum;
}

// ----------------------------------------------------------------
// 私有輔助函式 (繼電器/增益換算)
// ----------------------------------------------------------------
static void resetRelayState(RelayTuneState &rt, float current_value, unsigned long timeout_ms) {
    rt.relay_dir = 1;
    rt.start_time = millis();
    rt.last_switch_time = 0;
    rt.cycle_count = 0;
    rt.valid_cycles = 0;
    rt.cycle_min = current_value;
    rt.cycle_max = current_value;
    rt.sum_period_s = 0.0f;
    rt.sum_amplitude = 0.0f;
    rt.timeout_ms = timeout_ms;
    rt.done = false;
    rt.timed_out = false;
}

static void computeGainsFromRelay(float Ku, float Pu, float &Kp, float &Ki, float &Kd) {
    float Ti = 0, Td = 0;
    switch (AUTOTUNE_RULE) {
        case ZN_CLASSIC_PID:
            Kp = 0.60f * Ku;
            Ti = 0.50f * Pu;  Td = 0.125f * Pu;
            break;
        case ZN_PESSEN_INTEGRAL:
            Kp = 0.70f * Ku;
            Ti = 0.40f * Pu;  Td = 0.150f * Pu;
            break;
        case ZN_SOME_OVERSHOOT:
            Kp = 0.33f * Ku;
            Ti = 0.50f * Pu;  Td = 0.333f * Pu;
            break;
        case ZN_NO_OVERSHOOT:
        default:
            Kp = 0.20f * Ku;
            Ti = 0.50f * Pu;  Td = 0.333f * Pu;
            break;
    }
    Ki = Kp / Ti;
    Kd = Kp * Td;
}

static void printAutotuneResult(const char* loop_name, float Ku, float Pu, float Kp, float Ki, float Kd) {
    Serial.println("==========================================");
    Serial.print("✅ ["); Serial.print(loop_name); Serial.println("] 自動調校完成");
    Serial.print("  Ku="); Serial.print(Ku, 4);
    Serial.print(" Pu="); Serial.print(Pu, 4); Serial.println("s");
    Serial.print("  -> Kp=");
    Serial.print(Kp, 4);
    Serial.print(" Ki="); Serial.print(Ki, 4);
    Serial.print(" Kd="); Serial.println(Kd, 4);
    Serial.println("==========================================");
}

static void finalizeAxisTuning(RelayTuneState &rt, const char* loop_name, float relay_amplitude,
                                float &out_p, float &out_i, float &out_d) {
    float Pu = rt.sum_period_s / rt.valid_cycles;
    float a  = rt.sum_amplitude / rt.valid_cycles;

    if (a < 0.1f) {
        Serial.println("⚠️ 振盪振幅過小，調校結果可能不可靠！");
        a = 0.1f;
    }

    float Ku = (4.0f * relay_amplitude) / (PI * a);
    float Kp, Ki, Kd;
    computeGainsFromRelay(Ku, Pu, Kp, Ki, Kd);
    printAutotuneResult(loop_name, Ku, Pu, Kp, Ki, Kd);

    out_p = Kp;
    out_i = Ki; out_d = Kd;
}

static void advanceAutotuneChain(SystemMode finished_phase) {
    switch (finished_phase) {
        case MODE_AUTOTUNE_RATE_ROLL:
            roll_axis.mode = AXIS_NORMAL;
            resetAxisIntegrators(roll_axis);
            Serial.println("Roll Rate 完成 -> Pitch Rate");
            pitch_axis.mode = AXIS_TUNE_RATE;
            resetRelayState(pitch_axis.rt_rate, gy, AUTOTUNE_TIMEOUT_RATE_MS);
            current_mode = MODE_AUTOTUNE_RATE_PITCH;
            break;
        case MODE_AUTOTUNE_RATE_PITCH:
            pitch_axis.mode = AXIS_NORMAL;
            resetAxisIntegrators(pitch_axis);
            if (autotune_chain_mode == CHAIN_FULL) {
                Serial.println("Rate雙軸完成 -> Angle Roll");
                roll_axis.mode = AXIS_TUNE_ANGLE;
                resetRelayState(roll_axis.rt_angle, mahonyGetRoll(), AUTOTUNE_TIMEOUT_ANGLE_MS);
                current_mode = MODE_AUTOTUNE_ANGLE_ROLL;
            } else {
                Serial.println("Rate Loop 雙軸調校完成");
                current_mode = MODE_NORMAL;
                tune_done_flash_until = millis() + 3000;
            }
            break;
        case MODE_AUTOTUNE_ANGLE_ROLL:
            roll_axis.mode = AXIS_NORMAL;
            resetAxisIntegrators(roll_axis);
            Serial.println("Angle Roll 完成 -> Angle Pitch");
            pitch_axis.mode = AXIS_TUNE_ANGLE;
            resetRelayState(pitch_axis.rt_angle, mahonyGetPitch(), AUTOTUNE_TIMEOUT_ANGLE_MS);
            current_mode = MODE_AUTOTUNE_ANGLE_PITCH;
            break;
        case MODE_AUTOTUNE_ANGLE_PITCH:
            pitch_axis.mode = AXIS_NORMAL;
            resetAxisIntegrators(pitch_axis);
            Serial.println("================ 自動調校結束，已套用 ================");
            Serial.print("Rate  Roll : Kp="); Serial.print(roll_axis.pid_p_rate, 4);
            Serial.print(" Ki="); Serial.print(roll_axis.pid_i_rate, 4);
            Serial.print(" Kd="); Serial.println(roll_axis.pid_d_rate, 4);
            Serial.print("Rate  Pitch: Kp=");
            Serial.print(pitch_axis.pid_p_rate, 4);
            Serial.print(" Ki="); Serial.print(pitch_axis.pid_i_rate, 4);
            Serial.print(" Kd="); Serial.println(pitch_axis.pid_d_rate, 4);
            Serial.print("Angle Roll : Kp="); Serial.print(roll_axis.pid_p_angle, 4);
            Serial.print(" Ki="); Serial.print(roll_axis.pid_i_angle, 4);
            Serial.print(" Kd="); Serial.println(roll_axis.pid_d_angle, 4);
            Serial.print("Angle Pitch: Kp="); Serial.print(pitch_axis.pid_p_angle, 4);
            Serial.print(" Ki="); Serial.print(pitch_axis.pid_i_angle, 4);
            Serial.print(" Kd="); Serial.println(pitch_axis.pid_d_angle, 4);
            autotuneSaveGains();   // 四階段全部完成 -> 寫入 Flash 供下次開機沿用
            current_mode = MODE_NORMAL;
            tune_done_flash_until = millis() + 3000;
            break;
        default:
            current_mode = MODE_NORMAL;
            break;
    }
}

// ----------------------------------------------------------------
// 對外 API 實作 — 調校流程
// ----------------------------------------------------------------
float autotuneRelayStep(RelayTuneState &rt, float measured_value, float target_value, unsigned long now_ms,
                         float relay_amplitude, float hysteresis) {
    float error = target_value - measured_value;
    if (measured_value > rt.cycle_max) rt.cycle_max = measured_value;
    if (measured_value < rt.cycle_min) rt.cycle_min = measured_value;
    if (rt.relay_dir < 0 && error > hysteresis) {
        rt.relay_dir = +1;
        if (rt.last_switch_time != 0) {
            float period_s = (now_ms - rt.last_switch_time) / 1000.0f;
            rt.cycle_count++;
            if (rt.cycle_count > AUTOTUNE_SKIP_CYCLES) {
                float amplitude = (rt.cycle_max - rt.cycle_min) / 2.0f;
                rt.sum_period_s  += period_s;
                rt.sum_amplitude += amplitude;
                rt.valid_cycles++;
            }
        }
        rt.last_switch_time = now_ms;
        rt.cycle_min = measured_value;
        rt.cycle_max = measured_value;
    } else if (rt.relay_dir > 0 && error < -hysteresis) {
        rt.relay_dir = -1;
    }

    if (rt.valid_cycles >= AUTOTUNE_AVG_CYCLES) rt.done = true;
    if (now_ms - rt.start_time > rt.timeout_ms) rt.timed_out = true;
    return rt.relay_dir * relay_amplitude;
}

void resetAxisIntegrators(AxisState &axis) {
    axis.int_rate = 0.0f;  axis.err_prev_rate = 0.0f;
    axis.int_angle = 0.0f; axis.err_prev_angle = 0.0f;
}

bool autotunePreconditionsOk(unsigned long roll_us, unsigned long pitch_us, unsigned long throttle_us) {
    bool sticksCentered = (abs((long)roll_us  - (long)RC_US_MID) < (long)AUTOTUNE_STICK_DEADBAND_US) &&
                          (abs((long)pitch_us - (long)RC_US_MID) < (long)AUTOTUNE_STICK_DEADBAND_US);
    bool throttleOk = (throttle_us > AUTOTUNE_MIN_THROTTLE_US) && (throttle_us < AUTOTUNE_MAX_THROTTLE_US);
    return sticksCentered && throttleOk;
}

void startFullAutotune() {
    Serial.println("🚀 開始飛行中自動調校：Rate(Roll->Pitch) -> Angle(Roll->Pitch)");
    autotune_chain_mode = CHAIN_FULL;
    roll_axis.mode = AXIS_TUNE_RATE;
    resetRelayState(roll_axis.rt_rate, gx, AUTOTUNE_TIMEOUT_RATE_MS);
    current_mode = MODE_AUTOTUNE_RATE_ROLL;
}

void startRateOnlyAutotune() {
    Serial.println("🚀 開始內環 Rate Loop 自動調校 (Roll -> Pitch)");
    autotune_chain_mode = CHAIN_RATE_ONLY;
    roll_axis.mode = AXIS_TUNE_RATE;
    resetRelayState(roll_axis.rt_rate, gx, AUTOTUNE_TIMEOUT_RATE_MS);
    current_mode = MODE_AUTOTUNE_RATE_ROLL;
}

void startAngleOnlyAutotune() {
    Serial.println("🚀 開始外環 Angle Loop 自動調校 (Roll -> Pitch)，假設內環增益已可用");
    autotune_chain_mode = CHAIN_ANGLE_ONLY;
    roll_axis.mode = AXIS_TUNE_ANGLE;
    resetRelayState(roll_axis.rt_angle, mahonyGetRoll(), AUTOTUNE_TIMEOUT_ANGLE_MS);
    current_mode = MODE_AUTOTUNE_ANGLE_ROLL;
}

void abortAutotune(const char* reason) {
    Serial.print("⛔ 自動調校中止：");
    Serial.println(reason);
    current_mode = MODE_NORMAL;
    roll_axis.mode = AXIS_NORMAL;
    pitch_axis.mode = AXIS_NORMAL;
    resetAxisIntegrators(roll_axis);
    resetAxisIntegrators(pitch_axis);
}

void autotuneUpdate() {
    switch (current_mode) {
        case MODE_AUTOTUNE_RATE_ROLL:
            if (roll_axis.rt_rate.done) {
                finalizeAxisTuning(roll_axis.rt_rate, "Roll-RATE", RELAY_AMPLITUDE_RATE,
                                 roll_axis.pid_p_rate, roll_axis.pid_i_rate, roll_axis.pid_d_rate);
                advanceAutotuneChain(MODE_AUTOTUNE_RATE_ROLL);
            } else if (roll_axis.rt_rate.timed_out) {
                abortAutotune("Roll Rate Loop 逾時");
            }
            break;
        case MODE_AUTOTUNE_RATE_PITCH:
            if (pitch_axis.rt_rate.done) {
                finalizeAxisTuning(pitch_axis.rt_rate, "Pitch-RATE", RELAY_AMPLITUDE_RATE,
                                 pitch_axis.pid_p_rate, pitch_axis.pid_i_rate, pitch_axis.pid_d_rate);
                advanceAutotuneChain(MODE_AUTOTUNE_RATE_PITCH);
            } else if (pitch_axis.rt_rate.timed_out) {
                abortAutotune("Pitch Rate Loop 逾時");
            }
            break;
        case MODE_AUTOTUNE_ANGLE_ROLL:
            if (roll_axis.rt_angle.done) {
                finalizeAxisTuning(roll_axis.rt_angle, "Roll-ANGLE", RELAY_AMPLITUDE_ANGLE,
                                 roll_axis.pid_p_angle, roll_axis.pid_i_angle, roll_axis.pid_d_angle);
                advanceAutotuneChain(MODE_AUTOTUNE_ANGLE_ROLL);
            } else if (roll_axis.rt_angle.timed_out) {
                abortAutotune("Roll Angle Loop 逾時");
            }
            break;
        case MODE_AUTOTUNE_ANGLE_PITCH:
            if (pitch_axis.rt_angle.done) {
                finalizeAxisTuning(pitch_axis.rt_angle, "Pitch-ANGLE", RELAY_AMPLITUDE_ANGLE,
                                 pitch_axis.pid_p_angle, pitch_axis.pid_i_angle, pitch_axis.pid_d_angle);
                advanceAutotuneChain(MODE_AUTOTUNE_ANGLE_PITCH);
            } else if (pitch_axis.rt_angle.timed_out) {
                abortAutotune("Pitch Angle Loop 逾時");
            }
            break;
        default:
            break;
    }
}

const char* autotuneModeName(SystemMode m) {
    switch (m) {
        case MODE_NORMAL: return "NORMAL";
        case MODE_AUTOTUNE_RATE_ROLL: return "TUNE-RATE-ROLL";
        case MODE_AUTOTUNE_RATE_PITCH: return "TUNE-RATE-PITCH";
        case MODE_AUTOTUNE_ANGLE_ROLL: return "TUNE-ANGLE-ROLL";
        case MODE_AUTOTUNE_ANGLE_PITCH: return "TUNE-ANGLE-PITCH";
        case MODE_EMERGENCY_DESCEND: return "EMERGENCY-DESCEND";
        default: return "?";
    }
}

// ----------------------------------------------------------------
// 對外 API 實作 — Flash 增益持久化
// ----------------------------------------------------------------
void autotuneLoadGains() {
    EEPROM.begin(AUTOTUNE_EEPROM_SIZE);

    AutotuneGainRecord rec;
    EEPROM.get(AUTOTUNE_EEPROM_ADDR, rec);

    if (rec.magic != AUTOTUNE_GAIN_MAGIC) {
        Serial.println("ℹ️ Flash 中沒有先前調校紀錄，使用程式預設增益");
        return;
    }
    if (rec.checksum != autotuneChecksum(rec)) {
        Serial.println("⚠️ Flash 調校紀錄校驗失敗 (可能毀損)，使用程式預設增益");
        return;
    }

    roll_axis.pid_p_rate   = rec.roll_pid_p_rate;
    roll_axis.pid_i_rate   = rec.roll_pid_i_rate;
    roll_axis.pid_d_rate   = rec.roll_pid_d_rate;
    pitch_axis.pid_p_rate  = rec.pitch_pid_p_rate;
    pitch_axis.pid_i_rate  = rec.pitch_pid_i_rate;
    pitch_axis.pid_d_rate  = rec.pitch_pid_d_rate;
    roll_axis.pid_p_angle  = rec.roll_pid_p_angle;
    roll_axis.pid_i_angle  = rec.roll_pid_i_angle;
    roll_axis.pid_d_angle  = rec.roll_pid_d_angle;
    pitch_axis.pid_p_angle = rec.pitch_pid_p_angle;
    pitch_axis.pid_i_angle = rec.pitch_pid_i_angle;
    pitch_axis.pid_d_angle = rec.pitch_pid_d_angle;

    Serial.println("✅ 已從 Flash 載入先前自動調校增益：");
    Serial.print("  Rate  Roll : Kp="); Serial.print(roll_axis.pid_p_rate, 4);
    Serial.print(" Ki="); Serial.print(roll_axis.pid_i_rate, 4);
    Serial.print(" Kd="); Serial.println(roll_axis.pid_d_rate, 4);
    Serial.print("  Rate  Pitch: Kp="); Serial.print(pitch_axis.pid_p_rate, 4);
    Serial.print(" Ki="); Serial.print(pitch_axis.pid_i_rate, 4);
    Serial.print(" Kd="); Serial.println(pitch_axis.pid_d_rate, 4);
    Serial.print("  Angle Roll : Kp="); Serial.print(roll_axis.pid_p_angle, 4);
    Serial.print(" Ki="); Serial.print(roll_axis.pid_i_angle, 4);
    Serial.print(" Kd="); Serial.println(roll_axis.pid_d_angle, 4);
    Serial.print("  Angle Pitch: Kp="); Serial.print(pitch_axis.pid_p_angle, 4);
    Serial.print(" Ki="); Serial.print(pitch_axis.pid_i_angle, 4);
    Serial.print(" Kd="); Serial.println(pitch_axis.pid_d_angle, 4);
}

void autotuneSaveGains() {
    AutotuneGainRecord rec;
    rec.magic = AUTOTUNE_GAIN_MAGIC;
    rec.roll_pid_p_rate   = roll_axis.pid_p_rate;
    rec.roll_pid_i_rate   = roll_axis.pid_i_rate;
    rec.roll_pid_d_rate   = roll_axis.pid_d_rate;
    rec.pitch_pid_p_rate  = pitch_axis.pid_p_rate;
    rec.pitch_pid_i_rate  = pitch_axis.pid_i_rate;
    rec.pitch_pid_d_rate  = pitch_axis.pid_d_rate;
    rec.roll_pid_p_angle  = roll_axis.pid_p_angle;
    rec.roll_pid_i_angle  = roll_axis.pid_i_angle;
    rec.roll_pid_d_angle  = roll_axis.pid_d_angle;
    rec.pitch_pid_p_angle = pitch_axis.pid_p_angle;
    rec.pitch_pid_i_angle = pitch_axis.pid_i_angle;
    rec.pitch_pid_d_angle = pitch_axis.pid_d_angle;
    rec.checksum = autotuneChecksum(rec);

    EEPROM.put(AUTOTUNE_EEPROM_ADDR, rec);
    if (EEPROM.commit()) {
        Serial.println("💾 自動調校增益已寫入 Flash，下次開機將自動套用");
    } else {
        Serial.println("⚠️ 寫入 Flash 失敗，本次調校結果僅保留在記憶體中 (重新開機將遺失)");
    }
}
