#ifndef __CURIO_AUTOTUNE_H__
#define __CURIO_AUTOTUNE_H__

#include <Arduino.h>

// ================================================================
//  CURIO 自動調校 (繼電器回授 Åström–Hägglund 法) 模組
//  從 CURIO_AutoTune_Dual_PID.ino 拆分而來
//  本模組亦負責將調校完成後的 12 個 PID 增益寫入/讀出 Flash
//  (透過 arduino-pico 核心的 EEPROM.h，於 autotune.cpp 中實作)
//  製作 : 火箭鳥創客倉庫
// ================================================================

// ----------------------------------------------------------------
// 型態定義
// ----------------------------------------------------------------

// 單一軸目前所處的控制模式 (供 AxisState.mode 使用)
enum AxisMode {
    AXIS_NORMAL,      // 正常雙環 PID
    AXIS_TUNE_RATE,   // 內環 Rate Loop 繼電器調校中
    AXIS_TUNE_ANGLE,  // 外環 Angle Loop 繼電器調校中
};

// 單次繼電器回授量測的統計狀態 (供 AxisState.rt_rate / rt_angle 使用)
struct RelayTuneState {
    int8_t relay_dir         = 1;
    unsigned long start_time       = 0;
    unsigned long last_switch_time = 0;
    int    cycle_count       = 0;
    int    valid_cycles      = 0;
    float  cycle_min         = 0.0f;
    float  cycle_max         = 0.0f;
    float  sum_period_s      = 0.0f;
    float  sum_amplitude     = 0.0f;
    unsigned long timeout_ms = 10000;
    bool   done              = false;
    bool   timed_out         = false;
};

// 單一軸 (Roll 或 Pitch) 的雙環 PID 增益、誤差狀態與調校狀態
struct AxisState {
    float target_angle = 0.0f;
    float pid_p_angle = 4.0f, pid_i_angle = 0.0f, pid_d_angle = 0.0f;
    float err_prev_angle = 0.0f, int_angle = 0.0f;

    float target_rate = 0.0f;
    float pid_p_rate = 0.8f, pid_i_rate = 0.02f, pid_d_rate = 0.04f;
    float err_prev_rate = 0.0f, int_rate = 0.0f;
    AxisMode mode = AXIS_NORMAL;
    RelayTuneState rt_rate;
    RelayTuneState rt_angle;
};

// Ziegler-Nichols 增益換算規則
enum TuningRule {
    ZN_CLASSIC_PID,
    ZN_PESSEN_INTEGRAL,
    ZN_SOME_OVERSHOOT,
    ZN_NO_OVERSHOOT
};
const TuningRule AUTOTUNE_RULE = ZN_NO_OVERSHOOT;  // 飛行版建議維持最保守

// 飛控系統整體運作模式 (一般飛行 / 四階段調校 / 緊急緩降)
enum SystemMode {
    MODE_NORMAL = 0,
    MODE_AUTOTUNE_RATE_ROLL,
    MODE_AUTOTUNE_RATE_PITCH,
    MODE_AUTOTUNE_ANGLE_ROLL,
    MODE_AUTOTUNE_ANGLE_PITCH,
    MODE_EMERGENCY_DESCEND,
};

// ----------------------------------------------------------------
// 調校參數 (建議依實機調整)
// ----------------------------------------------------------------
const int   AUTOTUNE_SKIP_CYCLES = 2;
const int   AUTOTUNE_AVG_CYCLES  = 6;
const unsigned long AUTOTUNE_TIMEOUT_RATE_MS  = 10000;
const unsigned long AUTOTUNE_TIMEOUT_ANGLE_MS = 25000;

const float RELAY_AMPLITUDE_RATE   = 35.0f;
const float RELAY_HYSTERESIS_RATE  = 3.0f;

const float RELAY_AMPLITUDE_ANGLE  = 15.0f;
const float RELAY_HYSTERESIS_ANGLE = 0.6f;

const unsigned long AUTOTUNE_SWITCH_THRESHOLD_US = 1500;
const unsigned long AUTOTUNE_STICK_DEADBAND_US   = 40;
const unsigned long AUTOTUNE_MIN_THROTTLE_US     = 1400;
const unsigned long AUTOTUNE_MAX_THROTTLE_US     = 1750;

// ----------------------------------------------------------------
// 對外 API — 調校流程
// ----------------------------------------------------------------

// 繼電器回授核心步驟：依量測值與目標值翻轉方波，並在 rt 中累積振盪統計。
// 供主程式 computeAxisOutput() 在 AXIS_TUNE_RATE / AXIS_TUNE_ANGLE 模式下呼叫。
float autotuneRelayStep(RelayTuneState &rt, float measured_value, float target_value,
                         unsigned long now_ms, float relay_amplitude, float hysteresis);

// 重置單一軸的 PID 積分/微分誤差狀態 (解鎖時呼叫)
void resetAxisIntegrators(AxisState &axis);

// 啟動 / 維持調校的安全前提檢查：搖桿是否置中、油門是否在懸停區間
bool autotunePreconditionsOk(unsigned long roll_us, unsigned long pitch_us, unsigned long throttle_us);

// 啟動完整調校鏈 (Rate Roll->Pitch->Angle Roll->Pitch)
void startFullAutotune();
// 僅啟動內環 Rate Loop 調校鏈 (Roll->Pitch)
void startRateOnlyAutotune();
// 僅啟動外環 Angle Loop 調校鏈 (假設內環增益已可用)
void startAngleOnlyAutotune();
// 立即中止目前調校並回到 MODE_NORMAL，reason 會印到 Serial
void abortAutotune(const char* reason);

// 每個 PID tick 呼叫一次：檢查目前調校階段是否完成或逾時，
// 完成則套用新增益並自動進入下一階段 (全部完成時會自動呼叫 autotuneSaveGains() 寫入 Flash)，
// 逾時則中止。若目前不在任何調校模式中，呼叫此函式不會有任何效果。
void autotuneUpdate();

// 取得 SystemMode 的可讀字串 (除錯列印用)
const char* autotuneModeName(SystemMode m);

// ----------------------------------------------------------------
// 對外 API — Flash 增益持久化 (arduino-pico EEPROM.h，於 autotune.cpp 實作)
// ----------------------------------------------------------------

// 從 Flash 讀取先前調校紀錄並覆蓋 roll_axis/pitch_axis 的 12 個 PID 增益初始值；
// 若 Flash 中沒有有效紀錄 (從未調校過 / 校驗失敗)，則保留程式內建的預設增益。
// 請在 setup() 中、開始飛行控制迴路之前呼叫一次。
void autotuneLoadGains();

// 將目前 roll_axis/pitch_axis 的 12 個 PID 增益寫入 Flash。
// 會在 autotuneUpdate() 偵測到四階段調校全部完成時自動呼叫一次，
// 一般不需要手動呼叫，但保留為公開函式以備未來擴充 (例如地面 Serial 指令 's' 手動存檔)。
void autotuneSaveGains();

#endif
