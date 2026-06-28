# CURIO 雙環自調校飛行控制韌體 — 運作模式說明

對應程式：`CURIO_AutoTune_Dual_PID.ino`（搭配 `elrs.h/.cpp`、`autotune.h/.cpp`、`mahony.h/.cpp`、`BMI088.h/.cpp`）
硬體平台：CURIO (RP2350 / RP2040)，BMI088 + ELRS/CRSF

---

## 1. 總覽

本韌體採用 **雙環串接 PID**（外環 Angle Loop + 內環 Rate Loop）作為姿態控制核心，並內建 **繼電器回授法（Relay Feedback）自動調校**，可在飛行中（懸停狀態）自動量測系統動態並重新計算 PID 增益；調校完成後的增益會自動寫入 **Flash**，下次開機自動沿用，無需每次重新調校。

設計上的核心飛行流程是：

```
上電 → 讀取 Flash 歷史增益(若有) → 感測器校正 → 解鎖 → 正常 ELRS 遙控起飛/懸停
                                                            │
                                                            │ (穩定懸停後扳上 AUX1 開關)
                                                            ▼
                                          自動調校 (Rate→Angle，全自動四階段)
                                                            │
                                                            │ (完成後自動套用新增益，並寫入 Flash)
                                                            ▼
                                          回到正常 ELRS 遙控飛行

【任何狀態下，包含調校進行中】
                            扳上 AUX2 開關
                                    ▼
                      緊急自動減速緩降 → 自動上鎖
```

**重要：自動調校只能在已經起飛、穩定懸停的狀態下進行**，韌體會主動檢查搖桿是否置中、油門是否落在合理的懸停區間，任一條件不滿足就拒絕啟動或立即中止，不會在地面解鎖瞬間或大動作飛行時意外觸發。

---

## 2. 程式模組結構

韌體拆分為主程式 + 三個功能模組，各自獨立、職責單一：

| 檔案 | 職責 | 對外暴露的狀態 |
|---|---|---|
| `CURIO_AutoTune_Dual_PID.ino` | 主迴圈：IMU讀取、雙環PID輸出計算 (`computeAxisOutput`)、馬達混控、LED、安全互鎖、解鎖/失聯/傾斜斷電判斷 | `roll_axis`/`pitch_axis`（`AxisState`）、`current_mode`、`RC_US_MID` 等供其他模組 `extern` 參照 |
| `elrs.h` / `elrs.cpp` | ELRS/CRSF 封包解析、16 通道資料維護、連線逾時判斷 | `elrsInit()`、`elrsUpdate()`、`elrsGetChannel()`、`elrsLinkOk()`、`elrsGetCrcErrorCount()` |
| `autotune.h` / `autotune.cpp` | 繼電器回授調校全流程（含安全前提檢查、四階段鏈推進）+ **調校增益的 Flash 讀寫** | `AxisState`/`SystemMode` 等型態定義、`autotuneRelayStep()`、`startFullAutotune()`/`startRateOnlyAutotune()`/`startAngleOnlyAutotune()`、`abortAutotune()`、`autotuneUpdate()`、`autotuneLoadGains()`、`autotuneSaveGains()` |
| `mahony.h` / `mahony.cpp` | Mahony AHRS 姿態估計（四元數內部維護） | `mahonyUpdate()`、`mahonyGetRoll()`、`mahonyGetPitch()` |

雙環 PID 的核心數學（`computeAxisOutput`）刻意**留在主程式**而非調校模組：它是每一種模式都會跑的飛控核心邏輯，只是在 `AXIS_TUNE_RATE`/`AXIS_TUNE_ANGLE` 模式下會呼叫 `autotune.cpp` 提供的 `autotuneRelayStep()` 取代對應迴路的正常輸出。

---

## 3. 頻道對應 (RC Channel Mapping)

沿用 `CURIO_ELRS_Test.ino` 已驗證之 TX16S 實際輸出順序（定義於 `elrs.h`）：

| CRSF 頻道 | 索引常數 | 功能 | 備註 |
|---|---|---|---|
| CH1 | `CH_ROLL` (0) | Roll（橫滾） | 1000~2000us，中點1500 |
| CH2 | `CH_PITCH` (1) | Pitch（俯仰） | 同上 |
| CH3 | `CH_THROTTLE` (2) | Throttle（油門） | 1000=最低，2000=最高 |
| CH4 | `CH_YAW` (3) | Yaw（偏航） | 同上 |
| CH5 | `CH_ARM` (4) | **解鎖開關** | >1500us = 解鎖位置 |
| CH6 (AUX1) | `CH_AUTOTUNE` (5) | **自動調校觸發開關** | >1500us = 請求調校 |
| CH7 (AUX2) | `CH_EMERGENCY_DESCEND` (6) | **緊急自動減速緩降開關** | >1500us = 觸發緩降 |
| CH8 (AUX3) | — | 未使用，保留 | — |

> ⚠️ 請務必在您的 TX16S 上將一個先前未使用的開關設定到 CH7，並確認與韌體中 `CH_EMERGENCY_DESCEND` 的對應一致。

---

## 4. 各運作模式詳細說明

### 4.1 一般遙控飛行（`MODE_NORMAL`）

- Roll / Pitch 搖桿 → 經 `stickToTarget()` 映射為目標角度（滿桿 ±`MAX_STICK_ANGLE`＝30°）
- Yaw 搖桿 → 映射為目標偏航角速度（滿桿 ±`MAX_STICK_YAWRATE`＝180°/s）
- 油門搖桿 → 經 `throttleToBase()` 映射為混控基礎 PWM，**上限只到 `THROTTLE_MAX_MAPPED`(180)，而非馬達真正上限(216)**，刻意保留約17%的餘裕給雙環PID姿態修正，避免滿油門時混控直接撞頂、喪失姿態控制權威
- Roll/Pitch 各自跑「外環 Angle PID → 內環 Rate PID」的完整雙環串接（`computeAxisOutput()`）

### 4.2 自動調校（四階段，由內而外）

由 `AUX1` 開關（或地面測試時的 Serial 指令 `t`/`r`/`a`）觸發，依序自動執行：

1. **Rate Loop – Roll**：以繼電器（Relay）直接取代 Roll 內環 PID，輸出 ±35 PWM 的方波，觀察 Roll 角速度（陀螺儀）的極限振盪
2. **Rate Loop – Pitch**：同上，換成 Pitch 軸
3. **Angle Loop – Roll**：套用上一步測出的內環增益後，改用繼電器取代 Roll **外環** PID（輸出 ±15°/s 的目標角速度方波），觀察 Roll **角度**的極限振盪
4. **Angle Loop – Pitch**：同上，換成 Pitch 軸

每一階段量得「振盪振幅 a」與「振盪週期 Pu」後，依 `Ku ≈ 4d/(π·a)` 算出極限增益，再用 **Ziegler-Nichols「No Overshoot」規則**（飛行版預設，最保守）換算成 Kp/Ki/Kd，**完成後立即套用**並自動進入下一階段。

**為什麼要由內而外、分四階段，而不是一次調完？**
雙環系統若直接對最外層做一次繼電器測試，量到的振盪會混雜「內環+外環」兩個迴路的動態，沒有明確物理意義。必須先用已知的繼電器測出內環增益、讓內環確實閉環運作後，才能把外環當作「單一未知迴路」去測，這樣量出來的 Ku、Pu 才有效。

**啟動 / 維持自動調校的安全前提**（任一不滿足，立即拒絕啟動或中止）：

| 條件 | 預設門檻 |
|---|---|
| Roll/Pitch 搖桿必須在中點附近 | 偏移 < 40us |
| 油門必須落在「懸停區間」 | 1400us ~ 1750us（⚠️ 請依實際懸停油門校正）|
| AUX1 開關必須持續扳在高位 | — |
| 必須已解鎖且 ELRS 連線正常 | — |

**開關採邊緣觸發（edge-trigger）設計**：完成一次調校後，即使 AUX1 仍維持高位，也不會立刻重新觸發，必須先扳回低位、再扳上才能再次啟動，避免劇烈震盪沒完沒了。

**何時會寫入 Flash？** 見下方第 5 節，並非每種調校鏈結束都會觸發寫入。

### 4.3 緊急自動減速緩降（`MODE_EMERGENCY_DESCEND`）

由 `AUX2` 開關觸發，**優先權最高，可在任何狀態下觸發**（包含自動調校進行中，會立即中止調校）：

- 忽略 Roll/Pitch/Yaw 搖桿輸入，強制目標角度＝0、目標偏航角速度＝0（強制水平、不旋轉）
- 雙環姿態控制持續運作（用目前的 PID 增益自主穩定機身）
- 油門基礎值從**觸發瞬間的實際油門**開始，以 `EMERGENCY_DESCENT_RATE_PWM_PER_S`（預設 35 PWM/秒）線性下降
- 油門降至接近最低（`MOTOR_IDLE_PWM+5`）並維持 1 秒後，視為已落地，**自動上鎖**
- 飛行員可在緩降過程中隨時將 AUX2 扳回低位，**立即取消**並恢復正常遙控（適合誤觸發時即時改正）

> 此功能是飛行員主動觸發的「緩降」，與失聯保護（Failsafe）的「立即全斷電」是兩套獨立機制，請見下方說明。

---

## 5. 自動調校增益的 Flash 持久化（新功能）

調校得到的 12 個 PID 增益（Roll/Pitch 各自的 Rate-P/I/D + Angle-P/I/D）不再只存在 RAM 裡，韌體會在開機與調校完成時自動讀寫 **Flash**（透過 arduino-pico 核心內建的 `EEPROM.h`，以保留區塊模擬傳統 EEPROM 存取，並非 ESP32 的 `Preferences.h`，RP2040/RP2354A 平台沒有那個函式庫）。

### 5.1 開機讀取（`autotuneLoadGains()`）

`setup()` 一開始（`Serial.begin()` 之後、感測器初始化之前）就會呼叫，邏輯：

1. 從 Flash 偏移 0 讀出一筆 56-byte 紀錄（12 個 float + magic number + 簡易校驗碼）
2. 若 magic number 不符 → 印出「Flash 中沒有先前調校紀錄，使用程式預設增益」，沿用 `AxisState` 程式內建的初始值
3. 若 magic number 對但校驗碼不符（代表寫入過程被中斷或 Flash 毀損）→ 印出警告，同樣沿用預設值，**不會套用一半毀損的數值**
4. 兩項都通過 → 覆蓋 `roll_axis`/`pitch_axis` 的 12 個增益欄位，並把載入的數值完整印到 Serial

### 5.2 寫入時機（`autotuneSaveGains()`）—⚠️ 並非每種調校鏈都會寫入

寫入動作**只掛在 `advanceAutotuneChain()` 的 `MODE_AUTOTUNE_ANGLE_PITCH`（外環Pitch）完成分支**，原因是這是調校鏈裡唯一保證「Angle 增益已經是最新」的終點：

| 觸發方式 | 調校鏈 | 結束點 | 是否寫入 Flash |
|---|---|---|---|
| AUX1 開關 / Serial `t` | 完整鏈：Rate Roll→Pitch→Angle Roll→Pitch | `MODE_AUTOTUNE_ANGLE_PITCH` | ✅ 會寫入 |
| Serial `r` | 僅 Rate：Roll→Pitch | `MODE_AUTOTUNE_RATE_PITCH`（非完整鏈分支） | ❌ 不會寫入 |
| Serial `a` | 僅 Angle：Roll→Pitch（假設 Rate 增益已可用） | `MODE_AUTOTUNE_ANGLE_PITCH` | ✅ 會寫入 |

**重要含意**：若你先用 `r` 單獨調內環、得到新的 Rate 增益後，這組新增益**只存在 RAM**，重新開機就會遺失；要讓它連同 Angle 增益一起永久保存，必須接著再跑一次 `a`（或直接從頭跑 `t`），讓流程走到 `MODE_AUTOTUNE_ANGLE_PITCH` 才會觸發寫入——寫入的是當下 `roll_axis`/`pitch_axis` 的**完整 12 個增益**（包含這次 `r` 剛調出來、還留在記憶體裡的 Rate 增益），不是只存 Angle 的部分。

寫入流程：组裝 12 個增益 + magic + 校驗碼 → `EEPROM.put()` → `EEPROM.commit()` 真正落 Flash → 成功印出「💾 自動調校增益已寫入 Flash，下次開機將自動套用」，失敗則印出警告並提醒本次結果只存在記憶體中。

### 5.3 已知限制

- 目前**沒有**地面指令可以主動清除 Flash 紀錄、強制恢復程式內建預設值；如需重置，需自行覆寫該段 Flash 或改版加入清除指令
- 調校只在主動觸發時才寫入一次，頻率遠低於 Flash 抹寫壽命等級，不需額外做 wear-leveling

---

## 6. 安全機制總覽

| 機制 | 觸發條件 | 行為 |
|---|---|---|
| **解鎖互鎖** | 解鎖瞬間油門必須 < 1100us | 防止帶油門解鎖；解鎖開關關閉時立即上鎖 |
| **失聯保護 (Failsafe)** | 超過 300ms 未收到合法CRSF封包 | 立即強制上鎖、馬達全部歸零、中止任何調校 |
| **嚴重傾斜安全斷電** | \|roll\| 或 \|pitch\| > 75° | 視為墜機/失控，立即強制上鎖斷電（任何模式下皆適用）|
| **自動調校安全前提** | 搖桿未置中 / 油門不在懸停區間 / 開關放回 | 拒絕啟動或立即中止調校，回到正常遙控 |
| **緊急自動減速緩降** | AUX2 開關 | 任何狀態下強制接管，自主水平緩降後自動上鎖 |

優先順序（由高到低）：**失聯保護 > 嚴重傾斜斷電 > 緊急緩降 > 自動調校 > 一般遙控**。

---

## 7. LED 燈號說明

| LED | 用途 | 狀態說明 |
|---|---|---|
| `LED_A`（GPIO7） | 解鎖狀態 | 常亮＝已解鎖；熄滅＝未解鎖 |
| `LED_B`（GPIO8） | 狀態指示（藍） | 一般飛行時心跳慢閃；調校剛完成後常亮3秒；緊急緩降時與LED_C交替快閃 |
| `LED_C`（GPIO9） | 狀態指示（紅） | 失聯時常亮；自動調校中快閃；緊急緩降時與LED_B交替快閃 |

判斷優先序：**失聯（LED_C常亮） > 緊急緩降（交替快閃） > 調校中（LED_C快閃） > 調校剛完成（LED_B常亮3秒） > 一般心跳**。

> 注意：「調校剛完成常亮3秒」只代表**該次調校鏈跑完**，不代表一定已寫入 Flash（見第 5.2 節，僅 Rate-only 不會寫入）；目前沒有獨立燈號區分兩者。

---

## 8. 地面測試用 Serial 指令

USB 連接 Serial Monitor（115200 baud）時可用，行為與 AUX1/AUX2 觸發的安全前提相同：

| 指令 | 功能 | 是否會寫入 Flash |
|---|---|---|
| `t` | 完整自動調校（Rate→Angle，Roll→Pitch） | ✅ 完成時會 |
| `r` | 僅內環 Rate Loop 自動調校 | ❌ 不會 |
| `a` | 僅外環 Angle Loop 自動調校（假設內環增益已可用） | ✅ 完成時會 |
| `x` | 立即中止自動調校 | 不適用（中止不算完成）|

同時每 100ms 會印出一行除錯資訊：解鎖狀態、連線狀態、目前模式、roll/pitch、油門、AUX1/AUX2 數值、CRC錯誤計數，方便地面測試判讀。

---

## 9. 關鍵參數一覽（建議依實機調整）

| 參數 | 預設值 | 所在檔案 | 說明 |
|---|---|---|---|
| `MAX_STICK_ANGLE` | 30° | 主程式 | 滿桿對應目標角度 |
| `MAX_STICK_YAWRATE` | 180°/s | 主程式 | 滿桿對應目標偏航角速度 |
| `THROTTLE_MAX_MAPPED` | 180 | 主程式 | 油門對應上限，保留PID修正餘裕 |
| `ARM_THROTTLE_MAX_US` | 1100us | 主程式 | 解鎖互鎖油門門檻 |
| `FAILSAFE_TIMEOUT_MS` | 300ms | 主程式 | 失聯判定時間 |
| `SEVERE_TILT_CUTOFF_DEG` | 75° | 主程式 | 嚴重傾斜斷電門檻 |
| `AUTOTUNE_MIN/MAX_THROTTLE_US` | 1400~1750us | `autotune.h` | ⚠️ **務必依實測懸停油門調整** |
| `RELAY_AMPLITUDE_RATE` | 35 (PWM) | `autotune.h` | 內環調校繼電器振幅 |
| `RELAY_AMPLITUDE_ANGLE` | 15 (°/s) | `autotune.h` | 外環調校繼電器振幅 |
| `EMERGENCY_DESCENT_RATE_PWM_PER_S` | 35 (PWM/s) | 主程式 | 緊急緩降下降速率，數值越小越平緩 |
| `AUTOTUNE_RULE` | `ZN_NO_OVERSHOOT` | `autotune.h` | Ziegler-Nichols 規則，飛行版建議維持最保守 |
| `MAHONY_KP` / `MAHONY_KI` | 2.0 / 0.005 | `mahony.h` | AHRS 互補濾波增益 |
| `AUTOTUNE_EEPROM_ADDR` / `_SIZE` | 0 / 64 bytes | `autotune.cpp` | Flash 增益紀錄存放位置與保留大小 |
| `AUTOTUNE_GAIN_MAGIC` | `0x43555231`("CUR1") | `autotune.cpp` | Flash 紀錄有效性標記碼 |

---

## 10. 建議測試與飛行流程

1. **先在測試架上完成調校**（參考 `level1_balance_dualloop_autotune.ino`），得到一組安全可用的初始增益，寫入本韌體 `AxisState` 的初始值（這組是「出廠預設值」，Flash 內若已有紀錄會優先覆蓋它，僅在 Flash 從未寫入過或校驗失敗時才會生效）
2. 移除螺旋槳，上電確認解鎖/上鎖、搖桿映射、失聯保護（拔掉接收機天線測試）皆正常，並觀察開機時 Serial 印出的 Flash 讀取結果（無紀錄/校驗失敗/已載入三者之一）
3. 裝回螺旋槳，於室內淨空、低高度、貼近地面處試飛，確認自穩飛行手感正常
4. 確認 `AUTOTUNE_MIN/MAX_THROTTLE_US` 範圍涵蓋您機型實際懸停油門
5. 在穩定懸停狀態下，扳上 AUX1 嘗試飛行中自動調校（會跑完整鏈並於完成時寫入 Flash），**全程保持隨時可扳上 AUX2 或關閉解鎖開關的準備**
6. 調校完成後手感應更穩定；重新開機驗證 Serial 確實印出「已從 Flash 載入先前自動調校增益」且數值與上次調校結果一致
7. 若有異常震盪或飄移，先用 Serial 指令在地面以較低振幅參數重新測試；若只想微調內環可單獨用 `r`，但記得最後要再跑一次 `a` 或 `t` 才會把新增益落到 Flash

---

## 11. 限制與注意事項

- 本韌體為**自穩定（Angle Mode）**設計，未實作絕對航向鎖定、定高、或 Acro 模式
- 自動調校為**估計值**，繼電器回授法在自由飛行中的雜訊與機體耦合效應比測試架更複雜，結果僅供參考，務必先以保守規則（`ZN_NO_OVERSHOOT`）驗證安全後才提高侵略性
- 緊急自動減速緩降**不具備地形/障礙物感知能力**，僅單純線性降低油門，請只在淨空場地使用
- 失聯保護採**立即全斷電**而非緩降，這是刻意的保守選擇（避免無連線狀態下做出無法即時修正的自主決策）；如需改為緩降式失聯行為，需額外評估風險後再實作
- Flash 增益紀錄**目前無法從地面指令清除**，且僅內環(`r`)單獨完成時不會寫入（見第 5.2 節），規劃調校流程時請留意
