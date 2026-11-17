// 韌體設定（腳位 / 機構 / 時序）——硬體確定後只改這裡。
// 每一項各自用 #ifndef 包起來：可用 -D 單獨覆寫（電腦模擬器 firmware/host、arduino-cli 編譯參數）。
// 目標板：NUCLEO-G474RE（STM32G474RE，STM32duino 核心）。用板上的 Arduino Uno 排針 D0–D15 / A0–A5，
// 括號內是對應的 STM32 腳位（UM2505 表 15）。I/O 是 3.3 V。同一份程式也能編 Arduino Uno。
#pragma once

// ===== 腳位（依實際接線修改）=====
#ifndef PIN_YAW_STEP
#define PIN_YAW_STEP 4        // D4 (PB5)  TMC2209 STEP（水平/迴轉）
#endif
#ifndef PIN_YAW_DIR
#define PIN_YAW_DIR 7         // D7 (PA8)
#endif
#ifndef PIN_PITCH_STEP
#define PIN_PITCH_STEP 8      // D8 (PA9)  TMC2209 STEP（俯仰）
#endif
#ifndef PIN_PITCH_DIR
#define PIN_PITCH_DIR 12      // D12 (PA6)
#endif
#ifndef PIN_STEPPER_EN
#define PIN_STEPPER_EN A3     // A3 (PB0)  TMC2209 EN，兩軸共用，LOW = 致能
#endif
#ifndef PIN_FLY_L
#define PIN_FLY_L 5           // D5 (PB4, TIM3_CH1)  飛輪左 PWM（驅動板 IN1；IN2 接 GND）
#endif
#ifndef PIN_FLY_R
#define PIN_FLY_R 6           // D6 (PB10, TIM2_CH3) 飛輪右 PWM
#endif
#ifndef PIN_FEED
#define PIN_FEED 9            // D9 (PC7, TIM3_CH2)  推球（曲柄滑塊）馬達 PWM
#endif
#ifndef PIN_FEED_HOME
#if defined(ARDUINO_ARCH_STM32)
#define PIN_FEED_HOME PC10    // CN7-1 (PC10) 推球原點開關（INPUT_PULLUP，LOW = 在原點）；A0 給車輪 M3 編碼器
#else
#define PIN_FEED_HOME A0      // Uno：A0
#endif
#endif
#ifndef PIN_ESTOP_SENSE
#define PIN_ESTOP_SENSE A1    // A1 (PA1)  急停輔助接點（INPUT_PULLUP，HIGH = 已按下 / 斷線）
#endif
#ifndef PIN_VBAT
#define PIN_VBAT A2           // A2 (PA4)  飛輪電池電壓（分壓後接入，選用；不可超過 VBAT_ADC_VREF）
#endif
#ifndef PIN_TACH_L
#define PIN_TACH_L 2          // D2 (PA10) 飛輪轉速計（外部中斷，選用）
#endif
#ifndef PIN_TACH_R
#define PIN_TACH_R 3          // D3 (PB3)  飛輪轉速計（外部中斷，選用）
#endif
#ifndef PIN_LED
#define PIN_LED 13            // D13 (PA5) = 板上綠色 LED LD2
#endif

// ===== 車輪：BTS7960 × 4 + 霍爾編碼器（只有 Nucleo；Uno 計時器不夠）=====
// M1 左前、M2 左後、M3 右前、M4 右後（以車頭方向 = 底盤綠色箭頭為準）。PWM / 編碼器腳寫成 STM32duino 的 PinName，_ALTn 選定計時器：
//   編碼器用 TIM1/TIM4/TIM5/TIM20 的 encoder mode；TIM2/TIM3 留給 D5/D6/D9（飛輪、推球 PWM）。
//   PWM：M1 TIM8+TIM17、M2 TIM2+TIM16、M3 TIM15、M4 TIM8，全部 WHEEL_PWM_HZ（D6 與 M2 RPWM 共用 TIM2）。
//   11/06 實機：接線完成後車頭定在箭頭方向，四組腳位的順序由原本的 {左前, 左後, 右前, 右後} 改成 {原 M2, 原 M4, 原 M1, 原 M3}。
#ifndef WHEELS_ENABLED
#if defined(ARDUINO_ARCH_STM32)
#define WHEELS_ENABLED 1
#else
#define WHEELS_ENABLED 0
#endif
#endif
#ifndef PIN_WHEEL_EN
#define PIN_WHEEL_EN PB12      // CN10-16：四片 BTS7960 的 R_EN + L_EN（外接 10 kΩ 下拉；HIGH = 致能）
#endif
#ifndef WHEEL_RPWM_PINS
#define WHEEL_RPWM_PINS {PC_6_ALT1, PB_11, PB_14_ALT1, PC_8_ALT1}   // TIM8_CH1, TIM2_CH4, TIM15_CH1, TIM8_CH3
#endif
#ifndef WHEEL_LPWM_PINS
#define WHEEL_LPWM_PINS {PA_7_ALT3, PA_12_ALT2, PB_15_ALT2, PC_9_ALT1}  // TIM17_CH1, TIM16_CH1, TIM15_CH2, TIM8_CH4
#endif
#ifndef WHEEL_ENC_A_PINS
#define WHEEL_ENC_A_PINS {PB_6, PB_2_ALT1, PC_0, PA_0_ALT1}         // TIM4_CH1, TIM20_CH1, TIM1_CH1, TIM5_CH1
#endif
#ifndef WHEEL_ENC_B_PINS
#define WHEEL_ENC_B_PINS {PB_7_ALT1, PC_2_ALT1, PC_1, PC_12}        // TIM4_CH2, TIM20_CH2, TIM1_CH2, TIM5_CH2
#endif
#ifndef WHEEL_PWM_HZ
#define WHEEL_PWM_HZ 20000             // BTS7960 最高 25 kHz
#endif
#ifndef WHEEL_INVERT
#define WHEEL_INVERT 0x0A              // bit i = 馬達 i 反轉（指令與編碼器一起反）。11/06 隨腳位順序調整：M2、M4
#endif
#ifndef WHEEL_ENC_INVERT
#define WHEEL_ENC_INVERT 0x09          // bit i = 只反編碼器計數（A/B 接反時用）。11/06 實測：M1、M4 往前轉時計數為負
#endif
#ifndef WHEEL_ENC_PPR
#define WHEEL_ENC_PPR 11               // 編碼器每轉脈衝（馬達軸、單相）
#endif
#ifndef WHEEL_GEAR
#define WHEEL_GEAR 56                  // 減速比 1:56 → 輸出軸每轉 11 × 4 × 56 = 2464 計數
#endif
#ifndef WHEEL_MAX_PERMILLE
#define WHEEL_MAX_PERMILLE 1000        // DRIVE ±1000 縮放成 ±這個值。11/17 車輪電源改成 12 V（= 馬達額定）→ 1000；改回直接接 4S（滿電 16.8 V）時用 714 = 12 ÷ 16.8 × 1000
#endif
#ifndef WHEEL_SLEW_PER_S
#define WHEEL_SLEW_PER_S 2000          // 輸出變化率上限（千分比/秒）：0 → 全速 0.5 s，避免電流突波讓電壓掉
#endif
#ifndef DRIVE_TIMEOUT_MS
#define DRIVE_TIMEOUT_MS 500           // 這麼久沒收到 DRIVE → 車輪停（Orin 端每 0.2 s 重送）
#endif
#ifndef WHEEL_RPM_MS
#define WHEEL_RPM_MS 100               // 編碼器轉速計算週期
#endif

// ===== 機構（⚠️ 皆為估計值，待硬體組確認）=====
#ifndef PITCH_STEPS_PER_DEG
#define PITCH_STEPS_PER_DEG 26.67f   // 200 步 × 16 微步 × 皮帶輪 3:1 / 360°；馬達端 20 齒、俯仰軸 60 齒
#endif
#ifndef YAW_STEPS_PER_DEG
#define YAW_STEPS_PER_DEG 48.254f    // 200 步 × 16 微步 × 行星齒輪 38/7 / 360°；太陽 21 齒接馬達、外環 93 齒固定、行星支架輸出 → 減速比 1 + 93/21
#endif
#ifndef PITCH_MIN_CDEG
#define PITCH_MIN_CDEG (-500L)       // 0.01°
#endif
#ifndef PITCH_MAX_CDEG
#define PITCH_MAX_CDEG 6000L
#endif
#ifndef YAW_MIN_CDEG
#define YAW_MIN_CDEG (-9000L)
#endif
#ifndef YAW_MAX_CDEG
#define YAW_MAX_CDEG 9000L
#endif
#ifndef PITCH_MAX_DPS
#define PITCH_MAX_DPS 60.0f          // 位置模式最高角速度 (deg/s)
#endif
#ifndef YAW_MAX_DPS
#define YAW_MAX_DPS 90.0f
#endif
#ifndef PITCH_ACCEL_DPS2
#define PITCH_ACCEL_DPS2 240.0f      // 角加速度 (deg/s^2)
#endif
#ifndef YAW_ACCEL_DPS2
#define YAW_ACCEL_DPS2 360.0f
#endif
#ifndef JOG_MAX_DPS
#define JOG_MAX_DPS 60.0f            // 手動速度上限
#endif
#ifndef STEP_PULSE_US
#define STEP_PULSE_US 5              // STEP 脈寬：TMC2209 要 >= 0.1 us；XH5776C 說明書要 >= 2.5 us
#endif
#ifndef DIR_SETUP_US
#define DIR_SETUP_US 10              // DIR 改變後到下一個 STEP：XH5776C 說明書要 > 5 us；ULN2003 關斷比導通慢約 1 us，留餘裕
#endif

// ===== 飛輪 =====
#ifndef FLY_MAX_RPM
#define FLY_MAX_RPM 3000             // FLY_VNOM 電壓、PWM 255 時的轉速（開迴路換算；待量測）
#endif
#ifndef FLY_VNOM
#define FLY_VNOM 12.0f               // FLY_MAX_RPM 是在這個電壓下量的
#endif
#ifndef FLY_HAS_TACH
#define FLY_HAS_TACH 0               // 1 = 有轉速計 → PI 閉迴路穩速（6m 時 v0 穩定度關鍵）
#endif
#ifndef FLY_TACH_PPR
#define FLY_TACH_PPR 1               // 轉速計每轉脈衝數
#endif
#ifndef VBAT_ENABLED
#define VBAT_ENABLED 0               // 1 = 量飛輪電池電壓 → 前饋電壓補償（4S LiPo 滿電 16.8 V、用到 14.0 V 時差 17%，6m 必要）
#endif
#ifndef VBAT_DIVIDER
#define VBAT_DIVIDER 5.7f            // 分壓比 (R1+R2)/R2（例：47k/10k：4S 滿電 16.8 V 變 2.95 V，要低於 VBAT_ADC_VREF）
#endif
#ifndef VBAT_ADC_VREF
#if defined(ARDUINO_ARCH_AVR)
#define VBAT_ADC_VREF 5.0f           // Uno：ADC 參考 = 5 V
#else
#define VBAT_ADC_VREF 3.3f           // NUCLEO-G474RE：ADC 參考 = VREF+。JP8 在 [1-2]（出廠）時是板上 VREF（2.5 或 3.25 V，看 SB14），
                                     // 在 [2-3] 時是 3.3 V。用三用電表量 CN5 第 8 腳（VREFP）後填實際值
#endif
#endif
#ifndef FLY_TOL_PCT
#define FLY_TOL_PCT 2                // 到速判定 ±%（6m 時轉速差 1% ≈ 加分排落點差 2.6cm，別放太寬）
#endif
#ifndef FLY_STABLE_MS
#define FLY_STABLE_MS 200            // 有轉速計時：需連續在容許內的時間
#endif
#ifndef FLY_SPINUP_MS
#define FLY_SPINUP_MS 1500           // 無轉速計時：改目標後等這麼久才算到速
#endif

// ===== 推球 =====
#ifndef FEED_HAS_HOME_SWITCH
#define FEED_HAS_HOME_SWITCH 0       // 1 = 有原點開關（轉一圈回原點即停）；0 = 固定時間
#endif
#ifndef FEED_MS
#define FEED_MS 400                  // 無開關：推球馬達運轉時間
#endif
#ifndef FEED_MIN_MS
#define FEED_MIN_MS 120              // 有開關：至少轉這麼久才認回原點（防抖）
#endif
#ifndef FEED_TIMEOUT_MS
#define FEED_TIMEOUT_MS 1500         // 有開關：逾時視為卡彈 → EV FAULT
#endif
#ifndef FEED_PWM
#define FEED_PWM 200
#endif

// ===== 時序 / 安全 =====
#ifndef SETTLE_MS
#define SETTLE_MS 150                // 轉塔停止後等待穩定才算到位
#endif
#ifndef WDOG_MS
#define WDOG_MS 1000                 // 超過此時間沒收到合法指令 → 飛輪停、JOG 停（Orin 端每 100ms 輪詢 STAT）
#endif
#ifndef ALLOW_NO_CRC
#define ALLOW_NO_CRC 1               // 1 = 接受無 CRC 的手打指令（序列埠監控視窗除錯用）
#endif
#ifndef FW_VERSION
#define FW_VERSION 1
#endif
