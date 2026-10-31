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
#define PIN_FEED_HOME A0      // A0 (PA0)  推球原點開關（INPUT_PULLUP，LOW = 在原點）
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

// ===== 機構（⚠️ 皆為估計值，待硬體組確認）=====
#ifndef PITCH_STEPS_PER_DEG
#define PITCH_STEPS_PER_DEG 26.67f   // 200 步 × 16 微步 × 鏈輪 3:1 / 360°
#endif
#ifndef YAW_STEPS_PER_DEG
#define YAW_STEPS_PER_DEG 44.44f     // 200 步 × 16 微步 × 行星 5:1 / 360°
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
#define VBAT_ENABLED 0               // 1 = 量飛輪電池電壓 → 前饋電壓補償（3S 鋰電滿→空差 ~17%，6m 必要）
#endif
#ifndef VBAT_DIVIDER
#define VBAT_DIVIDER 5.7f            // 分壓比 (R1+R2)/R2（例：47k/10k → 3S 滿電 12.6V 變 2.21V，低於 ADC 參考電壓）
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
