// Arduino API 硬體實作（NUCLEO-G474RE 用 STM32duino；也可編 Uno）：TMC2209 STEP/DIR、PWM 馬達、轉速計、急停感測、USB 序列。
#pragma once
#ifdef ARDUINO
#include <Arduino.h>

#include "config.h"
#include "hal.h"

#ifndef ESTOP_SENSE_ENABLED
#define ESTOP_SENSE_ENABLED 1   // 1 = 讀 A1 的急停輔助接點（11/19 使用者接好、改 1）
#endif
#ifndef ESTOP_SENSE_NO
#define ESTOP_SENSE_NO 1        // 1 = A1 接常開（NO）接點：平常沒接通，按下才接 GND → LOW = 急停（11/19 實際接法）。
                                // 0 = 常閉（NC）：平常接 GND，按下或斷線 → HIGH = 急停。NC 斷線也會停，比較安全
#endif
#ifndef PITCH_DIR_INVERT
#define PITCH_DIR_INVERT 1      // 方向接反時改 1
#endif
#ifndef YAW_DIR_INVERT
#define YAW_DIR_INVERT 0
#endif

class ArduinoHal : public Hal {
 public:
  void begin();
  uint32_t millis() override { return ::millis(); }
  uint32_t micros() override { return ::micros(); }
  void step(uint8_t axis, bool forward) override;
  void enableSteppers(bool on) override { digitalWrite(PIN_STEPPER_EN, on ? LOW : HIGH); }
  void setFlywheelPwm(uint8_t pwm) override {
    analogWrite(PIN_FLY_L, pwm);
    analogWrite(PIN_FLY_R, pwm);
  }
  int32_t flywheelRpm() override;
  void enableFeeder(bool on) override { writeFeed(on); }
  bool estopActive() override {
#if ESTOP_SENSE_ENABLED
    return digitalRead(PIN_ESTOP_SENSE) == (ESTOP_SENSE_NO ? LOW : HIGH);
#else
    return false;
#endif
  }
  void write(const char* s, size_t n) override { Serial.write((const uint8_t*)s, n); }
  float batteryVolts() override;
#if WHEELS_ENABLED
  void enableWheels(bool on) override { digitalWrite(PIN_WHEEL_EN, on ? HIGH : LOW); }
  void setWheel(uint8_t i, int16_t duty) override;
  int32_t wheelCount(uint8_t i) override;
  uint16_t wheelRaw(uint8_t i) override;
  uint8_t wheelPins() override;
#endif

 private:
  void writeFeed(bool on);
  uint8_t feed_phase_ = 0;          // 推球目前通電的相（0–3）；斷電後再通電從同一相開始，不會跳步
#if WHEELS_ENABLED
  void beginWheels();
  uint16_t enc_last_[4] = {0, 0, 0, 0};
  int32_t enc_acc_[4] = {0, 0, 0, 0};
#endif
  int8_t last_dir_[2] = {-1, -1};   // -1 = 開機後還沒寫過 DIR：第一步一定寫（DIR 腳開機是 LOW，不能假設已經是正方向）
  uint32_t tach_t0_ = 0, vbat_t_ = 0;
  int32_t rpm_ = 0;
  float vbat_ = -1.0f;
};
#endif  // ARDUINO
