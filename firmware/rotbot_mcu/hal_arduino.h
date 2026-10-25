// Arduino（Uno）硬體實作：TMC2209 STEP/DIR、PWM 馬達、轉速計、急停感測、USB 序列。
#pragma once
#ifdef ARDUINO
#include <Arduino.h>

#include "config.h"
#include "hal.h"

#ifndef ESTOP_SENSE_ENABLED
#define ESTOP_SENSE_ENABLED 0   // 急停輔助接點接好後改 1（沒接線時腳位浮接會一直判成急停）
#endif
#ifndef PITCH_DIR_INVERT
#define PITCH_DIR_INVERT 0      // 方向接反時改 1
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
  void setFeeder(uint8_t pwm) override { analogWrite(PIN_FEED, pwm); }
  bool feederHome() override {
#if FEED_HAS_HOME_SWITCH
    return digitalRead(PIN_FEED_HOME) == LOW;
#else
    return true;
#endif
  }
  bool estopActive() override {
#if ESTOP_SENSE_ENABLED
    return digitalRead(PIN_ESTOP_SENSE) == HIGH;
#else
    return false;
#endif
  }
  void write(const char* s, size_t n) override { Serial.write((const uint8_t*)s, n); }
  float batteryVolts() override;

 private:
  bool last_dir_[2] = {true, true};
  uint32_t tach_t0_ = 0, vbat_t_ = 0;
  int32_t rpm_ = 0;
  float vbat_ = -1.0f;
};
#endif  // ARDUINO
