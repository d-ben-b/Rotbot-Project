// 韌體的硬體抽象層：App（協定 + 狀態機）只透過這個介面碰硬體。
// Arduino 實作：hal_arduino.h；電腦模擬器實作：firmware/host/sil_main.cpp。
#pragma once
#include <stddef.h>
#include <stdint.h>

enum Axis : uint8_t { AXIS_PITCH = 0, AXIS_YAW = 1 };

class Hal {
 public:
  virtual uint32_t millis() = 0;
  virtual uint32_t micros() = 0;
  virtual void step(uint8_t axis, bool forward) = 0;   // 發一個步進脈衝
  virtual void enableSteppers(bool on) = 0;
  virtual void setFlywheelPwm(uint8_t pwm) = 0;        // 0~255，兩輪同值
  virtual int32_t flywheelRpm() = 0;                   // 量測轉速；沒有轉速計回傳 -1
  virtual void setFeeder(uint8_t pwm) = 0;             // 推球馬達 0~255
  virtual bool feederHome() = 0;                       // 推球在原點（沒開關時回傳 true）
  virtual bool estopActive() = 0;
  virtual void write(const char* s, size_t n) = 0;     // 序列輸出
  virtual float batteryVolts() { return -1.0f; }       // 飛輪電池電壓；沒量測回傳 -1
};
