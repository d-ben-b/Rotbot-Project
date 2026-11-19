// 韌體的硬體抽象層：App（協定 + 狀態機）只透過這個介面碰硬體。
// Arduino 實作：hal_arduino.h；電腦模擬器實作：firmware/host/sil_main.cpp。
#pragma once
#include <stddef.h>
#include <stdint.h>

enum Axis : uint8_t { AXIS_PITCH = 0, AXIS_YAW = 1, AXIS_FEED = 2 };   // FEED = 推球 28BYJ-48

class Hal {
 public:
  virtual uint32_t millis() = 0;
  virtual uint32_t micros() = 0;
  virtual void step(uint8_t axis, bool forward) = 0;   // 發一個步進脈衝
  virtual void enableSteppers(bool on) = 0;
  virtual void setFlywheelPwm(uint8_t pwm) = 0;        // 0~255，兩輪同值
  virtual int32_t flywheelRpm() = 0;                   // 量測轉速；沒有轉速計回傳 -1
  virtual void enableFeeder(bool on) = 0;              // 推球線圈通電；false = 四相全關（停著時不發熱）
  virtual bool estopActive() = 0;
  virtual void write(const char* s, size_t n) = 0;     // 序列輸出
  virtual float batteryVolts() { return -1.0f; }       // 飛輪電池電壓；沒量測回傳 -1
  // 車輪（i = 0..3 → M1 左前、M2 左後、M3 右前、M4 右後）；沒有車輪的板子用預設的空實作
  virtual void enableWheels(bool) {}                   // BTS7960 EN
  virtual void setWheel(uint8_t, int16_t) {}           // -1000..1000（千分比），正 = 前進
  virtual int32_t wheelCount(uint8_t) { return 0; }    // 編碼器累計計數（4 倍頻，正 = 前進）
  virtual uint16_t wheelRaw(uint8_t) { return 0; }     // 除錯：計時器原始計數
  virtual uint8_t wheelPins() { return 0; }            // 除錯：bit 2i = Mi+1 的 A、bit 2i+1 = B 目前電位
};
