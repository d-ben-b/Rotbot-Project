// Rotbot MCU 韌體（Arduino Uno）
// 與 Jetson Orin 走 USB 序列 115200 bps、協定 v1（docs/mcu_orin_link.md）。
// 協定與狀態機在 app.cpp / proto.cpp（純 C++，電腦上的模擬器 firmware/host 也用同一份）；
// 腳位與機構參數改 config.h。
#include "app.h"
#include "hal_arduino.h"

ArduinoHal hal;
App app(hal);

void setup() {
  Serial.begin(115200);
  hal.begin();
  app.begin();          // 送出 EV,BOOT（Orin 端 SerialMCU.open() 會等它）
}

void loop() {
  while (Serial.available() > 0) app.onByte((char)Serial.read());
  app.update();
}
