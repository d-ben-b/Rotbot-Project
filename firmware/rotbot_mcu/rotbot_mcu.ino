// Rotbot MCU 韌體（NUCLEO-G474RE，STM32duino；同一份程式也能編 Arduino Uno）
// 與 Jetson Orin 走 USB 序列 115200 bps、協定 v1（docs/mcu_orin_link.md）。
// Nucleo 的 Serial = LPUART1（PA2/PA3）= 板上 ST-LINK 的虛擬序列埠，接 ST-LINK 那個 USB 孔就好。
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
