# firmware/ — MCU 韌體（NUCLEO-G474RE，STM32duino；也能編 Arduino Uno）

| 路徑 | 內容 |
|---|---|
| `rotbot_mcu/rotbot_mcu.ino` | Arduino 進入點（`setup()` / `loop()`） |
| `rotbot_mcu/config.h` | **腳位、步數比、角度限制、飛輪/推球/看門狗參數**——硬體確定後只改這裡 |
| `rotbot_mcu/app.cpp/.h` | 協定處理 + 狀態機（轉塔梯形速度、飛輪前饋+電壓補償+PI、推球、看門狗、急停、SEQ 去重） |
| `rotbot_mcu/proto.cpp/.h` | 協定 v1：組行、CRC-8、切欄位（純 C++） |
| `rotbot_mcu/hal_arduino.*` | Arduino 硬體實作（TMC2209 STEP/DIR、PWM、轉速計週期量測、電池電壓、急停感測） |
| `host/sil_main.cpp` | 在電腦上跑**同一份** app/proto 的模擬器（模擬硬體 + TCP 當序列埠） |

協定規格、腳位表、測試層級與 Orin 端設定：[docs/mcu_orin_link.md](../docs/mcu_orin_link.md)。

## 燒錄

- Arduino IDE：裝 STM32duino 核心（STM32 MCU based boards），開 `rotbot_mcu/rotbot_mcu.ino`，板子選 Nucleo-64 → Board part number「Nucleo G474RE」，上傳（Mass Storage）。逐步說明見 [docs/00_todo.md](../docs/00_todo.md)。
- arduino-cli：`arduino-cli compile --upload -p COM5 --fqbn STMicroelectronics:stm32:Nucleo_64:pnum=NUCLEO_G474RE firmware/rotbot_mcu`
- Nucleo 的 I/O 是 3.3 V；電池分壓要配合 `VBAT_ADC_VREF`（量 CN5 第 8 腳 VREFP）。
- 選配感測裝好後在 `config.h` 打開：`FLY_HAS_TACH`、`VBAT_ENABLED`、`FEED_HAS_HOME_SWITCH`、`ESTOP_SENSE_ENABLED`。

資源（STM32duino 3.0.0 / arm-none-eabi-gcc 14.2）：G474RE 出廠設定 33.3 KB flash（6%）、1.8 KB RAM（1%）；全感測 40.3 KB（7%）、1.9 KB（1%）。
Uno（avr-gcc 7.3）：13.1 KB（40%）、667 B（32%）。

## 測試

```bash
python -m tools.mcu_link_test --sil                       # 電腦上：建置並啟動模擬器，跑全部測試
python -m tools.mcu_link_test --port COM5                 # Nucleo 接電腦：只測通訊（不動馬達）
python -m tools.mcu_link_test --port /dev/ttyACM0 --allow-motion   # Orin + Nucleo + 馬達
```

序列埠監控視窗（115200、換行 LF；Nucleo 開埠不重開機，按黑色 RESET 才看得到 `EV,1,BOOT`）可直接手打：`PING,0`、`STAT,0`、`ZERO,0,0,0`、`AIM,0,1500,0`、`FLY,0,2400`、`FIRE,0`、`STOP,0`。
