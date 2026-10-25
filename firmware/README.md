# firmware/ — Arduino（MCU）韌體

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

- Arduino IDE：開 `rotbot_mcu/rotbot_mcu.ino`，板子選 Arduino Uno，上傳。
- arduino-cli：`arduino-cli compile --upload -p COM5 --fqbn arduino:avr:uno firmware/rotbot_mcu`
- 選配感測裝好後在 `config.h` 打開：`FLY_HAS_TACH`、`VBAT_ENABLED`、`FEED_HAS_HOME_SWITCH`、`ESTOP_SENSE_ENABLED`。

資源（arduino-cli 1.5 / avr-gcc 7.3）：出廠設定 13.1 KB flash（40%）、667 B RAM（32%）；全感測 15.1 KB（46%）、693 B（33%）。

## 測試

```bash
python -m tools.mcu_link_test --sil                       # 電腦上：建置並啟動模擬器，跑全部測試
python -m tools.mcu_link_test --port COM5                 # Arduino 接電腦：只測通訊（不動馬達）
python -m tools.mcu_link_test --port /dev/ttyACM0 --allow-motion   # Orin + Arduino + 馬達
```

序列埠監控視窗（115200、換行 LF）可直接手打：`PING,0`、`STAT,0`、`ZERO,0,0,0`、`AIM,0,1500,0`、`FLY,0,2400`、`FIRE,0`、`STOP,0`。
