# MCU ↔ Orin 連線：協定、函式庫與測試環境

> 2026-10-25。回答「MCU + Orin 函式庫能不能接起來、要用什麼測試環境」。
>
> **2026-10-31 更新：MCU 是 NUCLEO-G474RE（STM32G474RE），不是 Arduino Uno。** 下文寫 Uno 的地方以這段為準：
> - 韌體不用改寫：用 Arduino API 寫的，透過 ST 官方的 STM32duino 核心編譯。FQBN 是 `STMicroelectronics:stm32:Nucleo_64:pnum=NUCLEO_G474RE`。
>   出廠設定 33.3 KB flash（6%）/ 1.8 KB RAM（1%）；全感測 40.3 KB（7%）/ 1.9 KB（1%）；我們的檔案 0 警告。
> - 用板上的 Arduino Uno 排針，腳位編號不變，對應的 STM32 腳位見 `config.h`。I/O 是 **3.3 V**。
> - USB 序列：板上 ST-LINK 的虛擬序列埠（LPUART1 PA2/PA3），Jetson 上是 `/dev/ttyACM0`。
>   **開埠不會重置 MCU**：沒有開機訊息，歸零狀態也會保留；`SerialMCU` 依 USB VID 0x0483 自動判斷，不白等 3 秒。
> - 燒錄、接線、測試步驟見 [00_todo.md](00_todo.md)。

## 1. 結論

| 已驗證（不需要硬體） | 結果 |
|---|---|
| 協定 v1 雙端實作：Orin 端 `rotbot/actuation/{protocol,link}.py`、MCU 端 `firmware/rotbot_mcu/` | ✅ |
| **同一份韌體原始碼**在電腦上跑（SIL），Orin 端函式庫走真正的 pyserial 路徑連上去 | ✅ 21/21 項（全感測設定）、19/19 項（出廠設定） |
| 韌體用 arduino-cli 編譯成 **Arduino Uno**（`--warnings all`，我們的檔案 0 警告） | ✅ 出廠 13.1 KB flash（40%）/ 667 B RAM（32%）；全感測 15.1 KB（46%）/ 693 B（33%） |
| Orin 端 HAL `send(AimCommand)`（轉到位 → 到速 → 推球 → 等 FIRED）打真韌體 | ✅ |
| 自主主迴圈 / 手動操控網頁 → 函式庫 → 韌體 整串 | ✅（見 teleop_network.md、six_meter_reassessment.md §5） |

**還沒驗證、需要板子的：** 真 Arduino 的 USB 序列（延遲、開埠重置）、馬達方向/步數比、轉速計與電壓量測的實際讀值。測試工具已備好，板子到手照 §5 跑 L2→L4 即可。

```text
Jetson Orin (Python)                         NUCLEO-G474RE (C++，STM32duino)
  ControlLoop / TeleopServer                   rotbot_mcu.ino
        │ HAL: aim / flywheel / fire / jog            │
  SerialMCU（rotbot/actuation/link.py）          App（app.cpp：狀態機、看門狗、急停、去重）
   ├ 背景讀取緒：解析行、依 SEQ 配對回覆              ├ proto.cpp：組行、CRC-8、切欄位
   ├ 逾時重送（同 SEQ）                              └ Hal（hal_arduino.cpp：步進/PWM/轉速計/電壓/急停）
   └ 10 Hz 輪詢 STAT（＝心跳）
        │                                              │
        └──────── USB 序列 115200 bps（/dev/ttyACM0、COM5）─┘
                  或 socket://127.0.0.1:5760（電腦上的韌體模擬器 = 同一份 app.cpp/proto.cpp）
```

## 2. 協定 v1（ASCII 一行一幀 + CRC-8）

```text
<BODY>*<CRC>\n     BODY = TYPE,SEQ[,ARG...]（逗號分隔、只有整數）
                   CRC  = CRC-8/SMBUS（poly 0x07, init 0）對 BODY 計算，兩位大寫十六進位
例：AIM,12,2513,-150*34\n   → 轉到 俯仰 25.13°、水平 −1.50°
```

選 ASCII：隊友可直接在 Arduino 序列埠監控視窗閱讀與手打（`ALLOW_NO_CRC=1` 時接受無 CRC 的手打行）、換行自動重新同步、全整數（0.01° / rpm）免去 AVR 浮點解析。最長 64 bytes。

| 指令（Orin → MCU） | 參數 | 回覆 | 說明 |
|---|---|---|---|
| `PING` | – | `PONG,seq,proto,fw,uptime_ms` | 存活與版本 |
| `STAT` | – | `ST,seq,flags,pitch_cdeg,yaw_cdeg,rpm,rpm_target,shots,err,crc_errors,vbat_dv` | 狀態（Orin 每 100 ms 輪詢，兼看門狗心跳） |
| `ZERO` | pitch_cdeg, yaw_cdeg | `ACK` | 宣告目前姿態角度（開機擺好起始姿態後；**未 ZERO 不准 AIM**） |
| `AIM` | pitch_cdeg, yaw_cdeg | `ACK` / `NAK` | 轉到絕對角度（梯形速度、減速停在目標） |
| `FLY` | rpm | `ACK` / `NAK` | 飛輪目標轉速（0 = 停） |
| `FIRE` | – | `ACK` 後送 `EV FIRED` / `NAK` | **到位穩定 + 飛輪到速才推球** |
| `JOG` | pitch_cdps, yaw_cdps | `ACK` | 手動速度模式（需持續送，否則看門狗停） |
| `STOP` | – | `ACK` | JOG 歸零、飛輪停、車輪停（位置保持） |
| `DRIVE` | m1, m2, m3, m4（千分比 ±1000） | `ACK` / `NAK` | 四輪輸出（M1 左前、M2 左後、M3 右前、M4 右後；正 = 前進）。變化率限制 `WHEEL_SLEW_PER_S`；**`DRIVE_TIMEOUT_MS`（0.5 s）沒收到就停** |
| `WHL` | – | `WH,seq,rpm10×4` | 四輪輸出軸轉速 × 10（編碼器，每 100 ms 更新） |
| `ENC` | – | `ER,seq,raw×4,pins` | 除錯：四個計時器原始計數、A/B 腳位電位（bit 2i = Mi+1 的 A、2i+1 = B） |

| 事件（MCU → Orin） `EV,evseq,NAME,arg` | 何時 |
|---|---|
| `BOOT` | 開機（Uno 開埠被 DTR 重置 → 約 1.6 s 後；Nucleo 開埠不重置，只有上電或按 RESET 時才有） |
| `READY` | 進入可發射狀態 |
| `FIRED` | 推球完成（arg = 累計球數） |
| `WDOG` | 超過 `WDOG_MS`（1 s）沒收到合法指令 → 飛輪停、JOG 停 |
| `ESTOP` | 急停按下（1）/ 放開（0）；放開後須重新 `ZERO` |
| `FAULT` | 推球逾時（卡彈）；不計球數 |

NAK 錯誤碼：1 FORMAT、2 UNKNOWN、3 RANGE、4 NOT_READY、5 ESTOP、6 BUSY、7 NOT_HOMED。
STAT 旗標：0x01 READY、0x02 MOVING、0x04 FLY_OK、0x08 FIRING、0x10 ESTOP、0x20 WDOG、0x40 HOMED、0x80 JOGGING。

**可靠性設計**
- **SEQ 去重**：同 SEQ、同內容的重送只回上次的回覆不重做 → `FIRE` 的 ACK 遺失時重送**不會連發兩球**（測試 #16 驗證）。
- **CRC 錯**：MCU 直接丟棄、不回覆（SEQ 不可信），計入 `crc_errors`；Orin 逾時後同 SEQ 重送。
- **看門狗**：Orin 當機/斷線 1 s → 飛輪停、JOG 停；收到新指令自動解除。
- **READY 條件**：已 ZERO、未急停、未看門狗、未推球中、兩軸停止且穩定 `SETTLE_MS`、飛輪到速（有轉速計：連續 200 ms 在 ±2% 內，超出 ±4% 才取消；沒有轉速計：改目標後等 `FLY_SPINUP_MS`）。
- **飛輪穩速（6 m 關鍵）**：前饋 + 電池電壓補償（`VBAT_ENABLED`）+ PI（只在 ±5% 內積分，防起轉過衝）。轉速計用「脈衝週期」量（0.02% 解析度），不用 100 ms 計數（2.5% 量化誤差）。

## 3. 韌體設定（`firmware/rotbot_mcu/config.h`，每項可單獨 `-D` 覆寫）

| 腳位（Nucleo 的 Arduino 排針） | 用途 |
|---|---|
| D4 (PB5) / D7 (PA8) | 水平軸 TMC2209 STEP / DIR |
| D8 (PA9) / D12 (PA6) | 俯仰軸 TMC2209 STEP / DIR |
| A3 (PB0) | 兩軸 EN（LOW 致能） |
| D5 (PB4) / D6 (PB10) | 左 / 右飛輪 PWM |
| D9 (PC7) | 推球（曲柄滑塊）馬達 PWM |
| CN7-1 (PC10) | 推球原點開關（選用；11/03 前在 A0，A0 改給車輪編碼器） |
| A1 (PA1) | 急停輔助接點（選用，`ESTOP_SENSE_ENABLED`） |
| A2 (PA4) | 飛輪電池電壓分壓（選用，`VBAT_ENABLED`；不可超過 `VBAT_ADC_VREF`） |
| D2 (PA10) / D3 (PB3) | 飛輪轉速計（外部中斷，選用，`FLY_HAS_TACH`） |
| 車輪 × 4（BTS7960 + 霍爾編碼器） | 腳位表見 [00_todo.md](00_todo.md) 第 2-2 步；`config.h` 的 `WHEEL_*` |

> 2026-11-03：車輪是四顆編碼器馬達（每轉 11 脈衝、減速比 1:56 → 輸出軸每轉 2464 計數），BTS7960 驅動。
> 編碼器用 TIM1/TIM4/TIM5/TIM20 的 encoder mode，PWM 20 kHz；TIM2/TIM3 留給 D5/D6/D9。
> 四片模組的 R_EN/L_EN 共用 PB12（外接 10 kΩ 下拉），收到第一個合法幀才致能、急停時拉低。
> 摩擦輪沒有編碼器，D2/D3 預計改接彈倉的微動開關；彈倉 6 顆 N20 + TB6612 的腳位見 00_todo.md 第 2-4 步（韌體還沒寫）。

⚠️ 待硬體組確認：`PITCH/YAW_STEPS_PER_DEG`（目前 26.67 / 48.254：200 步 × 16 微步 × 減速比 3 / 38/7；俯仰是皮帶輪 20 → 60 齒，水平是行星齒輪，太陽 21 齒、外環 93 齒）、角度上下限、`FLY_MAX_RPM`、`FEED_MS`、方向是否要反（`*_DIR_INVERT`）。
6 m 建議配置：**轉速計 + 電池電壓量測 + 推球原點開關**都裝（「full」設定；Uno 資源足夠）。

## 4. 測試結果（韌體模擬器，2026-10-25）

`python -m tools.mcu_link_test --sil`（`--variant basic` 測出廠設定）：

| 項目 | full（轉速計+電壓+原點開關） | basic（出廠） |
|---|---|---|
| BOOT / PING / 版本 | ✅ | ✅ |
| 往返延遲（200 次） | 平均 0.25 ms、p95 0.35 ms | 平均 0.20 ms、p95 0.25 ms |
| 壞 CRC 丟棄、垃圾資料重同步、未知指令/格式錯/超範圍/未 ZERO/未就緒 FIRE → NAK | ✅ 全部 | ✅ 全部 |
| 吞吐量（STAT 一問一答） | ~1900 次/秒 | ~2700 次/秒 |
| 轉塔 AIM 到 (20°, −5°) | 569 ms | 569 ms |
| 飛輪 2400 rpm → READY | 1137 ms（量測 2405 rpm） | 1510 ms（開迴路等待） |
| FIRE → EV FIRED | 351 ms | 402 ms |
| 重送去重 | 送兩次只推 1 球 ✅ | ✅ |
| JOG 20°/s × 0.5 s → STOP | 轉 10.2° 後停住 ✅ | ✅ |
| 看門狗 | 沉默 1000 ms → EV WDOG、飛輪歸 0 ✅ | ✅ |
| 急停 | NAK ESTOP、放開需重新 ZERO ✅ | ✅ |
| 電池掉到 85%（10.2 V） | 1.1 s 到速、穩在 2422 rpm（0.9%）✅ | –（無電壓量測：會掉 15%） |
| 推球卡彈 | 逾時 EV FAULT、馬達停 ✅ | – |

⚠️ 模擬器的延遲是 TCP 本機迴路，**不代表 USB 序列**。真 Uno（16U2 USB-CDC）一般 1–4 ms；CH340 類似；FTDI 預設 latency_timer 16 ms（見 §6）。工具對實機的門檻是 p95 < 20 ms。

## 5. 測試環境（由軟到硬，每一層同一支工具）

| 層 | 環境 | 指令 | 驗什麼 |
|---|---|---|---|
| L0 | 任何電腦 | `pytest` | 協定編解碼、CRC 標準值、狀態機、伺服器規則 |
| L1 | 電腦 + 韌體模擬器（已完成） | `python -m tools.mcu_link_test --sil` | 同一份韌體邏輯 + 函式庫的全部行為 |
| L2 | Arduino 單板接電腦 USB | 燒錄 → `python -m tools.mcu_link_test --port COM5` | 真 USB 序列：開埠重置、延遲、CRC、NAK（**不會動馬達**） |
| L3 | Arduino + 驅動板 + 馬達（桌上，無機構） | `--port COM5 --allow-motion` | 步進方向/步數比、飛輪起轉、轉速計/電壓讀值、推球、看門狗 |
| L4 | Arduino 改接 Orin | `--port /dev/ttyACM0 --allow-motion` | Orin 上的權限、延遲（同 L2/L3 標準） |
| L5 | 加上機構 | `python -m tools.run_autonomous --mcu /dev/ttyACM0 --twin` | 轉塔真的動、影像與計分先用數位孿生（還沒裝發射機構也能跑） |
| L6 | 整機 | `python -m tools.run_autonomous --mcu /dev/ttyACM0`（相機預設用 IMX296） | 真相機 + YOLO + 真投球 |

燒錄：Arduino IDE 開 `firmware/rotbot_mcu/rotbot_mcu.ino` 上傳（步驟見 00_todo.md）；或
`arduino-cli compile --upload -p COM5 --fqbn STMicroelectronics:stm32:Nucleo_64:pnum=NUCLEO_G474RE firmware/rotbot_mcu`。
手動除錯：序列埠監控視窗 115200、換行 LF，直接打 `PING,0`、`STAT,0`、`ZERO,0,0,0`、`AIM,0,1000,0`（SEQ 0 = 不去重）。

## 6. Orin 端設定與常見問題

- **權限**：`sudo usermod -aG dialout $USER` 後重新登入。
- **CH340 板子被 brltty 搶走**（Ubuntu 22.04 常見，/dev/ttyUSB0 一閃即逝）：`sudo apt remove brltty`。
- **固定裝置名**（避免 ttyACM0/1 互換）：udev 規則
  `SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", SYMLINK+="rotbot-mcu"` → 用 `/dev/rotbot-mcu`。
- **FTDI 延遲**：`echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer`（預設 16 ms）。
- **開埠重置**：Uno 開埠會重置約 1.6 s，`SerialMCU.open()` 會等 `EV BOOT`（最多 3 s）再 PING。Nucleo（ST-LINK，USB VID 0x0483）開埠不重置，直接 PING。
- **連線中 MCU 重開機**（2026-10-31 加入）：收到 `EV BOOT`，或狀態從「已歸零」變「未歸零」且不是急停，`SerialMCU.reboots` 加 1。
  - 處理：網頁伺服器呼叫 `restore_after_reboot()`。
  - 補回歸零：ZERO 設成重開機前最後一筆角度，加上重開機後被 JOG 轉過的步數。
  - 重送：重開機前的 AIM 目標和飛輪轉速。
  - 網頁顯示紅色警告，按「重置」才消失。
  - 急停造成的未歸零不算重開機，舊角度也作廢，因為急停時步進斷電，轉塔可能被推動。
- **用 40-pin UART 而不是 USB 時**：Orin 和 Nucleo 都是 3.3 V，共地即可（Uno 是 5 V 才需要電位轉換）；建議直接用 USB（同時供電給 Nucleo）。
- **電源**：馬達電源與 Uno/Orin 分開，共地；紅色急停要切斷馬達電源（規則），`ESTOP_SENSE_ENABLED` 讓 MCU 也知道急停狀態。
