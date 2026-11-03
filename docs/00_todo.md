# 我要做什麼（照順序做）

2026-10-31 版。每一步都寫了「怎麼做」和「看到什麼就算成功」。
要確認軟體本身對不對，看另一份：[怎麼確認軟體是對的](00_verify.md)。

指令都在專案資料夾的終端機執行，先啟動環境：電腦上 `.venv\Scripts\activate`；Jetson 上把 `python` 換成 `python3`。

---

## 1. 把韌體燒進 MCU（NUCLEO-G474RE）

只要一條 USB 線，接板子上方 ST-LINK 那一端的 USB 孔，不用接任何馬達。
韌體是用 Arduino 寫的，用 ST 官方的 Arduino 核心（STM32duino）編譯。

1. 安裝 Arduino IDE 2，在「檔案 → 偏好設定 → 其他開發板管理員網址」貼上：
   `https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json`
2. 「工具 → 開發板 → 開發板管理員」搜尋 STM32，安裝 **STM32 MCU based boards**（STMicroelectronics）。
3. 打開 `firmware/rotbot_mcu/rotbot_mcu.ino`。同一個資料夾裡的其他檔案會一起編譯，不要搬動。
4. 選板子和埠：
   - 「工具 → 開發板 → STM32 MCU based boards」選 **Nucleo-64**。
   - 「工具 → Board part number」選 **Nucleo G474RE**。
   - 其他選項維持預設，上傳方式是 Mass Storage，不用另外裝軟體。
   - 「工具 → 連接埠」選 STMicroelectronics STLink Virtual COM Port 那個（這台電腦目前是 COM5；換電腦或換 USB 孔可能會變）。
     裝置管理員裡沒有這個 COM 埠的話，安裝 ST 的 ST-LINK 驅動（STSW-LINK009）。
5. 按左上角的「→」上傳。
   - ✅ 沒有紅色錯誤訊息，程式大小約 33 KB（6%）。
   - 上傳方式是把程式複製到板子模擬出來的隨身碟 NOD_G474RE。
6. 打開「工具 → 序列埠監控視窗」，右下角選 **115200 baud** 和 **Newline**，再按板子上**黑色的 RESET 鍵**。
   - ✅ 看到 `EV,1,BOOT,1*91`，這是開機訊息。
   - Nucleo 打開序列埠時不會自動重開機（Uno 會），所以要自己按 RESET 才會看到。
7. 在輸入框打 `PING,0`，按 Enter。
   - ✅ 回 `PONG,0,1,1,…`。兩個 1 是協定版本和韌體版本，最後的數字是開機後經過的毫秒數。
   - 也可以試試以下指令。每行第二個欄位一律打 0。

     | 輸入           | 意思                | 正常回應                               |
     | -------------- | ------------------- | -------------------------------------- |
     | `STAT,0`       | 查目前狀態          | `ST,0,…`                               |
     | `AIM,0,1500,0` | 轉到俯仰 15.00°     | 還沒歸零時回 `NAK,0,7`（7 = 還沒歸零） |
     | `ZERO,0,0,0`   | 把現在的姿勢設成 0° | `ACK,0,0`                              |

     歸零之後再打一次 `AIM,0,1500,0`，會回 `ACK,0,0`。

8. **關掉序列埠監控視窗**，按一下 RESET，然後在終端機執行：

   ```bash
   python -m tools.mcu_link_test --port COM5
   ```

   - ✅ 最後一行 `FAIL 0`，PASS 大約 11 項。
   - SKIP 的有三種：要動馬達的、只有模擬器能測的，以及開機訊息那一項。最後這項 SKIP 是因為 Nucleo 開埠不會重開機。
   - 板子上**還沒接任何馬達**的話，可以加 `--allow-motion` 再跑一次，應該是 `FAIL 0`、PASS 約 17 項。
     這時腳位會送出訊號，但沒有東西會動。

## 2. 接線（給硬體組，接馬達之前看）

Nucleo 有兩種排針：

- 和 Arduino Uno 同位置的 CN5、CN6、CN8、CN9，下表寫成 D4、A3 這種名稱。
- 板子兩側的長排針 CN7、CN10，下表寫成 CN10-28，意思是 CN10 第 28 腳。

括號內是 STM32 腳位名稱。

### 2-1 轉台步進馬達（TMC2209）

| 排針                 | 接到                             |
| -------------------- | -------------------------------- |
| D4 (PB5) / D7 (PA8)  | 水平轉台 TMC2209 的 STEP / DIR   |
| D8 (PA9) / D12 (PA6) | 俯仰 TMC2209 的 STEP / DIR       |
| A3 (PB0)             | 兩顆 TMC2209 的 EN（LOW = 通電） |

### 2-2 四顆編碼器馬達（BTS7960）

每顆馬達接一片 BTS7960 模組。PWM 和編碼器都要接在有計時器（TIM）功能的腳。

| 馬達 | 模組 RPWM               | 模組 LPWM              | 編碼器 A                   | 編碼器 B                  | 用途（硬體組填） |
| ---- | ----------------------- | ---------------------- | -------------------------- | ------------------------- | ---------------- |
| M1   | CN10-28 (PB14)          | CN10-26 (PB15)         | A5 (PC0)                   | A4 (PC1)                  |                  |
| M2   | CN7-35 (PC2)            | CN10-2 (PC8)           | CN10-4 (PC6)               | D11 (PA7)                 |                  |
| M3   | D14 (PB9)               | CN10-1 (PC9)           | D10 (PB6)                  | CN7-21 (PB7)              |                  |
| M4   | CN10-18 (PB11)          | CN10-12 (PA12)         | CN10-22 (PB2)              | CN7-3 (PC12)              |                  |

- 用到的計時器：
  - PWM：M1 用 TIM15，M2 用 TIM20，M3 用 TIM8，M4 用 TIM2 和 TIM16。
  - 編碼器：M1 到 M4 依序用 TIM1、TIM3、TIM4、TIM5 的 encoder mode。
  - 同一顆馬達的 A、B 要接在同一個計時器的 CH1、CH2，**不能換到別的腳**。換腳要先問我。
- 四片模組共用的線：
  - **R_EN、L_EN 八條全部接在一起，接到 CN10-16 (PB12)，另外用一顆 10 kΩ 電阻接到 GND。**
    - LOW = 四顆馬達全部斷電。
    - MCU 開機中、重開機、或韌體還沒控制這支腳時，電阻會把 EN 拉低，馬達不會突然轉。
  - VCC 接 Nucleo 的 3.3 V（CN6 第 4 腳或 CN7-16），**不要接 5 V**。
    - 模組上有一顆 74HC244 緩衝晶片。它用 4.5 V 供電時，輸入要 3.15 V 以上才保證算 HIGH，5 V 供電時門檻更高；Nucleo 的 3.3 V 訊號不夠保險。
    - VCC 改接 3.3 V 後，74HC244 送給 BTS7960 的訊號是 3.3 V；BTS7960 晶片的 HIGH 門檻最高約 2.15 V，夠用。
  - GND 接 Nucleo 的 GND。
  - R_IS、L_IS（電流偵測）不接。
- 每片模組的電源與馬達：
  - B+、B− 接馬達電池，要經過紅色急停開關。
  - M+、M− 接馬達的兩條電源線。
- 編碼器：
  - VCC 接 3.3 V，GND 接 GND。
  - 先確認馬達規格寫的編碼器電壓包含 3.3 V（例如「3.3–5 V」）。只能用 5 V 的話先跟我說，要確認腳位能不能接 5 V 訊號。
  - 編碼器的線不要和馬達電源線綁在一起走，減少雜訊。
- 轉向或編碼器計數方向反了不用重接，跟我說，韌體改設定就好。
- **韌體目前還沒有 BTS7960 和編碼器的程式。**
  - 照這張表接好後，告訴我每顆馬達的用途和編碼器規格（每轉幾個脈衝、減速比），我再改韌體。
  - 在那之前，EN 被電阻拉低，馬達不會轉。

### 2-3 選配與舊接法

| 排針               | 接到                                                                                          |
| ------------------ | --------------------------------------------------------------------------------------------- |
| A0、A1、A2         | 選配：推球原點開關、急停輔助接點、電池分壓。沒裝就不用接                                      |
| D2、D3             | 選配：兩個飛輪轉速計。飛輪改用上面的編碼器馬達的話不用接                                      |
| D5、D6、D9         | 舊版的左 / 右飛輪驅動板 PWM、推球馬達 PWM。馬達都改用 BTS7960 的話不用接，也不要接別的東西    |

**Nucleo 的腳位是 3.3 V，不是 Uno 的 5 V：**

- TMC2209 的 VIO 接板子的 3.3 V。
- 兩顆 TMC2209 的 MS1、MS2 都接 3.3 V，設成 16 微步，韌體是照 16 微步算角度的。
  兩隻都接地時是 8 微步，轉台會轉成指令角度的 2 倍。
- BTS7960 模組和編碼器的 VCC 接 3.3 V（見 2-2）。
- 開關和感測器只接到 GND。韌體已經打開內部上拉，不要把 5 V 直接接進腳位。
- 電池分壓用 47 kΩ / 10 kΩ（3S 滿電 12.6 V 會變成 2.2 V）。
  ADC 參考電壓要量 CN5 第 8 腳（VREFP），把量到的數字填進 `config.h` 的 `VBAT_ADC_VREF`。

其他設定：

- 腳位、每度幾步、角度上下限都寫在 `firmware/rotbot_mcu/config.h`，改完要重新上傳。BTS7960 和編碼器的腳位等韌體改好才會加進去。
- **第一次接馬達時**，飛輪不要放球，發射口前面不要站人，然後執行：

  ```bash
  python -m tools.mcu_link_test --port COM5 --allow-motion
  ```

  - ✅ 結果是 `FAIL 0`。
  - 轉的方向相反：把 `hal_arduino.h` 的 `PITCH_DIR_INVERT` 或 `YAW_DIR_INVERT` 改成 1。
  - 轉的角度不對：先確認上面的微步設定，再改 `config.h` 的 `PITCH_STEPS_PER_DEG` 或 `YAW_STEPS_PER_DEG`。

## 3. 相機（Jetson 上）

- 已經標定好了：IMX296 + 6 mm 鏡頭，數字在 `configs/field.yaml` 的 `camera_calib`。
- **不要再轉鏡頭的對焦環和光圈環。** 動到的話要重新標定，步驟在 `docs/imx296_jetson.md`。
- 確認相機正常：在 Jetson 上執行 `python3 -m tools.camera_check`。
  - ✅ 會存下一張照片，幀率約 47–60 fps。

## 4. 量出球速（等飛輪做好）

- 6 m 要打得到所有的孔，出球速度要 **至少 14 m/s**，大約是飛輪 2400 rpm。這是硬體最重要的一個數字。
- 在離牆 2 m、4 m、6 m 三個距離各打約 10 球，記下每一球的落點，存成 CSV。
  每一行是：距離、仰角、水平角、落點橫向位置、落點離地高度（單位：公尺、度）。
- 執行：

  ```bash
  python -m tools.fit_ballistics shots.csv --calib-out runs/calib/practice.json
  ```

  - ✅ 會印出出球速度 v0，以及一段要貼進 `configs/field.yaml` 的數字。
  - 貼進去以後，AI 瞄準和操控台的預估落點都會改用實測值。
  - 只打兩個距離的話，速度和空氣阻力會分不開，所以三個距離都要打。

## 5. 實拍孔板（不用等機器人）

1. 用紙板做一面等比例的孔板：孔徑 Ø40 / Ø20 cm，間距照 `configs/field.yaml`，上排的加分孔貼紅色 LED 燈條。
2. 相機放在 6 m 外，執行：

   ```bash
   python -m tools.capture_dataset --n 200 --every 0.5 --prelabel models/grid_holes_yolo11n.pt --out datasets/real/board1
   ```

   - ✅ 資料夾裡會有照片和自動標好的框。框大多正確，就代表模型在真實照片上能用。
   - 這批照片之後拿來**微調**模型，不用從頭訓練。

## 6. 整台機器人

1. Jetson 只要準備一次：
   - numpy 和 OpenCV 用 JetPack 內建的（numpy 1.21、OpenCV 4.8）。**不要裝 `requirements.txt`**，它會把這兩個換成新版。
   - 依序執行：

     ```bash
     pip3 install -r requirements-jetson.txt
     # 這裡裝 NVIDIA 給 JetPack 6 的 PyTorch 和 torchvision
     pip3 install --no-deps ultralytics==8.4.188
     ```

     ultralytics 一定要加 `--no-deps`，不加的話它也會把 numpy 和 OpenCV 換掉。

   - ✅ 執行下面這行，印出的三個值依序是 `1.21` 開頭、`4.8` 開頭、`True`（GPU 可用）：

     ```bash
     python3 -c "import numpy, cv2, torch; print(numpy.__version__, cv2.__version__, torch.cuda.is_available())"
     ```

   - 把 `models/grid_holes_yolo11n.pt` 複製過去。模型權重不在 git 裡。
   - 序列埠權限：`sudo usermod -aG dialout $USER`，重新登入才生效。Nucleo 接上 Jetson 會是 `/dev/ttyACM0`。

2. 在 Jetson 上執行 `python3 -m tools.robot_main --mcu /dev/ttyACM0`，相機會自動用 IMX296。
3. 筆電或手機連同一台路由器，打開它印出來的網址（`http://192.168.x.x:8080`）。
4. 開始前，先把轉塔擺到「槍管水平、正對前方」，然後按「歸零」。
5. 按「開始」就等於哨音：前 60 秒 AI 自己投，之後可以按「切手動」。
6. 手動時看畫面上的圈：**綠圈 = 這一球會進，橘圈 = 不會進。**
   右下角的小圖是整面網板，瞄高處時網板會跑出畫面，就看小圖。
7. 畫面上方出現紅色的「MCU 重開機」警告時：
   - 程式已經自動補回歸零，也重送了轉塔目標和飛輪轉速，比賽照常進行，不用按任何鍵。
   - 賽後檢查 USB 線和電源，按「重置」警告就會消失。
   - 警告寫「無法自動補回」，代表重開機前沒按過歸零。這時要回待命，擺好姿勢重新歸零。

## 7. 比賽前（練習日）

1. 在場地做一次第 4 步，更新 `configs/field.yaml`，並存好 `runs/calib/practice.json`。
2. 比賽時帶著這個校正檔啟動：

   ```bash
   python3 -m tools.robot_main --mcu /dev/ttyACM0 --calib runs/calib/practice.json
   ```

3. 從練習到比賽，飛輪轉速要穩定，誤差在 1% 以內。轉速漂掉的話，練習學到的校正會變得不準。

---

## 要跟硬體組要的數字

要的數字和要填的位置：

- **出球速度**：做第 4 步，跑完會自動算出來。
- **發射口到網框的水平距離**：填到 `field.yaml` 的 `dist_x`，目前是 6.00 m。
  機器人如果站在投球線後面一點，要算進去。
- **發射口離地高度**：填到 `field.yaml` 的 `launch_h`，目前是猜的 0.40 m。
- **相機相對槍管的位置和角度**：填到 `field.yaml` 的 `camera_mount`，目前假設同軸、對齊。
- **步進馬達每度幾步、方向**：改 `config.h` 和 `hal_arduino.h`。
- **投球時機器人擺在網框正前方，還是偏左/右多少**：填到 `field.yaml` 的 `grid_cx`。
  左右偏 1 m 以內，相機都看得到全部的孔。
  AI 瞄準靠相機實際量到的孔位，不受這個數字影響；操控台的預估落點和小地圖則需要它。
