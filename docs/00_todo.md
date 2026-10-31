# 我要做什麼（照順序做）

2026-10-25 版。每一步都寫了「怎麼做」和「看到什麼就算成功」。
要確認軟體本身對不對，看另一份：[怎麼確認軟體是對的](00_verify.md)。

指令都在專案資料夾的終端機執行，先啟動環境：電腦上 `.venv\Scripts\activate`；Jetson 上把 `python` 換成 `python3`。

---

## 1. 把韌體燒進 Arduino Uno

只要一條 USB 線，不用接任何馬達。

1. 安裝 Arduino IDE 2，打開 `firmware/rotbot_mcu/rotbot_mcu.ino`。
   同一個資料夾裡的其他檔案會一起編譯，不要搬動。
2. 「工具 → 開發板」選 **Arduino Uno**；「工具 → 連接埠」選有 Arduino Uno 字樣的那個（例如 COM5）。
3. 按左上角的「→」上傳。
   - ✅ 下方出現「上傳完成」，程式大小約 13 KB。
4. 打開「工具 → 序列埠監控視窗」，右下角選 **115200 baud** 和 **Newline**。
   - ✅ 看到 `EV,1,BOOT,1*91`，這是板子開機的訊息。沒看到就按板子上的 RESET 鍵。
5. 在輸入框打 `PING,0`，按 Enter。
   - ✅ 回 `PONG,0,1,1,…`。兩個 1 是協定版本和韌體版本，最後的數字是開機後經過的毫秒數。
   - 也可以試試以下指令。每行第二個欄位一律打 0。

     | 輸入 | 意思 | 正常回應 |
     | --- | --- | --- |
     | `STAT,0` | 查目前狀態 | `ST,0,…` |
     | `AIM,0,1500,0` | 轉到俯仰 15.00° | 還沒歸零時回 `NAK,0,7`（7 = 還沒歸零） |
     | `ZERO,0,0,0` | 把現在的姿勢設成 0° | `ACK,0,0` |

     歸零之後再打一次 `AIM,0,1500,0`，會回 `ACK,0,0`。
6. **關掉序列埠監控視窗**。不關的話，Python 會打不開這個埠。
7. 在終端機執行：

   ```bash
   python -m tools.mcu_link_test --port COM5
   ```

   - ✅ 最後一行是 `結果：PASS 12  FAIL 0  WARN 0  SKIP 9`。
     SKIP 的 9 項有兩種：要動馬達的，和只有模擬器能測的。
   - 板子上**還沒接任何馬達**的話，可以加 `--allow-motion` 再跑一次。
     ✅ 應該是 `PASS 18  FAIL 0  SKIP 3`。這時腳位會送出訊號，但沒有東西會動。

## 2. 接線（給硬體組，接馬達之前看）

| Arduino 腳位 | 接到 |
| --- | --- |
| D4 / D7 | 水平轉台 TMC2209 的 STEP / DIR |
| D8 / D12 | 俯仰 TMC2209 的 STEP / DIR |
| A3 | 兩顆 TMC2209 的 EN（LOW = 通電） |
| D5 / D6 | 左 / 右飛輪驅動板 PWM |
| D9 | 推球馬達 PWM |
| A0、A1、A2、D2、D3 | 選配：推球原點開關、急停輔助接點、電池分壓、兩個飛輪轉速計。沒裝就不用接 |

- 腳位、每度幾步、角度上下限都寫在 `firmware/rotbot_mcu/config.h`，改完要重新上傳。
- **第一次接馬達時**，飛輪不要放球，發射口前面不要站人，然後執行：

  ```bash
  python -m tools.mcu_link_test --port COM5 --allow-motion
  ```

  - ✅ 結果是 `PASS 18`。
  - 轉的方向相反：把 `hal_arduino.h` 的 `PITCH_DIR_INVERT` 或 `YAW_DIR_INVERT` 改成 1。
  - 轉的角度不對：改 `config.h` 的 `PITCH_STEPS_PER_DEG` 或 `YAW_STEPS_PER_DEG`。

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
   - 裝 `requirements.txt` 的套件，以及 Jetson 版的 PyTorch + ultralytics。
   - 把 `models/grid_holes_yolo11n.pt` 複製過去。模型權重不在 git 裡。
2. 在 Jetson 上執行 `python3 -m tools.robot_main --mcu /dev/ttyACM0`，相機會自動用 IMX296。
3. 筆電或手機連同一台路由器，打開它印出來的網址（`http://192.168.x.x:8080`）。
4. 開始前，先把轉塔擺到「槍管水平、正對前方」，然後按「歸零」。
5. 按「開始」就等於哨音：前 60 秒 AI 自己投，之後可以按「切手動」。
6. 手動時看畫面上的圈：**綠圈 = 這一球會進，橘圈 = 不會進。**
   右下角的小圖是整面網板，瞄高處時網板會跑出畫面，就看小圖。

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
