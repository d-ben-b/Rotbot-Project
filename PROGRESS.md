# Rotbot 軟體進度報告

> AI 自主棒球投手機器人 — 軟體端｜嘻諧們｜2026 TEL 東京威力科創機器人大賽
> 報告日期：2026-10-31｜負責：軟體（單人）｜成果報告書：11/7–11/9（通訊硬體之後凍結）｜比賽日：2026-11-21

---

## 0. 一句話摘要

投球距離改為 **6 m** 後重算全部誤差與假設：舊設計（v0 10 m/s、無阻力瞄準）在 6 m 打不到上排，
已改為**含阻力瞄準模型 + 飛輪 ≥ 14 m/s 設計點**，模擬成績回到 48.6 分 / 命中 87%（2.5 m 時 52 分）。
同時補齊硬體到位前的所有軟體：**Orin↔MCU 協定 + 函式庫 + 韌體**（MCU 是 NUCLEO-G474RE，用 STM32duino 編譯；同一份韌體在電腦上模擬，25/25 測試通過；含四輪 BTS7960 + 編碼器）、
**路由器區網的網頁手動操控**（鍵盤/搖桿）、**自主主迴圈與比賽主程式**（網頁按開始 → 60 秒 AI → 切手動）。
整條「看→算→轉→投」已在數位孿生（真韌體 + 合成相機 + 彈道世界）跑完整場；剩下的都是需要實機的驗證。
**相機定案**（IMX296 + 6 mm，10/07 實機標定）已入檔並用它重跑：6 m 時整面網框入鏡，YOLO 不用重訓（模擬/合成評估）。
手動操控畫面改成標出**預估落點**（原本中央的準星只是相機正前方）。給自己的入口文件：
[docs/00_todo.md](docs/00_todo.md)（要做什麼）、[docs/00_verify.md](docs/00_verify.md)（怎麼確認）。

| 里程碑 | 狀態 | 重點產出 |
|---|---|---|
| P0 專案骨架 + 硬體抽象層 | ✅ 完成 | 三層架構、HAL、集中式設定、原型模組化、CI |
| P1 感知（圓孔偵測 + 定位） | ✅ 完成 | 合成資料（2.5–7.5 m）、YOLO 重訓、整板 PnP、兩種位姿法 |
| P2 決策（圓孔版） | ✅ 完成 | 座標轉換、**含阻力瞄準**、轉塔運動學、加分優先策略、校正存檔 |
| P3 致動 / 通訊 | ✅ 核心完成 | 協定 v1、SerialMCU、NUCLEO-G474RE 韌體（STM32duino）、韌體模擬器、狀態機、看門狗/急停 |
| P4 模擬閉迴路 / 自主主迴圈 | ✅ 完成 | 蒙地卡羅、render+YOLO 閉迴路、數位孿生整場、比賽主程式 |
| 手動操控（網頁） | ✅ 完成 | 路由器區網、鍵盤/搖桿、60 秒鎖定、deadman、MJPEG 串流、**預估落點 + 網板小地圖** |
| 互動物理沙盒（PyBullet） | ✅ 核心完成 | 依 CAD/規則建模、手動⇄YOLO 自動 |
| P5 硬體整合 / P6 賽前 | ▶ 進行中 | 相機已到位並標定（IMX296 + 6 mm）；其餘等硬體，步驟見 docs/00_todo.md |

規模：Python ~11,700 行（git 追蹤的 .py，含測試/工具/原型）、韌體 C++ ~1,100 行、90 個自動化測試（全綠，另 2 項只在 Linux / 實體相機跑）。

---

## 1. 系統架構

三層 + 硬體抽象層（HAL），主邏輯只依賴介面，模擬與真硬體可互換：

```text
感知 perception → 決策 decision → 致動 actuation
        ↕ 皆依賴 hal.py 抽象介面（Camera / ActuatorLink）↕
   sim/（模擬替身）              hardware（未來真硬體）
```

**為什麼這樣設計**：硬體尚未到位。把「相機」「下位機」抽象成介面，沒硬體時塞模擬替身
（`SimCamera` / `MockArduino`），硬體到了換成真實實作，**主邏輯一行不改**——這是能在無硬體
狀態下先把整條閉迴路做完的關鍵。所有物理參數集中在 `configs/field.yaml`，硬體實測後只改設定檔。

---

## 2. 已完成工作明細

### P0 — 專案骨架（commit `36668bb`）
- 建立 `rotbot/` 套件：`perception / decision / actuation / sim / pipeline`
- `hal.py`：`Camera` / `ActuatorLink` 抽象介面 + `AimCommand` / `Ack` 資料契約
- `config.py` + `configs/field.yaml`：型別化集中設定
- 把既有兩支原型模組化：PnP 位姿、拋體模型、線上校正
- 建立 pytest 測試與 GitHub Actions CI

### P1 — 感知系統（圓孔）
- **合成場景** `sim/scene.py`：把圓孔以隨機相機視角投影成橢圓，輸出影像 + **像素級精確真值**
  （3D 中心、投影橢圓、距離/方位/傾角），並做域隨機化（背景、眩光、模糊、色偏）模擬現場干擾。
- **資料集工具** `tools/gen_synthetic_data.py`：輸出 Ultralytics YOLO 偵測格式（類別：一般孔 / 加分孔 / 亮起加分孔）。
- **模型訓練** `tools/train_detector.py`：本機 RTX 4050 訓練 yolo11n，2000 張 / 100 epochs。
- **偵測包裝** `perception/detector.py`：YOLO 推論 → `Detection`（中心 / bbox / 直徑 / 類別 / 網格編號）。
- **全鏈定位** `perception/localize.py`：偵測 → 穩健孔對應 → 整板 PnP → 每孔瞄準（`HoleAim`），並帶出亮起加分孔。
- **兩種位姿法** `perception/pose.py`：方法 A（孔中心整板 PnP + 表觀直徑測距）、方法 B（橢圓圓錐反投影，含傾角）。
- **評估與 demo**：`tools/compare_pose_methods.py`（兩法比較）、`tools/eval_perception.py`（端到端評估）、
  `tools/demo_perception.py`（瞄準 HUD 視覺 demo，產掃描影片）、`sim/perfect_detector.py`（訓練前的閉迴路替身）。

### 6 m 重評估 + P3 + 自主主迴圈 + 手動操控（2026-10-25）
- **6 m**：`decision/ballistics.DragAimModel`（含阻力數值瞄準）、`tools/reassess_6m.py`（A–F 節一鍵重算）、
  YOLO 以 2.5–7.5 m 合成資料重訓；結論與硬體規格見 [docs/six_meter_reassessment.md](docs/six_meter_reassessment.md)。
- **Orin↔MCU**：協定 v1（ASCII + CRC-8 + SEQ 去重）、`actuation/link.SerialMCU`、`firmware/rotbot_mcu`（Uno 韌體）、
  `firmware/host`（同一份韌體在電腦上跑）、`tools/mcu_link_test.py`；見 [docs/mcu_orin_link.md](docs/mcu_orin_link.md)。
- **手動操控**：`rotbot/teleop`（aiohttp + WebSocket + MJPEG、鍵盤/搖桿）、`tools/teleop_web.py`；
  見 [docs/teleop_network.md](docs/teleop_network.md)。
- **自主主迴圈**：`pipeline/loop.ControlLoop`（掃視→偵測→PnP→轉塔運動學→瞄準→校正→MCU）、`AutonomousRunner`、
  `sim/twin.py`（數位孿生）、`tools/run_autonomous.py`、**比賽主程式 `tools/robot_main.py`**。
- **硬體到位省時工具**：`tools/fit_ballistics.py`（打牆擬合 v0/Cd/偏差）、`tools/bench_pipeline.py`（Orin 延遲、TensorRT）、
  `tools/capture_dataset.py`（實拍 + 預標註）；計畫見 [docs/hardware_bringup_plan.md](docs/hardware_bringup_plan.md)。

### 相機定案 + 預估落點 + 入口文件（2026-10-25 晚）
- **合併 `feat/imx296-camera`**：`perception/imx296.py`（V4L2 + ISP + 去畸變）、`perception/intrinsics.py` + `tools/calibrate_camera.py`、
  `tools/camera_check.py`、10/07 實機標定（`camera_calib`）。`camera:`（模擬用）改成實機去畸變後的針孔內參；
  工具的 `--camera` 預設 `cfg` = 依 `camera_hw` 開 IMX296（`perception.camera.open_camera`）。
- **預估落點**（`teleop/aim_preview.py`）：轉塔實際角度 + 飛輪目標轉速 → 同一個含阻力模型 → 網板上的落點、最近的孔、會不會進；
  畫在 MJPEG 畫面上（綠 = 會進、橘 = 不會進，畫面外畫箭頭）＋右下角網板小地圖＋網頁文字。比賽主程式會帶入當場校正器的全域偏差。
- **入口文件**：[docs/00_todo.md](docs/00_todo.md)（燒錄、接線、量球速…照順序做）、[docs/00_verify.md](docs/00_verify.md)（怎麼確認）。

### MCU 重開機自動補回 + Jetson 套件清單（2026-10-31）
- **MCU 重開機**：`SerialMCU` 偵測重開機（`EV BOOT`，或已歸零 → 未歸零且不是急停），`restore_after_reboot()` 用重開機前的角度 ZERO、
  重送 AIM 目標與飛輪轉速；網頁伺服器自動執行並顯示紅色警告。
- **Jetson**：`requirements-jetson.txt`（不含 numpy / OpenCV，用 JetPack 內建的），ultralytics 用 `--no-deps` 安裝。

---

## 3. 重要調整與轉折

### ★ 最大轉折：目標形狀由「方形」改「圓孔」
- **起因**：原始原型 `pnp_frame_pose.py` 用 `SOLVEPNP_IPPE_SQUARE`（正方形）假設；但賽事工程圖與計劃書
  確認九宮格是**圓孔**（一般 Ø40cm、加分 Ø20cm，計劃書明列「霍夫圓偵測」「直徑」）。原始 sim 甚至自相矛盾
  （校正模擬把它當直徑=圓，PnP 卻當邊長=方）。
- **影響**：圓沒有角點，「YOLO-pose 四角點 + 方形 PnP」前提不成立。
- **處置**：全面改為「圓孔中心 + 整板 PnP」路線——重寫 `config / scene / detector / gen_data / pose`，
  並把座標佈局集中到 `Config.hole_centers()`（主 3×3 + 上排 3 加分孔）。
- **教訓**：**動工前先查一手規格**（工程圖），不要沿用既有程式碼的隱含假設。

### 其他調整
- 角點方案（YOLO-pose 關鍵點）→ 因改圓孔，改為 **bbox 偵測孔中心**（更好訓練、標註更省、Orin 更快）。
- 資料集格式：YOLO-pose → YOLO 偵測；`train_pose.py` → `train_detector.py`。

---

## 4. 遇到的問題與解法

| 問題 | 原因 | 解法 |
|---|---|---|
| 訓練當掉（記憶體不足） | 8 個 dataloader worker × 1920×1080 大圖 × mosaic 把系統 RAM 吃爆 | `workers` 降為 2、`cache=False`（`train_detector.py`）|
| 訓練輸出跑到別的專案資料夾 | ultralytics 全域 `runs_dir` 指向舊專案 | 輸出改用**絕對路徑** `REPO_ROOT/runs` |
| 整板 PnP 常失敗（6/50） | `solvePnPRansac` 在**共面點**上不穩 | 改用平面 PnP `SOLVEPNP_IPPE` 初估 |
| 個別孔瞄準大錯（95pct 114cm） | 2D 行列分群指派孔編號**脆弱**，真實偵測有多餘/邊緣框就配錯號 | **投影重匹配**：初估位姿→投影已知孔中心→就近認領正確編號→重解 |
| demo 顏色混淆 | 合成場景把孔緣隨機上色（域隨機化），與 HUD 語意色打架 | demo 用中性灰孔緣（`rim_jitter=False`），只留 HUD 有顏色 |

---

## 5. 關鍵技術取捨

| 取捨 | 選擇 | 理由 |
|---|---|---|
| 真硬體 vs 模擬 | **硬體抽象層 (HAL)** 隔離 | 無硬體先開發；硬體到位只換底層 |
| 合成資料引擎 | **OpenCV 程式化合成**（非 Unity/Blender） | 重用投影數學、標註免費精確、可進 CI、單人可控；擬真不足處靠域隨機化 |
| 圓孔測距/位姿 | **方法 A 整板 PnP 為主，B 橢圓補傾角，尺寸法備援** | A 最準最穩；B 唯一能回傳單孔傾角（孔被打歪時用）|
| 偵測 head | **一般 bbox 偵測**（非 pose / seg） | 圓孔中心定義清楚，bbox 最簡單、Orin 最快，對齊計劃書單目幾何 |
| 設定管理 | **YAML + 型別化 dataclass** | 單一參數來源；硬體實測後只改 YAML，程式不動 |

---

## 6. 量化成果（模擬 / 合成評估，6 m；2.5 m 時期數字見 §9 歷史條目）

**偵測模型**（yolo11n、2000 張 2.5–7.5 m 合成資料重訓）：驗證集 mAP50 一般孔 / 加分孔 / 亮孔 皆 **0.995**（R 0.996）。
**端到端感知 @ 6 m**（合成影像 60 場）：整板 PnP **60/60**、亮孔 **50/50**、距離誤差中位 **0.2 cm**、方位/俯仰 **0.01°**、孔編號配錯 0。
**換成實機相機（IMX296 + 6 mm，去畸變 1456×1088、f 1794）**：比賽掃視姿態 12 孔全入鏡（四邊 ≥ 218 px）；
render + YOLO 閉迴路（6 場）**51.0 分 / 90% / 加分孔 44%**，YOLO 不用重訓。隨機視角（方位 ±18°）時板子常被切掉：
PnP 46/60、孔編號配錯 7/571（舊 69° 佔位相機 60/60、0/707）——比賽時掃視姿態把網框置中，不會遇到。

**閉迴路（各 60 場）**：

| 條件 | 總分 | 命中率 | 加分孔 |
|---|---|---|---|
| 2.5 m 舊假設（對照） | 52.1 | 93% | 35% |
| 6 m 沿用舊假設（v0 10、無阻力） | 4.8 | 9% | 0% |
| **6 m 設計點（v0 14、含阻力、校正 + 賽前練習 9 球）** | **48.6** | **87%** | **30%** |
| 同上但比賽中凍結校正（沒有落點量測） | 49.0 | 87% | 34% |
| render 影像 + 真 YOLO（6 場） | 49.3 | 88% | 33% |
| 同上，實機相機 IMX296 + 6 mm（6 場） | 51.0 | 90% | 44% |

**硬體規格（誤差預算）**：v0 ≥ 14 m/s（10 m/s 只打得到 4/12 孔）；轉塔逐發抖動 ≤ 0.2°（0.4° → 加分孔 30%、0.2° → 48%）；
v0 逐發抖動 ≤ 0.1 m/s；練習→比賽轉速漂移 < 1%（否則需要比賽中落點量測）。

**系統（數位孿生 = 真韌體 + 合成相機 + 彈道世界）**：整場 27 球全由韌體推出、50 分、1.33 s/球（規則節拍 6.7 s/球；實機相機版 1.28 s/球、YOLO 1.40 s/球，都是 50 分）；
網頁 → MCU 指令 0.6 ms（本機區網）；PC 上感知 + 決策 13.6 ms/幀；韌體 NUCLEO-G474RE 35.1 KB flash（6%）/ 2.0 KB RAM（1%）。

---

## 7. 待確認事項與風險

| 項目 | 現況 | 需要的動作 |
|---|---|---|
| ✅ **亮孔顏色** | **官方 Q33 確認為紅色** LED 燈條（寬約14mm、網框邊緣內約5cm、~475±25lm、有線開關）。（先前暫定綠色）| 已改 `scene.py` 綠→**紅色燈環 + 紅色域隨機化**、`pybullet_world` marker 改紅；**待重生成合成資料 + 重訓 lit 類別** |
| ✅ 加分規則時窗 | **官方 Q34 確認**：哨響於 3 格**隨機亮 1 格、固定 60 秒後熄滅** → **加分孔只有前 60 秒能得分**。（先前口述「整場一直亮」為誤）| 已於 `ShotPlanner` 建模 60 秒窗口：窗口內優先加分、窗口後不再選加分孔 |
| 場地間距（孔徑/欄距/列距） | ✅ 已用**官方尺寸標示圖**實值：欄距0.90、列距0.60、加分排0.50（公差±2cm）| 完成 |
| ⚠️ **飛輪 v0** | 6 m 需 ≥ 14 m/s（約 2400 rpm）；舊設計 1700 rpm ≈ 10 m/s 打不到上排 | **硬體組決定馬達/驅動板**；飛輪試驗台量 v0 與逐發抖動 |
| ⚠️ 轉塔重複精度 | 6 m 加分孔只容許 ±0.6°；抖動 0.4° → 加分孔 30% | 目標 ≤ 0.2°；上桌量（hardware_bringup_plan H3） |
| dist_x / grid_cy / launch_h | dist_x 定為 6.0；grid_cy 由官方正視圖推為 1.80（假設板底=地面）；launch_h 仍暫估 0.40 | 練習日實測；PnP 本來就逐球量距離 |
| 合成→真實落差 | 模型全合成資料 | 現在就用等比例孔板 + 紅 LED 收實拍、預標註、微調（`tools/capture_dataset.py`） |
| 落點量測 | 線上校正需要；目前只有模擬（孿生）提供 | 轉速穩在 ±1% 時賽前練習暖機即可；比賽中自動判斷當保險 |
| ⚠️ 無線干擾 | 場館攝影機用 5.1–5.9 GHz、6.5 W；場館無 Wi‑Fi | 練習日測 2.4/5 GHz；**11/9 報告書前**決定是否加 RC 備案（通訊硬體之後凍結） |
| ✅ 相機內參 | 2026-10-07 實機標定完成（RMS 0.71 px、fx 不確定度約 1.2%），存於 `camera_calib:`；相機輸出去畸變影像。10/25 起 `camera:`（模擬）也用同一組去畸變內參 | 換鏡頭或重新對焦後重標定（`camera:` 跟著改）。想更準：再拍一輪近距離（0.4–0.8 m）並用 `--resolve` 合併 |
| ✅ 6 mm 視角（10/07 的 ★） | 2.5 m 時只看得到部分孔而認錯孔；**6 m 時整面網框入鏡**（掃視姿態四邊 ≥ 218 px），render+YOLO 51 分，不必換鏡頭 | 仍有的風險：板子被切掉時會認錯孔（隨機視角 7/571）→ 機器人偏離網框中心 > 1 m 時把 `grid_cx` 填進設定 |
| 預估落點的準度 | 依 v0 / Cd / 安裝偏差的猜測值：模擬中沒校正時差約 20 cm，帶入練習 9 球的校正後 ≤ 6 cm | 打牆擬合（`fit_ballistics`）+ 練習校正檔；實機用牆上膠帶驗證（docs/00_verify.md D 節） |
| 相機外參 `camera_mount` | 假設同軸、對齊；畫面上的預估落點標記位置依賴它（1° ≈ 畫面 31 px） | 硬體組量相機相對槍管的位置與角度 |
| Jetson 執行環境 | 機上有 numpy 1.21 / OpenCV 4.8 / pytest 6.2.5；沒有 ultralytics、torch、pybullet、模型權重。10/31 起用 `requirements-jetson.txt` + `ultralytics --no-deps`，在 Windows 用相同版本組合模擬過可推論 | 照 docs/00_todo.md 第 6 步建環境、跑檢查指令（確認 NVIDIA 的 PyTorch 能配 numpy 1.21）、把權重搬上去 |
| MCU 比賽中重開機 | 自動補回歸零並重送 AIM / 飛輪（模擬：重開機到下一球 1.06–1.31 s，正常 1.12 s）。重開機時轉塔在轉的話，補回的角度最多差 6°（俯仰）/ 9°（水平）；模擬中 AI 不受影響，手動的預估落點會跟著偏 | 實機按板上 RESET 驗證（docs/00_verify.md C 節）；查明重開機原因（電源、USB 線） |
| 車輪（BTS7960 + 四顆編碼器馬達） | 11/03 韌體、協定（`DRIVE` / `WHL`）、操控台開車都做好；韌體模擬器測試通過，G474RE 編譯通過。**還沒在實機上跑過**：encoder mode 與 20 kHz PWM 的設定只有編譯驗證 | 車架高後跑 `mcu_link_test --allow-drive`（docs/00_todo.md 第 2 步），確認四輪方向與編碼器 |
| 摩擦輪、推彈、彈倉 | 摩擦輪 ×2、推彈各用一片 BTS7960 單向（RPWM ← D5/D6/D9），現有韌體就能控制。⚠️ 摩擦輪沒有編碼器 → 轉速開迴路，建議接 A2 電池分壓做電壓補償。彈倉 6 顆 N20 + TB6612 × 3 + 微動開關，腳位已排（12 支），**韌體還沒寫**；腳位只剩 PC13、PA15 | 彈倉動作順序（一格放幾顆、何時轉）；N20 電壓版本與堵轉電流 |

---

## 8. 下一步

照順序做的清單：[docs/00_todo.md](docs/00_todo.md)；怎麼確認：[docs/00_verify.md](docs/00_verify.md)；
完整步驟與通過標準：[docs/hardware_bringup_plan.md](docs/hardware_bringup_plan.md)。

1. **本週（不必等整機）**：Nucleo 燒錄跑 `mcu_link_test --port`；等比例孔板 + 紅 LED 實拍（`bench_pipeline --camera cfg` 量距離、收微調資料）；
   Jetson 環境（ultralytics / PyTorch、權重）+ TensorRT 延遲；路由器實測網頁操控。
2. **硬體組決定**：飛輪 ≥ 14 m/s 方案、是否加轉速計/電壓量測/原點開關/急停感測、Wi‑Fi 是否加 RC 備案（11/9 前）；
   給數字：`launch_h`、`camera_mount`、步數比、投球時機器人左右位置。
3. **上桌**：步進方向/步數比、轉塔重複精度、飛輪 v0 與抖動、供彈 100 球、打牆擬合（`fit_ballistics`）。
4. **整機**：`run_autonomous --twin`（真轉塔）→ `robot_main`（真相機），60 秒自主彩排、手動彩排。
5. **練習日**：實拍微調、打牆擬合、存校正暖機檔、參數凍結。

> 完整階段規劃見 [ROADMAP.md](ROADMAP.md)；模組職責與快速試跑見 [README.md](README.md)。

---

## 9. 變更紀錄 (Changelog)

> **append-only**：每次更新只**新增**條目、不改舊條目，保留數字與決策的演進供工作日誌追蹤。
> 上方 §0–§8 為「最新快照」；本區為「歷史紀錄」。新到舊排列。

### 2026-11-04：路由器 + Orin 無線設定完成（只改文件）

- 使用者在沒有螢幕的情況下，用 USB-C（`ssh ben-orin@192.168.55.1`）設定 Orin：
  - Orin 連上隊上的 ASUS RT-AX1800HP。路由器已設 WPA2 密碼，並用 DHCP 保留把 Orin 的 IP 固定為 192.168.50.10。
  - Orin 的無線網卡名稱是 `wlP1p1s0`（文件原本寫的 `wlan0` 不對）。
- `docs/teleop_network.md` §3 補上實際設定、沒有螢幕時的登入方式、改 SSID 後怎麼重連，也把熱點指令的網卡名稱改對。
- 下一步：從筆電經 Wi-Fi 跑 `teleop_web --mcu none --camera none` 量延遲（docs/00_verify.md D 節「無線不會斷」）。
- 測試：程式沒改；全套 90 passed、2 skipped（11/03 最後一次）。

### 2026-11-03（續 5）：俯仰步進改評估 HBT5756C 閉環一體機；STEP / DIR 時序改成可設定

- 硬體組的俯仰馬達候選：HBT5756C（鴻寶達 57 系列一體式閉環步進：馬達 + 驅動器 + 編碼器）。
  - 規格書第 38 頁：1.5 N·m、4.0 A/相、機身 76 mm、DC 24–55 V；12 線 V+/V−/PU±/DR±/EN±/AL±。
  - 介面說明頁：PUL 上升緣有效、脈寬 > 1.5 µs；DIR 要比 PUL 早 ≥ 2 µs；高電位 4.5–28 V、低電位 0–0.5 V；ENA 導通 = 放鬆馬達、不收脈衝。
  - 細分：SW5–SW8 = on/on/off/on 為 3200 脈衝/轉（和韌體 200 步 × 16 微步一致）；SW4 一秒內來回撥一次 = 自動調參。
  - 文件之間不一致：規格書特點寫訊號「相容 3.3–28 V」、接線圖寫 5–24 V、介面頁寫 ≥ 4.5 V；電源 24–55 V / 24–75 V / 商品標題 24–48 V。
    介面頁還列了 A±/B± 線圈端子，推測是獨立驅動器的手冊，撥碼表要以馬達上實際的開關為準。
- Nucleo 輸出 3.3 V，低於介面頁的 4.5 V，建議用 ULN2003 以 5 V 共陽驅動（邏輯與現有韌體相同：HIGH = 光耦導通；A3 LOW = 馬達通電）。
- 韌體：`STEP_PULSE_US`、`DIR_SETUP_US` 改成 `config.h` 設定，預設都從 2 µs 改 5 µs（原本 DIR 2 µs 剛好壓在手冊下限）。
  TMC2209 只要 ≥ 0.1 µs，不受影響。G474RE 編譯 35,088 B（6%）/ 2,000 B（1%），0 警告；pytest 90 passed、2 skipped。
- 接線表（`00_todo.md` 2-1）等硬體組決定直接接或加 ULN2003、水平馬達型號後再改。

### 2026-11-03（續 4）：車輪是麥克納姆輪 → 操控台改三軸混控（前後、平移、旋轉）

- 硬體組告知車輪是麥克納姆輪。韌體不用改（`DRIVE` 本來就是四輪各給值）；原本操控台是左右同速的差速混控，只能前後和原地轉、不能平移。
- `configs/field.yaml` 新增 `chassis`（`mecanum: true`、`strafe_invert: false`），型別 `config.ChassisConfig`。
- `TeleopServer.mix(前進, 右移, 右轉)`：X 型排列，M1 左前 = f+s+r、M2 左後 = f−s+r、M3 右前 = f−s−r、M4 右後 = f+s−r，超過 1 時等比例縮小；
  `mecanum: false` 時橫移無效（退回差速），`strafe_invert` 讓橫移反向。`robot_main`、`teleop_web` 帶入 `cfg.chassis`。
- 操控台按鍵：↑↓ 前後、←→ 平移、`<` `>` 原地轉（半速）；搖桿左搖桿前後 / 平移、十字鍵左右原地轉、右搖桿瞄準。
- `docs/00_todo.md`：對角線兩顆用同一種輪；放地上依序試直走、原地轉（轉不動 = 左右旋裝反）、平移（反了改 `strafe_invert`）。
- 測試：`test_teleop_drive_mixing_and_deadman` 改成三軸；新增 `test_chassis_mix_options`；全套 **90 passed、2 skipped**。

### 2026-11-03（續 3）：彈倉接線（N20 + TB6612 + 微動開關）、摩擦輪沒有編碼器（只改文件）

- 硬體組回覆：
  - 摩擦輪、推彈用 BTS7960，電流夠（最大 43 A）。
  - 摩擦輪是直流馬達，沒有提到編碼器；這裡當作沒有。
  - 彈倉 6 顆都只往同方向轉，用微動開關判斷到位，馬達是 N20 直流減速馬達。
- 使用者翻到板子背面：CN10 外排旁印 2–38、CN7 外排印單數，和上一條的數法一致。`00_todo.md` 補「看背面最準」。
- 彈倉（`00_todo.md` 2-4）：
  - TB6612FNG × 3，單向：IN1 接 3.3 V、IN2 接 GND、PWM 接 MCU（HIGH 轉、LOW 煞車），STBY 接共用 EN（PB12）。
    規格（網路搜尋的 datasheet 摘要）：VM 2.5–13.5 V（絕對最大 15 V）、平均 1.2 A（部分版本 1.0 A）、VCC 2.7–5.5 V。
  - 馬達腳 PA11、PB13、PB1、PB9、PC3、PA5（D13，LED 會跟著亮）；開關腳 PC11、PD2、PC4 (D1)、PC5 (D0)、PA10 (D2)、PB3 (D3)。
  - 開關 COM 接 GND、NO 接 MCU（內部上拉）；建議小型滾輪槓桿式（例如 Omron SS-5GL2、KW11-3Z 滾輪型）。
  - 剩下沒用的腳：PC13（藍色按鈕）、PA15（焊橋 SB7）。
- 摩擦輪沒有轉速計 → D2/D3 改給彈倉；建議接 A2 電池分壓並開 `VBAT_ENABLED` 做電壓補償。
- 彈倉韌體還沒寫：要等動作順序（一格放幾顆、何時轉）。
- 測試：程式沒改；全套 89 passed、2 skipped。

### 2026-11-03（續 2）：排針怎麼數、摩擦輪 / 推彈 / 彈倉的接線建議（只改文件）

- 硬體組回覆：俯仰兩顆是步進馬達；彈倉 6 顆是直流馬達；摩擦輪、推彈還沒接線，請軟體安排。
- `docs/00_todo.md` 第 2 步：
  - 加「CN7、CN10 怎麼數」：第 1、2 腳在 USB 端；CN10 單數腳在內排、雙數腳在外排，CN7 相反。
    依據 UM2505 表 16：CN10 的單數腳對應 Arduino D15–D0、CN7 的雙數腳對應 Arduino 電源與 A0–A5，所以分別在靠 Arduino 母座的內排。
    用三用電表對 GND（CN10-20、CN7-19）確認。
  - 2-3：摩擦輪 ×2、推彈各一片 BTS7960 單向（RPWM ← D5/D6/D9、LPWM 接 GND、EN 接共用 PB12）。
    現有韌體已經在這三支腳輸出 PWM（20 kHz），不用改程式。
  - 2-4：彈倉預留 PA11、PB13、PB1、PB9、PC3、PC11（備用 PD2）；單向一顆一支腳。驅動板等馬達功能與電流確認後再定。
- 測試：程式沒改；全套 89 passed、2 skipped（上一條）。

### 2026-11-03（續）：車輪韌體（BTS7960 × 4 + 霍爾編碼器）與操控台開車

- **硬體組回覆**：馬達總共車輪 4、水平 1、俯仰 2、摩擦輪 2、推彈 1、彈倉 6；編碼器電壓含 3.3 V、每轉 11 脈衝、減速比 1:56；BTS7960 模組有 74HC244；摩擦輪、推彈的驅動板還沒定案。
  推論：四顆 BTS7960 編碼器馬達是車輪（只有這組是 4 顆）。
- **腳位重排**（上一條的計畫把 M2 編碼器放在 TIM3，和 D5/D9 的 PWM 衝突；摩擦輪、推彈未定案，所以保留 D5/D6/D9）：
  - 編碼器改用 TIM1 (PC0/PC1)、TIM4 (PB6/PB7)、TIM5 (PA0/PC12)、TIM20 (PB2/PC2)，不碰 TIM2/TIM3。
  - PWM：M1 TIM15 (PB14/PB15)、M2 TIM8+TIM17 (PC6/PA7)、M3 TIM8 (PC8/PC9)、M4 TIM2+TIM16 (PB11/PA12)；全部 20 kHz，D6 與 M4 RPWM 共用 TIM2 所以 `analogWriteFrequency` 也設 20 kHz。
  - 推彈原點開關從 A0 移到 PC10（A0 給 M3 編碼器）。
  - 俯仰兩顆步進：兩顆 TMC2209 的 STEP/DIR/EN 並聯（假設都是步進馬達）。
- **韌體**（`config.h`、`hal.h`、`hal_arduino.*`、`app.*`）：
  - `DRIVE,seq,m1,m2,m3,m4`（千分比）：變化率限制 2000‰/s、0.5 s 沒收到就停（deadman）；`STOP`、看門狗、急停都會停輪。
  - `WHL` → `WH,seq,rpm10×4`：每 100 ms 由編碼器計數差算輸出軸轉速。
  - BTS7960 EN（PB12）開機為 LOW，收到第一個合法幀才拉高，急停時拉低。
  - 方向設定 `WHEEL_INVERT`（預設 0x0C：右側 M3/M4 鏡像）、`WHEEL_ENC_INVERT`。
  - 編譯（arduino-cli，STM32duino 3.0.0）：G474RE 35,088 B（6%）/ 2,000 B（1%）；全感測 42,036 B（8%）/ 2,024 B（1%）；我們的檔案 0 警告。
    Uno（沒有車輪）14,780 B（45%）/ 765 B（37%）。
- **韌體模擬器**：加車輪模型（全速輸出軸 160 rpm、τ 0.15 s，模擬值）。`mcu_link_test --sil` **PASS 25、FAIL 0**（新增 4 項）：
  逐輪 30% 前進 1 s → 48.2 / 48.0 / 47.7 / 48.0 rpm；停送 DRIVE 1.3 s 後 ≤ 0.5 rpm；急停 → `DRIVE` NAK ESTOP、四輪 0.5 rpm。
  - 實機用 `--allow-drive`（車架高）才會轉車輪。
- **Python / 操控台**：
  - `SerialMCU.drive()`、`wheels()`；`protocol.py` 解析 `WH`。
  - 操控台：方向鍵 / 左搖桿開車（差速：左側 M1/M2、右側 M3/M4），W/S/A/D / 右搖桿瞄準（原本方向鍵和左搖桿是瞄準）；
    自主期間不開車；deadman 0.3 s、斷線 STOP 都會停輪。
- **驗證範圍**：encoder mode、20 kHz PWM、EN 腳的實際行為只有編譯驗證，還沒在 Nucleo 上量過。
- 測試：新增 `test_wheel_reply_parsed`、`test_stop_also_stops_wheels`（full / basic）、`test_teleop_drive_mixing_and_deadman`；全套 **89 passed、2 skipped**。

### 2026-11-03：BTS7960 + 四顆編碼器馬達的接線建議（只改文件）

- **起因**：硬體組改用 BTS7960 模組驅動四顆六線編碼器馬達，PWM 和編碼器都要接計時器腳。
- **腳位**（`docs/00_todo.md` 第 2-2 步）：
  - 來源：STM32duino 3.0.0 G474RE variant 的 `PinMap_TIM`，加上 UM2505 表 15–17。
  - 避開的腳：板上已用的 PA2/PA3（ST-LINK 序列）、PA13/PA14（SWD）、PB8（BOOT0）、PA5（LED）、PA15（要看焊橋 SB7），以及目前韌體用到的腳（含選配）。
  - 編碼器：TIM1 (PC0/PC1)、TIM3 (PC6/PA7)、TIM4 (PB6/PB7)、TIM5 (PB2/PC12)，各用 encoder mode。
  - PWM（RPWM/LPWM）：TIM15 (PB14/PB15)、TIM20 (PC2/PC8)、TIM8 (PB9/PC9)、TIM2 + TIM16 (PB11/PA12)。
  - 四片模組的 R_EN/L_EN 共接 PB12，加 10 kΩ 下拉：開機中、重開機、或韌體沒控制時馬達斷電。
  - 改韌體時：PC8、PC9、PB14、PB15、PB9、PC2、PA12 在 `PinMap_TIM` 的第一個選項不是上面這些計時器，要用 `_ALTx` 腳位名（例如 `PC8_ALT2` = TIM20_CH3）。
- **3.3 V 邏輯**：
  - BTS7960 模組（IBT-2）的輸入先經過 74HC244。74HC244 在 4.5 V 供電時 V_IH 最小 3.15 V，所以模組 VCC 改接 3.3 V。
  - BTS7960 的 IN/INH 高電位門檻約 1.75–2.15 V（Infineon datasheet Rev. 1.1，網路搜尋摘要，表格欄位沒有逐字確認）。
  - 編碼器 VCC 接 3.3 V；只能用 5 V 的編碼器要先確認腳位耐壓。
  - R_IS/L_IS 不接。
- 沒有判斷四顆馬達各自的用途；文件假設舊的飛輪 / 推球 PWM（D5、D6、D9）不再使用。
- **韌體尚未支援** BTS7960 和編碼器；`mcu_orin_link.md` 的腳位表下加註。
- 測試：程式沒改；全套 **85 passed、2 skipped**。

### 2026-10-31（續）：MCU 重開機自動補回歸零、Jetson 套件清單

- **起因**：上一條之後列出的兩個「接上韌體後」的問題。
  - 比賽中 MCU 重開機（電壓掉、碰到 RESET）後韌體忘記歸零，AIM 一律回 NAK 7（未歸零）。
    網頁的「歸零」只在待命時能按；按「重置」回待命會清掉比賽計時，再開始又鎖 60 秒自主。
  - `requirements.txt` 的 `numpy>=1.24` 在 Jetson（numpy 1.21）上會讓 pip 另裝新版 numpy。
    PyPI 上 `opencv-python` 5.0.0.93 要求 `numpy>=2`，`ultralytics` 8.4.188 要求 `numpy>=1.23.0` 和 `opencv-python`，所以只改 numpy 下限擋不住。
- **重開機偵測與補回**（`actuation/link.py`、`teleop/server.py`、`teleop/static/index.html`）：
  - `SerialMCU` 收到 `EV BOOT`，或狀態從已歸零變未歸零且不是急停 → `reboots` +1，`pose_before_reboot` 記下重開機前最後一筆角度。
    急停時舊角度作廢（步進斷電，轉塔可能被推動）。
  - `restore_after_reboot()`：ZERO = 重開機前角度 + 重開機後被 JOG 轉過的步數；再重送最後的 AIM 目標（JOG / STOP 之後不送）和飛輪轉速。
  - 網頁伺服器每 0.1 s 檢查一次，自動補回，頁面上方顯示紅色警告，按「重置」才消失。
- **驗證（模擬）**：
  - 韌體模擬器 `!RESET`：只算 1 次重開機；角度補回 (12.00°, −5.00°)；補回前 JOG 轉過的量有加上；轉到一半重開機時會繼續轉到目標；飛輪轉速恢復。
  - AI 自主中重開機（`robot_main.build_robot` + 數位孿生，3 次）：重開機到下一球投完 1.06–1.31 s，正常每球間隔中位數 1.11–1.12 s；重開機後 6 球中 5。
    沒有重送 AIM / 飛輪的版本是 5.53 s（進行中那一發等 4 s 逾時）。
  - 補回的角度有誤差時（轉塔實際角度 = MCU 計數 + δ，27 球，模擬/合成評估）：δ = (0, 0)、(1°, 1°)、(3°, −3°)、(−2°, 5°)、(6°, 0)、(0, 9°)、(−6°, −9°) 都是命中 24、50 分；
    誤差中位數 5.2 cm（δ = 0）、最大 7.6 cm（δ = (−6°, −9°)）。推測原因：AI 用同一個角度計數算相機位置和下瞄準指令，偏差在兩邊抵銷。手動的預估落點會跟著偏。
  - 重開機時轉塔在轉的話，補回的角度最多差一次狀態輪詢（0.1 s）轉的量：俯仰 60°/s → 6°、水平 90°/s → 9°，在上面測過的範圍內。
- **Jetson 套件**：
  - 新增 `requirements-jetson.txt`（pyyaml、pyserial、aiohttp、pillow、pytest）；numpy、OpenCV 用 JetPack 內建的；ultralytics 用 `pip3 install --no-deps ultralytics==8.4.188`。
  - 驗證（Windows 上模擬 Jetson 的版本組合）：Python 3.10.22、numpy 1.21.6、OpenCV 4.7.0、torch 2.14.1 CPU、ultralytics 8.4.188 `--no-deps` → YOLO 偵測 12 孔、PnP 有解。
    matplotlib 換成 3.5.3 也一樣；不裝 matplotlib 也能推論；不裝 pillow 會 import 失敗。
  - ultralytics 載入 `.pt` 權重時不會自動 pip 安裝；`.engine` 只在 Python ≤ 3.8.10 的 Jetson 上要求 numpy 1.23.5。
  - 還沒在 Jetson 上跑過；NVIDIA 的 PyTorch 能不能配 numpy 1.21 要在機上確認（`00_todo.md` 第 6 步的檢查指令）。
- **文件**：`00_todo.md` 第 6 步改成 Jetson 清單 + 檢查指令，並加「出現紅色 MCU 重開機警告時」；`00_verify.md` C 節加「按板上 RESET 看自動補回」；
  `mcu_orin_link.md` 加重開機處理；`requirements.txt`、`requirements-gpu.txt`、README 註明 Jetson 不用它們。
- 測試：新增 `test_reboot_detected_from_status_without_boot_event`、`test_reboot_detected_and_zero_restored`（full / basic 兩個韌體變體）、
  `test_teleop_restores_zero_after_mcu_reboot`；全套 **85 passed、2 skipped**。

### 2026-10-31：MCU 改為 NUCLEO-G474RE（韌體以 STM32duino 編譯）

- **起因**：使用者指出 MCU 是 NUCLEO-G474RE（STM32G474RE），附 ST 使用手冊 UM2505（`data/` 下，不入庫）。
  10/25 的韌體與文件以 Arduino Uno 為目標，依據是 repo 既有的 `MockArduino` / `SerialArduino` 命名，這個假設是錯的。
- **韌體不用改寫**：程式用 Arduino API，以 ST 官方 STM32duino 核心 3.0.0 編譯（FQBN `STMicroelectronics:stm32:Nucleo_64:pnum=NUCLEO_G474RE`）。
  - 出廠設定 33,312 B flash（6%）/ 1,828 B RAM（1%）；全感測 40,264 B（7%）/ 1,852 B（1%）；我們的檔案 0 警告。
  - 同一份程式仍可編 Uno：13,124 B / 667 B。
  - 對照 UM2505 表 15 與 STM32duino 的 variant：Arduino 排針 D2–D13 / A0–A3 對到的 STM32 腳位寫進 `config.h` 註解。
    `Serial` = LPUART1（PA2/PA3）= 板上 ST-LINK 的虛擬序列埠。
- **改動**：
  - 電池量測的 ADC 參考改成 `VBAT_ADC_VREF`（Nucleo 預設 3.3 V，Uno 5 V）。
    手冊沒有標出出廠的 VREF+ 是 2.5 V 還是 3.25 V（JP8 / SB14），要量 CN5 第 8 腳後填入。
  - `VBAT_DIVIDER` 預設改 5.7（47k/10k，12.6 V → 2.2 V）。
  - `SerialMCU`：ST-LINK（USB VID 0x0483）開埠不會重置 MCU，不再白等 3 s 的 `EV BOOT`（新函式 `resets_on_open`）。
    `mcu_link_test` 的開機事件在這種板子上回 SKIP。
  - CI 的韌體 job 改編 G474RE（GitHub Actions 上未驗證：repo 是 private，讀不到結果）。
- **文件**：
  - `docs/00_todo.md` 第 1、2 步改成 Nucleo：Arduino IDE 加 STM32duino、選 Nucleo-64 / Nucleo G474RE、Mass Storage 上傳、按 RESET 看開機訊息。
  - 接線表加 STM32 腳位與 3.3 V 注意事項：TMC2209 VIO 3.3 V、MS1/MS2 接高 = 16 微步、分壓、不可直接接 5 V。
  - Jetson 準備加上 `dialout` 群組與不要升級 numpy。
  - `00_verify.md` C 節、`mcu_orin_link.md`（頂端更新說明、腳位表）、`firmware/README.md`、README 同步。
- **另外驗證**：在這台電腦重現 Jetson 的 Python 3.10 + numpy 1.21（OpenCV 4.7 代替 JetPack 的 4.8），完整測試 80 passed、2 skipped。
- **環境**：Windows Smart App Control 約在 10/31 擋下 `.venv` 的 python.exe 與本機編譯的模擬器（WinError 4551）；使用者關閉後恢復。
- 測試：新增 `test_resets_on_open_by_usb_vendor`；全套 **81 passed、2 skipped**。

### 2026-10-25（續）：相機定案入檔、手動操控的預估落點、兩份入口文件

- **合併 `feat/imx296-camera`**（10/07 的相機工作：IMX296 驅動 + ISP + 去畸變、內參標定工具、實機標定 RMS 0.71 px）。
  衝突只在 `config.py`（兩邊欄位都留）、README、PROGRESS（§9 兩邊條目都保留）。合併後 73 passed、2 skipped。
- **`camera:` 改成實機去畸變後的針孔內參**（1456×1088、fx=fy 1794.1、主點 777.7 / 578.9；舊值 1920×1080、fx 1400 是佔位）。
  - 10/07 刻意不改，是因為 2.5 m 時 6 mm 只看得到部分孔、會認錯孔。
  - **6 m 時掃視姿態 12 孔全入鏡，四邊 ≥ 218 px**（幾何推算），這個理由已不存在。
  - 重跑 `reassess_6m --sections ECF`（模擬/合成評估，全文 docs/results/2026-10-25_reassess_6m_imx296.md）：
    - 感知 6 m 距離 0.5 / 1.5 cm、角度 0.014 / 0.034°；閉迴路 48.6 分 / 87% / 30%，都跟佔位相機相同。
    - render + YOLO 閉迴路 **51.0 分 / 90% / 加分孔 44%**（佔位相機 49.3 / 88% / 33%）→ **YOLO 不用重訓**。
    - 隨機視角（方位 ±18°）板子常被切掉：PnP 46/60、孔編號配錯 7/571（佔位相機 60/60、0/707）。
      比賽時掃視姿態置中不會遇到；機器人偏離中心 > 1 m 時要設 `grid_cx`。
  - 數位孿生整場：完美偵測 50 分 / 24/27 / 1.28 s/球；YOLO 50 分 / 24/27 / 1.40 s/球。
- **工具的相機改用設定檔**：`perception.camera.open_camera`，`--camera cfg` 依 `camera_hw` 開 IMX296。
  - `robot_main`、`run_autonomous`、`capture_dataset` 預設改為 `cfg`；`teleop_web`、`bench_pipeline` 多一個選項。
  - 拿掉 docstring 裡的 `nvarguscamerasrc` 範例：這顆相機不走 Argus。
  - `bench_pipeline` 結束時印出量到的中排中距離，給實拍孔板拿捲尺比對。
- **預估落點**（新檔 `rotbot/teleop/aim_preview.py`）：
  - **起因**：使用者問操控台中央的準星是不是落點。不是，那只是相機正前方；6 m 時落點比它低 1.2–1.6 m。
  - **算法**：轉塔實際角度 + 飛輪目標轉速 → v0 = v0_model × rpm / `launch.rpm_model`（新欄位，預設 2400）
    → 同一個含阻力模型（`model_shoot(cfg, v0)`）→ 網板上的落點、最近的孔、會不會整顆進。到網板前就落地時回報「到不了」。
  - **畫面**：MJPEG 上畫標記（綠 = 會進、橘 = 不會進、畫面外畫邊緣箭頭），右下角網板小地圖，網頁文字「✓ 會進『中排中』」。
    拿掉中央準星；影片框改 4:3。
  - 比賽主程式帶入當場校正器的全域偏差；開賽前用 `--calib` 載入的校正檔。
  - **驗證（模擬）**：
    - 依模型瞄準 12 孔，預估都落在該孔、離孔心 < 1 cm。
    - 標記像素跟數位孿生畫面的孔心差 < 2 px，而且在畫面中心下方 > 250 px。
    - 跟模擬真實世界（v0 13.44、Cd 0.50、安裝偏差）比：沒校正時中位差 20.1 cm；帶入練習 9 球的校正後中位 3.1 cm、最大 5.7 cm。
    - 用韌體模擬器 + 孿生相機存 HUD 截圖目視確認。網頁文字以 Node 跑 `renderPreview` 確認。
- **入口文件**：使用者表示新增的東西太多，要兩份簡單的資料。
  - 新增 `docs/00_todo.md`（燒錄 Arduino → 接線 → 相機 → 量球速 → 實拍 → 整機 → 練習日，每步附「成功的樣子」）。
  - 新增 `docs/00_verify.md`（每個說法對應一個指令與應有輸出；另列實機才能證明的事與過關標準）。
  - 序列埠手打的回應是從韌體模擬器實際抓的：`EV,1,BOOT,1*91`、`PONG,0,1,1,…`、未歸零 `NAK,0,7`。
  - 舊文件的過時處加註更新：hardware_team_questions、imx296_jetson、鏡頭需求「不必再買短焦鏡頭」、bringup plan、mcu/teleop 文件的 `--camera`。
- 其他：`reassess_6m` E 節改用設定檔的相機（第二台為舊佔位相機對照）。
- 測試：新增 `tests/test_aim_preview.py` 7 項；全套 **80 passed、2 skipped**（Windows；skip = Linux 限定的 V4L2 結構、實體相機煙霧測試）。

### 2026-10-25（6 m 重評估、Orin↔MCU、網頁手動操控、自主主迴圈）

- **環境**：本機預設 Python 3.15 沒有任何套件 → 建 `.venv`（Python 3.12，同 CI）。CUDA：RTX 4050 Laptop、驅動 621.01（CUDA 13.5）、
  **沒有 CUDA Toolkit 也不需要**——`torch 2.14.1+cu130` wheel 自帶執行環境，`torch.cuda.is_available()` = True。
  pybullet 在 Windows/Py3.12 無 wheel（需 MSVC）→ 移到 `requirements-sim3d.txt`；新增 `requirements-gpu.txt`。
  韌體模擬器在 Windows 用 pip 的 `ziglang` 當 C++ 編譯器。
- **6 m 參數**（`configs/field.yaml`）：`dist_x` 2.50→**6.00**；`grid_cy` 1.50→**1.80**（官方正視圖：中排孔心距板底 180 cm，
  假設板底=地面）；`v0_model/v0_real` 10/9.6→**14/13.44**（需求值）；新增 `aim` 區（`model: drag`、`cd_model: 0.45`）；
  `robot.yaml flywheel_rpm` 1700→**2400**。舊 YAML 無 `aim` 區時退回無阻力模型（向下相容，有測試）。
- **為什麼**：舊假設在 6 m 上排/加分孔打不到；無阻力解差 2.7–3.5°（v0 10 時 8–14°，超過校正上限 ±5°）。
  新增 `decision/ballistics.DragAimModel`（向量化 RK4 查表，反解 0.02 ms，與真值積分器差 < 0.01 cm、無阻力極限差 1e-4°）。
- **模擬誠實度修正**：校正靈敏度改用腦內模型估（舊版用真實世界＝偷看答案）；感知誤差拆成「每場每孔系統偏差 + 逐幀雜訊」；
  新增賽前練習暖機 `practice_calibration`、`AdaptiveCalibrator.save/load`、比賽中凍結校正 `learn=False`。
- **YOLO 重訓**：2000 張 2.5–7.5 m 合成資料（舊 2.0–3.2 m）、yolo11n、80 epochs、imgsz 960、3.5 h →
  驗證 mAP50 三類皆 0.995；6 m 端到端 PnP 60/60、亮孔 50/50、距離中位 0.2 cm、方位 0.01°（合成影像）。
- **6 m 重評估**（`tools/reassess_6m.py`，各 60 場；全文 docs/six_meter_reassessment.md、原始 docs/results/）：
  校正 OFF 13.6 分/25% → ON 47.3/85% → **ON + 練習 9 球 48.6/87%/加分孔 30%**；2.5 m 舊假設 52.1/93%；
  6 m 沿用舊假設 4.8/9%。v0 掃描：10 m/s 只打得到 4/12 孔，14 m/s 48.6 分，16 m/s 50.5 分。
  誤差預算：抖動 0.4°→0.2° 加分孔 30%→48%；v0 抖動 ≤ 0.1 m/s。凍結校正（無落點量測）49.0 分（v0 漂移 1%）；
  漂移 2%/3% 時凍結 45.6/39.5、線上 48.2/47.8。
- **Orin↔MCU（P3）**：協定 v1（ASCII + CRC-8/SMBUS、SEQ 去重、NAK 碼、事件）；`SerialMCU`（重送、STAT 輪詢兼心跳、
  等 BOOT）；`firmware/rotbot_mcu`（梯形速度、飛輪前饋+電壓補償+PI、推球、看門狗、急停、未 ZERO 不准動）；
  `firmware/host` 韌體模擬器 + `tools/mcu_link_test.py`：**full 21/21、basic 19/19**（RTT 平均 0.25 ms）。
  arduino-cli 編譯 **Uno：13.1 KB（40%）/ 667 B RAM（32%）**，全感測 15.1 KB / 693 B，我們的檔案 0 警告。
  過程抓到並修正：pyserial `read(256)` 每次多 50 ms（改先讀 1 byte）、飛輪 PI 起轉 windup 過衝 + READY 閃爍
  （改 ±5% 內積分 + 到速遲滯）、電池下降 15% 時積分區外卡住（加電壓前饋補償）、轉速計改量週期（2.5%→0.02%）、
  `config.h` 群組 `#ifndef` 被 `-D` 覆寫時漏定義。
- **手動操控**：`rotbot/teleop`（aiohttp、WebSocket、MJPEG、鍵盤/搖桿、單一駕駛、60 秒鎖定、300 ms deadman、
  斷線停機）+ `tools/teleop_web.py`。經本機區網 IP（192.168.1.102）實測：瀏覽器↔伺服器 0.41 ms、指令→MCU ACK 0.61 ms。
  搖桿需安全連線（Gamepad API）→ SSH 通道開 localhost 或 HTTPS。規則：禁場館外遙控、場館 5.1–5.9 GHz 攝影機干擾。
- **自主主迴圈**：`pipeline/loop.ControlLoop` + `AutonomousRunner`、`decision/frames` 轉塔運動學（`camera_extrinsic_at`）、
  `sim/twin.py` 數位孿生、`perception/camera.SharedCamera`；`tools/run_autonomous.py --sim` 整場 27 球、50 分、24/27、
  1.33 s/球（YOLO 版 1.45 s/球）；**比賽主程式 `tools/robot_main.py`**（網頁開始 → 60 秒 AI → 切手動），整合測試通過。
- **省時工具**：`tools/fit_ballistics.py`（打牆擬合；自我驗證 2/4/6 m × 27 球 → v0 13.22±0.37（真 13.44）、Cd 0.52±0.11；
  只打 3/6 m 時 v0 與 Cd 相關 1.00 會飄走 → 加 Cd 先驗 + 三距離）、`tools/bench_pipeline.py`（PC 13.6 ms/幀）、
  `tools/capture_dataset.py`。CI 新增 Uno 編譯 job。
- 文件：`docs/six_meter_reassessment.md`、`docs/mcu_orin_link.md`、`docs/teleop_network.md`、`docs/hardware_bringup_plan.md`、
  `firmware/README.md`；README / ROADMAP / models/README 更新。
- 測試：37 → **57 passed**。

### 2026-10-07（續 4）：整理要跟硬體組確認的事項

- **新增 `docs/hardware_team_questions.md`**（只有文件，沒有改程式或設定，未跑測試）：軟體端整合前需要硬體組回覆的 8 項。
  - 優先三項會決定「6 mm 視角認錯孔」怎麼解：投球位置（距離能否 ≥ 3 m、左右是否固定、底盤是否移動）、
    轉塔角度回報與歸零、相機安裝與 CSI 排線走線。
  - 其餘：發射口高度／出射速度等實測值、加買 3.5–4 mm 鏡頭、不能補光（規則第 9 條）、Jetson 供電與固定、
    UART 協定（目前 `protocol.py` 沒有角度回報）與整合時程。
- **為什麼**：孔編號指派的改寫要用到機器人位置與轉塔角度這兩個輸入，有沒有、準不準要先問清楚；
  `launch_h`、出射速度、`camera_mount` 外參也都卡在實機量測。
- **待辦**：收到回覆後更新 `configs/field.yaml`／`configs/robot.yaml` 的對應數值，並決定孔編號改寫要依賴哪些輸入。

### 2026-10-07（續 3）：實機相機標定完成 + 去畸變；發現 6mm 視角讓定位出錯

- **實機標定（非模擬）**，同一天拍了三輪：
  1. 手拿一張紙：RMS 1.22 px、主點 cy 偏 200 px、隨機子集合的 fx 在 1596–2386 之間跳 → **紙是彎的，作廢**。
  2. 紙貼在紙箱上、距離 0.52–1.20 m：RMS 0.93 px，fx 1907、主點 (758, 582)。
  3. 同樣貼紙箱、距離 1.32–1.76 m（板寬只佔畫面 15–20%）：RMS 0.45 px，但 fx 2040、主點 (847, 483)。
  - **交叉驗證**：第 2 輪的參數解釋第 3 輪的照片 0.47 px（與第 3 輪自己擬合的 0.45 一樣好）；反過來 1.04 px。
    → 第 3 輪拍太遠，每張約束少，誤差小但參數定不準。**RMS 低不等於參數準。**
  - **採用兩輪合併**（40 張取 39 張）：RMS **0.71 px**；原始 fx 1922.6 / fy 1924.0、主點 (777.7, 578.9)、
    k1 −0.508、k2 +0.269、p1 −0.0016、p2 +0.0001。隨機 20/20 分割 8 次：fx 1911 ± 23（1.2%）、cx ± 27、cy ± 12 px，
    留出照片 RMS 0.75 px。加 k3 或 rational 模型對留出誤差沒有幫助。
  - 鏡頭實際視角（含畸變）**44.6° × 32.9°** → 確認是標稱 6mm。**桶狀畸變很大：四角位移 129 px**。
- **相機輸出去畸變影像**（`rotbot/perception/imx296.py`）：有 `camera_calib` 時 `read()` 回傳去畸變後的影像、
  `intrinsics()` 回傳其針孔內參（fx=fy=1794.1、主點同上、視角 44.2°×33.7°），下游不需要知道有畸變。
  另有 `read_distorted()`（標定用）。合成測試：帶畸變投影的點去畸變後落在針孔位置上 < 1 px。
  實機照片：棋盤格偏離理想針孔投影的程度 1.48 → 0.92 px（第 2 輪照片）。
  - 內插預設最近鄰（**2.8 ms**）；雙線性在這台要 18 ms（單執行緒 94 ms，OpenCV 內部已平行化，自己再分帶反而變慢到 28 ms）。
  - 實機 `read()`（ISP + 去畸變）**47–50 fps**（每張 20–21 ms）；未去畸變時 59 fps。
- **設定**：新增 `camera_calib:` 區段（`CameraCalibration`：解析度、原始內參、畸變係數、輸出焦距）與
  `camera_hw.undistort_interp`。**`camera:` 沒有改成實機內參**，理由見下一點。
- **★ 發現：6mm 的視角讓現有定位出錯**。把 `camera:` 換成實機內參後 `test_full_chain_random_views_accurate` 失敗
  （距離誤差中位數 17 cm，門檻 3 cm）。隔離變因：解析度與主點偏移都沒影響，**只有視角有影響**。
  同一測試改用各焦距（完美偵測、隨機視角 2.0–3.2 m、各 120 場景；**合成評估**），「有孔定位偏差 > 10 cm」的場景比例：
  3.5mm 8%、4mm 23%、5mm 62%、6mm **91%**（平均看到的孔數 12 / 10 / 7 / 6）。
  原因：孔編號指派假設大部分板面在畫面內，只看到部分時會認錯欄/列。→ 待決定：換 3.5–4mm 鏡頭，和/或改寫孔編號指派。
- **標定工具改進**：板子在畫面上太小（< 畫面寬度 1/5）的畫面拒收並提示「太遠」；
  新增 `--resolve DIR...` 合併多次標定重新計算；輸出改為單一 `camera_calib:` 區塊。
- 文件：`docs/imx296_jetson.md`（標定結果、去畸變、6mm 與定位）、`Parts purchasing/Camera/IMX296_Lens_Requirements.md`
  （實測視角與畸變、各焦距定位可靠度表、建議改 3.5–4mm）。
- 測試：全套 **54 passed、1 skipped**（`python3 -m pytest`）；`ROTBOT_HW=1` 時 `test_imx296.py` 13 passed。

### 2026-10-07（續 2）：相機內參標定工具

- **標定演算法**（新檔 `rotbot/perception/intrinsics.py`，不碰硬體）：`find_corners`（半解析度找棋盤格 + 原圖次像素精修）、
  `ViewCollector`（只收「拿穩、跟已收的不同」的畫面，並統計角點涵蓋畫面的比例）、
  `solve_intrinsics`（`cv2.calibrateCamera`，固定 k3，剔除誤差過大的照片後重算）、`make_board_pdf`（A4 棋盤格）。
- **操作工具**（新檔 `tools/calibrate_camera.py`）：開相機 + 網頁預覽（`http://localhost:8081`），自動收集 20 張後計算，
  輸出可直接貼進 `field.yaml` 的 `camera:` 區塊、換算焦距、視角、畸變在四角的位移。
  做成網頁是因為 Jetson 是透過 SSH 遠端操作、沒有螢幕視窗可用。
- 可列印的棋盤格放在 `docs/checkerboard_A4.pdf`（9×6 內角點、25 mm 格）；操作步驟寫進 `docs/imx296_jetson.md`。
- **驗證（合成資料，非實機標定結果）**：
  - 單元測試：由已知內參 + 畸變產生角點（加 0.1 px 雜訊）→ 解回 fx/fy 誤差 < 1%、主點 < 10 px、RMS < 0.3 px；
    渲染棋盤格影像 → 角點偵測誤差 < 1 px；混入一張壞照片會被剔除。
  - 整個工具用「模擬相機」（渲染棋盤格、每個位姿停 4 張）端到端跑過：6 秒收滿 20 張、涵蓋 96%，
    解出 fx 1738.7 / fy 1740.7 / cx 734.7 / cy 540.0（真值 1739 / 1741 / 735 / 540），RMS 0.04 px。
  - 過程中修正：「新角度」門檻原本設角點平均位移 6% 畫面對角線，模擬操作收到 13 張就再也收不進去
    （板子須整張在畫面內，可移動範圍有限）→ 改 3%。
  - 實機只驗證到「工具能開相機、預覽 39.7 fps、沒有棋盤格時正確回報找不到、Ctrl+C 正常釋放相機」；
    偵測速度在實機影像上 9.4 ms（無棋盤格）。**真的棋盤格還沒拍過，內參仍未標定。**
- `imx296.py` 新增常數 `PIXEL_PITCH_MM`（由 fx 換算鏡頭焦距用）。
- 測試：新增 `tests/test_intrinsics.py` 5 個；全套 **51 passed、1 skipped**（`python3 -m pytest`），
  `ROTBOT_HW=1` 時 `test_imx296.py` 10 passed。

### 2026-10-07（續）

- **`.gitignore` 移除 `rotbot` 那一行**（9/15 加入）。它讓 `rotbot/` 底下**新增**的檔案不出現在 `git status`
  （`rotbot/perception/imx296.py` 因此一度被忽略）。`rotbot/sim/assets/` 另有專屬規則，仍然忽略。
- **鏡頭需求文件改寫為 IMX296 版**：`Parts purchasing/Camera/e-CAM25_CUONX_Lens_Requirements.md`
  → 改名 `IMX296_Lens_Requirements.md` 並整份重算（C/CS-mount、像圈 ≥ 1/2.9"、相機內建 IR-cut）。
  舊版的「距離 7m、九宮格 5×3m、球速 18.3m/s」與規則書（檔板距網框約 250cm）及 `field.yaml` 不符，不再採用。
- **視角重新計算**（用 `Config.hole_centers()` 的實際孔位、相機在發射口高度仰視、針孔模型、四邊留 40px；**推算，非實測**）：
  完整入鏡孔數 6mm：2.5m → 7/12、3.0m → 12/12；5mm 與 4mm：2.5m 即 12/12；8mm：2.5m → 3/12。
  **更正**：前一則快照寫「需約 48°×44°、焦距 ≤ 4.7mm」是用正對板面粗估的，高估了垂直需求；以本條為準。
- **實測像圈**（目前的 6mm 鏡頭）：四角亮度約為中心的 88%，無暗角 → 涵蓋 1/2.9"（對角 6.27mm）沒問題。
- **實測短曝光的進光量**（6mm 光圈全開、室內辦公室燈光）：8333µs/0dB 畫面平均亮度 115/255；
  250µs/18dB 只有 62/255 → 這種燈光下 1/4000s 約需 30dB，雜訊會很重。比賽場地照明待現場量。
- 本則只改文件與 `.gitignore`，程式碼未變動，測試結果同上一則（46 passed、1 skipped，替代執行器）。
- **補：Jetson 裝了 pytest 6.2.5 後用真的 pytest 重跑**：`python3 -m pytest -q` → **46 passed、1 skipped**
  （與替代執行器結果一致）；`ROTBOT_HW=1` 時 `tests/test_imx296.py` 10 passed（含實機煙霧測試）。
  pytest 6 不認得 `pyproject.toml` 的 `pythonpath` 設定（只會多一行警告），在這台要用 `python3 -m pytest` 執行。

### 2026-10-07

- **★ 第一個實體硬體到位：機上相機（P5 開始）**。Raspberry Pi Global Shutter Camera（Sony IMX296，
  1456×1088@60fps、global shutter）接上 Jetson Orin Nano Super 的 CAM1。取代原規劃的 e-CAM25_CUONX（AR0234）。
  驅動是另外編的 kernel module + device-tree overlay（不在本 repo），建置方式與注意事項見新檔
  `docs/imx296_jetson.md`。
- **相機 HAL 實作**（新檔 `rotbot/perception/imx296.py`）：
  - `V4L2Capture`：用 ioctl + mmap 直接從 `/dev/video0` 取 raw Bayer（RG10），不經 Argus、不依賴額外套件。
    `read()` 一律回傳最新一張，處理較慢時丟舊張不累積延遲。
  - `BayerISP`：黑階 → 白平衡 → 去馬賽克 → 色彩矩陣（Raspberry Pi 標定資料）→ sRGB，輸出 BGR uint8；
    切 4 條水平帶平行處理。純 numpy/OpenCV，不碰硬體，可單元測試。
  - `IMX296Camera(hal.Camera)`：曝光/增益可在執行中改；另提供 `read_raw()`、`timestamp`、`grey_world_gains`。
  - `perception.camera.make_camera(cfg)`：依設定建立實體相機（`imx296` / `webcam`）。
- **設定**：`configs/field.yaml` 新增 `camera_hw:` 區段（裝置、曝光、增益、白平衡、是否套色彩矩陣），
  型別化為 `CameraHardwareConfig`；區段可省略（舊設定檔照常載入）。**沒有動 `camera:` 內參**
  （仍是 1920×1080 範例值，合成資料與既有模型不受影響）。
- **工具**：`tools/camera_check.py`（拍一張、量幀率與亮度、`--auto-wb` 量白平衡）。
- **實機量測**（Jetson Orin Nano Super、L4T R36.4.4、25W；**非模擬**）：
  - raw 取像 **60.4 fps、120 張零丟張**；含 ISP 的 `read()` **59.2 fps（每張 16.9 ms）**。
  - ISP 單獨耗時：含色彩矩陣 **10.7 ms**（優化前單執行緒 48 ms）、不含 **3.4 ms**；輸出與浮點參考實作最多差 1 個灰階。
  - 相鄰兩張時間戳固定差 **16.56 ms**（抖動 < 0.001 ms）；暫停 1 秒後下一次讀取 26 ms 內拿到新張（時間戳前進 1.027 s）。
  - 曝光/增益寫入後直接讀 sensor 暫存器確認生效（8333µs→SHS1=0x22d、4000µs/6dB→SHS1=0x351/GAIN=0x3c）。
  - 鏡頭對焦後從 raw 量到：邊緣 10–90% 寬度約 3.4 px、平坦區訊雜比約 33（0 dB、8333 µs、室內燈）。
- **測試**：新增 `tests/test_imx296.py` 9 個（ISP 顏色/飽和/接縫、白平衡估計、設定相容、內參防呆、V4L2 結構大小）
  + 1 個實機煙霧測試（`ROTBOT_HW=1` 才跑）。全套 **46 passed、1 skipped**；設 `ROTBOT_HW=1` 時 47 passed。
  ⚠ 這台 Jetson 沒有裝 pytest/pip，以上是用**替代執行器**（逐一呼叫 test 函式）跑的，不是真的 `pytest`；
  改動前同一方法的基準是 37 passed。
- **還不能做的事**：`IMX296Camera.intrinsics()` 目前會拒絕回傳（內參未標定、解析度與設定不符）→ PnP 定位還不能在實機跑。
  這台 Jetson 也還沒有 `ultralytics`/`torch`/`pybullet` 與模型權重，YOLO 尚未在實機影像上試過。

### 2026-09-09（追趕補記：8/19 之後的變更）

- **webcam 即時測距工具**（`tools/webcam_ranger.py`）：筆電鏡頭手拿已知直徑的圓（比賽 40/20cm），
  即時算距離/方位/俯仰/傾角；重用 `perception.pose` 的 `distance_from_diameter`（方法A）與
  `pose_from_ellipse`（方法B）→ 與機器人同一套幾何。合成圓驗證：FOV60@1280，40cm@242px→1.83m（理論1.85）。
- **PyBullet 網板圓孔平滑化**（`sim/board_mesh.py`）：改「粗背景格 + 孔邊沿真圓弧貼齊」三角化，
  同等圓滑度三角形由 ~18.9萬 降到 ~1.0萬（載入/記憶體/GUI 大勝）；孔圓滑度由 `arc_seg` 控、與格子脫鉤。
- **亮孔改紅色 LED 燈環**（`sim/board_mesh.py` `ring_obj_path` + `sim/pybullet_world.py`）：
  由「半透明紅圓盤（整格發紅）」改為「沿孔緣一圈紅環」，符合官方 Q33（網框邊緣紅色 LED 燈條）。
- **tools 支援直接路徑執行**（`teleop_sim.py`/`webcam_ranger.py`）：`python tools/xxx.py` 也可用（補 sys.path）。
- **teleop 錄影**（`--record out.mp4`）：PyBullet 3D 視窗 + 機器人相機視窗各存一份 mp4。
- **teleop 掃視自動框整板**（`_survey_pose`）：自動掃描瞄準高度使 12 孔全入框（先前對準主格中心會切到底/加分排）。
- **teleop 偵測疊圖 + 資料視窗**：每孔畫投影橢圓（型變）+ 距離標籤（亮孔紅）；另開「Detections」視窗列
  距離/方位/俯仰/傾角/型變（短長軸比）。`--no-data` 可關。
- **物理×校正實驗**（`sim/closed_loop.monte_carlo`，各 60 場）：
  - 校正 ON vs OFF：總分 43.2→**51.0**、命中率 80%→**91%**、加分孔命中 1%→**27%**。
  - 逐球收斂：校正約 4~6 球把誤差壓到 ~5cm 並穩住。
  - 穩健度（命中率 OFF→ON）：2×安裝偏差 19%→91%、1.5×阻力 69%→91%、2×逐發抖動 68%→83%
    → 校正能吸收**系統性**誤差，但**隨機抖動**只能部分改善（機構重複精度是硬體責任）。
- **參考文件**（`docs/software_architecture_and_experiments.md`）：field.yaml 參數來源分類、實驗表、
  架構與資料流、各模組邏輯、函式庫、假設、硬體前提。
- **工作準則**（`CLAUDE.md`）：新增「每次變更都要 append PROGRESS §9」等規則。
- 全程 37 tests passed。

### 2026-08-19

- **官方確認加分孔亮燈機制（賽事 Q&A Q33/Q34）**——解決 §7 兩項長期未定風險：
  - **顏色＝紅**：指定得分格用**紅色 LED 燈條**（寬約 14mm、設於網框邊緣內約 5cm、光通量 475±25lm、有線開關控制）。推翻先前「暫定綠色」。
  - **時窗＝僅前 60 秒**：哨響於 3 個加分孔**隨機亮 1 格、固定 60 秒、之後熄滅**（期間不換格）。推翻先前口述「整場一直亮」。→ 加分孔只有前 60 秒（＝強制自主窗口）能得分。
- **對應程式修正**：
  - (A) `rotbot/sim/scene.py` 亮孔由**綠色填滿**改為**紅色 LED 燈環 + 光暈 + 紅色域隨機化**；`rotbot/sim/pybullet_world.py` 亮孔 marker 綠→紅。
  - (C) `rotbot/decision/strategy.py` `ShotPlanner` 新增 **60 秒加分窗口**建模（`bonus_window_s`、以已投球數×每球均時估流逝，或可傳入牆鐘 `elapsed_s`）：窗口內加分最優先、窗口後不再選加分孔。`plan_next_shot` 增 `elapsed_s` 參數。
  - (B) **完成**：重生成 2000 張合成資料（紅燈環，train1600/val400）+ 重訓 yolo11n（100 epochs、imgsz960、RTX4050、2.28h）。
    驗證集 per-class mAP50：main 0.995、bonus 0.985、**bonus_lit（紅環）0.985**（R 0.982）；best.pt 已覆蓋 `models/grid_holes_yolo11n.pt`。
    端到端評估（50 場）：整板 PnP 44/50、**亮孔正確帶出 8/8**、距離誤差平均 **4.5cm**、方位 1.47°、俯仰 0.55°（與改色前相當，紅燈辨識成立）。
  - (D) 本條目。全測試 37 passed。
- **來源**：賽事說明會/線上 Q&A 截圖（使用者提供）。

### 2026-08-18

- **基礎模型定版（階段 A）**：以 `models/grid_holes_yolo11n.pt` 跑端到端感知評估（50 場合成）：
  整板 PnP **46/50**、亮孔正確帶出 **7/7**、距離誤差平均 **4.9cm**（p95 16.4）、方位 **1.20°**、俯仰 **0.44°**；
  偵測 489 vs 可見 439（略微過偵測，屬日後調 conf）。定為**基準模型**，模型卡見 `models/README.md`。
- **PyBullet 真實物理沙盒（階段 B，新檔）**：`tools/teleop_sim.py` +
  `rotbot/sim/{pybullet_world,pybullet_camera,board_mesh,robot_spec}.py` + `configs/robot.yaml`。
  - **機器人建模**：依規則尺寸上限（50×50×60cm）+ CAD 比例反推幾何（底盤 0.45×0.45、雙飛輪 Ø12cm/v0≈10m/s、
    轉塔 yaw 整圈 + pitch 0–90°、彈倉每倉4球、同軸相機）；多為推論值，標「待實測」於 `robot.yaml`。
  - **物理**：球為剛體受重力 + 平方空氣阻力（沿用 `cfg.k_drag`）；程序化「有洞網板」網格使穿孔/撞板反彈；
    計分沿用 x=dist_x 平面解析法，與 `sim.world`/`sim.closed_loop` 一致（物理可比、校正仍有意義）。
  - **互動**：手動（pitch/yaw/v0 滑桿 + SHOOT）⇄ 自動（掃視相機→感知→決策→投球→線上校正「找手感」）一鍵切換；
    另開「Robot camera」視窗顯示機器人同軸視角。自動感知來源可選 `truth`（幾何真值，穩）或 `yolo`（真模型，有域差）。
  - **驗證**：DIRECT 煙霧測試（drag off、aim==v0）→ **8/8 命中**、策略正確（亮孔×2 先滿 3 顆再投最近一般孔）；
    相機渲染 1080p 正常。`launch_h` 佔位 0.50→**0.40**（依 CAD 反推）。
- **引擎定位**：PyBullet 當單人快速迭代的物理/控制沙盒；日後可將**渲染**移至 Blender（照片級合成資料、縮小 YOLO 域差）
  或 Unity（即時擬真孿生）——因幾何集中於 `field.yaml`/`robot.yaml`、感知/決策不綁引擎，屬「換後端」非重寫。
- **機器人規格來源**：`data/嘻諧們_參賽計劃書_0629.pdf` + `data/嘻諧們.pdf`(CAD) + `data/2026_rule-1.pdf`(尺寸上限)。

### 2026-08-14

- **投球 demo 影片工具**（`3385da0`，`tools/demo_match_video.py`）：閉迴路一場的視覺化影片，供進度報告成果。
  `--mode truth`＝板面落點視圖；`--mode render`＝相機+YOLO ｜ 板面 分割視圖。板面顯示圓孔/亮孔x2/目標/
  落點(綠命中紅未中)/比分/命中率，逐球累積可見「找手感」落點收斂。輸出於 `runs/demo/`（大檔不入 git）。
- **P4 強化：真 render 影像 + 真 YOLO 版閉迴路**（`f421e82`）：`run_closed_loop_render`（`--render`）每球渲染
  相機影像→真 YoloDetector→localize→決策→物理→計分→校正。幾何一致性正解——外參 (R_cl,t_cl)
  由場景真實板位姿反算，使網板成為 x=dist_x 垂直平面，感知與計分真值完全一致（無尤拉角近似偏差）；
  渲染視角只決定影像長相、與世界解耦。結果：真 YOLO 單場 **總分52/命中93%/加分33%**（與 truth+noise 版相當）。
  （一場約 30s：每球 render 大圖 + YOLO；不進 CI，快速回歸仍用 truth+noise 版。）
- **P4 模擬閉迴路核心完成**（`sim/closed_loop.py` + `tools/run_match.py`）：串起 感知(真值+雜訊)→決策→
  致動(AimCommand→MockArduino)→物理(BallisticWorld)→計分→線上校正，跑完整場 27 球；每發驗證走過通訊路徑。
  蒙地卡羅（30 場）：**校正後 總分51/命中92%/加分32% vs 未校正 45/83%/4%**——校正對 Ø20 加分孔提升最顯著
  （4%→32%）。測試 +2（全套 **37 passed**）。單場可見「找手感」：首發亮孔誤差 14.7cm 未中，校正後第 2–3 球即命中。
- **P2 決策層核心完成**（`9944d2a`）：`decision/frames.py` 相機系量測→發射點系 3D 目標
  （軸置換 + 相機外參 `camera_mount`，外參待 P5 標定）；`ballistics.aim_at_point` 對任意 3D 目標
  解俯仰/水平；`strategy.ShotPlanner` + `plan_next_shot`（亮孔優先上限3球、一般孔每孔上限3球、
  未亮加分孔不投、27 顆排程、射程過濾）。測試 +9（全套 **35 passed**）。驗證 P1→P2 串接：前3球投亮孔、後續改投最近一般孔。
- **模型權重集中到 `models/`**（`c3f5c9d`）：預留多模型；`*.pt` 不入 git + README 命名慣例；
  eval/demo 預設讀 `models/`、訓練後自動複製一份。
- **場地間距改用官方尺寸標示圖實值**（`3604c94`）：列距 0.63→**0.60**、加分排高度差 0.49→**0.50**、
  欄距 0.90 與孔徑 40/20 不變（公差 ±2cm）。`dist_x` / `grid_cy` / `launch_h` 該圖未標，仍待實測。
- **整板 PnP 由 IPPE 改 SQPNP**（`3cc2c37`）：修正正對 (fronto-parallel) 平面退化。端到端指標：
  - 整板 PnP 成功 45/50 → **46/50**
  - 距離誤差平均 8.3cm → **4.9cm**（95pct 27.2cm → **16.4cm**）
  - 方位誤差 95pct 12.9° → **1.19°**
- **感知視覺 demo（瞄準 HUD）**（`fc86af1`）：掃描影片、亮孔標「TARGET x2」；場景加 `rim_jitter`（中性孔緣避免顏色混淆）。
- **新增本進度報告**（`49b2bcd`）。
- **規則確認（使用者）**：指定得分格**整場一直亮**到當次比賽結束（上限 3 球 / 12 分）。
- **開放風險**：亮孔顏色未定——合成資料暫畫綠色，隊員記得為紅色；答案在官方「五月中旬亮燈機制說明」文件，待取得。

### 2026-08-13

- **★ 目標形狀由「方形」改「圓孔」**（`05b4ecb`）：依官方工程圖 + 計劃書修正；重寫 config/scene/detector/gen_data/pose，
  座標佈局集中至 `Config.hole_centers()`；新增兩種位姿法（A 整板 PnP / B 橢圓圓錐）與比較工具。
- **端到端首次跑通（當時用 IPPE）**（`9f0fdce`）：整板 PnP 45/50、距離平均 **8.3cm**、方位 1.58°、俯仰 0.64°。
- **YOLO 圓孔偵測 baseline**：驗證集 mAP50 一般孔 0.995 / 加分孔 0.973 / 亮起加分孔 0.954。
- **感知全鏈定位 `localize.py` + 完美偵測器替身**（`e869fe2`）；後續加入穩健孔對應（投影重匹配）修正孔編號誤配。
- **P1-1～P1-4**：合成場景 `scene.py`、資料集工具、訓練腳本、`detector.py`。
- **修正**：訓練系統 RAM OOM（workers 降為 2 + cache off）、ultralytics 輸出外洩（改絕對路徑）。
- **P0 專案骨架**（`36668bb`）：三層架構 + HAL + 集中式設定 + 原型模組化 + CI。
