# Rotbot 軟體進度報告

> AI 自主棒球投手機器人 — 軟體端｜嘻諧們｜2026 TEL 東京威力科創機器人大賽
> 報告日期：2026-10-25｜負責：軟體（單人）｜成果報告書：11/7–11/9（通訊硬體之後凍結）｜比賽日：2026-11-21

---

## 0. 一句話摘要

投球距離改為 **6 m** 後重算全部誤差與假設：舊設計（v0 10 m/s、無阻力瞄準）在 6 m 打不到上排，
已改為**含阻力瞄準模型 + 飛輪 ≥ 14 m/s 設計點**，模擬成績回到 48.6 分 / 命中 87%（2.5 m 時 52 分）。
同時補齊硬體到位前的所有軟體：**Orin↔Arduino 協定 + 函式庫 + Uno 韌體**（同一份韌體在電腦上模擬，21/21 測試通過）、
**路由器區網的網頁手動操控**（鍵盤/搖桿）、**自主主迴圈與比賽主程式**（網頁按開始 → 60 秒 AI → 切手動）。
整條「看→算→轉→投」已在數位孿生（真韌體 + 合成相機 + 彈道世界）跑完整場；剩下的都是需要實機的驗證。

| 里程碑 | 狀態 | 重點產出 |
|---|---|---|
| P0 專案骨架 + 硬體抽象層 | ✅ 完成 | 三層架構、HAL、集中式設定、原型模組化、CI |
| P1 感知（圓孔偵測 + 定位） | ✅ 完成 | 合成資料（2.5–7.5 m）、YOLO 重訓、整板 PnP、兩種位姿法 |
| P2 決策（圓孔版） | ✅ 完成 | 座標轉換、**含阻力瞄準**、轉塔運動學、加分優先策略、校正存檔 |
| P3 致動 / 通訊 | ✅ 核心完成 | 協定 v1、SerialMCU、Uno 韌體、韌體模擬器、狀態機、看門狗/急停 |
| P4 模擬閉迴路 / 自主主迴圈 | ✅ 完成 | 蒙地卡羅、render+YOLO 閉迴路、數位孿生整場、比賽主程式 |
| 手動操控（網頁） | ✅ 完成 | 路由器區網、鍵盤/搖桿、60 秒鎖定、deadman、MJPEG 串流 |
| 互動物理沙盒（PyBullet） | ✅ 核心完成 | 依 CAD/規則建模、手動⇄YOLO 自動 |
| P5 硬體整合 / P6 賽前 | ⏳（等硬體） | 步驟與工具已備：docs/hardware_bringup_plan.md |

規模：Python ~8,000 行、韌體 C++ ~1,100 行、57 個自動化測試（全綠）。

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

**閉迴路（各 60 場）**：

| 條件 | 總分 | 命中率 | 加分孔 |
|---|---|---|---|
| 2.5 m 舊假設（對照） | 52.1 | 93% | 35% |
| 6 m 沿用舊假設（v0 10、無阻力） | 4.8 | 9% | 0% |
| **6 m 設計點（v0 14、含阻力、校正 + 賽前練習 9 球）** | **48.6** | **87%** | **30%** |
| 同上但比賽中凍結校正（沒有落點量測） | 49.0 | 87% | 34% |
| render 影像 + 真 YOLO（6 場） | 49.3 | 88% | 33% |

**硬體規格（誤差預算）**：v0 ≥ 14 m/s（10 m/s 只打得到 4/12 孔）；轉塔逐發抖動 ≤ 0.2°（0.4° → 加分孔 30%、0.2° → 48%）；
v0 逐發抖動 ≤ 0.1 m/s；練習→比賽轉速漂移 < 1%（否則需要比賽中落點量測）。

**系統（數位孿生 = 真韌體 + 合成相機 + 彈道世界）**：整場 27 球全由韌體推出、50 分、1.33 s/球（規則節拍 6.7 s/球）；
網頁 → MCU 指令 0.6 ms（本機區網）；PC 上感知 + 決策 13.6 ms/幀；韌體 Uno 13.1 KB flash / 667 B RAM。

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
| 相機內參 | 10 月已實拍標定（0.71 px），但數字不在 repo；config 仍是佔位值 | 把 K / 畸變 / 解析度填進 `configs/field.yaml` |
| 合成→真實落差 | 模型全合成資料 | 現在就用等比例孔板 + 紅 LED 收實拍、預標註、微調（`tools/capture_dataset.py`） |
| 落點量測 | 線上校正需要；目前只有模擬（孿生）提供 | 轉速穩在 ±1% 時賽前練習暖機即可；比賽中自動判斷當保險 |
| ⚠️ 無線干擾 | 場館攝影機用 5.1–5.9 GHz、6.5 W；場館無 Wi‑Fi | 練習日測 2.4/5 GHz；**11/9 報告書前**決定是否加 RC 備案（通訊硬體之後凍結） |
| 邊緣被切的孔 | 2.5 m 隨機視角下偶有孔編號配錯（6 m 不會） | 需要時 PnP 排除碰到畫面邊緣的偵測 |

---

## 8. 下一步

詳細步驟與通過標準：[docs/hardware_bringup_plan.md](docs/hardware_bringup_plan.md)。

1. **本週（不必等整機）**：等比例孔板 + 紅 LED 實拍（驗證 6 m 感知、收微調資料）；相機內參入檔；Orin 環境 + TensorRT 延遲；
   Arduino 單板燒錄跑 `mcu_link_test --port`；路由器實測 `teleop_web`。
2. **硬體組決定**：飛輪 ≥ 14 m/s 方案、相機/鏡頭、是否加轉速計/電壓量測/原點開關/急停感測、Wi‑Fi 是否加 RC 備案（11/9 前）。
3. **上桌**：步進方向/步數比、轉塔重複精度、飛輪 v0 與抖動、供彈 100 球、打牆擬合（`fit_ballistics`）。
4. **整機**：`run_autonomous --twin`（真轉塔）→ `robot_main`（真相機），60 秒自主彩排、手動彩排。
5. **練習日**：實拍微調、打牆擬合、存校正暖機檔、參數凍結。

> 完整階段規劃見 [ROADMAP.md](ROADMAP.md)；模組職責與快速試跑見 [README.md](README.md)。

---

## 9. 變更紀錄 (Changelog)

> **append-only**：每次更新只**新增**條目、不改舊條目，保留數字與決策的演進供工作日誌追蹤。
> 上方 §0–§8 為「最新快照」；本區為「歷史紀錄」。新到舊排列。

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
