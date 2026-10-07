# Rotbot 軟體進度報告

> AI 自主棒球投手機器人 — 軟體端｜嘻諧們｜2026 TEL 東京威力科創機器人大賽
> 報告日期：2026-08-14｜負責：軟體（單人）｜比賽日：2026-11-21

---

## 0. 一句話摘要

已完成**軟體骨架（P0）**與**感知系統（P1）**：從一張相機影像，能用訓練好的 YOLO 模型
偵測九宮格圓孔、鎖定亮起的加分孔、並算出每個孔相對機器人的**距離與方位角**——
整條感知鏈在合成環境中端到端跑通、有測試、有視覺 demo。目前無實體硬體，全程以
**模擬 + 硬體抽象層**開發，硬體到位後可直接抽換底層。

| 里程碑 | 狀態 | 重點產出 |
|---|---|---|
| P0 專案骨架 + 硬體抽象層 | ✅ 完成 | 三層架構、HAL、集中式設定、原型模組化、CI |
| P1 感知（圓孔偵測 + 定位） | ✅ 完成 | 合成資料、YOLO 訓練、整板 PnP 定位、兩種位姿法比較、視覺 demo |
| P2 決策（圓孔版） | ✅ 核心完成 | 座標轉換、3D 瞄準反解、加分優先投球策略（+ 測試、P1→P2 串接驗證）|
| P3 致動 / 通訊 | ⏳ | UART 協定、賽制狀態機、安全停止 |
| P4 模擬閉迴路 | ✅ 核心完成 | 感知→決策→致動→物理→計分→校正 全鏈跑完整場，校正提升成績 |
| 互動物理沙盒（PyBullet） | ✅ 核心完成 | 依機器人 CAD/規則建模、真實剛體物理、手動⇄YOLO自動、機器人相機視窗 |
| P5 硬體整合 / P6 賽前 | ▶ P5 進行中（相機已到位，10/07）| 相機 HAL 已在實機取像；待：相機標定、Orin 部署、實機校正 |

規模：Python ~2,300 行、26 個自動化測試（全綠）、13 個開發 commit。

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

## 6. 量化成果（合成環境）

**偵測模型**（yolo11n，驗證集 mAP50）：一般孔 **0.995**、加分孔 0.973、亮起加分孔 0.954。

**兩種位姿法比較**（60 場、注入偵測雜訊）：

| 方法 | 距離誤差 | 方位/俯仰 | 傾角 | 適用 |
|---|---|---|---|---|
| A-整板 PnP | 0.2–0.5 cm | ~0.06° | ✗ | 主力（需 ≥4 孔）|
| A-尺寸測距 | ~6 cm 系統性 | — | ✗ | <4 孔備援 |
| B-橢圓圓錐 | 0–2 cm | ~0.08° | 0.6–1.8° | 補單孔傾角 |

**端到端**（真模型，50 場，整板 PnP 用 SQPNP）：整板 PnP 成功 **46/50**、亮孔正確鎖定 **7/7**、
距離誤差平均 **4.9 cm**（95pct 16.4cm）、方位 **1.20°**、俯仰 0.44°。
（一般孔容許球心誤差約 ±16.5cm，此精度足以命中；系統性偏差由線上校正吸收。）

---

## 7. 待確認事項與風險

| 項目 | 現況 | 需要的動作 |
|---|---|---|
| ✅ **亮孔顏色** | **官方 Q33 確認為紅色** LED 燈條（寬約14mm、網框邊緣內約5cm、~475±25lm、有線開關）。（先前暫定綠色）| 已改 `scene.py` 綠→**紅色燈環 + 紅色域隨機化**、`pybullet_world` marker 改紅；**待重生成合成資料 + 重訓 lit 類別** |
| ✅ 加分規則時窗 | **官方 Q34 確認**：哨響於 3 格**隨機亮 1 格、固定 60 秒後熄滅** → **加分孔只有前 60 秒能得分**。（先前口述「整場一直亮」為誤）| 已於 `ShotPlanner` 建模 60 秒窗口：窗口內優先加分、窗口後不再選加分孔 |
| 場地間距（孔徑/欄距/列距） | ✅ 已用**官方尺寸標示圖**實值：欄距0.90、列距0.60、加分排0.50（公差±2cm）| 完成 |
| dist_x / grid_cy / launch_h | 官方孔位圖未標（屬場地擺放+機器人幾何），仍用暫估 | 現場實測 / 依機器人設計確認 |
| 邊緣被切的孔 | 其 bbox 中心偏差，會拉高 PnP 誤差 | PnP 求解**排除碰到畫面邊緣**的偵測（只顯示不定位）|
| 精度精修 | 距離平均 8.3cm，主因 bbox 中心≠透視圓心 | 需要更高精度時改用**橢圓中心**（順帶取得傾角）|
| 合成→真實落差 | 目前全合成、無真實影像 | 硬體到位後用少量實拍微調（P5）|
| **相機內參未標定** | 實機相機是 IMX296 1456×1088，`field.yaml` 的 `camera:` 仍是 1920×1080 範例值；`intrinsics()` 會拒絕回傳。標定工具已備妥（`tools/calibrate_camera.py`），尚未實拍 | 印 `docs/checkerboard_A4.pdf` → 跑標定 → 覆蓋 `camera:`；之後決定畸變要不要處理、合成資料/模型是否改用新內參 |
| **鏡頭視角** | 目前裝 6mm（45°×35°）。依 `field.yaml` 幾何推算（未實測）：相機離網框 2.5m 時一次只看得到 **7/12 孔**（3 個加分孔都在）、3.0m 以上才看得到全部；4–5mm 在 2.5m 就能看到全部。像圈已實測足夠（四角亮度約中心 88%）| 量相機到網框的實際距離；決定留 6mm（靠轉塔掃視）或換 4–5mm。詳見 `Parts purchasing/Camera/IMX296_Lens_Requirements.md` |
| Jetson 執行環境 | 機上有 numpy 1.21 / OpenCV 4.8 / pytest 6.2.5；沒有 ultralytics、torch、pybullet、模型權重 | 建環境（Jetson 版 PyTorch 或 TensorRT）、把權重搬上去 |

---

## 8. 下一步

1. **P3 致動/通訊**：UART 協定（含 checksum/ack/逾時）、賽制狀態機（前 60 秒自主、安全停止、失效處理）。
2. **P4 強化**：感知改接「真 YOLO + render 影像」的重版閉迴路；各階段延遲量測（估 Orin 預算）。
3. **P5 硬體整合（相機已可取像）**：相機標定 → 覆蓋 `camera:` 內參、`camera_mount` 外參標定、
   Orin/TensorRT 部署、用實拍影像驗證/微調 YOLO、實機校正表。
4. 併行處理 §7 待確認事項（尤其**亮孔顏色**與 **dist_x/grid_cy 實測**）。

> 完整階段規劃見 [ROADMAP.md](ROADMAP.md)；模組職責與快速試跑見 [README.md](README.md)。

---

## 9. 變更紀錄 (Changelog)

> **append-only**：每次更新只**新增**條目、不改舊條目，保留數字與決策的演進供工作日誌追蹤。
> 上方 §0–§8 為「最新快照」；本區為「歷史紀錄」。新到舊排列。

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
