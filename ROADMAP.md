# Rotbot 軟體 Roadmap

> AI 自主棒球投手機器人 — 軟體端規劃
> 團隊：嘻諧們｜賽事：2026 TEL 東京威力科創機器人大賽
> 軟體負責：單人全包｜時程：3 個月以上｜起始狀態：**硬體尚未到位，先以模擬與離線開發為主**

---

## 0. 核心策略（先讀這段）

因為**硬體還沒到**，而你**一個人**要在 3 個月內做完整條 pipeline，最有效率的路徑是：

1. **先把整條閉迴路在模擬裡跑通** — 合成影像 → 偵測 → 解算位姿 → 決策瞄準 → 通訊指令 → 物理落點 → 誤差回授 → 校正。硬體到位前就能驗證 90% 的邏輯。
2. **用硬體抽象層 (HAL) 把「真硬體」與「模擬」隔開** — 相機、Arduino 都定義成介面 (interface)，模擬與實機各自實作。硬體到了只換底層實作，主邏輯一行不動。
3. **一次只開一條戰線** — 單人開發最大的風險是同時開太多半成品。每個 Phase 有明確「完成定義 (DoD)」才往下走。

三層架構對應到三個模組，全程不變：
`感知 (perception) → 決策 (decision) → 致動 (actuation)`

---

## 1. 建議的 Repo 結構

```
rotbot/
  perception/          # 感知：看場地
    camera.py          #   相機擷取（HAL：SimCamera / RealCamera）
    detector.py        #   YOLO 包裝（Ultralytics → TensorRT）
    pose.py            #   PnP 網框位姿解算（← pnp_frame_pose.py 移進來）
    calib/             #   相機內參檔
  decision/            # 決策：算怎麼投
    ballistics.py      #   拋體模型求解器（目標 3D → 俯仰/水平/飛輪轉速）
    calibration.py     #   線上自適應校正（← sim_adaptive_calibration.py 抽出）
    strategy.py        #   目標選擇（加分格優先、27 顆排程）
  actuation/           # 致動：把指令送出去
    protocol.py        #   UART 指令協定（frame 格式、ack、安全）
    link.py            #   序列通訊（HAL：MockArduino / RealArduino）
  sim/                 # 模擬：沒有硬體時的替身
    world.py           #   物理世界（拋體+空氣阻力，← sim 現有程式）
    scene.py           #   合成場景/影像產生器（訓練與測試用）
  pipeline/
    loop.py            #   主迴圈：perceive → decide → actuate
    state_machine.py   #   賽制狀態機（前 60 秒自主、安全停止…）
  tools/               # 離線工具（資料產生、評估、匯出模型）
  configs/             # 場地/相機/機構參數（YAML，集中管理）
  tests/
```

原則：`perception / decision / actuation` **不 import `sim`**，只依賴介面。`sim` 與未來的 `hardware` 都去實作那些介面。

---

## 2. 里程碑總覽

| Phase | 主題 | 產出 (DoD) | 依賴硬體？ |
|---|---|---|---|
| **P0** | 專案骨架 + HAL | repo 結構、config、CI、介面定義、原型搬進模組 | ❌ |
| **P1** | 感知（離線） | 合成資料管線、YOLO baseline、PnP 位姿模組 | ❌ |
| **P2** | 決策（離線） | 拋體求解器、線上校正、策略模組，全程模擬驗證 | ❌ |
| **P3** | 致動與通訊 | UART 協定、Mock Arduino、賽制狀態機、安全停止 | ❌ |
| **P4** | 系統整合（模擬閉迴路） | 端到端跑完一場 27 顆的模擬，含評估指標 | ❌ |
| **P5** | 硬體到位整合 | 相機標定、Orin/TensorRT 部署、實機通訊、HIL | ✅ |
| **P6** | 賽前強化 | 現場抗干擾、實機校正表、失效保護、實戰演練 | ✅ |

**P0–P4 可以在完全沒有硬體時做完** → 這是你未來數週的主戰場。
**P5–P6 是硬體到位後的清單**，先寫好等硬體。

---

## 3. 各階段工作 TODO

### P0 — 專案骨架 + 抽象層（約 1 週）
- [ ] 建立上述 `rotbot/` 套件結構與 `pyproject.toml`（或 requirements.txt）
- [ ] `configs/field.yaml`：把 sim 裡的場地/球/機構參數集中（距離、九宮格間距、球徑質量等）
- [ ] 定義 HAL 介面：`Camera`（回傳影像）、`ActuatorLink`（送指令/收 ack）
- [ ] 把 `pnp_frame_pose.py` → `perception/pose.py`（去掉 self-test，改成可 import 的函式 + 單元測試）
- [ ] 把 `sim_adaptive_calibration.py` 拆成 `sim/world.py`（物理）+ `decision/calibration.py`（校正演算法）+ `decision/ballistics.py`（拋體解）
- [ ] 建 `tests/` 與最小 CI（pytest 跑通、lint）
- [ ] 補 README：如何跑模擬、模組職責

**DoD：** 現有兩支原型的邏輯都以模組化形式存在、有測試、能被 pipeline import。

### P1 — 感知（離線，約 2–3 週）
> **已定案（見 §4）：目標是「圓孔」；用 YOLO bbox 偵測孔中心 → 已知孔中心整板
> PnP（方法 A，主力）；合成資料用 OpenCV/PIL 程式化合成。方法 B（橢圓圓錐）備供逐孔傾角。**

- [x] `sim/scene.py`：圓孔九宮格合成器（主 3×3 Ø40 + 上排 3× Ø20 bonus），透視→橢圓、精確真值、域隨機化
- [x] `tools/gen_synthetic_data.py`：批次輸出 YOLO **偵測**資料集（bbox + 類別 main/bonus/bonus_lit）
- [x] `perception/pose.py`：方法 A（孔中心整板 PnP + 表觀直徑測距）與方法 B（橢圓圓錐反投影，含傾角）
- [x] `tools/compare_pose_methods.py`：兩法在距離/方位/傾角 vs 雜訊的比較（見 §4）
- [x] `perception/detector.py`：Ultralytics 推論包裝 + 後處理，輸出 `Detection`（孔中心 + bbox + 類別 + 網格編號）
- [x] 訓練 YOLO 偵測 baseline（`tools/train_detector.py`，yolo11n，2000 圖/100ep）
      → 驗證集 mAP50：main 0.995、bonus 0.973、bonus_lit 0.954
- [x] 全鏈定位 `perception/localize.py`（偵測→穩健孔對應→整板 PnP→每孔瞄準）+ `sim/perfect_detector.py` 替身
- [x] 評估腳本 `tools/eval_perception.py`
      → 真模型端到端（50 場）：整板 PnP 45/50、亮孔 7/7、距離平均 8.3cm、方位 1.58°

> 殘差精修（留待需要更高精度時）：用**橢圓中心**取代 bbox 中心（去除透視偏差、順帶方法 B 傾角）；改善少數整板 PnP 失敗場景。

**DoD：** ✅ 給一張合成影像，能穩定輸出「有哪些孔、亮起的加分孔在哪、每孔的距離與方位角」。

### P2 — 決策（離線，約 2 週，可與 P1 交錯）
- [ ] `decision/ballistics.py`：正解（角度→落點）與反解（目標→俯仰/水平/飛輪轉速），含空氣阻力
- [ ] `decision/calibration.py`：線上自適應校正（全域偏移 + 單格殘差 + 異常剔除 + clamp，← 現有 sim）
- [ ] `decision/strategy.py`：目標選擇 — 加分格最高優先、27 顆的投球排程、剩彈/時間管理
- [ ] 座標系轉換：相機系 → 機器人系 → 世界系（明確定義並測試）
- [ ] 蒙地卡羅回歸測試（沿用現有收斂性驗證，納入 CI）

**DoD：** 給定「目標 3D 位置 + 歷史落點」，輸出下一發的瞄準參數，且模擬中命中率隨投球數收斂。

### P3 — 致動與通訊（約 1–2 週）
- [ ] `actuation/protocol.py`：UART 指令 frame（俯仰/水平/飛輪速/觸發），含 checksum、ack、逾時
- [ ] `actuation/link.py` + `MockArduino`：模擬 Arduino 端回應，驗證協定
- [ ] `pipeline/state_machine.py`：賽制狀態機 — 啟動 → 前 60 秒全自主 → 後半場 → **紅色急停/安全**
- [ ] 前 60 秒「無無線」路徑驗證（純機上視覺，符合規則）
- [ ] 安全與失效處理：偵測不到目標、通訊逾時、超出射程時的行為

**DoD：** 主邏輯能透過協定對 MockArduino 下完整一發指令並確認回應；狀態機涵蓋賽制與安全。

### P4 — 系統整合：模擬閉迴路（約 2 週）
- [ ] `pipeline/loop.py`：串起 scene → detector → pose → strategy → ballistics → protocol → world → 落點 → calibration
- [ ] 端到端跑完一整場 27 顆的模擬，輸出命中率/加分格命中/收斂曲線
- [ ] 效能量測：模擬各階段耗時，預估 Orin 上的延遲預算
- [ ] 整合測試納入 CI

**DoD：** 一鍵跑完整場模擬並產出成績報告 — 這就是硬體到位前的「軟體完成」。

### P5 — 硬體到位後整合（硬體 milestone 觸發）
- [ ] 相機標定：實拍棋盤格求真實內參與畸變，替換 `calib/`
- [ ] `RealCamera` / `RealArduino` 實作，抽換掉 Sim/Mock
- [ ] Jetson Orin 部署：YOLO → TensorRT 匯出、FP16/INT8、實測 FPS 與延遲
- [ ] 用**真實影像**微調/重訓 YOLO（合成→真實的域差距修正）
- [ ] 硬體在環 (HIL)：先手動下指令驗證機構、再接自主迴路

**DoD：** 實機能在桌面環境跑完「看→算→投」單發閉迴路。

### P6 — 賽前強化（最後數週）
- [ ] 現場抗干擾：LED/眩光/別隊訊號下的偵測穩健度
- [ ] 建**實機校正表**：實測落點資料回填，取代模擬假設值
- [ ] 失效保護與 contingency：偵測失敗、通訊斷線、球卡彈的 fallback
- [ ] 完整實戰演練（含前 60 秒自主計時），量測實際收斂顆數
- [ ] 賽前 checklist 與參數凍結

**DoD：** 在接近比賽的場地條件下，自主命中率與收斂顆數達標且可重現。

---

## 4. 關鍵技術決策

1. ✅ **目標形狀與位姿方案** → 目標是**圓孔**（賽事工程圖確認：主 Ø40cm、上排加分 Ø20cm；
   非方框——先前 `pnp_frame_pose.py` 的方形假設是錯的）。方案：**YOLO bbox 偵測孔中心
   → 已知 12 孔中心整板 PnP（方法 A，主力）**。逐孔傾角需求時用**方法 B（橢圓圓錐反投影）**。
   合成真值上兩法比較（60 場、注入雜訊）：

   | 方法 | 距離誤差 | 方位/俯仰 | 傾角 | 說明 |
   |---|---|---|---|---|
   | A-PnP（多孔） | 0.2–0.5 cm | ~0.06° | ✗ | 最準最穩，需 ≥4 孔 |
   | A-尺寸（單孔深度） | ~6 cm 系統性 | — | ✗ | <4 孔時備援 |
   | B（橢圓圓錐，單孔） | 0–2 cm | ~0.08° | 0.6–1.8° | 唯一回傳傾角 |

   → **A 主力、B 補傾角、A-尺寸備援**。次像素/seg 精修留待 P5。
2. ✅ **合成資料引擎？** → **OpenCV/PIL 程式化合成**先行（重用投影數學、標註免費精確、可進 CI、直接做域隨機化）。
   升級路徑保留 **Blender (bpy)**；Unity 僅保留為簡報敘事，不投單人工時。
3. ⏳ **座標系慣例：** 相機、機器人、世界三個座標系的軸向與原點要一開始就寫死在文件裡，之後所有轉換都對照它（P2 處理）。
4. ⏳ **延遲預算：** 前 60 秒要即時，先估「偵測 + 解算 + 通訊」總延遲目標（例如 < 100 ms/迴圈），P4 就開始量。

---

## 5. 目前進度

- ✅ **P0 完成**（commit `36668bb`）：骨架、HAL、config、原型模組化、14 個測試 + CI。
- ▶ **P1 進行中**：兩個主幹決策已定案（見 §4），下一步從 `sim/scene.py` 合成場景產生器開始。

> 進度追蹤：完成一項就勾一個 checkbox；每個 Phase 收尾時對照 DoD 再往下走。
