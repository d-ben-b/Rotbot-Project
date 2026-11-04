# 軟體架構與物理／校正實驗（工作參考）

> ⚠️ 2026-10-25 起投球距離改為 **6 m**：本文 §1–§2 的實驗數字是 2.5 m 時期（2026-09）的結果，保留作歷史對照。
> 6 m 的參數、成績與硬體規格見 [six_meter_reassessment.md](six_meter_reassessment.md)；
> 新增的協定/韌體/網頁操控/自主主迴圈見 [mcu_orin_link.md](mcu_orin_link.md)、[teleop_network.md](teleop_network.md)、
> [hardware_bringup_plan.md](hardware_bringup_plan.md)。

> AI 自主棒球投手機器人 — 軟體（感知・決策・模擬）。本文件涵蓋：設定參數來源分類、
> 物理與校正實驗結果、整體程式架構、各模組邏輯、用到的函式庫、關鍵假設、以及要提前
> 跟硬體組講的前提。日期：2026-09。

---

## 1. `configs/field.yaml` 參數來源分類

| 參數 | 來源 | 說明 |
|---|---|---|
| `frame_d=0.40`、`bonus_d=0.20` | **規則明說** | 一般孔 Ø40、加分孔 Ø20（官方尺寸圖） |
| `col_pitch=0.90`、`row_pitch=0.60`、`bonus_col_pitch=0.90`、`bonus_dy=0.50` | **規則／官方尺寸圖** | 由孔徑＋邊距推得（公差 ±2cm） |
| `ball.diameter=0.07`、`ball.mass=0.030` | **規則／計劃書** | PU 棒球 7cm／30g |
| `match.n_balls=27` | **規則明說** | 上半場 27 球 |
| `dist_x=2.50` | **硬體／現場實測** | 發射口到網板水平距（場地擺放＋車體幾何） |
| `launch_h=0.40` | **硬體要給** | 發射口離地高（依機器人設計） |
| `grid_cy=1.50` | **硬體／現場實測** | 九宮格中心離地高（依網板架設） |
| `camera.fx/fy/cx/cy/width/height` | **硬體要給（標定）** | 目前為佔位值，真 webcam 需做內參標定 |
| `camera_mount.*` | **硬體要給（標定）** | 相機相對槍口外參，全 0 為暫定 |
| `launch.v0_real`、`launch.v0_jitter` | **硬體要量** | 實際飛輪出射速度與逐發變異 |
| `launch.v0_model=10.0` | 硬體／假設 | 腦內模型認定的初速（設為硬體標稱值） |
| `grid_cx=0.0` | **慣例／假設** | 假設九宮格橫向對齊發射軸 |
| `air.rho=1.20`、`air.g=9.81` | 物理常數 | 標準值 |
| `air.cd=0.50` | **假設** | 球體阻力係數估值（可再校） |
| `mechanism.bias_pitch/bias_yaw/jitter_deg` | **假設（佔位）** | 真值待量；系統偏差會被線上校正吸收 |
| `calibration.*` | **假設／可調** | 校正增益、上限、量測雜訊（調參） |
| `sim.dt/seed` | 模擬用 | 積分步長／亂數種子 |

---

## 2. 物理 × 校正 實驗結果

> 以 `rotbot/sim/closed_loop.py` 的蒙地卡羅跑；每場 27 球。

### 表1｜線上校正 ON vs OFF（各 60 場）
| | 總分 | 命中率 | 加分孔命中率 |
|---|---|---|---|
| 校正 OFF | 43.2 ± 2.6 | 80% | 1% |
| 校正 ON | **51.0 ± 3.1** | **91%** | **27%** |

### 表2｜逐球誤差收斂（60 場平均；平均落點誤差 / 命中率）
| | 球 1–3 | 球 4–9 | 球 10–18 | 球 19–27 |
|---|---|---|---|---|
| OFF | 18.4cm / 1% | 9.4cm / 99% | 10.8cm / 94% | 13.5cm / 77% |
| ON | 10.9cm / 31% | 5.4cm / 100% | 5.7cm / 100% | 6.7cm / 98% |

### 表3｜對物理惡化的穩健度（命中率 OFF → ON）
| 條件 | OFF | ON | 改善 |
|---|---|---|---|
| 基準（現況） | 79% | 91% | +12pp |
| 2× 安裝偏差 | 19% | 91% | +72pp |
| 2× 逐發抖動 | 68% | 83% | +15pp |
| 1.5× 空氣阻力 | 69% | 91% | +22pp |

**結論**
1. 校正對**系統性誤差**（安裝偏差、空氣阻力）幾乎完全吸收（+72pp／+22pp）。
2. 對**隨機逐發抖動**只能部分改善（+15pp）→ **機構重複精度是硬體端的責任**。
3. ⚠️ **加分孔是弱點（命中 27%）**：因 Ø20 容許誤差僅 ~6.5cm（一般孔 ~16.5cm），且策略把加分孔排在最前 3 球投，那時校正尚未收斂（球 1–3 誤差 10.9cm）。
   - 對策：(A) **賽前練習先投幾顆把全域偏移 `g_bias` 種好**；(B) 先 1–2 顆試射打大孔收斂再打加分孔；(C) 兩者並用。

---

## 3. 整體架構

**分層（`rotbot/`）**
```
config.py     單一設定來源（field.yaml → 型別化 dataclass）
hal.py        硬體抽象層：Camera / ActuatorLink 介面 + AimCommand/Ack
perception/   detector(YOLO) → localize(整板 PnP) → pose(測距/圓錐)
decision/     ballistics(瞄準) + calibration(校正) + strategy(選目標) + frames(座標轉換)
actuation/    link(MockArduino + UART 協定)
sim/          world(真實物理) + scene(合成影像) + pybullet_world(3D 沙盒) + closed_loop(整場)
tools/        train / eval / teleop_sim / webcam_ranger / demo 影片
```

**一發球的資料流（閉迴路）**
```
相機影像 →[perception] 每孔(距離,方位,俯仰)
        →[strategy]   選目標(亮孔優先)
        →[frames]     轉到發射點座標系 3D
        →[ballistics] 解析拋體算(俯仰,水平)
        →[calibration]疊加已學到的角度修正
        →[actuation]  AimCommand 經 UART 送 Arduino
        →[world/物理] 含阻力+機構偏差投出 → 落點
        →[計分]+[calibration.update] 用落點誤差修正 → 下一發更準
```

**物理：刻意用兩個模型**
- 腦內模型（`decision/ballistics.py`）：**無阻力**解析拋體解，快、可解釋、即時。
- 真實世界（`sim/world.py`）：**RK4＋平方阻力＋機構偏差＋逐發抖動＋初速變異**（PyBullet 另加 Magnus）。
- 兩者落差＝校正線上要學掉的東西。真機上「真實世界」換成真發射器，校正照樣運作。

**校正演算法（`decision/calibration.py`）**
- 維護**全域偏移 `g_bias`**（吸收安裝偏差／阻力）＋**每格殘差 `cell_off`**。
- 落點位置誤差(m) → 經**靈敏度 Jacobian**（∂高度/∂俯仰、∂橫向/∂水平）→ 角度修正量(rad)。
- EMA 增益更新（`alpha_global=0.5`、`alpha_cell=0.3`）＋異常值剔除（>60cm 不更新）＋修正上限（±5°）。

---

## 4. 感知模組邏輯

**`perception/detector.py`** — 影像 → `Detection` 清單
- `Detection`：`group/idx/center/bbox/diameter_px/is_bonus/is_lit/confidence`；類別 0=main、1=bonus、2=bonus_lit。
- `YoloDetector.detect`：延後 import ultralytics → predict → 每框轉 `Detection` → `assign_ids`。
- `assign_ids`：main 依 y 分列（門檻 0.6×中位框高）、列內依 x → 0~8；bonus 依 x → 0~2。
- ⚠️ 假設 `bbox 中心 ≈ 孔中心`（正對成立，傾斜時橢圓中心≠框中心）；此編號只是初步，靠 localize 修正。

**`perception/pose.py`** — 兩種測距／位姿
- 方法 A `solve_board_pose`：已知孔中心做 **PnP(SQPNP)** → 整板位姿 `BoardPose.point_aim()` 得距離/方位/俯仰。`distance_from_diameter` 為單孔尺寸測距備援。
- 方法 B `pose_from_ellipse`：橢圓 → 圓錐矩陣 → `B=KᵀQK` 特徵分解 → 單孔 3D 圓心＋法向（含 tilt）；正對退化回 None。
- 選 SQPNP：共面＋正對都穩；IPPE 正對退化、RANSAC 共面不穩。

**`perception/localize.py`** — 偵測 → 每孔瞄準（穩健接合）
- `_solve_group`：SQPNP 初估 → `_rematch_by_projection`（投影已知孔中心、就近重配、去重，門檻＝投影孔最近鄰距的一半）。
- `localize_holes`：解 main 群組 → 重解整板位姿 `bp` → 逐孔 `HoleAim`（main 用 `bp.point_aim`；bonus 用投影就近；無位姿 fallback 尺寸測距）。
- 動機：真實偵測有多餘框/邊緣孔/誤分類，投影重匹配修掉編號錯（曾出現 114cm 誤差）。

---

## 5. 決策：投球策略（`decision/strategy.py`）
- `ShotPlanner.select`：**前 60 秒** → 亮加分孔最優先（未投滿選最近）；否則 → 最近的未投滿一般孔（每孔上限 3）；未亮加分孔／窗口後不選。
- `plan_next_shot`：每 aim →`frames.hole_target_point(_ext)` 轉發射點系 3D →`ballistics.aim_at_point` 解瞄準 → 射程可達過濾 →`select` → `ShotPlan`。
- `hole_target_point` 用 `camera_mount` 尤拉角（近似）；`hole_target_point_ext` 用完整外參矩陣（render 閉迴路，幾何精確）。

---

## 6. 模擬（`sim/`）
| 檔 | 角色 |
|---|---|
| `world.py` `BallisticWorld` | 真實世界物理：RK4＋平方阻力；`shoot`=理想、`shoot_real`=加機構偏差/抖動/初速變異 |
| `scene.py` `SceneGenerator` | 程序化合成影像（YOLO 訓練/測）：隨機視角＋域隨機化，圓→橢圓，附像素級真值；亮孔畫紅環 |
| `camera.py` `SimCamera` | HAL 相機實作：回 scene 影像或空白 + 內參 |
| `perfect_detector.py` | 由 scene 真值直接產 `Detection`（YOLO 未就緒前測下游） |
| `closed_loop.py` | 整場閉迴路：`run_closed_loop`（真值+雜訊）、`run_closed_loop_render`（真 YOLO+render+精確外參）、`monte_carlo` |
| `match.py` | 最小 3×3 閉迴路，校正回歸測試基準 |
| `board_mesh.py` | 程序化含孔網板網格（圓弧貼齊）＋ LED 圓環＋`board_extents` |
| `pybullet_world.py` | 3D 剛體沙盒：機器人＋網板/外框/暗網/Truss＋球（阻力+Magnus+進孔接球）＋計分 |
| `pybullet_camera.py` | 相機姿態渲染（TinyRenderer+陰影/打光，後製雜訊/眩光）＋外參/投影工具 |
| `textures.py` | 程序化材質 PNG（木紋/暗網/地板/金屬/棒球） |

---

## 7. 用到的函式庫
| 函式庫 | 用途 |
|---|---|
| numpy | 數學/陣列/亂數（物理積分、幾何、統計） |
| opencv (cv2) | PnP(`solvePnP` SQPNP)、橢圓擬合、合成影像、影片、繪圖 |
| ultralytics YOLO + torch | 圓孔偵測訓練與推論（GPU） |
| pybullet (+pybullet_data) | 3D 剛體物理沙盒與相機渲染 |
| pyyaml | 讀 `configs/field.yaml` |
| pytest | 自動測試 |

---

## 8. 關鍵假設
1. 腦內瞄準**忽略空氣阻力**（差異靠校正補）。
2. 真實物理用**定值 Cd=0.5、空氣密度 1.2**、平方阻力；無風、球不變形、剛體、不模擬入網彈跳。
3. 計分＝球心在網板平面時落在 `(孔徑−球徑)/2` 容許半徑內（當 2D 平面接球）。
4. 靈敏度（角度↔落點）在工作範圍內近似常數（單點估一次）。
5. 轉塔能達到指令角度（未建模伺服延遲／背隙）。
6. 校正假設誤差以系統性為主＋小量隨機；純隨機抖動學不掉。
7. 出射速度 `v0≈10 m/s` 已知且逐發變異小。

---

## 9. 要提前跟硬體組講的前提
1. **出射球速 v0** 要可控且逐發穩定（理想 <±0.2 m/s）；隨電量漂移需回授或中途重校。
2. **轉塔重複精度/背隙**：隨機抖動校正救不了（表3），扇形齒輪＋迴轉軸承背隙建議 <0.3°。
3. **投球前安定時間**：轉塔到位才發射，需「到位訊號」或固定 settle 延遲。
4. **相機**：(a) 與槍管同軸、(b) 需內參標定、(c) 需量相對槍口外參、(d) FOV 在 2.5m 涵蓋整塊板（~78° webcam 可）、(e) 鎖曝光/白平衡防眩光。
5. **現場實測 4 值**：`dist_x`、`launch_h`、`grid_cy`、板安裝角。
6. **供彈**：一次 shoot＝一顆球，可靠不卡彈。
7. **UART 協定**：Arduino 收 `AimCommand(pitch,yaw,flywheel,trigger)` 並回 ack。
8. **賽前練習留投球額度**：校正需 4~6 球收斂；要吃加分孔（小又排最前）務必賽前先種好手感。
