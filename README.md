# Rotbot — AI 自主棒球投手機器人（軟體）

嘻諧們｜2026 TEL 東京威力科創機器人大賽

三層架構：**感知 (perception) → 決策 (decision) → 致動 (actuation)**，
以 `hal` 抽象層隔離真硬體與 `sim` 模擬 —— 硬體到位前先在模擬裡跑通整條閉迴路，
硬體到了只換底層實作，主邏輯不動。

> 進度與數字見 [PROGRESS.md](PROGRESS.md)；規劃見 [ROADMAP.md](ROADMAP.md)。
> 2026-10-25 起投球距離 **6 m**：[docs/six_meter_reassessment.md](docs/six_meter_reassessment.md)；
> 硬體到位前後的驗證計畫：[docs/hardware_bringup_plan.md](docs/hardware_bringup_plan.md)。

## 專案結構

```text
rotbot/
  config.py            # 集中式設定（讀 configs/field.yaml）
  hal.py               # 硬體抽象層：Camera / ActuatorLink 介面
  perception/          # detector(YOLO) → localize(整板 PnP) → pose；camera(make_camera / SharedCamera)
                       # imx296(機上相機 V4L2 + ISP + 去畸變) / intrinsics(棋盤格內參標定)
  decision/            # ballistics(含阻力瞄準) / calibration(線上校正, 可存檔) / strategy / frames(轉塔運動學)
  actuation/           # protocol(Orin↔MCU 協定 v1) / link(SerialMCU / MockArduino)
  pipeline/            # loop(自主主迴圈 + AutonomousRunner) / state_machine(60 秒自主鎖定)
  teleop/              # 手動操控 Web 伺服器 + 控制頁（鍵盤 / 搖桿）
  sim/                 # world(真實物理) / scene(合成影像) / closed_loop / twin(數位孿生) / pybullet_*
firmware/rotbot_mcu/   # Arduino Uno 韌體（同一份原始碼也在電腦上跑成模擬器：firmware/host）
configs/               # field.yaml（場地/球/機構/校正）、robot.yaml（機器人幾何）
tools/                 # 各種可直接執行的工具（見下）
docs/                  # 6 m 評估、MCU 連線、網路操控、硬體驗證計畫
tests/                 # pytest（57 項）
```

## 安裝

```bash
uv venv .venv --python 3.12            # 或 python -m venv（CI 用 3.12）
uv pip install -r requirements.txt     # 核心 + 測試
# GPU（YOLO 訓練/推論）：只要 NVIDIA 驅動，不必裝 CUDA Toolkit
uv pip install torch torchvision --index-url https://download.pytorch.org/whl/cu130
uv pip install -r requirements-gpu.txt
# 韌體模擬器需要 C++ 編譯器：Linux/Orin 內建 g++；Windows 沒有的話 `uv pip install ziglang`
pytest -q
```

## 常用工具

| 指令 | 用途 |
|---|---|
| `python -m tools.robot_main --sim` | **比賽主程式**（網頁按開始 → 60 秒 AI 自主 → 可切手動）；`--sim` = 韌體模擬器 + 數位孿生 |
| `python -m tools.run_autonomous --sim` | 自主主迴圈跑一整場並印逐球明細 |
| `python -m tools.teleop_web --mcu sil --camera sim` | 手動操控網頁（路由器區網 / 鍵盤 / 搖桿） |
| `python -m tools.mcu_link_test --sil` | Orin↔MCU 連線驗證（`--port COM5` / `/dev/ttyACM0` 打實機） |
| `python -m tools.mcu_sil` | 建置並執行韌體模擬器（`socket://127.0.0.1:5760`） |
| `python -m tools.reassess_6m` | 6 m 全部誤差與假設重算（模擬/合成） |
| `python -m tools.fit_ballistics shots.csv` | 打牆資料擬合 v0 / Cd / 安裝偏差 |
| `python -m tools.bench_pipeline` | 感知+決策延遲（Orin 上加 `--export-trt`） |
| `python -m tools.capture_dataset --camera 0 --prelabel models/grid_holes_yolo11n.pt --out …` | 實拍收集 + 預標註（微調用） |
| `python -m tools.train_detector` / `eval_perception` | 訓練 / 評估 YOLO |
| `python -m tools.run_match --trials 50` | 閉迴路蒙地卡羅（真值+誤差感知，快） |

原型視覺化腳本仍保留在 [presentation/](presentation/)。
