# Rotbot — AI 自主棒球投手機器人（軟體）

嘻諧們｜2026 TEL 東京威力科創機器人大賽

三層架構：**感知 (perception) → 決策 (decision) → 致動 (actuation)**，
以 `hal` 抽象層隔離真硬體與 `sim` 模擬 —— 硬體到位前先在模擬裡跑通整條閉迴路，
硬體到了只換底層實作，主邏輯不動。

> 規劃與里程碑見 [ROADMAP.md](ROADMAP.md)。目前處於 **P0（骨架建置）已完成**。

## 專案結構

```text
rotbot/
  config.py          # 集中式設定（讀 configs/field.yaml）
  hal.py             # 硬體抽象層：Camera / ActuatorLink 介面
  perception/
    pose.py          # PnP 網框位姿解算（距離/方位/傾角）✅
    detector.py      # YOLO 偵測介面（P1 實作）
    camera.py        # RealCamera（P5）
  decision/
    ballistics.py    # 拋體瞄準模型（無阻力解析解）✅
    calibration.py   # 線上自適應校正「找手感」✅
    strategy.py      # 九宮格幾何、目標選擇
  actuation/
    protocol.py      # UART 指令協定
    link.py          # MockArduino ✅ / SerialArduino（P5）
  sim/
    world.py         # 真實拋體物理（含空氣阻力 RK4）✅
    match.py         # 模擬一場 27 顆的閉迴路 ✅
    camera.py        # SimCamera 替身
  pipeline/
    loop.py          # 主迴圈（P4）
    state_machine.py # 賽制狀態機（P3）
configs/field.yaml   # 場地/球/機構/校正 參數
tests/               # pytest
```

## 安裝與測試

```bash
pip install -r requirements.txt
pytest -q
```

## 快速試跑（模擬一場比賽）

```python
from rotbot.config import load_config
from rotbot.sim.match import monte_carlo

cfg = load_config()
h_on, h_off, b_on = monte_carlo(cfg, n_trials=100)
print(f"一般框命中：校正前 {h_off.mean():.0%} → 校正後 {h_on[:,3:].mean():.0%}")
print(f"加分格命中（校正後，第4顆起）：{b_on[:,3:].mean():.0%}")
```

原型視覺化腳本仍保留在 [presentation/](presentation/)（收斂曲線、命中率、PnP 示意圖）。
