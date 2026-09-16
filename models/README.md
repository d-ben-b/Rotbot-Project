# models/ — 訓練好的模型權重

放置本專案使用的模型權重（`.pt` / 未來的 `.engine` TensorRT 等）。

> **權重檔 (`*.pt`) 不入 git**（見 `.gitignore`，二進位大檔）。本資料夾只有這份 README 進版控；
> 權重請由訓練產生或另行保存/分享。

## 命名慣例

`<用途>_<模型架構>[_<版本或日期>].pt`，例如：

| 檔名 | 說明 |
|---|---|
| `grid_holes_yolo11n.pt` | 九宮格圓孔偵測（yolo11n，合成資料訓練）— 目前主用 |

## 目前模型

- **`grid_holes_yolo11n.pt`** — ✅ **基準模型（已定版 2026-08-18）**
  - 任務：圓孔偵測（類別 0=main / 1=bonus / 2=bonus_lit）
  - 輸入：BGR 影像；建議 imgsz 960、conf 0.25
  - 訓練：`tools/train_detector.py`，2000 張合成資料 / 100 epochs / imgsz 960
  - 驗證集 mAP50：main 0.995、bonus 0.973、bonus_lit 0.954
  - **端到端感知評估**（`tools/eval_perception.py`，50 場合成，2026-08-18）：
    整板 PnP 46/50、亮孔正確帶出 7/7、距離誤差平均 4.9cm（p95 16.4）、方位 1.20°、俯仰 0.44°
  - 已知限制：略微過偵測（偵測 489 vs 可見 439，可調高 conf）；遠孔距離 p95 偏大；
    合成資料訓練，真實賽場/PyBullet 渲染有**域差**（日後以 Blender 合成或實拍微調收斂）
  - 來源：`runs/detect/grid_holes-2/weights/best.pt` 複製而來

## 如何產生 / 使用

```bash
# 訓練（完成後會自動複製一份到 models/）
python -m tools.train_detector --gen --n 2000 --epochs 100

# 用權重跑評估 / demo（預設即讀 models/grid_holes_yolo11n.pt）
python -m tools.eval_perception --weights models/grid_holes_yolo11n.pt
python -m tools.demo_perception --weights models/grid_holes_yolo11n.pt
```
