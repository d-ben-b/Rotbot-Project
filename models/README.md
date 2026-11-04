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

- **`grid_holes_yolo11n.pt`** — ✅ **6 m 版（2026-10-25 重訓，取代 08-18 版）**
  - 任務與類別同下（0=main / 1=bonus / 2=bonus_lit 紅色 LED 環）；建議 imgsz 960、conf 0.25
  - 資料：`tools/gen_synthetic_data.py --n 2000 --dist-min 2.5 --dist-max 7.5`（涵蓋整個機器人作業區；舊版只有 2.0–3.2 m）
  - 訓練：`tools/train_detector.py --data datasets/grid_holes_6m/dataset.yaml --epochs 80 --imgsz 960 --name grid_holes_6m`
    （yolo11n、RTX 4050、3.5 h；torch 2.14.1+cu130、ultralytics 8.4.188）
  - 驗證集（400 張）：P 1.000、R 0.996；mAP50 main / bonus / bonus_lit 皆 0.995；mAP50-95 0.995 / 0.989 / 0.993；推論 5.0 ms/張
  - **端到端（合成影像，60 場；`tools/reassess_6m.py --sections F`）**：6 m（5.5–6.5）整板 PnP 60/60、亮孔 50/50、
    距離誤差中位 0.2 cm（p95 0.4）、方位/俯仰 0.01°、孔編號配錯 0/707；2.5–7.5 m 全區 60/60、配錯 0/658
  - 已知限制：仍是純合成資料 → 真實相機需實拍驗證與微調（`tools/capture_dataset.py`）；2.5 m 邊緣孔被切時偶有配錯
  - 來源：`runs/detect/grid_holes_6m/weights/best.pt`

- `grid_holes_yolo11n.pt`（舊，2026-08-18 基準，已被上面取代；權重未保留在本機）
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
