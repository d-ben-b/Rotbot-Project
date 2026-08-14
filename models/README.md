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

- **`grid_holes_yolo11n.pt`**
  - 任務：圓孔偵測（類別 0=main / 1=bonus / 2=bonus_lit）
  - 訓練：`tools/train_detector.py`，2000 張合成資料 / 100 epochs / imgsz 960
  - 驗證集 mAP50：main 0.995、bonus 0.973、bonus_lit 0.954
  - 來源：`runs/detect/grid_holes-2/weights/best.pt` 複製而來

## 如何產生 / 使用

```bash
# 訓練（完成後會自動複製一份到 models/）
python -m tools.train_detector --gen --n 2000 --epochs 100

# 用權重跑評估 / demo（預設即讀 models/grid_holes_yolo11n.pt）
python -m tools.eval_perception --weights models/grid_holes_yolo11n.pt
python -m tools.demo_perception --weights models/grid_holes_yolo11n.pt
```
