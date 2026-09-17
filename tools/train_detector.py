# -*- coding: utf-8 -*-
"""訓練 YOLO 圓孔偵測模型（本機 GPU）。

需求：ultralytics + torch(CUDA)。不放進 requirements.txt / CI（CI 無 GPU）。

用法：
    # 一次搞定：產 2000 張資料 → 訓練
    python -m tools.train_detector --gen --n 2000 --epochs 100

    # 資料已存在，直接訓練
    python -m tools.train_detector --data datasets/grid_holes/dataset.yaml --epochs 100

參數預設為 6GB VRAM（RTX 4050）調校：yolo11n、imgsz 960、batch 自動。
"""
from __future__ import annotations

import argparse
from pathlib import Path

# 專案根目錄：讓輸出固定落在本 repo，不受 ultralytics 全域 runs_dir 影響
REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_PROJECT = str(REPO_ROOT / "runs" / "detect")


def train(data_yaml: str, model: str = "yolo11n.pt", epochs: int = 100,
          imgsz: int = 960, batch: int | float = -1, device: str = "0",
          project: str = DEFAULT_PROJECT, name: str = "grid_holes",
          workers: int = 2):
    """跑訓練，回傳 ultralytics 的 results。batch=-1 讓 ultralytics 自動抓 VRAM。

    workers 調低 + cache=False：避免 1920×1080 大圖 × 多 worker × mosaic 把系統 RAM 吃爆。
    """
    from ultralytics import YOLO   # 延後 import：無 GPU/未安裝環境不受影響
    m = YOLO(model)
    return m.train(data=data_yaml, epochs=epochs, imgsz=imgsz, batch=batch,
                   device=device, project=project, name=name, plots=True,
                   workers=workers, cache=False)


def main() -> None:
    ap = argparse.ArgumentParser(description="訓練 YOLO 圓孔偵測模型")
    ap.add_argument("--data", default="datasets/grid_holes/dataset.yaml")
    ap.add_argument("--gen", action="store_true", help="訓練前先產生資料集")
    ap.add_argument("--n", type=int, default=2000, help="--gen 時的樣本數")
    ap.add_argument("--model", default="yolo11n.pt")
    ap.add_argument("--epochs", type=int, default=100)
    ap.add_argument("--imgsz", type=int, default=960)
    ap.add_argument("--batch", type=float, default=-1, help="-1 = 自動")
    ap.add_argument("--device", default="0", help="'0'=第一張 GPU，'cpu'=CPU")
    ap.add_argument("--workers", type=int, default=2, help="dataloader worker 數（RAM 吃緊調低）")
    args = ap.parse_args()

    if args.gen:
        from rotbot.config import load_config
        from tools.gen_synthetic_data import generate_dataset
        out = generate_dataset(load_config(), args.n, Path(args.data).parent, seed=0)
        print(f"資料集已產生：{out.resolve()}")

    batch = int(args.batch) if args.batch >= 1 else args.batch
    results = train(args.data, model=args.model, epochs=args.epochs,
                    imgsz=args.imgsz, batch=batch, device=args.device,
                    workers=args.workers)
    save_dir = getattr(results, "save_dir", None)
    print("訓練完成。輸出：", save_dir or "runs/detect/…")

    # 自動複製最佳權重到 models/（統一存放，方便管理多個模型）
    if save_dir:
        best = Path(save_dir) / "weights" / "best.pt"
        if best.exists():
            import shutil
            models_dir = REPO_ROOT / "models"
            models_dir.mkdir(exist_ok=True)
            dest = models_dir / "grid_holes_yolo11n.pt"
            shutil.copy2(best, dest)
            print("已複製最佳權重 →", dest)


if __name__ == "__main__":
    main()
