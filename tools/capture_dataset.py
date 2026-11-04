# -*- coding: utf-8 -*-
"""實拍資料收集 + 預標註 —— 縮短「合成→真實」微調的時間（不必從零重訓）。

現在就能做：用真相機拍「等比例自製孔板」（或練習場的真網板），每隔一段時間存一張，
並用目前的合成模型自動預標註（YOLO 格式）；人工只需用 labelImg / CVAT / Roboflow 修正錯框。
之後以合成模型為起點微調（約 100~300 張、20~30 epochs，RTX 4050 幾分鐘）：

    python -m tools.capture_dataset --n 200 --every 0.5 --out datasets/real/practice1
    python -m tools.train_detector --data datasets/real/practice1/dataset.yaml \\
        --model models/grid_holes_yolo11n.pt --epochs 30 --name finetune_real

拍攝建議：不同距離（2.5~7.5m）、左右偏角、亮燈/不亮、場館燈光與逆光、曝光鎖定與否都各拍一些。
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cv2

from rotbot.config import load_config
from rotbot.perception.camera import open_camera

CLASS_NAMES = {0: "main", 1: "bonus", 2: "bonus_lit"}


def main():
    ap = argparse.ArgumentParser(description="實拍資料收集 + 預標註")
    ap.add_argument("--camera", default="cfg", help="cfg = field.yaml 的 camera_hw（IMX296）/ USB 相機編號 / GStreamer 管線")
    ap.add_argument("--n", type=int, default=100, help="張數")
    ap.add_argument("--every", type=float, default=0.5, help="每幾秒存一張")
    ap.add_argument("--out", required=True)
    ap.add_argument("--prelabel", default=None, help="用這個權重預標註（例：models/grid_holes_yolo11n.pt）")
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--val-every", type=int, default=5, help="每 N 張放 1 張到 val")
    args = ap.parse_args()
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")

    cfg = load_config()
    cam = open_camera(args.camera, cfg)
    model = None
    if args.prelabel:
        from ultralytics import YOLO
        model = YOLO(args.prelabel)
    out = Path(args.out)
    for split in ("train", "val"):
        (out / "images" / split).mkdir(parents=True, exist_ok=True)
        (out / "labels" / split).mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    for i in range(args.n):
        t0 = time.monotonic()
        img = cam.read()
        split = "val" if i % args.val_every == 0 else "train"
        stem = f"{stamp}_{i:04d}"
        cv2.imwrite(str(out / "images" / split / f"{stem}.jpg"), img, [cv2.IMWRITE_JPEG_QUALITY, 95])
        lines = []
        if model is not None:
            r = model.predict(img, conf=args.conf, imgsz=960, verbose=False)[0]
            for (cx, cy, w, h), c in zip(r.boxes.xywhn.cpu().numpy(), r.boxes.cls.cpu().numpy()):
                lines.append(f"{int(c)} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")
        (out / "labels" / split / f"{stem}.txt").write_text("\n".join(lines), encoding="utf-8")
        print(f"[{i + 1}/{args.n}] {split}/{stem}.jpg  預標註 {len(lines)} 框")
        time.sleep(max(0.0, args.every - (time.monotonic() - t0)))
    cam.release()
    names = "\n".join(f"  {k}: {v}" for k, v in CLASS_NAMES.items())
    (out / "dataset.yaml").write_text(
        f"path: {out.resolve().as_posix()}\ntrain: images/train\nval: images/val\n\nnames:\n{names}\n",
        encoding="utf-8")
    print(f"完成 → {out.resolve()}（先人工修正 labels/ 再微調）")


if __name__ == "__main__":
    main()
