# -*- coding: utf-8 -*-
"""產生 YOLO 偵測資料集 —— 把 sim.scene 的圓孔合成樣本轉成 Ultralytics 格式。

目標為圓孔（一般 Ø40 / 加分 Ø20），採「中心整板 PnP」路線 → 用一般 bbox 偵測。

輸出結構（Ultralytics detect 標準）：
    <out>/
      images/train/*.png   labels/train/*.txt
      images/val/*.png     labels/val/*.txt
      dataset.yaml
      preview/*.png

標籤每行：class  cx cy w h   （皆正規化到 [0,1]）
類別：0=main、1=bonus（未亮）、2=bonus_lit（亮起指定得分格）

用法：
    python -m tools.gen_synthetic_data --n 2000 --out datasets/grid_holes
"""
from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

from rotbot.config import Config, load_config
from rotbot.perception.detector import CLASS_BONUS, CLASS_BONUS_LIT, CLASS_MAIN
from rotbot.sim.scene import HoleLabel, SceneGenerator, SceneSample

CLASS_NAMES = {0: "main", 1: "bonus", 2: "bonus_lit"}


def _class_of(hole: HoleLabel) -> int:
    if not hole.is_bonus:
        return CLASS_MAIN
    return CLASS_BONUS_LIT if hole.is_lit else CLASS_BONUS


def hole_to_line(hole: HoleLabel, W: int, H: int) -> str | None:
    """把一個圓孔轉成一行 YOLO 偵測標籤；在相機後方或完全出框回傳 None。"""
    if hole.center_cam[2] <= 0:
        return None
    xs, ys = hole.rim_px[:, 0], hole.rim_px[:, 1]
    ix1, iy1 = max(0.0, xs.min()), max(0.0, ys.min())
    ix2, iy2 = min(W, xs.max()), min(H, ys.max())
    if ix2 <= ix1 or iy2 <= iy1:
        return None
    cx, cy = (ix1 + ix2) / 2 / W, (iy1 + iy2) / 2 / H
    w, h = (ix2 - ix1) / W, (iy2 - iy1) / H
    return f"{_class_of(hole)} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}"


def sample_to_label(sample: SceneSample, W: int, H: int) -> str:
    lines = [ln for hole in sample.holes if (ln := hole_to_line(hole, W, H))]
    return "\n".join(lines)


def _annotate(sample: SceneSample) -> np.ndarray:
    img = sample.image.copy()
    for hole in sample.holes:
        if hole.center_cam[2] <= 0:
            continue
        col = (80, 255, 120) if hole.is_lit else ((0, 200, 255) if hole.is_bonus else (0, 180, 255))
        cv2.polylines(img, [hole.rim_px.astype(np.int32)], True, col, 1)
        c = hole.center_px.astype(int)
        cv2.circle(img, tuple(c), 4, col, -1)
    return img


def generate_dataset(cfg: Config, n: int, out_dir: str | Path,
                     val_split: float = 0.2, seed: int = 0,
                     n_preview: int = 8) -> Path:
    out = Path(out_dir)
    W, H = cfg.camera.width, cfg.camera.height
    gen = SceneGenerator(cfg, rng=np.random.default_rng(seed), randomize=True)

    for split in ("train", "val"):
        (out / "images" / split).mkdir(parents=True, exist_ok=True)
        (out / "labels" / split).mkdir(parents=True, exist_ok=True)
    (out / "preview").mkdir(parents=True, exist_ok=True)

    n_val = int(n * val_split)
    for i in range(n):
        split = "val" if i < n_val else "train"
        sample = gen.sample()
        stem = f"{i:06d}"
        cv2.imwrite(str(out / "images" / split / f"{stem}.png"), sample.image)
        (out / "labels" / split / f"{stem}.txt").write_text(
            sample_to_label(sample, W, H), encoding="utf-8")
        if i < n_preview:
            cv2.imwrite(str(out / "preview" / f"{stem}.png"), _annotate(sample))

    _write_yaml(out)
    return out


def _write_yaml(out: Path) -> None:
    names = "\n".join(f"  {k}: {v}" for k, v in CLASS_NAMES.items())
    yaml_text = (
        f"# YOLO 圓孔偵測資料集（由 tools/gen_synthetic_data.py 產生）\n"
        f"path: {out.resolve().as_posix()}\n"
        f"train: images/train\n"
        f"val: images/val\n\n"
        f"names:\n{names}\n"
    )
    (out / "dataset.yaml").write_text(yaml_text, encoding="utf-8")


def main() -> None:
    ap = argparse.ArgumentParser(description="產生 YOLO 圓孔偵測資料集")
    ap.add_argument("--n", type=int, default=2000, help="樣本數")
    ap.add_argument("--out", default="datasets/grid_holes", help="輸出目錄")
    ap.add_argument("--val-split", type=float, default=0.2)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--n-preview", type=int, default=8)
    args = ap.parse_args()

    cfg = load_config()
    out = generate_dataset(cfg, args.n, args.out, args.val_split, args.seed, args.n_preview)
    n_val = int(args.n * args.val_split)
    print(f"完成：{args.n} 張（train {args.n - n_val} / val {n_val}）→ {out.resolve()}")
    print(f"訓練指令：yolo detect train data={out.as_posix()}/dataset.yaml "
          f"model=yolo11n.pt epochs=100 imgsz=1280")


if __name__ == "__main__":
    main()
