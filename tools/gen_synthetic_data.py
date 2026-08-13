# -*- coding: utf-8 -*-
"""產生 YOLO-pose 訓練資料集 —— 把 sim.scene 的合成樣本轉成 Ultralytics 格式。

輸出結構（Ultralytics YOLO-pose 標準）：
    <out>/
      images/train/*.png   labels/train/*.txt
      images/val/*.png     labels/val/*.txt
      dataset.yaml
      preview/*.png        # 幾張疊上角點/bbox 的抽查圖

標籤每行：class  cx cy w h  x1 y1 v1  x2 y2 v2  x3 y3 v3  x4 y4 v4
    座標皆正規化到 [0,1]；角點順序 左上→右上→右下→左下；
    v=2 角點在畫面內、v=1 在畫面外但已標註。

用法：
    python -m tools.gen_synthetic_data --n 400 --out datasets/grid_pose
"""
from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

from rotbot.config import Config, load_config
from rotbot.sim.scene import CellLabel, SceneGenerator, SceneSample

# 類別對應（對手類別預留給後續，暫不產生）
CLASS_CELL = 0
CLASS_BONUS = 1
CLASS_NAMES = {0: "cell", 1: "bonus", 2: "opponent"}


def _clip01(v: float) -> float:
    return min(1.0, max(0.0, v))


def cell_to_line(cell: CellLabel, W: int, H: int) -> str | None:
    """把一格轉成一行 YOLO-pose 標籤；不在相機前方或完全出框回傳 None。"""
    if cell.z <= 0:
        return None
    xs, ys = cell.corners[:, 0], cell.corners[:, 1]
    # bbox 與畫面的交集
    ix1, iy1 = max(0.0, xs.min()), max(0.0, ys.min())
    ix2, iy2 = min(W, xs.max()), min(H, ys.max())
    if ix2 <= ix1 or iy2 <= iy1:
        return None
    cx, cy = (ix1 + ix2) / 2 / W, (iy1 + iy2) / 2 / H
    w, h = (ix2 - ix1) / W, (iy2 - iy1) / H

    cls = CLASS_BONUS if cell.is_bonus else CLASS_CELL
    parts = [str(cls), f"{cx:.6f}", f"{cy:.6f}", f"{w:.6f}", f"{h:.6f}"]
    for px, py in cell.corners:
        vis = 2 if (0 <= px < W and 0 <= py < H) else 1
        parts += [f"{_clip01(px / W):.6f}", f"{_clip01(py / H):.6f}", str(vis)]
    return " ".join(parts)


def sample_to_label(sample: SceneSample, W: int, H: int) -> str:
    """把一張樣本的所有格轉成標籤文字（多行）。"""
    lines = [ln for c in sample.cells if (ln := cell_to_line(c, W, H))]
    return "\n".join(lines)


def _annotate(sample: SceneSample) -> np.ndarray:
    img = sample.image.copy()
    for c in sample.cells:
        if c.z <= 0:
            continue
        col = (80, 255, 120) if c.is_bonus else (0, 180, 255)
        pts = c.corners.astype(int)
        cv2.polylines(img, [pts], True, col, 1)
        for j, (x, y) in enumerate(pts):
            cv2.circle(img, (int(x), int(y)), 4, col, -1)
            cv2.putText(img, str(j), (int(x) + 5, int(y)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, col, 1)
    return img


def generate_dataset(cfg: Config, n: int, out_dir: str | Path,
                     val_split: float = 0.2, seed: int = 0,
                     n_preview: int = 8) -> Path:
    """產生資料集，回傳輸出根目錄。"""
    out = Path(out_dir)
    W, H = cfg.camera.width, cfg.camera.height
    rng = np.random.default_rng(seed)
    gen = SceneGenerator(cfg, rng=rng, randomize=True)

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
        f"# YOLO-pose 資料集（由 tools/gen_synthetic_data.py 產生）\n"
        f"path: {out.resolve().as_posix()}\n"
        f"train: images/train\n"
        f"val: images/val\n\n"
        f"kpt_shape: [4, 3]        # 4 角點，每點 (x, y, visibility)\n"
        f"flip_idx: [1, 0, 3, 2]   # 水平翻轉時角點重新對應（左右互換）\n\n"
        f"names:\n{names}\n"
    )
    (out / "dataset.yaml").write_text(yaml_text, encoding="utf-8")


def main() -> None:
    ap = argparse.ArgumentParser(description="產生 YOLO-pose 合成資料集")
    ap.add_argument("--n", type=int, default=400, help="樣本數")
    ap.add_argument("--out", default="datasets/grid_pose", help="輸出目錄")
    ap.add_argument("--val-split", type=float, default=0.2)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--n-preview", type=int, default=8)
    args = ap.parse_args()

    cfg = load_config()
    out = generate_dataset(cfg, args.n, args.out, args.val_split, args.seed, args.n_preview)
    n_val = int(args.n * args.val_split)
    print(f"完成：{args.n} 張（train {args.n - n_val} / val {n_val}）→ {out.resolve()}")
    print(f"訓練指令：yolo pose train data={out.as_posix()}/dataset.yaml "
          f"model=yolo11n-pose.pt epochs=100 imgsz=1280")


if __name__ == "__main__":
    main()
