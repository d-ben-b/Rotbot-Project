# -*- coding: utf-8 -*-
"""用一張真實場地照片（docs/preview_arena.png）產生 YOLO 訓練資料。

照片來源：主辦單位 IG 截圖（2026-09-23）。合成資料的加分孔沒有外框，實際的加分孔有一圈白色外框，
用舊模型在 6.9 m 的大小下 3 個加分孔都判成 main（docs/camera_yolo_risks.md「場地照片」）。

每張：
  1. 約 7 成的圖選一個加分孔，把白色外框塗成紅色並加光暈，標成 bonus_lit（亮燈的樣子是猜的，官方只說紅色 LED 燈圈）
  2. 縮放成 IMX296 在 5–8 m 看到的大小（一般孔 = fx × 0.4 / 距離），網板放在畫面隨機位置，旋轉 ±3°，四角各抖動 5%（視角）
  3. 亮度、gamma、白平衡、雜訊、模糊、JPEG 壓縮
標籤：12 個孔（9 main + 3 bonus），框是在截圖上手動校正的。

用法：
    python -m tools.augment_arena_photo --out datasets/arena_aug --train 400 --val 100
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

REPO_ROOT = Path(__file__).resolve().parent.parent
PHOTO = REPO_ROOT / "docs" / "preview_arena.png"
PHOTO_Y = (565, 2120)          # 截圖中照片的上下範圍（上面、下面是 IG 介面）
BONUS, LIT = 1, 2
# 截圖座標 (x1, y1, x2, y2)。bonus 框到白色外框的外緣
BOXES = [
    (BONUS, (349.2, 1016.1, 388.7, 1056.1)), (BONUS, (464.2, 1030.3, 501.6, 1070.0)),
    (BONUS, (572.7, 1043.6, 605.4, 1081.0)),
    (0, (340.3, 1076.0, 393.8, 1128.3)), (0, (455.1, 1087.9, 505.2, 1139.7)), (0, (563.4, 1099.2, 609.5, 1148.5)),
    (0, (338.3, 1155.0, 387.3, 1206.7)), (0, (454.0, 1164.2, 502.7, 1213.5)), (0, (560.0, 1171.4, 605.2, 1220.8)),
    (0, (337.1, 1235.9, 381.1, 1281.6)), (0, (450.0, 1239.3, 496.7, 1287.3)), (0, (557.3, 1247.3, 601.4, 1291.4)),
]


def light_ring(img: np.ndarray, box, rng: np.random.Generator) -> None:
    """把 box 內的白色外框塗成紅色、加光暈（原地修改）。"""
    x1, y1, x2, y2 = box
    cx, cy, r = (x1 + x2) / 2, (y1 + y2) / 2, (x2 - x1 + y2 - y1) / 4
    yy, xx = np.mgrid[0:img.shape[0], 0:img.shape[1]]
    rr = np.hypot(xx - cx, yy - cy)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    ring = (rr <= r * 1.1) & (rr >= r * 0.55) & (hsv[..., 2] > 120) & (hsv[..., 1] < 90)
    if rng.random() < 0.4:                                        # 燈光也照到孔內
        ring |= rr <= r * 0.75
    mask = ring.astype(np.float32)
    if rng.random() < 0.5:                                        # 深紅
        color = np.array([rng.uniform(0, 50), rng.uniform(0, 50), rng.uniform(190, 255)], np.float32)
    else:                                                         # 過曝：中間偏白、外圍紅
        color = np.array([rng.uniform(140, 210), rng.uniform(140, 210), 255], np.float32)
    out = img.astype(np.float32)
    out = out * (1 - mask[..., None]) + color * mask[..., None]
    glow = cv2.GaussianBlur(mask, (0, 0), max(1.0, r * rng.uniform(0.15, 0.45)))
    out[..., 2] += 255 * rng.uniform(0.3, 0.9) * glow             # 紅色光暈
    out[..., :2] += 60 * rng.uniform(0, 0.5) * glow[..., None]
    img[:] = np.clip(out, 0, 255).astype(np.uint8)


def ellipse_pts(box, n: int = 24) -> np.ndarray:
    x1, y1, x2, y2 = box
    t = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return np.stack([(x1 + x2) / 2 + (x2 - x1) / 2 * np.cos(t), (y1 + y2) / 2 + (y2 - y1) / 2 * np.sin(t)], 1)


def sample(photo: np.ndarray, rng: np.random.Generator, W: int = 1456, H: int = 1088,
           fx: float = 1794.1, dist: tuple[float, float] = (5.0, 8.0)):
    """回傳 (影像, [(cls, cx, cy, w, h) 正規化])。"""
    img = photo.copy()
    boxes = [(c, (b[0], b[1] - PHOTO_Y[0], b[2], b[3] - PHOTO_Y[0])) for c, b in BOXES]
    if rng.random() < 0.7:
        k = int(rng.integers(3))
        light_ring(img, boxes[k][1], rng)
        boxes[k] = (LIT, boxes[k][1])
    allb = np.array([b for _, b in boxes])
    main_px = np.mean([(b[2] - b[0] + b[3] - b[1]) / 2 for c, b in boxes if c == 0])
    c_src = np.array([allb[:, [0, 2]].mean(), allb[:, [1, 3]].mean()])
    s = fx * 0.40 / rng.uniform(*dist) / main_px
    th = np.radians(rng.uniform(-3, 3))
    R = s * np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    c_dst = np.array([rng.uniform(0.3, 0.7) * W, rng.uniform(0.3, 0.7) * H])
    x1, y1 = allb[:, 0].min() - main_px, allb[:, 1].min() - main_px
    x2, y2 = allb[:, 2].max() + main_px, allb[:, 3].max() + main_px
    src = np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]], np.float32)
    dst = (src - c_src) @ R.T + c_dst
    dst += rng.uniform(-0.05, 0.05, dst.shape) * np.array([x2 - x1, y2 - y1]) * s
    M = cv2.getPerspectiveTransform(src, dst.astype(np.float32))
    out = cv2.warpPerspective(img, M, (W, H), flags=cv2.INTER_CUBIC, borderMode=cv2.BORDER_REFLECT_101)

    labels = []
    for c, b in boxes:
        p = cv2.perspectiveTransform(ellipse_pts(b)[None].astype(np.float32), M)[0]
        bx1, by1 = p.min(0)
        bx2, by2 = p.max(0)
        area = (bx2 - bx1) * (by2 - by1)
        cx1, cy1, cx2, cy2 = max(bx1, 0), max(by1, 0), min(bx2, W), min(by2, H)
        if cx2 <= cx1 or cy2 <= cy1 or (cx2 - cx1) * (cy2 - cy1) < 0.6 * area:
            continue
        labels.append((c, (cx1 + cx2) / 2 / W, (cy1 + cy2) / 2 / H, (cx2 - cx1) / W, (cy2 - cy1) / H))

    f = out.astype(np.float32) * rng.uniform(0.85, 1.15, 3) * rng.uniform(0.55, 1.35)
    f = 255 * (np.clip(f, 0, 255) / 255) ** rng.uniform(0.75, 1.35)
    f += rng.normal(0, rng.uniform(0, 5), f.shape)
    out = np.clip(f, 0, 255).astype(np.uint8)
    if rng.random() < 0.5:
        out = cv2.GaussianBlur(out, (0, 0), rng.uniform(0.3, 1.2))
    ok, buf = cv2.imencode(".jpg", out, [cv2.IMWRITE_JPEG_QUALITY, int(rng.integers(55, 96))])
    return cv2.imdecode(buf, cv2.IMREAD_COLOR), labels


def write_split(photo, out: Path, split: str, n: int, seed: int) -> None:
    rng = np.random.default_rng(seed)
    (out / "images" / split).mkdir(parents=True, exist_ok=True)
    (out / "labels" / split).mkdir(parents=True, exist_ok=True)
    for i in range(n):
        img, labels = sample(photo, rng)
        cv2.imwrite(str(out / "images" / split / f"arena_{i:04d}.jpg"), img)
        (out / "labels" / split / f"arena_{i:04d}.txt").write_text(
            "".join(f"{c} {x:.6f} {y:.6f} {w:.6f} {h:.6f}\n" for c, x, y, w, h in labels))


def main() -> None:
    ap = argparse.ArgumentParser(description="用場地照片產生 YOLO 訓練資料")
    ap.add_argument("--out", default="datasets/arena_aug")
    ap.add_argument("--train", type=int, default=400)
    ap.add_argument("--val", type=int, default=100)
    ap.add_argument("--synthetic", default="datasets/grid_holes_6m", help="一起訓練的合成資料集（寫進 mix.yaml）")
    args = ap.parse_args()
    out = REPO_ROOT / args.out
    photo = cv2.imread(str(PHOTO))[PHOTO_Y[0]:PHOTO_Y[1]]
    write_split(photo, out, "train", args.train, seed=0)
    write_split(photo, out, "val", args.val, seed=1)
    syn = (REPO_ROOT / args.synthetic).as_posix()
    (out / "mix.yaml").write_text(
        f"# 合成資料 + 場地照片擴增（tools/augment_arena_photo.py）\n"
        f"train:\n  - {syn}/images/train\n  - {out.as_posix()}/images/train\n"
        f"val:\n  - {syn}/images/val\n  - {out.as_posix()}/images/val\n"
        "names:\n  0: main\n  1: bonus\n  2: bonus_lit\n", encoding="utf-8")
    print(f"{out}：train {args.train}、val {args.val}；混合資料集 {out / 'mix.yaml'}")


if __name__ == "__main__":
    main()
