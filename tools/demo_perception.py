# -*- coding: utf-8 -*-
"""感知視覺 Demo —— 在合成影像上跑真模型，疊上「瞄準 HUD」。

流程：SceneGenerator（平滑掃描視角）→ YoloDetector → localize_holes
     → 每孔畫出橢圓框 + 編號 + 距離/方位；亮起加分孔標為優先目標。
輸出：一段掃描影片 (mp4) + 幾張靜態圖。

用法：
    python -m tools.demo_perception --weights runs/detect/grid_holes-2/weights/best.pt
"""
from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

from rotbot.config import load_config
from rotbot.perception.detector import YoloDetector
from rotbot.perception.localize import localize_holes
from rotbot.sim.scene import SceneGenerator

# HUD 配色 (BGR)
C_MAIN = (255, 210, 90)     # 一般孔：青
C_BONUS = (80, 200, 255)    # 加分孔（未亮）：橙
C_LIT = (90, 255, 130)      # 亮起加分孔：綠
C_TEXT = (245, 245, 245)
C_PANEL = (25, 25, 30)


def _sweep_viewpoints(n: int, cfg):
    """平滑掃描的一組視角：方位左右擺、距離前後、板略傾。"""
    for k in range(n):
        p = k / max(1, n - 1)
        ph = np.sin(2 * np.pi * p)                  # -1..1 週期
        yield dict(distance=3.0 + 0.2 * np.sin(2 * np.pi * p * 0.5),
                   bearing=10 * ph,
                   elev=-17 + 6 * np.cos(2 * np.pi * p * 0.5),   # 負仰角=相機往上仰，讓上排加分孔入鏡
                   board_yaw=12 * np.sin(2 * np.pi * p * 0.7),   # 板繞垂直軸傾，秀出橢圓形變
                   board_pitch=4 * ph,
                   roll=0.0)


def _draw_hud(img, dets, est, cfg):
    # localize 依偵測相同順序產生 aims，可直接 zip 對齊
    for det, aim in zip(dets, est.aims):
        x1, y1, x2, y2 = [int(v) for v in det.bbox]
        cx, cy = int(det.center[0]), int(det.center[1])
        ax, ay = (x2 - x1) // 2, (y2 - y1) // 2
        if det.is_lit:
            color, tag = C_LIT, "TARGET x2"
        elif det.is_bonus:
            color, tag = C_BONUS, "bonus"
        else:
            color, tag = C_MAIN, f"#{aim.idx}"
        cv2.ellipse(img, (cx, cy), (max(ax, 3), max(ay, 3)), 0, 0, 360, color, 2, cv2.LINE_AA)
        cv2.circle(img, (cx, cy), 3, color, -1)
        label = f"{tag}  {aim.distance:.2f}m  {aim.bearing:+.1f}°"
        cv2.putText(img, label, (cx - ax, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, color, 2, cv2.LINE_AA)
        if det.is_lit:                              # 優先目標十字準星
            cv2.drawMarker(img, (cx, cy), C_LIT, cv2.MARKER_CROSS, 40, 2)

    # 左上資訊面板
    lines = ["ROTBOT  perception (simulation)"]
    if est.board_pose is not None:
        d0, _, _ = est.board_pose.point_aim(np.zeros(3))
        lines.append(f"board center: {d0:.2f} m   holes: {est.board_pose.n_holes}"
                     f"   reproj: {est.board_pose.reproj_err:.1f}px")
    else:
        lines.append("board PnP: n/a (size fallback)")
    lit = est.lit_bonus
    lines.append(f"priority: lit bonus @ {lit.distance:.2f} m" if lit else "priority: (no lit bonus)")
    cv2.rectangle(img, (10, 10), (760, 20 + 26 * len(lines)), C_PANEL, -1)
    for i, ln in enumerate(lines):
        cv2.putText(img, ln, (22, 40 + 26 * i), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, C_TEXT, 2, cv2.LINE_AA)
    return img


def run(weights: str, frames: int, out_dir: str, device: str = "0",
        conf: float = 0.25, imgsz: int = 960):
    cfg = load_config()
    K = cfg.camera.matrix()
    det = YoloDetector(weights, conf=conf, device=device, imgsz=imgsz)
    # rim_jitter=False：孔緣中性灰，畫面只有 HUD 有顏色，語意清楚
    gen = SceneGenerator(cfg, rng=np.random.default_rng(7), rim_jitter=False)

    out = Path(out_dir)
    (out / "stills").mkdir(parents=True, exist_ok=True)
    W, H = cfg.camera.width, cfg.camera.height
    writer = cv2.VideoWriter(str(out / "demo.mp4"),
                             cv2.VideoWriter_fourcc(*"mp4v"), 12, (W, H))

    for i, vp in enumerate(_sweep_viewpoints(frames, cfg)):
        s = gen.sample(vp)
        dets = det.detect(s.image)
        est = localize_holes(dets, cfg, K)
        frame = _draw_hud(s.image.copy(), dets, est, cfg)
        writer.write(frame)
        if i % max(1, frames // 4) == 0:
            cv2.imwrite(str(out / "stills" / f"frame_{i:03d}.png"), frame)
    writer.release()
    print(f"完成：{frames} 幀 → {(out / 'demo.mp4').resolve()}")
    print(f"靜態圖：{(out / 'stills').resolve()}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", default="models/grid_holes_yolo11n.pt")
    ap.add_argument("--frames", type=int, default=72)
    ap.add_argument("--out", default="runs/demo")
    ap.add_argument("--device", default="0")
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--imgsz", type=int, default=960)
    args = ap.parse_args()
    run(args.weights, args.frames, args.out, device=args.device,
        conf=args.conf, imgsz=args.imgsz)


if __name__ == "__main__":
    main()
