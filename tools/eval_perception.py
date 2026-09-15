# -*- coding: utf-8 -*-
"""端到端感知評估 —— 用訓練好的模型跑全鏈，對比合成真值。

流程：render 影像 → YoloDetector（真模型）→ localize_holes（整板 PnP）
     → 每孔瞄準 vs 真值。回報偵測召回、亮孔分類、端到端距離/方位誤差。

用法：
    python -m tools.eval_perception --weights runs/detect/grid_holes-2/weights/best.pt --scenes 50
"""
from __future__ import annotations

import argparse

import numpy as np

from rotbot.config import load_config
from rotbot.perception.detector import YoloDetector
from rotbot.perception.localize import localize_holes
from rotbot.sim.scene import SceneGenerator


def _match(aim_group, aim_idx, truth):
    return truth.get((aim_group, aim_idx))


def run(weights: str, scenes: int, seed: int = 123, conf: float = 0.25,
        imgsz: int = 960, device: str = "0"):
    cfg = load_config()
    K = cfg.camera.matrix()
    det = YoloDetector(weights, conf=conf, device=device, imgsz=imgsz)
    gen = SceneGenerator(cfg, rng=np.random.default_rng(seed), randomize=True)

    n_truth = n_det = n_board = 0
    lit_truth = lit_hit = 0
    d_err, b_err, e_err = [], [], []

    for _ in range(scenes):
        s = gen.sample()
        vis = [h for h in s.holes if h.visible]
        n_truth += len(vis)
        truth = {(h.group, h.idx): h for h in s.holes}

        dets = det.detect(s.image)
        n_det += len(dets)
        est = localize_holes(dets, cfg, K)
        if est.board_pose is not None:
            n_board += 1

        # 亮起加分孔分類是否正確帶出
        lit_h = next((h for h in vis if h.is_lit), None)
        if lit_h is not None:
            lit_truth += 1
            if est.lit_bonus is not None:
                lit_hit += 1

        for aim in est.aims:
            h = _match(aim.group, aim.idx, truth)
            if h is None or not h.visible:
                continue
            d_err.append(abs(aim.distance - h.distance) * 100)   # cm
            b_err.append(abs(aim.bearing - h.bearing))
            e_err.append(abs(aim.elev - h.elev))

    m = lambda x: float(np.mean(x)) if x else float("nan")
    p95 = lambda x: float(np.percentile(x, 95)) if x else float("nan")
    print("=" * 62)
    print(f"  端到端感知評估（{scenes} 場，真模型）")
    print("=" * 62)
    print(f"  可見孔數           {n_truth}")
    print(f"  偵測數             {n_det}（平均 {n_det / scenes:.1f}/場）")
    print(f"  整板 PnP 成功場次   {n_board}/{scenes}")
    print(f"  亮孔正確帶出        {lit_hit}/{lit_truth}")
    print("-" * 62)
    print(f"  距離誤差   平均 {m(d_err):5.1f} cm   95百分位 {p95(d_err):5.1f} cm")
    print(f"  方位誤差   平均 {m(b_err):5.2f}°    95百分位 {p95(b_err):5.2f}°")
    print(f"  俯仰誤差   平均 {m(e_err):5.2f}°    95百分位 {p95(e_err):5.2f}°")
    print("=" * 62)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", default="models/grid_holes_yolo11n.pt")
    ap.add_argument("--scenes", type=int, default=50)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--imgsz", type=int, default=960)
    ap.add_argument("--device", default="0")
    ap.add_argument("--seed", type=int, default=123)
    args = ap.parse_args()
    run(args.weights, args.scenes, seed=args.seed, conf=args.conf,
        imgsz=args.imgsz, device=args.device)


if __name__ == "__main__":
    main()
