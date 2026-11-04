# -*- coding: utf-8 -*-
"""端到端感知評估 —— 用訓練好的模型跑全鏈，對比合成真值。

流程：render 影像 → YoloDetector（真模型）→ localize_holes（整板 PnP）
     → 每孔瞄準 vs 真值。回報偵測召回、亮孔分類、端到端距離/方位誤差。
平均值會被少數「孔編號配錯」的離群值拉高，所以同時列中位數、95 百分位與離群數。

用法：
    python -m tools.eval_perception --weights models/grid_holes_yolo11n.pt --scenes 50
    python -m tools.eval_perception --dist-min 5.5 --dist-max 6.5     # 只評 6m 附近
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
        imgsz: int = 960, device: str = "0",
        dist_range: tuple[float, float] | None = None, verbose: bool = True) -> dict:
    """跑評估並回傳指標 dict（同時印出表格）。"""
    cfg = load_config()
    K = cfg.camera.matrix()
    det = YoloDetector(weights, conf=conf, device=device, imgsz=imgsz)
    gen = SceneGenerator(cfg, rng=np.random.default_rng(seed), randomize=True,
                         dist_range=dist_range)

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

    stat = lambda x, f: float(f(x)) if x else float("nan")
    res = dict(
        scenes=scenes, dist_range=list(gen.dist_range), visible=n_truth, detections=n_det,
        board_pnp_ok=n_board, lit_truth=lit_truth, lit_hit=lit_hit,
        dist_cm_mean=stat(d_err, np.mean), dist_cm_median=stat(d_err, np.median),
        dist_cm_p95=stat(d_err, lambda x: np.percentile(x, 95)),
        bearing_deg_mean=stat(b_err, np.mean), bearing_deg_median=stat(b_err, np.median),
        bearing_deg_p95=stat(b_err, lambda x: np.percentile(x, 95)),
        elev_deg_mean=stat(e_err, np.mean), elev_deg_median=stat(e_err, np.median),
        elev_deg_p95=stat(e_err, lambda x: np.percentile(x, 95)),
        gross_outliers=int(sum(d > 50 for d in d_err)), n_aims=len(d_err),
    )
    if verbose:
        lo, hi = gen.dist_range
        print("=" * 66)
        print(f"  端到端感知評估（{scenes} 場，真模型，距離 {lo:.1f}~{hi:.1f} m）")
        print("=" * 66)
        print(f"  可見孔數           {n_truth}")
        print(f"  偵測數             {n_det}（平均 {n_det / scenes:.1f}/場）")
        print(f"  整板 PnP 成功場次   {n_board}/{scenes}")
        print(f"  亮孔正確帶出        {lit_hit}/{lit_truth}")
        print("-" * 66)
        print(f"  距離誤差   中位 {res['dist_cm_median']:5.1f} cm  平均 {res['dist_cm_mean']:5.1f} cm"
              f"  95百分位 {res['dist_cm_p95']:5.1f} cm")
        print(f"  方位誤差   中位 {res['bearing_deg_median']:5.2f}°   平均 {res['bearing_deg_mean']:5.2f}°"
              f"   95百分位 {res['bearing_deg_p95']:5.2f}°")
        print(f"  俯仰誤差   中位 {res['elev_deg_median']:5.2f}°   平均 {res['elev_deg_mean']:5.2f}°"
              f"   95百分位 {res['elev_deg_p95']:5.2f}°")
        print(f"  離群（距離誤差 >50cm，多為孔編號配錯）  {res['gross_outliers']}/{res['n_aims']}")
        print("=" * 66)
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", default="models/grid_holes_yolo11n.pt")
    ap.add_argument("--scenes", type=int, default=50)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--imgsz", type=int, default=960)
    ap.add_argument("--device", default="0")
    ap.add_argument("--seed", type=int, default=123)
    ap.add_argument("--dist-min", type=float, default=None, help="視角最近距離 (m)；省略用場景預設")
    ap.add_argument("--dist-max", type=float, default=None)
    args = ap.parse_args()
    dr = (args.dist_min, args.dist_max) if args.dist_min is not None and args.dist_max is not None else None
    run(args.weights, args.scenes, seed=args.seed, conf=args.conf,
        imgsz=args.imgsz, device=args.device, dist_range=dr)


if __name__ == "__main__":
    main()
