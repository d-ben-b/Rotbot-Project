# -*- coding: utf-8 -*-
"""比較兩種圓孔姿態/測距方法（在合成真值上，注入偵測雜訊）。

方法 A：孔中心整板 PnP（多孔過定）＋ 表觀直徑單孔測距。
方法 B：橢圓擬合 → 圓錐反投影（單孔，含傾角）。

對每個雜訊等級，統計距離/方位/俯仰(/傾角)的平均絕對誤差。
σ=0 時方法 B 誤差應趨近 0（驗證圓錐解算器正確）。

用法：python -m tools.compare_pose_methods --scenes 60
"""
from __future__ import annotations

import argparse

import cv2
import numpy as np

from rotbot.config import load_config
from rotbot.perception import pose
from rotbot.sim.scene import SceneGenerator


def _noisy_ellipse(rim_px, sigma, rng):
    pts = rim_px + rng.normal(0, sigma, rim_px.shape)
    try:
        return cv2.fitEllipse(pts.astype(np.float32))
    except cv2.error:
        return None


def run(scenes: int, sigmas=(0.0, 1.0, 2.0, 3.0), seed: int = 0):
    cfg = load_config()
    K = cfg.camera.matrix()
    fx = cfg.camera.fx
    gen = SceneGenerator(cfg, rng=np.random.default_rng(seed), randomize=True)
    samples = [gen.sample() for _ in range(scenes)]

    print("=" * 82)
    print("  圓孔姿態/測距方法比較（合成真值 + 偵測雜訊，僅計可見孔）")
    print("=" * 82)
    header = (f"{'σ(px)':>6} | {'A-PnP 距離cm':>11} {'方位°':>7} {'俯仰°':>7} | "
              f"{'A-尺寸 深度cm':>12} | {'B 距離cm':>9} {'方位°':>7} {'俯仰°':>7} {'傾角°':>7}")
    print(header)
    print("-" * 82)

    for sigma in sigmas:
        rng = np.random.default_rng(1000 + int(sigma * 10))
        a_d, a_b, a_e, a_size, b_d, b_b, b_e, b_t = ([] for _ in range(8))

        for s in samples:
            vis = [h for h in s.holes if h.visible and h.ellipse is not None]
            if len(vis) < 4:
                continue
            # 共用雜訊：擾動 rim → 重新擬合橢圓，取其中心
            noisy = {}
            for h in vis:
                el = _noisy_ellipse(h.rim_px, sigma, rng) if sigma > 0 else h.ellipse
                if el is not None:
                    noisy[id(h)] = el

            # 方法 A：整板 PnP（用觀測到的橢圓中心）
            obj = np.array([h.center_board for h in vis if id(h) in noisy])
            img = np.array([noisy[id(h)][0] for h in vis if id(h) in noisy])
            bp = pose.solve_board_pose(obj, img, K)
            if bp is not None:
                for h in vis:
                    if id(h) not in noisy:
                        continue
                    d, b, e = bp.point_aim(h.center_board)
                    a_d.append(abs(d - h.distance) * 100)
                    a_b.append(abs(b - h.bearing))
                    a_e.append(abs(e - h.elev))

            # 方法 A-尺寸 與 方法 B：逐孔
            for h in vis:
                el = noisy.get(id(h))
                if el is None:
                    continue
                major = max(el[1])                       # 橢圓長軸 ≈ 表觀直徑
                depth_sz = pose.distance_from_diameter(major, h.diameter, fx)
                a_size.append(abs(depth_sz - h.center_cam[2]) * 100)  # 尺寸法給的是深度 Z

                cp = pose.pose_from_ellipse(el, K, h.diameter / 2)
                if cp is not None:
                    b_d.append(abs(cp.distance - h.distance) * 100)
                    b_b.append(abs(cp.bearing - h.bearing))
                    b_e.append(abs(cp.elev - h.elev))
                    b_t.append(abs(cp.tilt - h.tilt))

        m = lambda x: np.mean(x) if x else float("nan")
        print(f"{sigma:>6.1f} | {m(a_d):>11.1f} {m(a_b):>7.2f} {m(a_e):>7.2f} | "
              f"{m(a_size):>12.1f} | {m(b_d):>9.1f} {m(b_b):>7.2f} {m(b_e):>7.2f} {m(b_t):>7.2f}")

    print("-" * 82)
    print("  A-PnP：多孔過定，方位/俯仰最穩；A-尺寸：單孔備援；B：單孔且能回傳傾角")
    print("=" * 82)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenes", type=int, default=60)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()
    run(args.scenes, seed=args.seed)


if __name__ == "__main__":
    main()
