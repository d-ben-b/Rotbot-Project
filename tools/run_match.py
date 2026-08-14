# -*- coding: utf-8 -*-
"""跑模擬閉迴路一場比賽並印成績（P4）。

用法：
    python -m tools.run_match --trials 50          # 校正 vs 未校正 蒙地卡羅比較
    python -m tools.run_match --one --seed 0       # 只跑一場、印每球明細
"""
from __future__ import annotations

import argparse

import numpy as np

from rotbot.config import load_config
from rotbot.sim.closed_loop import monte_carlo, run_closed_loop


def _print_one(cfg, seed):
    r = run_closed_loop(cfg, np.random.default_rng(seed), correct=True)
    print("=" * 60)
    print(f"  單場閉迴路（seed={seed}，含校正）")
    print("=" * 60)
    for s in r.shots:
        tag = "★亮孔x2" if s.is_lit else f"{s.group}#{s.idx}"
        mark = "○命中" if s.hit else "×未中"
        err = "  -  " if np.isnan(s.error_m) else f"{s.error_m * 100:4.1f}cm"
        print(f"  第{s.ball:2d}球 {tag:10s} {mark}  誤差{err}  +{s.points}分")
    print("-" * 60)
    print(f"  總分 {r.total_points}   命中率 {r.hit_rate * 100:.0f}%   "
          f"加分孔命中 {r.bonus_hit_rate * 100:.0f}%")
    print("=" * 60)


def _print_mc(cfg, trials):
    pon, hon, bon = monte_carlo(cfg, trials, correct=True)
    poff, hoff, boff = monte_carlo(cfg, trials, correct=False)
    print("=" * 60)
    print(f"  閉迴路蒙地卡羅（{trials} 場）— 感知→決策→致動→物理→校正")
    print("=" * 60)
    print(f"  {'':10s}{'總分':>8}{'命中率':>10}{'加分孔命中':>12}")
    print(f"  {'未校正':10s}{poff.mean():>8.0f}{hoff.mean() * 100:>9.0f}%{boff.mean() * 100:>11.0f}%")
    print(f"  {'校正後':10s}{pon.mean():>8.0f}{hon.mean() * 100:>9.0f}%{bon.mean() * 100:>11.0f}%")
    print("-" * 60)
    print("  ★ 校正的最大價值在 Ø20 加分孔（容許誤差最小、最吃校正）")
    print("=" * 60)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=50)
    ap.add_argument("--one", action="store_true", help="只跑一場並印每球明細")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()
    cfg = load_config()
    if args.one:
        _print_one(cfg, args.seed)
    else:
        _print_mc(cfg, args.trials)


if __name__ == "__main__":
    main()
