# -*- coding: utf-8 -*-
"""模擬一場比賽 —— 把 decision（瞄準+校正+策略）接上 sim.world（真實物理）。

這是 P4「模擬閉迴路」的最小雛形，也是校正演算法的回歸測試依據。
完整的端到端管線（含合成影像→YOLO→PnP）留待 P1/P4。

原型：presentation/sim_adaptive_calibration.py 的 run_match() + 蒙地卡羅段。
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..config import Config
from ..decision import ballistics, strategy
from ..decision.calibration import AdaptiveCalibrator
from .world import BallisticWorld


@dataclass
class MatchResult:
    errs: np.ndarray          # 每顆的徑向誤差 (m)，NaN = 未命中平面
    hits: np.ndarray          # 是否落入一般框
    bonus_hits: np.ndarray    # 是否落入加分格


def run_match(cfg: Config, rng: np.random.Generator, correct: bool = True) -> MatchResult:
    """跑一整場（cfg.n_balls 顆）；correct=False 為關閉校正的對照組。"""
    world = BallisticWorld(cfg, rng)
    cells = strategy.grid_cells(cfg)

    # 靈敏度以中心格為基準，用腦內模型估計（實機同樣只能用模型）
    j_pitch, j_yaw = ballistics.estimate_sensitivity(ballistics.model_shoot(cfg), cells[4], cfg)
    calib = AdaptiveCalibrator(cfg.calibration, j_pitch, j_yaw, n_cells=len(cells))

    errs, hits, bonus_hits = [], [], []
    for k in range(cfg.n_balls):
        idx = strategy.select_next_target(cfg, k)
        target = cells[idx]

        aim = ballistics.aim_for_target(target, cfg)
        if aim is None:
            errs.append(np.nan); hits.append(False); bonus_hits.append(False)
            continue
        pitch, yaw = aim
        if correct:
            dp, dy = calib.correction(idx)
            pitch += dp
            yaw += dy

        land = world.shoot_real(pitch, yaw)
        if land is None:
            errs.append(np.nan); hits.append(False); bonus_hits.append(False)
            continue

        z_l, y_l = land
        e_z, e_y = target[0] - z_l, target[1] - y_l
        radial = float(np.hypot(e_z, e_y))
        errs.append(radial)
        hits.append(radial <= cfg.tol_std)
        bonus_hits.append(radial <= cfg.tol_bonus)

        if correct:
            m_z = e_z + rng.normal(0, cfg.calibration.meas_noise_m)   # 相機量測雜訊
            m_y = e_y + rng.normal(0, cfg.calibration.meas_noise_m)
            calib.update(idx, (m_z, m_y))

    return MatchResult(np.array(errs), np.array(hits), np.array(bonus_hits))


def monte_carlo(cfg: Config, n_trials: int = 100, seed: int | None = None):
    """回傳 (校正後命中矩陣, 未校正命中矩陣, 校正後加分命中矩陣)。"""
    rng = np.random.default_rng(cfg.sim.seed if seed is None else seed)
    h_on = np.zeros((n_trials, cfg.n_balls), bool)
    h_off = np.zeros((n_trials, cfg.n_balls), bool)
    b_on = np.zeros((n_trials, cfg.n_balls), bool)
    for t in range(n_trials):
        on = run_match(cfg, rng, correct=True)
        off = run_match(cfg, rng, correct=False)
        h_on[t], b_on[t] = on.hits, on.bonus_hits
        h_off[t] = off.hits
    return h_on, h_off, b_on
