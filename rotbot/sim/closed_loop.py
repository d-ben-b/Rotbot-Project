# -*- coding: utf-8 -*-
"""模擬閉迴路（P4）—— 感知 → 決策 → 致動 → 物理落點 → 校正，跑完整一場。

把各層串成一場 27 球的比賽（圓孔版）：
  1. 感知（模擬）：由真值孔位算相機量測 (距離/方位/俯仰) 並加雜訊 → HoleAim
  2. 決策：plan_next_shot 選目標（亮孔優先）並算俯仰/水平
  3. 致動：AimCommand 經協定送 MockArduino（驗證通訊路徑）
  4. 物理：BallisticWorld 含空氣阻力 + 機構偏差投射 → 落點
  5. 計分 + 線上校正：依落點誤差微調，後續逐步收斂（「找手感」）

這是硬體到位前的「軟體完成」里程碑：一鍵跑完整場並產出成績。
真實影像的 YOLO 感知已於 P1 端到端驗證；此處感知用真值+雜訊以聚焦下游閉迴路。
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..actuation.link import MockArduino
from ..config import Config
from ..decision import ballistics, frames
from ..decision.calibration import AdaptiveCalibrator
from ..decision.strategy import ShotPlanner, plan_next_shot
from ..hal import AimCommand
from ..perception.localize import HoleAim
from .world import BallisticWorld


def _hole_int_index(group: str, idx: int) -> int:
    """孔 → 校正器用的整數索引（main 0~8、bonus 9~11）。"""
    return idx if group == "main" else 9 + idx


@dataclass
class ShotResult:
    ball: int
    group: str
    idx: int
    is_lit: bool
    error_m: float          # 落點徑向誤差；NaN=未命中平面
    hit: bool
    points: int


@dataclass
class MatchReport:
    shots: list[ShotResult] = field(default_factory=list)

    @property
    def total_points(self) -> int:
        return sum(s.points for s in self.shots)

    @property
    def hit_rate(self) -> float:
        return np.mean([s.hit for s in self.shots]) if self.shots else 0.0

    @property
    def bonus_hit_rate(self) -> float:
        b = [s.hit for s in self.shots if s.is_lit]
        return float(np.mean(b)) if b else 0.0

    @property
    def errors_cm(self) -> np.ndarray:
        return np.array([s.error_m for s in self.shots]) * 100


def _truth_holes(cfg: Config):
    """回傳每孔真值：(group, idx, diameter, 橫向 z, 絕對高度 y)。"""
    out = []
    for (group, idx, diam, C) in cfg.hole_centers():
        z = cfg.field.grid_cx + C[0]       # 橫向位置
        y_abs = cfg.field.grid_cy + C[1]   # 離地高度
        out.append((group, idx, diam, z, y_abs))
    return out


def run_closed_loop(cfg: Config, rng: np.random.Generator, correct: bool = True,
                    noise_d: float = 0.03, noise_ang: float = 0.5) -> MatchReport:
    """跑一整場閉迴路；correct=False 為關閉線上校正的對照組。"""
    holes = _truth_holes(cfg)
    truth = {(g, i): (diam, z, y) for (g, i, diam, z, y) in holes}
    lit_idx = int(rng.integers(0, 3))                  # 隨機亮起一個加分孔

    world = BallisticWorld(cfg, rng)
    j_pitch, j_yaw = ballistics.estimate_sensitivity(
        world.shoot, (0.0, cfg.field.grid_cy), cfg)
    calib = AdaptiveCalibrator(cfg.calibration, j_pitch, j_yaw, n_cells=12)
    planner = ShotPlanner(cfg)
    arduino = MockArduino()
    report = MatchReport()

    while planner.remaining > 0:
        # 1. 感知（模擬）：真值孔位 → 相機量測 + 雜訊
        aims = []
        for (g, i, diam, z, y_abs) in holes:
            xf, yu, zr = cfg.field.dist_x, y_abs - cfg.field.launch_h, z
            d = float(np.linalg.norm([xf, yu, zr])) + rng.normal(0, noise_d)
            b = np.degrees(np.arctan2(zr, xf)) + rng.normal(0, noise_ang)
            e = np.degrees(np.arctan2(yu, xf)) + rng.normal(0, noise_ang)
            aims.append(HoleAim(group=g, idx=i, is_bonus=(g == "bonus"),
                                is_lit=(g == "bonus" and i == lit_idx),
                                distance=d, bearing=b, elev=e, source="sim"))

        # 2. 決策
        plan = plan_next_shot(aims, planner, cfg)
        if plan is None:
            break
        t = plan.target
        hidx = _hole_int_index(t.group, t.idx)
        pitch, yaw = np.radians(plan.pitch_deg), np.radians(plan.yaw_deg)
        if correct:                                    # 疊加已學到的校正
            dp, dy = calib.correction(hidx)
            pitch += dp
            yaw += dy

        # 3. 致動（經協定送 MockArduino，驗證通訊路徑）
        arduino.send(AimCommand(pitch_deg=np.degrees(pitch), yaw_deg=np.degrees(yaw),
                                flywheel=cfg.launch.v0_model, trigger=True))

        # 4. 物理（真實世界：含阻力 + 機構偏差）
        land = world.shoot_real(pitch, yaw)
        planner.record(t)
        diam, tz, ty = truth[(t.group, t.idx)]
        tol = (diam - cfg.ball.diameter) / 2

        if land is None:
            report.shots.append(ShotResult(report.shots.__len__() + 1, t.group, t.idx,
                                           t.is_lit, float("nan"), False, 0))
            continue
        lz, ly = land
        e_z, e_y = tz - lz, ty - ly
        err = float(np.hypot(e_z, e_y))
        hit = err <= tol
        pts = (4 if t.is_lit else 2) if hit else 0
        report.shots.append(ShotResult(len(report.shots) + 1, t.group, t.idx,
                                       t.is_lit, err, hit, pts))

        # 5. 線上校正（用帶雜訊的量測落點）
        if correct:
            m_z = e_z + rng.normal(0, cfg.calibration.meas_noise_m)
            m_y = e_y + rng.normal(0, cfg.calibration.meas_noise_m)
            calib.update(hidx, (m_z, m_y))

    assert len(arduino.history) == len(report.shots)   # 每發都確實走過通訊路徑
    return report


def monte_carlo(cfg: Config, n_trials: int = 30, correct: bool = True,
                seed: int | None = None):
    """回傳 (每場總分, 每場命中率, 每場加分孔命中率)。"""
    rng = np.random.default_rng(cfg.sim.seed if seed is None else seed)
    pts, hr, bhr = [], [], []
    for _ in range(n_trials):
        r = run_closed_loop(cfg, rng, correct=correct)
        pts.append(r.total_points)
        hr.append(r.hit_rate)
        bhr.append(r.bonus_hit_rate)
    return np.array(pts), np.array(hr), np.array(bhr)
