# -*- coding: utf-8 -*-
"""數位孿生 —— 讓自主主迴圈在沒有機器人的情況下，以「真韌體 + 合成相機 + 彈道世界」完整跑一場。

  TwinCamera        依 MCU 回報的轉塔實際角度算出網板在相機中的位姿 → 合成影像（轉到哪就看到哪）
  TwinDetector      讀 TwinCamera 上一張的真值 → Detection（可加像素雜訊；或改用真 YOLO 看影像）
  TwinImpactObserver 以 MCU 實際角度在 BallisticWorld 投出（含阻力、機構偏差、v0 抖動）→ 落點誤差與得分

MCU 可以是韌體模擬器（tools/mcu_sil.py）或真 Arduino（只接馬達、還沒有發射機構時也能跑）。
"""
from __future__ import annotations

import numpy as np

from ..config import Config
from ..decision import frames
from ..hal import Camera
from ..perception.detector import Detector
from .perfect_detector import detections_from_scene
from .scene import SceneGenerator
from .world import BallisticWorld


class TwinCamera(Camera):
    def __init__(self, cfg: Config, mcu, rng: np.random.Generator | None = None,
                 lit_bonus: int = 1, randomize: bool = False):
        self.cfg, self.mcu, self.lit_bonus = cfg, mcu, lit_bonus
        self.gen = SceneGenerator(cfg, rng=rng, randomize=randomize, rim_jitter=False)
        self.R_bl, self.t_bl = frames.board_to_launcher(cfg)
        self.last_sample = None

    def read(self) -> np.ndarray:
        st = self.mcu.get_status()
        R_cl, t_cl = frames.camera_extrinsic_at(self.cfg, st.pitch_deg, st.yaw_deg)
        R_bc = R_cl.T @ self.R_bl                       # board → cam
        t_bc = R_cl.T @ (self.t_bl - t_cl)
        self.last_sample = self.gen.sample_pose(R_bc, t_bc, lit_bonus=self.lit_bonus)
        return self.last_sample.image

    def intrinsics(self) -> np.ndarray:
        return self.cfg.camera.matrix()


class TwinDetector(Detector):
    def __init__(self, camera: TwinCamera, noise_px: float = 0.5,
                 rng: np.random.Generator | None = None):
        self.camera, self.noise_px = camera, noise_px
        self.rng = rng if rng is not None else np.random.default_rng(0)

    def detect(self, image):
        return detections_from_scene(self.camera.last_sample, noise_px=self.noise_px, rng=self.rng)


class TwinImpactObserver:
    """observe_impact(target, pitch_deg, yaw_deg) -> ((e_z, e_y), hit, points) 或 None（沒打到板面）。"""

    def __init__(self, cfg: Config, rng: np.random.Generator | None = None,
                 meas_noise_m: float | None = None):
        self.cfg = cfg
        self.rng = rng if rng is not None else np.random.default_rng(cfg.sim.seed)
        self.world = BallisticWorld(cfg, self.rng)
        self.meas = cfg.calibration.meas_noise_m if meas_noise_m is None else meas_noise_m
        self.truth = {(g, i): (d, cfg.field.grid_cx + C[0], cfg.field.grid_cy + C[1])
                      for (g, i, d, C) in cfg.hole_centers()}
        self.points = 0

    def __call__(self, target, pitch_deg: float, yaw_deg: float):
        land = self.world.shoot_real(np.radians(pitch_deg), np.radians(yaw_deg))
        if land is None:
            return None
        d, tz, ty = self.truth[(target.group, target.idx)]
        e_z, e_y = tz - land[0], ty - land[1]
        hit = bool(np.hypot(e_z, e_y) <= (d - self.cfg.ball.diameter) / 2)
        pts = (4 if target.is_lit else 2) if hit else 0
        self.points += pts
        noisy = (e_z + self.rng.normal(0, self.meas), e_y + self.rng.normal(0, self.meas))
        return noisy, hit, pts
