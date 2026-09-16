# -*- coding: utf-8 -*-
"""拋體模型 —— 機器人「內部」用來瞄準的解析模型（刻意忽略空氣阻力）。

真實世界的空氣阻力與旋轉造成的系統性偏差，交由 decision.calibration 線上校正。
原型：presentation/sim_adaptive_calibration.py 的 ideal_elevation()。
"""
from __future__ import annotations

import numpy as np

from ..config import Config


def ideal_elevation(v0: float, dist: float, dy: float, g: float) -> float | None:
    """無空氣阻力的理想拋體解（低平彈道解），回傳俯仰角 (rad)。

    v0: 出射速度 (m/s)；dist: 水平距離 (m)；dy: 目標相對發射口的高度差 (m)。
    射程不足時回傳 None。
    """
    disc = v0 ** 4 - g * (g * dist ** 2 + 2 * dy * v0 ** 2)
    if disc < 0:
        return None
    return float(np.arctan((v0 ** 2 - np.sqrt(disc)) / (g * dist)))


def aim_for_target(target_zy: tuple[float, float], cfg: Config) -> tuple[float, float] | None:
    """由目標的 (橫向 z, 高度 y) 位置，解出瞄準 (俯仰 rad, 水平 rad)。

    這是決策層對外的主要介面：給目標位置 → 得瞄準角。
    射程不足回傳 None。
    """
    z_t, y_t = target_zy
    d = float(np.hypot(cfg.field.dist_x, z_t))            # 水平斜距
    pitch = ideal_elevation(cfg.launch.v0_model, d, y_t - cfg.field.launch_h, cfg.air.g)
    if pitch is None:
        return None
    yaw = float(np.arctan2(z_t, cfg.field.dist_x))
    return pitch, yaw


def aim_at_point(target_launcher: tuple[float, float, float] | np.ndarray,
                 cfg: Config) -> tuple[float, float] | None:
    """對發射點系任意 3D 目標 (x前, y上, z右) 解瞄準 (俯仰 rad, 水平 rad)。

    圓孔版主用介面：目標 3D 由 frames.hole_target_point 得來（距離逐孔不同）。
    水平距離取 hypot(x, z)、高度差取 y；射程不足回傳 None。
    """
    x, y, z = np.asarray(target_launcher, float)
    horiz = float(np.hypot(x, z))
    pitch = ideal_elevation(cfg.launch.v0_model, horiz, y, cfg.air.g)
    if pitch is None:
        return None
    yaw = float(np.arctan2(z, x))
    return pitch, yaw


def estimate_sensitivity(shoot, target_zy: tuple[float, float], cfg: Config,
                         step_deg: float = 0.5) -> tuple[float, float]:
    """數值估計靈敏度 ∂高度/∂俯仰、∂橫向/∂水平 (m/rad)。

    用於把「落點位置誤差」換算成「角度修正量」。
    shoot: callable(pitch_rad, yaw_rad) -> (z, y) 落點；模擬時傳 sim 世界，
           實機時可用實測資料擬合。
    """
    z_t, y_t = target_zy
    d = float(np.hypot(cfg.field.dist_x, z_t))
    p0 = ideal_elevation(cfg.launch.v0_model, d, y_t - cfg.field.launch_h, cfg.air.g)
    yaw0 = float(np.arctan2(z_t, cfg.field.dist_x))
    h = np.radians(step_deg)

    _, y_a = shoot(p0 + h, yaw0)
    _, y_b = shoot(p0 - h, yaw0)
    z_a, _ = shoot(p0, yaw0 + h)
    z_b, _ = shoot(p0, yaw0 - h)
    j_pitch = (y_a - y_b) / (2 * h)
    j_yaw = (z_a - z_b) / (2 * h)
    return float(j_pitch), float(j_yaw)
