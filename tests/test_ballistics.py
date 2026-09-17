# -*- coding: utf-8 -*-
import numpy as np

from rotbot.config import load_config
from rotbot.decision import ballistics


def test_ideal_elevation_reachable():
    cfg = load_config()
    pitch = ballistics.ideal_elevation(cfg.launch.v0_model, 2.5, 1.0, cfg.air.g)
    assert pitch is not None
    assert 0 < pitch < np.pi / 2


def test_ideal_elevation_out_of_range():
    # 初速太低、距離太遠 → 射程不足
    assert ballistics.ideal_elevation(3.0, 50.0, 5.0, 9.81) is None


def test_aim_lands_on_target_no_drag():
    """核心正確性：無阻力下，以解出的角度發射應恰好穿過目標點。"""
    cfg = load_config()
    g, v0 = cfg.air.g, cfg.launch.v0_model
    dist, dy = 2.5, 1.0
    theta = ballistics.ideal_elevation(v0, dist, dy, g)
    assert theta is not None
    # 無阻力彈道：y(x) = x*tanθ - g x² / (2 v0² cos²θ)
    y_at_target = dist * np.tan(theta) - g * dist ** 2 / (2 * v0 ** 2 * np.cos(theta) ** 2)
    assert abs(y_at_target - dy) < 1e-6


def test_aim_for_target_center_cell():
    cfg = load_config()
    aim = ballistics.aim_for_target((0.0, cfg.field.grid_cy), cfg)
    assert aim is not None
    pitch, yaw = aim
    assert abs(yaw) < 1e-9          # 正中心格：水平角應為 0
    assert pitch > 0
