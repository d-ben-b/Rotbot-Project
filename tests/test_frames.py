# -*- coding: utf-8 -*-
"""座標轉換：相機系量測 → 發射點系 3D 目標。"""
import numpy as np

from rotbot.config import load_config
from rotbot.decision import frames


def test_straight_ahead():
    cfg = load_config()
    p = frames.hole_target_point(2.5, 0.0, 0.0, cfg)   # 正前方
    assert abs(p[0] - 2.5) < 1e-6      # x 前 ≈ 距離
    assert abs(p[1]) < 1e-6            # y 上 ≈ 0
    assert abs(p[2]) < 1e-6            # z 右 ≈ 0


def test_right_and_up_signs():
    cfg = load_config()
    right = frames.hole_target_point(2.5, 10.0, 0.0, cfg)   # 方位右
    assert right[2] > 0                                     # z 右為正
    up = frames.hole_target_point(2.5, 0.0, 10.0, cfg)      # 俯仰上
    assert up[1] > 0                                        # y 上為正


def test_camera_point_roundtrip():
    """相機系點 → (距離/方位/俯仰) 定義一致（還原後再取角度應相符）。"""
    p = frames.aim_to_camera_point(3.0, 12.0, -5.0)
    dist = np.linalg.norm(p)
    bearing = np.degrees(np.arctan2(p[0], p[2]))
    elev = np.degrees(np.arctan2(-p[1], p[2]))
    assert abs(dist - 3.0) < 1e-6
    assert abs(bearing - 12.0) < 1e-6
    assert abs(elev - (-5.0)) < 1e-6
