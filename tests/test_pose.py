# -*- coding: utf-8 -*-
"""PnP 位姿：真值 → 投影 → 反解，驗證幾何與程式正確。

搬自 presentation/pnp_frame_pose.py 的自我驗證案例。
"""
import numpy as np
import pytest

from rotbot.config import load_config
from rotbot.perception.pose import project_frame_corners, solve_frame_pose

FRAME = 0.40  # 網框邊長 (m)


@pytest.fixture
def K():
    return load_config().camera.matrix()


@pytest.mark.parametrize("distance,bearing,elev,tilt", [
    (2.50, 0.0, 0.0, 0.0),      # A. 正前方
    (3.00, 15.0, 10.0, 0.0),    # B. 右偏 15° 仰 10°
    (2.50, -8.0, 5.0, 25.0),    # C. 網框被打歪 25°
])
def test_solve_roundtrip_noise_free(K, distance, bearing, elev, tilt):
    """無雜訊時，反解應精確還原真值。

    注意：瞄準角是相機視線方向角，bearing≠0 時
    pitch_aim = arctan(tan(elev)/cos(bearing))，並不等於 elev。
    """
    pts = project_frame_corners(K, FRAME, distance, bearing, elev, tilt)
    pose = solve_frame_pose(pts, K, FRAME)
    b, e = np.radians(bearing), np.radians(elev)
    exp_yaw = bearing
    exp_pitch = np.degrees(np.arctan2(np.sin(e), np.cos(b) * np.cos(e)))
    assert pose is not None
    assert abs(pose.distance - distance) < 1e-3
    assert abs(pose.yaw_aim - exp_yaw) < 1e-2
    assert abs(pose.pitch_aim - exp_pitch) < 1e-2
    assert abs(pose.tilt - tilt) < 1e-1
    assert pose.reproj_err < 1e-3


def test_solve_with_detection_noise(K):
    """1.5px 偵測雜訊下，距離誤差仍在可預期的合理範圍。"""
    pts = project_frame_corners(K, FRAME, 2.50, 0.0, 0.0, 0.0, noise_px=1.5, seed=0)
    pose = solve_frame_pose(pts, K, FRAME)
    assert pose is not None
    assert abs(pose.distance - 2.50) < 0.1   # 10 cm 內
