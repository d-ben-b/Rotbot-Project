# -*- coding: utf-8 -*-
"""合成場景：驗證投影幾何與 pose 一致，以及輸出格式正確。"""
import numpy as np

from rotbot.config import load_config
from rotbot.perception.pose import solve_frame_pose
from rotbot.sim.scene import SceneGenerator


def test_sample_format():
    cfg = load_config()
    gen = SceneGenerator(cfg, rng=np.random.default_rng(0))
    s = gen.sample()
    assert s.image.shape == (cfg.camera.height, cfg.camera.width, 3)
    assert s.image.dtype == np.uint8
    assert len(s.cells) == 9
    assert all(c.corners.shape == (4, 2) for c in s.cells)
    assert sum(c.is_bonus for c in s.cells) <= 1


def test_canonical_center_cell_pnp():
    """canonical 視角下，中心格角點丟回 PnP 應解回 dist_x、正對。"""
    cfg = load_config()
    gen = SceneGenerator(cfg, randomize=False)
    s = gen.sample()
    center = s.cells[4]                       # 中心格
    assert center.visible
    pose = solve_frame_pose(center.corners, cfg.camera.matrix(), cfg.field.frame_d)
    assert pose is not None
    assert abs(pose.distance - cfg.field.dist_x) < 1e-2
    assert abs(pose.yaw_aim) < 0.5
    assert abs(pose.pitch_aim) < 0.5


def test_canonical_all_cells_visible():
    cfg = load_config()
    gen = SceneGenerator(cfg, randomize=False)
    s = gen.sample()
    assert all(c.visible for c in s.cells)   # 正對 2.5m 時九格應全在畫面內
