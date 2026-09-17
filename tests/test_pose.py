# -*- coding: utf-8 -*-
"""圓孔位姿：方法 A（中心整板 PnP + 尺寸測距）與方法 B（橢圓圓錐反投影）。"""
import numpy as np
import pytest

from rotbot.config import load_config
from rotbot.perception import pose
from rotbot.sim.scene import SceneGenerator


@pytest.fixture
def cfg():
    return load_config()


# ── 方法 A：整板 PnP ──────────────────────────────────
def test_board_pose_recovers_distance(cfg):
    s = SceneGenerator(cfg, randomize=False).sample()
    mains = [h for h in s.holes if h.group == "main"]
    obj = np.array([h.center_board for h in mains])
    img = np.array([h.center_px for h in mains])
    bp = pose.solve_board_pose(obj, img, cfg.camera.matrix())
    assert bp is not None
    assert bp.reproj_err < 1.0
    d, b, e = bp.point_aim(np.zeros(3))            # 中心主孔在板原點
    assert abs(d - cfg.field.dist_x) < 1e-2
    assert abs(b) < 0.5 and abs(e) < 0.5


def test_board_pose_needs_min_points(cfg):
    obj = np.zeros((3, 3)); img = np.zeros((3, 2))
    assert pose.solve_board_pose(obj, img, cfg.camera.matrix()) is None


def test_distance_from_diameter(cfg):
    fx = cfg.camera.fx
    # 直徑 40cm、深度 2.5m → 表觀直徑 fx*0.4/2.5，反算回 2.5m
    d_px = fx * 0.40 / 2.5
    assert abs(pose.distance_from_diameter(d_px, 0.40, fx) - 2.5) < 1e-6


# ── 方法 B：橢圓圓錐反投影 ─────────────────────────────
def test_conic_pose_canonical(cfg):
    s = SceneGenerator(cfg, randomize=False).sample()
    center = next(h for h in s.holes if h.group == "main" and h.idx == 4)
    cp = pose.pose_from_ellipse(center.ellipse, cfg.camera.matrix(), center.diameter / 2)
    assert cp is not None
    assert abs(cp.distance - cfg.field.dist_x) < 1e-2
    assert abs(cp.tilt) < 0.5


def test_conic_pose_matches_truth_over_views(cfg):
    """隨機視角下（無雜訊橢圓），方法 B 應貼近真值距離與傾角。"""
    gen = SceneGenerator(cfg, rng=np.random.default_rng(1))
    d_err, t_err = [], []
    for _ in range(15):
        for h in gen.sample().holes:
            if h.visible and h.ellipse is not None:
                cp = pose.pose_from_ellipse(h.ellipse, cfg.camera.matrix(), h.diameter / 2)
                if cp is not None:
                    d_err.append(abs(cp.distance - h.distance))
                    t_err.append(abs(cp.tilt - h.tilt))
    assert np.median(d_err) < 0.02          # < 2 cm
    assert np.median(t_err) < 2.0           # < 2°
