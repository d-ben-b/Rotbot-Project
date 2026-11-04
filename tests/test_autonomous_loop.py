# -*- coding: utf-8 -*-
"""自主主迴圈（數位孿生）：轉塔運動學、相機外參，與「真韌體 + 合成相機 + 彈道世界」端到端。"""
from types import SimpleNamespace

import numpy as np
import pytest

from rotbot.config import load_config
from rotbot.decision import frames
from rotbot.perception.localize import localize_holes
from rotbot.sim.twin import TwinCamera, TwinDetector


class PoseMCU:
    """只回報轉塔角度的替身（測運動學用）。"""

    def __init__(self, pitch, yaw):
        self.pitch, self.yaw = pitch, yaw

    def get_status(self):
        return SimpleNamespace(pitch_deg=self.pitch, yaw_deg=self.yaw)


def test_turret_rotation_matches_ballistics_direction():
    for p, y in [(0, 0), (25, 0), (0, 15), (30, -12)]:
        fwd = frames.turret_rotation(p, y) @ np.array([1.0, 0, 0])
        pr, yr = np.radians(p), np.radians(y)
        want = [np.cos(pr) * np.cos(yr), np.sin(pr), np.cos(pr) * np.sin(yr)]
        assert np.allclose(fwd, want)


def test_coaxial_camera_kinematic_chain():
    """不同轉塔姿態下看到的孔，經 camera_extrinsic_at 還原到底座座標，應與真值重合（< 1 cm）。"""
    cfg = load_config()
    R_bl, t_bl = frames.board_to_launcher(cfg)
    truth = {(g, i): R_bl @ C + t_bl for (g, i, _d, C) in cfg.hole_centers()}
    survey_p, survey_y = 15.0, 0.0
    for dp, dy in [(0, 0), (2.5, 0), (-2, 3), (1, -4)]:
        mcu = PoseMCU(survey_p + dp, survey_y + dy)
        cam = TwinCamera(cfg, mcu)
        cam.read()
        est = localize_holes(TwinDetector(cam, noise_px=0.0).detect(None), cfg, cfg.camera.matrix())
        assert est.board_pose is not None
        extr = frames.camera_extrinsic_at(cfg, mcu.pitch, mcu.yaw)
        for a in est.aims:
            p = frames.hole_target_point_ext(a.distance, a.bearing, a.elev, *extr)
            assert np.linalg.norm(p - truth[(a.group, a.idx)]) < 0.01, (dp, dy, a)


def test_autonomous_loop_with_firmware():
    """自主迴圈 → SerialMCU → 韌體模擬器（轉塔/飛輪/推球）→ 彈道世界計分 → 線上校正。"""
    from tools.mcu_sil import CompilerNotFound, SilProcess, build
    try:
        build("full")
    except CompilerNotFound as e:
        pytest.skip(str(e))
    from rotbot.actuation.link import SerialMCU
    from rotbot.pipeline.loop import ControlLoop
    from rotbot.sim.closed_loop import practice_calibration
    from rotbot.sim.twin import TwinImpactObserver

    cfg = load_config()
    with SilProcess("full") as sil, SerialMCU(sil.url) as mcu:
        mcu.zero(0.0, 0.0)
        cam = TwinCamera(cfg, mcu, lit_bonus=1)
        loop = ControlLoop(cfg, cam, TwinDetector(cam, noise_px=0.5), mcu, rpm=2400,
                           calib=practice_calibration(cfg, np.random.default_rng(1), 9),
                           observe_impact=TwinImpactObserver(cfg, np.random.default_rng(2)))
        logs = loop.run(max_shots=8)
        st = mcu.get_status()
    assert len(logs) == 8 and st.shots == 8                       # 每一發都真的走到韌體推球
    assert logs[0].is_lit and logs[0].target.startswith("bonus")  # 前 60 秒先打亮孔
    assert all(l.error_cm is not None for l in logs)
    assert sum(l.hit for l in logs) >= 5
    assert np.mean([l.total_ms for l in logs]) < 6700            # 27 球 / 180 秒的節拍
