# -*- coding: utf-8 -*-
"""預估落點（手動操控的瞄準輔助）：模型自洽、畫面標記與數位孿生一致、校正後接近（模擬）真實落點。"""
import math

import numpy as np
import pytest

from rotbot.actuation.protocol import F_HOMED, Status
from rotbot.config import load_config
from rotbot.decision import ballistics, frames
from rotbot.sim.closed_loop import practice_calibration
from rotbot.sim.scene import SceneGenerator
from rotbot.sim.world import BallisticWorld
from rotbot.teleop.aim_preview import AimPreview, hole_label
from rotbot.teleop.server import TeleopServer

CFG = load_config()
RPM = CFG.launch.rpm_model


def _aim(group, idx):
    """AI 會下的瞄準角（度）。"""
    C = next(C for g, i, _, C in CFG.hole_centers() if (g, i) == (group, idx))
    zy = (CFG.field.grid_cx + C[0], CFG.field.grid_cy + C[1])
    return tuple(math.degrees(a) for a in ballistics.aim_for_target(zy, CFG))


def test_aimed_by_model_lands_in_that_hole():
    pv = AimPreview(CFG)
    for g, i, _, _ in CFG.hole_centers():
        r = pv.predict(*_aim(g, i), RPM)
        assert r.hole == f"{g}#{i}" and r.inside
        assert math.hypot(*r.off_cm) < 1.0


def test_marker_is_where_the_camera_sees_the_hole_not_the_image_center():
    """瞄準姿態下：標記像素 = 數位孿生畫面裡該孔孔心的像素；而且在畫面中心下方很遠（準星不是落點）。"""
    K = CFG.camera.matrix()
    gen = SceneGenerator(CFG, randomize=False, rim_jitter=False)
    R_bl, t_bl = frames.board_to_launcher(CFG)
    pv = AimPreview(CFG)
    for g, i in (("main", 4), ("main", 6), ("bonus", 2)):
        p, y = _aim(g, i)
        R_cl, t_cl = frames.camera_extrinsic_at(CFG, p, y)
        scene = gen.sample_pose(R_cl.T @ R_bl, R_cl.T @ (t_bl - t_cl), lit_bonus=None)
        hole = next(h for h in scene.holes if (h.group, h.idx) == (g, i))
        r = pv.predict(p, y, RPM, K)
        assert hole.visible
        assert np.hypot(*(np.asarray(r.px) - hole.center_px)) < 2.0
        assert r.px[1] - CFG.camera.cy > 250


def test_rpm_scales_v0_and_flywheel_off_means_no_shot():
    pv = AimPreview(CFG)
    assert pv.v0_for(RPM) == pytest.approx(CFG.launch.v0_model)
    p, y = _aim("main", 4)
    assert pv.predict(p, y, 0) is None
    assert pv.landing(p, y, RPM * 1.05)[1] > pv.landing(p, y, RPM * 0.95)[1]


def test_practice_calibration_makes_preview_match_simulated_world():
    """模擬世界（v0 13.44、Cd 0.50、安裝偏差）：沒校正時預估差約 20 cm；賽前練習 9 球的校正帶進來後 < 6 cm。"""
    world = BallisticWorld(CFG, np.random.default_rng(0))
    m = CFG.mechanism
    raw = AimPreview(CFG)
    cal = AimPreview(CFG, calib=lambda: practice_calibration(CFG, np.random.default_rng(3), 9))
    cal.calib = cal.calib()                       # 只算一次（也驗證 callable 介面）
    e_raw, e_cal = [], []
    for g, i, _, _ in CFG.hole_centers():
        p, y = _aim(g, i)
        rz, ry = world.shoot(math.radians(p + m.bias_pitch), math.radians(y + m.bias_yaw))
        for pv, errs in ((raw, e_raw), (cal, e_cal)):
            z, yy = pv.landing(p, y, RPM)
            errs.append(math.hypot(z - rz, yy - ry))
    assert np.median(e_raw) > 0.10
    assert max(e_cal) < 0.06


def test_draw_offscreen_arrow_and_no_shot():
    """落點在畫面外 → 邊緣畫箭頭；球到不了網板 → 小地圖寫 NO SHOT。兩種都不能讓串流執行緒出錯。"""
    K = CFG.camera.matrix()
    pv = AimPreview(CFG)
    r = pv.predict(45.0, 0.0, 1800, K)                     # 仰角大、轉速低：落點掉到畫面下緣外
    assert r is not None and not r.inside and r.px[1] > CFG.camera.height
    img = np.zeros((717, 960, 3), np.uint8)
    pv.draw(img, r, 960 / CFG.camera.width)
    assert img[690:717, 300:660].any()                     # 下緣有箭頭
    assert pv.predict(5.0, 0.0, RPM, K) is None            # 俯仰 5°：到網板前就落地
    img2 = np.zeros((717, 960, 3), np.uint8)
    pv.draw(img2, None, 960 / CFG.camera.width)
    assert img2[-40:, -100:].any() and not img2[:600, :700].any()


def test_hole_labels():
    assert [hole_label("main", i) for i in (0, 4, 8)] == ["上排左", "中排中", "下排右"]
    assert hole_label("bonus", 2) == "加分右"


def test_server_reports_preview_and_draws_hud():
    class StatusMCU:
        rtt_ms: list = []

        def __init__(self, pitch, yaw, rpm_target):
            self.status = Status(flags=F_HOMED, pitch_deg=pitch, yaw_deg=yaw, rpm=-1, rpm_target=rpm_target,
                                 shots=0, err=0, crc_errors=0)

    p, y = _aim("main", 4)
    server = TeleopServer(mcu=StatusMCU(p, y, 0), camera=None, preview=AimPreview(CFG), rpm_preset=RPM)
    info = server.snapshot()["preview"]
    assert info["ok"] and info["inside"] and info["label"] == "中排中" and info["fly_on"] is False
    server.mcu = StatusMCU(p - 3.0, y, RPM)                 # 低 3°：落在中排與下排之間，兩個都進不去
    info = server.snapshot()["preview"]
    assert info["fly_on"] and not info["inside"]
    assert info["label"] == "下排中" and info["off_cm"][1] > 20

    img = np.zeros((717, 960, 3), np.uint8)
    server._draw_hud(img, 960 / CFG.camera.width)          # 沒有相機：沒有畫面標記，但有小地圖
    assert img[-40:, -100:].any()
    assert TeleopServer(mcu=None, camera=None).snapshot()["preview"] is None


def test_preview_uses_measured_board_pose():
    """AI 量到的網板位置：和設定相同時結果一樣；網板往右 0.5 m 時落點在網板上往左 0.5 m；傾斜時落點仍在網板平面上。"""
    pv = AimPreview(CFG)
    p, y = _aim("main", 4)
    a = pv.predict(p, y, RPM)
    R_bl, t_bl = frames.board_to_launcher(CFG)
    b = pv.predict(p, y, RPM, board=(R_bl, t_bl))
    assert b.measured and not a.measured
    assert (b.z, b.y) == pytest.approx((a.z, a.y), abs=1e-6) and b.hole == "main#4"
    c = pv.predict(p, y, RPM, board=(R_bl, t_bl + np.array([0.0, 0.0, 0.5])))
    assert c.z == pytest.approx(a.z - 0.5, abs=0.005) and c.hole == "main#3"
    t = math.radians(5.0)                                   # 網板上緣往後倒 5°（安裝角約 85°）
    Rx = np.array([[1, 0, 0], [0, math.cos(t), -math.sin(t)], [0, math.sin(t), math.cos(t)]])
    P = pv.landing_point(p, y, RPM, board=(R_bl @ Rx, t_bl))
    assert abs((R_bl @ Rx)[:, 2] @ (P - t_bl)) < 1e-3
