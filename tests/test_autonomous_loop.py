# -*- coding: utf-8 -*-
"""自主主迴圈（數位孿生）：轉塔運動學、相機外參，與「真韌體 + 合成相機 + 彈道世界」端到端。"""
import time
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


class FakeMCU:
    """轉塔立刻到位、發射立刻完成的替身（測主迴圈的流程，不測韌體）。"""

    def __init__(self):
        self.pitch = self.yaw = 0.0
        self.shots = 0

    def aim(self, pitch, yaw):
        self.pitch, self.yaw = pitch, yaw

    def flywheel(self, rpm):
        pass

    def get_status(self):
        return SimpleNamespace(pitch_deg=self.pitch, yaw_deg=self.yaw, moving=False)

    def wait_ready(self, timeout):
        pass

    def event_mark(self):
        return 0

    def fire(self):
        self.shots += 1

    def wait_event(self, name, timeout, since):
        pass


class FlakyDetector:
    """依呼叫順序決定這次看不看得到（False = 回傳空清單，例如有人擋住）；用完之後都看得到。"""

    def __init__(self, inner, pattern):
        self.inner, self.pattern, self.calls = inner, list(pattern), 0

    def detect(self, image):
        ok = self.pattern[self.calls] if self.calls < len(self.pattern) else True
        self.calls += 1
        return self.inner.detect(image) if ok else []


def _miss_steps(n):
    """n 次 step 都看不到板：每次 perceive 拍 3 張（perceive_retries=2）。"""
    return [False] * (3 * n)


def test_scattered_misses_do_not_stop_the_loop():
    """看不到板的次數累計超過 5 次、但中間有看到時，主迴圈照樣投（以前累計 6 次就停）。"""
    from rotbot.pipeline.loop import ControlLoop
    cfg = load_config()
    mcu = FakeMCU()
    cam = TwinCamera(cfg, mcu, lit_bonus=1)
    det = FlakyDetector(TwinDetector(cam, noise_px=0.0), (_miss_steps(3) + [True]) * 2)
    loop = ControlLoop(cfg, cam, det, mcu, rpm=2400, reuse_board=False)   # 只測計數，不沿用上次的網板
    logs = loop.run(max_shots=3)
    assert len(logs) == 3 and mcu.shots == 3
    assert loop.skipped_total == 6 and loop.skipped == 0


def test_runner_keeps_trying_after_consecutive_misses():
    """連續 7 次看不到板：AI 不放棄（網頁顯示原因），看到板之後繼續投。"""
    from rotbot.pipeline.loop import AutonomousRunner, ControlLoop
    from rotbot.pipeline.state_machine import GameState, MatchStateMachine
    cfg = load_config()
    mcu = FakeMCU()
    cam = TwinCamera(cfg, mcu, lit_bonus=1)
    det = FlakyDetector(TwinDetector(cam, noise_px=0.0), _miss_steps(7))
    sm = MatchStateMachine(auto_s=60.0)
    runner = AutonomousRunner(sm, lambda: ControlLoop(
        cfg, cam, det, mcu, rpm=2400, sm=sm, is_active=lambda: sm.update() == GameState.AUTONOMOUS))
    sm.start()
    runner.start()
    warnings = set()
    try:
        end = time.monotonic() + 15.0
        while mcu.shots < 2 and time.monotonic() < end:
            warnings.add(runner.warning())
            time.sleep(0.02)
    finally:
        runner.stop()
    assert mcu.shots >= 2, runner.errors
    assert any("看不到網板" in w for w in warnings)
    assert runner.loop.skipped_total == 7


class DyingTwinCamera:
    """數位孿生相機，前 n 次正常，之後每次逾時（相機在比賽中停掉）。"""

    def __init__(self, inner, n):
        self.inner, self.n, self.calls = inner, n, 0

    def read(self):
        self.calls += 1
        if self.calls > self.n:
            raise RuntimeError("SharedCamera：2.0 s 內沒有新畫面")
        return self.inner.read()

    def intrinsics(self):
        return self.inner.intrinsics()


def _run_six(camera_ok_reads):
    """6 球；相機前 camera_ok_reads 次正常（None = 一直正常）。回傳 logs。"""
    from rotbot.pipeline.loop import ControlLoop
    from rotbot.sim.closed_loop import practice_calibration
    from rotbot.sim.twin import TwinImpactObserver
    cfg = load_config()
    mcu = FakeMCU()
    twin = TwinCamera(cfg, mcu, lit_bonus=1)
    cam = twin if camera_ok_reads is None else DyingTwinCamera(twin, camera_ok_reads)
    loop = ControlLoop(cfg, cam, TwinDetector(twin, noise_px=0.0), mcu, rpm=2400,
                       calib=practice_calibration(cfg, np.random.default_rng(1), 9),
                       observe_impact=TwinImpactObserver(cfg, np.random.default_rng(2)))
    return loop.run(max_shots=6)


def test_camera_dies_mid_match_ai_keeps_shooting_at_last_seen_board():
    """第 1 球之後相機停掉：AI 沿用第 1 球看到的網板繼續投；瞄準與相機正常時完全相同（底盤和網板沒動）。"""
    healthy, dying = _run_six(None), _run_six(1)
    assert len(dying) == 6
    assert [l.source for l in dying] == ["camera"] + ["last_seen"] * 5
    assert [l.target for l in dying] == [l.target for l in healthy]
    assert np.allclose([(l.pitch_deg, l.yaw_deg) for l in dying],
                       [(l.pitch_deg, l.yaw_deg) for l in healthy])
    assert [l.hit for l in dying] == [l.hit for l in healthy]


class AgedTwinCamera:
    """數位孿生相機 + 可設定的 age()（SharedCamera 的介面），模擬相機停掉。"""

    def __init__(self, inner):
        self.inner, self.stale = inner, False

    def read(self):
        if self.stale:
            raise RuntimeError("SharedCamera：2.0 s 內沒有新畫面")
        return self.inner.read()

    def age(self):
        return 5.0 if self.stale else 0.0

    def intrinsics(self):
        return self.inner.intrinsics()


def test_manual_keeps_last_seen_board_until_chassis_moves():
    """切手動只轉轉塔：網板位置照樣能沿用、可以交還 AI。開過車且相機停了：不能交還 AI，網頁先警告。"""
    from rotbot.pipeline.loop import AutonomousRunner, ControlLoop
    from rotbot.pipeline.state_machine import GameState, MatchStateMachine
    cfg = load_config()
    mcu = FakeMCU()
    twin = TwinCamera(cfg, mcu, lit_bonus=1)
    cam = AgedTwinCamera(twin)
    sm = MatchStateMachine(auto_s=0.5)
    runner = AutonomousRunner(sm, lambda: ControlLoop(
        cfg, cam, TwinDetector(twin, noise_px=0.0), mcu, rpm=2400, sm=sm,
        is_active=lambda: sm.update() == GameState.AUTONOMOUS))
    sm.start()
    runner.start()
    try:
        end = time.monotonic() + 10.0
        while (runner.loop is None or runner.loop.last_seen is None) and time.monotonic() < end:
            time.sleep(0.02)
        while not sm.request_manual() and time.monotonic() < end:
            time.sleep(0.05)
        time.sleep(0.3)
        cam.stale = True                                        # 手動時相機停了
        assert runner.loop.last_seen is not None                # 只切手動：網板位置還在
        assert runner.resume_blocker() == "" and runner.warning() == ""
        runner.note_chassis_moved()                             # 開過車
        assert runner.loop.last_seen is None
        assert "沒有畫面" in runner.resume_blocker()
        assert runner.warning().startswith("現在不能交還 AI")
        cam.stale = False                                       # 相機回來：又可以交還
        assert runner.resume_blocker() == ""
    finally:
        runner.stop()


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
