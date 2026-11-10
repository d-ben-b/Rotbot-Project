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


def _no_search(cfg):
    """關掉找網板和盲投：只測「看不到板」的計數（找網板會多拍好幾張，打亂 FlakyDetector 的順序）。"""
    import dataclasses

    from rotbot.config import SearchConfig
    return dataclasses.replace(cfg, search=SearchConfig(after_misses=10 ** 6, blind_after_s=0.0))


def test_scattered_misses_do_not_stop_the_loop():
    """看不到板的次數累計超過 5 次、但中間有看到時，主迴圈照樣投（以前累計 6 次就停）。"""
    from rotbot.pipeline.loop import ControlLoop
    cfg = _no_search(load_config())
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
    cfg = _no_search(load_config())
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


def test_search_finds_board_and_warns_when_start_box_is_wrong():
    """選錯起始框（設定網板在左前方，實際在右前方）：掃視姿態看不到 → 轉塔找到網板 → 照常投，並提醒左右選錯。"""
    from rotbot.decision import ballistics
    from rotbot.pipeline.loop import ControlLoop
    base = load_config()
    truth, wrong = base.at_start_box("left"), base.at_start_box("right")
    mcu = FakeMCU()
    cam = TwinCamera(truth, mcu, lit_bonus=1)
    loop = ControlLoop(wrong, cam, TwinDetector(cam, noise_px=0.0), mcu, rpm=2400)
    assert loop.survey[1] < -10                             # 依錯的設定往左看
    log = None
    for _ in range(4):
        log = loop.step()
        if log is not None:
            break
    assert log is not None and log.source == "camera"
    assert abs(loop.survey[1] - ControlLoop.default_survey_pose(truth)[1]) < 3.0   # 掃視姿態改到網板那邊
    assert "看到網板在右邊" in loop.side_warning and "設定的網板在左邊" in loop.side_warning
    R_bl, t_bl = frames.board_to_launcher(truth)          # 瞄的是真正的孔位
    g, i = log.target.split("#")
    C = next(C for gg, ii, _d, C in truth.hole_centers() if (gg, ii) == (g, int(i)))
    want = [np.degrees(v) for v in ballistics.aim_at_point(R_bl @ C + t_bl, truth)]
    assert (log.pitch_deg, log.yaw_deg) == pytest.approx(want, abs=0.05)


def test_blind_shot_after_search_finds_nothing():
    """開賽後一直看不到網板：先轉塔找一輪，過了 blind_after_s 就用起始框設定的位置盲投一般孔（不投沒亮的加分孔）。"""
    import dataclasses

    from rotbot.config import SearchConfig
    from rotbot.decision import ballistics
    from rotbot.pipeline.loop import AutonomousRunner, ControlLoop
    from rotbot.pipeline.state_machine import MatchStateMachine
    cfg = dataclasses.replace(load_config().at_start_box("left"), search=SearchConfig(blind_after_s=0.3))
    mcu = FakeMCU()
    cam = TwinCamera(cfg, mcu)
    det = FlakyDetector(TwinDetector(cam), [False] * 10 ** 4)          # 什麼都拍不到
    loop = ControlLoop(cfg, cam, det, mcu, rpm=2400)
    log = None
    end = time.monotonic() + 10.0
    while log is None and time.monotonic() < end:
        log = loop.step()
    assert log is not None and log.source == "blind" and log.target.startswith("main#")
    assert det.calls >= 3 * (1 + 8)                         # 掃視姿態 + 找網板 8 個方向都拍過
    R_bl, t_bl = frames.board_to_launcher(cfg)
    C = next(C for g, i, _d, C in cfg.hole_centers() if (g, i) == ("main", int(log.target.split("#")[1])))
    want = [np.degrees(v) for v in ballistics.aim_at_point(R_bl @ C + t_bl, cfg)]
    assert (log.pitch_deg, log.yaw_deg) == pytest.approx(want, abs=0.01)
    sm = MatchStateMachine()
    sm.start()
    runner = AutonomousRunner(sm, lambda: loop)
    runner.loop = loop
    assert "盲投" in runner.warning()


def test_board_pose_for_preview_matches_truth():
    """AI 看到網板後，給預估落點用的網板位姿（發射點系）和真實位置差 1 cm、0.1° 以內；底盤動過就作廢。"""
    from rotbot.pipeline.loop import ControlLoop
    cfg = load_config().at_start_box("right")
    mcu = FakeMCU()
    cam = TwinCamera(cfg, mcu, lit_bonus=1)
    loop = ControlLoop(cfg, cam, TwinDetector(cam, noise_px=0.0), mcu, rpm=2400)
    assert loop.board_in_launcher() is None
    assert loop.step() is not None
    R, t = loop.board_in_launcher()
    R_bl, t_bl = frames.board_to_launcher(cfg)
    assert np.linalg.norm(t - t_bl) < 0.01
    assert np.degrees(np.arccos(np.clip((np.trace(R_bl.T @ R) - 1) / 2, -1, 1))) < 0.1
    loop.forget_board()
    assert loop.board_in_launcher() is None
