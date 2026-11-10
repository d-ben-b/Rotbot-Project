# -*- coding: utf-8 -*-
"""賽制狀態機：前 60 秒鎖定手動、第 61 秒起可切換、3 分鐘結束、急停。"""
from rotbot.pipeline.state_machine import GameState, MatchStateMachine


class FakeClock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def test_autonomous_lock_then_manual():
    clk = FakeClock()
    sm = MatchStateMachine(clock=clk)
    assert sm.state == GameState.IDLE and not sm.request_manual()
    assert sm.start() and sm.state == GameState.AUTONOMOUS
    assert sm.bonus_window_open
    clk.t += 59.9
    assert sm.autonomous_locked and not sm.request_manual()   # 規則：60 秒內不得遙控
    clk.t += 0.2
    assert not sm.autonomous_locked and not sm.bonus_window_open
    assert sm.state == GameState.AUTONOMOUS                   # 60 秒後不會自動切，由操作手決定
    assert sm.request_manual() and sm.manual_allowed
    assert sm.request_auto() and sm.state == GameState.AUTONOMOUS


def test_match_ends_and_estop():
    clk = FakeClock()
    sm = MatchStateMachine(clock=clk)
    sm.start()
    clk.t += 61
    sm.request_manual()
    clk.t += 120
    assert sm.update() == GameState.FINISHED and not sm.manual_allowed
    sm.reset()
    assert sm.state == GameState.IDLE and sm.start()
    sm.estop()
    assert sm.state == GameState.ESTOP and not sm.start()    # 急停後要先 reset
    sm.reset()
    assert sm.start()


def test_resume_after_estop_keeps_match_clock():
    """救援卡：急停後「繼續比賽」不重設計時；60 秒內只能回 AI，之後回到急停前的模式，時間到就結束。"""
    clk = FakeClock()
    sm = MatchStateMachine(clock=clk)
    sm.estop()
    assert not sm.can_resume and sm.resume() is None          # 開賽前急停：要重置再開始
    sm.reset()
    sm.start()
    t0 = sm.t_start
    clk.t += 20
    sm.estop()
    sm.estop()                                                 # 按兩次：急停前的狀態還是 AI
    clk.t += 15                                                # 救援 15 s，時間照算
    assert sm.resume() == GameState.AUTONOMOUS and sm.t_start == t0
    assert sm.elapsed == 35 and sm.autonomous_locked and sm.bonus_window_open
    clk.t += 30
    assert sm.request_manual()
    sm.estop()
    clk.t += 10
    assert sm.resume() == GameState.MANUAL                     # 60 秒後回到急停前的手動
    sm.estop()
    clk.t += 200
    assert sm.resume() == GameState.FINISHED
