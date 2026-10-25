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
