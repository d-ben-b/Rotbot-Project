# -*- coding: utf-8 -*-
"""圓孔投球策略：加分優先、每孔上限、27 顆排程、瞄準指令。"""
import numpy as np

from rotbot.config import load_config
from rotbot.decision import ballistics
from rotbot.decision.strategy import ShotPlanner, plan_next_shot
from rotbot.perception.localize import HoleAim


def _aim(group, idx, dist, bearing=0.0, elev=0.0, is_bonus=False, is_lit=False):
    return HoleAim(group=group, idx=idx, is_bonus=is_bonus, is_lit=is_lit,
                   distance=dist, bearing=bearing, elev=elev, source="test")


def _nine_main():
    return [_aim("main", i, 2.5 + 0.1 * i) for i in range(9)]


def test_ballistics_aim_at_point():
    cfg = load_config()
    ang = ballistics.aim_at_point((2.5, 1.0, 0.0), cfg)     # 正前上方
    assert ang is not None
    pitch, yaw = ang
    assert abs(yaw) < 1e-9 and pitch > 0
    right = ballistics.aim_at_point((2.5, 1.0, 0.5), cfg)   # 偏右
    assert right[1] > 0                                     # yaw 右為正
    assert ballistics.aim_at_point((100.0, 50.0, 0.0), cfg) is None   # 射程不足


def test_planner_prioritizes_lit_bonus():
    cfg = load_config()
    aims = _nine_main() + [_aim("bonus", 1, 2.6, is_bonus=True, is_lit=True)]
    planner = ShotPlanner(cfg)
    chosen = planner.select(aims)
    assert chosen.is_lit


def test_planner_bonus_cap_then_main():
    cfg = load_config()
    lit = _aim("bonus", 1, 2.6, is_bonus=True, is_lit=True)
    aims = _nine_main() + [lit]
    planner = ShotPlanner(cfg, bonus_cap=3)
    for _ in range(3):                       # 前三球都投亮孔
        c = planner.select(aims)
        assert c.is_lit
        planner.record(c)
    assert not planner.select(aims).is_lit   # 加分達上限 → 改投一般孔


def test_planner_ignores_unlit_bonus():
    cfg = load_config()
    aims = _nine_main() + [_aim("bonus", 0, 2.6, is_bonus=True, is_lit=False)]
    planner = ShotPlanner(cfg)
    for _ in range(27):
        c = planner.select(aims)
        if c is None:
            break
        assert not c.is_bonus            # 未亮的加分孔永遠不選
        planner.record(c)


def test_planner_runs_out_of_balls():
    cfg = load_config()
    planner = ShotPlanner(cfg, total_balls=2)
    aims = _nine_main()
    planner.record(planner.select(aims))
    planner.record(planner.select(aims))
    assert planner.select(aims) is None


def test_plan_next_shot_end_to_end():
    cfg = load_config()
    aims = _nine_main() + [_aim("bonus", 1, 2.6, is_bonus=True, is_lit=True)]
    planner = ShotPlanner(cfg)
    plan = plan_next_shot(aims, planner, cfg)
    assert plan is not None
    assert plan.target.is_lit                      # 優先亮孔
    assert -90 < plan.pitch_deg < 90
    assert plan.point.shape == (3,)
