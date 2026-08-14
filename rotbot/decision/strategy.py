# -*- coding: utf-8 -*-
"""投球策略 —— 九宮格幾何、命中容許值、目標選擇。

P0 先提供幾何與容許值工具，以及一個「依序輪投」的基準策略。
P2 再擴充：加分格最高優先、雙倍分排程、剩彈/時間管理（見 ROADMAP）。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from ..config import Config
from . import ballistics, frames

if TYPE_CHECKING:                       # 僅型別提示，執行期以鴨子型別使用，避免反向相依
    from ..perception.localize import HoleAim


def grid_cells(cfg: Config) -> list[tuple[float, float]]:
    """九宮格 9 個目標中心 (橫向 z, 高度 y)，順序：上排左 → 下排右。"""
    f = cfg.field
    return [(f.grid_cx + c * f.col_pitch, f.grid_cy + r * f.row_pitch)
            for r in (1, 0, -1) for c in (-1, 0, 1)]


def select_next_target(cfg: Config, shot_index: int,
                       bonus_cell: int | None = None) -> int:
    """（舊 3×3 sim 用）回傳下一發要投的格 index；有亮孔優先，否則輪投。"""
    if bonus_cell is not None:
        return bonus_cell
    return shot_index % 9


# ══════════════════════════ 圓孔版投球策略（P2）══════════════════════════
@dataclass
class ShotPlan:
    """一發的決策結果。"""
    target: "HoleAim"        # 選定的目標孔
    pitch_deg: float         # 俯仰角指令
    yaw_deg: float           # 水平角指令
    point: np.ndarray        # 目標在發射點系的 3D 座標 (x前,y上,z右)


class ShotPlanner:
    """27 顆的投球排程：亮起加分孔最高優先，各孔有球數上限。

    規則對應：加分孔（亮起）每球 4 分、最多 3 球（12 分）、整場有效；
             一般孔每球 2 分、每孔最多 3 球（6 分）。未亮的加分孔不投（不計分）。
    """

    def __init__(self, cfg: Config, total_balls: int | None = None,
                 bonus_cap: int = 3, main_cap: int = 3):
        self.cfg = cfg
        self.remaining = cfg.n_balls if total_balls is None else total_balls
        self.bonus_cap = bonus_cap
        self.main_cap = main_cap
        self.counts: dict[tuple[str, int], int] = {}   # (group,idx) -> 已投球數

    def _key(self, aim) -> tuple[str, int]:
        return (aim.group, aim.idx)

    def select(self, aims) -> "HoleAim | None":
        """從候選孔挑下一個目標；沒球或無可投目標回傳 None。"""
        if self.remaining <= 0:
            return None
        lit = [a for a in aims if a.is_lit
               and self.counts.get(self._key(a), 0) < self.bonus_cap]
        if lit:
            return min(lit, key=lambda a: a.distance)     # 近的較好命中
        mains = [a for a in aims if not a.is_bonus
                 and self.counts.get(self._key(a), 0) < self.main_cap]
        if mains:
            return min(mains, key=lambda a: a.distance)
        return None

    def record(self, aim) -> None:
        """記錄已投一發（不論命中與否，實彈都消耗）。"""
        self.remaining -= 1
        self.counts[self._key(aim)] = self.counts.get(self._key(aim), 0) + 1


def plan_next_shot(aims, planner: ShotPlanner, cfg: Config,
                   extrinsic: tuple | None = None) -> ShotPlan | None:
    """感知每孔瞄準 → 過濾射程可達 → 策略選目標 → 回傳瞄準指令。

    aims: perception.localize 的 HoleAim 清單（具 distance/bearing/elev/group/idx/is_bonus/is_lit）。
    extrinsic: 選填 (R_cl, t_cl) 完整外參；給定時取代 cfg.camera_mount 尤拉角。
    """
    reachable = []
    for a in aims:
        if extrinsic is not None:
            pt = frames.hole_target_point_ext(a.distance, a.bearing, a.elev, *extrinsic)
        else:
            pt = frames.hole_target_point(a.distance, a.bearing, a.elev, cfg)
        ang = ballistics.aim_at_point(pt, cfg)
        if ang is not None:
            reachable.append((a, ang, pt))
    if not reachable:
        return None
    chosen = planner.select([r[0] for r in reachable])
    if chosen is None:
        return None
    for a, (pitch, yaw), pt in reachable:
        if a is chosen:
            return ShotPlan(target=a, pitch_deg=float(np.degrees(pitch)),
                            yaw_deg=float(np.degrees(yaw)), point=pt)
    return None
