# -*- coding: utf-8 -*-
"""投球策略 —— 九宮格幾何、命中容許值、目標選擇。

P0 先提供幾何與容許值工具，以及一個「依序輪投」的基準策略。
P2 再擴充：加分格最高優先、雙倍分排程、剩彈/時間管理（見 ROADMAP）。
"""
from __future__ import annotations

from ..config import Config


def grid_cells(cfg: Config) -> list[tuple[float, float]]:
    """九宮格 9 個目標中心 (橫向 z, 高度 y)，順序：上排左 → 下排右。"""
    f = cfg.field
    return [(f.grid_cx + c * f.col_pitch, f.grid_cy + r * f.row_pitch)
            for r in (1, 0, -1) for c in (-1, 0, 1)]


def select_next_target(cfg: Config, shot_index: int,
                       bonus_cell: int | None = None) -> int:
    """回傳下一發要投的格 index (0~8)。

    基準策略（P0）：有亮起的加分格就優先投它，否則依序輪投。
    P2 會換成完整的雙倍分排程與剩彈管理。
    """
    if bonus_cell is not None:
        return bonus_cell
    return shot_index % 9
