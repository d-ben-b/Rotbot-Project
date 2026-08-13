# -*- coding: utf-8 -*-
"""偵測後處理：九宮格編號指派（不需 ultralytics/GPU）。"""
import numpy as np

from rotbot.config import load_config
from rotbot.perception.detector import Detection, assign_grid_ids
from rotbot.sim.scene import SceneGenerator


def test_assign_grid_ids_recovers_row_major_order():
    cfg = load_config()
    s = SceneGenerator(cfg, randomize=False).sample()   # 正對九宮格，格排列整齊

    # 用打亂順序的偵測結果，看能否還原 row-major 的 0~8
    rng = np.random.default_rng(0)
    idx = rng.permutation(9)
    dets = [Detection(cell_id=-1, corners=s.cells[i].corners.copy()) for i in idx]

    assign_grid_ids(dets)

    for det, src in zip(dets, idx):
        assert det.cell_id == s.cells[src].cell_id


def test_assign_grid_ids_empty():
    assign_grid_ids([])   # 不應報錯
