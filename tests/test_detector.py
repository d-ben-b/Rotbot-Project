# -*- coding: utf-8 -*-
"""偵測後處理：圓孔編號指派與 bbox→Detection（不需 ultralytics/GPU）。"""
import numpy as np

from rotbot.config import load_config
from rotbot.perception.detector import (CLASS_BONUS_LIT, CLASS_MAIN,
                                        assign_ids, detection_from_bbox)
from rotbot.sim.scene import SceneGenerator


def test_assign_ids_recovers_main_grid():
    cfg = load_config()
    s = SceneGenerator(cfg, randomize=False).sample()
    mains = [h for h in s.holes if h.group == "main"]

    rng = np.random.default_rng(0)
    order = rng.permutation(len(mains))
    dets = []
    for i in order:
        h = mains[i]
        x1, y1 = h.rim_px.min(axis=0)
        x2, y2 = h.rim_px.max(axis=0)
        dets.append(detection_from_bbox((x1, y1, x2, y2), CLASS_MAIN))

    assign_ids(dets)
    for det, i in zip(dets, order):
        assert det.idx == mains[i].idx


def test_detection_from_bbox_classes():
    d = detection_from_bbox((10, 20, 50, 60), CLASS_BONUS_LIT, conf=0.9)
    assert d.is_bonus and d.is_lit
    assert d.group == "bonus"
    assert np.allclose(d.center, [30, 40])
    assert d.diameter_px == 40.0

    m = detection_from_bbox((0, 0, 40, 40), CLASS_MAIN)
    assert not m.is_bonus and not m.is_lit and m.group == "main"


def test_assign_ids_empty():
    assign_ids([])
