# -*- coding: utf-8 -*-
"""偵測後處理：圓孔編號指派與 bbox→Detection（不需 ultralytics/GPU）。"""
import numpy as np

from rotbot.config import load_config
from rotbot.perception.detector import (CLASS_BONUS, CLASS_BONUS_LIT, CLASS_MAIN,
                                        assign_ids, detection_from_bbox, drop_contained,
                                        drop_non_square)
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


def test_drop_contained_removes_half_hole_boxes():
    full = detection_from_bbox((0, 0, 100, 100), CLASS_MAIN, conf=0.8)
    half = detection_from_bbox((55, 0, 102, 100), CLASS_MAIN, conf=0.96)   # 信心值較高也要刪
    thin = detection_from_bbox((0, 2, 30, 98), CLASS_MAIN, conf=0.5)
    other = detection_from_bbox((200, 0, 300, 100), CLASS_MAIN, conf=0.9)
    assert drop_contained([half, full, thin, other]) == [full, other]


def test_drop_non_square_keeps_round_and_edge_boxes():
    shape = (1088, 1456, 3)
    full = detection_from_bbox((100, 100, 200, 198), CLASS_MAIN)
    half = detection_from_bbox((300, 100, 352, 200), CLASS_MAIN)            # 寬高比 0.52
    clipped = detection_from_bbox((0, 400, 40, 500), CLASS_MAIN)            # 被畫面左緣切掉
    assert drop_non_square([full, half, clipped], shape) == [full, clipped]


def test_drop_contained_same_hole_keeps_higher_confidence():
    bonus = detection_from_bbox((0, 0, 40, 40), CLASS_BONUS, conf=0.6)
    lit = detection_from_bbox((1, 1, 40, 40), CLASS_BONUS_LIT, conf=0.9)
    assert drop_contained([bonus, lit]) == [lit]
    assert drop_contained([bonus]) == [bonus]
    assert drop_contained([]) == []


def test_half_hole_boxes_do_not_shift_main_ids():
    """左欄每個孔多兩個半孔框（PyBullet 畫面實際發生過）時，編號仍是 0~8。"""
    cfg = load_config()
    s = SceneGenerator(cfg, randomize=False).sample()
    mains = [h for h in s.holes if h.group == "main"]
    dets, truth = [], []
    for h in mains:
        x1, y1 = h.rim_px.min(axis=0)
        x2, y2 = h.rim_px.max(axis=0)
        d = detection_from_bbox((x1, y1, x2, y2), CLASS_MAIN)
        dets.append(d)
        truth.append((d, h.idx))
        if h.idx % 3 == 0:                                   # 左欄
            xm = (x1 + x2) / 2
            dets += [detection_from_bbox((x1, y1, xm + 3, y2), CLASS_MAIN, conf=0.97),
                     detection_from_bbox((xm - 3, y1, x2, y2), CLASS_MAIN, conf=0.97)]
    kept = drop_contained(dets)
    assign_ids(kept)
    assert len(kept) == 9
    for d, idx in truth:
        assert d.idx == idx
