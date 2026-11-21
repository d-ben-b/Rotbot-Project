# -*- coding: utf-8 -*-
"""偵測後處理：圓孔編號指派與 bbox→Detection（不需 ultralytics/GPU）。"""
import cv2
import numpy as np

from rotbot.config import load_config
from rotbot.perception.detector import (CLASS_BONUS, CLASS_BONUS_LIT, CLASS_MAIN,
                                        assign_ids, detection_from_bbox, drop_contained,
                                        drop_non_square, mark_lit_by_color)
from rotbot.sim.scene import SceneGenerator

_BONUS_BOXES = [(50, 50, 110, 110), (250, 50, 310, 110), (450, 50, 510, 110)]


def _bonus_scene(red_holes=(), white_core=True):
    """木板色背景上三個暗色加分孔；red_holes 的孔畫 LED 上半圈（紅色光暈 + 過曝白芯）。"""
    img = np.full((200, 600, 3), (115, 165, 205), np.uint8)
    for i, (x1, y1, x2, y2) in enumerate(_BONUS_BOXES):
        c, r = ((x1 + x2) // 2, (y1 + y2) // 2), (x2 - x1) // 2
        cv2.circle(img, c, r, (24, 24, 27), -1)
        if i in red_holes:
            cv2.ellipse(img, c, (r - 6, r - 6), 0, 190, 350, (30, 30, 200), 9)
            cv2.ellipse(img, c, (r - 6, r - 6), 0, 190, 350,
                        (225, 225, 255) if white_core else (40, 40, 240), 3)
    return img, [detection_from_bbox(b, CLASS_BONUS) for b in _BONUS_BOXES]


def test_color_fallback_marks_lit_hole_yolo_missed():
    """YOLO 把過曝的亮孔判成未亮 bonus 時，紅色比例明顯最高的那個標成亮孔。"""
    img, dets = _bonus_scene(red_holes=(1,))
    got = mark_lit_by_color(img, dets)
    assert got is dets[1] and dets[1].is_lit and dets[1].extra["lit_by"] == "color"
    assert not dets[0].is_lit and not dets[2].is_lit
    assert dets[1].extra["red_frac"] > 0.05 and dets[0].extra["red_frac"] == 0.0


def test_color_fallback_needs_a_clear_single_winner():
    img, dets = _bonus_scene(red_holes=())                 # 沒有紅色（60 秒後燈熄了）
    assert mark_lit_by_color(img, dets) is None and not any(d.is_lit for d in dets)
    img, dets = _bonus_scene(red_holes=(0, 2))             # 兩個一樣紅：分不出來就不標
    assert mark_lit_by_color(img, dets) is None and not any(d.is_lit for d in dets)
    img, dets = _bonus_scene(red_holes=(1,))               # 只看到一個加分孔：無從比較
    assert mark_lit_by_color(img, dets[1:2]) is None and not dets[1].is_lit


def test_color_fallback_keeps_yolo_lit_answer():
    img, dets = _bonus_scene(red_holes=(1,))
    dets[2] = detection_from_bbox(_BONUS_BOXES[2], CLASS_BONUS_LIT)   # YOLO 已經回報亮孔
    assert mark_lit_by_color(img, dets) is None
    assert [d.is_lit for d in dets] == [False, False, True]


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


def test_onnx_decode_maps_back_and_nms_per_class():
    """.onnx（OpenCV dnn）解碼：letterbox 座標轉回原圖；同類別重疊只留一個、不同類別都留；低於 conf 丟掉。"""
    from rotbot.perception.detector import YoloDetector
    r, top = 960 / 1456, 121            # 1456×1088 → 960×717，上下各補約 121 px

    def row(x1, y1, x2, y2, cls, s):
        sc = [0.0, 0.0, 0.0]
        sc[cls] = s
        return [(x1 + x2) / 2 * r, (y1 + y2) / 2 * r + top, (x2 - x1) * r, (y2 - y1) * r, *sc]

    class FakeNet:
        def setInput(self, blob):
            assert blob.shape == (1, 3, 960, 960)

        def forward(self):
            rows = [row(100, 100, 200, 200, 0, 0.9), row(102, 101, 201, 199, 0, 0.8),
                    row(100, 100, 200, 200, 1, 0.7), row(500, 500, 600, 600, 0, 0.1)]
            return np.array(rows, np.float32).T[None]

    det = YoloDetector.__new__(YoloDetector)
    det.conf, det.imgsz, det.net = 0.25, 960, FakeNet()
    xyxy, clss, confs = det._predict_onnx(np.zeros((1088, 1456, 3), np.uint8))
    assert sorted(clss.tolist()) == [0, 1]
    np.testing.assert_allclose(xyxy[clss == 0][0], [100, 100, 200, 200], atol=0.5)
    assert confs[clss == 0][0] == np.float32(0.9)
