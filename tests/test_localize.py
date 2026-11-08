# -*- coding: utf-8 -*-
"""全鏈驗證：合成影像真值 → 偵測 → 網格編號 → 整板 PnP → 每孔瞄準。"""
import numpy as np

from rotbot.config import load_config
from rotbot.perception.detector import CLASS_BONUS_LIT, CLASS_MAIN, assign_ids, detection_from_bbox
from rotbot.perception.localize import localize_holes
from rotbot.sim.perfect_detector import detections_from_scene
from rotbot.sim.scene import SceneGenerator


def test_full_chain_canonical():
    cfg = load_config()
    s = SceneGenerator(cfg, randomize=False).sample()
    dets = detections_from_scene(s)
    est = localize_holes(dets, cfg, cfg.camera.matrix())

    assert est.board_pose is not None
    # 主孔真值：以 (group, idx) 對照
    truth = {("main", h.idx): h for h in s.holes if h.group == "main"}
    for aim in est.aims:
        if aim.group != "main":
            continue
        assert aim.source == "board_pnp"
        h = truth[("main", aim.idx)]
        assert abs(aim.distance - h.distance) < 0.01     # < 1 cm
        assert abs(aim.bearing - h.bearing) < 0.5
        assert abs(aim.elev - h.elev) < 0.5


def test_extra_half_hole_box_not_output():
    """一個孔被切成左右兩半各一個框（沒有完整框）時：不輸出不存在的編號、每孔只一個瞄準。"""
    cfg = load_config()
    s = SceneGenerator(cfg, randomize=False).sample()
    dets = detections_from_scene(s)
    d = next(d for d in dets if d.group == "main" and d.idx == 6)
    x1, y1, x2, y2 = d.bbox
    xm = (x1 + x2) / 2
    left = detection_from_bbox((x1, y1, xm + 4, y2), CLASS_MAIN)
    right = detection_from_bbox((xm - 4, y1, x2, y2), CLASS_MAIN)
    dets = [o for o in dets if o is not d] + [left, right]
    assign_ids(dets)
    assert left.idx == right.idx == 6                              # 分欄分列：兩半落在同一格
    right.idx = 9               # 退回逐排編號時會出現的不存在編號，定位不能把它交給策略

    est = localize_holes(dets, cfg, cfg.camera.matrix())
    keys = [(a.group, a.idx) for a in est.aims]
    known = {(g, i) for (g, i, _d, _C) in cfg.hole_centers()}
    assert set(keys) <= known and len(keys) == len(set(keys))
    truth = {("main", h.idx): h for h in s.holes if h.group == "main"}
    for aim in est.aims:
        if aim.group == "main":
            h = truth[("main", aim.idx)]
            assert abs(aim.bearing - h.bearing) < 0.5 and abs(aim.elev - h.elev) < 0.5


def test_false_bonus_far_from_holes_not_output():
    """場上的球被當成亮燈加分孔（離任何加分孔都遠）時不輸出，亮孔仍是真的那個。"""
    cfg = load_config()
    s = SceneGenerator(cfg, randomize=False).sample()
    dets = detections_from_scene(s)
    m7 = next(d for d in dets if d.group == "main" and d.idx == 7)
    x1, y1, x2, y2 = m7.bbox
    ball = detection_from_bbox((x1, y2 + 40, x1 + 12, y2 + 52), CLASS_BONUS_LIT, conf=0.9)
    est = localize_holes(dets + [ball], cfg, cfg.camera.matrix())
    keys = [(a.group, a.idx) for a in est.aims]
    assert len(keys) == len(set(keys))
    lit_truth = [h.idx for h in s.holes if h.is_lit]
    assert [a.idx for a in est.aims if a.is_lit] == lit_truth


def test_full_chain_random_views_accurate():
    cfg = load_config()
    gen = SceneGenerator(cfg, rng=np.random.default_rng(3))
    d_err = []
    for _ in range(15):
        s = gen.sample()
        dets = detections_from_scene(s)
        if sum(not d.is_bonus for d in dets) < 4:
            continue
        est = localize_holes(dets, cfg, cfg.camera.matrix())
        truth = {(h.group, h.idx): h for h in s.holes}
        for aim in est.aims:
            h = truth.get((aim.group, aim.idx))
            if h is not None:
                d_err.append(abs(aim.distance - h.distance))
    assert np.median(d_err) < 0.03                        # < 3 cm 中位誤差


def test_lit_bonus_surfaced():
    cfg = load_config()
    # 找一個「亮起加分孔可見」的隨機場景，確認 lit_bonus 會被帶出
    gen = SceneGenerator(cfg, rng=np.random.default_rng(0))
    for _ in range(40):
        s = gen.sample()
        if any(h.is_lit and h.visible for h in s.holes):
            est = localize_holes(detections_from_scene(s), cfg, cfg.camera.matrix())
            assert est.lit_bonus is not None
            assert est.lit_bonus.is_bonus and est.lit_bonus.is_lit
            return
    # 理論上 40 次內必遇到，未遇到則跳過
