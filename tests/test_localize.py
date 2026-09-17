# -*- coding: utf-8 -*-
"""全鏈驗證：合成影像真值 → 偵測 → 網格編號 → 整板 PnP → 每孔瞄準。"""
import numpy as np

from rotbot.config import load_config
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
