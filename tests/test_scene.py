# -*- coding: utf-8 -*-
"""合成圓孔場景：輸出格式與 canonical 幾何真值。"""
import numpy as np

from rotbot.config import load_config
from rotbot.sim.scene import SceneGenerator


def test_sample_format():
    cfg = load_config()
    s = SceneGenerator(cfg, rng=np.random.default_rng(0)).sample()
    assert s.image.shape == (cfg.camera.height, cfg.camera.width, 3)
    assert s.image.dtype == np.uint8
    assert len(s.holes) == 12                     # 9 主孔 + 3 加分孔
    assert sum(h.is_bonus for h in s.holes) == 3
    assert sum(h.is_lit for h in s.holes) <= 1    # 至多一個亮起
    for h in s.holes:
        assert h.rim_px.shape[1] == 2


def test_canonical_center_hole_truth():
    """正對 2.5m 時，中心主孔的真值距離≈dist_x、方位/俯仰≈0。"""
    cfg = load_config()
    s = SceneGenerator(cfg, randomize=False).sample()
    center = next(h for h in s.holes if h.group == "main" and h.idx == 4)
    assert abs(center.distance - cfg.field.dist_x) < 1e-6
    assert abs(center.bearing) < 1e-6
    assert abs(center.elev) < 1e-6
    assert abs(center.tilt) < 1e-2      # 正對，理論為 0（餘為浮點誤差）


def test_canonical_all_main_visible():
    cfg = load_config()
    s = SceneGenerator(cfg, randomize=False).sample()
    mains = [h for h in s.holes if h.group == "main"]
    assert all(h.visible for h in mains)
