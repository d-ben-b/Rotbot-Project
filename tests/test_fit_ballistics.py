# -*- coding: utf-8 -*-
"""試投擬合：用已知「真實世界」參數產生打牆資料，擬合應找回 v0 / Cd / 偏差（含逐發抖動與量測雜訊）。"""
import numpy as np

from rotbot.config import load_config
from tools.fit_ballistics import fit, synthetic_shots


def test_fit_recovers_launch_parameters():
    cfg = load_config()
    shots = synthetic_shots(cfg, 27, np.random.default_rng(0))       # 2/4/6m × 10/20/30°
    r = fit(shots, cfg)
    v0, cd, bp, by = r["theta"]
    assert abs(v0 - cfg.launch.v0_real) < 0.6
    assert abs(cd - cfg.air.cd) < 0.2
    assert abs(by - cfg.mechanism.bias_yaw) < 0.3
    assert abs(bp - cfg.mechanism.bias_pitch) < 0.8
    assert r["rms_y"] < 0.06                                          # 殘差 ≈ 逐發散布等級


def test_fixed_v0_tightens_cd():
    cfg = load_config()
    shots = synthetic_shots(cfg, 27, np.random.default_rng(0))
    free = fit(shots, cfg)
    fixed = fit(shots, cfg, fix_v0=cfg.launch.v0_real)                 # v0 另外量到時
    assert fixed["theta"][0] == cfg.launch.v0_real
    assert fixed["se"][1] < free["se"][1]                             # Cd 的不確定度變小
