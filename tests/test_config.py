# -*- coding: utf-8 -*-
from rotbot.config import load_config


def test_load_default_config():
    cfg = load_config()
    assert cfg.field.dist_x == 2.50
    assert cfg.ball.mass == 0.030
    assert cfg.n_balls == 27


def test_derived_constants():
    cfg = load_config()
    # 容許球心誤差 = (框徑 - 球徑) / 2
    assert abs(cfg.tol_std - (0.40 - 0.07) / 2) < 1e-9
    assert abs(cfg.tol_bonus - (0.20 - 0.07) / 2) < 1e-9
    assert cfg.k_drag > 0
    assert cfg.camera.matrix().shape == (3, 3)
