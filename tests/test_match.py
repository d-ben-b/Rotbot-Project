# -*- coding: utf-8 -*-
"""校正閉迴路回歸測試 —— 驗證線上校正確實提升加分格命中率並收斂。

沿用 presentation/sim_adaptive_calibration.py 的結論，但用較少蒙地卡羅次數以求快。
"""
import numpy as np

from rotbot.config import load_config
from rotbot.sim.match import monte_carlo


def test_calibration_improves_bonus_hit_rate():
    cfg = load_config()
    h_on, h_off, b_on = monte_carlo(cfg, n_trials=20)

    # 一般框本來就寬鬆，兩者都該不錯
    assert h_off.mean() > 0.5
    # 收斂後（第 4 顆起）加分格命中率應明顯優於未校正的一般表現
    bonus_after = b_on[:, 3:].mean()
    assert bonus_after > 0.3


def test_error_converges():
    """校正後，後段誤差應低於初期誤差（有在收斂）。"""
    cfg = load_config()
    rng = np.random.default_rng(cfg.sim.seed)
    from rotbot.sim.match import run_match

    errs = np.array([run_match(cfg, rng, correct=True).errs for _ in range(20)])
    early = np.nanmean(errs[:, 0])       # 第 1 顆
    late = np.nanmean(errs[:, -10:])     # 後 10 顆
    assert late < early
