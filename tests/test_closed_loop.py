# -*- coding: utf-8 -*-
"""模擬閉迴路（P4）：整場能跑、通訊路徑走過、線上校正提升成績。"""
import numpy as np

from rotbot.config import load_config
from rotbot.sim.closed_loop import monte_carlo, run_closed_loop


def test_match_runs_and_scores():
    cfg = load_config()
    r = run_closed_loop(cfg, np.random.default_rng(0), correct=True)
    assert len(r.shots) == cfg.n_balls          # 27 球全投完
    assert 0 <= r.total_points <= 60            # 分數在合理上限內（12 加分 + 24×2）
    assert 0.0 <= r.hit_rate <= 1.0


def test_calibration_improves_outcome():
    cfg = load_config()
    pon, hon, bon = monte_carlo(cfg, n_trials=12, correct=True)
    poff, hoff, boff = monte_carlo(cfg, n_trials=12, correct=False)
    # 校正後總分與命中率不應更差；加分孔（Ø20 最吃校正）應明顯提升
    assert pon.mean() >= poff.mean()
    assert bon.mean() > boff.mean()
