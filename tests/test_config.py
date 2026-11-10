# -*- coding: utf-8 -*-
from rotbot.config import load_config


def test_load_default_config():
    cfg = load_config()
    assert cfg.field.dist_x == 6.00        # 2026-10-25 起投球位置 6m
    assert cfg.field.grid_cy == 1.80       # 官方正視圖：中排孔心離板底 180cm
    assert cfg.ball.mass == 0.030
    assert cfg.n_balls == 27
    assert cfg.aim.model == "drag"         # 6m 必須用含阻力的腦內模型


def test_derived_constants():
    cfg = load_config()
    # 容許球心誤差 = (框徑 - 球徑) / 2
    assert abs(cfg.tol_std - (0.40 - 0.07) / 2) < 1e-9
    assert abs(cfg.tol_bonus - (0.20 - 0.07) / 2) < 1e-9
    assert cfg.k_drag > 0
    assert cfg.k_drag_model > 0
    assert cfg.camera.matrix().shape == (3, 3)


def test_missing_aim_section_defaults_to_vacuum(tmp_path):
    """舊版 YAML 沒有 aim 區段時，退回無阻力模型（向下相容）。"""
    import yaml
    from rotbot.config import DEFAULT_CONFIG
    raw = yaml.safe_load(DEFAULT_CONFIG.read_text(encoding="utf-8"))
    raw.pop("aim")
    p = tmp_path / "old.yaml"
    p.write_text(yaml.safe_dump(raw, allow_unicode=True), encoding="utf-8")
    assert load_config(p).aim.model == "vacuum"


def test_start_box_replaces_grid_cx_and_dist_x():
    """起始框：左框網板在右邊（grid_cx 正）、右框在左邊；只換 grid_cx 與 dist_x，field 原本的值留給模擬。"""
    cfg = load_config()
    left, right = cfg.at_start_box("left"), cfg.at_start_box("right")
    assert left.field.grid_cx == cfg.start_box.left_grid_cx > 0
    assert right.field.grid_cx == cfg.start_box.right_grid_cx < 0
    assert left.field.dist_x == right.field.dist_x == cfg.start_box.dist_x
    assert left.field.grid_cy == cfg.field.grid_cy and cfg.field.grid_cx == 0.0
