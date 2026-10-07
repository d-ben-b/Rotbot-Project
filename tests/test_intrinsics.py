# -*- coding: utf-8 -*-
"""相機內參標定演算法（合成資料，不需硬體）。"""
import math

import cv2
import numpy as np
import pytest

from rotbot.perception.intrinsics import (PATTERN, SQUARE_M, ViewCollector, find_corners,
                                          make_board_pdf, solve_intrinsics)

W, H = 1456, 1088
K_TRUE = np.array([[1739.0, 0, 735.0], [0, 1741.0, 540.0], [0, 0, 1.0]])
DIST_TRUE = np.array([-0.12, 0.08, 0.0005, -0.0003, 0.0])


def _board_points():
    g = np.zeros((PATTERN[0] * PATTERN[1], 3), np.float32)
    g[:, :2] = np.mgrid[0:PATTERN[0], 0:PATTERN[1]].T.reshape(-1, 2) * SQUARE_M
    return g


def _poses(n, rng):
    """板子在相機前的隨機位姿：不同位置、距離、傾斜。"""
    out = []
    while len(out) < n:
        rvec = rng.uniform(-0.5, 0.5, 3) * np.array([1, 1, 0.3])
        tvec = np.array([rng.uniform(-0.25, 0.05), rng.uniform(-0.15, 0.05), rng.uniform(0.35, 0.7)])
        pts, _ = cv2.projectPoints(_board_points(), rvec, tvec, K_TRUE, DIST_TRUE)
        pts = pts.reshape(-1, 2)
        if pts.min() > 20 and pts[:, 0].max() < W - 20 and pts[:, 1].max() < H - 20:
            out.append((rvec, tvec, pts.astype(np.float32)))
    return out


def test_solve_recovers_known_intrinsics():
    rng = np.random.default_rng(3)
    views = [p + rng.normal(0, 0.1, p.shape).astype(np.float32) for _, _, p in _poses(25, rng)]
    res = solve_intrinsics(views, (W, H))
    assert res.rms_px < 0.3 and res.quality() == "良好"
    assert abs(res.fx - K_TRUE[0, 0]) / K_TRUE[0, 0] < 0.01
    assert abs(res.fy - K_TRUE[1, 1]) / K_TRUE[1, 1] < 0.01
    assert abs(res.cx - K_TRUE[0, 2]) < 10 and abs(res.cy - K_TRUE[1, 2]) < 10
    assert abs(res.dist[0] - DIST_TRUE[0]) < 0.03
    hfov, vfov = res.fov_deg()
    assert abs(hfov - 2 * math.degrees(math.atan(W / 2 / 1739))) < 0.6
    assert "fx:" in res.yaml_block() and str(W) in res.yaml_block()


def test_solve_drops_a_bad_view_and_needs_enough_views():
    rng = np.random.default_rng(4)
    views = [p for _, _, p in _poses(15, rng)]
    bad = views[0] + rng.normal(0, 6.0, views[0].shape).astype(np.float32)   # 一張嚴重模糊/錯位
    res = solve_intrinsics([bad] + views[1:], (W, H))
    assert res.n_views == 14 and res.rms_px < 0.3
    with pytest.raises(ValueError):
        solve_intrinsics(views[:3], (W, H))


def test_find_corners_on_rendered_board():
    # 畫一張棋盤格 → 用已知位姿做透視變換貼到畫面上 → 找到的角點要落在真值上
    sq = 60
    nx, ny = PATTERN[0] + 1, PATTERN[1] + 1
    board = np.full(((ny + 2) * sq, (nx + 2) * sq), 255, np.uint8)
    for i in range(nx):
        for j in range(ny):
            if (i + j) % 2 == 0:
                board[(j + 1) * sq:(j + 2) * sq, (i + 1) * sq:(i + 2) * sq] = 0
    rvec, tvec = np.array([0.25, -0.3, 0.05]), np.array([-0.12, -0.08, 0.5])
    K = K_TRUE

    def project(xy_m):
        p, _ = cv2.projectPoints(np.hstack([xy_m, np.zeros((len(xy_m), 1))]).astype(np.float32),
                                 rvec, tvec, K, None)
        return p.reshape(-1, 2).astype(np.float32)

    m_per_px = SQUARE_M / sq          # 板圖像素 → 板面公尺；內角點 (0,0) 在板圖的 (2sq, 2sq)
    src = np.float32([[0, 0], [board.shape[1], 0], [board.shape[1], board.shape[0]], [0, board.shape[0]]])
    dst = project((src - 2 * sq) * m_per_px)
    img = cv2.warpPerspective(board, cv2.getPerspectiveTransform(src, dst), (W, H), borderValue=128)
    img = cv2.GaussianBlur(img, (0, 0), 1.0)

    found = find_corners(img)
    assert found is not None and found.shape == (PATTERN[0] * PATTERN[1], 2)
    truth = project(_board_points()[:, :2])
    # 棋盤格轉 180° 還是同一個圖樣，兩種點序都對
    err = min(np.abs(found - truth).max(), np.abs(found - truth[::-1]).max())
    assert err < 1.0, err
    assert find_corners(np.full((H, W), 128, np.uint8)) is None


def test_collector_wants_steady_new_and_spread_views():
    col = ViewCollector((W, H), target=3)
    base = np.stack(np.meshgrid(np.arange(9) * 40.0, np.arange(6) * 40.0), -1).reshape(-1, 2) + 50
    assert col.offer(None, 0.0) == "none"
    assert col.offer(base, 1.0) == "moving"            # 第一次看到：還不知道穩不穩
    assert col.offer(base + 30, 1.1) == "moving"       # 還在動
    assert col.offer(base + 30.5, 1.2) == "added"      # 穩住了
    assert col.offer(base + 31, 1.3) == "duplicate"    # 同一個位置不重複收
    far = base + np.array([900.0, 600.0])
    assert col.offer(far, 2.0) == "moving" and col.offer(far, 2.1) == "added"
    assert len(col.views) == 2 and not col.done
    assert "左上" not in col.missing_zones() and "右上" in col.missing_zones()
    assert 0 < col.coverage < 0.5


def test_board_pdf_is_written(tmp_path):
    pytest.importorskip("matplotlib")
    out = tmp_path / "board.pdf"
    make_board_pdf(str(out))
    assert out.read_bytes()[:4] == b"%PDF" and out.stat().st_size > 1000
