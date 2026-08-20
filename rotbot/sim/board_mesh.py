# -*- coding: utf-8 -*-
"""網板碰撞網格 —— 程序化生成「含 12 個真孔的平板」OBJ，供 PyBullet 載入。

為什麼要自己生：PyBullet 的內建幾何（box/cylinder）挖不出孔。這裡把平板切成細方格，
凡格心落在任一孔內就跳過 → 得到一片「有洞的板」，球飛到孔會穿過、飛到實心處會反彈，
與計分（落點是否在孔內）在物理上一致。

輸出頂點已烤進世界座標（x 前=dist_x 平面、y 右對 world -Y、高度對 world +Z），
所以載入時 body 放原點即可。幾何一致：孔中心用 cfg.hole_centers()（與感知/計分共用）。
"""
from __future__ import annotations

import hashlib
from pathlib import Path

import numpy as np

from ..config import Config

ASSETS_DIR = Path(__file__).resolve().parent / "assets"


def _board_holes_world(cfg: Config):
    """回傳 [(u, v, radius)]：孔在板面座標 (u=右, v=相對主格中心的高度) 與半徑 (m)。"""
    out = []
    for (_g, _i, diam, C) in cfg.hole_centers():
        out.append((float(C[0]), float(C[1]), float(diam) / 2.0))
    return out


def _to_world(dist_x: float, grid_cx: float, grid_cy: float, u: float, v: float):
    """板面座標 (u 右, v 高度相對主格中心) → PyBullet 世界座標。

    world: X 前(朝板), Y 左, Z 上。板在 X=dist_x 垂直平面；板右 = world -Y、板高 = world Z。
    """
    return (dist_x, -(grid_cx + u), grid_cy + v)


def board_extents(cfg: Config, margin: float = 0.30):
    """回傳板面 (u_min, u_max, v_min, v_max)（板座標：u=右, v=相對主格中心高度）。"""
    holes = _board_holes_world(cfg)
    us = [(u, r) for (u, _v, r) in holes]
    vs = [(v, r) for (_u, v, r) in holes]
    u_min = min(u - r for u, r in us) - margin
    u_max = max(u + r for u, r in us) + margin
    v_min = min(v - r for v, r in vs) - margin
    v_max = max(v + r for v, r in vs) + margin
    return u_min, u_max, v_min, v_max


def board_obj_path(cfg: Config, cell: float = 0.03, margin: float = 0.30) -> Path:
    """回傳（必要時生成）此設定對應的網板 OBJ 路徑，依幾何雜湊快取。"""
    f = cfg.field
    holes = _board_holes_world(cfg)
    key = (round(f.dist_x, 4), round(f.grid_cx, 4), round(f.grid_cy, 4),
           round(cell, 4), round(margin, 4),
           tuple((round(u, 4), round(v, 4), round(r, 4)) for (u, v, r) in holes))
    h = hashlib.md5(repr(key).encode()).hexdigest()[:10]
    ASSETS_DIR.mkdir(parents=True, exist_ok=True)
    path = ASSETS_DIR / f"board_{h}.obj"
    if not path.exists():
        _write_board_obj(path, cfg, cell, margin)
    return path


def _write_board_obj(path: Path, cfg: Config, cell: float, margin: float) -> None:
    f = cfg.field
    holes = _board_holes_world(cfg)
    u_min, u_max, v_min, v_max = board_extents(cfg, margin)

    nu = max(1, int(round((u_max - u_min) / cell)))
    nv = max(1, int(round((v_max - v_min) / cell)))
    du = (u_max - u_min) / nu
    dv = (v_max - v_min) / nv

    holes_np = np.array(holes, dtype=np.float64)          # (H,3): u,v,r

    verts: list[tuple[float, float, float]] = []
    faces: list[tuple[int, int, int]] = []
    vindex: dict[tuple[int, int], int] = {}

    def vid(iu: int, iv: int) -> int:
        k = (iu, iv)
        if k not in vindex:
            u = u_min + iu * du
            v = v_min + iv * dv
            verts.append(_to_world(f.dist_x, f.grid_cx, f.grid_cy, u, v))
            vindex[k] = len(verts)                          # OBJ 1-based
        return vindex[k]

    hu, hv, hr = holes_np[:, 0], holes_np[:, 1], holes_np[:, 2]
    for iu in range(nu):
        uc = u_min + (iu + 0.5) * du
        for iv in range(nv):
            vc = v_min + (iv + 0.5) * dv
            if np.any((uc - hu) ** 2 + (vc - hv) ** 2 < hr ** 2):   # 格心在孔內 → 跳過
                continue
            a, b = vid(iu, iv), vid(iu + 1, iv)
            c, d = vid(iu + 1, iv + 1), vid(iu, iv + 1)
            faces.append((a, b, c))
            faces.append((a, c, d))

    with open(path, "w", encoding="utf-8") as fh:
        fh.write("# rotbot procedural board-with-holes (world coords)\n")
        for (x, y, z) in verts:
            fh.write(f"v {x:.5f} {y:.5f} {z:.5f}\n")
        for (a, b, c) in faces:
            fh.write(f"f {a} {b} {c}\n")
