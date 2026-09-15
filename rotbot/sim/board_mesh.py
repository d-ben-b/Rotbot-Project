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
import math
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


def board_obj_path(cfg: Config, cell: float = 0.04, arc_seg: float = 0.012,
                   margin: float = 0.30) -> Path:
    """回傳（必要時生成）此設定對應的網板 OBJ 路徑，依幾何雜湊快取。

    cell   : 背景方格邊長（平坦處，粗即可）。
    arc_seg: 孔緣圓弧的取樣段長（越小越圓）；孔邊沿真圓貼齊，與 cell 無關 → 又圓又輕。
    """
    f = cfg.field
    holes = _board_holes_world(cfg)
    key = (round(f.dist_x, 4), round(f.grid_cx, 4), round(f.grid_cy, 4),
           round(cell, 4), round(arc_seg, 4), round(margin, 4),
           tuple((round(u, 4), round(v, 4), round(r, 4)) for (u, v, r) in holes))
    h = hashlib.md5(repr(key).encode()).hexdigest()[:10]
    ASSETS_DIR.mkdir(parents=True, exist_ok=True)
    path = ASSETS_DIR / f"board_{h}.obj"
    if not path.exists():
        _write_board_obj(path, cfg, cell, arc_seg, margin)
    return path


def ring_obj_path(inner_r: float, outer_r: float, seg: int = 72) -> Path:
    """回傳（必要時生成）一個平面圓環（annulus）OBJ，供亮孔紅色 LED 燈環用。

    幾何在區域 XY 平面、z=0、圓心原點；雙面（正反都可見）。載入後由呼叫端擺位/轉向。
    """
    key = (round(inner_r, 4), round(outer_r, 4), int(seg))
    h = hashlib.md5(repr(key).encode()).hexdigest()[:10]
    ASSETS_DIR.mkdir(parents=True, exist_ok=True)
    path = ASSETS_DIR / f"ring_{h}.obj"
    if path.exists():
        return path
    verts, faces = [], []
    for k in range(seg):
        a = 2 * math.pi * k / seg
        verts.append((inner_r * math.cos(a), inner_r * math.sin(a), 0.0))
        verts.append((outer_r * math.cos(a), outer_r * math.sin(a), 0.0))
    for k in range(seg):
        i0, o0 = 2 * k + 1, 2 * k + 2                     # 1-based
        i1, o1 = 2 * ((k + 1) % seg) + 1, 2 * ((k + 1) % seg) + 2
        faces += [(i0, o0, o1), (i0, o1, i1),             # 正面
                  (i0, o1, o0), (i0, i1, o1)]             # 背面（雙面可見）
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("# rotbot LED ring (annulus, local XY plane)\n")
        for (x, y, z) in verts:
            fh.write(f"v {x:.5f} {y:.5f} {z:.5f}\n")
        for (a, b, c) in faces:
            fh.write(f"f {a} {b} {c}\n")
    return path


def _seg_circle_hit(ax, ay, bx, by, cx, cy, r):
    """線段 A→B 與圓(cx,cy,r)的交點（A、B 一內一外，必有一解），回傳 (x, y)。"""
    dx, dy = bx - ax, by - ay
    fx, fy = ax - cx, ay - cy
    a = dx * dx + dy * dy
    b = 2 * (fx * dx + fy * dy)
    c = fx * fx + fy * fy - r * r
    disc = max(0.0, b * b - 4 * a * c) ** 0.5
    for t in ((-b - disc) / (2 * a), (-b + disc) / (2 * a)):
        if -1e-9 <= t <= 1 + 1e-9:
            return ax + t * dx, ay + t * dy
    return ax + 0.5 * dx, ay + 0.5 * dy


def _clip_cell(corners, inside, hole, arc_seg):
    """把方格的「實心部分」（扣掉圓孔）建成多邊形頂點（板面座標），孔緣以圓弧貼齊。"""
    cx, cy, r = hole
    pts, isx = [], []                                    # 頂點、是否為交點
    for i in range(4):
        ax, ay = corners[i]
        bx, by = corners[(i + 1) % 4]
        if not inside[i]:
            pts.append((ax, ay)); isx.append(False)
        if inside[i] != inside[(i + 1) % 4]:
            pts.append(_seg_circle_hit(ax, ay, bx, by, cx, cy, r)); isx.append(True)
    xs = [i for i, x in enumerate(isx) if x]
    if len(xs) != 2:
        return pts                                       # 退化（罕見）：直接用直線切
    j, k = xs
    if (j + 1) % len(pts) != k:                          # 讓兩交點相鄰（切邊在 j→k）
        j, k = k, j
    p1, p2 = pts[j], pts[k]
    a1 = math.atan2(p1[1] - cy, p1[0] - cx)
    a2 = math.atan2(p2[1] - cy, p2[0] - cx)
    d = (a2 - a1) % (2 * math.pi)                         # a1→a2 逆時針角度
    # 選「弧中點落在格內」的方向（凸向被挖掉的角），即孔緣真正在格內的那段弧
    umin = min(c[0] for c in corners); umax = max(c[0] for c in corners)
    vmin = min(c[1] for c in corners); vmax = max(c[1] for c in corners)
    mid = a1 + d / 2
    if not (umin - 1e-9 <= cx + r * math.cos(mid) <= umax + 1e-9
            and vmin - 1e-9 <= cy + r * math.sin(mid) <= vmax + 1e-9):
        d -= 2 * math.pi                                 # 改走另一向
    n = max(1, int(round(abs(d) * r / arc_seg)))
    arc = [(cx + r * math.cos(a1 + d * t / n), cy + r * math.sin(a1 + d * t / n))
           for t in range(1, n)]                         # 不含端點（p1,p2 已在）
    return pts[:j + 1] + arc + pts[k:]


def _write_board_obj(path: Path, cfg: Config, cell: float, arc_seg: float,
                     margin: float) -> None:
    f = cfg.field
    holes = _board_holes_world(cfg)
    u_min, u_max, v_min, v_max = board_extents(cfg, margin)
    nu = max(1, int(round((u_max - u_min) / cell)))
    nv = max(1, int(round((v_max - v_min) / cell)))
    du = (u_max - u_min) / nu
    dv = (v_max - v_min) / nv

    holes_np = np.array(holes, dtype=np.float64)
    hu, hv, hr = holes_np[:, 0], holes_np[:, 1], holes_np[:, 2]

    verts: list[tuple[float, float, float]] = []
    faces: list[tuple[int, int, int]] = []

    def add(u, v) -> int:
        verts.append(_to_world(f.dist_x, f.grid_cx, f.grid_cy, u, v))
        return len(verts)                                # OBJ 1-based

    def fan(poly):                                       # 以形心扇形三角化（對星凸多邊形穩健）
        if len(poly) < 3:
            return
        gx = sum(p[0] for p in poly) / len(poly)
        gy = sum(p[1] for p in poly) / len(poly)
        g = add(gx, gy)
        ring = [add(u, v) for (u, v) in poly]
        for i in range(len(ring)):
            faces.append((g, ring[i], ring[(i + 1) % len(ring)]))

    for iu in range(nu):
        u0 = u_min + iu * du; u1 = u0 + du
        for iv in range(nv):
            v0 = v_min + iv * dv; v1 = v0 + dv
            corners = [(u0, v0), (u1, v0), (u1, v1), (u0, v1)]
            uc, vc = 0.5 * (u0 + u1), 0.5 * (v0 + v1)
            hi = int(np.argmin((uc - hu) ** 2 + (vc - hv) ** 2))   # 最近孔（各格至多受一孔影響）
            hole = (hu[hi], hv[hi], hr[hi])
            inside = [((cx - hole[0]) ** 2 + (cy - hole[1]) ** 2) < hole[2] ** 2
                      for (cx, cy) in corners]
            n_in = sum(inside)
            if n_in == 4:
                continue                                 # 全在孔內 → 跳過
            if n_in == 0:
                a, b = add(u0, v0), add(u1, v0)
                c, d = add(u1, v1), add(u0, v1)
                faces.append((a, b, c)); faces.append((a, c, d))
            else:
                fan(_clip_cell(corners, inside, hole, arc_seg))    # 孔邊格：圓弧貼齊

    with open(path, "w", encoding="utf-8") as fh:
        fh.write("# rotbot procedural board-with-holes (smooth arc rims, world coords)\n")
        for (x, y, z) in verts:
            fh.write(f"v {x:.5f} {y:.5f} {z:.5f}\n")
        for (a, b, c) in faces:
            fh.write(f"f {a} {b} {c}\n")
