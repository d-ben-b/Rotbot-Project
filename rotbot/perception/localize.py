# -*- coding: utf-8 -*-
"""感知定位 —— 把偵測結果轉成「每孔瞄準幾何」。

串接：Detection 清單（含孔編號）→ 對應已知 3D 孔中心 → 整板 PnP（方法 A）
     → 每孔 (距離/方位/俯仰)。孔數不足以整板求解時，逐孔用表觀直徑測距備援。

這是 P4 主迴圈的「感知半段」：detector 之後、decision 之前。
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..config import Config
from .detector import Detection
from .pose import BoardPose, distance_from_diameter, solve_board_pose


@dataclass
class HoleAim:
    """單孔的瞄準幾何。"""
    group: str
    idx: int
    is_bonus: bool
    is_lit: bool
    distance: float          # m
    bearing: float           # deg
    elev: float              # deg
    source: str              # "board_pnp" | "size"


@dataclass
class SceneEstimate:
    board_pose: BoardPose | None
    aims: list[HoleAim]

    @property
    def lit_bonus(self) -> HoleAim | None:
        """亮起的加分孔（策略最高優先目標）。"""
        return next((a for a in self.aims if a.is_lit), None)


def _diameter_real(det: Detection, cfg: Config) -> float:
    return cfg.field.bonus_d if det.is_bonus else cfg.field.frame_d


def localize_holes(detections: list[Detection], cfg: Config, K: np.ndarray,
                   dist_coeffs: np.ndarray | None = None) -> SceneEstimate:
    """由偵測結果算出整板位姿與每孔瞄準幾何。"""
    centers = {(g, i): C for (g, i, _d, C) in cfg.hole_centers()}
    Kinv = np.linalg.inv(K)

    # 收集可對應到已知孔中心的偵測，做整板 PnP
    obj, img = [], []
    for det in detections:
        key = (det.group, det.idx)
        if det.idx >= 0 and key in centers:
            obj.append(centers[key])
            img.append(det.center)
    bp = solve_board_pose(np.array(obj), np.array(img), K, dist_coeffs) \
        if len(obj) >= 4 else None

    aims: list[HoleAim] = []
    for det in detections:
        key = (det.group, det.idx)
        if bp is not None and det.idx >= 0 and key in centers:
            distance, bearing, elev = bp.point_aim(centers[key])
            source = "board_pnp"
        else:
            # 備援：表觀直徑 → 深度；中心像素 → 視線方向
            depth = distance_from_diameter(det.diameter_px, _diameter_real(det, cfg), K[0, 0])
            ray = Kinv @ np.array([det.center[0], det.center[1], 1.0])
            p = ray * depth                       # ray_z = 1 → 尺度即深度
            distance = float(np.linalg.norm(p))
            bearing = float(np.degrees(np.arctan2(p[0], p[2])))
            elev = float(np.degrees(np.arctan2(-p[1], p[2])))
            source = "size"
        aims.append(HoleAim(group=det.group, idx=det.idx, is_bonus=det.is_bonus,
                            is_lit=det.is_lit, distance=distance, bearing=bearing,
                            elev=elev, source=source))
    return SceneEstimate(board_pose=bp, aims=aims)
