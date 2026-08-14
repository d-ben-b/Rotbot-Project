# -*- coding: utf-8 -*-
"""圓孔位姿與測距 —— 兩種方法，供比較（見 ROADMAP §4）。

目標是「圓孔」（Ø40cm / 加分 Ø20cm）。提供兩條路線：

方法 A（中心 + 尺寸）：
  YOLO 偵測每個圓孔 → 孔中心像素 → 以「已知 3D 孔中心」做整板 PnP，
  得整塊板位姿，再算每孔距離/方位。單孔時用「表觀直徑 → 距離」測距備援。
  簡單、Orin 快、對齊計劃書；不回復單孔傾角。

方法 B（橢圓擬合 + 圓錐反投影）：
  圓在透視下投影成橢圓 → 由橢圓 + 內參 + 已知半徑，反投影橢圓錐，
  解出單孔的 3D 中心與法向（含傾角）。能應對孔被打歪/形變。
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


# ══════════════════════════ 方法 A：中心整板 PnP + 尺寸測距 ══════════════════════════
@dataclass(frozen=True)
class BoardPose:
    """整塊九宮格板相對相機的位姿（以已知孔中心過定求解，最穩健）。"""
    rvec: np.ndarray
    tvec: np.ndarray
    reproj_err: float          # 中心點重投影誤差 (px)
    n_holes: int               # 參與求解的孔數

    def point_aim(self, center_board: np.ndarray) -> tuple[float, float, float]:
        """給板座標系某點，回傳其 (距離 m, 方位角 deg, 俯仰角 deg)。"""
        R, _ = cv2.Rodrigues(self.rvec)
        x, y, z = R @ np.asarray(center_board, float) + self.tvec.ravel()
        distance = float(np.linalg.norm([x, y, z]))
        bearing = float(np.degrees(np.arctan2(x, z)))
        elev = float(np.degrees(np.arctan2(-y, z)))
        return distance, bearing, elev


def solve_board_pose(obj_pts: np.ndarray, img_pts: np.ndarray, K: np.ndarray,
                     dist_coeffs: np.ndarray | None = None) -> BoardPose | None:
    """用已知孔中心（共面）過定求解整板位姿。

    obj_pts: (N, 3) 各孔在板座標系的 3D 中心；img_pts: (N, 2) 對應觀測像素中心。
    N >= 4；孔數越多越穩。共面點用 IPPE，失敗退回 ITERATIVE。
    """
    obj_pts = np.asarray(obj_pts, np.float64)
    img_pts = np.asarray(img_pts, np.float64)
    if len(obj_pts) < 4:
        return None
    dist = np.zeros((5, 1)) if dist_coeffs is None else np.asarray(dist_coeffs, np.float64)
    # SQPNP：對共面且「正對(fronto-parallel)」都穩定、單一解，避開 IPPE 的正對退化
    ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, dist, flags=cv2.SOLVEPNP_SQPNP)
    if not ok:
        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, dist,
                                      flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, K, dist)
    reproj = float(np.mean(np.linalg.norm(proj.reshape(-1, 2) - img_pts, axis=1)))
    return BoardPose(rvec=rvec, tvec=tvec, reproj_err=reproj, n_holes=len(obj_pts))


def distance_from_diameter(pixel_diameter: float, real_diameter: float,
                           focal: float) -> float:
    """表觀直徑 → 距離（單目尺寸測距，近似正對）。Z ≈ f · D_real / d_px。"""
    if pixel_diameter <= 0:
        return float("inf")
    return float(focal * real_diameter / pixel_diameter)


# ══════════════════════════ 方法 B：橢圓 → 圓錐反投影 ══════════════════════════
@dataclass(frozen=True)
class CirclePose:
    """單一圓孔的 3D 位姿（由橢圓反投影解出）。"""
    center_cam: np.ndarray     # (3,) 圓心在相機座標 (m)
    normal_cam: np.ndarray     # (3,) 圓面法向（單位向量，朝相機）
    distance: float            # 圓心到相機 (m)

    @property
    def bearing(self) -> float:
        return float(np.degrees(np.arctan2(self.center_cam[0], self.center_cam[2])))

    @property
    def elev(self) -> float:
        return float(np.degrees(np.arctan2(-self.center_cam[1], self.center_cam[2])))

    @property
    def tilt(self) -> float:
        """入射傾角 (deg)：法向與視線夾角，0=正對。"""
        los = -self.center_cam / (np.linalg.norm(self.center_cam) + 1e-9)
        return float(np.degrees(np.arccos(np.clip(abs(self.normal_cam @ los), 0, 1))))


def _ellipse_to_conic(ellipse) -> np.ndarray:
    """cv2 橢圓 ((cx,cy),(d1,d2),ang_deg) → 3x3 圓錐矩陣（像素座標）。"""
    (cx, cy), (d1, d2), ang = ellipse
    A, B = d1 / 2.0, d2 / 2.0                       # 半軸
    phi = np.radians(ang)
    R = np.array([[np.cos(phi), -np.sin(phi)], [np.sin(phi), np.cos(phi)]])
    M = R @ np.diag([1.0 / A**2, 1.0 / B**2]) @ R.T
    c = np.array([cx, cy])
    Mc = M @ c
    Q = np.array([[M[0, 0], M[0, 1], -Mc[0]],
                  [M[0, 1], M[1, 1], -Mc[1]],
                  [-Mc[0], -Mc[1], float(c @ M @ c - 1.0)]])
    return Q


def pose_from_ellipse(ellipse, K: np.ndarray, radius: float) -> CirclePose | None:
    """由投影橢圓 + 內參 + 已知半徑，反投影圓錐解出圓孔 3D 位姿。

    回傳物理上有效的解（圓心在前方、法向朝相機）；退化時回傳 None。
    """
    Q = _ellipse_to_conic(ellipse)
    B = K.T @ Q @ K
    B = (B + B.T) / 2.0
    w, V = np.linalg.eigh(B)                        # 升冪；特徵向量為欄
    # 需要簽名 (+,+,-)：兩正一負。若為 (-,-,+) 則整體變號。
    if np.sum(w > 0) == 1:
        w, B = -w, -B
        w, V = np.linalg.eigh(B)
    if not (np.sum(w > 0) == 2 and np.sum(w < 0) == 1):
        return None
    # 排成 λ1 >= λ2 > 0 > λ3
    order = np.argsort(w)[::-1]                     # 由大到小：正、正、負
    lam = w[order]
    e = V[:, order]
    l1, l2, l3 = lam
    e1, e3 = e[:, 0], e[:, 2]

    g = np.sqrt((l1 - l2) / (l1 - l3))
    h = np.sqrt((l2 - l3) / (l1 - l3))

    best = None
    for s1 in (1.0, -1.0):
        for s2 in (1.0, -1.0):
            normal = s1 * g * e1 + s2 * h * e3
            center = (radius * l2 / np.sqrt(-l1 * l3)) * (
                s1 * (l3 / l2) * g * e1 + s2 * (l1 / l2) * h * e3)
            if center[2] <= 0:                     # 圓心須在相機前方
                continue
            n = normal / (np.linalg.norm(normal) + 1e-12)
            if n @ center > 0:                     # 法向須朝相機（與視線相對）
                n = -n
            los = -center / (np.linalg.norm(center) + 1e-12)
            score = n @ los                        # 越接近正對越大
            if best is None or score > best[0]:
                best = (score, center, n)
    if best is None:
        return None
    _, center, normal = best
    return CirclePose(center_cam=center, normal_cam=normal,
                      distance=float(np.linalg.norm(center)))
