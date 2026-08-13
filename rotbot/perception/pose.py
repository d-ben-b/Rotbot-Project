# -*- coding: utf-8 -*-
"""網框位姿解算 —— 由網框四角像素座標，用 cv2.solvePnP 解出
「距離 + 瞄準角 + 網框傾角」。

原型：presentation/pnp_frame_pose.py（已含真值→投影→反解的自我驗證，
搬進本模組後改為可 import 的純函式，驗證邏輯移到 tests/test_pose.py）。
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

# 角點順序：左上 → 右上 → 右下 → 左下（cv2.SOLVEPNP_IPPE_SQUARE 規定）
_CORNER_ORDER = ("top_left", "top_right", "bottom_right", "bottom_left")


def frame_object_points(size: float) -> np.ndarray:
    """網框四角的 3D 座標（框中心為原點，Z=0，單位：公尺）。

    size: 網框邊長 (m)。
    """
    s = size
    return np.array([[-s / 2,  s / 2, 0.0],
                     [ s / 2,  s / 2, 0.0],
                     [ s / 2, -s / 2, 0.0],
                     [-s / 2, -s / 2, 0.0]], dtype=np.float64)


@dataclass(frozen=True)
class FramePose:
    """網框相對相機的位姿解。"""
    distance: float    # 相機到框中心距離 (m)
    yaw_aim: float     # 瞄準方位角 (deg)，右為正
    pitch_aim: float   # 瞄準俯仰角 (deg)，上為正
    tilt: float        # 網框自身傾角 (deg)，0 = 正對相機
    reproj_err: float  # 重投影誤差 (px)，品質指標


def solve_frame_pose(
    img_pts: np.ndarray,
    K: np.ndarray,
    frame_size: float,
    dist_coeffs: np.ndarray | None = None,
) -> FramePose | None:
    """由四角像素座標解位姿。

    img_pts: (4, 2) 像素座標，順序須為 左上→右上→右下→左下。
    K: (3, 3) 相機內參矩陣。
    frame_size: 網框邊長 (m)。
    dist_coeffs: 畸變係數，None 視為無畸變。
    回傳 FramePose；solvePnP 失敗時回傳 None。
    """
    img_pts = np.asarray(img_pts, dtype=np.float64)
    obj = frame_object_points(frame_size)
    dist = np.zeros((5, 1)) if dist_coeffs is None else np.asarray(dist_coeffs, dtype=np.float64)

    ok, rvec, tvec = cv2.solvePnP(obj, img_pts, K, dist,
                                  flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None

    x, y, z = tvec.ravel()
    distance = float(np.linalg.norm(tvec))
    yaw_aim = float(np.degrees(np.arctan2(x, z)))
    pitch_aim = float(np.degrees(np.arctan2(-y, z)))

    R, _ = cv2.Rodrigues(rvec)
    normal = R @ np.array([0.0, 0.0, 1.0])
    tilt = float(np.degrees(np.arccos(min(1.0, abs(normal[2])))))

    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    reproj = float(np.mean(np.linalg.norm(proj.reshape(-1, 2) - img_pts, axis=1)))

    return FramePose(distance=distance, yaw_aim=yaw_aim, pitch_aim=pitch_aim,
                     tilt=tilt, reproj_err=reproj)


def project_frame_corners(
    K: np.ndarray,
    frame_size: float,
    distance: float,
    bearing_deg: float,
    elev_deg: float,
    frame_tilt_deg: float = 0.0,
    noise_px: float = 0.0,
    seed: int = 0,
) -> np.ndarray:
    """給定真值，算出網框四角的像素座標（模擬完美/含雜訊的偵測）。

    可用於單元測試（真值→投影→反解）與 P1 合成資料產生。
    相機座標系：+x 右、+y 下、+z 前。
    """
    obj = frame_object_points(frame_size)
    b, e = np.radians(bearing_deg), np.radians(elev_deg)
    center = distance * np.array([np.sin(b) * np.cos(e),
                                  -np.sin(e),
                                  np.cos(b) * np.cos(e)])
    t = np.radians(frame_tilt_deg)
    R_frame = np.array([[np.cos(t), 0, np.sin(t)],
                        [0, 1, 0],
                        [-np.sin(t), 0, np.cos(t)]])
    pts_cam = (R_frame @ obj.T).T + center
    uv = (K @ pts_cam.T).T
    uv = uv[:, :2] / uv[:, 2:3]
    if noise_px:
        uv = uv + np.random.default_rng(seed).normal(0, noise_px, uv.shape)
    return uv
