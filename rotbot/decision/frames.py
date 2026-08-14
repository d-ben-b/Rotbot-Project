# -*- coding: utf-8 -*-
"""座標系轉換 —— 把感知的「相機系」量測，換到瞄準用的「發射點系」。

三個座標系（右手系）：
  相機系  cam       : +x 右、+y 下、+z 前（與 perception 一致）
  發射點系 launcher  : +x 前（朝網板）、+y 上、+z 右（拋體/瞄準用）
  相機↔發射點以外參 CameraMount 連接（安裝位移 + 旋轉，待實機標定）

感知給的是每孔 (距離, 方位角, 俯仰角)（相機系）；本模組還原成相機系 3D 點，
再經外參轉到發射點系 3D 目標，交給 ballistics 求瞄準角。
"""
from __future__ import annotations

import numpy as np

from ..config import CameraMount, Config

# 相機系→發射點系 的軸置換：x_l=z_c, y_l=-y_c, z_l=x_c
_AXIS_CAM_TO_LAUNCHER = np.array([[0.0, 0.0, 1.0],
                                  [0.0, -1.0, 0.0],
                                  [1.0, 0.0, 0.0]])


def aim_to_camera_point(distance: float, bearing_deg: float, elev_deg: float) -> np.ndarray:
    """由 (距離, 方位角, 俯仰角) 還原相機系 3D 點 (x右, y下, z前)。

    定義：bearing = atan2(x, z)、elev = atan2(-y, z)、distance = |點|。
    """
    tb, te = np.tan(np.radians(bearing_deg)), np.tan(np.radians(elev_deg))
    z = distance / np.sqrt(tb * tb + te * te + 1.0)
    return np.array([z * tb, -z * te, z])


def _mount_rotation(m: CameraMount) -> np.ndarray:
    """外參旋轉（發射點系內的安裝姿態）：roll 繞前軸 x、pitch 繞右軸 z、yaw 繞上軸 y。"""
    rr, rp, ry = np.radians(m.roll), np.radians(m.pitch), np.radians(m.yaw)
    cr, sr = np.cos(rr), np.sin(rr)
    cp, sp = np.cos(rp), np.sin(rp)
    cy, sy = np.cos(ry), np.sin(ry)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])   # roll 繞前軸 x_l
    Rz = np.array([[cp, -sp, 0], [sp, cp, 0], [0, 0, 1]])   # pitch 繞右軸 z_l
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])   # yaw 繞上軸 y_l
    return Ry @ Rz @ Rx


def camera_point_to_launcher(p_cam: np.ndarray, cfg: Config) -> np.ndarray:
    """相機系 3D 點 → 發射點系 3D 點（軸置換 + 外參旋轉/位移）。"""
    m = cfg.camera_mount
    p = _AXIS_CAM_TO_LAUNCHER @ np.asarray(p_cam, float)
    p = _mount_rotation(m) @ p
    return p + np.array([m.dz, m.dy, m.dx])   # 前(x)=dz、上(y)=dy、右(z)=dx


def hole_target_point(distance: float, bearing_deg: float, elev_deg: float,
                      cfg: Config) -> np.ndarray:
    """感知量測 (距離/方位/俯仰) → 發射點系 3D 目標 (x前, y上, z右)。"""
    return camera_point_to_launcher(
        aim_to_camera_point(distance, bearing_deg, elev_deg), cfg)
