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


def turret_rotation(pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """轉塔姿態（先水平 yaw、再俯仰 pitch）在發射點系的旋轉矩陣。

    與 ballistics 的出射方向定義一致：前方單位向量 → (cos p·cos y, sin p, cos p·sin y)，
    即俯仰上仰為正、水平往右(+z)為正。
    """
    p, y = np.radians(pitch_deg), np.radians(yaw_deg)
    Rp = np.array([[np.cos(p), -np.sin(p), 0.0], [np.sin(p), np.cos(p), 0.0], [0.0, 0.0, 1.0]])
    Ry = np.array([[np.cos(y), 0.0, -np.sin(y)], [0.0, 1.0, 0.0], [np.sin(y), 0.0, np.cos(y)]])
    return Ry @ Rp


def camera_extrinsic_at(cfg: Config, turret_pitch_deg: float,
                        turret_yaw_deg: float) -> tuple[np.ndarray, np.ndarray]:
    """同軸相機隨轉塔轉動：在給定轉塔角度下，相機系 → 發射點系（固定在底座）的外參 (R_cl, t_cl)。

    p_l = R_t · (R_m · A · p_c + t_m)；R_t = 轉塔姿態、A = 軸置換、R_m/t_m = camera_mount（相機相對槍管）。
    轉塔歸零 (0°,0°) 時與 camera_point_to_launcher 完全相同。掃視時用 MCU 回報的實際角度代入。
    （近似：轉塔樞軸視為在發射口；樞軸到槍口的位移交給線上校正吸收。）
    """
    m = cfg.camera_mount
    R_t = turret_rotation(turret_pitch_deg, turret_yaw_deg)
    R_cl = R_t @ _mount_rotation(m) @ _AXIS_CAM_TO_LAUNCHER
    t_cl = R_t @ np.array([m.dz, m.dy, m.dx])
    return R_cl, t_cl


def board_to_launcher(cfg: Config) -> tuple[np.ndarray, np.ndarray]:
    """網板座標系（+x 右、+y 上、+z 朝機器人）→ 發射點系 的 (R_bl, t_bl)（網板視為 x=dist_x 的鉛直面）。"""
    f = cfg.field
    R_bl = np.array([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])
    t_bl = np.array([f.dist_x, f.grid_cy - f.launch_h, f.grid_cx])
    return R_bl, t_bl


def hole_target_point_ext(distance: float, bearing_deg: float, elev_deg: float,
                          R_cl: np.ndarray, t_cl: np.ndarray) -> np.ndarray:
    """同上，但用完整外參矩陣 (R_cl, t_cl) 取代 CameraMount 尤拉角。

    用於模擬閉迴路：外參由場景真實板位姿精確反算（避免尤拉角近似的殘餘偏差）。
    P5 實機標定同樣可得完整 R,t。
    """
    p_cam = aim_to_camera_point(distance, bearing_deg, elev_deg)
    return np.asarray(R_cl, float) @ p_cam + np.asarray(t_cl, float)
