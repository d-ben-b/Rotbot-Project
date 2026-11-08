# -*- coding: utf-8 -*-
"""PyBullet 相機渲染 —— 從機器人相機視角取一張 BGR 影像。

投影矩陣由 field.yaml 的相機內參 (fx,fy,cx,cy,w,h) 導出，讓渲染影像的視野與
真實相機/偵測器/PnP 用的 K 一致（同軸相機隨轉塔轉動）。

也提供 camera_extrinsic()：由「已知的相機世界位姿」精確反算 相機系→發射點系 的 (R_cl, t_cl)，
供自動模式把感知量測換算成發射點系 3D 目標（與 sim.closed_loop 的做法一致，避免尤拉角近似）。
"""
from __future__ import annotations

import numpy as np

from ..config import CameraConfig

# 發射點系基底以世界座標表示（world: X 前, Y 左, Z 上）：
#   x_l=前=(1,0,0)、y_l=上=(0,0,1)、z_l=右=world -Y=(0,-1,0)
_R_LW = np.array([[1.0, 0.0, 0.0],
                  [0.0, 0.0, 1.0],
                  [0.0, -1.0, 0.0]])   # world → launcher


def _cam_axes(forward: np.ndarray):
    """由前方向建相機系軸（OpenCV 慣例：x 右, y 下, z 前），世界上向為輔助。"""
    f = np.asarray(forward, float)
    f = f / (np.linalg.norm(f) + 1e-12)
    w = np.array([0.0, 0.0, 1.0])                      # 世界上向
    up = w - np.dot(w, f) * f                          # 投影到與 f 垂直
    n = np.linalg.norm(up)
    up = up / n if n > 1e-6 else np.array([0.0, 1.0, 0.0])
    down = -up
    right = np.cross(down, f)                          # x = y × z（右手系）
    right = right / (np.linalg.norm(right) + 1e-12)
    return right, down, f, up


def projection_matrix(cam: CameraConfig, near: float = 0.02, far: float = 20.0):
    """由內參 fx/fy/cx/cy 導出 OpenGL 投影矩陣（column-major，PyBullet 格式）。

    主點不在畫面中央時（實機標定 cx 777.7 / 寬 1456），要放進第三欄；
    舊版用 computeProjectionMatrixFOV 只取 fy，渲染出的孔比 K 投影的位置偏約 50 px。
    """
    W, H = float(cam.width), float(cam.height)
    P = np.array([
        [2 * cam.fx / W, 0.0, (W - 2 * cam.cx) / W, 0.0],
        [0.0, 2 * cam.fy / H, (2 * cam.cy - H) / H, 0.0],
        [0.0, 0.0, -(far + near) / (far - near), -2 * far * near / (far - near)],
        [0.0, 0.0, -1.0, 0.0]])
    return P.T.reshape(-1).tolist()


def render(eye, forward, cam: CameraConfig, client: int,
           near: float = 0.02, far: float = 20.0,
           light_dir=(-1.0, 0.4, 1.2), noise: float = 3.0, glare: float = 0.0,
           rng: np.random.Generator | None = None, gpu: bool = False) -> np.ndarray:
    """渲染一張 BGR (H,W,3) uint8 影像。eye/forward 為世界座標。

    noise: 感測器高斯雜訊 σ (灰階)；glare: 0~1 眩光強度（模擬強光直射過曝）。
    gpu: 用 GUI 的 OpenGL 渲染（只在 GUI 模式可用；728×544 約 24 ms，CPU 渲染約 240 ms），
         沒有陰影、背景是 GUI 的底色。感知 / 合成資料維持 CPU 渲染。
    """
    import pybullet as p
    _r, _d, f, up = _cam_axes(np.asarray(forward, float))
    eye = np.asarray(eye, float)
    view = p.computeViewMatrix(eye.tolist(), (eye + f).tolist(), up.tolist())
    proj = projection_matrix(cam, near, far)
    w, h, rgba, _depth, _seg = p.getCameraImage(
        cam.width, cam.height, view, proj,
        lightDirection=list(light_dir), shadow=1,
        renderer=p.ER_BULLET_HARDWARE_OPENGL if gpu else p.ER_TINY_RENDERER,
        physicsClientId=client)
    rgb = np.reshape(np.asarray(rgba, np.uint8), (h, w, 4))[:, :, :3]
    img = rgb[:, :, ::-1].astype(np.float32)            # RGB → BGR float
    return _post(img, noise, glare, rng)


def _post(img: np.ndarray, noise: float, glare: float,
          rng: np.random.Generator | None) -> np.ndarray:
    """後製：感測器雜訊 + 眩光過曝（讓渲染更像真 USB webcam）。"""
    rng = rng if rng is not None else np.random.default_rng()
    h, w = img.shape[:2]
    if glare > 0:                                       # 一團亮斑 → 局部過曝
        cx, cy = int(w * rng.uniform(0.3, 0.7)), int(h * rng.uniform(0.15, 0.5))
        yy, xx = np.mgrid[0:h, 0:w]
        r2 = ((xx - cx) ** 2 + (yy - cy) ** 2) / (0.18 * w) ** 2
        halo = np.exp(-r2)[:, :, None] * (255 * glare)
        img = img + halo
    if noise > 0:
        img = img + rng.normal(0, noise, img.shape)
    return np.clip(img, 0, 255).astype(np.uint8)


def camera_extrinsic(eye, forward, launch_h: float):
    """由相機世界位姿反算 相機系→發射點系 的 (R_cl, t_cl)。

    發射點系原點在槍口 (0,0,launch_h)（world）。回傳可直接餵 frames.hole_target_point_ext。
    """
    right, down, f, _up = _cam_axes(np.asarray(forward, float))
    R_wc = np.column_stack([right, down, f])            # 相機系 → world
    o_l = np.array([0.0, 0.0, launch_h])                # 發射點原點（world）
    R_cl = _R_LW @ R_wc
    t_cl = _R_LW @ (np.asarray(eye, float) - o_l)
    return R_cl, t_cl
