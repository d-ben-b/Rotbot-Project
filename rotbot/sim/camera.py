# -*- coding: utf-8 -*-
"""相機 HAL 實作 —— 模擬相機（沒有硬體時的替身）。

P1 會接上 sim.scene 的合成場景產生器，回傳擬真影像供 detector 測試。
P0 先給最小可用版本：回傳空白畫面 + 設定內參，讓 pipeline 能先組起來。
"""
from __future__ import annotations

import numpy as np

from ..config import CameraConfig
from ..hal import Camera


class SimCamera(Camera):
    """模擬相機。scene 省略時回傳空白畫面（P0）；P1 傳入場景產生器。"""

    def __init__(self, cam_cfg: CameraConfig, scene=None):
        self.cam_cfg = cam_cfg
        self.scene = scene

    def read(self) -> np.ndarray:
        if self.scene is not None:
            return self.scene.render()          # P1：合成影像
        return np.zeros((self.cam_cfg.height, self.cam_cfg.width, 3), dtype=np.uint8)

    def intrinsics(self) -> np.ndarray:
        return self.cam_cfg.matrix()
