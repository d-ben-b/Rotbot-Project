# -*- coding: utf-8 -*-
"""相機 HAL 實作 —— 真實相機（P5 硬體到位後啟用）。"""
from __future__ import annotations

import numpy as np

from ..config import CameraConfig
from ..hal import Camera


class RealCamera(Camera):
    """USB Webcam（cv2.VideoCapture）。P5 硬體到位後接上並以標定內參覆蓋。"""

    def __init__(self, cam_cfg: CameraConfig, device: int = 0):
        self.cam_cfg = cam_cfg
        self.device = device
        self._cap = None  # 延後開啟，避免無硬體環境 import 就失敗

    def _ensure_open(self):
        if self._cap is None:
            import cv2  # 局部 import：CI/模擬環境不需要真的開相機
            self._cap = cv2.VideoCapture(self.device)
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.cam_cfg.width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cam_cfg.height)

    def read(self) -> np.ndarray:
        self._ensure_open()
        ok, frame = self._cap.read()
        if not ok:
            raise RuntimeError("RealCamera: 讀取影像失敗")
        return frame

    def intrinsics(self) -> np.ndarray:
        return self.cam_cfg.matrix()

    def release(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None
