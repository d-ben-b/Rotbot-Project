# -*- coding: utf-8 -*-
"""相機 HAL 實作 —— 真實相機（P5 硬體到位後啟用）+ 多使用者共用包裝。"""
from __future__ import annotations

import threading
import time

import numpy as np

from ..config import CameraConfig, Config
from ..hal import Camera


def make_camera(cfg: Config) -> Camera:
    """依 configs/field.yaml 的 camera_hw.backend 建立實體相機。"""
    hw = cfg.camera_hw
    if hw.backend == "imx296":
        from .imx296 import IMX296Camera   # 局部 import：沒用到就不載入 V4L2 那一包
        return IMX296Camera(cfg.camera, hw, cfg.camera_calib)
    if hw.backend == "webcam":
        return RealCamera(cfg.camera, device=int(hw.device) if hw.device.isdigit() else 0)
    raise ValueError(f"未知的 camera_hw.backend: {hw.backend!r}（可用：imx296 / webcam）")


class RealCamera(Camera):
    """真實相機（cv2.VideoCapture）。P5 硬體到位後接上並以標定內參覆蓋。

    device：整數 = USB 相機編號；含 '!' 的字串 = GStreamer 管線（Orin 的 CSI 相機用 nvarguscamerasrc）。
    """

    def __init__(self, cam_cfg: CameraConfig, device: int | str = 0):
        self.cam_cfg = cam_cfg
        self.device = device
        self._cap = None  # 延後開啟，避免無硬體環境 import 就失敗

    def _ensure_open(self):
        if self._cap is None:
            import cv2  # 局部 import：CI/模擬環境不需要真的開相機
            if isinstance(self.device, str) and "!" in self.device:
                self._cap = cv2.VideoCapture(self.device, cv2.CAP_GSTREAMER)
                return
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


class SharedCamera(Camera):
    """一台相機給多個使用者（自主迴圈 + 網頁串流）：背景執行緒持續擷取。

    read()   ：等一張「呼叫之後才開始拍」的畫面（自主迴圈用：轉塔停穩後才拍，不會拿到移動中的舊畫面）
    latest() ：立刻回傳最新一張（串流用，可能是舊的）
    """

    def __init__(self, camera: Camera, fps: float = 30.0):
        self.camera, self.period = camera, 1.0 / fps
        self._cond = threading.Condition()
        self._frame: np.ndarray | None = None
        self._t_start = -1.0                       # 最新一張「開始拍」的時間
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="shared-camera", daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            t0 = time.monotonic()
            try:
                img = self.camera.read()
            except Exception:
                img = None
            if img is not None:
                with self._cond:
                    self._frame, self._t_start = img, t0
                    self._cond.notify_all()
            self._stop.wait(max(0.0, self.period - (time.monotonic() - t0)))

    def read(self, timeout: float = 2.0) -> np.ndarray:
        t_req = time.monotonic()
        with self._cond:
            if not self._cond.wait_for(lambda: self._t_start >= t_req, timeout):
                raise RuntimeError(f"SharedCamera：{timeout:.1f} s 內沒有新畫面")
            return self._frame

    def latest(self) -> np.ndarray | None:
        with self._cond:
            return self._frame

    def intrinsics(self) -> np.ndarray:
        return self.camera.intrinsics()

    def release(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        self.camera.release()
