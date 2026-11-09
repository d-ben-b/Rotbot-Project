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


def open_camera(spec: str, cfg: Config) -> Camera:
    """工具的 --camera 參數：cfg = 依 field.yaml 的 camera_hw（目前 IMX296）；數字 = USB 相機；含 '!' = GStreamer 管線。"""
    if spec == "cfg":
        return make_camera(cfg)
    return RealCamera(cfg.camera, device=int(spec) if spec.isdigit() else spec)


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

    read()   ：等一張「曝光在呼叫之後 settle_s 秒以上才開始」的畫面
               （自主迴圈用：轉塔停下後等支架晃動停了才拍，不會拿到移動中的舊畫面）
    latest() ：立刻回傳最新一張（串流用，可能是舊的）
    age()    ：距離上一張新畫面幾秒（相機停掉時一直變大；串流用來顯示警告）

    曝光開始時間：相機有 exposure_start()（IMX296：由 sensor 時間戳保守估計）就用它；
    沒有的話用呼叫 camera.read() 的時間（數位孿生在 read() 當下才算圖，這樣是對的）。

    自動重開：連續 REOPEN_AFTER 次讀取失敗（IMX296 每次逾時 2.5 s）就 release()，下一次 read() 重新開串流。
    推測 CSI 傳輸出錯後 Jetson 的 VI 要重開串流才恢復（未實測）；reopens 記錄重開幾次，網頁警告會顯示。
    """

    REOPEN_AFTER = 2
    RETRY_S = 0.2              # 讀取失敗後等多久再試（開不起來的相機會立刻失敗，不要一秒試幾十次）

    def __init__(self, camera: Camera, fps: float = 30.0, settle_s: float = 0.0):
        self.camera, self.period, self.settle_s = camera, 1.0 / fps, settle_s
        self._cond = threading.Condition()
        self._frame: np.ndarray | None = None
        self._t_start = -1.0                       # 最新一張開始曝光的時間
        self._t_got: float | None = None           # 最新一張拿到的時間
        self._fails = 0                            # 連續讀取失敗次數
        self.reopens = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="shared-camera", daemon=True)
        self._thread.start()

    def _exposure_start(self, t_call: float) -> float:
        f = getattr(self.camera, "exposure_start", None)
        if f is None:
            return t_call
        try:
            return float(f())
        except Exception:
            return t_call

    def _reopen(self):
        try:
            self.camera.release()                  # IMX296 / RealCamera：下一次 read() 會重新開啟
        except Exception:
            pass
        self.reopens += 1
        self._fails = 0

    def _run(self):
        while not self._stop.is_set():
            t0 = time.monotonic()
            try:
                img = self.camera.read()
            except Exception:
                img = None
            if img is None:
                self._fails += 1
                if self._fails >= self.REOPEN_AFTER:
                    self._reopen()
                self._stop.wait(self.RETRY_S)
                continue
            self._fails = 0
            t_start = self._exposure_start(t0)
            with self._cond:
                self._frame, self._t_start, self._t_got = img, t_start, time.monotonic()
                self._cond.notify_all()
            self._stop.wait(max(0.0, self.period - (time.monotonic() - t0)))

    def read(self, timeout: float = 2.0) -> np.ndarray:
        t_req = time.monotonic() + self.settle_s
        with self._cond:
            if not self._cond.wait_for(lambda: self._t_start >= t_req, timeout + self.settle_s):
                raise RuntimeError(f"SharedCamera：{timeout:.1f} s 內沒有新畫面")
            return self._frame

    def latest(self) -> np.ndarray | None:
        with self._cond:
            return self._frame

    def age(self) -> float:
        """距離上一張新畫面的秒數；還沒有任何畫面時是 inf。"""
        with self._cond:
            return float("inf") if self._t_got is None else time.monotonic() - self._t_got

    def intrinsics(self) -> np.ndarray:
        return self.camera.intrinsics()

    def release(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        self.camera.release()
