# -*- coding: utf-8 -*-
"""SharedCamera：轉塔停下後只用之後才曝光的畫面；相機停掉時串流與網頁要看得出來（不需硬體）。"""
import time
from types import SimpleNamespace

import cv2
import numpy as np

from rotbot.config import load_config
from rotbot.perception.camera import SharedCamera
from rotbot.perception.imx296 import IMX296Camera
from rotbot.teleop.server import FrameSource, TeleopServer


class FakeSensor:
    """60 fps 連續出圖、曝光 8 ms；read() 回傳最新一張已拍完的（沒有新的就等），同 V4L2Capture.grab()。"""

    P, EXP = 1 / 60, 0.008

    def __init__(self):
        self.t0 = time.monotonic()
        self.last_k = -1
        self.start_of: dict[int, float] = {}       # id(畫面) → 實際開始曝光的時間

    def read(self):
        while True:
            k = int((time.monotonic() - self.t0) / self.P) - 1      # 最新一張拍完的編號
            if k > self.last_k:
                break
            time.sleep(0.001)
        self.last_k = k
        img = np.zeros((4, 4, 3), np.uint8)
        self.start_of[id(img)] = self.exposure_start()
        return img

    def exposure_start(self):
        done = self.t0 + (self.last_k + 1) * self.P             # 讀出結束
        return done - self.P - self.EXP

    def intrinsics(self):
        return np.eye(3)

    def release(self):
        pass


class DyingCamera:
    """前 n 張正常，之後每次逾時（排線鬆脫、VI 停止）。"""

    def __init__(self, n=5):
        self.n, self.calls = n, 0

    def read(self):
        self.calls += 1
        if self.calls > self.n:
            time.sleep(0.05)
            raise RuntimeError("相機沒有送出影像（逾時）")
        return np.full((240, 320, 3), 90, np.uint8)

    def intrinsics(self):
        return np.eye(3)

    def release(self):
        pass


def test_read_waits_for_exposure_after_settle():
    """read() 拿到的畫面，曝光開始一定晚於「呼叫時間 + settle_s」（以前會拿到呼叫前就開始拍的）。"""
    sensor = FakeSensor()
    cam = SharedCamera(sensor, fps=30, settle_s=0.05)
    try:
        for _ in range(8):
            t_req = time.monotonic()
            img = cam.read()
            assert sensor.start_of[id(img)] >= t_req + 0.05
            time.sleep(0.013)                              # 讓要求時間落在 frame 週期的不同相位
    finally:
        cam.release()


def test_frozen_camera_is_flagged_on_stream_and_page():
    """相機停掉：latest() 仍是最後一張，但 age() 變大，串流畫面加紅條、網頁狀態有警告。"""
    cam = SharedCamera(DyingCamera(), fps=50)
    fs = FrameSource(cam, fps=20)
    try:
        end = time.monotonic() + 3.0
        while (cam.latest() is None or cam.age() < 0.8) and time.monotonic() < end:   # 還沒有畫面時 age 是 inf
            time.sleep(0.05)
        assert cam.latest() is not None and 0.8 <= cam.age() < 5.0
        warning = TeleopServer(camera=cam).snapshot()["camera_warning"]
        assert "沒有新畫面" in warning and "小地圖" in warning
        fs.start()
        end = time.monotonic() + 2.0
        while fs.jpeg is None and time.monotonic() < end:
            time.sleep(0.02)
        img = cv2.imdecode(np.frombuffer(fs.jpeg, np.uint8), cv2.IMREAD_COLOR)
        bar = img[30:58].reshape(-1, 3).mean(axis=0)       # BGR；上方狀態列下面的紅條
        assert bar[2] > 150 and bar[0] < 100
    finally:
        fs.stop()
        cam.release()


class GlitchCamera:
    """第 5 次起每次失敗，直到被 release()（重開串流）才恢復，像 CSI 出錯後要 stream off / on 的 VI。"""

    def __init__(self):
        self.calls, self.releases, self.broken = 0, 0, False

    def read(self):
        self.calls += 1
        if self.calls == 5:
            self.broken = True
        if self.broken:
            raise RuntimeError("相機沒有送出影像（逾時）")
        return np.full((240, 320, 3), self.calls % 256, np.uint8)

    def intrinsics(self):
        return np.eye(3)

    def release(self):
        self.releases += 1
        self.broken = False


def test_camera_reopens_after_repeated_failures():
    """連續失敗 REOPEN_AFTER 次就 release()，下一次 read() 重新開；畫面恢復。"""
    inner = GlitchCamera()
    cam = SharedCamera(inner, fps=50)
    try:
        end = time.monotonic() + 3.0
        while (cam.reopens == 0 or cam.age() > 0.1) and time.monotonic() < end:
            time.sleep(0.02)
        assert cam.reopens == 1 and inner.releases == 1
        assert cam.age() < 0.1 and inner.calls > 6          # 重開後又有新畫面
    finally:
        cam.release()


def test_healthy_camera_has_no_warning():
    cam = SharedCamera(FakeSensor(), fps=30)
    try:
        cam.read()
        assert TeleopServer(camera=cam).snapshot()["camera_warning"] == ""
    finally:
        cam.release()


def test_imx296_exposure_start_is_conservative():
    """sensor 時間戳是 monotonic 時鐘時往前扣一個週期 + 曝光；不是的話改用呼叫 grab() 的時間再多扣一個週期。"""
    cfg = load_config()
    cam = IMX296Camera(cfg.camera, cfg.camera_hw, cfg.camera_calib)    # 不開裝置
    exp, period = cfg.camera_hw.exposure_us * 1e-6, 1 / 60
    ts = time.monotonic() - 0.010
    cam._cap = SimpleNamespace(timestamp=ts)
    assert abs(cam.exposure_start() - (ts - period - exp)) < 1e-9
    cam._cap = SimpleNamespace(timestamp=time.time())                  # 別的時鐘（差幾十年）
    cam._t_call = time.monotonic()
    assert abs(cam.exposure_start() - (cam._t_call - 2 * period - exp)) < 1e-9
    cam._cap = None
