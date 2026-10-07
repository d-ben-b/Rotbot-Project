# -*- coding: utf-8 -*-
"""IMX296 相機後端：ISP 與設定（不需硬體）。實機煙霧測試要設 ROTBOT_HW=1 才跑。"""
import ctypes
import os
import sys

import numpy as np
import pytest

import cv2

from rotbot.config import CameraCalibration, CameraConfig, CameraHardwareConfig, load_config
from rotbot.perception import imx296
from rotbot.perception.camera import make_camera
from rotbot.perception.imx296 import BLACK_LEVEL, BayerISP, IMX296Camera, grey_world_gains

H, W = 64, 96


def _raw(r, g, b, shape=(H, W)):
    """用 10-bit 的 R/G/B 站值組一張 RGGB raw（值放在 16-bit 高位，跟 V4L2 給的一樣）。"""
    raw = np.empty(shape, np.uint16)
    raw[0::2, 0::2] = r
    raw[0::2, 1::2] = g
    raw[1::2, 0::2] = g
    raw[1::2, 1::2] = b
    return raw << 6


def test_isp_shape_and_neutral_grey():
    # 灰色物體在 sensor 上：R、B 站比 G 站暗，暗的倍數就是白平衡增益
    wb_r, wb_b, g = 3.0, 2.0, 300
    raw = _raw(BLACK_LEVEL + g / wb_r, BLACK_LEVEL + g, BLACK_LEVEL + g / wb_b)
    for ccm in (imx296.CCM_4560K, None):
        out = BayerISP(wb_r, wb_b, ccm)(raw)
        assert out.shape == (H, W, 3) and out.dtype == np.uint8
        px = out[H // 2, W // 2].astype(int)
        assert px.max() - px.min() <= 2, px          # 中性灰：三通道相等
        assert 100 < px.mean() < 200


def test_isp_black_and_saturated_are_neutral():
    for ccm in (imx296.CCM_4560K, None):
        isp = BayerISP(3.0, 2.0, ccm)
        assert isp(_raw(BLACK_LEVEL, BLACK_LEVEL, BLACK_LEVEL)).max() == 0
        # 過曝：三個站都到頂 → 必須是白色，不能因為白平衡增益而偏洋紅
        assert isp(_raw(1023, 1023, 1023)).min() == 255


def test_isp_channel_order_is_bgr():
    # 只有 R 站有訊號 → 輸出的紅色通道（BGR 的 index 2）最大
    out = BayerISP(1.0, 1.0, None)(_raw(800, BLACK_LEVEL, BLACK_LEVEL))
    b, g, r = out[H // 2, W // 2]
    assert r > 150 and r > g and r > b
    out = BayerISP(1.0, 1.0, None)(_raw(BLACK_LEVEL, BLACK_LEVEL, 800))
    b, g, r = out[H // 2, W // 2]
    assert b > 150 and b > g and b > r


def test_isp_ccm_increases_saturation():
    # 偏紅的物體：色彩矩陣應該把它拉得更紅（sensor 原生 RGB 偏淡）
    raw = _raw(BLACK_LEVEL + 400, BLACK_LEVEL + 250, BLACK_LEVEL + 200)
    flat = BayerISP(1.0, 1.0, None)(raw)[H // 2, W // 2].astype(int)
    vivid = BayerISP(1.0, 1.0, imx296.CCM_4560K)(raw)[H // 2, W // 2].astype(int)
    assert (vivid[2] - vivid[1]) > (flat[2] - flat[1])


def test_isp_banding_has_no_seams(monkeypatch):
    # 平行分帶處理的結果必須跟整張一次處理完全相同（含接縫列）
    rng = np.random.default_rng(0)
    raw = (rng.integers(0, 1024, (240, 320)).astype(np.uint16)) << 6
    for ccm in (imx296.CCM_4560K, None):
        isp = BayerISP(3.05, 1.85, ccm)
        banded = isp(raw)
        monkeypatch.setattr(imx296, "N_BANDS", 1)
        assert np.array_equal(banded, isp(raw))


def test_grey_world_recovers_gains():
    g = 300
    raw = _raw(BLACK_LEVEL + g / 2.5, BLACK_LEVEL + g, BLACK_LEVEL + g / 1.6, shape=(128, 128))
    wb_r, wb_b = grey_world_gains(raw)
    assert abs(wb_r - 2.5) < 0.05 and abs(wb_b - 1.6) < 0.05
    with pytest.raises(ValueError):
        grey_world_gains(_raw(BLACK_LEVEL, BLACK_LEVEL, BLACK_LEVEL, shape=(128, 128)))


def test_camera_hw_config_defaults_and_optional(tmp_path):
    cfg = load_config()
    assert cfg.camera_hw.backend == "imx296"
    assert cfg.camera_hw.exposure_us == 8333
    # 舊的設定檔沒有 camera_hw 區段也要能載入（用預設值）
    import yaml
    from rotbot.config import DEFAULT_CONFIG
    raw = yaml.safe_load(open(DEFAULT_CONFIG, encoding="utf-8"))
    raw.pop("camera_hw")
    legacy = tmp_path / "legacy.yaml"
    legacy.write_text(yaml.safe_dump(raw), encoding="utf-8")
    assert load_config(legacy).camera_hw == CameraHardwareConfig()


def test_camera_refuses_wrong_intrinsics_without_opening_device():
    # 建構不能碰硬體；沒標定且內參解析度對不上 sensor 時 intrinsics() 要拒絕，而不是回傳錯的 K
    example = CameraConfig(fx=1400, fy=1400, cx=960, cy=540, width=1920, height=1080)
    cam = IMX296Camera(example, CameraHardwareConfig(device="/dev/does-not-exist"))
    assert (cam.width, cam.height) == (1456, 1088)
    with pytest.raises(RuntimeError):
        cam.intrinsics()
    matching = CameraConfig(fx=1700, fy=1700, cx=728, cy=544, width=1456, height=1088)
    assert IMX296Camera(matching).intrinsics()[0, 0] == 1700
    assert isinstance(make_camera(load_config()), IMX296Camera)


CALIB = CameraCalibration(width=1456, height=1088, fx=1900.0, fy=1905.0, cx=750.0, cy=580.0,
                          dist=[-0.49, 0.17, -0.001, 0.003, 0.0], out_f=1770.0)


def test_calibrated_camera_reports_output_intrinsics():
    # 有標定資料時：解析度與內參都以標定為準，跟模擬用的 camera: 無關
    example = CameraConfig(fx=1400, fy=1400, cx=960, cy=540, width=1920, height=1080)
    cam = IMX296Camera(example, calib=CALIB)
    assert (cam.width, cam.height) == (1456, 1088)
    K = cam.intrinsics()
    assert K[0, 0] == K[1, 1] == 1770.0 and (K[0, 2], K[1, 2]) == (750.0, 580.0)
    with pytest.raises(ValueError):
        IMX296Camera(example, calib=CameraCalibration(width=640, height=480, fx=1, fy=1, cx=1, cy=1))


def test_undistort_puts_points_where_the_pinhole_model_says():
    # 在「鏡頭原始影像」上，把一批 3D 點畫在帶畸變的投影位置；去畸變後，
    # 它們必須落在 intrinsics() 的針孔投影位置上（誤差 < 1 px）
    cam = IMX296Camera(load_config().camera, calib=CALIB)
    xs, ys = np.meshgrid(np.linspace(-0.36, 0.36, 7), np.linspace(-0.26, 0.26, 5))
    pts3d = np.stack([xs.ravel(), ys.ravel(), np.ones(xs.size)], 1)
    raw_px, _ = cv2.projectPoints(pts3d, np.zeros(3), np.zeros(3), CALIB.matrix(), np.array(CALIB.dist))
    img = np.zeros((cam.height, cam.width, 3), np.uint8)
    for x, y in raw_px.reshape(-1, 2):
        cv2.circle(img, (int(round(x)), int(round(y))), 4, (255, 255, 255), -1)
    out = cam.undistort(img)
    assert out.shape == img.shape
    n, _, _, centroids = cv2.connectedComponentsWithStats((out[..., 0] > 100).astype(np.uint8))
    found = centroids[1:]
    expected = (pts3d[:, :2] / pts3d[:, 2:]) * 1770.0 + np.array([750.0, 580.0])
    assert len(found) == len(expected)
    for e in expected:
        assert np.linalg.norm(found - e, axis=1).min() < 1.0
    # 畸變量夠大，沒去畸變的話這個測試一定會失敗
    assert np.abs(raw_px.reshape(-1, 2) - expected).max() > 10
    # 雙線性內插的結果也要落在同樣的位置
    lin = IMX296Camera(load_config().camera, CameraHardwareConfig(undistort_interp="linear"), CALIB)
    _, _, _, c2 = cv2.connectedComponentsWithStats((lin.undistort(img)[..., 0] > 100).astype(np.uint8))
    for e in expected:
        assert np.linalg.norm(c2[1:] - e, axis=1).min() < 1.0
    # 沒有標定資料：影像原樣回傳
    assert IMX296Camera(load_config().camera).undistort(img) is img


def test_camera_calib_section_is_optional(tmp_path):
    import yaml
    from rotbot.config import DEFAULT_CONFIG
    raw = yaml.safe_load(open(DEFAULT_CONFIG, encoding="utf-8"))
    raw.pop("camera_calib", None)
    plain = tmp_path / "no_calib.yaml"
    plain.write_text(yaml.safe_dump(raw), encoding="utf-8")
    assert load_config(plain).camera_calib is None
    raw["camera_calib"] = dict(width=1456, height=1088, fx=1900.0, fy=1900.0, cx=728.0, cy=544.0,
                               dist=[-0.4, 0.1, 0, 0, 0], out_f=1750.0)
    withc = tmp_path / "calib.yaml"
    withc.write_text(yaml.safe_dump(raw), encoding="utf-8")
    c = load_config(withc).camera_calib
    assert c.dist == (-0.4, 0.1, 0.0, 0.0, 0.0) and c.output_matrix()[0, 0] == 1750.0


@pytest.mark.skipif(not sys.platform.startswith("linux") or ctypes.sizeof(ctypes.c_long) != 8,
                    reason="V4L2 結構大小只在 64-bit Linux 上有意義")
def test_v4l2_struct_layout_matches_kernel():
    assert ctypes.sizeof(imx296._Buffer) == 88
    assert ctypes.sizeof(imx296._Format) == 208
    assert ctypes.sizeof(imx296._ExtControl) == 20
    assert ctypes.sizeof(imx296._ExtControls) == 32
    assert imx296.VIDIOC_DQBUF == 0xC0585611
    assert imx296.VIDIOC_S_EXT_CTRLS == 0xC0205648


@pytest.mark.skipif(os.environ.get("ROTBOT_HW") != "1", reason="需要實體相機：ROTBOT_HW=1 pytest")
def test_hardware_smoke():
    cam = make_camera(load_config())
    try:
        frame = cam.read()
        assert frame.shape == (cam.height, cam.width, 3) and frame.dtype == np.uint8
        t0 = cam.timestamp
        cam.read()
        assert cam.timestamp > t0
    finally:
        cam.release()
