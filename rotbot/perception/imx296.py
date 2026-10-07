# -*- coding: utf-8 -*-
"""相機 HAL 實作 —— Raspberry Pi Global Shutter Camera（Sony IMX296）@ Jetson Orin。

資料流：sensor → CSI → Tegra VI → V4L2 (/dev/video0, RG10 raw Bayer) → 本檔 ISP → BGR uint8。
不經過 NVIDIA Argus：Argus 沒有這顆 sensor 的 tuning，自動曝光會持續震盪，而且無法固定短曝光。

三個部分：
  * BayerISP      —— raw Bayer → BGR（黑階、白平衡、去馬賽克、色彩矩陣、gamma）。純 numpy/OpenCV，
                     不碰硬體，任何平台都能跑、可單元測試。
  * V4L2Capture   —— 直接用 ioctl + mmap 取 raw frame。只在 Linux 可用（延後 import fcntl）。
  * IMX296Camera  —— 接到 hal.Camera 介面，pipeline 用這個。

驅動（kernel module + device-tree overlay）不在本 repo，安裝方式見 docs/imx296_jetson.md。
"""
from __future__ import annotations

import ctypes
import os
import time

import cv2
import numpy as np

from ..config import CameraConfig, CameraHardwareConfig
from ..hal import Camera

# ══════════════════════════ sensor 常數 ══════════════════════════
# (寬, 高, 最高 fps)。兩個都是驅動提供的模式；720p 是中央裁切，不是縮放。
SENSOR_MODES = ((1456, 1088, 60), (1280, 720, 90))
BLACK_LEVEL = 60          # 10-bit 黑階（Raspberry Pi 標定資料 3840/64）
WHITE_LEVEL = 1023
EXPOSURE_US_RANGE = (15, 15700)
GAIN_DB_RANGE = (0.0, 48.0)

# 色彩校正矩陣（sensor RGB → sRGB，線性域、白平衡之後套用）。
# 來源：Raspberry Pi libcamera 的 imx296 標定檔，4560K 那一組。每列總和為 1（白色不變）。
CCM_4560K = np.array([[2.1526, -0.8479, -0.3047],
                      [-0.4842, 2.2896, -0.8054],
                      [-0.1511, -0.5301, 1.6813]])


def _srgb_encode(x: np.ndarray) -> np.ndarray:
    """線性 0~1 → sRGB 編碼 0~1。"""
    return np.where(x <= 0.0031308, 12.92 * x, 1.055 * np.power(x, 1 / 2.4) - 0.055)


# ══════════════════════════ ISP ══════════════════════════
class BayerISP:
    """RG10 raw（RGGB 排列，10-bit 值放在 16-bit 的高位）→ BGR uint8。

    ccm=None 走快速路徑（8-bit 去馬賽克、不做色彩矩陣）：顏色較淡但更快，
    適合只看亮度/形狀的用途。
    整張影像切成數條水平帶平行處理（numpy/OpenCV 運算時會放開 GIL）。
    """

    BANDS = 4
    _MARGIN = 2     # 每條帶上下多算 2 列再裁掉，去馬賽克在接縫處才不會有邊界誤差（偶數＝不動 Bayer 相位）

    def __init__(self, wb_r: float = 3.05, wb_b: float = 1.85,
                 ccm: np.ndarray | None = CCM_4560K):
        lin = np.clip(np.arange(1024, dtype=np.float64) - BLACK_LEVEL, 0, None)
        lin /= WHITE_LEVEL - BLACK_LEVEL
        # 白平衡後截在 1.0：過曝的像素三個通道都會到頂 → 白色，不會偏洋紅
        sites = [np.clip(lin * g, 0, 1) for g in (wb_r, 1.0, wb_b)]      # R, G, B
        if ccm is None:
            # 查表直接得到 sRGB 編碼的 8-bit 馬賽克
            self._site_lut = np.concatenate(
                [(_srgb_encode(v) * 255 + 0.5).astype(np.uint8) for v in sites])
            self._ccm_bgr = self._gamma = None
        else:
            # 線性 16-bit 馬賽克 → 去馬賽克 → 色彩矩陣（順便縮到 12-bit）→ gamma 查表
            self._site_lut = np.concatenate(
                [(v * 65535 + 0.5).astype(np.uint16) for v in sites])
            # cv2.transform 作用在 BGR 上：把 RGB 矩陣的列與行都反過來
            self._ccm_bgr = (np.asarray(ccm, dtype=np.float32)[::-1, ::-1] / 16).copy()
            # 矩陣有負係數與 >1 的係數，輸出可能超過 4095：表做大一點，超出的都是 255
            self._gamma = np.full(16384, 255, np.uint8)
            self._gamma[:4096] = (_srgb_encode(np.arange(4096) / 4095.0) * 255 + 0.5).astype(np.uint8)
        self._site_offset: np.ndarray | None = None
        self._pool = None

    def _offsets(self, shape) -> np.ndarray:
        """每個像素在 _site_lut 裡的起點：R 站 0、G 站 1024、B 站 2048。"""
        if self._site_offset is None or self._site_offset.shape != shape:
            off = np.full(shape, 1024, np.uint16)
            off[0::2, 0::2] = 0
            off[1::2, 1::2] = 2048
            self._site_offset = off
        return self._site_offset

    def _band(self, raw16, offsets, out, y0, y1):
        h = raw16.shape[0]
        a, b = max(y0 - self._MARGIN, 0), min(y1 + self._MARGIN, h)
        idx = raw16[a:b] >> 6
        idx += offsets[a:b]
        mosaic = np.take(self._site_lut, idx, mode="clip")
        # OpenCV 的 "BayerBG" 指的是 (1,1) 為 B，也就是 R 在 (0,0) 的 RGGB
        bgr = cv2.cvtColor(mosaic, cv2.COLOR_BayerBG2BGR)
        if self._ccm_bgr is not None:
            bgr = np.take(self._gamma, cv2.transform(bgr, self._ccm_bgr), mode="clip")
        out[y0:y1] = bgr[y0 - a:y1 - a]

    def __call__(self, raw16: np.ndarray) -> np.ndarray:
        """raw16: (H, W) uint16，V4L2 給的原始值（寬高須為偶數）。回傳 (H, W, 3) BGR uint8。"""
        h, w = raw16.shape
        out = np.empty((h, w, 3), np.uint8)
        offsets = self._offsets(raw16.shape)
        step = -(-h // self.BANDS // 2) * 2          # 帶的高度取偶數，接縫才會落在 Bayer 週期上
        bands = [(y, min(y + step, h)) for y in range(0, h, step)]
        if len(bands) == 1:
            self._band(raw16, offsets, out, 0, h)
            return out
        if self._pool is None:
            from concurrent.futures import ThreadPoolExecutor
            self._pool = ThreadPoolExecutor(self.BANDS, thread_name_prefix="isp")
        list(self._pool.map(lambda yy: self._band(raw16, offsets, out, *yy), bands))
        return out


def grey_world_gains(raw16: np.ndarray) -> tuple[float, float]:
    """從一張 raw 估白平衡增益 (wb_r, wb_b)：假設畫面平均是灰的。

    排除接近全黑與過曝的像素。現場對著白紙/灰牆量一次，把結果寫進 field.yaml。
    """
    v = (raw16 >> 6).astype(np.float64) - BLACK_LEVEL
    r, g1, g2, b = v[0::2, 0::2], v[0::2, 1::2], v[1::2, 0::2], v[1::2, 1::2]
    hi = 0.9 * (WHITE_LEVEL - BLACK_LEVEL)
    ok = (np.minimum(np.minimum(r, b), np.minimum(g1, g2)) > 8) & \
         (np.maximum(np.maximum(r, b), np.maximum(g1, g2)) < hi)
    if ok.sum() < 1000:
        raise ValueError("可用像素太少（畫面太暗或過曝），無法估白平衡")
    g = (g1[ok].mean() + g2[ok].mean()) / 2
    return float(g / r[ok].mean()), float(g / b[ok].mean())


# ══════════════════════════ V4L2 ══════════════════════════
_u32, _i32 = ctypes.c_uint32, ctypes.c_int32


class _PixFormat(ctypes.Structure):
    _fields_ = [(n, _u32) for n in (
        "width", "height", "pixelformat", "field", "bytesperline", "sizeimage",
        "colorspace", "priv", "flags", "ycbcr_enc", "quantization", "xfer_func")]


class _FormatUnion(ctypes.Union):
    _fields_ = [("pix", _PixFormat), ("raw", ctypes.c_uint8 * 200), ("_align", ctypes.c_void_p)]


class _Format(ctypes.Structure):
    _fields_ = [("type", _u32), ("fmt", _FormatUnion)]


class _RequestBuffers(ctypes.Structure):
    _fields_ = [("count", _u32), ("type", _u32), ("memory", _u32),
                ("capabilities", _u32), ("flags", ctypes.c_uint8), ("reserved", ctypes.c_uint8 * 3)]


class _Timeval(ctypes.Structure):
    _fields_ = [("tv_sec", ctypes.c_long), ("tv_usec", ctypes.c_long)]


class _Timecode(ctypes.Structure):
    _fields_ = [("type", _u32), ("flags", _u32), ("frames", ctypes.c_uint8),
                ("seconds", ctypes.c_uint8), ("minutes", ctypes.c_uint8),
                ("hours", ctypes.c_uint8), ("userbits", ctypes.c_uint8 * 4)]


class _BufferM(ctypes.Union):
    _fields_ = [("offset", _u32), ("userptr", ctypes.c_ulong),
                ("planes", ctypes.c_void_p), ("fd", _i32)]


class _Buffer(ctypes.Structure):
    _fields_ = [("index", _u32), ("type", _u32), ("bytesused", _u32), ("flags", _u32),
                ("field", _u32), ("timestamp", _Timeval), ("timecode", _Timecode),
                ("sequence", _u32), ("memory", _u32), ("m", _BufferM),
                ("length", _u32), ("reserved2", _u32), ("request_fd", _i32)]


class _ExtControl(ctypes.Structure):
    _pack_ = 1
    _fields_ = [("id", _u32), ("size", _u32), ("reserved2", _u32), ("value64", ctypes.c_int64)]


class _ExtControls(ctypes.Structure):
    _fields_ = [("which", _u32), ("count", _u32), ("error_idx", _u32), ("request_fd", _i32),
                ("reserved", _u32), ("controls", ctypes.POINTER(_ExtControl))]


def _ioc(direction: int, nr: int, struct) -> int:
    return (direction << 30) | (ctypes.sizeof(struct) << 16) | (ord("V") << 8) | nr


_W, _RW = 1, 3
VIDIOC_S_FMT = _ioc(_RW, 5, _Format)
VIDIOC_REQBUFS = _ioc(_RW, 8, _RequestBuffers)
VIDIOC_QUERYBUF = _ioc(_RW, 9, _Buffer)
VIDIOC_QBUF = _ioc(_RW, 15, _Buffer)
VIDIOC_DQBUF = _ioc(_RW, 17, _Buffer)
VIDIOC_STREAMON = _ioc(_W, 18, ctypes.c_int)
VIDIOC_STREAMOFF = _ioc(_W, 19, ctypes.c_int)
VIDIOC_S_EXT_CTRLS = _ioc(_RW, 72, _ExtControls)

_BUF_TYPE_CAPTURE, _MEMORY_MMAP, _FIELD_NONE = 1, 1, 1
_FOURCC_RG10 = int.from_bytes(b"RG10", "little")
# Tegra 相機框架的控制項 ID（v4l2-ctl -l 可列出）
CID_GAIN, CID_EXPOSURE, CID_PREFERRED_STRIDE = 0x009A2009, 0x009A200A, 0x009A206E


class V4L2Capture:
    """用 mmap 緩衝區直接從 V4L2 取 raw Bayer frame（只支援 Linux）。"""

    N_BUFFERS = 4
    STARTUP_SKIP = 4      # stream-on 後前幾張是 sensor 啟動雜訊

    def __init__(self, device: str, width: int, height: int):
        self.device, self.width, self.height = device, width, height
        # Orin 的 VI 要求每列位元組數是 64 的倍數，否則 raw 資料會錯位
        self.stride = -(-width * 2 // 64) * 64
        self._fd = -1
        self._maps: list = []
        self.timestamp = 0.0      # 最近一張的 V4L2 時間戳（秒）；算時間差用這個
        self.sequence = 0         # 驅動已擷取的張數（緩衝區滿時沒擷取的不算，不能拿來算時間）

    # ---- 低階 ----
    def _ioctl(self, request: int, arg):
        import fcntl
        return fcntl.ioctl(self._fd, request, arg)

    def set_control(self, cid: int, value: int) -> None:
        ctrl = _ExtControl(id=cid, value64=int(value))
        self._ioctl(VIDIOC_S_EXT_CTRLS, _ExtControls(count=1, controls=ctypes.pointer(ctrl)))

    def _new_buffer(self, index: int = 0) -> _Buffer:
        return _Buffer(index=index, type=_BUF_TYPE_CAPTURE, memory=_MEMORY_MMAP)

    # ---- 生命週期 ----
    def start(self) -> None:
        import mmap
        self._fd = os.open(self.device, os.O_RDWR | os.O_NONBLOCK)
        try:
            self.set_control(CID_PREFERRED_STRIDE, self.stride)
            fmt = _Format(type=_BUF_TYPE_CAPTURE)
            fmt.fmt.pix.width, fmt.fmt.pix.height = self.width, self.height
            fmt.fmt.pix.pixelformat, fmt.fmt.pix.field = _FOURCC_RG10, _FIELD_NONE
            self._ioctl(VIDIOC_S_FMT, fmt)
            pix = fmt.fmt.pix
            if (pix.width, pix.height, pix.bytesperline) != (self.width, self.height, self.stride):
                raise RuntimeError(
                    f"相機不接受 {self.width}x{self.height} stride={self.stride}，"
                    f"回報 {pix.width}x{pix.height} stride={pix.bytesperline}")
            req = _RequestBuffers(count=self.N_BUFFERS, type=_BUF_TYPE_CAPTURE, memory=_MEMORY_MMAP)
            self._ioctl(VIDIOC_REQBUFS, req)
            for i in range(req.count):
                buf = self._new_buffer(i)
                self._ioctl(VIDIOC_QUERYBUF, buf)
                self._maps.append(mmap.mmap(self._fd, buf.length, mmap.MAP_SHARED,
                                            mmap.PROT_READ | mmap.PROT_WRITE, offset=buf.m.offset))
                self._ioctl(VIDIOC_QBUF, buf)
            self._ioctl(VIDIOC_STREAMON, ctypes.c_int(_BUF_TYPE_CAPTURE))
        except OSError as e:
            self.stop()
            hint = "（另一個程式正在用相機？）" if e.errno == 16 else ""
            raise RuntimeError(f"無法開啟相機串流 {self.device}: {e}{hint}") from e
        for _ in range(self.STARTUP_SKIP):
            self.grab()

    def stop(self) -> None:
        if self._fd < 0:
            return
        try:
            self._ioctl(VIDIOC_STREAMOFF, ctypes.c_int(_BUF_TYPE_CAPTURE))
        except OSError:
            pass
        for m in self._maps:
            m.close()
        self._maps = []
        os.close(self._fd)
        self._fd = -1

    # ---- 取像 ----
    def _dequeue(self) -> _Buffer | None:
        buf = self._new_buffer()
        try:
            self._ioctl(VIDIOC_DQBUF, buf)
        except BlockingIOError:
            return None
        return buf

    def grab(self, timeout: float = 2.5) -> np.ndarray:
        """回傳最新的一張 raw frame (H, W) uint16；積在佇列裡的舊張全部丟掉。"""
        import select
        deadline = time.monotonic() + timeout
        while True:
            done = []
            while (buf := self._dequeue()) is not None:
                done.append(buf)
            # 全部緩衝區都滿了 = 驅動沒緩衝區可用、已經停止取像一陣子，裡面每一張都是舊的
            stale = len(done) == len(self._maps)
            newest = None if (stale or not done) else done[-1]
            frame = None
            if newest is not None:
                view = np.frombuffer(self._maps[newest.index], dtype="<u2",
                                     count=self.height * self.stride // 2)
                frame = view.reshape(self.height, self.stride // 2)[:, :self.width].copy()
                del view
                self.timestamp = newest.timestamp.tv_sec + newest.timestamp.tv_usec * 1e-6
                self.sequence = newest.sequence
            for buf in done:
                self._ioctl(VIDIOC_QBUF, buf)
            if frame is not None:
                return frame
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([self._fd], [], [], remaining)[0]:
                raise RuntimeError("相機沒有送出影像（逾時）：檢查排線，或看 dmesg 有沒有 VI 錯誤")


# ══════════════════════════ HAL ══════════════════════════
class IMX296Camera(Camera):
    """Raspberry Pi Global Shutter Camera。read() 回傳最新一張 BGR 影像。

    擷取解析度跟著 cam_cfg（內參）的寬高走，必須是 sensor 支援的模式；
    內參還是範例值（寬高對不上）時仍可 read()，但 intrinsics() 會拒絕回傳錯的 K。
    """

    def __init__(self, cam_cfg: CameraConfig, hw: CameraHardwareConfig | None = None):
        self.cam_cfg = cam_cfg
        self.hw = hw or CameraHardwareConfig()
        size = (cam_cfg.width, cam_cfg.height)
        self._calibrated = size in [(w, h) for w, h, _ in SENSOR_MODES]
        self.width, self.height = size if self._calibrated else SENSOR_MODES[0][:2]
        self.isp = BayerISP(self.hw.wb_r, self.hw.wb_b, CCM_4560K if self.hw.ccm else None)
        self._cap: V4L2Capture | None = None   # 延後開啟：沒有硬體的環境也能 import 與建構
        self._exposure_us = self._clamp(self.hw.exposure_us, EXPOSURE_US_RANGE)
        self._gain_db = self._clamp(self.hw.gain_db, GAIN_DB_RANGE)

    @staticmethod
    def _clamp(v, rng):
        return min(max(v, rng[0]), rng[1])

    def _ensure_open(self) -> V4L2Capture:
        if self._cap is None:
            cap = V4L2Capture(self.hw.device, self.width, self.height)
            cap.start()
            self._cap = cap
            self._write_controls(force=True)
        return self._cap

    def _write_controls(self, force: bool = False) -> None:
        """把曝光/增益寫進 sensor。必須在串流進行中寫，否則值到不了硬體。

        force：每次 stream-on 驅動都會把 sensor 重設成預設值，但 V4L2 仍快取著上次的值，
        而「新值等於快取」時核心不會呼叫驅動。所以開串流後先寫一個相鄰值，再寫目標值。
        """
        exposure, gain = int(round(self._exposure_us)), int(round(self._gain_db * 10))
        if force:
            self._cap.set_control(CID_EXPOSURE, exposure - 1 if exposure > 100 else exposure + 1)
            self._cap.set_control(CID_GAIN, gain - 1 if gain > 0 else gain + 1)
        self._cap.set_control(CID_EXPOSURE, exposure)
        self._cap.set_control(CID_GAIN, gain)

    # ---- hal.Camera ----
    def read(self) -> np.ndarray:
        return self.isp(self.read_raw())

    def intrinsics(self) -> np.ndarray:
        if not self._calibrated:
            raise RuntimeError(
                f"configs/field.yaml 的 camera 內參是 {self.cam_cfg.width}x{self.cam_cfg.height}，"
                f"但 IMX296 輸出 {self.width}x{self.height}。請先標定相機並覆蓋 camera: 區段，"
                "不然 PnP 算出的距離會是錯的。")
        return self.cam_cfg.matrix()

    def release(self) -> None:
        if self._cap is not None:
            self._cap.stop()
            self._cap = None

    # ---- IMX296 專屬 ----
    def read_raw(self) -> np.ndarray:
        """最新一張 raw Bayer (H, W) uint16（10-bit 值在高位，>>6 得 0~1023）。"""
        return self._ensure_open().grab()

    @property
    def timestamp(self) -> float:
        """最近一張影像的擷取時間戳（秒）。來自 sensor 時序，抖動極小，適合算兩張之間的時間差。"""
        return self._cap.timestamp if self._cap else 0.0

    @property
    def sequence(self) -> int:
        """驅動已擷取的張數。read() 太慢導致緩衝區全滿時，沒擷取的那些不會計入。"""
        return self._cap.sequence if self._cap else 0

    def set_exposure(self, exposure_us: float) -> None:
        self._exposure_us = self._clamp(exposure_us, EXPOSURE_US_RANGE)
        if self._cap is not None:
            self._write_controls()

    def set_gain(self, gain_db: float) -> None:
        self._gain_db = self._clamp(gain_db, GAIN_DB_RANGE)
        if self._cap is not None:
            self._write_controls()

    def set_white_balance(self, wb_r: float, wb_b: float) -> None:
        self.isp = BayerISP(wb_r, wb_b, CCM_4560K if self.hw.ccm else None)
