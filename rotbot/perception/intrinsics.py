# -*- coding: utf-8 -*-
"""相機內參標定 —— 用棋盤格求 fx/fy/cx/cy 與畸變係數。

純演算法、不碰硬體（相機擷取與網頁預覽在 tools/calibrate_camera.py）：
  * make_board_pdf   —— 產生可列印的棋盤格
  * find_corners     —— 在一張灰階影像裡找棋盤格角點（次像素）
  * ViewCollector    —— 決定哪些畫面值得收：要拿穩、要跟已收的不一樣、要涵蓋整個畫面
  * solve_intrinsics —— cv2.calibrateCamera + 剔除壞照片

內參（像素單位的焦距、主點、畸變）跟棋盤格的實際格子大小無關，格子大小只影響「板子離多遠」的估計；
所以列印比例不準沒關係，重點是板子要平、格子不能變形。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import cv2
import numpy as np

PATTERN = (9, 6)        # 內角點數（欄, 列）＝ 10×7 個格子
SQUARE_M = 0.025        # 格子邊長 (m)
ZONES = ("左上", "上", "右上", "左", "中", "右", "左下", "下", "右下")


def make_board_pdf(path: str, pattern: tuple[int, int] = PATTERN, square_mm: float = 25.0) -> None:
    """輸出 A4 橫式的棋盤格 PDF（列印時選「實際大小 / 100%」）。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    page_w, page_h = 297.0, 210.0
    nx, ny = pattern[0] + 1, pattern[1] + 1
    x0, y0 = (page_w - nx * square_mm) / 2, (page_h - ny * square_mm) / 2
    fig = plt.figure(figsize=(page_w / 25.4, page_h / 25.4))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, page_w)
    ax.set_ylim(0, page_h)
    ax.axis("off")
    for i in range(nx):
        for j in range(ny):
            if (i + j) % 2 == 0:
                ax.add_patch(Rectangle((x0 + i * square_mm, y0 + j * square_mm),
                                       square_mm, square_mm, color="black", lw=0))
    ax.text(page_w / 2, y0 / 2, f"{pattern[0]}x{pattern[1]} inner corners, {square_mm:g} mm squares"
            " - print at 100% (no scaling), glue onto something flat and rigid",
            ha="center", va="center", fontsize=8)
    fig.savefig(path)
    plt.close(fig)


def find_corners(gray: np.ndarray, pattern: tuple[int, int] = PATTERN) -> np.ndarray | None:
    """找棋盤格內角點，回傳 (N, 2) float32（原圖像素座標）；沒找到回傳 None。

    先在縮小一半的圖上找（快），再回到原圖做次像素精修。
    """
    small = cv2.resize(gray, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
    ok, corners = cv2.findChessboardCorners(
        small, pattern,
        flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK)
    if not ok:
        return None
    corners = corners.astype(np.float32) * 2.0 + 0.5
    cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                     (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.01))
    return corners.reshape(-1, 2)


class ViewCollector:
    """從連續畫面裡挑出適合標定的幾張。"""

    GRID = (6, 4)            # 把畫面切成 6×4 格來統計角點涵蓋到哪裡

    def __init__(self, image_size: tuple[int, int], target: int = 20,
                 steady_px: float = 2.0, min_coverage: float = 0.8):
        self.width, self.height = image_size
        self.target, self.steady_px, self.min_coverage = target, steady_px, min_coverage
        # 角點平均要移動這麼多才算新的一張。板子必須整張在畫面內，能移動的範圍有限，
        # 門檻太高的話收十幾張就再也收不進去（原本用 6%，模擬操作卡在 13 張）
        self.novel_px = 0.03 * math.hypot(*image_size)
        self.views: list[np.ndarray] = []
        self.covered = np.zeros(self.GRID[::-1], bool)
        self._last: tuple[np.ndarray, float] | None = None

    def offer(self, corners: np.ndarray | None, now: float) -> str:
        """餵一張畫面的角點。回傳 "none" | "moving" | "duplicate" | "added"。"""
        if corners is None:
            self._last = None
            return "none"
        last, self._last = self._last, (corners, now)
        # 跟上一張比：沒拿穩的畫面有動態模糊，角點不準
        if last is None or now - last[1] > 1.0 or \
                np.abs(corners - last[0]).max() > self.steady_px:
            return "moving"
        if any(np.linalg.norm(corners - v, axis=1).mean() < self.novel_px for v in self.views):
            return "duplicate"
        self.views.append(corners)
        gx = np.clip((corners[:, 0] * self.GRID[0] / self.width).astype(int), 0, self.GRID[0] - 1)
        gy = np.clip((corners[:, 1] * self.GRID[1] / self.height).astype(int), 0, self.GRID[1] - 1)
        self.covered[gy, gx] = True
        return "added"

    @property
    def coverage(self) -> float:
        return float(self.covered.mean())

    @property
    def done(self) -> bool:
        return len(self.views) >= self.target and self.coverage >= self.min_coverage

    def missing_zones(self) -> list[str]:
        """還沒被角點涵蓋到的區域（九宮格方位名稱），給使用者看要把板子移去哪。"""
        rows = np.array_split(np.arange(self.GRID[1]), 3)
        cols = np.array_split(np.arange(self.GRID[0]), 3)
        return [ZONES[r * 3 + c] for r in range(3) for c in range(3)
                if not self.covered[np.ix_(rows[r], cols[c])].all()]


@dataclass
class IntrinsicsResult:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    dist: list[float]            # k1, k2, p1, p2, k3
    rms_px: float                # 重投影誤差（均方根，像素）
    n_views: int
    per_view_px: list[float] = field(default_factory=list)

    def matrix(self) -> np.ndarray:
        return np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1.0]])

    def fov_deg(self) -> tuple[float, float]:
        return (2 * math.degrees(math.atan(self.width / 2 / self.fx)),
                2 * math.degrees(math.atan(self.height / 2 / self.fy)))

    def max_distortion_px(self) -> float:
        """畸變在畫面四角造成的位移（像素）：判斷畸變要不要處理。"""
        corners = np.array([[[0.0, 0.0]], [[self.width - 1.0, self.height - 1.0]]])
        ideal = cv2.undistortPoints(corners, self.matrix(), np.array(self.dist), P=self.matrix())
        return float(np.abs(ideal - corners).max())

    def quality(self) -> str:
        if self.rms_px < 0.5:
            return "良好"
        return "可用" if self.rms_px < 1.0 else "不佳，建議重拍"

    def yaml_block(self, note: str = "") -> str:
        hfov, vfov = self.fov_deg()
        return (f"camera:           # 實機標定{note}：RMS {self.rms_px:.2f} px、{self.n_views} 張、"
                f"FOV {hfov:.1f}°×{vfov:.1f}°\n"
                f"  fx: {self.fx:.1f}\n  fy: {self.fy:.1f}\n  cx: {self.cx:.1f}\n  cy: {self.cy:.1f}\n"
                f"  width: {self.width}\n  height: {self.height}\n"
                f"# 畸變係數 (k1, k2, p1, p2, k3)：{[round(d, 5) for d in self.dist]}"
                f"，四角位移 {self.max_distortion_px():.1f} px\n")


def solve_intrinsics(views: list[np.ndarray], image_size: tuple[int, int],
                     pattern: tuple[int, int] = PATTERN,
                     square_m: float = SQUARE_M) -> IntrinsicsResult:
    """由多張角點求內參。誤差明顯偏大的照片會剔除後重算一次。"""
    if len(views) < 6:
        raise ValueError(f"至少要 6 張，目前只有 {len(views)} 張")
    grid = np.zeros((pattern[0] * pattern[1], 3), np.float32)
    grid[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2) * square_m

    def run(vs):
        obj = [grid] * len(vs)
        img = [v.reshape(-1, 1, 2).astype(np.float32) for v in vs]
        rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
            obj, img, image_size, None, None, flags=cv2.CALIB_FIX_K3)
        errs = []
        for o, i, r, t in zip(obj, img, rvecs, tvecs):
            proj, _ = cv2.projectPoints(o, r, t, K, dist)
            errs.append(float(np.sqrt(((proj - i) ** 2).sum(axis=2).mean())))
        return rms, K, dist.ravel(), errs

    rms, K, dist, errs = run(views)
    limit = max(2.0 * float(np.median(errs)), 0.5)
    keep = [v for v, e in zip(views, errs) if e <= limit]
    if 6 <= len(keep) < len(views):
        views = keep
        rms, K, dist, errs = run(views)
    return IntrinsicsResult(
        fx=float(K[0, 0]), fy=float(K[1, 1]), cx=float(K[0, 2]), cy=float(K[1, 2]),
        width=image_size[0], height=image_size[1],
        dist=[float(d) for d in np.resize(dist, 5)], rms_px=float(rms),
        n_views=len(views), per_view_px=errs)
