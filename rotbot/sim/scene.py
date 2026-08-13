# -*- coding: utf-8 -*-
"""合成場景產生器 —— P1 感知的資料源頭（OpenCV/PIL 程式化合成）。

把九宮格當成「板座標系」裡的剛性平面：每格 40cm 網框的 4 角座標已知，
以隨機相機視角投影，得到**像素級精確**的角點標註（給 YOLO-pose 訓練）。
再疊上域隨機化（背景、眩光、亮度色偏、模糊、雜訊）逼近現場干擾。

座標慣例（相機系）：+x 右、+y 下、+z 前，與 perception.pose 一致。
板座標系：原點在九宮格中心，+x 右、+y 上、+z 出板面（正對時朝相機）。
"""
from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np

from ..config import Config

# 每格 4 角在板座標系的偏移，順序 左上→右上→右下→左下（與 pose.SOLVEPNP_IPPE_SQUARE 一致）
_CORNER_SIGNS = np.array([[-1, 1], [1, 1], [1, -1], [-1, -1]], dtype=np.float64)

# 九宮格 9 格在板座標系的 (col, row) 係數，順序：上排左 → 下排右（與 strategy.grid_cells 對齊）
_CELL_RC = [(c, r) for r in (1, 0, -1) for c in (-1, 0, 1)]


@dataclass
class CellLabel:
    """一格的標註。"""
    cell_id: int              # 0~8，上排左→下排右
    corners: np.ndarray       # (4, 2) 像素座標，順序 左上→右上→右下→左下
    bbox: tuple[float, float, float, float]   # (x1, y1, x2, y2)
    is_bonus: bool
    visible: bool             # 4 角是否都在畫面內且在相機前方
    z: float = 0.0            # 格中心在相機座標的深度 (m)；>0 表示在相機前方


@dataclass
class SceneSample:
    """一張合成影像 + 其標註。"""
    image: np.ndarray         # HxWx3 BGR uint8
    cells: list[CellLabel]
    viewpoint: dict = field(default_factory=dict)   # 供除錯：距離/方位/傾角…


def _rot_x(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _rot_y(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rot_z(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


class SceneGenerator:
    """隨機視角 + 域隨機化的九宮格合成器。"""

    def __init__(self, cfg: Config, rng: np.random.Generator | None = None,
                 randomize: bool = True):
        self.cfg = cfg
        self.rng = rng if rng is not None else np.random.default_rng(cfg.sim.seed)
        self.randomize = randomize
        self.K = cfg.camera.matrix()
        self.W = cfg.camera.width
        self.H = cfg.camera.height
        # 九宮格所有格的 4 角在板座標系 (36, 3)，Z=0
        s = cfg.field.frame_d
        pts = []
        for (c, r) in _CELL_RC:
            cx, cy = c * cfg.field.col_pitch, r * cfg.field.row_pitch
            for sx, sy in _CORNER_SIGNS:
                pts.append([cx + sx * s / 2, cy + sy * s / 2, 0.0])
        self._obj = np.array(pts, dtype=np.float64)      # (36, 3)

    # ── 幾何 ─────────────────────────────────────────────
    def _random_viewpoint(self) -> dict:
        r = self.rng
        if not self.randomize:
            return dict(distance=self.cfg.field.dist_x, bearing=0.0, elev=0.0,
                        board_yaw=0.0, board_pitch=0.0, roll=0.0)
        return dict(
            distance=float(r.uniform(2.0, 3.2)),
            bearing=float(r.uniform(-18, 18)),      # 板中心在畫面的水平角 (deg)
            elev=float(r.uniform(-6, 12)),          # 垂直角 (deg)，相機通常略低於格中心
            board_yaw=float(r.uniform(-22, 22)),    # 板繞垂直軸傾斜（被打歪/斜看）
            board_pitch=float(r.uniform(-10, 10)),  # 板繞水平軸傾斜
            roll=float(r.uniform(-5, 5)),           # 相機滾轉 (deg)
        )

    def _project(self, vp: dict) -> np.ndarray:
        """把 36 個角點投影到像素，回傳 (9, 4, 2)。"""
        b, e = np.radians(vp["bearing"]), np.radians(vp["elev"])
        d = vp["distance"]
        center = d * np.array([np.sin(b) * np.cos(e), -np.sin(e), np.cos(b) * np.cos(e)])
        # 板→相機基礎旋轉（正對時：板 up → 相機 up=-y，板法線 → 朝相機 -z）
        R0 = np.diag([1.0, -1.0, -1.0])
        R_tilt = _rot_y(np.radians(vp["board_yaw"])) @ _rot_x(np.radians(vp["board_pitch"]))
        R_roll = _rot_z(np.radians(vp["roll"]))
        p_cam = (R0 @ R_tilt @ self._obj.T).T + center
        p_cam = (R_roll @ p_cam.T).T
        uv = (self.K @ p_cam.T).T
        uv = uv[:, :2] / uv[:, 2:3]
        # 記下深度以判可見與繪圖順序
        self._last_z = p_cam[:, 2].reshape(9, 4).mean(axis=1)
        return uv.reshape(9, 4, 2)

    def _make_labels(self, corners_9x4: np.ndarray, bonus_id: int | None) -> list[CellLabel]:
        labels = []
        for i in range(9):
            c = corners_9x4[i]
            x1, y1 = c.min(axis=0)
            x2, y2 = c.max(axis=0)
            inside = (c[:, 0] >= 0).all() and (c[:, 0] < self.W).all() and \
                     (c[:, 1] >= 0).all() and (c[:, 1] < self.H).all()
            visible = bool(inside and self._last_z[i] > 0)
            labels.append(CellLabel(cell_id=i, corners=c,
                                    bbox=(float(x1), float(y1), float(x2), float(y2)),
                                    is_bonus=(i == bonus_id), visible=visible,
                                    z=float(self._last_z[i])))
        return labels

    # ── 繪圖 ─────────────────────────────────────────────
    def _draw_background(self, img: np.ndarray) -> None:
        r = self.rng
        base = r.integers(20, 90, size=3) if self.randomize else np.array([40, 45, 50])
        img[:] = base.astype(np.uint8)
        if not self.randomize:
            return
        # 線性亮度漸層
        grad = np.linspace(r.uniform(-30, 0), r.uniform(0, 40), self.H).reshape(-1, 1, 1)
        img[:] = np.clip(img.astype(np.float32) + grad, 0, 255).astype(np.uint8)
        # 隨機雜物矩形（背景干擾）
        for _ in range(r.integers(0, 6)):
            p1 = (int(r.uniform(0, self.W)), int(r.uniform(0, self.H)))
            p2 = (int(r.uniform(0, self.W)), int(r.uniform(0, self.H)))
            col = tuple(int(x) for x in r.integers(0, 120, size=3))
            cv2.rectangle(img, p1, p2, col, -1)

    def _draw_cell(self, img: np.ndarray, label: CellLabel) -> None:
        r = self.rng
        pts = label.corners.astype(np.int32)
        # 框後方的暗色面板
        overlay = img.copy()
        cv2.fillConvexPoly(overlay, pts, (25, 25, 28))
        cv2.addWeighted(overlay, 0.5, img, 0.5, 0, img)
        # 框邊（網框輪廓）
        thick = int(r.integers(3, 8)) if self.randomize else 5
        rim = (200, 200, 205) if not self.randomize else tuple(int(x) for x in r.integers(120, 255, size=3))
        cv2.polylines(img, [pts], isClosed=True, color=rim, thickness=thick, lineType=cv2.LINE_AA)
        # 亮起的加分格：綠色高亮 + 內圈 + 光暈
        if label.is_bonus:
            cv2.polylines(img, [pts], isClosed=True, color=(80, 255, 120),
                          thickness=thick + 3, lineType=cv2.LINE_AA)
            inner = (label.corners * 0.5 + label.corners.mean(axis=0) * 0.5).astype(np.int32)
            cv2.polylines(img, [inner], isClosed=True, color=(120, 255, 160),
                          thickness=max(2, thick - 1), lineType=cv2.LINE_AA)

    def _post_process(self, img: np.ndarray) -> np.ndarray:
        if not self.randomize:
            return img
        r = self.rng
        out = img.astype(np.float32)
        # 眩光光斑（LED/反光）
        for _ in range(r.integers(0, 3)):
            cx, cy = r.uniform(0, self.W), r.uniform(0, self.H)
            rad = r.uniform(40, 240)
            yy, xx = np.ogrid[:self.H, :self.W]
            g = np.exp(-((xx - cx) ** 2 + (yy - cy) ** 2) / (2 * rad ** 2))
            out += (g[..., None] * r.uniform(60, 200)).astype(np.float32)
        # 亮度/對比/色偏
        out = out * r.uniform(0.7, 1.3) + r.uniform(-25, 25)
        out *= r.uniform(0.9, 1.1, size=3)              # 每通道色偏
        out = np.clip(out, 0, 255).astype(np.uint8)
        # 模糊（散焦/運動）
        k = int(r.integers(0, 3)) * 2 + 1
        if k > 1:
            out = cv2.GaussianBlur(out, (k, k), 0)
        # 感測雜訊
        out = np.clip(out.astype(np.float32) + r.normal(0, r.uniform(1, 8), out.shape),
                      0, 255).astype(np.uint8)
        return out

    # ── 對外 ─────────────────────────────────────────────
    def sample(self) -> SceneSample:
        """產生一張合成影像 + 標註。"""
        vp = self._random_viewpoint()
        corners = self._project(vp)
        bonus_id = None
        if self.rng.random() < 0.9:                     # 多數情況有一格亮起
            bonus_id = int(self.rng.integers(0, 9))
        labels = self._make_labels(corners, bonus_id)

        img = np.zeros((self.H, self.W, 3), dtype=np.uint8)
        self._draw_background(img)
        for i in np.argsort(-self._last_z):             # 由遠到近繪製
            if labels[i].visible or self._last_z[i] > 0:
                self._draw_cell(img, labels[i])
        img = self._post_process(img)
        vp["bonus_id"] = bonus_id
        return SceneSample(image=img, cells=labels, viewpoint=vp)

    def render(self) -> np.ndarray:
        """便利介面：回傳一張新樣本的影像（供 sim.camera.SimCamera 使用）。"""
        return self.sample().image
