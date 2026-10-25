# -*- coding: utf-8 -*-
"""合成場景產生器 —— 圓孔九宮格（P1 感知資料源 + 姿態方法比較的真值來源）。

目標是「圓孔」（見賽事計劃書：一般孔 Ø40cm、加分孔 Ø20cm，用單目幾何/霍夫圓）。
佈局：主九宮格 3×3（Ø40）+ 最上排 3 個加分孔（Ø20，會隨機亮起指定得分格）。

以隨機相機視角把每個圓孔投影成橢圓，並提供**像素級精確的真值**：
每孔的 3D 中心、投影中心、投影橢圓、真值距離/方位/傾角 —— 供 pose 方法比較與訓練。

座標慣例（相機系）：+x 右、+y 下、+z 前。
板座標系：原點在主九宮格中心，+x 右、+y 上、+z 出板面（正對時朝相機）。
"""
from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np

from ..config import Config

_RIM_N = 48   # 每個圓取樣的 rim 點數（佈局見 Config.hole_centers）


@dataclass
class HoleLabel:
    """一個圓孔的標註 + 真值。"""
    group: str                 # "main" | "bonus"
    idx: int                   # 群組內索引（main 0~8 上左→下右；bonus 0~2 左→右）
    diameter: float            # 實體直徑 (m)
    is_bonus: bool
    is_lit: bool               # 是否亮起（僅 bonus 可能 True）
    center_board: np.ndarray   # (3,) 板座標系 3D 中心
    center_cam: np.ndarray     # (3,) 相機座標系 3D 中心（真值）
    center_px: np.ndarray      # (2,) 投影中心像素
    rim_px: np.ndarray         # (N, 2) 投影 rim 點
    ellipse: tuple | None      # ((cx,cy),(d1,d2),angle_deg) 由 rim 擬合；失敗為 None
    visible: bool
    # 真值幾何
    distance: float            # 中心到相機 (m)
    bearing: float             # 方位角 (deg)
    elev: float                # 俯仰角 (deg)
    tilt: float                # 入射傾角 (deg)：孔法線與視線夾角，0=正對


@dataclass
class SceneSample:
    image: np.ndarray
    holes: list[HoleLabel]
    viewpoint: dict = field(default_factory=dict)
    rvec: np.ndarray | None = None    # 真值板位姿（board→cam）
    tvec: np.ndarray | None = None


def _rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


class SceneGenerator:
    """隨機視角 + 域隨機化的圓孔九宮格合成器。"""

    def __init__(self, cfg: Config, rng: np.random.Generator | None = None,
                 randomize: bool = True, rim_jitter: bool = True,
                 dist_range: tuple[float, float] | None = None):
        self.cfg = cfg
        self.rng = rng if rng is not None else np.random.default_rng(cfg.sim.seed)
        self.randomize = randomize
        self.rim_jitter = rim_jitter        # False = 中性灰邊緣（demo 用，避免顏色混淆）
        # 隨機視角的距離範圍 (m)；省略時取 dist_x×(0.8, 1.28)（2.5m 時即舊版 2.0~3.2m）
        d = cfg.field.dist_x
        self.dist_range = tuple(dist_range) if dist_range is not None else (0.8 * d, 1.28 * d)
        self.K = cfg.camera.matrix()
        self.W, self.H = cfg.camera.width, cfg.camera.height
        self._holes_board = cfg.hole_centers()   # [(group, idx, diameter, center_board)]

    # ── 幾何 ──────────────────────────────────────────
    def _random_viewpoint(self) -> dict:
        r = self.rng
        if not self.randomize:
            return dict(distance=self.cfg.field.dist_x, bearing=0.0, elev=0.0,
                        board_yaw=0.0, board_pitch=0.0, roll=0.0)
        return dict(
            distance=float(r.uniform(*self.dist_range)),
            bearing=float(r.uniform(-18, 18)),
            elev=float(r.uniform(-6, 12)),
            board_yaw=float(r.uniform(-22, 22)),
            board_pitch=float(r.uniform(-10, 10)),
            roll=float(r.uniform(-5, 5)),
        )

    def _board_pose(self, vp: dict):
        """回傳真值板位姿 (R_total, t_total)：board→cam。"""
        b, e = np.radians(vp["bearing"]), np.radians(vp["elev"])
        d = vp["distance"]
        center = d * np.array([np.sin(b) * np.cos(e), -np.sin(e), np.cos(b) * np.cos(e)])
        R0 = np.diag([1.0, -1.0, -1.0])
        R_tilt = _rot_y(np.radians(vp["board_yaw"])) @ _rot_x(np.radians(vp["board_pitch"]))
        R_roll = _rot_z(np.radians(vp["roll"]))
        R_total = R_roll @ R0 @ R_tilt
        t_total = R_roll @ center
        return R_total, t_total

    def _project(self, P_board: np.ndarray, R, t):
        """board 座標點 (N,3) → 像素 (N,2) 與相機座標 (N,3)。"""
        p_cam = (R @ P_board.T).T + t
        uv = (self.K @ p_cam.T).T
        uv = uv[:, :2] / uv[:, 2:3]
        return uv, p_cam

    def _make_hole(self, group, idx, diameter, C_board, R, t) -> HoleLabel:
        # rim 點（板平面上的圓）
        th = np.linspace(0, 2 * np.pi, _RIM_N, endpoint=False)
        rim_board = C_board + (diameter / 2) * np.stack(
            [np.cos(th), np.sin(th), np.zeros_like(th)], axis=1)
        rim_px, _ = self._project(rim_board, R, t)
        (cpx,), (ccam,) = self._project(C_board[None], R, t)

        ellipse = None
        if len(rim_px) >= 5:
            try:
                ellipse = cv2.fitEllipse(rim_px.astype(np.float32))
            except cv2.error:
                ellipse = None

        z = float(ccam[2])
        visible = bool(z > 0 and 0 <= cpx[0] < self.W and 0 <= cpx[1] < self.H)
        distance = float(np.linalg.norm(ccam))
        bearing = float(np.degrees(np.arctan2(ccam[0], ccam[2])))
        elev = float(np.degrees(np.arctan2(-ccam[1], ccam[2])))
        normal_cam = R @ np.array([0.0, 0.0, 1.0])
        los = -ccam / (np.linalg.norm(ccam) + 1e-9)     # 由孔指向相機
        tilt = float(np.degrees(np.arccos(np.clip(abs(normal_cam @ los), 0, 1))))
        return HoleLabel(group=group, idx=idx, diameter=diameter,
                         is_bonus=(group == "bonus"), is_lit=False,
                         center_board=C_board, center_cam=ccam, center_px=cpx,
                         rim_px=rim_px, ellipse=ellipse, visible=visible,
                         distance=distance, bearing=bearing, elev=elev, tilt=tilt)

    # ── 繪圖 ──────────────────────────────────────────
    def _draw_background(self, img):
        r = self.rng
        base = r.integers(20, 90, size=3) if self.randomize else np.array([40, 45, 50])
        img[:] = base.astype(np.uint8)
        if not self.randomize:
            return
        grad = np.linspace(r.uniform(-30, 0), r.uniform(0, 40), self.H).reshape(-1, 1, 1)
        img[:] = np.clip(img.astype(np.float32) + grad, 0, 255).astype(np.uint8)
        for _ in range(r.integers(0, 6)):
            p1 = (int(r.uniform(0, self.W)), int(r.uniform(0, self.H)))
            p2 = (int(r.uniform(0, self.W)), int(r.uniform(0, self.H)))
            col = tuple(int(x) for x in r.integers(0, 120, size=3))
            cv2.rectangle(img, p1, p2, col, -1)

    def _draw_hole(self, img, hole: HoleLabel):
        r = self.rng
        pts = hole.rim_px.astype(np.int32)
        # 網袋內部（暗）
        cv2.fillConvexPoly(img, pts, (18, 18, 20))
        # 孔緣
        thick = int(r.integers(3, 8)) if self.randomize else 5
        if self.randomize and self.rim_jitter:
            rim = tuple(int(x) for x in r.integers(120, 255, size=3))
        else:
            rim = (200, 200, 205)
        cv2.polylines(img, [pts], True, rim, thick, cv2.LINE_AA)
        if hole.is_lit:                                 # 指定得分格：紅色 LED 燈環（官方 Q33 確認為紅）
            # 官方：網框邊緣紅色 LED 燈條、寬約14mm、~475±25lm → 呈紅色亮環 + 光暈
            if self.randomize:
                red = (int(r.integers(0, 55)), int(r.integers(0, 70)), int(r.integers(200, 256)))
                glow = int(r.integers(6, 16))
            else:
                red = (40, 40, 235)
                glow = 8
            cv2.polylines(img, [pts], True, tuple(int(c * 0.45) for c in red),
                          thick + glow, cv2.LINE_AA)      # 外圈暗紅光暈（模擬 LED bloom）
            cv2.polylines(img, [pts], True, red, thick + 3, cv2.LINE_AA)  # 主燈環（亮紅）

    def _post_process(self, img):
        if not self.randomize:
            return img
        r = self.rng
        out = img.astype(np.float32)
        for _ in range(r.integers(0, 3)):               # 眩光
            cx, cy = r.uniform(0, self.W), r.uniform(0, self.H)
            rad = r.uniform(40, 240)
            yy, xx = np.ogrid[:self.H, :self.W]
            g = np.exp(-((xx - cx) ** 2 + (yy - cy) ** 2) / (2 * rad ** 2))
            out += (g[..., None] * r.uniform(60, 200)).astype(np.float32)
        out = out * r.uniform(0.7, 1.3) + r.uniform(-25, 25)
        out *= r.uniform(0.9, 1.1, size=3)
        out = np.clip(out, 0, 255).astype(np.uint8)
        k = int(r.integers(0, 3)) * 2 + 1
        if k > 1:
            out = cv2.GaussianBlur(out, (k, k), 0)
        out = np.clip(out.astype(np.float32) + r.normal(0, r.uniform(1, 8), out.shape),
                      0, 255).astype(np.uint8)
        return out

    # ── 對外 ──────────────────────────────────────────
    def sample(self, vp: dict | None = None,
               lit_bonus: int | str | None = "random") -> SceneSample:
        """產生一張樣本。

        vp: 省略=隨機視角，給定=指定視角（demo/閉迴路用）。
        lit_bonus: "random"=隨機亮一個(預設)、None=都不亮、int=固定亮該加分孔（整場比賽用）。
        """
        vp = self._random_viewpoint() if vp is None else dict(vp)
        R, t = self._board_pose(vp)
        return self._render_pose(R, t, vp, lit_bonus)

    def sample_pose(self, R: np.ndarray, t: np.ndarray,
                    lit_bonus: int | str | None = "random") -> SceneSample:
        """以給定板位姿（board→cam 的 R, t）產生樣本。

        數位孿生用：相機姿態由轉塔實際角度算出（見 sim/twin.py），影像與幾何完全一致。
        """
        return self._render_pose(np.asarray(R, float), np.asarray(t, float).ravel(), {}, lit_bonus)

    def _render_pose(self, R, t, vp: dict, lit_bonus) -> SceneSample:
        holes = [self._make_hole(g, i, d, C, R, t) for (g, i, d, C) in self._holes_board]

        bonus = [h for h in holes if h.is_bonus]
        lit_idx = None
        if lit_bonus == "random":
            if bonus and self.rng.random() < 0.9:
                lit_idx = int(self.rng.integers(0, len(bonus)))
        elif lit_bonus is not None:
            lit_idx = int(lit_bonus)
        if lit_idx is not None and 0 <= lit_idx < len(bonus):
            bonus[lit_idx].is_lit = True
        vp["lit_bonus_idx"] = lit_idx

        img = np.zeros((self.H, self.W, 3), dtype=np.uint8)
        self._draw_background(img)
        for h in sorted(holes, key=lambda h: -h.center_cam[2]):   # 由遠到近
            if h.center_cam[2] > 0:
                self._draw_hole(img, h)
        img = self._post_process(img)

        rvec, _ = cv2.Rodrigues(R)
        return SceneSample(image=img, holes=holes, viewpoint=vp,
                           rvec=rvec, tvec=t.reshape(3, 1))

    def render(self) -> np.ndarray:
        return self.sample().image
