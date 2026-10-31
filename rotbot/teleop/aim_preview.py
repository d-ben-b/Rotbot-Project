# -*- coding: utf-8 -*-
"""預估落點 —— 轉塔「目前」角度 + 飛輪轉速 → 球會打在網板哪裡（手動操控的瞄準輔助）。

畫面正中央只是「相機正前方」，不是落點：球走拋物線，而相機跟著槍管一起上仰，
6 m 時真正的落點比畫面中心低 1.2–1.6 m（約在畫面下方 1/6 處）。
這裡用跟 AI 瞄準同一個彈道模型（含阻力）往前算：
  1. v0 = v0_model × 飛輪目標轉速 / rpm_model（打牆擬合後 v0_model 就是實測值，見 tools/fit_ballistics.py）
  2. 有校正器時扣掉它學到的全域偏差（AI 自主 60 秒、或賽前練習存的校正檔學到的「手感」）
  3. 彈道穿過網板平面 x = dist_x 的點 → 最近的孔、球會不會整顆進
  4. 用轉塔角度算同軸相機的外參，把落點投影到畫面上（HUD 標記）＋右下角網板小地圖
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ..config import Config
from ..decision import ballistics, frames

ROWS, COLS = ("上", "中", "下"), ("左", "中", "右")
GREEN, ORANGE = (60, 220, 60), (0, 165, 255)          # BGR：會進 / 不會進


def hole_label(group: str, idx: int) -> str:
    """孔的中文名：main 0~8（上左→下右）→「中排中」；bonus 0~2 →「加分左」。"""
    return f"{ROWS[idx // 3]}排{COLS[idx % 3]}" if group == "main" else f"加分{COLS[idx]}"


@dataclass
class Preview:
    z: float                          # 網板平面上的落點：橫向（m，右為正）
    y: float                          # 落點離地高度（m）
    hole: str                         # 最近的孔 "main#4" / "bonus#1"
    label: str                        # 中文名「中排中」
    off_cm: tuple[float, float]       # 落點 − 孔心（右 +、上 +；cm）
    inside: bool                      # 球整顆進孔（球心離孔心 ≤ (孔徑 − 球徑)/2）
    v0: float                         # 計算用的出射速度 (m/s)
    rpm: float                        # 計算用的飛輪轉速
    px: tuple[float, float] | None    # 落點在相機影像上的像素（原解析度）；None = 沒有內參或在相機後方
    px_r: float = 0.0                 # 球在影像上的半徑（像素）


class AimPreview:
    def __init__(self, cfg: Config, calib=None):
        """calib：AdaptiveCalibrator，或回傳它（或 None）的函式——比賽主程式每場換新的校正器。"""
        self.cfg, self.calib = cfg, calib
        f = cfg.field
        self.holes = [(g, i, d, f.grid_cx + C[0], f.grid_cy + C[1]) for g, i, d, C in cfg.hole_centers()]

    def v0_for(self, rpm: float) -> float:
        return self.cfg.launch.v0_model * rpm / self.cfg.launch.rpm_model

    def _bias(self) -> tuple[float, float]:
        c = self.calib() if callable(self.calib) else self.calib
        return (0.0, 0.0) if c is None else (float(c.g_bias[0]), float(c.g_bias[1]))

    def landing(self, pitch_deg: float, yaw_deg: float, rpm: float) -> tuple[float, float] | None:
        """(橫向 z, 離地 y)；球到不了網板平面（含先落地）回傳 None。"""
        if rpm <= 0:
            return None
        # 校正器下指令 = 模型解 + 修正 → 實際落點 ≈ 模型(指令 − 修正)
        bp, by = self._bias()
        shoot = ballistics.model_shoot(self.cfg, self.v0_for(rpm))
        land = shoot(math.radians(pitch_deg) - bp, math.radians(yaw_deg) - by)
        if land is None or land[1] < 0:                  # 到網板前就落地
            return None
        return land

    def predict(self, pitch_deg: float, yaw_deg: float, rpm: float,
                K: np.ndarray | None = None) -> Preview | None:
        land = self.landing(pitch_deg, yaw_deg, rpm)
        if land is None:
            return None
        z, y = land
        g, i, d, hz, hy = min(self.holes, key=lambda h: math.hypot(z - h[3], y - h[4]))
        ez, ey = z - hz, y - hy
        inside = math.hypot(ez, ey) <= (d - self.cfg.ball.diameter) / 2
        px, px_r = None, 0.0
        if K is not None:                             # 同軸相機：外參跟著轉塔角度走（不受校正偏差影響）
            R_cl, t_cl = frames.camera_extrinsic_at(self.cfg, pitch_deg, yaw_deg)
            pc = R_cl.T @ (np.array([self.cfg.field.dist_x, y - self.cfg.field.launch_h, z]) - t_cl)
            if pc[2] > 0.1:
                uv = K @ pc
                px = (float(uv[0] / uv[2]), float(uv[1] / uv[2]))
                px_r = float(K[0, 0] * self.cfg.ball.diameter / 2 / pc[2])
        return Preview(z=z, y=y, hole=f"{g}#{i}", label=hole_label(g, i), off_cm=(ez * 100, ey * 100),
                       inside=inside, v0=self.v0_for(rpm), rpm=rpm, px=px, px_r=px_r)

    # ── HUD ─────────────────────────────────────────
    def draw(self, img: np.ndarray, pv: Preview | None, scale: float = 1.0) -> None:
        """畫面上的落點標記（畫面外就在邊緣畫箭頭）＋右下角網板小地圖。scale = HUD 影像寬 / 原影像寬。"""
        import cv2
        h, w = img.shape[:2]
        col = GREEN if pv is not None and pv.inside else ORANGE
        if pv is not None and pv.px is not None:
            u, v = pv.px[0] * scale, pv.px[1] * scale
            if 0 <= u < w and 0 <= v < h:
                c, r = (int(round(u)), int(round(v))), max(5, int(round(pv.px_r * scale)))
                cv2.circle(img, c, r + 1, (0, 0, 0), 3, cv2.LINE_AA)
                cv2.circle(img, c, r, col, 2, cv2.LINE_AA)
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):        # 四個短刻度，球很小時也看得到
                    p0 = (c[0] + dx * (r + 3), c[1] + dy * (r + 3))
                    p1 = (c[0] + dx * (r + 11), c[1] + dy * (r + 11))
                    cv2.line(img, p0, p1, col, 2, cv2.LINE_AA)
            else:                                     # 落點在畫面外：邊緣箭頭指向它
                cx, cy = w / 2, h / 2
                du, dv = u - cx, v - cy
                k = min((cx - 24) / abs(du) if du else math.inf, (cy - 24) / abs(dv) if dv else math.inf)
                tip = (int(cx + du * k), int(cy + dv * k))
                tail = (int(cx + du * k * 0.8), int(cy + dv * k * 0.8))
                cv2.arrowedLine(img, tail, tip, col, 3, cv2.LINE_AA, tipLength=0.35)
        self._draw_map(img, pv, col)

    def _draw_map(self, img: np.ndarray, pv: Preview | None, col) -> None:
        """右下角：網板示意圖（孔位按比例）＋預估落點。網板不在畫面內時也看得到瞄到哪。"""
        import cv2
        h, w = img.shape[:2]
        f = self.cfg.field
        z0, z1 = f.grid_cx - 1.5, f.grid_cx + 1.5                   # 顯示範圍：寬 3 m
        y0, y1 = f.grid_cy - 1.1, f.grid_cy + 1.5                   # 高 2.6 m（下排以下到加分排以上）
        mw = max(120, int(w * 0.2))
        s = mw / (z1 - z0)
        mh = int((y1 - y0) * s)
        x_off, y_off = w - mw - 8, h - mh - 8
        if x_off < 0 or y_off < 0:                                   # 畫面太小就不畫
            return
        roi = img[y_off:y_off + mh, x_off:x_off + mw]
        roi[:] = (roi * 0.2).astype(np.uint8)                        # 半透明暗底
        to_px = lambda z, y: (int(x_off + (z - z0) * s), int(y_off + (y1 - y) * s))
        for g, i, d, hz, hy in self.holes:
            hit = pv is not None and pv.inside and pv.hole == f"{g}#{i}"
            cv2.circle(img, to_px(hz, hy), max(2, int(d / 2 * s)), GREEN if hit else (170, 170, 170), 1,
                       cv2.LINE_AA)
        cv2.rectangle(img, (x_off, y_off), (x_off + mw - 1, y_off + mh - 1), (90, 90, 90), 1)
        if pv is None:
            cv2.putText(img, "NO SHOT", (x_off + 6, y_off + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (170, 170, 170), 1, cv2.LINE_AA)
            return
        zc = min(max(pv.z, z0 + 0.05), z1 - 0.05)                   # 超出範圍就貼在邊上
        yc = min(max(pv.y, y0 + 0.05), y1 - 0.05)
        cv2.circle(img, to_px(zc, yc), max(3, int(self.cfg.ball.diameter / 2 * s) + 1), col, -1, cv2.LINE_AA)
