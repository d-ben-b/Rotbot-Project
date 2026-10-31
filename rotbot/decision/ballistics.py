# -*- coding: utf-8 -*-
"""拋體模型 —— 機器人「內部」用來瞄準的模型。

兩種腦內模型（configs/field.yaml 的 aim.model）：
  * vacuum：無空氣阻力解析解（舊版；2.5m 時阻力影響小，堪用）。
  * drag  ：含平方空氣阻力的數值解。6m 必須用它：30g/Ø7cm 泡棉球終端速度僅 ~16 m/s，
            6m 處阻力讓落點比無阻力解低 40~110cm（需多抬 8~14°），超出線上校正 ±5° 的範圍。
腦內模型用的是「認定值」(v0_model, aim.cd_model)；與真實世界 (v0_real, air.cd) 的落差
交給 decision.calibration 線上校正。

原型：presentation/sim_adaptive_calibration.py 的 ideal_elevation()。
"""
from __future__ import annotations

from functools import lru_cache

import numpy as np

from ..config import Config


def ideal_elevation(v0: float, dist: float, dy: float, g: float) -> float | None:
    """無空氣阻力的理想拋體解（低平彈道解），回傳俯仰角 (rad)。

    v0: 出射速度 (m/s)；dist: 水平距離 (m)；dy: 目標相對發射口的高度差 (m)。
    射程不足時回傳 None。
    """
    disc = v0 ** 4 - g * (g * dist ** 2 + 2 * dy * v0 ** 2)
    if disc < 0:
        return None
    return float(np.arctan((v0 ** 2 - np.sqrt(disc)) / (g * dist)))


# ══════════════════════════ 腦內瞄準模型 ══════════════════════════
class VacuumAimModel:
    """無阻力解析模型。"""

    def __init__(self, v0: float, g: float):
        self.v0, self.g = v0, g

    def elevation(self, r: float, dy: float) -> float | None:
        """水平距 r、高度差 dy → 低平彈道俯仰角 (rad)；射程不足回傳 None。"""
        return ideal_elevation(self.v0, r, dy, self.g)

    def height(self, r: float, pitch: float) -> float | None:
        """以俯仰角 pitch 發射，在水平距 r 處相對發射口的高度 (m)。"""
        c = np.cos(pitch)
        if c <= 1e-9:
            return None
        return float(r * np.tan(pitch) - self.g * r * r / (2 * self.v0 ** 2 * c * c))


class DragAimModel:
    """含平方阻力的數值模型：預先積分一族仰角的彈道，查表反解仰角。

    無風、無旋轉時彈道留在含初速度的鉛直面內 → 2D 問題（水平距 s、垂直 y）。
    對固定 (v0, k/m) 一次向量化 RK4 積分 pitch 格點上每條彈道，記錄等距水平距格點上的高度
    Y[i, j]；反解時取 s=r 的那一欄，沿仰角找第一個由下往上穿越 dy 的點（低平彈道解）再內插。
    建表約 0.1 s、反解約 0.1 ms；(v0, k) 改變（例如試投擬合後）才需重建。
    """

    def __init__(self, v0: float, k_over_m: float, g: float, dt: float = 0.002,
                 pitch_min_deg: float = -15.0, pitch_max_deg: float = 75.0,
                 pitch_step_deg: float = 0.25, s_max: float = 12.0, s_step: float = 0.02,
                 t_max: float = 3.0, y_floor: float = -3.0):
        self.v0, self.kd, self.g = v0, k_over_m, g
        self.pitch = np.radians(np.arange(pitch_min_deg, pitch_max_deg + 1e-9, pitch_step_deg))
        self.d_pitch = np.radians(pitch_step_deg)
        self.s_step = s_step
        n_p, n_s = len(self.pitch), int(round(s_max / s_step)) + 1
        Y = np.full((n_p, n_s), np.nan)
        Y[:, 0] = 0.0

        s = np.zeros(n_p)
        y = np.zeros(n_p)
        vs = v0 * np.cos(self.pitch)
        vy = v0 * np.sin(self.pitch)
        nxt = np.ones(n_p, dtype=int)            # 下一個要填的水平距格點
        done = np.zeros(n_p, dtype=bool)
        kd = k_over_m

        def acc(a_vs, a_vy):
            sp = np.sqrt(a_vs * a_vs + a_vy * a_vy)
            return -kd * sp * a_vs, -g - kd * sp * a_vy

        for _ in range(int(t_max / dt)):
            s0, y0 = s.copy(), y.copy()
            a1 = acc(vs, vy)
            a2 = acc(vs + 0.5 * dt * a1[0], vy + 0.5 * dt * a1[1])
            a3 = acc(vs + 0.5 * dt * a2[0], vy + 0.5 * dt * a2[1])
            a4 = acc(vs + dt * a3[0], vy + dt * a3[1])
            s = s + dt * vs + dt * dt / 6 * (a1[0] + a2[0] + a3[0])
            y = y + dt * vy + dt * dt / 6 * (a1[1] + a2[1] + a3[1])
            vs = vs + dt / 6 * (a1[0] + 2 * a2[0] + 2 * a3[0] + a4[0])
            vy = vy + dt / 6 * (a1[1] + 2 * a2[1] + 2 * a3[1] + a4[1])

            j_hi = np.minimum(np.floor(s / s_step).astype(int), n_s - 1)
            while True:                              # 本步跨過的格點逐一線性內插
                m = ~done & (nxt <= j_hi)
                if not m.any():
                    break
                idx = np.nonzero(m)[0]
                sj = nxt[idx] * s_step
                f = (sj - s0[idx]) / np.maximum(s[idx] - s0[idx], 1e-12)
                Y[idx, nxt[idx]] = y0[idx] + f * (y[idx] - y0[idx])
                nxt[idx] += 1
            done |= (y < y_floor) | (nxt >= n_s)
            if done.all():
                break
        self.Y = Y

    def _column(self, r: float) -> np.ndarray | None:
        j = r / self.s_step
        j0 = int(np.floor(j))
        if j0 < 0 or j0 + 1 >= self.Y.shape[1]:
            return None
        f = j - j0
        return self.Y[:, j0] * (1 - f) + self.Y[:, j0 + 1] * f

    def elevation(self, r: float, dy: float) -> float | None:
        """水平距 r、高度差 dy → 低平彈道俯仰角 (rad)；射程不足回傳 None。"""
        h = self._column(r)
        if h is None:
            return None
        d = h - dy
        ok = np.isfinite(d[:-1]) & np.isfinite(d[1:]) & (d[:-1] <= 0) & (d[1:] > 0)
        hits = np.nonzero(ok)[0]
        if len(hits) == 0:
            return None
        i = int(hits[0])
        frac = -d[i] / (d[i + 1] - d[i])
        return float(self.pitch[i] + frac * self.d_pitch)

    def height(self, r: float, pitch: float) -> float | None:
        """以俯仰角 pitch 發射，在水平距 r 處相對發射口的高度 (m)；到不了回傳 None。"""
        h = self._column(r)
        if h is None:
            return None
        k = (pitch - self.pitch[0]) / self.d_pitch
        i = int(np.floor(k))
        if i < 0 or i + 1 >= len(self.pitch):
            return None
        val = h[i] * (1 - (k - i)) + h[i + 1] * (k - i)
        return float(val) if np.isfinite(val) else None


@lru_cache(maxsize=8)
def _cached_model(kind: str, v0: float, k_over_m: float, g: float, dt: float):
    if kind == "vacuum":
        return VacuumAimModel(v0, g)
    if kind == "drag":
        return DragAimModel(v0, k_over_m, g, dt=dt)
    raise ValueError(f"未知的 aim.model: {kind!r}（可用 drag / vacuum）")


def aim_model(cfg: Config, v0: float | None = None):
    """依設定取得（快取的）腦內瞄準模型；v0 省略時用 v0_model。"""
    v0 = cfg.launch.v0_model if v0 is None else v0
    k_over_m = cfg.k_drag_model / cfg.ball.mass
    return _cached_model(cfg.aim.model, float(v0), float(k_over_m), float(cfg.air.g),
                         float(cfg.aim.dt))


# ══════════════════════════ 對外介面 ══════════════════════════
def aim_at_point(target_launcher: tuple[float, float, float] | np.ndarray,
                 cfg: Config) -> tuple[float, float] | None:
    """對發射點系任意 3D 目標 (x前, y上, z右) 解瞄準 (俯仰 rad, 水平 rad)。

    圓孔版主用介面：目標 3D 由 frames.hole_target_point 得來（距離逐孔不同）。
    水平距離取 hypot(x, z)、高度差取 y；射程不足回傳 None。
    """
    x, y, z = np.asarray(target_launcher, float)
    pitch = aim_model(cfg).elevation(float(np.hypot(x, z)), float(y))
    if pitch is None:
        return None
    yaw = float(np.arctan2(z, x))
    return pitch, yaw


def aim_for_target(target_zy: tuple[float, float], cfg: Config) -> tuple[float, float] | None:
    """由網框平面上目標的 (橫向 z, 離地高度 y)，解出瞄準 (俯仰 rad, 水平 rad)。

    這是決策層對外的主要介面：給目標位置 → 得瞄準角。射程不足回傳 None。
    """
    z_t, y_t = target_zy
    return aim_at_point((cfg.field.dist_x, y_t - cfg.field.launch_h, z_t), cfg)


def model_shoot(cfg: Config, v0: float | None = None):
    """回傳腦內模型的「投射」函式 shoot(pitch, yaw) -> (z, y離地) 或 None。

    落點取彈道穿過 x = dist_x 平面處，座標慣例同 sim.world.BallisticWorld.shoot；
    供 estimate_sensitivity 用（實機上沒有「真實世界」可呼叫，只能用模型估靈敏度），
    以及手動操控的預估落點（v0 依飛輪轉速換算；省略時用 v0_model）。
    """
    model = aim_model(cfg, v0)
    x_plane, h0 = cfg.field.dist_x, cfg.field.launch_h

    def shoot(pitch: float, yaw: float):
        c = np.cos(yaw)
        if c <= 1e-6:
            return None
        h = model.height(x_plane / c, pitch)
        if h is None:
            return None
        return float(x_plane * np.tan(yaw)), float(h0 + h)

    return shoot


def estimate_sensitivity(shoot, target_zy: tuple[float, float], cfg: Config,
                         step_deg: float = 0.5) -> tuple[float, float]:
    """數值估計靈敏度 ∂高度/∂俯仰、∂橫向/∂水平 (m/rad)。

    用於把「落點位置誤差」換算成「角度修正量」。
    shoot: callable(pitch_rad, yaw_rad) -> (z, y) 落點；實機用 model_shoot(cfg)，
           也可傳模擬世界或實測資料擬合的函式。
    """
    aim = aim_for_target(target_zy, cfg)
    if aim is None:
        raise ValueError(f"目標 {target_zy} 超出腦內模型射程，無法估靈敏度")
    p0, yaw0 = aim
    h = np.radians(step_deg)

    _, y_a = shoot(p0 + h, yaw0)
    _, y_b = shoot(p0 - h, yaw0)
    z_a, _ = shoot(p0, yaw0 + h)
    z_b, _ = shoot(p0, yaw0 - h)
    j_pitch = (y_a - y_b) / (2 * h)
    j_yaw = (z_a - z_b) / (2 * h)
    return float(j_pitch), float(j_yaw)
