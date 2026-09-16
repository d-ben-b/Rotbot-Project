# -*- coding: utf-8 -*-
"""模擬世界 —— 「真實」拋體物理（含平方空氣阻力），沒有硬體時的替身。

這是機器人內部拋體模型（decision.ballistics，無阻力）所不知道的真實世界；
兩者的落差正是 decision.calibration 要線上學掉的東西。

原型：presentation/sim_adaptive_calibration.py 的 simulate_shot()。
"""
from __future__ import annotations

from math import cos, sin, sqrt

import numpy as np

from ..config import Config


class BallisticWorld:
    """含空氣阻力的拋體世界。shoot() 回傳落在網框平面上的 (z, y) 落點。"""

    def __init__(self, cfg: Config, rng: np.random.Generator | None = None):
        self.cfg = cfg
        self.rng = rng if rng is not None else np.random.default_rng(cfg.sim.seed)
        self._kd = cfg.k_drag / cfg.ball.mass          # 每單位質量的阻力係數

    def _acc(self, vx: float, vy: float, vz: float) -> tuple[float, float, float]:
        s = sqrt(vx * vx + vy * vy + vz * vz)
        g = self.cfg.air.g
        return (-self._kd * s * vx, -g - self._kd * s * vy, -self._kd * s * vz)

    def shoot(self, pitch: float, yaw: float,
              v0: float | None = None) -> tuple[float, float] | None:
        """理想（無機構誤差）投射：RK4 積分到 x = dist_x 平面。

        pitch/yaw 為弧度；v0 省略時用實際出射速度 v0_real。
        射程不足或落地回傳 None。
        """
        cfg = self.cfg
        dt, dist_x = cfg.sim.dt, cfg.field.dist_x
        v0 = cfg.launch.v0_real if v0 is None else v0

        x, y, z = 0.0, cfg.field.launch_h, 0.0
        vx = v0 * cos(pitch) * cos(yaw)
        vy = v0 * sin(pitch)
        vz = v0 * cos(pitch) * sin(yaw)
        xp = yp = zp = 0.0

        for _ in range(int(2.0 / dt)):
            if x >= dist_x:
                break
            xp, yp, zp = x, y, z
            a1 = self._acc(vx, vy, vz)
            a2 = self._acc(vx + 0.5 * dt * a1[0], vy + 0.5 * dt * a1[1], vz + 0.5 * dt * a1[2])
            a3 = self._acc(vx + 0.5 * dt * a2[0], vy + 0.5 * dt * a2[1], vz + 0.5 * dt * a2[2])
            a4 = self._acc(vx + dt * a3[0], vy + dt * a3[1], vz + dt * a3[2])
            x += dt * vx + dt * dt / 6 * (a1[0] + a2[0] + a3[0])
            y += dt * vy + dt * dt / 6 * (a1[1] + a2[1] + a3[1])
            z += dt * vz + dt * dt / 6 * (a1[2] + a2[2] + a3[2])
            vx += dt / 6 * (a1[0] + 2 * a2[0] + 2 * a3[0] + a4[0])
            vy += dt / 6 * (a1[1] + 2 * a2[1] + 2 * a3[1] + a4[1])
            vz += dt / 6 * (a1[2] + 2 * a2[2] + 2 * a3[2] + a4[2])
            if y < -0.5:
                return None
        if x < dist_x:
            return None
        f = (dist_x - xp) / (x - xp)                    # 線性內插到 x = dist_x
        return zp + f * (z - zp), yp + f * (y - yp)

    def shoot_real(self, pitch: float, yaw: float) -> tuple[float, float] | None:
        """含機構系統偏差、逐發抖動、初速變異的一發（模擬真實不完美投射）。"""
        m = self.cfg.mechanism
        p_real = pitch + np.radians(m.bias_pitch + self.rng.normal(0, m.jitter_deg))
        y_real = yaw + np.radians(m.bias_yaw + self.rng.normal(0, m.jitter_deg))
        v_real = self.cfg.launch.v0_real + self.rng.normal(0, self.cfg.launch.v0_jitter)
        return self.shoot(p_real, y_real, v_real)
