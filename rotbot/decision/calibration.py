# -*- coding: utf-8 -*-
"""線上自適應校正 —— 「找手感」。

每投一顆，依相機量測到的落點誤差微調瞄準：
  * 全域偏移 (g_bias)：修正所有格共通的系統性偏差（安裝偏差、空氣阻力）
  * 單格殘差 (cell_off)：修正各格獨有的殘差
含異常值剔除與修正量上限 (clamp)，避免單次壞量測把校正帶歪。

原型：presentation/sim_adaptive_calibration.py 的 run_match() 回授段。
"""
from __future__ import annotations

import numpy as np

from ..config import CalibrationConfig


class AdaptiveCalibrator:
    """線上校正器；維護全域偏移與每格殘差，回傳角度修正量。"""

    def __init__(self, cfg: CalibrationConfig, j_pitch: float, j_yaw: float,
                 n_cells: int = 9):
        self.cfg = cfg
        self.j_pitch = j_pitch                 # 靈敏度 ∂高度/∂俯仰 (m/rad)
        self.j_yaw = j_yaw                     # 靈敏度 ∂橫向/∂水平 (m/rad)
        self.g_bias = np.zeros(2)              # [pitch, yaw] 全域偏移 (rad)
        self.cell_off = np.zeros((n_cells, 2)) # 每格殘差 (rad)

    def correction(self, cell_idx: int) -> tuple[float, float]:
        """回傳某格目前的角度修正量 (dpitch_rad, dyaw_rad)。"""
        dp = self.g_bias[0] + self.cell_off[cell_idx, 0]
        dy = self.g_bias[1] + self.cell_off[cell_idx, 1]
        return float(dp), float(dy)

    def update(self, cell_idx: int, err_zy: tuple[float, float]) -> bool:
        """依落點誤差更新校正。

        err_zy: (橫向誤差 e_z, 高度誤差 e_y) = 目標 - 落點 (m)，含量測雜訊。
        回傳 True 表示有更新；被判為異常值則回傳 False。
        """
        e_z, e_y = err_zy
        if np.hypot(e_z, e_y) > self.cfg.outlier_m:     # 異常值剔除
            return False

        d_pitch = e_y / self.j_pitch                    # 位置誤差 → 角度修正
        d_yaw = e_z / self.j_yaw

        self.g_bias += np.array([self.cfg.alpha_global * d_pitch,
                                 self.cfg.alpha_global * d_yaw])
        self.cell_off[cell_idx] += np.array([self.cfg.alpha_cell * d_pitch,
                                             self.cfg.alpha_cell * d_yaw])
        lim = np.radians(self.cfg.clamp_deg)
        self.g_bias = np.clip(self.g_bias, -lim, lim)
        self.cell_off = np.clip(self.cell_off, -lim, lim)
        return True
