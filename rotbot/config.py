# -*- coding: utf-8 -*-
"""集中式設定 —— 從 configs/field.yaml 載入成型別化的 dataclass。

所有模組都透過這裡取得參數，硬體到位後只需改 YAML，不動程式碼。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

# 預設設定檔位置：專案根目錄的 configs/field.yaml
DEFAULT_CONFIG = Path(__file__).resolve().parent.parent / "configs" / "field.yaml"


@dataclass(frozen=True)
class FieldConfig:
    dist_x: float
    launch_h: float
    grid_cx: float
    grid_cy: float
    col_pitch: float
    row_pitch: float
    frame_d: float           # 一般圓孔直徑 (m)
    bonus_d: float           # 加分圓孔直徑 (m)
    bonus_col_pitch: float   # 上排加分孔橫向間距 (m)
    bonus_dy: float          # 上排加分孔高於主格最上排的高度差 (m)


@dataclass(frozen=True)
class BallConfig:
    diameter: float
    mass: float

    @property
    def area(self) -> float:
        """迎風截面積 (m^2)。"""
        return math.pi * (self.diameter / 2) ** 2


@dataclass(frozen=True)
class LaunchConfig:
    v0_model: float
    v0_real: float
    v0_jitter: float


@dataclass(frozen=True)
class AirConfig:
    rho: float
    cd: float
    g: float


@dataclass(frozen=True)
class AimConfig:
    """腦內瞄準模型。drag=含平方阻力數值解；vacuum=無阻力解析解（舊版）。"""
    model: str = "vacuum"
    cd_model: float = 0.50   # 腦內模型認定的阻力係數（實機以試投擬合）
    dt: float = 0.002        # 數值積分步長 (s)


@dataclass(frozen=True)
class MechanismConfig:
    bias_pitch: float
    bias_yaw: float
    jitter_deg: float


@dataclass(frozen=True)
class CalibrationConfig:
    alpha_global: float
    alpha_cell: float
    meas_noise_m: float
    outlier_m: float
    clamp_deg: float


@dataclass(frozen=True)
class CameraConfig:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    def matrix(self) -> np.ndarray:
        """相機內參矩陣 K (3x3)。"""
        return np.array([[self.fx, 0.0, self.cx],
                         [0.0, self.fy, self.cy],
                         [0.0, 0.0, 1.0]], dtype=np.float64)


@dataclass(frozen=True)
class CameraMount:
    """相機相對發射點的外參（camera→launcher）。全 0 = 相機在發射口且對齊。"""
    dx: float       # 右 (m)
    dy: float       # 上 (m)
    dz: float       # 前 (m)
    roll: float     # deg
    pitch: float    # deg
    yaw: float      # deg


@dataclass(frozen=True)
class SimConfig:
    dt: float
    seed: int


@dataclass(frozen=True)
class Config:
    field: FieldConfig
    ball: BallConfig
    launch: LaunchConfig
    air: AirConfig
    mechanism: MechanismConfig
    calibration: CalibrationConfig
    camera: CameraConfig
    camera_mount: CameraMount
    sim: SimConfig
    n_balls: int
    aim: AimConfig = AimConfig()

    @property
    def k_drag(self) -> float:
        """平方阻力係數：F_drag = k_drag * |v| * v。"""
        return 0.5 * self.air.rho * self.air.cd * self.ball.area

    @property
    def k_drag_model(self) -> float:
        """腦內模型認定的平方阻力係數（用 aim.cd_model）。"""
        return 0.5 * self.air.rho * self.aim.cd_model * self.ball.area

    @property
    def tol_std(self) -> float:
        """一般框容許球心誤差 (m)。"""
        return (self.field.frame_d - self.ball.diameter) / 2

    @property
    def tol_bonus(self) -> float:
        """加分格容許球心誤差 (m)。"""
        return (self.field.bonus_d - self.ball.diameter) / 2

    def hole_centers(self) -> list[tuple[str, int, float, np.ndarray]]:
        """全部圓孔的 (群組, 索引, 直徑, 板座標系3D中心)。

        佈局：主九宮格 3×3（上排左→下排右）+ 上排 3 個加分孔（左→右）。
        板座標系原點在主九宮格中心，+x 右、+y 上、+z 出板面。
        scene 與 pose 共用這份佈局，確保幾何一致。
        """
        f = self.field
        holes: list[tuple[str, int, float, np.ndarray]] = []
        for i, (c, r) in enumerate((c, r) for r in (1, 0, -1) for c in (-1, 0, 1)):
            holes.append(("main", i, f.frame_d,
                          np.array([c * f.col_pitch, r * f.row_pitch, 0.0])))
        top_y = f.row_pitch + f.bonus_dy
        for i, c in enumerate((-1, 0, 1)):
            holes.append(("bonus", i, f.bonus_d,
                          np.array([c * f.bonus_col_pitch, top_y, 0.0])))
        return holes


def load_config(path: str | Path | None = None) -> Config:
    """從 YAML 載入設定；path 省略時使用 configs/field.yaml。"""
    path = Path(path) if path else DEFAULT_CONFIG
    with open(path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    return Config(
        field=FieldConfig(**raw["field"]),
        ball=BallConfig(**raw["ball"]),
        launch=LaunchConfig(**raw["launch"]),
        air=AirConfig(**raw["air"]),
        mechanism=MechanismConfig(**raw["mechanism"]),
        calibration=CalibrationConfig(**raw["calibration"]),
        camera=CameraConfig(**raw["camera"]),
        camera_mount=CameraMount(**raw["camera_mount"]),
        sim=SimConfig(**raw["sim"]),
        n_balls=int(raw["match"]["n_balls"]),
        aim=AimConfig(**raw.get("aim", {})),
    )
