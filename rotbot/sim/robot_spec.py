# -*- coding: utf-8 -*-
"""機器人規格 —— 從 configs/robot.yaml 載入成型別化 dataclass。

比照 config.py 的做法：所有幾何集中在 YAML，硬體到位後只改 YAML、程式不動。
座標約定與 field.yaml / 發射點系一致：x 前(朝網板)、y 上、z 右。
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

DEFAULT_ROBOT_CONFIG = Path(__file__).resolve().parent.parent.parent / "configs" / "robot.yaml"


@dataclass(frozen=True)
class ChassisSpec:
    length: float
    width: float
    height: float
    mass: float
    wheel_diameter: float


@dataclass(frozen=True)
class LauncherSpec:
    type: str
    flywheel_diameter: float
    flywheel_rpm: float
    muzzle_height: float
    pivot_forward: float


@dataclass(frozen=True)
class TurretSpec:
    yaw_min_deg: float
    yaw_max_deg: float
    pitch_min_deg: float
    pitch_max_deg: float


@dataclass(frozen=True)
class MagazineSpec:
    balls_per_chamber: int
    chambers: int
    drum_diameter: float
    drum_length: float


@dataclass(frozen=True)
class RobotCameraSpec:
    coaxial: bool
    forward_offset: float


@dataclass(frozen=True)
class RobotSpec:
    chassis: ChassisSpec
    launcher: LauncherSpec
    turret: TurretSpec
    magazine: MagazineSpec
    camera: RobotCameraSpec


def load_robot_spec(path: str | Path | None = None) -> RobotSpec:
    """從 YAML 載入機器人規格；path 省略時使用 configs/robot.yaml。"""
    path = Path(path) if path else DEFAULT_ROBOT_CONFIG
    with open(path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    return RobotSpec(
        chassis=ChassisSpec(**raw["chassis"]),
        launcher=LauncherSpec(**raw["launcher"]),
        turret=TurretSpec(**raw["turret"]),
        magazine=MagazineSpec(**raw["magazine"]),
        camera=RobotCameraSpec(**raw["camera"]),
    )
