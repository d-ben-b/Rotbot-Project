# -*- coding: utf-8 -*-
"""主迴圈 —— 串起 感知 → 決策 → 致動（P4 端到端整合）。

P4 待辦（見 ROADMAP §P4）：
  camera.read → detector.detect → pose.solve_frame_pose
    → strategy.select_next_target → ballistics.aim_for_target (+校正)
    → protocol/link.send → 觀測落點 → calibration.update
本檔目前只留骨架簽章，讓各模組有明確的組裝點。
"""
from __future__ import annotations

from ..config import Config
from ..hal import ActuatorLink, Camera
from ..perception.detector import Detector


class ControlLoop:
    """感知→決策→致動 的主迴圈（P4 實作 step()）。"""

    def __init__(self, cfg: Config, camera: Camera, detector: Detector,
                 actuator: ActuatorLink):
        self.cfg = cfg
        self.camera = camera
        self.detector = detector
        self.actuator = actuator

    def step(self) -> None:
        """跑一次「看→算→投」。P4 實作。"""
        raise NotImplementedError("ControlLoop.step 於 P4 實作（見 ROADMAP）")
