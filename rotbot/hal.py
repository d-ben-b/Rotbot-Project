# -*- coding: utf-8 -*-
"""硬體抽象層 (HAL) —— 定義「相機」與「致動器」的介面。

核心設計：主邏輯只依賴這些抽象介面，不依賴具體實作。
  * 沒有硬體時：sim.camera.SimCamera / actuation.link.MockArduino
  * 硬體到位後：perception.camera.RealCamera / actuation.link.SerialArduino
換底層實作時，pipeline 一行都不用改。
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np


# ══════════════════════════ 相機 ══════════════════════════
class Camera(ABC):
    """影像來源介面。"""

    @abstractmethod
    def read(self) -> np.ndarray:
        """取得一張影像 (BGR, HxWx3, uint8)。"""

    @abstractmethod
    def intrinsics(self) -> np.ndarray:
        """相機內參矩陣 K (3x3)。"""

    def release(self) -> None:
        """釋放資源（預設不做事）。"""


# ══════════════════════════ 致動器 ══════════════════════════
@dataclass(frozen=True)
class AimCommand:
    """一發投球指令。"""
    pitch_deg: float          # 俯仰角
    yaw_deg: float            # 水平角
    flywheel: float           # 飛輪轉速指令（單位由機構定義，如 rpm 或 0~1）
    trigger: bool = False     # 是否觸發送球


@dataclass(frozen=True)
class Ack:
    """致動器回應。"""
    ok: bool
    message: str = ""


class ActuatorLink(ABC):
    """與下位機（Arduino）的通訊介面。"""

    @abstractmethod
    def send(self, cmd: AimCommand) -> Ack:
        """送出一發投球指令，回傳下位機的 ack。"""

    @abstractmethod
    def is_connected(self) -> bool:
        """連線是否正常。"""

    def close(self) -> None:
        """關閉連線（預設不做事）。"""
