# -*- coding: utf-8 -*-
"""網框偵測 —— YOLO 包裝（P1 實作）。

P1 待辦（見 ROADMAP §P1）：
  * 決定角點取得方式：YOLO-pose 關鍵點 / OBB / bbox+古典角點精修
  * Ultralytics 推論 → 後處理取四角 → 交給 perception.pose
  * Orin 上以 TensorRT 加速（P5）
本檔目前只定義介面與資料結構，讓 pipeline 能先接上模擬替身。
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import numpy as np


@dataclass
class Detection:
    """一個偵測到的網框。"""
    cell_id: int                 # 九宮格中的格編號 0~8
    corners: np.ndarray          # (4, 2) 四角像素座標（左上→右上→右下→左下）
    is_bonus: bool = False       # 是否為亮起的加分格
    confidence: float = 1.0
    extra: dict = field(default_factory=dict)


class Detector(ABC):
    """影像 → 偵測結果 介面。"""

    @abstractmethod
    def detect(self, image: np.ndarray) -> list[Detection]:
        """偵測影像中的網框，回傳 Detection 清單。"""
