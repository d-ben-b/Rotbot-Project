# -*- coding: utf-8 -*-
"""完美偵測器（模擬替身）—— 由合成場景真值直接產生 Detection。

用途：
  * 在 YOLO 模型訓練好之前，先驗證「感知→決策→致動」下游閉迴路（P4）。
  * 全鏈測試：以已知真值當偵測輸入，隔離下游邏輯的正確性。
可選加入像素雜訊，模擬不完美偵測。
"""
from __future__ import annotations

import numpy as np

from ..perception.detector import (CLASS_BONUS, CLASS_BONUS_LIT, CLASS_MAIN,
                                    Detection, assign_ids, detection_from_bbox)
from .scene import SceneSample


def detections_from_scene(sample: SceneSample, noise_px: float = 0.0,
                          rng: np.random.Generator | None = None) -> list[Detection]:
    """把場景中可見的圓孔轉成 Detection（bbox 由投影 rim 導出）。"""
    dets = []
    for h in sample.holes:
        if not h.visible:
            continue
        pts = h.rim_px
        if noise_px and rng is not None:
            pts = pts + rng.normal(0, noise_px, pts.shape)
        x1, y1 = pts.min(axis=0)
        x2, y2 = pts.max(axis=0)
        cls = CLASS_BONUS_LIT if h.is_lit else (CLASS_BONUS if h.is_bonus else CLASS_MAIN)
        dets.append(detection_from_bbox((x1, y1, x2, y2), cls))
    assign_ids(dets)
    return dets
