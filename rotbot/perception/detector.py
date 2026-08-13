# -*- coding: utf-8 -*-
"""網框偵測 —— YOLO-pose 包裝。

推論流程：影像 → YOLO-pose → 每實例(4 角點 + 類別 + 信心)
         → 依空間位置指派九宮格編號 → Detection 清單 → 交給 perception.pose。

角點取得方式已定案為 YOLO-pose 關鍵點（見 ROADMAP §4）。
Orin 上以 TensorRT 加速留待 P5。
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import numpy as np

# 類別索引（與 tools/gen_synthetic_data.py 一致）
CLASS_CELL = 0
CLASS_BONUS = 1


@dataclass
class Detection:
    """一個偵測到的網框。"""
    cell_id: int                 # 九宮格中的格編號 0~8（依空間位置指派）
    corners: np.ndarray          # (4, 2) 四角像素座標（左上→右上→右下→左下）
    is_bonus: bool = False       # 是否為亮起的加分格
    confidence: float = 1.0
    extra: dict = field(default_factory=dict)


class Detector(ABC):
    """影像 → 偵測結果 介面。"""

    @abstractmethod
    def detect(self, image: np.ndarray) -> list[Detection]:
        """偵測影像中的網框，回傳 Detection 清單。"""


def assign_grid_ids(dets: list[Detection]) -> None:
    """依偵測框的空間位置，就地指派九宮格編號 0~8（上排左→下排右）。

    先用 y 座標分列（以中位框高為間隔門檻），列內再依 x 排序。
    假設每列約 3 格；缺格時仍儘量維持 row-major 順序（穩健化留待後續）。
    """
    if not dets:
        return
    centers = np.array([d.corners.mean(axis=0) for d in dets])
    heights = np.array([np.ptp(d.corners[:, 1]) for d in dets])
    row_gap = 0.6 * float(np.median(heights))

    order = np.argsort(centers[:, 1])            # 依 y 由上到下
    rows: list[list[int]] = [[order[0]]]
    for idx in order[1:]:
        if centers[idx, 1] - centers[rows[-1][-1], 1] > row_gap:
            rows.append([idx])
        else:
            rows[-1].append(idx)

    cid = 0
    for row in rows:
        for idx in sorted(row, key=lambda i: centers[i, 0]):   # 列內依 x
            dets[idx].cell_id = cid
            cid += 1


class YoloPoseDetector(Detector):
    """Ultralytics YOLO-pose 推論包裝。"""

    def __init__(self, weights: str, conf: float = 0.25, device: str = "0",
                 imgsz: int = 960):
        from ultralytics import YOLO   # 延後 import：無 GPU/未安裝環境不受影響
        self.model = YOLO(weights)
        self.conf = conf
        self.device = device
        self.imgsz = imgsz

    def detect(self, image: np.ndarray) -> list[Detection]:
        res = self.model.predict(image, conf=self.conf, device=self.device,
                                 imgsz=self.imgsz, verbose=False)[0]
        dets: list[Detection] = []
        if res.keypoints is None or res.boxes is None:
            return dets
        kpts = res.keypoints.xy.cpu().numpy()        # (N, 4, 2) 像素座標
        clss = res.boxes.cls.cpu().numpy().astype(int)
        confs = res.boxes.conf.cpu().numpy()
        for k, c, cf in zip(kpts, clss, confs):
            if k.shape[0] != 4:
                continue
            dets.append(Detection(cell_id=-1, corners=k.astype(np.float64),
                                  is_bonus=(c == CLASS_BONUS), confidence=float(cf)))
        assign_grid_ids(dets)
        return dets
