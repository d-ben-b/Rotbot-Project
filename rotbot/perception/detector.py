# -*- coding: utf-8 -*-
"""圓孔偵測 —— YOLO 包裝。

推論流程：影像 → YOLO → 每個圓孔(中心 + bbox + 類別)
         → 依空間位置指派孔編號 → Detection 清單 → 交給 perception.pose。

主力方法為「中心整板 PnP」（方法 A，見 pose.py）：偵測給中心，pose 用已知
孔中心過定求解。需要單孔傾角時，另用 YOLO-seg/橢圓擬合走方法 B。

類別（與 tools/gen_synthetic_data.py 一致）：
  0 = main（一般孔 Ø40）、1 = bonus（加分孔 Ø20 未亮）、2 = bonus_lit（亮起指定得分格）
Orin 上以 TensorRT 加速留待 P5。
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import numpy as np

CLASS_MAIN = 0
CLASS_BONUS = 1
CLASS_BONUS_LIT = 2


@dataclass
class Detection:
    """一個偵測到的圓孔。"""
    group: str                   # "main" | "bonus"
    idx: int                     # 群組內編號（main 0~8 上左→下右；bonus 0~2 左→右）
    center: np.ndarray           # (2,) 中心像素座標
    bbox: tuple[float, float, float, float]   # (x1, y1, x2, y2)
    diameter_px: float           # 表觀直徑（供尺寸測距備援）
    is_bonus: bool = False
    is_lit: bool = False         # 是否為亮起的指定得分格
    confidence: float = 1.0
    extra: dict = field(default_factory=dict)


class Detector(ABC):
    """影像 → 偵測結果 介面。"""

    @abstractmethod
    def detect(self, image: np.ndarray) -> list[Detection]:
        """偵測影像中的圓孔，回傳 Detection 清單。"""


def _assign_main_grid(mains: list[Detection]) -> None:
    """把 main 類偵測依空間位置就地指派 0~8（上排左→下排右）。"""
    if not mains:
        return
    centers = np.array([d.center for d in mains])
    heights = np.array([d.bbox[3] - d.bbox[1] for d in mains])
    row_gap = 0.6 * float(np.median(heights))
    order = np.argsort(centers[:, 1])
    rows: list[list[int]] = [[order[0]]]
    for idx in order[1:]:
        if centers[idx, 1] - centers[rows[-1][-1], 1] > row_gap:
            rows.append([idx])
        else:
            rows[-1].append(idx)
    cid = 0
    for row in rows:
        for idx in sorted(row, key=lambda i: centers[i, 0]):
            mains[idx].idx = cid
            cid += 1


def assign_ids(dets: list[Detection]) -> None:
    """就地指派孔編號：main 依九宮格 0~8，bonus 依 x 由左至右 0~2。"""
    _assign_main_grid([d for d in dets if not d.is_bonus])
    bonus = sorted((d for d in dets if d.is_bonus), key=lambda d: d.center[0])
    for j, d in enumerate(bonus):
        d.idx = j


class YoloDetector(Detector):
    """Ultralytics YOLO 圓孔偵測包裝（bbox 偵測）。"""

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
        if res.boxes is None:
            return dets
        xyxy = res.boxes.xyxy.cpu().numpy()
        clss = res.boxes.cls.cpu().numpy().astype(int)
        confs = res.boxes.conf.cpu().numpy()
        for (x1, y1, x2, y2), c, cf in zip(xyxy, clss, confs):
            dets.append(detection_from_bbox((x1, y1, x2, y2), c, float(cf)))
        assign_ids(dets)
        return dets


def detection_from_bbox(bbox, cls: int, conf: float = 1.0) -> Detection:
    """由 bbox + 類別建立 Detection（中心/表觀直徑由 bbox 導出）。"""
    x1, y1, x2, y2 = bbox
    center = np.array([(x1 + x2) / 2, (y1 + y2) / 2], dtype=np.float64)
    diameter = float((abs(x2 - x1) + abs(y2 - y1)) / 2)
    is_bonus = cls in (CLASS_BONUS, CLASS_BONUS_LIT)
    return Detection(group="bonus" if is_bonus else "main", idx=-1,
                     center=center, bbox=(float(x1), float(y1), float(x2), float(y2)),
                     diameter_px=diameter, is_bonus=is_bonus,
                     is_lit=(cls == CLASS_BONUS_LIT), confidence=conf)
