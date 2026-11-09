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

import cv2
import numpy as np

from ..config import LitCheckConfig

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


def _clusters(vals: np.ndarray, gap: float) -> list[list[int]]:
    """一維分群：排序後相鄰差 > gap 就分開。回傳由小到大的群（索引）。"""
    order = np.argsort(vals)
    groups: list[list[int]] = [[int(order[0])]]
    for idx in order[1:]:
        if vals[idx] - vals[groups[-1][-1]] > gap:
            groups.append([int(idx)])
        else:
            groups[-1].append(int(idx))
    return groups


def _assign_main_grid(mains: list[Detection]) -> None:
    """把 main 類偵測依空間位置就地指派 0~8（上排左→下排右）。

    先依 y 分列、依 x 分欄；分得出 3 列 × 3 欄時編號 = 列 × 3 + 欄，少一個孔或多一個框
    （半個孔的框落在同一格）都不影響其他孔。分不出來時（例如整排在畫面外）退回逐排由左到右。
    """
    if not mains:
        return
    centers = np.array([d.center for d in mains])
    heights = np.array([d.bbox[3] - d.bbox[1] for d in mains])
    gap = 0.6 * float(np.median(heights))          # 列距 1.5、欄距 2.25 個孔徑，同孔的框差 < 0.5 個
    rows = _clusters(centers[:, 1], gap)
    cols = _clusters(centers[:, 0], gap)
    if len(rows) == 3 and len(cols) == 3:
        col_of = {i: c for c, g in enumerate(cols) for i in g}
        for r, row in enumerate(rows):
            for i in row:
                mains[i].idx = 3 * r + col_of[i]
        return
    cid = 0
    for row in rows:
        for idx in sorted(row, key=lambda i: centers[i, 0]):
            mains[idx].idx = cid
            cid += 1


def _area(d: Detection) -> float:
    x1, y1, x2, y2 = d.bbox
    return max(0.0, x2 - x1) * max(0.0, y2 - y1)


def drop_non_square(dets: list[Detection], image_shape: tuple[int, ...],
                    min_ratio: float = 0.75, margin: float = 4.0) -> list[Detection]:
    """刪掉寬高比不在 [min_ratio, 1/min_ratio] 的框，排編號前用；碰到畫面邊緣的框不檢查。

    孔是圓的，完整孔的框接近正方形（PyBullet 掃視畫面 0.98–1.09；斜看 40° 時約 0.77）。
    YOLO 把一個孔切成兩半時，每半個框約 0.4–0.7，中心偏掉約 1/4 個孔徑。
    被畫面邊緣切掉的孔框本來就不是正方形，仍保留（合成評估中非正方形的框全是這種）。
    """
    H, W = image_shape[:2]
    out = []
    for d in dets:
        x1, y1, x2, y2 = d.bbox
        at_edge = x1 <= margin or y1 <= margin or x2 >= W - margin or y2 >= H - margin
        w, h = x2 - x1, y2 - y1
        if at_edge or (w > 0 and h > 0 and min_ratio <= w / h <= 1.0 / min_ratio):
            out.append(d)
    return out


def drop_contained(dets: list[Detection], frac: float = 0.6,
                   same_size: float = 0.8) -> list[Detection]:
    """刪掉被另一個框包住的框（交集占自己面積 ≥ frac），排編號前用。

    網框上的孔彼此不重疊，正常的孔框不會被包住。YOLO 有時把一個孔切成兩半各框一次
    （PyBullet 畫面中孔後網子的格線），多出來的框會讓 assign_ids 整排編號錯開。
    兩框面積比 ≥ same_size 時視為同一個孔的重複框（例如加分孔與亮孔各一個），留信心值高的；
    否則留大的。回傳保留原順序。
    """
    kept: list[Detection] = []
    for d in sorted(dets, key=_area, reverse=True):
        a = _area(d)
        for k, o in enumerate(kept):
            ix = min(d.bbox[2], o.bbox[2]) - max(d.bbox[0], o.bbox[0])
            iy = min(d.bbox[3], o.bbox[3]) - max(d.bbox[1], o.bbox[1])
            if a > 0 and ix > 0 and iy > 0 and ix * iy / a >= frac:
                if a >= same_size * _area(o) and d.confidence > o.confidence:
                    kept[k] = d
                break
        else:
            kept.append(d)
    ids = {id(d) for d in kept}
    return [d for d in dets if id(d) in ids]


def red_fraction(image: np.ndarray, bbox) -> float:
    """框的內切橢圓中，紅色像素（HSV 色相 < 10 或 > 170、飽和度 > 80、亮度 > 40）的比例。

    只看內切橢圓：框的四角是孔外的木板，不是 LED。過曝成白色的燈條芯不算紅，但它周圍的紅色光暈算。
    """
    H, W = image.shape[:2]
    x1, y1 = max(0, int(round(bbox[0]))), max(0, int(round(bbox[1])))
    x2, y2 = min(W, int(round(bbox[2]))), min(H, int(round(bbox[3])))
    if x2 - x1 < 2 or y2 - y1 < 2:
        return 0.0
    h, s, v = cv2.split(cv2.cvtColor(image[y1:y2, x1:x2], cv2.COLOR_BGR2HSV))
    red = ((h < 10) | (h > 170)) & (s > 80) & (v > 40)
    mask = np.zeros(red.shape, np.uint8)
    cv2.ellipse(mask, (((x2 - x1 - 1) / 2, (y2 - y1 - 1) / 2), (x2 - x1, y2 - y1), 0), 1, -1)
    inside = mask > 0
    return float(red[inside].mean()) if inside.any() else 0.0


def mark_lit_by_color(image: np.ndarray, dets: list[Detection],
                      min_frac: float = 0.02, min_ratio: float = 3.0) -> Detection | None:
    """YOLO 沒有回報亮孔時的備援：紅色比例最高、而且明顯高於其他加分孔的框，標成亮孔。

    YOLO 在 LED 只露出上半圈、過曝成白色、或只有孔內紅光時，會把亮孔判成未亮的 bonus（框的位置是對的）。
    前 60 秒三個加分孔剛好亮一個，所以只比相對大小：最高者 ≥ min_frac，且 ≥ 第二高 × min_ratio。
    少於兩個加分孔框時無從比較，不標。每個加分孔框的比例記在 extra["red_frac"]（實拍時用來定門檻）。
    回傳被標成亮孔的偵測，沒有則 None。
    """
    bonus = [d for d in dets if d.is_bonus]
    for d in bonus:
        d.extra["red_frac"] = red_fraction(image, d.bbox)
    if len(bonus) < 2 or any(d.is_lit for d in bonus):
        return None
    ranked = sorted(bonus, key=lambda d: d.extra["red_frac"], reverse=True)
    top, second = ranked[0].extra["red_frac"], ranked[1].extra["red_frac"]
    if top < min_frac or top < min_ratio * second:
        return None
    ranked[0].is_lit = True
    ranked[0].extra["lit_by"] = "color"
    return ranked[0]


def assign_ids(dets: list[Detection]) -> None:
    """就地指派孔編號：main 依九宮格 0~8，bonus 依 x 由左至右 0~2。"""
    _assign_main_grid([d for d in dets if not d.is_bonus])
    bonus = sorted((d for d in dets if d.is_bonus), key=lambda d: d.center[0])
    for j, d in enumerate(bonus):
        d.idx = j


class YoloDetector(Detector):
    """Ultralytics YOLO 圓孔偵測包裝（bbox 偵測）。"""

    def __init__(self, weights: str, conf: float = 0.25, device: str = "0",
                 imgsz: int = 960, lit_check: LitCheckConfig = LitCheckConfig()):
        from ultralytics import YOLO   # 延後 import：無 GPU/未安裝環境不受影響
        self.model = YOLO(weights)
        self.conf = conf
        self.device = device
        self.imgsz = imgsz
        self.lit_check = lit_check     # 亮孔的顏色備援（mark_lit_by_color）；enabled=False 關掉

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
        dets = drop_contained(drop_non_square(dets, image.shape))
        assign_ids(dets)
        if self.lit_check.enabled:
            mark_lit_by_color(image, dets, self.lit_check.min_frac, self.lit_check.min_ratio)
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
