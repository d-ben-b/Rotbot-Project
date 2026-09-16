# -*- coding: utf-8 -*-
"""感知定位 —— 把偵測結果轉成「每孔瞄準幾何」。

串接：Detection 清單 → 穩健建立「偵測↔已知孔中心」對應 → 整板 PnP（方法 A）
     → 每孔 (距離/方位/俯仰)。孔數不足以整板求解時，逐孔用表觀直徑測距備援。

對應為何要穩健：真實偵測會有多餘框/邊緣孔/偶發誤分類，純 2D 行列分群易指派錯
編號。這裡改用「RANSAC 初估位姿 → 投影已知孔中心 → 就近重匹配」修正對應。

這是 P4 主迴圈的「感知半段」：detector 之後、decision 之前。
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from ..config import Config
from .detector import Detection
from .pose import BoardPose, distance_from_diameter, solve_board_pose


@dataclass
class HoleAim:
    group: str
    idx: int
    is_bonus: bool
    is_lit: bool
    distance: float          # m
    bearing: float           # deg
    elev: float              # deg
    source: str              # "board_pnp" | "size"


@dataclass
class SceneEstimate:
    board_pose: BoardPose | None
    aims: list[HoleAim]

    @property
    def lit_bonus(self) -> HoleAim | None:
        return next((a for a in self.aims if a.is_lit), None)


def _diameter_real(det: Detection, cfg: Config) -> float:
    return cfg.field.bonus_d if det.is_bonus else cfg.field.frame_d


def _rematch_by_projection(dets, obj_ids, obj_pts, rvec, tvec, K, dist):
    """把已知孔中心投影回影像，將每個偵測就近匹配到正確孔 id（去重）。

    回傳 {det_index: matched_obj_id}；未匹配的偵測不列入。
    """
    proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, K, dist)
    proj = proj.reshape(-1, 2)                       # (M, 2) 已知孔投影
    # 門檻：投影孔間最近鄰距離的一半（容忍雜訊、拒絕張冠李戴）
    dd = np.linalg.norm(proj[:, None] - proj[None], axis=2)
    np.fill_diagonal(dd, np.inf)
    thresh = 0.5 * float(np.median(dd.min(axis=1)))

    best: dict[int, tuple[int, float]] = {}          # obj_id -> (det_index, dist)
    for di, d in dets:
        dist_to = np.linalg.norm(proj - d.center, axis=1)
        j = int(np.argmin(dist_to))
        if dist_to[j] <= thresh and (j not in best or dist_to[j] < best[j][1]):
            best[j] = (di, float(dist_to[j]))
    return {di: obj_ids[j] for j, (di, _) in best.items()}


def _solve_group(dets_idx, known, group, K, dist):
    """對某群組（main/bonus）穩健求解對應：RANSAC 初估 → 投影重匹配。

    dets_idx: [(det_index, Detection)]；known: {(group,idx): center_board}。
    回傳 (rvec, tvec, {det_index: matched_idx}) 或 None。
    """
    ids = [i for (g, i) in known if g == group]
    obj_pts = np.array([known[(group, i)] for i in ids], np.float64)

    # 初始對應：用偵測目前的 idx（少數指派錯也沒關係，靠下面投影重匹配修正）
    obj0, img0 = [], []
    for di, d in dets_idx:
        if (group, d.idx) in known:
            obj0.append(known[(group, d.idx)])
            img0.append(d.center)
    if len(obj0) < 4:
        return None
    # SQPNP：共面且正對都穩定（solvePnPRansac 共面不穩、IPPE 正對退化，故都不用）
    ok, rvec, tvec = cv2.solvePnP(np.array(obj0, np.float64),
                                  np.array(img0, np.float64), K, dist,
                                  flags=cv2.SOLVEPNP_SQPNP)
    if not ok:
        ok, rvec, tvec = cv2.solvePnP(np.array(obj0, np.float64),
                                      np.array(img0, np.float64), K, dist,
                                      flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    match = _rematch_by_projection(dets_idx, ids, obj_pts, rvec, tvec, K, dist)
    return rvec, tvec, match


def localize_holes(detections: list[Detection], cfg: Config, K: np.ndarray,
                   dist_coeffs: np.ndarray | None = None) -> SceneEstimate:
    """由偵測結果算出整板位姿與每孔瞄準幾何（穩健對應）。"""
    known = {(g, i): C for (g, i, _d, C) in cfg.hole_centers()}
    dist = np.zeros((5, 1)) if dist_coeffs is None else np.asarray(dist_coeffs, np.float64)
    Kinv = np.linalg.inv(K)

    mains = [(i, d) for i, d in enumerate(detections) if not d.is_bonus]
    solved = _solve_group(mains, known, "main", K, dist)

    matched: dict[int, int] = {}           # det_index -> corrected main idx
    bp = None
    if solved is not None:
        _rv, _tv, match = solved
        matched = match
        # 用修正後對應重解整板位姿（IPPE，最穩）
        obj = np.array([known[("main", j)] for j in match.values()], np.float64)
        img = np.array([detections[di].center for di in match], np.float64)
        bp = solve_board_pose(obj, img, K, dist) if len(obj) >= 4 else None

    aims: list[HoleAim] = []
    for i, det in enumerate(detections):
        idx = matched.get(i, det.idx)
        key = (det.group, idx)
        if bp is not None and not det.is_bonus and key in known:
            distance, bearing, elev = bp.point_aim(known[key])
            source = "board_pnp"
        elif bp is not None and det.is_bonus:
            # 加分孔：用主板位姿 + 投影就近找到它對應的 bonus 中心
            idx_b, key_b = _nearest_group_id(det.center, known, "bonus", bp, K, dist)
            if key_b is not None:
                idx = idx_b
                distance, bearing, elev = bp.point_aim(known[key_b])
                source = "board_pnp"
            else:
                distance, bearing, elev, source = _size_aim(det, cfg, K, Kinv)
        else:
            distance, bearing, elev, source = _size_aim(det, cfg, K, Kinv)
        aims.append(HoleAim(group=det.group, idx=idx, is_bonus=det.is_bonus,
                            is_lit=det.is_lit, distance=distance, bearing=bearing,
                            elev=elev, source=source))
    return SceneEstimate(board_pose=bp, aims=aims)


def _nearest_group_id(center_px, known, group, bp: BoardPose, K, dist):
    """把某群組已知孔中心用主板位姿投影，回傳離 center_px 最近的 (idx, key)。"""
    ids = [i for (g, i) in known if g == group]
    if not ids:
        return None, None
    obj = np.array([known[(group, i)] for i in ids], np.float64)
    proj, _ = cv2.projectPoints(obj, bp.rvec, bp.tvec, K, dist)
    proj = proj.reshape(-1, 2)
    j = int(np.argmin(np.linalg.norm(proj - center_px, axis=1)))
    return ids[j], (group, ids[j])


def _size_aim(det: Detection, cfg: Config, K, Kinv):
    """備援：表觀直徑 → 深度；中心像素 → 視線方向。"""
    depth = distance_from_diameter(det.diameter_px, _diameter_real(det, cfg), K[0, 0])
    ray = Kinv @ np.array([det.center[0], det.center[1], 1.0])
    p = ray * depth
    distance = float(np.linalg.norm(p))
    bearing = float(np.degrees(np.arctan2(p[0], p[2])))
    elev = float(np.degrees(np.arctan2(-p[1], p[2])))
    return distance, bearing, elev, "size"
