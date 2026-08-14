# -*- coding: utf-8 -*-
"""投球 demo 影片 —— 閉迴路一場比賽的視覺化（進度報告用）。

兩種模式：
  --mode truth   : 板面落點視圖（感知=真值+雜訊；快）
  --mode render  : 左=相機視角+YOLO偵測、右=板面落點（感知=真 render 影像+真 YOLO；慢）

板面視圖：正面看網板，圓孔、亮起加分孔、每球落點（綠命中/紅未中）、比分與命中率，
逐球累積可見「找手感」——落點越投越集中。

用法：
    python -m tools.demo_match_video --mode truth  --out runs/demo/match_truth.mp4
    python -m tools.demo_match_video --mode render --out runs/demo/match_render.mp4
（標註用英文，因 OpenCV 無法渲染中文字。）
"""
from __future__ import annotations

import argparse

import cv2
import numpy as np

from rotbot.config import load_config
from rotbot.sim.closed_loop import run_closed_loop, run_closed_loop_render

# 配色 (BGR)
C_BG = (24, 20, 18)
C_MAIN = (150, 150, 155)
C_BONUS = (80, 170, 235)
C_LIT = (90, 235, 120)
C_HIT = (90, 235, 120)
C_MISS = (70, 70, 235)
C_TARGET = (60, 220, 235)
C_TEXT = (240, 240, 240)


class MatchVideo:
    """逐球接收 on_shot payload，畫成影格寫入影片。"""

    def __init__(self, cfg, out_path, mode="truth", fps=15, hold=7):
        self.cfg = cfg
        self.mode = mode
        self.out_path = out_path
        self.fps = fps
        self.hold = hold
        self.landings: list[tuple[int, int, bool]] = []
        self.lit_key = None          # 本場亮孔 (group, idx)，一旦得知就整場畫綠
        self.writer = None
        # 板面座標範圍（世界：z 橫向、y 高度）
        self.zmin, self.zmax = -1.3, 1.3
        self.ymin, self.ymax = 0.5, 2.9
        self.BW, self.BH, self.M = 760, 800, 50
        self.scale = min((self.BW - 2 * self.M) / (self.zmax - self.zmin),
                         (self.BH - 2 * self.M) / (self.ymax - self.ymin))
        self.zc, self.yc = 0.0, 0.5 * (self.ymin + self.ymax)

    def _to_px(self, z, y):
        px = int(self.BW / 2 + (z - self.zc) * self.scale)
        py = int(self.BH / 2 - (y - self.yc) * self.scale)
        return px, py

    def _draw_board(self, p):
        img = np.full((self.BH, self.BW, 3), C_BG, np.uint8)
        # 圓孔
        for (g, i, diam, C) in self.cfg.hole_centers():
            z = self.cfg.field.grid_cx + C[0]
            y = self.cfg.field.grid_cy + C[1]
            px, py = self._to_px(z, y)
            r = max(4, int(diam / 2 * self.scale))
            is_lit = (g, i) == self.lit_key
            col = C_LIT if is_lit else (C_BONUS if g == "bonus" else C_MAIN)
            cv2.circle(img, (px, py), r, col, 2, cv2.LINE_AA)
            if is_lit:
                cv2.putText(img, "x2", (px - 14, py + 6), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, C_LIT, 2, cv2.LINE_AA)
        # 過往落點（淡）
        for (px, py, hit) in self.landings[:-1]:
            cv2.circle(img, (px, py), 4, (90, 90, 90), -1, cv2.LINE_AA)
        # 目標高亮 + 本球落點
        tz, ty = p["target_zy"]
        tpx, tpy = self._to_px(tz, ty)
        cv2.circle(img, (tpx, tpy), int(0.25 * self.scale), C_TARGET, 1, cv2.LINE_AA)
        if p["land_zy"] is not None and self.landings:
            lpx, lpy, hit = self.landings[-1]
            cv2.line(img, (tpx, tpy), (lpx, lpy), (110, 110, 110), 1, cv2.LINE_AA)
            cv2.circle(img, (lpx, lpy), 7, C_HIT if hit else C_MISS, -1, cv2.LINE_AA)
        self._hud(img, p)
        return img

    def _hud(self, img, p):
        res = p["result"]
        cv2.rectangle(img, (0, 0), (self.BW, 78), (12, 12, 14), -1)
        tag = "TARGET x2" if res.is_lit else f"{res.group}#{res.idx}"
        err = "-" if np.isnan(res.error_m) else f"{res.error_m * 100:.1f}cm"
        mark = "HIT" if res.hit else "MISS"
        line1 = f"Ball {res.ball}/{self.cfg.n_balls}   Target: {tag}   err {err}   {mark}"
        n = len(self.landings)
        hr = 100 * sum(h for *_, h in self.landings) / n if n else 0
        line2 = f"Score: {p['total_points']}   Hit rate: {hr:.0f}%   calibration ON"
        cv2.putText(img, line1, (16, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, C_TEXT, 2, cv2.LINE_AA)
        cv2.putText(img, line2, (16, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.7, C_TEXT, 2, cv2.LINE_AA)
        cv2.putText(img, "Board (landing points)", (16, self.BH - 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (150, 150, 150), 1, cv2.LINE_AA)

    def _draw_camera(self, p):
        img = p["image"].copy()
        for d in (p.get("detections") or []):
            x1, y1, x2, y2 = [int(v) for v in d.bbox]
            cx, cy = int(d.center[0]), int(d.center[1])
            col = C_LIT if d.is_lit else (C_BONUS if d.is_bonus else C_MAIN)
            cv2.ellipse(img, (cx, cy), ((x2 - x1) // 2, (y2 - y1) // 2), 0, 0, 360,
                        col, 3, cv2.LINE_AA)
        h = self.BH
        w = int(img.shape[1] * h / img.shape[0])
        img = cv2.resize(img, (w, h))
        cv2.rectangle(img, (0, 0), (w, 40), (12, 12, 14), -1)
        cv2.putText(img, "Camera view + YOLO detection", (16, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, C_TEXT, 2, cv2.LINE_AA)
        return img

    def on_shot(self, p):
        res = p["result"]
        if res.is_lit:                       # 記住本場亮孔，整場畫綠
            self.lit_key = (res.group, res.idx)
        if p["land_zy"] is not None:
            lz, ly = p["land_zy"]
            self.landings.append((*self._to_px(lz, ly), p["result"].hit))
        board = self._draw_board(p)
        frame = board if self.mode == "truth" else cv2.hconcat([self._draw_camera(p), board])
        if self.writer is None:
            h, w = frame.shape[:2]
            self.writer = cv2.VideoWriter(self.out_path,
                                          cv2.VideoWriter_fourcc(*"mp4v"), self.fps, (w, h))
        for _ in range(self.hold):
            self.writer.write(frame)

    def close(self):
        if self.writer:
            self.writer.release()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["truth", "render"], default="truth")
    ap.add_argument("--out", default=None)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--weights", default="models/grid_holes_yolo11n.pt")
    args = ap.parse_args()
    cfg = load_config()
    out = args.out or f"runs/demo/match_{args.mode}.mp4"
    import os
    os.makedirs(os.path.dirname(out), exist_ok=True)

    vid = MatchVideo(cfg, out, mode=args.mode)
    rng = np.random.default_rng(args.seed)
    if args.mode == "render":
        from rotbot.perception.detector import YoloDetector
        det = YoloDetector(args.weights)
        r = run_closed_loop_render(cfg, det, rng, correct=True, on_shot=vid.on_shot)
    else:
        r = run_closed_loop(cfg, rng, correct=True, on_shot=vid.on_shot)
    vid.close()
    print(f"完成：{out}  總分 {r.total_points}  命中率 {r.hit_rate * 100:.0f}%")


if __name__ == "__main__":
    main()
