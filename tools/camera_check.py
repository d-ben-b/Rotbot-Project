# -*- coding: utf-8 -*-
"""實機相機檢查 —— 確認 configs/field.yaml 的 camera_hw 設定拍得到東西，並量速度。

在接上相機的機器（Jetson）上跑；輸出存到 runs/camera_check/（不入 git）。

用法：
    python -m tools.camera_check                     # 用 field.yaml 的曝光/增益/白平衡
    python -m tools.camera_check --exposure-us 250 --gain-db 18    # 試追球用的短曝光
    python -m tools.camera_check --auto-wb           # 對著白紙/灰牆量白平衡，印出要寫進 YAML 的值

印出：read() 幀率與每張耗時、畫面亮度與過曝比例、兩張之間的時間差抖動。
"""
from __future__ import annotations

import argparse
import dataclasses
import time
from pathlib import Path

import cv2
import numpy as np

# 允許直接以檔案路徑執行（python tools/camera_check.py）：把專案根目錄補進 sys.path
if __package__ in (None, ""):
    import os
    import sys
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rotbot.config import load_config
from rotbot.perception.camera import make_camera

OUT_DIR = Path(__file__).resolve().parent.parent / "runs" / "camera_check"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--frames", type=int, default=120, help="量測用的張數")
    ap.add_argument("--exposure-us", type=float, help="覆蓋 YAML 的曝光 (µs)")
    ap.add_argument("--gain-db", type=float, help="覆蓋 YAML 的增益 (dB)")
    ap.add_argument("--auto-wb", action="store_true", help="用這個畫面估白平衡（畫面要是白/灰色）")
    ap.add_argument("--no-save", action="store_true")
    args = ap.parse_args()

    cfg = load_config()
    hw = cfg.camera_hw
    overrides = {k: v for k, v in (("exposure_us", args.exposure_us), ("gain_db", args.gain_db))
                 if v is not None}
    if overrides:
        cfg = dataclasses.replace(cfg, camera_hw=dataclasses.replace(hw, **overrides))
        hw = cfg.camera_hw
    cam = make_camera(cfg)
    print(f"相機：{hw.backend} {hw.device}  曝光 {hw.exposure_us:g} µs  增益 {hw.gain_db:g} dB  "
          f"白平衡 R×{hw.wb_r:g} B×{hw.wb_b:g}  ccm={hw.ccm}")
    try:
        t0 = time.perf_counter()
        frame = cam.read()
        print(f"開啟到第一張：{time.perf_counter() - t0:.2f} s，影像 {frame.shape[1]}x{frame.shape[0]}")

        if args.auto_wb:
            from rotbot.perception.imx296 import grey_world_gains
            wb_r, wb_b = grey_world_gains(cam.read_raw())
            cam.set_white_balance(wb_r, wb_b)
            print(f"量到的白平衡 → 寫進 configs/field.yaml：  wb_r: {wb_r:.2f}    wb_b: {wb_b:.2f}")

        stamps = []
        t0 = time.perf_counter()
        for _ in range(args.frames):
            frame = cam.read()
            stamps.append(getattr(cam, "timestamp", time.perf_counter()))
        dt = time.perf_counter() - t0
        gaps = np.diff(stamps) * 1000
        print(f"read()：{args.frames / dt:.1f} fps（每張 {dt / args.frames * 1000:.1f} ms）；"
              f"相鄰兩張相隔 {gaps.mean():.1f} ms（最大 {gaps.max():.1f}）")

        grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        sat = float((frame.max(axis=2) >= 250).mean() * 100)
        print(f"亮度：平均 {grey.mean():.0f}/255，最暗 1% = {np.percentile(grey, 1):.0f}，"
              f"過曝 {sat:.1f}%")
        if grey.mean() < 40:
            print("  ⚠ 畫面偏暗：開大光圈、加長曝光或加 gain")
        if sat > 5:
            print("  ⚠ 過曝面積偏大：縮短曝光或降低 gain（過曝處的顏色與邊緣都不可信）")

        try:
            cam.intrinsics()
        except RuntimeError as e:
            print(f"  ⚠ 內參尚未標定：{e}")

        if not args.no_save:
            OUT_DIR.mkdir(parents=True, exist_ok=True)
            path = OUT_DIR / time.strftime("frame_%Y%m%d_%H%M%S.png")
            cv2.imwrite(str(path), frame)
            print(f"已存：{path}")
    finally:
        cam.release()


if __name__ == "__main__":
    main()
