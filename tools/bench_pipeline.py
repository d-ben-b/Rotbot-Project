# -*- coding: utf-8 -*-
"""感知 + 決策 每階段延遲量測 —— 在 Orin 上跑，確認自主迴圈的延遲預算（每球節拍 6.7 s）。

    python -m tools.bench_pipeline                                   # 合成影像 + PyTorch 權重
    python -m tools.bench_pipeline --camera cfg                      # 含真相機擷取（field.yaml 的 IMX296）
    python -m tools.bench_pipeline --export-trt                      # （Orin 上）.pt → TensorRT FP16 引擎
    python -m tools.bench_pipeline --weights models/grid_holes_yolo11n.engine

TensorRT 引擎與裝置綁定，必須在 Orin 本機匯出；PC 上沒有 TensorRT 時 --export-trt 會失敗屬正常。
"""
from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np

from rotbot.config import load_config
from rotbot.decision.strategy import ShotPlanner, plan_next_shot
from rotbot.perception.detector import YoloDetector
from rotbot.perception.localize import localize_holes

REPO_ROOT = Path(__file__).resolve().parent.parent


def _stats(xs: list[float]) -> str:
    xs = sorted(xs)
    p95 = xs[int(0.95 * (len(xs) - 1))]
    return f"p50 {statistics.median(xs):7.1f}  p95 {p95:7.1f}  max {xs[-1]:7.1f} ms"


def main():
    ap = argparse.ArgumentParser(description="感知/決策延遲量測")
    ap.add_argument("--weights", default=str(REPO_ROOT / "models" / "grid_holes_yolo11n.pt"))
    ap.add_argument("--camera", default=None, help="cfg（field.yaml 的 IMX296）/ USB 相機編號 / GStreamer 管線；省略 = 合成影像（6m 正前方）")
    ap.add_argument("--n", type=int, default=100)
    ap.add_argument("--imgsz", type=int, default=960)
    ap.add_argument("--device", default="0")
    ap.add_argument("--export-trt", action="store_true", help="先匯出 TensorRT FP16 引擎再量")
    args = ap.parse_args()
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    cfg = load_config()

    weights = args.weights
    if args.export_trt:
        from ultralytics import YOLO
        weights = YOLO(weights).export(format="engine", half=True, imgsz=args.imgsz, device=args.device)
        print(f"TensorRT 引擎：{weights}")

    if args.camera is not None:
        from rotbot.perception.camera import open_camera
        cam = open_camera(args.camera, cfg)
        grab = cam.read
        K = cam.intrinsics()
    else:
        from rotbot.sim.scene import SceneGenerator
        img = SceneGenerator(cfg, randomize=False).sample(lit_bonus=1).image
        grab = lambda: img                                   # 合成影像只做一次，不計入擷取時間
        K = cfg.camera.matrix()

    det = YoloDetector(weights, device=args.device, imgsz=args.imgsz, lit_check=cfg.lit_check)
    t = {k: [] for k in ("capture", "detect", "localize", "plan", "total")}
    for i in range(args.n + 10):
        t0 = time.perf_counter()
        frame = grab()
        t1 = time.perf_counter()
        dets = det.detect(frame)
        t2 = time.perf_counter()
        est = localize_holes(dets, cfg, K)
        t3 = time.perf_counter()
        plan_next_shot(est.aims, ShotPlanner(cfg), cfg)
        t4 = time.perf_counter()
        if i >= 10:                                          # 前 10 次暖機（CUDA/TensorRT 初始化）
            for k, a, b in (("capture", t0, t1), ("detect", t1, t2), ("localize", t2, t3),
                            ("plan", t3, t4), ("total", t0, t4)):
                t[k].append((b - a) * 1000)
    print(f"權重 {Path(weights).name}、imgsz {args.imgsz}、{'相機 ' + args.camera if args.camera else '合成影像'}、"
          f"n={args.n}、偵測 {len(dets)} 孔")
    for k, v in t.items():
        print(f"  {k:9s} {_stats(v)}")
    print(f"  → 感知+決策 {1000 / statistics.median(t['total']):.1f} 次/秒")
    bonus = sorted((d for d in dets if d.is_bonus), key=lambda d: d.center[0])
    if bonus:                                                # 實拍時用來定 field.yaml 的 lit_check 門檻
        print("  加分孔（左→右）：" + "、".join(
            f"{'亮' if d.is_lit else '暗'}{'（顏色備援）' if d.extra.get('lit_by') == 'color' else ''} "
            f"紅色比例 {d.extra.get('red_frac', float('nan')):.3f}" for d in bonus))
    center = next((a for a in est.aims if (a.group, a.idx) == ("main", 4)), None)
    if est.board_pose is None or center is None:
        print("  定位：整板 PnP 失敗（孔沒拍全或偵測太少）")
    else:                                                    # 實拍時拿捲尺量「鏡頭 → 中排中孔心」比對
        print(f"  定位：看到 {len(est.aims)} 孔；中排中孔心距離 {center.distance:.3f} m、"
              f"方位 {center.bearing:+.2f}°、俯仰 {center.elev:+.2f}°")


if __name__ == "__main__":
    main()
