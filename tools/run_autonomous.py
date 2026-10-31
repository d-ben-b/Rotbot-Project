# -*- coding: utf-8 -*-
"""自主主迴圈 —— 跑一整場（前 60 秒 AI 自主的那支程式）。

    # 電腦上：真韌體（模擬器）+ 數位孿生相機（轉塔轉到哪就看到哪）+ 彈道世界計分
    python -m tools.run_autonomous --sim
    python -m tools.run_autonomous --sim --detector yolo          # 改用真 YOLO 看合成影像
    # 只有 Arduino + 馬達（還沒有發射機構/相機）：轉塔真的會動，影像與計分仍用孿生
    python -m tools.run_autonomous --mcu COM5 --twin
    # 實機（Orin）：真相機 + YOLO + Arduino；沒有落點回授時線上校正不更新（用賽前練習存的手感）
    python -m tools.run_autonomous --mcu /dev/ttyACM0 --calib runs/calib/practice.json

結果另存 runs/autonomous/<時間>.json（逐球：目標、指令、各階段耗時、誤差、命中）。
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import sys
import time
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np

from rotbot.config import load_config
from rotbot.pipeline.loop import ControlLoop
from rotbot.pipeline.state_machine import MatchStateMachine

REPO_ROOT = Path(__file__).resolve().parent.parent


def main():
    ap = argparse.ArgumentParser(description="自主主迴圈：跑一整場")
    ap.add_argument("--sim", action="store_true", help="韌體模擬器 + 數位孿生（不需任何硬體）")
    ap.add_argument("--mcu", help="COM5 / /dev/ttyACM0 / socket://…（--sim 時自動啟動模擬器）")
    ap.add_argument("--twin", action="store_true", help="影像與計分用數位孿生（接真 Arduino 測轉塔時用）")
    ap.add_argument("--camera", default="cfg", help="非孿生時：cfg = field.yaml 的 camera_hw（IMX296）/ USB 相機編號 / GStreamer 管線")
    ap.add_argument("--detector", choices=["perfect", "yolo"], default="perfect",
                    help="孿生時：perfect=讀真值、yolo=真模型看合成影像；實機一律 yolo")
    ap.add_argument("--weights", default=str(REPO_ROOT / "models" / "grid_holes_yolo11n.pt"))
    ap.add_argument("--rpm", type=float, default=2400)
    ap.add_argument("--balls", type=int, default=27)
    ap.add_argument("--practice", type=int, default=9, help="孿生時：先跑幾球賽前練習暖機校正（0=不暖機）")
    ap.add_argument("--calib", help="載入賽前練習存下的校正狀態 JSON")
    ap.add_argument("--save-calib", help="結束後把校正狀態存到這裡（練習時用）")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")

    cfg = load_config()
    twin = args.sim or args.twin
    sil = None
    from rotbot.actuation.link import SerialMCU
    url = args.mcu
    if args.sim and not url:
        from tools.mcu_sil import SilProcess
        sil = SilProcess("full").start()
        url = sil.url
        print(f"[MCU] {sil.banner}")
    if not url:
        ap.error("請給 --mcu 或 --sim")
    mcu = SerialMCU(url).open()
    try:
        if sil is not None:
            mcu.zero(0.0, 0.0)
        elif not mcu.get_status().homed:
            print("⚠️ MCU 尚未 ZERO：請把轉塔擺到起始姿態後用操控頁「歸零」，或先執行 mcu.zero()")
            return

        observe = None
        if twin:
            from rotbot.sim.twin import TwinCamera, TwinDetector, TwinImpactObserver
            camera = TwinCamera(cfg, mcu, rng=np.random.default_rng(args.seed), lit_bonus=1)
            if args.detector == "yolo":
                from rotbot.perception.detector import YoloDetector
                detector = YoloDetector(args.weights)
            else:
                detector = TwinDetector(camera, noise_px=0.5, rng=np.random.default_rng(args.seed))
            observe = TwinImpactObserver(cfg, np.random.default_rng(args.seed + 1))
        else:
            from rotbot.perception.camera import open_camera
            from rotbot.perception.detector import YoloDetector
            camera = open_camera(args.camera, cfg)
            detector = YoloDetector(args.weights)

        calib = None
        if args.calib:
            from rotbot.decision.calibration import AdaptiveCalibrator
            calib = AdaptiveCalibrator.load(args.calib, cfg.calibration)
        elif twin and args.practice > 0:
            from rotbot.sim.closed_loop import practice_calibration
            calib = practice_calibration(cfg, np.random.default_rng(args.seed + 2), args.practice)
        else:
            from rotbot.sim.closed_loop import new_calibrator
            calib = new_calibrator(cfg)

        from rotbot.decision.strategy import ShotPlanner
        sm = MatchStateMachine()
        loop = ControlLoop(cfg, camera, detector, mcu, rpm=args.rpm, calib=calib,
                           planner=ShotPlanner(cfg, total_balls=args.balls), sm=sm,
                           observe_impact=observe)
        print(f"掃視姿態 (俯仰, 水平) = ({loop.survey[0]:.2f}°, {loop.survey[1]:.2f}°)；"
              f"飛輪 {args.rpm:.0f} rpm；{args.balls} 球")
        sm.start()
        mcu.flywheel(args.rpm)                           # 哨音一響就先起轉
        print(f"{'球':>3} {'目標':>9} {'仰角°':>7} {'水平°':>7} {'校正°':>13} {'看':>6} {'轉+等':>6} "
              f"{'推':>5} {'總ms':>6} {'誤差cm':>7} 結果")
        t0 = time.monotonic()
        while loop.planner.remaining > 0:
            if sm.update().value != "AUTONOMOUS":
                break
            log = loop.step()
            if log is None:
                if loop.no_target or loop.skipped > 5:
                    break
                continue
            tag = f"{log.target}{'★' if log.is_lit else ''}"
            res = "" if log.hit is None else ("○ +%d" % log.points if log.hit else "×")
            err = "" if log.error_cm is None else f"{log.error_cm:7.1f}"
            print(f"{log.ball:>3} {tag:>9} {log.pitch_deg:7.2f} {log.yaw_deg:7.2f} "
                  f"({log.corr_deg[0]:+.2f},{log.corr_deg[1]:+.2f}) {log.perceive_ms:6.0f} "
                  f"{log.aim_ms:6.0f} {log.fire_ms:5.0f} {log.total_ms:6.0f} {err:>7} {res}")
        wall = time.monotonic() - t0
        logs = loop.logs
        mcu.flywheel(0)
        n = len(logs)
        print("-" * 86)
        line = (f"共 {n} 球、韌體推球計數 {mcu.get_status().shots}、用時 {wall:.1f} s"
                f"（平均 {wall / max(n, 1):.2f} s/球；規則節拍 6.7 s/球）")
        if observe is not None and n:
            hits = sum(bool(l.hit) for l in logs)
            lit = [l for l in logs if l.is_lit]
            line += (f"\n得分 {observe.points}、命中 {hits}/{n}（{hits / n:.0%}）、"
                     f"亮孔 {sum(bool(l.hit) for l in lit)}/{len(lit)}（模擬/合成評估）")
        print(line)
        if args.save_calib:
            calib.save(args.save_calib)
            print(f"校正狀態已存：{args.save_calib}")
        out = REPO_ROOT / "runs" / "autonomous"
        out.mkdir(parents=True, exist_ok=True)
        p = out / f"{time.strftime('%Y%m%d_%H%M%S')}.json"
        p.write_text(json.dumps(dict(args=vars(args), wall_s=wall,
                                     points=getattr(observe, "points", None),
                                     shots=[dataclasses.asdict(l) for l in logs]),
                                ensure_ascii=False, indent=1), encoding="utf-8")
        print(f"→ {p}")
    finally:
        try:
            mcu.stop()
        except Exception:
            pass
        mcu.close()
        if sil is not None:
            sil.stop()


if __name__ == "__main__":
    main()
