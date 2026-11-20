# -*- coding: utf-8 -*-
"""機器人主程式（比賽用）—— 網頁操控台 + 自主主迴圈，同一個程式、共用 MCU 連線與相機。

比賽流程：操作手在網頁按「開始」（規則允許用遙控器啟動）→ 前 60 秒 AI 自主投球（網頁手動輸入一律被拒）
→ 第 61 秒起操作手可按「切手動」接手，或讓 AI 繼續 →「交還 AI」可再交回。

    # 電腦上（不需任何硬體）：韌體模擬器 + 數位孿生相機 + 彈道計分
    python -m tools.robot_main --sim
    # Orin：真 Arduino + 真相機 + YOLO，載入賽前練習存的校正
    python -m tools.robot_main --mcu /dev/ttyACM0 --calib runs/calib/practice.json
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np

from rotbot.config import Config, load_config
from rotbot.perception.camera import SharedCamera
from rotbot.pipeline.loop import AutonomousRunner, ControlLoop
from rotbot.pipeline.state_machine import GameState, MatchStateMachine
from rotbot.teleop.aim_preview import AimPreview
from rotbot.teleop.server import TeleopServer

REPO_ROOT = Path(__file__).resolve().parent.parent


def build_robot(cfg: Config, mcu, camera, detector, *, rpm: float = 960, calib_path: str | None = None,
                practice: int = 0, observe_impact=None, auto_s: float = 60.0, match_s: float = 180.0,
                seed: int = 0, on_log=None, start_box: str | None = None):
    """組裝 (TeleopServer, AutonomousRunner, SharedCamera)；runner 尚未 start()。

    start_box："left" / "right" = 起始框（field.yaml 的 start_box，網頁待命時可切換）；None = field 原本的位置。
    """
    from rotbot.decision.calibration import AdaptiveCalibrator
    from rotbot.sim.closed_loop import new_calibrator, practice_calibration
    from rotbot.teleop.start_box import StartBoxChoice

    box = StartBoxChoice(cfg, start_box)
    shared = SharedCamera(camera, settle_s=cfg.camera_hw.settle_ms / 1000)   # 轉塔停下後等支架晃動停
    sm = MatchStateMachine(auto_s=auto_s, match_s=match_s)
    if calib_path:
        base_calib = lambda: AdaptiveCalibrator.load(calib_path, cfg.calibration)
    elif practice > 0:
        base_calib = lambda: practice_calibration(cfg, np.random.default_rng(seed), practice)
    else:
        base_calib = lambda: new_calibrator(cfg)

    def make_loop():
        mcu.flywheel(rpm)                                # 哨音一響就起轉
        return ControlLoop(box.cfg, shared, detector, mcu, rpm=rpm, calib=base_calib(), sm=sm,
                           observe_impact=observe_impact,
                           is_active=lambda: sm.update() == GameState.AUTONOMOUS)

    runner = AutonomousRunner(sm, make_loop, on_log=on_log)
    # 預估落點用「這一場」的校正器（AI 自主時學到的偏差，切手動後繼續適用）；開賽前用載入的校正檔
    idle_calib = AdaptiveCalibrator.load(calib_path, cfg.calibration) if calib_path else None
    preview = AimPreview(box.cfg, calib=lambda: runner.loop.calib if runner.loop is not None else idle_calib)
    box.listeners.append(preview.set_cfg)
    server = TeleopServer(mcu=mcu, camera=shared, sm=sm, rpm_preset=int(rpm), preview=preview,
                          chassis=cfg.chassis, ai=runner, start_box=box)
    return server, runner, shared


def main():
    ap = argparse.ArgumentParser(description="機器人主程式：網頁操控台 + 自主主迴圈")
    ap.add_argument("--sim", action="store_true", help="韌體模擬器 + 數位孿生（不需任何硬體）")
    ap.add_argument("--mcu", help="COM5 / /dev/ttyACM0 / socket://…")
    ap.add_argument("--camera", default="cfg", help="cfg = field.yaml 的 camera_hw（IMX296）/ USB 相機編號 / GStreamer 管線")
    ap.add_argument("--weights", default=str(REPO_ROOT / "models" / "grid_holes_yolo11n.pt"))
    ap.add_argument("--rpm", type=float, default=960)   # = field.yaml 的 rpm_model（11/20 實測 v0 約 18 m/s）
    ap.add_argument("--calib", help="賽前練習存下的校正狀態 JSON")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--auto-s", type=float, default=60.0)
    ap.add_argument("--start-box", choices=["left", "right"],
                    help="起始框（預設 field.yaml 的 start_box.side；--sim 預設網板置中 6 m，給了就模擬這個起始框）")
    args = ap.parse_args()
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    from aiohttp import web

    from rotbot.actuation.link import SerialMCU
    from tools.teleop_web import lan_ips, mcu_start_message, open_pose_store

    cfg = load_config()
    sil = None
    url = args.mcu
    if args.sim and not url:
        from tools.mcu_sil import SilProcess
        sil = SilProcess("full").start()
        url = sil.url
        print(f"[MCU] {sil.banner}")
    if not url:
        ap.error("請給 --mcu 或 --sim")
    store = open_pose_store(cfg, sim=args.sim)
    mcu = SerialMCU(url, pose_store=store, keep_pose_on_estop=cfg.turret.holds_unpowered).open()
    if sil is not None:
        mcu.zero(0.0, 0.0)
        mcu.aim(15.0, 0.0)
    observe = None
    if args.sim:
        from rotbot.sim.twin import TwinCamera, TwinDetector, TwinImpactObserver
        truth = cfg.at_start_box(args.start_box) if args.start_box else cfg   # 數位孿生的網板真實位置
        camera = TwinCamera(truth, mcu, lit_bonus=1)
        detector = TwinDetector(camera, noise_px=0.5)
        observe = TwinImpactObserver(truth, np.random.default_rng(1))
    else:
        from rotbot.perception.camera import open_camera
        from rotbot.perception.detector import YoloDetector
        camera = open_camera(args.camera, cfg)
        detector = YoloDetector(args.weights, lit_check=cfg.lit_check)

    def on_log(log):
        res = "" if log.hit is None else (f"命中 +{log.points}" if log.hit else "未中")
        if log.source == "last_seen":
            res += "（沒看到網板，沿用上次的位置）"
        elif log.source == "blind":
            res += "（一直找不到網板，用起始框設定的位置盲投）"
        print(f"[AI] 第 {log.ball:2d} 球 {log.target}{'★' if log.is_lit else ''} "
              f"仰角 {log.pitch_deg:6.2f}° 水平 {log.yaw_deg:6.2f}° {log.total_ms:5.0f} ms {res}")

    server, runner, shared = build_robot(cfg, mcu, camera, detector, rpm=args.rpm,
                                         calib_path=args.calib, practice=9 if args.sim else 0,
                                         observe_impact=observe, auto_s=args.auto_s, on_log=on_log,
                                         start_box=args.start_box or (None if args.sim else cfg.start_box.side))
    print(mcu_start_message(mcu))
    if server.start_box.side is not None:
        print(f"[起始框] {server.start_box.describe()}：不對就在網頁待命時切換左框、右框")
    runner.start()
    print("\n操控頁：http://localhost:%d" % args.port)
    for ip in lan_ips():
        print(f"        http://{ip}:{args.port}")
    print("按「開始」= 哨音：前 %.0f 秒 AI 自主投球，之後可切手動。\n" % args.auto_s)
    try:
        web.run_app(server.build_app(), host=args.host, port=args.port, print=None)
    finally:
        runner.stop()
        shared.release()
        try:
            mcu.stop()
        except Exception:
            pass
        mcu.close()
        if store is not None:
            store.close()                          # 最後一筆角度寫進檔案
        if sil is not None:
            sil.stop()


if __name__ == "__main__":
    main()
