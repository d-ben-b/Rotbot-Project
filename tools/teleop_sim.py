# -*- coding: utf-8 -*-
"""互動物理沙盒 —— PyBullet 真實投球模擬（手動 / YOLO自動 可切換）。

畫面：
  * 左：PyBullet 3D 世界（機器人、轉塔、網板、飛行中的球、落點）+ 右側滑桿/按鈕
  * 另開視窗「Robot camera」：機器人同軸相機視角（手動＝槍口方向；自動＝掃視整板）

手動：拖 pitch/yaw/v0 滑桿瞄準，按 SHOOT 投球。
自動：按 TOGGLE 切到 auto → 感知(掃視相機)→定位→策略選目標(亮孔優先)→瞄準→投球，
      逐球用落點誤差線上校正（找手感）。感知來源 --perception truth（穩定）或 yolo（真模型，有域差）。

用法：
    python -m tools.teleop_sim                        # 手動起手，truth 自動感知
    python -m tools.teleop_sim --perception yolo --weights models/grid_holes_yolo11n.pt
    python -m tools.teleop_sim --no-drag              # 關空氣阻力（瞄準模型即命中，適合先看幾何）
"""
from __future__ import annotations

import argparse
import math
import time

import cv2
import numpy as np

from rotbot.config import CameraConfig, load_config
from rotbot.decision import ballistics
from rotbot.decision.calibration import AdaptiveCalibrator
from rotbot.decision.strategy import ShotPlanner, plan_next_shot
from rotbot.perception.localize import HoleAim, localize_holes
from rotbot.sim import pybullet_camera as pcam
from rotbot.sim.pybullet_world import PyBulletWorld
from rotbot.sim.robot_spec import load_robot_spec
from rotbot.sim.world import BallisticWorld

C_TARGET = (60, 220, 235)
C_TEXT = (240, 240, 240)


def _scaled_cam(cam: CameraConfig, s: float) -> CameraConfig:
    return CameraConfig(fx=cam.fx * s, fy=cam.fy * s, cx=cam.cx * s, cy=cam.cy * s,
                        width=int(cam.width * s), height=int(cam.height * s))


def _hole_int_index(group: str, idx: int) -> int:
    return idx if group == "main" else 9 + idx


class TeleopSim:
    def __init__(self, args):
        self.args = args
        self.cfg = load_config()
        self.robot = load_robot_spec()
        self.world = PyBulletWorld(self.cfg, self.robot, gui=True,
                                   drag=not args.no_drag, spin=not args.no_spin)
        self.glare = args.glare
        self.disp_cam = _scaled_cam(self.cfg.camera, 0.5)
        self.K = self.cfg.camera.matrix()

        self.mode = "manual"
        self.detector = None
        if args.perception == "yolo":
            from rotbot.perception.detector import YoloDetector
            self.detector = YoloDetector(args.weights, device=args.device)

        # 決策 / 校正（自動模式用）
        self.planner = ShotPlanner(self.cfg)
        jp, jy = ballistics.estimate_sensitivity(
            BallisticWorld(self.cfg).shoot, (0.0, self.cfg.field.grid_cy), self.cfg)
        self.calib = AdaptiveCalibrator(self.cfg.calibration, jp, jy, n_cells=12)
        self.total_points = 0
        self.n_shots = 0
        self.n_hits = 0
        self.pending = None            # 自動模式：飛行中那球的 (target dict, hidx, tz, ty)
        self.next_auto_t = 0.0
        self.last_line = "manual mode - aim with sliders, press SHOOT"

        self._make_controls()

    # ── GUI 控制項 ────────────────────────────────────────────
    def _make_controls(self):
        p = self.world.p
        self.s_pitch = p.addUserDebugParameter("pitch_deg", -5, 70, 25)
        self.s_yaw = p.addUserDebugParameter("yaw_deg", -30, 30, 0)
        self.s_v0 = p.addUserDebugParameter("v0_mps", 5, 15, self.cfg.launch.v0_real)
        self.b_shoot = p.addUserDebugParameter("SHOOT (click)", 1, 0, 1)
        self.b_mode = p.addUserDebugParameter("TOGGLE manual/auto", 1, 0, 1)
        self.b_reset = p.addUserDebugParameter("RESET", 1, 0, 1)
        self.b_relight = p.addUserDebugParameter("relight bonus", 1, 0, 1)
        self._c_shoot = self._c_mode = self._c_reset = self._c_relight = 0
        self._hud = p.addUserDebugText("", [0, 0, 2.0], [1, 1, 1])

    def _clicked(self, pid, cache_attr) -> bool:
        v = self.world.p.readUserDebugParameter(pid)
        prev = getattr(self, cache_attr)
        setattr(self, cache_attr, v)
        return v > prev

    # ── 感知（自動模式）─────────────────────────────────────────
    def _survey_pose(self):
        board_c = np.array([self.cfg.field.dist_x, -self.cfg.field.grid_cx, self.cfg.field.grid_cy])
        forward = board_c - self.world.pivot
        forward = forward / np.linalg.norm(forward)
        eye = self.world.pivot + forward * 0.30        # 相機在槍口，避免拍到自己的發射器
        return eye, forward

    def _truth_aims(self, eye, forward) -> list[HoleAim]:
        right, down, f, _ = pcam._cam_axes(forward)
        R_wc = np.column_stack([right, down, f])
        aims = []
        for (g, i, _d, C) in self.cfg.hole_centers():
            pw = np.array([self.cfg.field.dist_x, -(self.cfg.field.grid_cx + C[0]),
                           self.cfg.field.grid_cy + C[1]])
            pc = R_wc.T @ (pw - eye)
            if pc[2] <= 0:
                continue
            dist = float(np.linalg.norm(pc))
            bearing = math.degrees(math.atan2(pc[0], pc[2]))
            elev = math.degrees(math.atan2(-pc[1], pc[2]))
            aims.append(HoleAim(group=g, idx=i, is_bonus=(g == "bonus"),
                                is_lit=(g == "bonus" and i == self.world.lit_idx),
                                distance=dist, bearing=bearing, elev=elev, source="truth"))
        return aims

    def _perceive(self, eye, forward):
        """回傳 (aims, extrinsic, display_image)。"""
        extr = pcam.camera_extrinsic(eye, forward, self.world.launch_h)
        if self.detector is not None:
            img = pcam.render(eye, forward, self.cfg.camera, self.world.client,
                              glare=self.glare)
            dets = self.detector.detect(img)
            aims = localize_holes(dets, self.cfg, self.K).aims
        else:
            img = pcam.render(eye, forward, self.disp_cam, self.world.client,
                              glare=self.glare)
            aims = self._truth_aims(eye, forward)
        return aims, extr, img

    # ── 自動投球一發 ──────────────────────────────────────────
    def _auto_shot(self):
        if self.planner.remaining <= 0:
            self.last_line = f"AUTO done - {self.total_points} pts, {self._hitrate()}"
            return
        eye, forward = self._survey_pose()
        aims, extr, _img = self._perceive(eye, forward)
        plan = plan_next_shot(aims, self.planner, self.cfg, extrinsic=extr)
        if plan is None:
            self.last_line = "AUTO: no reachable target"
            return
        t = plan.target
        hidx = _hole_int_index(t.group, t.idx)
        pitch = math.radians(plan.pitch_deg)
        yaw = math.radians(plan.yaw_deg)
        dp, dy = self.calib.correction(hidx)
        pitch += dp
        yaw += dy
        tz, ty = self.world._hole_world_zy(t.group, t.idx)
        self.world.shoot(pitch, yaw, v0=self.args_v0(), mech_noise=True,
                         target=dict(group=t.group, idx=t.idx, is_lit=t.is_lit))
        self.planner.record(t)
        self.pending = (hidx, tz, ty)
        tag = "BONUS x2" if t.is_lit else f"{t.group}#{t.idx}"
        self.last_line = f"AUTO shoot {tag}  (ball {self.n_shots + 1})"

    def args_v0(self):
        return self.world.p.readUserDebugParameter(self.s_v0)

    # ── 落點事件處理 ──────────────────────────────────────────
    def _on_events(self, events):
        for ev in events:
            self.n_shots += 1
            if ev.hit:
                self.n_hits += 1
                self.total_points += ev.points
            # 自動模式線上校正（用該球的目標與實際落點）
            if self.pending is not None and ev.target is not None and ev.landing is not None:
                hidx, tz, ty = self.pending
                lz, ly = ev.landing
                mz = (tz - lz) + np.random.normal(0, self.cfg.calibration.meas_noise_m)
                my = (ty - ly) + np.random.normal(0, self.cfg.calibration.meas_noise_m)
                self.calib.update(hidx, (mz, my))
                self.pending = None
                self.next_auto_t = time.time() + 0.4
            mark = "HIT" if ev.hit else "miss"
            err = "-" if math.isnan(ev.error_m) else f"{ev.error_m * 100:.0f}cm"
            self.last_line = f"{mark} {ev.group}#{ev.idx}  err {err}  ({self.total_points} pts)"

    def _hitrate(self):
        return f"{100 * self.n_hits / self.n_shots:.0f}% hit" if self.n_shots else "0% hit"

    # ── 相機視窗 ──────────────────────────────────────────────
    def _draw_camera(self):
        if self.mode == "auto":
            eye, forward = self._survey_pose()
        else:
            pitch = math.radians(self.world.p.readUserDebugParameter(self.s_pitch))
            yaw = math.radians(self.world.p.readUserDebugParameter(self.s_yaw))
            eye, forward = self.world.muzzle(pitch, yaw), self.world.aim_dir(pitch, yaw)
        img = pcam.render(eye, forward, self.disp_cam, self.world.client, glare=self.glare)
        cv2.rectangle(img, (0, 0), (img.shape[1], 34), (12, 12, 14), -1)
        cv2.putText(img, f"Robot camera [{self.mode}]", (12, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, C_TEXT, 2, cv2.LINE_AA)
        cv2.putText(img, self.last_line, (12, img.shape[0] - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, C_TARGET, 2, cv2.LINE_AA)
        cv2.imshow("Robot camera", img)

    # ── 主迴圈 ───────────────────────────────────────────────
    def run(self):
        p = self.world.p
        try:
            while True:
                if self._clicked(self.b_mode, "_c_mode"):
                    self.mode = "auto" if self.mode == "manual" else "manual"
                    self.next_auto_t = time.time() + 0.3
                if self._clicked(self.b_reset, "_c_reset"):
                    self._reset()
                if self._clicked(self.b_relight, "_c_relight"):
                    self.world.set_lit(int(np.random.randint(0, 3)))

                pitch = math.radians(p.readUserDebugParameter(self.s_pitch))
                yaw = math.radians(p.readUserDebugParameter(self.s_yaw))
                self.world.set_turret(pitch, yaw)

                if self.mode == "manual":
                    if self._clicked(self.b_shoot, "_c_shoot"):
                        self.world.shoot(pitch, yaw, v0=self.args_v0(), mech_noise=True)
                elif self.pending is None and time.time() >= self.next_auto_t:
                    self._auto_shot()

                for _ in range(4):
                    self._on_events(self.world.step())

                self._draw_camera()
                p.addUserDebugText(f"{self.mode} | {self.total_points} pts | {self._hitrate()}",
                                   [0, 0, 2.0], [1, 1, 0.4], replaceItemUniqueId=self._hud)
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break
                time.sleep(1 / 120)
        finally:
            cv2.destroyAllWindows()
            self.world.disconnect()

    def _reset(self):
        self.world.reset_balls()
        self.planner = ShotPlanner(self.cfg)
        self.total_points = self.n_shots = self.n_hits = 0
        self.pending = None
        self.last_line = "reset"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--perception", choices=["truth", "yolo"], default="truth",
                    help="自動模式的感知來源：truth=幾何真值(穩)、yolo=真模型(有域差)")
    ap.add_argument("--weights", default="models/grid_holes_yolo11n.pt")
    ap.add_argument("--device", default="0")
    ap.add_argument("--no-drag", action="store_true", help="關閉空氣阻力")
    ap.add_argument("--no-spin", action="store_true", help="關閉球旋轉/Magnus 力")
    ap.add_argument("--glare", type=float, default=0.0, help="鏡頭眩光強度 0~1（模擬強光過曝）")
    args = ap.parse_args()
    TeleopSim(args).run()


if __name__ == "__main__":
    main()
