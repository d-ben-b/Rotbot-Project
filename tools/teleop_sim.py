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
    python -m tools.teleop_sim --auto --quit-when-done --record runs/video/sim.mp4   # 自動錄一整場
"""
from __future__ import annotations

import argparse
import math
import time

import cv2
import numpy as np

# 允許直接以檔案路徑執行（python tools/teleop_sim.py）：把專案根目錄補進 sys.path
if __package__ in (None, ""):
    import os
    import sys
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rotbot.config import CameraConfig, load_config
from rotbot.decision import ballistics
from rotbot.decision.calibration import AdaptiveCalibrator
from rotbot.decision.strategy import ShotPlanner, plan_next_shot
from rotbot.perception.localize import HoleAim, localize_holes
from rotbot.sim import pybullet_camera as pcam
from rotbot.sim.pybullet_world import PyBulletWorld
from rotbot.sim.robot_spec import load_robot_spec

C_TARGET = (60, 220, 235)
C_TEXT = (240, 240, 240)
REC_FPS = 30                # 錄影：每圈 = 模擬時間 1/REC_FPS s


def _scaled_cam(cam: CameraConfig, s: float) -> CameraConfig:
    return CameraConfig(fx=cam.fx * s, fy=cam.fy * s, cx=cam.cx * s, cy=cam.cy * s,
                        width=int(cam.width * s), height=int(cam.height * s))


def _hole_int_index(group: str, idx: int) -> int:
    return idx if group == "main" else 9 + idx


def hole_overlays(cfg, eye, forward, cam):
    """對目前相機姿態，算每個可見孔的：投影橢圓(型變)、距離、方位/俯仰、傾角。

    型變由「圓在透視下投影成的橢圓」短/長軸比反映（1.0=正圓、越小越斜）；
    傾角=孔面法向與視線夾角。與感知用的是同一套投影幾何。
    """
    K = cam.matrix()
    W, H = cam.width, cam.height
    r_, d_, f_, _ = pcam._cam_axes(np.asarray(forward, float))
    R = np.column_stack([r_, d_, f_])
    eye = np.asarray(eye, float)
    board_n = np.array([-1.0, 0.0, 0.0])                 # 網板面向 -X（朝機器人）
    th = np.linspace(0, 2 * np.pi, 40, endpoint=False)
    out = []
    for (g, i, dia, C) in cfg.hole_centers():
        cw = np.array([cfg.field.dist_x, -(cfg.field.grid_cx + float(C[0])),
                       cfg.field.grid_cy + float(C[1])])
        pc = R.T @ (cw - eye)
        if pc[2] <= 0:
            continue
        cpx = K @ (pc / pc[2])
        u, v = float(cpx[0]), float(cpx[1])
        if not (0 <= u < W and 0 <= v < H):
            continue
        rim = cw + (dia / 2) * np.stack([np.zeros_like(th), -np.cos(th), np.sin(th)], 1)
        pcs = (R.T @ (rim - eye).T).T
        pcs = pcs[pcs[:, 2] > 0]
        ell, ratio = None, 1.0
        if len(pcs) >= 5:
            rpx = (K @ (pcs / pcs[:, 2:3]).T).T[:, :2].astype(np.float32)
            try:
                ell = cv2.fitEllipse(rpx)
                d1, d2 = ell[1]
                ratio = float(min(d1, d2) / max(d1, d2)) if max(d1, d2) > 0 else 1.0
            except cv2.error:
                pass
        dist = float(np.linalg.norm(cw - eye))
        los = (cw - eye) / (dist + 1e-9)
        out.append(dict(
            g=g, i=i, is_bonus=(g == "bonus"), center=(u, v), ell=ell, ratio=ratio,
            dist=dist, bearing=float(np.degrees(np.arctan2(pc[0], pc[2]))),
            elev=float(np.degrees(np.arctan2(-pc[1], pc[2]))),
            tilt=float(np.degrees(np.arccos(np.clip(abs(board_n @ los), 0, 1))))))
    return out


class TeleopSim:
    def __init__(self, args):
        self.args = args
        self.cfg = load_config()
        self.robot = load_robot_spec()
        self.world = PyBulletWorld(self.cfg, self.robot, gui=True,
                                   drag=not args.no_drag, spin=not args.no_spin)
        # 3D 視角：機器人在前景、網框在後方，看得到球的飛行（滑鼠可再調整）
        self.world.p.resetDebugVisualizerCamera(4.5, -60, -12, [3.2, 0.0, 1.0],
                                                physicsClientId=self.world.client)
        self.glare = args.glare
        self.show_data = not getattr(args, "no_data", False)
        self.disp_cam = _scaled_cam(self.cfg.camera, 0.5)
        self.K = self.cfg.camera.matrix()

        self.mode = "auto" if getattr(args, "auto", False) else "manual"
        self.detector = None
        if args.perception == "yolo":
            from rotbot.perception.detector import YoloDetector
            self.detector = YoloDetector(args.weights, device=args.device)

        # 決策 / 校正（自動模式用）
        self.planner = ShotPlanner(self.cfg)
        jp, jy = ballistics.estimate_sensitivity(
            ballistics.model_shoot(self.cfg), (0.0, self.cfg.field.grid_cy), self.cfg)
        self.calib = AdaptiveCalibrator(self.cfg.calibration, jp, jy, n_cells=12)
        self.total_points = 0
        self.n_shots = 0
        self.n_hits = 0
        self.pending = None            # 自動模式：飛行中那球的 (target dict, hidx, tz, ty)
        self.next_auto_t = 0.0
        self.auto_aim = None           # 自動模式最後一球的 (pitch, yaw) rad；轉塔畫在這個角度
        self._done_t = None            # --quit-when-done：最後一球落地的時間
        self.last_line = "manual mode - aim with sliders, press SHOOT"

        self._make_controls()

        # 錄影：逐格渲染。每圈 = 模擬時間 1/30 s，3D 視角與相機各存一張，
        # 所以影片固定 30 fps、兩份同步、球速是真實速度，不受電腦快慢影響；代價是錄影時畫面變慢。
        # （PyBullet 內建的 MP4 記錄依 GUI 重畫次數存檔，重畫速度一變影片就不同步，已不用。）
        self.record = getattr(args, "record", None)
        self.t_sim = 0.0
        self._writers = {}
        if self.record:
            import os as _os
            _os.makedirs(_os.path.dirname(_os.path.abspath(self.record)), exist_ok=True)
            base = self.record[:-4] if self.record.lower().endswith(".mp4") else self.record
            self._rec_paths = (self.record, base + "_camera.mp4", base + "_both.mp4")

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
        # 按鈕讀值是累計按下次數，初始值是 1（不是 0）；先讀一次，第一圈才不會誤判成按下
        self._c_shoot = p.readUserDebugParameter(self.b_shoot)
        self._c_mode = p.readUserDebugParameter(self.b_mode)
        self._c_reset = p.readUserDebugParameter(self.b_reset)
        self._c_relight = p.readUserDebugParameter(self.b_relight)
        self._hud = p.addUserDebugText("", [0, 0, 2.0], [1, 1, 1])
        self._hud_text = ""

    def _clicked(self, pid, cache_attr) -> bool:
        v = self.world.p.readUserDebugParameter(pid)
        prev = getattr(self, cache_attr)
        setattr(self, cache_attr, v)
        return v > prev

    # ── 感知（自動模式）─────────────────────────────────────────
    def _survey_pose(self):
        """掃視姿態：自動挑「讓 12 孔都完整入框」的瞄準高度（掃描 min-margin 最大者）。"""
        if getattr(self, "_survey_cache", None) is not None:
            return self._survey_cache
        K = self.cfg.camera.matrix()
        W, H = self.cfg.camera.width, self.cfg.camera.height
        holes = [(self.cfg.field.grid_cx + float(C[0]), self.cfg.field.grid_cy + float(C[1]),
                  float(d) / 2) for (_g, _i, d, C) in self.cfg.hole_centers()]

        def pose(az):
            bc = np.array([self.cfg.field.dist_x, -self.cfg.field.grid_cx, az])
            fwd = bc - self.world.pivot
            fwd = fwd / np.linalg.norm(fwd)
            return self.world.pivot + fwd * 0.30, fwd   # 相機在槍口，避免拍到自己的發射器

        def min_margin(az):
            eye, fwd = pose(az)
            r_, d_, f_, _ = pcam._cam_axes(fwd)
            R = np.column_stack([r_, d_, f_])
            m = 1e9
            for (hz, hy, rr) in holes:
                pc = R.T @ (np.array([self.cfg.field.dist_x, -hz, hy]) - eye)
                u = K @ (pc / pc[2])
                rpx = K[0, 0] * rr / pc[2]
                m = min(m, u[1] - rpx, H - (u[1] + rpx), u[0] - rpx, W - (u[0] + rpx))
            return m

        azs = np.linspace(self.cfg.field.grid_cy - 0.1, self.cfg.field.grid_cy + 0.4, 26)
        best = max(azs, key=min_margin)
        self._survey_cache = pose(best)
        return self._survey_cache

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
        self.auto_aim = (pitch, yaw)
        self.world.set_turret(pitch, yaw)
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
                self.next_auto_t = self._now() + 0.4
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
        # 顯示用：GPU 渲染、不加感測器雜訊（雜訊每張約 28 ms）；感知用的影像在 _perceive 另外渲染
        img = pcam.render(eye, forward, self.disp_cam, self.world.client, glare=self.glare,
                          noise=0.0, gpu=True)
        cv2.rectangle(img, (0, 0), (img.shape[1], 34), (12, 12, 14), -1)
        cv2.putText(img, f"Robot camera [{self.mode}]", (12, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, C_TEXT, 2, cv2.LINE_AA)
        cv2.putText(img, self.last_line, (12, img.shape[0] - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, C_TARGET, 2, cv2.LINE_AA)
        ov = hole_overlays(self.cfg, eye, forward, self.disp_cam)
        self._draw_overlays(img, ov)
        if self.show_data:
            self._draw_data_window(ov)
        cv2.imshow("Robot camera", img)
        return img

    # ── 錄影 ──────────────────────────────────────────────────
    def _now(self) -> float:
        """錄影時用模擬時間（逐格），否則用實際時間。"""
        return self.t_sim if self.record else time.time()

    def _write(self, path, img):
        """寫一張到 path；尺寸固定為第一張（GUI 視窗被拉大小時縮放）。回傳實際寫入的影像。"""
        if path not in self._writers:
            size = (img.shape[1], img.shape[0])
            self._writers[path] = (cv2.VideoWriter(
                path, cv2.VideoWriter_fourcc(*"mp4v"), REC_FPS, size), size)
            print(f"[錄影] → {path}")
        w, size = self._writers[path]
        if (img.shape[1], img.shape[0]) != size:
            img = cv2.resize(img, size)
        w.write(img)
        return img

    def _record_frame(self, cam_img):
        """存一格：3D（GUI 目前的視角重新渲染，滑鼠轉視角也會錄到）、相機、左右並排。"""
        p = self.world.p
        W, H, view, proj = p.getDebugVisualizerCamera(physicsClientId=self.world.client)[:4]
        _w, _h, rgba, _d, _s = p.getCameraImage(W, H, view, proj,
                                                renderer=p.ER_BULLET_HARDWARE_OPENGL,
                                                physicsClientId=self.world.client)
        img = np.reshape(np.asarray(rgba, np.uint8), (_h, _w, 4))[:, :, 2::-1].copy()
        hud = f"PyBullet simulation | {self.mode} | {self.total_points} pts | {self._hitrate()}"
        cv2.putText(img, hud, (14, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (20, 20, 20), 4, cv2.LINE_AA)
        cv2.putText(img, hud, (14, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, C_TARGET, 2, cv2.LINE_AA)
        path_3d, path_cam, path_both = self._rec_paths
        img = self._write(path_3d, img)
        self._write(path_cam, cam_img)
        h = img.shape[0]
        cam = cv2.resize(cam_img, (round(cam_img.shape[1] * h / cam_img.shape[0]), h))
        self._write(path_both, np.hstack([img, cam]))

    def _draw_overlays(self, img, ov):
        """在相機影像上，為每個孔畫投影橢圓 + 簡短標籤（亮孔紅、加分橙、一般青）。"""
        lit = self.world.lit_idx
        for o in ov:
            is_lit = o["is_bonus"] and o["i"] == lit
            col = (60, 60, 235) if is_lit else ((60, 170, 235) if o["is_bonus"]
                                                else (200, 200, 120))
            if o["ell"] is not None:
                cv2.ellipse(img, o["ell"], col, 1, cv2.LINE_AA)
            cx, cy = int(o["center"][0]), int(o["center"][1])
            cv2.putText(img, f"{o['g'][0]}{o['i']} {o['dist']:.2f}m", (cx - 24, cy + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1, cv2.LINE_AA)

    def _draw_data_window(self, ov):
        """額外視窗：每孔的距離/方位/俯仰/傾角/型變(短長軸比)表格，避免疊在畫面上。"""
        rows = sorted(ov, key=lambda o: (o["g"], o["i"]))
        W, rh = 470, 22
        panel = np.full((rh * (len(rows) + 2) + 8, W, 3), (18, 18, 20), np.uint8)
        F = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(panel, "ID    dist   bear   elev   tilt  defo", (10, 18), F, 0.5,
                    (180, 180, 185), 1, cv2.LINE_AA)
        lit = self.world.lit_idx
        for k, o in enumerate(rows):
            is_lit = o["is_bonus"] and o["i"] == lit
            col = (90, 90, 240) if is_lit else ((90, 190, 240) if o["is_bonus"]
                                                else (220, 220, 160))
            tag = f"{o['g'][0]}{o['i']}" + ("*" if is_lit else " ")
            txt = (f"{tag:5s} {o['dist']:4.2f}m {o['bearing']:+5.1f} {o['elev']:+5.1f} "
                   f"{o['tilt']:4.0f}d {o['ratio']:4.2f}")
            cv2.putText(panel, txt, (10, 18 + (k + 1) * rh), F, 0.48, col, 1, cv2.LINE_AA)
        cv2.putText(panel, "defo=minor/major (1=circle)   *=lit bonus",
                    (10, 18 + (len(rows) + 1) * rh), F, 0.42, (130, 130, 135), 1, cv2.LINE_AA)
        cv2.imshow("Detections", panel)

    # ── 主迴圈 ───────────────────────────────────────────────
    def run(self):
        p = self.world.p
        t_phys = time.time()
        try:
            while True:
                if self._clicked(self.b_mode, "_c_mode"):
                    self.mode = "auto" if self.mode == "manual" else "manual"
                    self.next_auto_t = self._now() + 0.3
                if self._clicked(self.b_reset, "_c_reset"):
                    self._reset()
                if self._clicked(self.b_relight, "_c_relight"):
                    self.world.set_lit(int(np.random.randint(0, 3)))

                pitch = math.radians(p.readUserDebugParameter(self.s_pitch))
                yaw = math.radians(p.readUserDebugParameter(self.s_yaw))
                if self.mode == "manual" or self.auto_aim is None:
                    self.world.set_turret(pitch, yaw)

                if self.mode == "manual":
                    if self._clicked(self.b_shoot, "_c_shoot"):
                        self.world.shoot(pitch, yaw, v0=self.args_v0(), mech_noise=True)
                elif self.pending is None and self._now() >= self.next_auto_t:
                    self._auto_shot()
                if (getattr(self.args, "quit_when_done", False) and self.mode == "auto"
                        and self.planner.remaining <= 0 and self.pending is None):
                    if self._done_t is None:
                        self._done_t = self._now()
                    elif self._now() - self._done_t > 2.0:    # 最後一球落地後再停 2 s
                        break

                if self.record:
                    # 錄影：每圈固定走 1/30 s 的模擬時間
                    n = round(1.0 / REC_FPS / self.world.timestep)
                    self.t_sim += n * self.world.timestep
                else:
                    # 物理依實際經過的時間推進（球速與真實時間一致，不受迴圈快慢影響）；
                    # 一圈最多補 0.25 s，自動模式感知卡住時不會一次跳太多
                    now = time.time()
                    n = min(int((now - t_phys) / self.world.timestep), 60)
                    t_phys = now if n == 60 else t_phys + n * self.world.timestep
                for _ in range(n):
                    self._on_events(self.world.step())

                cam_img = self._draw_camera()
                if self.record:
                    self._record_frame(cam_img)
                hud = f"{self.mode} | {self.total_points} pts | {self._hitrate()}"
                if hud != self._hud_text:                  # 每次呼叫約 4 ms，只在內容改變時更新
                    p.addUserDebugText(hud, [0, 0, 2.0], [1, 1, 0.4],
                                       replaceItemUniqueId=self._hud)
                    self._hud_text = hud
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break
                if not self.record:
                    time.sleep(1 / 120)
        finally:
            for w, _size in self._writers.values():
                w.release()
            if self._writers:
                print(f"[錄影] 已存檔：模擬時間 {self.t_sim:.1f} s")
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
    ap.add_argument("--record", default=None,
                    help="錄影到此 .mp4（3D 視角）；另存 *_camera.mp4（機器人相機）、*_both.mp4（左右並排）。"
                         "逐格渲染、30 fps，錄影時畫面會比實際慢")
    ap.add_argument("--auto", action="store_true", help="一開始就是自動模式（錄影時不用按 TOGGLE）")
    ap.add_argument("--quit-when-done", action="store_true",
                    help="自動模式投完最後一球、落地 2 s 後自動結束（自動錄影用）")
    ap.add_argument("--no-data", action="store_true",
                    help="關閉額外的偵測資料視窗（距離/傾角/型變表）")
    args = ap.parse_args()
    TeleopSim(args).run()


if __name__ == "__main__":
    main()
