# -*- coding: utf-8 -*-
"""PyBullet 物理沙盒 —— 機器人 + 網板 + 球的真實剛體世界（擬真版）。

沒有硬體時的「真實物理世界」替身：
  * 物理：球為剛體、受重力 + 平方空氣阻力 + 旋轉 Magnus 力；會穿孔或撞板反彈；孔後有網子接球。
  * 視覺：木紋網板 + 外框 + Truss 立柱 + 賽場地板 + 光影；孔後深色網子 → 鏡頭裡孔變暗（縮小 YOLO 域差）。
  * 計分：沿用解析法（球心於 x=dist_x 平面是否落在孔容許值內），與 sim.world/closed_loop 一致。

座標（world, Z 上）：X 前(朝板)、Y 左、Z 上。發射點系(x前,y上,z右) 對應 (X, Z, -Y)。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ..config import Config
from . import textures
from .board_mesh import board_extents, board_obj_path
from .robot_spec import RobotSpec

_C_DRUM = (0.80, 0.45, 0.15, 1.0)
_C_FLYWHEEL = (0.55, 0.35, 0.75, 1.0)
_C_BALL = (0.95, 0.95, 0.95, 1.0)
_C_LIT = (0.95, 0.18, 0.18, 0.60)      # 半透明紅：亮起的指定得分格（官方 Q33 確認為紅 LED）
_WHITE = (1, 1, 1, 1)


@dataclass
class LandEvent:
    group: str
    idx: int
    hit: bool
    points: int
    error_m: float                       # 落點徑向誤差 (m)，NaN=沒落到板平面
    landing: tuple[float, float] | None  # (橫向 z, 高度 y)，None=未達平面
    target: dict | None                  # 投球時附帶的目標資訊（供 HUD）


class PyBulletWorld:
    """機器人 + 網板 + 球 的物理世界（擬真）。"""

    def __init__(self, cfg: Config, robot: RobotSpec, gui: bool = True,
                 drag: bool = True, spin: bool = True, magnus_k: float = 5e-5,
                 rng: np.random.Generator | None = None, max_flight_s: float = 2.5):
        import pybullet as p
        import pybullet_data
        self.p = p
        self.cfg = cfg
        self.robot = robot
        self.drag = drag
        self.spin = spin
        self.magnus_k = magnus_k
        self.rng = rng if rng is not None else np.random.default_rng(cfg.sim.seed)
        self.timestep = 1.0 / 240.0
        self.max_flight_steps = int(max_flight_s / self.timestep)
        self.light_dir = [-1.0, 0.4, 1.2]

        self.client = p.connect(p.GUI if gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.client)
        p.setGravity(0, 0, -cfg.air.g, physicsClientId=self.client)
        p.setTimeStep(self.timestep, physicsClientId=self.client)
        if gui:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=self.client)
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1, physicsClientId=self.client)
            p.resetDebugVisualizerCamera(1.9, 50, -18, [0.05, 0.0, 0.40],
                                         physicsClientId=self.client)

        self.tex = {k: p.loadTexture(v, physicsClientId=self.client)
                    for k, v in textures.paths().items()}
        self._build_scene()
        self._build_robot()

        self.launch_h = robot.launcher.muzzle_height
        self.pivot = np.array([robot.launcher.pivot_forward, 0.0, self.launch_h])
        self.balls: list[dict] = []
        self.lit_idx = 0
        self._lit_body = None
        self.set_lit(0)
        self.set_turret(math.radians(25.0), 0.0)

    # ── 世界建置 ──────────────────────────────────────────────
    def _tex(self, body, tid, rgba=_WHITE):
        self.p.changeVisualShape(body, -1, textureUniqueId=tid, rgbaColor=rgba,
                                 physicsClientId=self.client)

    def _box(self, half, pos, rgba, tid=None, mass=0.0, collision=True):
        p = self.p
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half,
                                     physicsClientId=self.client) if collision else -1
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=rgba,
                                  physicsClientId=self.client)
        b = p.createMultiBody(mass, col, vis, pos, physicsClientId=self.client)
        if tid is not None:
            self._tex(b, tid)
        return b

    def _build_scene(self):
        p = self.p
        f = self.cfg.field
        plane = p.loadURDF("plane.urdf", physicsClientId=self.client)
        self._tex(plane, self.tex["floor"])

        # 網板（程序化有洞平板）+ 木紋
        obj = str(board_obj_path(self.cfg))
        col = p.createCollisionShape(p.GEOM_MESH, fileName=obj,
                                     flags=p.GEOM_FORCE_CONCAVE_TRIMESH,
                                     physicsClientId=self.client)
        vis = p.createVisualShape(p.GEOM_MESH, fileName=obj, rgbaColor=_WHITE,
                                  physicsClientId=self.client)
        self.board_id = p.createMultiBody(0, col, vis, [0, 0, 0], physicsClientId=self.client)
        self._tex(self.board_id, self.tex["wood"])

        u0, u1, v0, v1 = board_extents(self.cfg)
        cy_u = -(f.grid_cx + 0.5 * (u0 + u1))         # 板中心 world Y
        cz = f.grid_cy + 0.5 * (v0 + v1)              # 板中心 world Z
        w2, h2 = 0.5 * (u1 - u0), 0.5 * (v1 - v0)
        fr = 0.045
        # 外框（木）：上下橫樑 + 左右立樑
        self._box([0.04, w2 + fr, fr], [f.dist_x, cy_u, f.grid_cy + v1], _WHITE, self.tex["wood"])
        self._box([0.04, w2 + fr, fr], [f.dist_x, cy_u, f.grid_cy + v0], _WHITE, self.tex["wood"])
        self._box([0.04, fr, h2 + fr], [f.dist_x, -(f.grid_cx + u1), cz], _WHITE, self.tex["wood"])
        self._box([0.04, fr, h2 + fr], [f.dist_x, -(f.grid_cx + u0), cz], _WHITE, self.tex["wood"])
        # 孔後深色網子（接球 + 讓鏡頭裡孔變暗）
        self._box([0.012, w2, h2], [f.dist_x + 0.35, cy_u, cz], _WHITE, self.tex["net"])
        # Truss 立柱（金屬）+ 頂樑
        top = f.grid_cy + v1 + 0.12
        for sgn in (+1, -1):
            yp = -(f.grid_cx + (u1 + 0.12) * sgn)
            self._box([0.05, 0.05, top / 2], [f.dist_x + 0.18, yp, top / 2], _WHITE, self.tex["metal"])
        self._box([0.05, w2 + 0.2, 0.05], [f.dist_x + 0.18, cy_u, top], _WHITE, self.tex["metal"])

    def _mk_visual(self, shape, rgba, tid=None, **kw):
        p = self.p
        vis = p.createVisualShape(shape, rgbaColor=rgba, physicsClientId=self.client, **kw)
        b = p.createMultiBody(0, -1, vis, [0, 0, 0], physicsClientId=self.client)
        if tid is not None:
            self._tex(b, tid, rgba)
        return b

    def _build_robot(self):
        p = self.p
        ch = self.robot.chassis
        he = [ch.length / 2, ch.width / 2, ch.height / 2]
        self._box(he, [-0.05, 0, ch.height / 2], _WHITE, self.tex["metal"], collision=True)
        # 四輪（純視覺）
        self._wheels = []
        wr = ch.wheel_diameter / 2
        for sx in (0.16, -0.24):
            for sy in (ch.width / 2, -ch.width / 2):
                w = self._mk_visual(p.GEOM_CYLINDER, (0.1, 0.1, 0.1, 1), radius=wr, length=0.04)
                q = p.getQuaternionFromEuler([math.pi / 2, 0, 0])
                p.resetBasePositionAndOrientation(w, [sx, sy, wr], q, physicsClientId=self.client)
        mag = self.robot.magazine
        self._drum = self._mk_visual(p.GEOM_CYLINDER, _C_DRUM,
                                     radius=mag.drum_diameter / 2, length=mag.drum_length)
        self._barrel = self._mk_visual(p.GEOM_CYLINDER, _WHITE, self.tex["metal"],
                                       radius=0.03, length=0.26)
        fw_r = self.robot.launcher.flywheel_diameter / 2
        self._fw = [self._mk_visual(p.GEOM_CYLINDER, _C_FLYWHEEL, radius=fw_r, length=0.03),
                    self._mk_visual(p.GEOM_CYLINDER, _C_FLYWHEEL, radius=fw_r, length=0.03)]

    # ── 幾何工具 ──────────────────────────────────────────────
    def aim_dir(self, pitch: float, yaw: float) -> np.ndarray:
        cp, sp = math.cos(pitch), math.sin(pitch)
        return np.array([cp * math.cos(yaw), -cp * math.sin(yaw), sp])

    def _quat_z_to(self, d: np.ndarray):
        d = d / (np.linalg.norm(d) + 1e-12)
        z = np.array([0.0, 0.0, 1.0])
        axis = np.cross(z, d)
        s = np.linalg.norm(axis)
        if s < 1e-8:
            return [0.0, 0.0, 0.0, 1.0] if d[2] > 0 else [1.0, 0.0, 0.0, 0.0]
        axis = axis / s
        ang = math.acos(float(np.clip(np.dot(z, d), -1, 1)))
        sh = math.sin(ang / 2)
        return [axis[0] * sh, axis[1] * sh, axis[2] * sh, math.cos(ang / 2)]

    def muzzle(self, pitch: float, yaw: float) -> np.ndarray:
        return self.pivot + self.aim_dir(pitch, yaw) * 0.30

    def _right(self, d: np.ndarray) -> np.ndarray:
        r = np.cross(d, [0, 0, 1])
        return r / (np.linalg.norm(r) + 1e-9)

    # ── 轉塔 / 亮孔 ───────────────────────────────────────────
    def set_turret(self, pitch: float, yaw: float):
        p = self.p
        d = self.aim_dir(pitch, yaw)
        q = self._quat_z_to(d)
        right = self._right(d)
        p.resetBasePositionAndOrientation(self._barrel, (self.pivot + d * 0.13).tolist(), q,
                                          physicsClientId=self.client)
        p.resetBasePositionAndOrientation(self._drum, (self.pivot - d * 0.18).tolist(), q,
                                          physicsClientId=self.client)
        q_fw = self._quat_z_to(right)
        for sgn, body in zip((+1, -1), self._fw):
            pos = self.pivot + d * 0.06 + right * (0.06 * sgn)
            p.resetBasePositionAndOrientation(body, pos.tolist(), q_fw,
                                              physicsClientId=self.client)
        self._pitch, self._yaw = pitch, yaw

    def set_lit(self, idx: int):
        p = self.p
        self.lit_idx = int(idx)
        z, y = self._hole_world_zy("bonus", self.lit_idx)
        pos = [self.cfg.field.dist_x - 0.002, -z, y]
        q = self._quat_z_to(np.array([-1.0, 0.0, 0.0]))
        if self._lit_body is None:
            self._lit_body = self._mk_visual(
                p.GEOM_CYLINDER, _C_LIT,
                radius=self.cfg.field.bonus_d / 2 * 0.9, length=0.004)
        p.resetBasePositionAndOrientation(self._lit_body, pos, q, physicsClientId=self.client)

    # ── 投球 / 物理步進 ───────────────────────────────────────
    def _hole_world_zy(self, group: str, idx: int) -> tuple[float, float]:
        for (g, i, _d, C) in self.cfg.hole_centers():
            if g == group and i == idx:
                return self.cfg.field.grid_cx + float(C[0]), self.cfg.field.grid_cy + float(C[1])
        raise KeyError((group, idx))

    def shoot(self, pitch: float, yaw: float, v0: float | None = None,
              mech_noise: bool = False, target: dict | None = None):
        p = self.p
        cfg = self.cfg
        v0 = cfg.launch.v0_real if v0 is None else v0
        if mech_noise:
            m = cfg.mechanism
            pitch = pitch + math.radians(m.bias_pitch + self.rng.normal(0, m.jitter_deg))
            yaw = yaw + math.radians(m.bias_yaw + self.rng.normal(0, m.jitter_deg))
            v0 = v0 + self.rng.normal(0, cfg.launch.v0_jitter)
        self.set_turret(pitch, yaw)
        d = self.aim_dir(pitch, yaw)
        spawn = self.muzzle(pitch, yaw)
        r = cfg.ball.diameter / 2
        col = p.createCollisionShape(p.GEOM_SPHERE, radius=r, physicsClientId=self.client)
        vis = p.createVisualShape(p.GEOM_SPHERE, radius=r, rgbaColor=_C_BALL,
                                  physicsClientId=self.client)
        bid = p.createMultiBody(cfg.ball.mass, col, vis, spawn.tolist(),
                                physicsClientId=self.client)
        self._tex(bid, self.tex["baseball"])
        spin_vec = np.zeros(3)
        if self.spin:
            spin_vec = self._right(d) * 80.0          # 飛輪賦予的後旋 (rad/s，含打滑估值)
        p.resetBaseVelocity(bid, linearVelocity=(d * v0).tolist(),
                            angularVelocity=spin_vec.tolist(), physicsClientId=self.client)
        p.changeDynamics(bid, -1, restitution=0.4, lateralFriction=0.4,
                         physicsClientId=self.client)
        self.balls.append(dict(id=bid, prev=spawn.copy(), scored=False, spin=spin_vec,
                               best_gap=float("inf"), best_zy=None, steps=0, target=target))
        return bid

    def _aero(self, b):
        """對飛行中的球施加空氣阻力 + Magnus 力。"""
        p = self.p
        v, _ = p.getBaseVelocity(b["id"], physicsClientId=self.client)
        v = np.asarray(v)
        s = float(np.linalg.norm(v))
        if s < 1e-6:
            return
        force = np.zeros(3)
        if self.drag:
            force = force + (-self.cfg.k_drag * s) * v          # F = -k_drag |v| v
        if self.spin:
            force = force + self.magnus_k * np.cross(b["spin"], v)
        p.applyExternalForce(b["id"], -1, force.tolist(), [0, 0, 0], p.WORLD_FRAME,
                             physicsClientId=self.client)

    def _score(self, z: float, y: float, crossed: bool) -> LandEvent:
        cfg = self.cfg
        best = None
        for (g, i, diam, C) in cfg.hole_centers():
            hz = cfg.field.grid_cx + float(C[0])
            hy = cfg.field.grid_cy + float(C[1])
            err = math.hypot(z - hz, y - hy)
            tol = (float(diam) - cfg.ball.diameter) / 2
            if best is None or err < best[3]:
                best = (g, i, tol, err)
        g, i, tol, err = best
        is_lit = (g == "bonus" and i == self.lit_idx)
        within = crossed and err <= tol
        pts = (4 if is_lit else 2 if g == "main" else 0) if within else 0
        hit = within and pts > 0
        return LandEvent(g, i, hit, pts, err, (z, y), None)

    def step(self) -> list[LandEvent]:
        p = self.p
        events: list[LandEvent] = []
        for b in self.balls:
            if not b["scored"]:
                self._aero(b)
        p.stepSimulation(physicsClientId=self.client)
        dist_x = self.cfg.field.dist_x
        for b in self.balls:
            if b["scored"]:
                continue
            b["steps"] += 1
            pos, _ = p.getBasePositionAndOrientation(b["id"], physicsClientId=self.client)
            pos = np.asarray(pos)
            gap = abs(pos[0] - dist_x)
            if gap < b["best_gap"]:
                b["best_gap"] = gap
                b["best_zy"] = (-pos[1], pos[2])
            prev = b["prev"]
            if prev[0] < dist_x <= pos[0]:
                fr = (dist_x - prev[0]) / (pos[0] - prev[0] + 1e-12)
                z = -(prev[1] + fr * (pos[1] - prev[1]))
                y = prev[2] + fr * (pos[2] - prev[2])
                ev = self._score(z, y, crossed=True)
                ev.target = b["target"]
                events.append(ev)
                b["scored"] = True
            elif b["steps"] > self.max_flight_steps:
                if b["best_zy"] is not None and b["best_gap"] < 0.25:
                    ev = self._score(b["best_zy"][0], b["best_zy"][1], crossed=False)
                    ev.landing = b["best_zy"]
                else:
                    ev = LandEvent("none", -1, False, 0, float("nan"), None, b["target"])
                ev.target = b["target"]
                events.append(ev)
                b["scored"] = True
            b["prev"] = pos
        return events

    def reset_balls(self):
        for b in self.balls:
            self.p.removeBody(b["id"], physicsClientId=self.client)
        self.balls.clear()

    def disconnect(self):
        self.p.disconnect(physicsClientId=self.client)
