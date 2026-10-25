# -*- coding: utf-8 -*-
"""6m 重新評估 —— 一鍵重跑所有誤差與假設（全部為模擬/合成評估，非實機成績）。

各節：
  A 彈道：每孔所需仰角（真實 vs 腦內模型 vs 無阻力）、飛行時間、角度容許值、各參數靈敏度
  B 初速掃描：v0 對可達性、v0 抖動放大倍率、整場命中率的影響 → 飛輪轉速需求
  C 閉迴路：校正 OFF/ON/賽前暖機、逐球收斂、穩健度、腦內模型比較、2.5m 對照
  D 誤差預算：機構抖動 / v0 抖動 / 感知雜訊 / 落點量測雜訊 各自對命中率的影響 → 硬體規格
  E 感知幾何：完美偵測 + 像素雜訊下，整板 PnP 在 2.5m / 6m、兩種焦距的誤差
  F（選用 --weights）YOLO 端到端感知與 render 閉迴路

用法：
    python -m tools.reassess_6m                     # 全部（CPU 約數分鐘），結果寫 runs/reassess_6m/
    python -m tools.reassess_6m --quick             # 少量場次（煙霧測試）
    python -m tools.reassess_6m --weights models/grid_holes_yolo11n.pt
    python -m tools.reassess_6m --sections F --weights models/grid_holes_yolo11n.pt   # 只跑 YOLO 節
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

if __package__ in (None, ""):                       # 支援 python tools/reassess_6m.py
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from rotbot.config import CameraConfig, Config, load_config
from rotbot.decision import ballistics
from rotbot.perception.localize import localize_holes
from rotbot.sim.closed_loop import (_truth_holes, practice_calibration,
                                    run_closed_loop)
from rotbot.sim.perfect_detector import detections_from_scene
from rotbot.sim.scene import SceneGenerator
from rotbot.sim.world import BallisticWorld

REPO_ROOT = Path(__file__).resolve().parent.parent
OUT_DIR = REPO_ROOT / "runs" / "reassess_6m"


def cfg_with(cfg: Config, **sections) -> Config:
    """回傳改了部分欄位的設定副本，例：cfg_with(cfg, field=dict(dist_x=2.5))。"""
    kw = {name: dataclasses.replace(getattr(cfg, name), **chg) for name, chg in sections.items()}
    return dataclasses.replace(cfg, **kw)


def old_25m(cfg: Config) -> Config:
    """舊版假設（2026-09）：2.5m、網框中心 1.50m、v0 10 m/s、無阻力瞄準。"""
    return cfg_with(cfg, field=dict(dist_x=2.5, grid_cy=1.5),
                    launch=dict(v0_model=10.0, v0_real=9.6),
                    aim=dict(model="vacuum", cd_model=cfg.air.cd))


# ══════════════════════════ A 彈道 ══════════════════════════
def _traj_stats(v0, pitch, kd, g, r_target, dt=0.0005):
    """2D RK4 積分到水平距 r_target：回傳 (飛行時間 s, 撞擊速度 m/s, 撞擊航跡角 deg, 頂點高 m)。"""
    s = y = 0.0
    vs, vy = v0 * math.cos(pitch), v0 * math.sin(pitch)
    t, apex = 0.0, 0.0

    def acc(a, b):
        sp = math.hypot(a, b)
        return -kd * sp * a, -g - kd * sp * b

    while s < r_target and t < 3.0:
        a1 = acc(vs, vy)
        a2 = acc(vs + 0.5 * dt * a1[0], vy + 0.5 * dt * a1[1])
        a3 = acc(vs + 0.5 * dt * a2[0], vy + 0.5 * dt * a2[1])
        a4 = acc(vs + dt * a3[0], vy + dt * a3[1])
        s += dt * vs + dt * dt / 6 * (a1[0] + a2[0] + a3[0])
        y += dt * vy + dt * dt / 6 * (a1[1] + a2[1] + a3[1])
        vs += dt / 6 * (a1[0] + 2 * a2[0] + 2 * a3[0] + a4[0])
        vy += dt / 6 * (a1[1] + 2 * a2[1] + 2 * a3[1] + a4[1])
        t += dt
        apex = max(apex, y)
    return t, math.hypot(vs, vy), math.degrees(math.atan2(vy, vs)), apex


def section_a(cfg: Config) -> dict:
    g = cfg.air.g
    kd_true = cfg.k_drag / cfg.ball.mass
    truth_model = ballistics.DragAimModel(cfg.launch.v0_real, kd_true, g)
    robot = ballistics.aim_model(cfg)
    vac = ballistics.VacuumAimModel(cfg.launch.v0_model, g)
    world = BallisticWorld(cfg, np.random.default_rng(0))
    w_cd = BallisticWorld(cfg_with(cfg, air=dict(cd=cfg.air.cd + 0.05)), np.random.default_rng(0))
    w_far = BallisticWorld(cfg_with(cfg, field=dict(dist_x=cfg.field.dist_x + 0.10)),
                           np.random.default_rng(0))
    rows = []
    for (grp, idx, diam, z, y_abs) in _truth_holes(cfg):
        x, dy = cfg.field.dist_x, y_abs - cfg.field.launch_h
        r = math.hypot(x, z)
        yaw = math.atan2(z, x)
        p_true = truth_model.elevation(r, dy)
        p_robot = robot.elevation(r, dy)
        p_vac = vac.elevation(r, dy)
        row = dict(hole=f"{grp}#{idx}", z=round(z, 2), y=round(y_abs, 2), diam=diam,
                   reachable=p_true is not None)
        if p_true is not None:
            tof, v_imp, gamma, apex = _traj_stats(cfg.launch.v0_real, p_true, kd_true, g, r)
            h = math.radians(0.25)
            ya = world.shoot(p_true + h, yaw)[1]
            yb = world.shoot(p_true - h, yaw)[1]
            za = world.shoot(p_true, yaw + h)[0]
            zb = world.shoot(p_true, yaw - h)[0]
            dy_dp = (ya - yb) / 0.5 * 100                         # cm/deg
            dz_dyaw = (za - zb) / 0.5 * 100
            yv = world.shoot(p_true, yaw, v0=cfg.launch.v0_real + 0.1)[1]
            y0 = world.shoot(p_true, yaw)[1]
            tol = (diam - cfg.ball.diameter) / 2 * 100
            row.update(
                pitch_true_deg=round(math.degrees(p_true), 2),
                pitch_robot_deg=None if p_robot is None else round(math.degrees(p_robot), 2),
                model_err_deg=None if p_robot is None else round(math.degrees(p_robot - p_true), 2),
                pitch_vacuum_deg=None if p_vac is None else round(math.degrees(p_vac), 2),
                vacuum_err_deg=None if p_vac is None else round(math.degrees(p_vac - p_true), 2),
                tof_s=round(tof, 3), v_impact=round(v_imp, 1), impact_angle_deg=round(gamma, 1),
                apex_above_muzzle_m=round(apex, 2),
                dy_dpitch_cm_per_deg=round(dy_dp, 1), dz_dyaw_cm_per_deg=round(dz_dyaw, 1),
                dy_per_0p1mps_cm=round((yv - y0) * 100, 1),
                dy_per_cd0p05_cm=round((w_cd.shoot(p_true, yaw)[1] - y0) * 100, 1),
                dy_per_range10cm_cm=round((w_far.shoot(p_true, yaw)[1] - y0) * 100, 1),
                tol_cm=round(tol, 1),
                tol_pitch_deg=round(tol / abs(dy_dp), 2), tol_yaw_deg=round(tol / abs(dz_dyaw), 2),
            )
        rows.append(row)
    return dict(rows=rows)


# ══════════════════════════ 蒙地卡羅工具 ══════════════════════════
def mc(cfg: Config, n: int, seed: int = 7, practice: int = 0, correct: bool = True, **kw) -> dict:
    """跑 n 場，回傳總分/命中率/加分孔命中率與逐球誤差。"""
    rng = np.random.default_rng(seed)
    pts, hr, bhr, per_ball_err, per_ball_hit = [], [], [], [], []
    for _ in range(n):
        calib = (practice_calibration(cfg, rng, practice,
                                      v0_drift_frac=kw.get("v0_drift_frac", 0.0))
                 if practice > 0 else None)
        rep = run_closed_loop(cfg, rng, correct=correct, calib=calib, **kw)
        pts.append(rep.total_points)
        hr.append(rep.hit_rate)
        bhr.append(rep.bonus_hit_rate)
        per_ball_err.append([s.error_m * 100 for s in rep.shots])
        per_ball_hit.append([s.hit for s in rep.shots])
    L = max(len(e) for e in per_ball_err)
    E = np.full((n, L), np.nan)
    Hm = np.zeros((n, L))
    for k, (e, h) in enumerate(zip(per_ball_err, per_ball_hit)):
        E[k, :len(e)] = e
        Hm[k, :len(h)] = h
    return dict(points=float(np.mean(pts)), points_sd=float(np.std(pts)),
                hit=float(np.mean(hr)), bonus_hit=float(np.mean(bhr)), E=E, H=Hm)


def _fmt(r: dict) -> str:
    return f"{r['points']:5.1f} ± {r['points_sd']:4.1f} | {r['hit'] * 100:4.0f}% | {r['bonus_hit'] * 100:4.0f}%"


# ══════════════════════════ E 感知幾何 ══════════════════════════
def _cam(fx: float, w: int, h: int) -> CameraConfig:
    return CameraConfig(fx=fx, fy=fx, cx=w / 2, cy=h / 2, width=w, height=h)


def section_e(cfg: Config, n_scenes: int) -> dict:
    """完美偵測（rim 加像素雜訊 → bbox）→ localize_holes：統計每孔 (距離, 方位, 俯仰) 誤差。"""
    cams = {"webcam 1080p fx1400 (placeholder)": _cam(1400.0, 1920, 1080),
            "AR0234 + 6mm fx2000 (1920x1200)": _cam(2000.0, 1920, 1200)}
    ranges = {"2.5m (2.0-3.2)": (2.0, 3.2), "6m (5.5-6.5)": (5.5, 6.5)}
    out = []
    for cam_name, cam in cams.items():
        c = dataclasses.replace(cfg, camera=cam)
        K = cam.matrix()
        for rname, dr in ranges.items():
            for sigma in (0.5, 1.0, 2.0):
                rng = np.random.default_rng(11)
                gen = SceneGenerator(c, rng=np.random.default_rng(5), dist_range=dr)
                dd, bb, ee, px_main, px_bonus, fails = [], [], [], [], [], 0
                for _ in range(n_scenes):
                    s = gen.sample()
                    dets = detections_from_scene(s, noise_px=sigma, rng=rng)
                    est = localize_holes(dets, c, K)
                    if est.board_pose is None:
                        fails += 1
                        continue
                    truth = {(h.group, h.idx): h for h in s.holes}
                    for a in est.aims:
                        h = truth.get((a.group, a.idx))
                        if h is None or not h.visible:
                            continue
                        dd.append((a.distance - h.distance) * 100)
                        bb.append(a.bearing - h.bearing)
                        ee.append(a.elev - h.elev)
                    for h in s.holes:
                        if h.visible and h.ellipse is not None:
                            (px_bonus if h.is_bonus else px_main).append(max(h.ellipse[1]))
                rob = lambda x: float(1.4826 * np.median(np.abs(np.asarray(x) - np.median(x))))
                out.append(dict(camera=cam_name, range=rname, sigma_px=sigma, pnp_fail=fails,
                                n=len(dd),
                                dist_cm_med=float(np.median(np.abs(dd))),
                                dist_cm_p95=float(np.percentile(np.abs(dd), 95)),
                                dist_cm_sigma=rob(dd),
                                bearing_deg_med=float(np.median(np.abs(bb))),
                                bearing_deg_p95=float(np.percentile(np.abs(bb), 95)),
                                bearing_deg_sigma=rob(bb),
                                elev_deg_med=float(np.median(np.abs(ee))),
                                elev_deg_p95=float(np.percentile(np.abs(ee), 95)),
                                elev_deg_sigma=rob(ee),
                                main_px=float(np.median(px_main)) if px_main else float("nan"),
                                bonus_px=float(np.median(px_bonus)) if px_bonus else float("nan")))
    return dict(rows=out)



# ══════════════════════════ 各節輸出 ══════════════════════════
def _strip(r: dict) -> dict:
    return {k: v for k, v in r.items() if k not in ("E", "H")}


def report_e(cfg, p, res, n_scenes) -> dict:
    """E：感知幾何；回傳閉迴路用的感知誤差模型（B/C/D 都需要）。"""
    e = section_e(cfg, n_scenes)
    res["E_perception_geometry"] = e
    p("## E 感知幾何（完美偵測 + 像素雜訊 → 整板 PnP，每孔誤差）\n")
    p("| 相機 | 距離 | σpx | PnP失敗 | 距離 中位/p95 cm | 方位 中位/p95 ° | 俯仰 中位/p95 ° | Ø40 / Ø20 px |")
    p("|---|---|---|---|---|---|---|---|")
    for r in e["rows"]:
        p(f"| {r['camera']} | {r['range']} | {r['sigma_px']} | {r['pnp_fail']} | "
          f"{r['dist_cm_med']:.1f} / {r['dist_cm_p95']:.1f} | {r['bearing_deg_med']:.3f} / "
          f"{r['bearing_deg_p95']:.3f} | {r['elev_deg_med']:.3f} / {r['elev_deg_p95']:.3f} | "
          f"{r['main_px']:.0f} / {r['bonus_px']:.0f} |")
    base_e = next(r for r in e["rows"] if r["range"].startswith("6m") and r["sigma_px"] == 1.0
                  and "fx1400" in r["camera"])
    perc = dict(bias_d=base_e["dist_cm_sigma"] / 100,
                bias_ang=max(base_e["bearing_deg_sigma"], base_e["elev_deg_sigma"]),
                noise_d=0.01, noise_ang=0.1)
    res["perception_model"] = perc
    p(f"\n→ 閉迴路感知誤差模型（6m、fx1400、σ=1px 的穩健標準差）：系統偏差 距離 {perc['bias_d'] * 100:.1f} cm、"
      f"角度 {perc['bias_ang']:.3f}°；逐幀雜訊（假設）距離 1 cm、角度 0.1°\n")
    return perc


def report_a(cfg, p, res):
    a = section_a(cfg)
    res["A_ballistics"] = a
    p("## A 彈道（真實世界 v0_real、Cd 真值）\n")
    p("| 孔 | 高 m | 所需仰角° | 腦內模型誤差° | 無阻力誤差° | 飛行 s | 撞擊角° | ∂y/∂仰角 cm/° | ∂z/∂水平 cm/° | "
      "v0+0.1 → cm | Cd+0.05 → cm | 距離+10cm → cm | 容許 cm | 容許 仰角°/水平° |")
    p("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for r in a["rows"]:
        if not r["reachable"]:
            p(f"| {r['hole']} | {r['y']} | 打不到 | | | | | | | | | | | |")
            continue
        p(f"| {r['hole']} | {r['y']} | {r['pitch_true_deg']} | {r['model_err_deg']} | {r['vacuum_err_deg']} | "
          f"{r['tof_s']} | {r['impact_angle_deg']} | {r['dy_dpitch_cm_per_deg']} | {r['dz_dyaw_cm_per_deg']} | "
          f"{r['dy_per_0p1mps_cm']} | {r['dy_per_cd0p05_cm']} | {r['dy_per_range10cm_cm']} | {r['tol_cm']} | "
          f"{r['tol_pitch_deg']} / {r['tol_yaw_deg']} |")


def report_b(cfg, p, res, n, plan_kw):
    p("\n## B 初速掃描（v0_real = 0.96 × v0；校正 ON + 賽前練習 9 球）\n")
    p("| v0 m/s | 可達孔數 | 仰角範圍° | v0+0.1→加分排 cm | 總分 | 命中率 | 加分孔命中 |")
    p("|---|---|---|---|---|---|---|")
    res["B_v0_sweep"] = []
    for v0 in (10.0, 11.0, 12.0, 13.0, 14.0, 16.0, 18.0):
        c = cfg_with(cfg, launch=dict(v0_model=v0, v0_real=0.96 * v0))
        rows = section_a(c)["rows"]
        ok = [r for r in rows if r["reachable"]]
        bon = [r for r in ok if r["hole"].startswith("bonus")]
        pitches = [r["pitch_true_deg"] for r in ok]
        m = dict(points=float("nan"), points_sd=0.0, hit=float("nan"), bonus_hit=float("nan"))
        if any(r["hole"].startswith("main") for r in ok):   # 打不到的孔由策略自動略過
            try:
                m = mc(c, n, practice=9, **plan_kw)
            except ValueError:                              # 腦內模型連中心孔都判定打不到
                pass
        row = dict(v0=v0, reachable=len(ok), pitch_min=min(pitches) if pitches else None,
                   pitch_max=max(pitches) if pitches else None,
                   dy_bonus_0p1=bon[0]["dy_per_0p1mps_cm"] if bon else None,
                   points=m["points"], hit=m["hit"], bonus_hit=m["bonus_hit"])
        res["B_v0_sweep"].append(row)
        rng_s = f"{row['pitch_min']:.1f}~{row['pitch_max']:.1f}" if pitches else "-"
        p(f"| {v0:.0f} | {len(ok)}/12 | {rng_s} | {row['dy_bonus_0p1'] if bon else '-'} | "
          f"{m['points']:.1f} | {m['hit'] * 100:.0f}% | {m['bonus_hit'] * 100:.0f}% |")


def report_c(cfg, p, res, n, plan_kw, perc):
    p("\n## C1 校正與賽前暖機（總分 ± SD | 命中率 | 加分孔命中率）\n")
    p("| 條件 | 結果 |")
    p("|---|---|")
    c1 = {}
    for name, kw in [("校正 OFF", dict(correct=False)),
                     ("校正 ON（從零開始）", dict()),
                     ("校正 ON + 賽前練習 9 球", dict(practice=9)),
                     ("校正 ON + 賽前練習 18 球", dict(practice=18))]:
        r = mc(cfg, n, **kw, **plan_kw)
        c1[name] = r
        p(f"| {name} | {_fmt(r)} |")
    res["C1"] = {k: _strip(r) for k, r in c1.items()}

    p("\n## C2 逐球收斂（平均落點誤差 cm / 命中率）\n")
    p("| 條件 | 球 1–3 | 球 4–9 | 球 10–18 | 球 19–27 |")
    p("|---|---|---|---|---|")
    res["C2"] = {}
    for name in ("校正 OFF", "校正 ON（從零開始）", "校正 ON + 賽前練習 9 球"):
        r = c1[name]
        cells, rec = [], []
        for lo, hi in ((0, 3), (3, 9), (9, 18), (18, 27)):
            err = float(np.nanmean(r["E"][:, lo:hi]))
            hit = float(np.mean(r["H"][:, lo:hi]))
            cells.append(f"{err:.1f} / {hit * 100:.0f}%")
            rec.append((err, hit))
        res["C2"][name] = rec
        p(f"| {name} | " + " | ".join(cells) + " |")

    p("\n## C3 穩健度（校正 ON + 賽前練習 9 球）\n")
    p("| 條件 | 結果 |")
    p("|---|---|")
    res["C3"] = {}
    m0 = cfg.mechanism
    for name, c, kw in [
        ("基準", cfg, {}),
        ("2× 安裝偏差", cfg_with(cfg, mechanism=dict(bias_pitch=2 * m0.bias_pitch,
                                                    bias_yaw=2 * m0.bias_yaw)), {}),
        ("2× 機構抖動", cfg_with(cfg, mechanism=dict(jitter_deg=2 * m0.jitter_deg)), {}),
        ("2× v0 抖動", cfg_with(cfg, launch=dict(v0_jitter=2 * cfg.launch.v0_jitter)), {}),
        ("1.5× 空氣阻力", cfg_with(cfg, air=dict(cd=1.5 * cfg.air.cd)), {}),
        ("2× 感知系統偏差", cfg, dict(bias_d=2 * perc["bias_d"], bias_ang=2 * perc["bias_ang"])),
        ("v0 練習→比賽漂移 3%", cfg, dict(v0_drift_frac=0.03)),
    ]:
        kw2 = dict(plan_kw)
        kw2.update(kw)
        r = mc(c, n, practice=9, **kw2)
        res["C3"][name] = _strip(r)
        p(f"| {name} | {_fmt(r)} |")

    p("\n## C4 腦內瞄準模型（校正 ON + 賽前練習 9 球）\n")
    p("| 模型 | 結果 |")
    p("|---|---|")
    res["C4"] = {}
    for name, c in [("drag（含阻力，現行）", cfg),
                    ("vacuum（無阻力，舊版）", cfg_with(cfg, aim=dict(model="vacuum")))]:
        r = mc(c, n, practice=9, **plan_kw)
        res["C4"][name] = _strip(r)
        p(f"| {name} | {_fmt(r)} |")

    p("\n## C5 2.5m 舊假設 vs 6m（同一套程式、校正 ON，從零開始）\n")
    p("| 情境 | 結果 |")
    p("|---|---|")
    res["C5"] = {}
    for name, c in [("2.5m 舊假設（v0 10、無阻力、grid_cy 1.5）", old_25m(cfg)),
                    ("6m 但沿用舊假設（v0 10、無阻力）",
                     cfg_with(cfg, launch=dict(v0_model=10.0, v0_real=9.6),
                              aim=dict(model="vacuum"))),
                    ("6m 新設計點（v0 14、含阻力模型）", cfg)]:
        r = mc(c, n, **plan_kw)
        res["C5"][name] = _strip(r)
        p(f"| {name} | {_fmt(r)} |")


def report_d(cfg, p, res, n, plan_kw):
    p("\n## D 誤差預算（一次只改一項；校正 ON + 賽前練習 9 球；一般孔命中率 / 加分孔命中率）\n")
    res["D"] = {}
    sweeps = [
        ("機構抖動 jitter_deg (°)", "mechanism", "jitter_deg", (0.1, 0.2, 0.4, 0.8)),
        ("v0 抖動 v0_jitter (m/s)", "launch", "v0_jitter", (0.05, 0.1, 0.2, 0.4)),
        ("落點量測雜訊 meas_noise_m (m)", "calibration", "meas_noise_m", (0.01, 0.02, 0.05, 0.10)),
    ]
    for title, sec, key, vals in sweeps:
        cells = []
        for v in vals:
            r = mc(cfg_with(cfg, **{sec: {key: v}}), n, practice=9, **plan_kw)
            cells.append(f"{v}: {r['hit'] * 100:.0f}% / {r['bonus_hit'] * 100:.0f}%")
            res["D"].setdefault(title, []).append(dict(value=v, hit=r["hit"], bonus_hit=r["bonus_hit"],
                                                        points=r["points"]))
        p(f"- **{title}**：" + "；".join(cells))
    cells = []
    for v in (0.05, 0.1, 0.3, 0.5):
        kw2 = dict(plan_kw)
        kw2["noise_ang"] = v
        r = mc(cfg, n, practice=9, **kw2)
        cells.append(f"{v}: {r['hit'] * 100:.0f}% / {r['bonus_hit'] * 100:.0f}%")
        res["D"].setdefault("逐幀感知角度雜訊 (°)", []).append(dict(value=v, hit=r["hit"],
                                                                bonus_hit=r["bonus_hit"]))
    p("- **逐幀感知角度雜訊 noise_ang (°)**：" + "；".join(cells))


def report_f(cfg, p, res, weights, n_scenes, n_render):
    from rotbot.perception.detector import YoloDetector
    from rotbot.sim.closed_loop import run_closed_loop_render
    from tools.eval_perception import run as eval_run
    p("\n## F YOLO 端到端感知（合成影像）\n")
    res["F"] = {}
    for name, dr in (("2.5m (2.0-3.2)", (2.0, 3.2)), ("6m (5.5-6.5)", (5.5, 6.5)),
                     ("全作業區 (2.5-7.5)", (2.5, 7.5))):
        r = eval_run(weights, n_scenes, dist_range=dr, verbose=False)
        res["F"][name] = r
        p(f"- **{name}**：PnP {r['board_pnp_ok']}/{r['scenes']}、亮孔 {r['lit_hit']}/{r['lit_truth']}、"
          f"距離 中位 {r['dist_cm_median']:.1f} / p95 {r['dist_cm_p95']:.1f} cm、"
          f"方位 中位 {r['bearing_deg_median']:.2f}° / p95 {r['bearing_deg_p95']:.2f}°、"
          f"俯仰 中位 {r['elev_deg_median']:.2f}° / p95 {r['elev_deg_p95']:.2f}°、"
          f"離群 {r['gross_outliers']}/{r['n_aims']}")
    det = YoloDetector(weights)
    reps = [run_closed_loop_render(cfg, det, np.random.default_rng(s),
                                   calib=practice_calibration(cfg, np.random.default_rng(100 + s), 9))
            for s in range(n_render)]
    pts = [r.total_points for r in reps]
    hr = [r.hit_rate for r in reps]
    bh = [r.bonus_hit_rate for r in reps]
    res["F"]["render_closed_loop"] = dict(points=pts, hit=hr, bonus_hit=bh)
    p(f"- **render + YOLO 閉迴路（{len(reps)} 場，賽前練習 9 球）**：總分 {np.mean(pts):.1f}、"
      f"命中率 {np.mean(hr) * 100:.0f}%、加分孔命中 {np.mean(bh) * 100:.0f}%")


# ══════════════════════════ 主程式 ══════════════════════════
def main():
    ap = argparse.ArgumentParser(description="6m 重新評估（模擬/合成）")
    ap.add_argument("--quick", action="store_true", help="少量場次（煙霧測試）")
    ap.add_argument("--n", type=int, default=60, help="每個條件的蒙地卡羅場數")
    ap.add_argument("--sections", default="ABCDEF", help="要跑的節（F 需要 --weights）")
    ap.add_argument("--weights", default=None, help="YOLO 權重（F 節：端到端感知與 render 閉迴路）")
    ap.add_argument("--out", default=str(OUT_DIR))
    args = ap.parse_args()
    if hasattr(sys.stdout, "reconfigure"):          # Windows 主控台 cp950 印不出 Ø 等字元
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    want = set(args.sections.upper())
    n = 6 if args.quick else args.n
    n_scenes = 8 if args.quick else 60
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    cfg = load_config()
    t0 = time.time()
    md: list[str] = []
    res: dict = dict(config=dict(dist_x=cfg.field.dist_x, grid_cy=cfg.field.grid_cy,
                                 launch_h=cfg.field.launch_h, v0_model=cfg.launch.v0_model,
                                 v0_real=cfg.launch.v0_real, v0_jitter=cfg.launch.v0_jitter,
                                 cd=cfg.air.cd, cd_model=cfg.aim.cd_model, aim=cfg.aim.model,
                                 jitter_deg=cfg.mechanism.jitter_deg,
                                 bias=(cfg.mechanism.bias_pitch, cfg.mechanism.bias_yaw)),
                     n_matches=n, sections="".join(sorted(want)))

    def p(line=""):
        print(line)
        md.append(line)

    p(f"# 6m 重新評估結果（模擬/合成評估，非實機）— 節 {''.join(sorted(want))}\n")
    p(f"設定：dist_x={cfg.field.dist_x} m、grid_cy={cfg.field.grid_cy} m、launch_h={cfg.field.launch_h} m、"
      f"v0_model={cfg.launch.v0_model} / v0_real={cfg.launch.v0_real} m/s（σ {cfg.launch.v0_jitter}）、"
      f"Cd 真實 {cfg.air.cd} / 模型 {cfg.aim.cd_model}、瞄準模型 {cfg.aim.model}、"
      f"機構抖動 {cfg.mechanism.jitter_deg}°、每條件 {n} 場\n")

    perc = None
    if want & set("EBCD"):                          # B/C/D 需要 E 的感知誤差模型
        perc = report_e(cfg, p, res, n_scenes)
    plan_kw = dict(perc or {}, v0_drift_frac=0.01)  # 練習→正式賽 v0 漂移 1%（電量/溫度，假設）
    if "A" in want:
        report_a(cfg, p, res)
    if "B" in want:
        report_b(cfg, p, res, n, plan_kw)
    if "C" in want:
        report_c(cfg, p, res, n, plan_kw, perc)
    if "D" in want:
        report_d(cfg, p, res, n, plan_kw)
    if "F" in want and args.weights:
        report_f(cfg, p, res, args.weights, n_scenes, 3 if args.quick else 6)

    p(f"\n（耗時 {time.time() - t0:.0f} s）")
    tag = "" if want >= set("ABCDE") else "_" + "".join(sorted(want))
    (out / f"results{tag}.json").write_text(json.dumps(res, ensure_ascii=False, indent=1, default=float),
                                            encoding="utf-8")
    (out / f"report{tag}.md").write_text("\n".join(md) + "\n", encoding="utf-8")
    print(f"\n→ {out / f'report{tag}.md'}")


if __name__ == "__main__":
    main()
