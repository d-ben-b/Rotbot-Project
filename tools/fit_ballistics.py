# -*- coding: utf-8 -*-
"""試投擬合 —— 用「打牆」的落點資料擬合 出射速度 v0、阻力係數 Cd、轉塔安裝偏差（俯仰/水平）。

硬體到位後最省時間的校正法：不需要網板、不需要重新訓練，30 分鐘拿到 6m 瞄準最需要的參數。
  1. 機器人擺在牆前已知距離：2m、4m、6m 三個距離（近距離幾乎不受阻力 → 釘住 v0；遠距離 → 決定 Cd；
     只打 3m+6m 時 v0 與 Cd 幾乎完全相關，擬合會飄走——自我驗證實測過）
  2. 每個距離用 3 個仰角（例：10/20/30°）各打 2~3 球，記錄指令角度與落點（手機慢動作 + 牆上捲尺）
  3. 填 CSV：dist_x,pitch_deg,yaw_deg,impact_z,impact_y[,launch_h]（m；impact_y 為離地高、impact_z 右為正）
  4. python -m tools.fit_ballistics shots.csv → 印出要貼進 configs/field.yaml 的值，並寫校正暖機檔

模型與真實世界模擬同一套物理（sim/world.py：RK4 + 平方阻力），以 Levenberg–Marquardt 最小平方擬合。

    python -m tools.fit_ballistics runs/practice/shots.csv --calib-out runs/calib/practice.json
    python -m tools.fit_ballistics --synthetic 40          # 自我驗證：用已知參數產生假資料再擬合回來
"""
from __future__ import annotations

import argparse
import csv
import dataclasses
import json
import sys
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np

from rotbot.config import Config, load_config
from rotbot.sim.world import BallisticWorld

PARAMS = ("v0", "cd", "bias_pitch", "bias_yaw")
STEPS = np.array([0.05, 0.01, 0.05, 0.05])          # 數值微分步長（m/s, -, deg, deg）
LOWER = np.array([3.0, 0.1, -10.0, -10.0])          # 物理範圍（避免擬合跑到不合理的區域）
UPPER = np.array([40.0, 1.5, 10.0, 10.0])
SIGMA_MEAS = 0.03                                   # 先驗換算成「幾公尺的殘差」用的典型落點散布 (m)


@dataclasses.dataclass
class Shot:
    dist_x: float
    pitch_deg: float
    yaw_deg: float
    impact_z: float
    impact_y: float
    launch_h: float


def load_shots(path: str | Path, cfg: Config) -> list[Shot]:
    shots = []
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            shots.append(Shot(float(row["dist_x"]), float(row["pitch_deg"]), float(row["yaw_deg"]),
                              float(row["impact_z"]), float(row["impact_y"]),
                              float(row.get("launch_h") or cfg.field.launch_h)))
    return shots


def _world(cfg: Config, dist_x: float, launch_h: float, cd: float) -> BallisticWorld:
    # 積分步長 2ms：RK4 在 0.6s 飛行內誤差仍 < 1mm，擬合快一倍
    c = dataclasses.replace(cfg, field=dataclasses.replace(cfg.field, dist_x=dist_x, launch_h=launch_h),
                            air=dataclasses.replace(cfg.air, cd=cd),
                            sim=dataclasses.replace(cfg.sim, dt=0.002))
    return BallisticWorld(c)


def predict(theta: np.ndarray, shots: list[Shot], cfg: Config) -> np.ndarray:
    """參數 → 每球預測落點 (z, y)；打不到牆的球給一個很遠的值。"""
    v0, cd, bp, by = theta
    worlds = {}
    out = np.empty((len(shots), 2))
    for k, s in enumerate(shots):
        key = (s.dist_x, s.launch_h)
        if key not in worlds:
            worlds[key] = _world(cfg, s.dist_x, s.launch_h, max(cd, 0.0))
        land = worlds[key].shoot(np.radians(s.pitch_deg + bp), np.radians(s.yaw_deg + by), v0=v0)
        out[k] = land if land is not None else (0.0, -5.0)
    return out


def fit(shots: list[Shot], cfg: Config, theta0=None, iters: int = 30,
        cd_prior: tuple[float, float] | None = (0.5, 0.1), fix_v0: float | None = None) -> dict:
    """Levenberg–Marquardt（含參數範圍）；回傳參數、標準誤、殘差。

    cd_prior=(平均, 標準差)：圓球在 Re≈6e4 的阻力係數約 0.4~0.55，加弱先驗避免 v0/Cd 沿相關方向飄走。
    fix_v0：若 v0 另外量到（測速器/慢動作），固定它只擬合 Cd 與偏差。
    """
    meas = np.array([[s.impact_z, s.impact_y] for s in shots])
    theta = np.array(theta0 if theta0 is not None else
                     [cfg.launch.v0_model, cfg.aim.cd_model, 0.0, 0.0], float)
    free = np.ones(len(theta), bool)
    if fix_v0 is not None:
        theta[0], free[0] = fix_v0, False

    def resid(th):
        r_ = (predict(th, shots, cfg) - meas).ravel()
        if cd_prior is not None:
            r_ = np.append(r_, (th[1] - cd_prior[0]) / cd_prior[1] * SIGMA_MEAS)
        return r_

    r = resid(theta)
    cost = float(r @ r)
    lam = 1e-2
    idx = np.nonzero(free)[0]
    rel = 1.0
    for _ in range(iters):
        J = np.zeros((r.size, len(theta)))
        for j in idx:
            d = np.zeros(len(theta))
            d[j] = STEPS[j]
            J[:, j] = (resid(theta + d) - resid(theta - d)) / (2 * STEPS[j])
        Jf = J[:, idx]
        A, g = Jf.T @ Jf, Jf.T @ r
        improved = False
        for _ in range(8):
            step = np.zeros(len(theta))
            step[idx] = np.linalg.solve(A + lam * np.diag(np.diag(A) + 1e-12), -g)
            cand = np.clip(theta + step, LOWER, UPPER)
            r_new = resid(cand)
            c_new = float(r_new @ r_new)
            if c_new < cost:
                rel = (cost - c_new) / max(cost, 1e-12)
                theta, r, cost, lam, improved = cand, r_new, c_new, lam * 0.3, True
                break
            lam *= 10
        if not improved or rel < 1e-7:
            break
    n_data = 2 * len(shots)
    dof = max(1, n_data - len(idx))
    s2 = float(r[:n_data] @ r[:n_data]) / dof
    cov = np.zeros((len(theta), len(theta)))
    Jf = J[:, idx]
    cov[np.ix_(idx, idx)] = s2 * np.linalg.pinv(Jf.T @ Jf)
    res = r[:n_data].reshape(-1, 2)
    corr = cov[0, 1] / np.sqrt(cov[0, 0] * cov[1, 1]) if cov[0, 0] > 0 and cov[1, 1] > 0 else 0.0
    return dict(theta=theta, se=np.sqrt(np.clip(np.diag(cov), 0, None)),
                rms_z=float(np.sqrt(np.mean(res[:, 0] ** 2))),
                rms_y=float(np.sqrt(np.mean(res[:, 1] ** 2))),
                corr_v0_cd=float(corr), n=len(shots))


def synthetic_shots(cfg: Config, n: int, rng: np.random.Generator) -> list[Shot]:
    """用設定裡的「真實世界」（v0_real、air.cd、機構偏差與抖動）產生試投資料。"""
    plan = [(d, p) for d in (2.0, 4.0, 6.0) for p in (10.0, 20.0, 30.0)]
    shots = []
    for k in range(n):
        d, p = plan[k % len(plan)]
        yaw = float(rng.uniform(-5, 5))
        w = _world(cfg, d, cfg.field.launch_h, cfg.air.cd)
        w.rng = rng
        land = w.shoot_real(np.radians(p), np.radians(yaw))
        if land is None:
            continue
        shots.append(Shot(d, p, yaw, land[0] + rng.normal(0, 0.01), land[1] + rng.normal(0, 0.01),
                          cfg.field.launch_h))
    return shots


def save_shots(shots: list[Shot], path: str | Path) -> None:
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["dist_x", "pitch_deg", "yaw_deg", "impact_z", "impact_y", "launch_h"])
        for s in shots:
            w.writerow([s.dist_x, s.pitch_deg, f"{s.yaw_deg:.3f}", f"{s.impact_z:.4f}",
                        f"{s.impact_y:.4f}", s.launch_h])


def main():
    ap = argparse.ArgumentParser(description="試投擬合 v0 / Cd / 轉塔安裝偏差")
    ap.add_argument("csv", nargs="?", help="試投資料 CSV")
    ap.add_argument("--synthetic", type=int, default=0, help="自我驗證：產生 N 球假資料再擬合")
    ap.add_argument("--calib-out", help="寫出校正暖機檔（給 run_autonomous --calib）")
    ap.add_argument("--fix-v0", type=float, help="v0 已另外量到（測速器/慢動作）就固定它")
    ap.add_argument("--no-cd-prior", action="store_true", help="不加 Cd 先驗（資料距離夠分散時）")
    args = ap.parse_args()
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    cfg = load_config()
    if args.synthetic:
        shots = synthetic_shots(cfg, args.synthetic, np.random.default_rng(0))
        truth = dict(v0=cfg.launch.v0_real, cd=cfg.air.cd, bias_pitch=cfg.mechanism.bias_pitch,
                     bias_yaw=cfg.mechanism.bias_yaw)
    elif args.csv:
        shots, truth = load_shots(args.csv, cfg), None
    else:
        ap.error("請給 CSV 或 --synthetic N")

    r = fit(shots, cfg, fix_v0=args.fix_v0, cd_prior=None if args.no_cd_prior else (0.5, 0.1))
    print(f"擬合 {r['n']} 球（距離 {sorted({s.dist_x for s in shots})} m）：")
    for name, v, se in zip(PARAMS, r["theta"], r["se"]):
        t = f"   真值 {truth[name]:.3f}" if truth else ""
        print(f"  {name:11s} = {v:8.3f} ± {se:.3f}{t}")
    print(f"  殘差 RMS：橫向 {r['rms_z'] * 100:.1f} cm、高度 {r['rms_y'] * 100:.1f} cm"
          f"（= 逐發散布 + 量測誤差；6m 加分孔容許 6.5cm）")
    if abs(r["corr_v0_cd"]) > 0.95:
        print(f"  ⚠️ v0 與 Cd 高度相關（{r['corr_v0_cd']:.2f}）：請在兩個不同距離各打幾球")
    v0, cd, bp, by = r["theta"]
    print("\n貼進 configs/field.yaml：")
    print(f"  launch:\n    v0_model: {v0:.2f}    # 試投擬合（±{r['se'][0]:.2f}）")
    print(f"  aim:\n    cd_model: {cd:.3f}    # 試投擬合（±{r['se'][1]:.3f}）")
    if args.calib_out:
        from rotbot.sim.closed_loop import new_calibrator
        c2 = dataclasses.replace(cfg, launch=dataclasses.replace(cfg.launch, v0_model=float(v0)),
                                 aim=dataclasses.replace(cfg.aim, cd_model=float(cd)))
        calib = new_calibrator(c2)
        calib.g_bias = np.radians([-bp, -by])            # 指令 + 偏差 = 實際 → 先扣掉偏差
        Path(args.calib_out).parent.mkdir(parents=True, exist_ok=True)
        calib.save(args.calib_out)
        print(f"\n校正暖機檔（全域偏移 = −偏差）：{args.calib_out}")
    return r


if __name__ == "__main__":
    main()
