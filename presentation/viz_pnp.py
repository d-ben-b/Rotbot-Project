# -*- coding: utf-8 -*-
"""把 solvePnP 四角測距的結果可視化：
  圖1 相機視角 — 不同姿態的網框四角偵測與解算結果
  圖2 精度 vs 偵測雜訊 — 蒙地卡羅，證明距離/瞄準角對雜訊穩健"""
import io
import sys
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8")

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle
from pnp_frame_pose import solve_frame_pose, project_truth, K   # 重用已驗證的模組

# ── 主題色（與簡報一致）──
BG, INK, MUTED = "#0B1220", "#F1F5F9", "#94A3B8"
CYAN, BLUE, AMBER, GREEN, RED = "#22D3EE", "#38BDF8", "#FBBF24", "#34D399", "#F87171"
plt.rcParams.update({
    "font.sans-serif": ["Microsoft JhengHei"], "axes.unicode_minus": False,
    "figure.facecolor": BG, "axes.facecolor": BG, "savefig.facecolor": BG,
    "text.color": INK, "axes.labelcolor": INK,
    "xtick.color": MUTED, "ytick.color": MUTED,
})
IMG_W, IMG_H = 1920, 1080

# ════════════════════ 圖 1：相機視角 ════════════════════
cases = [
    ("正對 2.5m",       (2.50,   0.0,  0.0,  0.0), CYAN),
    ("右15°仰10° 3m",   (3.00,  15.0, 10.0,  0.0), BLUE),
    ("被打歪 25°",      (2.50,  -8.0,  5.0, 25.0), AMBER),
]
NUM = ["①", "②", "③"]
fig, ax = plt.subplots(figsize=(9.8, 5.8))
ax.add_patch(Rectangle((0, 0), IMG_W, IMG_H, fill=False, ec="#334155", lw=1.5))
ax.plot([IMG_W/2], [IMG_H/2], "+", color=MUTED, ms=16, mew=1.6)          # 影像中心
ax.plot([IMG_W/2, IMG_W/2], [IMG_H/2, 150], color=MUTED, ls=":", lw=1)    # 引線到空白處
ax.text(IMG_W/2, 130, "影像中心", color=MUTED, fontsize=9.5, ha="center", va="bottom")

solved = []
for i, (name, truth, c) in enumerate(cases):
    uv = project_truth(*truth)                        # 完美四角偵測
    p = solve_frame_pose(uv); solved.append(p)
    ax.add_patch(Polygon(uv, closed=True, fill=True, fc=c, ec=c, alpha=0.14, lw=2.2))
    ax.add_patch(Polygon(uv, closed=True, fill=False, ec=c, lw=2.4))
    ax.scatter(uv[:, 0], uv[:, 1], s=48, color=c, zorder=5, ec=BG, lw=1.2)
    cx, cy = uv[:, 0].mean(), uv[:, 1].mean()
    ax.text(cx, cy, NUM[i], color=BG, fontsize=15, ha="center", va="center",
            fontweight="bold", zorder=6,
            bbox=dict(boxstyle="circle,pad=0.18", fc=c, ec="none"))

# 左下角圖例面板（避開所有框）
ax.add_patch(Rectangle((55, 735), 1120, 300, fc="#101A30", ec="#334155", lw=1.2, zorder=8))
for i, (name, truth, c) in enumerate(cases):
    yy = 800 + i * 82
    ax.scatter([110], [yy], s=150, color=c, marker="s", zorder=9)
    ax.text(150, yy, f"{NUM[i]} {name}", color=INK, fontsize=12.5, va="center",
            fontweight="bold", zorder=9)
    ax.text(650, yy, f"解出 {solved[i]['distance']:.2f} m ｜ 方位 {solved[i]['yaw_aim']:+.0f}°",
            color=MUTED, fontsize=12, va="center", zorder=9)

ax.set_xlim(-60, IMG_W + 60); ax.set_ylim(IMG_H + 60, -60)              # y 反轉=影像座標
ax.set_aspect("equal")
ax.set_xlabel("影像 X (px)", fontsize=11); ax.set_ylabel("影像 Y (px)", fontsize=11)
ax.set_title("相機視角：四角偵測 → solvePnP 解出距離與角度",
             fontsize=16, fontweight="bold", pad=12)
ax.tick_params(labelsize=9)
for sp in ("top", "right"): ax.spines[sp].set_visible(False)
for sp in ("left", "bottom"): ax.spines[sp].set_color("#334155")
fig.text(0.99, 0.02, "四角為模型頂點投影 · 顏色代表不同姿態", color=MUTED,
         fontsize=9, ha="right")
fig.tight_layout()
fig.savefig("assets/pnp_camera_view.png", dpi=300)
print("已輸出：assets/pnp_camera_view.png")

# ════════════════════ 圖 2：精度 vs 偵測雜訊 ════════════════════
POSE = (3.00, 15.0, 10.0, 0.0)          # 用一個有代表性的斜角姿態
clean = project_truth(*POSE)            # 無雜訊四角
_p = solve_frame_pose(clean)
noise_levels = np.linspace(0, 3.0, 16)  # 0~3 px
N = 400
rng = np.random.default_rng(1)

d_mean, d_std, a_mean, a_std = [], [], [], []
for npx in noise_levels:
    d_err, a_err = [], []
    for _ in range(N):
        pts = clean + (rng.normal(0, npx, clean.shape) if npx > 0 else 0)
        p = solve_frame_pose(pts)
        if p is None:
            continue
        d_err.append(abs(p["distance"] - POSE[0]) * 100)          # cm
        a_err.append(np.hypot(p["yaw_aim"] - _p["yaw_aim"],       # 瞄準角綜合誤差 deg
                              p["pitch_aim"] - _p["pitch_aim"]))
    d_mean.append(np.mean(d_err)); d_std.append(np.std(d_err))
    a_mean.append(np.mean(a_err)); a_std.append(np.std(a_err))

d_mean, d_std = np.array(d_mean), np.array(d_std)
a_mean, a_std = np.array(a_mean), np.array(a_std)
REAL = 1.5                               # 真實 YOLO 關鍵點雜訊量級

fig2, (axd, axa) = plt.subplots(2, 1, figsize=(8.4, 6.4), sharex=True)
# 上：距離誤差
axd.fill_between(noise_levels, np.maximum(d_mean - d_std, 0), d_mean + d_std,
                 color=CYAN, alpha=0.16, lw=0)
axd.plot(noise_levels, d_mean, color=CYAN, lw=2.4, marker="o", ms=5, mfc=BG, mew=1.5)
axd.axvline(REAL, color=MUTED, ls="--", lw=1.2)
axd.text(REAL + 0.06, axd.get_ylim()[1] * 0.1, "真實 YOLO\n雜訊量級",
         color=MUTED, fontsize=9, va="bottom")
axd.set_ylabel("距離誤差 (cm)", fontsize=12)
axd.set_title("測距精度 vs 偵測雜訊（400 次蒙地卡羅）", fontsize=16, fontweight="bold", pad=12)
axd.set_ylim(0, None)

# 下：瞄準角誤差
axa.fill_between(noise_levels, np.maximum(a_mean - a_std, 0), a_mean + a_std,
                 color=BLUE, alpha=0.16, lw=0)
axa.plot(noise_levels, a_mean, color=BLUE, lw=2.4, marker="o", ms=5, mfc=BG, mew=1.5)
axa.axvline(REAL, color=MUTED, ls="--", lw=1.2)
axa.set_ylabel("瞄準角誤差 (°)", fontsize=12)
axa.set_xlabel("四角偵測雜訊 (px, 1σ)", fontsize=12)
axa.set_ylim(0, None)

for a in (axd, axa):
    a.grid(color="#1E293B", lw=0.8); a.set_axisbelow(True)
    for sp in ("top", "right"): a.spines[sp].set_visible(False)
    for sp in ("left", "bottom"): a.spines[sp].set_color("#334155")
    a.tick_params(labelsize=10)

# 在真實雜訊處標數值
i_real = int(np.argmin(abs(noise_levels - REAL)))
axd.annotate(f"{d_mean[i_real]:.1f} cm", (REAL, d_mean[i_real]),
             xytext=(REAL + 0.4, d_mean[i_real] + d_std.max() * 0.6),
             color=CYAN, fontsize=12, fontweight="bold",
             arrowprops=dict(arrowstyle="->", color=CYAN, lw=1.5))
axa.annotate(f"{a_mean[i_real]:.2f}°", (REAL, a_mean[i_real]),
             xytext=(REAL + 0.4, a_mean[i_real] + a_std.max() * 0.6),
             color=BLUE, fontsize=12, fontweight="bold",
             arrowprops=dict(arrowstyle="->", color=BLUE, lw=1.5))
fig2.text(0.99, 0.01, "模擬結果 (Simulation) · 姿態 右15°仰10°距3.0m", color=MUTED,
          fontsize=9, ha="right")
fig2.tight_layout()
fig2.savefig("assets/pnp_precision.png", dpi=300)
print("已輸出：assets/pnp_precision.png")

print(f"\n真實雜訊(1.5px)下：距離誤差 {d_mean[i_real]:.1f} cm、瞄準角誤差 {a_mean[i_real]:.2f}°")
