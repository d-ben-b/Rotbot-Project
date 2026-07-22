# -*- coding: utf-8 -*-
"""線上自適應校正 —— 收斂性模擬驗證
驗證：以「解析拋體模型（無空氣阻力）」瞄準會產生系統性偏差，
      而誤差回授校正能在少數幾顆球內收斂到可命中範圍。
輸出：收斂曲線 PNG、命中率對比 PNG、終端關鍵數字。
"""
import io
import sys
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8")

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ══════════════════════════ 參數區（集中管理，方便調整）══════════════════════════
# --- 場地與目標 ---
DIST_X        = 2.50      # 發射點到網框平面的水平距離 (m)
LAUNCH_H      = 0.50      # 發射口離地高度 (m)
GRID_CX       = 0.0       # 九宮格中心的橫向位置 (m)
GRID_CY       = 1.50      # 九宮格中心的高度 (m)
COL_PITCH     = 0.80      # 行間距 (m)
ROW_PITCH     = 0.60      # 列間距 (m)
FRAME_D       = 0.40      # 一般網框直徑 (m)
BONUS_D       = 0.20      # 加分格直徑 (m)

# --- 球 ---
BALL_D        = 0.07      # 球徑 (m)
BALL_M        = 0.030     # 球質量 (kg)
V0            = 10.0      # 機器人「模型認定」的出射速度 (m/s) — 雙飛輪 r=6cm @1700rpm
V0_REAL       = 9.60      # 實際出射速度 (m/s) — 飛輪打滑與球體擠壓造成約 4% 損失
V0_JITTER     = 0.20      # 逐發初速變異 (m/s, 1σ) — 電池電壓、球體差異

# --- 空氣阻力（真實世界才有；機器人內部模型刻意忽略）---
RHO           = 1.20      # 空氣密度 (kg/m^3)
CD            = 0.50      # 球體阻力係數
G             = 9.81

# --- 機構系統誤差（真實世界存在，模型不知道）---
BIAS_PITCH    = -0.40     # 俯仰安裝偏差 (deg)
BIAS_YAW      = +0.80     # 水平安裝偏差 (deg)
JITTER_DEG    = 0.40      # 逐發隨機抖動 (deg, 1σ) — 機構背隙、球體差異

# --- 校正演算法 ---
ALPHA_GLOBAL  = 0.50      # 全域偏移增益
ALPHA_CELL    = 0.30      # 單格殘差增益
MEAS_NOISE_M  = 0.02      # 相機落點量測雜訊 (m, 1σ) = 2cm
OUTLIER_M     = 0.60      # 誤差超過此值視為異常，不更新 (m)
CLAMP_DEG     = 5.0       # 修正量上限 (deg)

# --- 模擬設定 ---
N_BALLS       = 27        # 比賽球數
N_TRIALS      = 100       # 蒙地卡羅次數
DT            = 1e-3      # 積分步長 (s) — RK4 四階，此步長精度已足夠
SEED          = 42

# --- 衍生常數 ---
AREA   = np.pi * (BALL_D / 2) ** 2
K_DRAG = 0.5 * RHO * CD * AREA          # F_drag = K_DRAG * |v| * v
TOL_STD   = (FRAME_D - BALL_D) / 2      # 一般框容許球心誤差 = 0.165 m
TOL_BONUS = (BONUS_D - BALL_D) / 2      # 加分格容許球心誤差 = 0.065 m

# 九宮格 9 個目標中心 (橫向 z, 高度 y)
CELLS = [(GRID_CX + c * COL_PITCH, GRID_CY + r * ROW_PITCH)
         for r in (1, 0, -1) for c in (-1, 0, 1)]      # 由上排左→下排右

# ══════════════════════════ 物理 ══════════════════════════
def ideal_elevation(dist, dy):
    """無空氣阻力的理想拋體解 —— 機器人內部模型用的就是這個（低平彈道解）"""
    disc = V0**4 - G * (G * dist**2 + 2 * dy * V0**2)
    if disc < 0:
        return None                                    # 射程不足
    return np.arctan((V0**2 - np.sqrt(disc)) / (G * dist))


def simulate_shot(pitch, yaw, v0=None):
    """真實世界：含平方空氣阻力，RK4 積分到 x = DIST_X 平面，回傳 (z, y) 落點。
    純量運算（不用 numpy 陣列）以求速度。"""
    from math import cos, sin, sqrt
    v0 = V0_REAL if v0 is None else v0
    kd = K_DRAG / BALL_M

    def acc(vx, vy, vz):
        s = sqrt(vx * vx + vy * vy + vz * vz)
        return (-kd * s * vx, -G - kd * s * vy, -kd * s * vz)

    x, y, z = 0.0, LAUNCH_H, 0.0
    vx = v0 * cos(pitch) * cos(yaw)
    vy = v0 * sin(pitch)
    vz = v0 * cos(pitch) * sin(yaw)

    for _ in range(int(2.0 / DT)):
        if x >= DIST_X:
            break
        xp, yp, zp = x, y, z
        a1 = acc(vx, vy, vz)
        a2 = acc(vx + 0.5 * DT * a1[0], vy + 0.5 * DT * a1[1], vz + 0.5 * DT * a1[2])
        a3 = acc(vx + 0.5 * DT * a2[0], vy + 0.5 * DT * a2[1], vz + 0.5 * DT * a2[2])
        a4 = acc(vx + DT * a3[0], vy + DT * a3[1], vz + DT * a3[2])
        # 位置：a 只與 v 有關，用標準 RK4 位置式
        x += DT * vx + DT * DT / 6 * (a1[0] + a2[0] + a3[0])
        y += DT * vy + DT * DT / 6 * (a1[1] + a2[1] + a3[1])
        z += DT * vz + DT * DT / 6 * (a1[2] + a2[2] + a3[2])
        vx += DT / 6 * (a1[0] + 2 * a2[0] + 2 * a3[0] + a4[0])
        vy += DT / 6 * (a1[1] + 2 * a2[1] + 2 * a3[1] + a4[1])
        vz += DT / 6 * (a1[2] + 2 * a2[2] + 2 * a3[2] + a4[2])
        if y < -0.5:
            return None
    if x < DIST_X:
        return None
    f = (DIST_X - xp) / (x - xp)                       # 線性內插到 x = DIST_X
    return zp + f * (z - zp), yp + f * (y - yp)


def sensitivity():
    """數值求 ∂高度/∂俯仰角、∂橫向/∂水平角 —— 用來把位置誤差換算成角度修正量"""
    z_t, y_t = CELLS[4]                                # 以中心格為基準
    d = np.hypot(DIST_X, z_t)
    p0 = ideal_elevation(d, y_t - LAUNCH_H)
    yaw0 = np.arctan2(z_t, DIST_X)
    h = np.radians(0.5)
    _, y_a = simulate_shot(p0 + h, yaw0); _, y_b = simulate_shot(p0 - h, yaw0)
    z_a, _ = simulate_shot(p0, yaw0 + h); z_b, _ = simulate_shot(p0, yaw0 - h)
    return (y_a - y_b) / (2 * h), (z_a - z_b) / (2 * h)


J_PITCH, J_YAW = sensitivity()

# ══════════════════════════ 一場比賽（27 顆）══════════════════════════
def run_match(rng, correct=True):
    """回傳每顆球的徑向誤差(m) 與是否命中；correct=False 代表關閉校正（對照組）"""
    g_bias = np.zeros(2)                                # [pitch, yaw] 全域偏移
    cell_off = np.zeros((9, 2))                         # 每格殘差
    errs, hits, bonus_hits = [], [], []

    for k in range(N_BALLS):
        idx = k % 9                                     # 依序輪投 9 格 × 3 輪
        z_t, y_t = CELLS[idx]
        d = np.hypot(DIST_X, z_t)

        pitch = ideal_elevation(d, y_t - LAUNCH_H)      # 內部模型（不含阻力）
        yaw = np.arctan2(z_t, DIST_X)
        if correct:                                     # 疊加已學到的修正
            pitch += g_bias[0] + cell_off[idx, 0]
            yaw   += g_bias[1] + cell_off[idx, 1]

        # 真實世界：加上機構系統偏差、逐發抖動、初速變異
        p_real = pitch + np.radians(BIAS_PITCH + rng.normal(0, JITTER_DEG))
        y_real = yaw   + np.radians(BIAS_YAW   + rng.normal(0, JITTER_DEG))
        v_real = V0_REAL + rng.normal(0, V0_JITTER)

        land = simulate_shot(p_real, y_real, v_real)
        if land is None:
            errs.append(np.nan); hits.append(False); bonus_hits.append(False); continue
        z_l, y_l = land

        e_z, e_y = z_t - z_l, y_t - y_l                 # 真實誤差
        radial = np.hypot(e_z, e_y)
        errs.append(radial)
        hits.append(radial <= TOL_STD)
        bonus_hits.append(radial <= TOL_BONUS)

        if not correct:
            continue
        # ---- 回授更新（用帶雜訊的量測值，模擬相機）----
        m_z = e_z + rng.normal(0, MEAS_NOISE_M)
        m_y = e_y + rng.normal(0, MEAS_NOISE_M)
        if np.hypot(m_z, m_y) > OUTLIER_M:              # 異常值剔除
            continue
        d_pitch = m_y / J_PITCH                         # 位置誤差 → 角度修正
        d_yaw   = m_z / J_YAW
        g_bias += np.array([ALPHA_GLOBAL * d_pitch, ALPHA_GLOBAL * d_yaw])
        cell_off[idx] += np.array([ALPHA_CELL * d_pitch, ALPHA_CELL * d_yaw])
        lim = np.radians(CLAMP_DEG)
        g_bias = np.clip(g_bias, -lim, lim)
        cell_off = np.clip(cell_off, -lim, lim)

    return np.array(errs), np.array(hits), np.array(bonus_hits)


# ══════════════════════════ 蒙地卡羅 ══════════════════════════
rng = np.random.default_rng(SEED)
E_on = np.full((N_TRIALS, N_BALLS), np.nan)
E_off = np.full((N_TRIALS, N_BALLS), np.nan)
H_on = np.zeros((N_TRIALS, N_BALLS), bool)
H_off = np.zeros((N_TRIALS, N_BALLS), bool)
B_on = np.zeros((N_TRIALS, N_BALLS), bool)
B_off = np.zeros((N_TRIALS, N_BALLS), bool)

for t in range(N_TRIALS):
    E_on[t], H_on[t], B_on[t] = run_match(rng, correct=True)
    E_off[t], H_off[t], B_off[t] = run_match(rng, correct=False)

mean_on = np.nanmean(E_on, axis=0) * 100        # 轉 cm
std_on = np.nanstd(E_on, axis=0) * 100
mean_off = np.nanmean(E_off, axis=0) * 100

shots = np.arange(1, N_BALLS + 1)
tol_std_cm, tol_bonus_cm = TOL_STD * 100, TOL_BONUS * 100

# 第幾顆進入可命中範圍
conv_std = next((int(s) for s, e in zip(shots, mean_on) if e <= tol_std_cm), None)
conv_bonus = next((int(s) for s, e in zip(shots, mean_on) if e <= tol_bonus_cm), None)

hit_off = H_off.mean() * 100
hit_on = H_on.mean() * 100
hit_on_after = H_on[:, 3:].mean() * 100          # 收斂後（第 4 顆起）
bonus_on_after = B_on[:, 3:].mean() * 100
bonus_off = B_off.mean() * 100                   # 加分格：未校正

# ══════════════════════════ 終端輸出 ══════════════════════════
print("=" * 62)
print("  線上自適應校正 — 收斂性模擬（含平方空氣阻力）")
print("=" * 62)
print(f"  容許球心誤差   一般框 ±{tol_std_cm:.1f} cm ｜ 加分格 ±{tol_bonus_cm:.1f} cm")
print(f"  靈敏度         ∂高度/∂俯仰 = {J_PITCH:.3f} m/rad")
print("-" * 62)
print(f"  初始誤差(第1顆)        {mean_on[0]:6.1f} cm")
print(f"  未校正平均誤差         {np.nanmean(mean_off):6.1f} cm")
print(f"  收斂到 ±{tol_std_cm:.1f}cm         第 {conv_std} 顆")
print(f"  收斂到 ±{tol_bonus_cm:.1f}cm(加分格)  第 {conv_bonus} 顆")
print(f"  最終穩態誤差(後10顆)   {np.nanmean(mean_on[-10:]):6.1f} cm")
print("-" * 62)
print(f"  一般框(40cm)  校正前 {hit_off:5.1f} %  →  校正後 {hit_on_after:5.1f} %")
print(f"  加分格(20cm)  校正前 {bonus_off:5.1f} %  →  校正後 {bonus_on_after:5.1f} %")
print("=" * 62)
print("  ★ 關鍵結論：一般框本來就寬鬆，校正的真正價值在「加分格雙倍分」")
print("=" * 62)

# ══════════════════════════ 繪圖 ══════════════════════════
BG, INK, MUTED = "#0B1220", "#F1F5F9", "#94A3B8"
ACCENT, GREY, AMBER = "#22D3EE", "#64748B", "#FBBF24"
plt.rcParams.update({
    "font.sans-serif": ["Microsoft JhengHei"], "axes.unicode_minus": False,
    "figure.facecolor": BG, "axes.facecolor": BG, "savefig.facecolor": BG,
    "text.color": INK, "axes.labelcolor": INK,
    "xtick.color": MUTED, "ytick.color": MUTED,
})

# ---------- 圖 1：收斂曲線 ----------
fig, ax = plt.subplots(figsize=(11, 5.6))
ax.fill_between(shots, np.maximum(mean_on - std_on, 0), mean_on + std_on,
                color=ACCENT, alpha=0.16, linewidth=0, label="±1σ 變異範圍")
ax.plot(shots, mean_on, color=ACCENT, lw=2.4, marker="o", ms=5,
        mfc=BG, mew=1.6, label="校正後誤差（100 次平均）", zorder=5)
ax.axhline(tol_std_cm, color=MUTED, ls="--", lw=1.4, zorder=2)
ax.axhline(tol_bonus_cm, color=AMBER, ls=":", lw=1.4, zorder=2)
ax.text(N_BALLS, tol_std_cm + 0.45, f"一般框容許 ±{tol_std_cm:.1f} cm",
        color=MUTED, fontsize=10, ha="right", va="bottom")
ax.text(N_BALLS, tol_bonus_cm + 0.45, f"加分格容許 ±{tol_bonus_cm:.1f} cm",
        color=AMBER, fontsize=10, ha="right", va="bottom")

# 真正有意義的收斂點：跨進「加分格」門檻（一般框門檻初始就已滿足）
if conv_bonus:
    ax.annotate(f"第 {conv_bonus} 顆起進入\n加分格命中範圍",
                xy=(conv_bonus, mean_on[conv_bonus - 1]),
                xytext=(conv_bonus + 3.0, 11.5),
                color=AMBER, fontsize=12.5, fontweight="bold",
                arrowprops=dict(arrowstyle="->", color=AMBER, lw=1.8))
ax.scatter([1], [mean_on[0]], s=90, facecolor="none", edgecolor="#F87171", lw=2, zorder=6)
ax.text(1.4, mean_on[0], f"初始 {mean_on[0]:.0f} cm", color="#F87171",
        fontsize=11, va="center", fontweight="bold")

ax.set_xlabel("投球數（第幾顆）", fontsize=12)
ax.set_ylabel("落點誤差 (cm)", fontsize=12)
ax.set_title("線上自適應校正：誤差隨投球數收斂", fontsize=17, fontweight="bold", pad=14)
ax.set_xlim(0.5, N_BALLS + 0.5)
ax.set_ylim(0, max(tol_std_cm, np.nanmax(mean_on + std_on)) * 1.14)
ax.set_xticks([1, 5, 10, 15, 20, 25, 27])
ax.grid(color="#1E293B", lw=0.8)
ax.set_axisbelow(True)
for sp in ("top", "right"): ax.spines[sp].set_visible(False)
for sp in ("left", "bottom"): ax.spines[sp].set_color("#334155")
leg = ax.legend(loc="upper right", frameon=False, fontsize=11)
for t_ in leg.get_texts(): t_.set_color(INK)
fig.text(0.99, 0.02, "模擬結果 (Simulation) — 含平方空氣阻力，非實測",
         color=MUTED, fontsize=9.5, ha="right")
fig.tight_layout()
fig.savefig("assets/convergence_curve.png", dpi=300)
print("已輸出：assets/convergence_curve.png")

# ---------- 圖 2：命中率對比 ----------
fig2, ax2 = plt.subplots(figsize=(7.6, 5.0))
groups = ["一般框 40cm\n(容許 ±16.5cm)", "加分格 20cm\n(容許 ±6.5cm)"]
before, after = [hit_off, bonus_off], [hit_on_after, bonus_on_after]
xs = np.arange(2); bw = 0.32
b1 = ax2.bar(xs - bw / 2 - 0.01, before, bw, color=GREY, label="校正前", zorder=3)
b2 = ax2.bar(xs + bw / 2 + 0.01, after, bw, color=ACCENT, label="校正後", zorder=3)
for bars_ in (b1, b2):
    for b in bars_:
        ax2.text(b.get_x() + b.get_width() / 2, b.get_height() + 2.5,
                 f"{b.get_height():.0f}%", ha="center", color=INK,
                 fontsize=16, fontweight="bold")
ax2.set_xticks(xs); ax2.set_xticklabels(groups)
ax2.set_ylabel("命中率 (%)", fontsize=12)
ax2.set_title("校正對命中率的影響", fontsize=17, fontweight="bold", pad=14)
ax2.set_ylim(0, 118)
leg2 = ax2.legend(loc="upper right", frameon=False, fontsize=11)
for t_ in leg2.get_texts(): t_.set_color(INK)
ax2.grid(axis="y", color="#1E293B", lw=0.8); ax2.set_axisbelow(True)
for sp in ("top", "right"): ax2.spines[sp].set_visible(False)
for sp in ("left", "bottom"): ax2.spines[sp].set_color("#334155")
ax2.tick_params(labelsize=12)
ax2.text(0.01, 0.965, "模擬結果 (Simulation)", transform=ax2.transAxes,
         color=MUTED, fontsize=9.5, ha="left", va="top")
fig2.tight_layout()
fig2.savefig("assets/hit_rate.png", dpi=300)
print("已輸出：assets/hit_rate.png")
