# -*- coding: utf-8 -*-
"""四角 + cv2.solvePnP：由網框四角像素座標解出「距離 + 瞄準角 + 網框傾角」。
本檔含自我驗證：先用相機模型把已知的真值投影成四角（模擬完美偵測），
再丟回 solvePnP 反解，比對能否還原真值。"""
import io
import sys
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8")

import numpy as np
import cv2

# ══════ 1. 相機內參（範例值：1920x1080、水平 FOV≈69°）══════
K = np.array([[1400.0,    0.0, 960.0],
              [   0.0, 1400.0, 540.0],
              [   0.0,    0.0,   1.0]], dtype=np.float64)
dist = np.zeros((5, 1))                     # 未校正先放 0

# ══════ 2. 網框四角 3D 座標（框中心為原點，Z=0，單位:公尺）══════
# 順序：左上 → 右上 → 右下 → 左下（IPPE_SQUARE 規定）
S = 0.40
OBJ = np.array([[-S/2,  S/2, 0.0],
                [ S/2,  S/2, 0.0],
                [ S/2, -S/2, 0.0],
                [-S/2, -S/2, 0.0]], dtype=np.float64)


def solve_frame_pose(img_pts):
    """img_pts:(4,2) 像素座標 → 距離/瞄準角/傾角/重投影誤差"""
    img_pts = np.asarray(img_pts, dtype=np.float64)
    ok, rvec, tvec = cv2.solvePnP(OBJ, img_pts, K, dist,
                                  flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    x, y, z = tvec.ravel()
    distance = float(np.linalg.norm(tvec))
    yaw_aim   = np.degrees(np.arctan2(x, z))
    pitch_aim = np.degrees(np.arctan2(-y, z))
    R, _ = cv2.Rodrigues(rvec)
    normal = R @ np.array([0.0, 0.0, 1.0])
    tilt = np.degrees(np.arccos(min(1.0, abs(normal[2]))))
    proj, _ = cv2.projectPoints(OBJ, rvec, tvec, K, dist)
    reproj = float(np.mean(np.linalg.norm(proj.reshape(-1, 2) - img_pts, axis=1)))
    return dict(distance=distance, yaw_aim=yaw_aim, pitch_aim=pitch_aim,
                tilt=tilt, reproj_err=reproj)


def project_truth(distance, bearing_deg, elev_deg, frame_tilt_deg, noise_px=0.0):
    """給定真值 → 算出網框四角在影像上的像素座標（= 模擬完美/含雜訊的偵測）"""
    # 網框中心相對相機的位置（相機座標：+x右 +y下 +z前）
    b, e = np.radians(bearing_deg), np.radians(elev_deg)
    center = distance * np.array([np.sin(b) * np.cos(e), -np.sin(e), np.cos(b) * np.cos(e)])
    # 網框自身繞垂直軸傾斜 frame_tilt_deg（模擬被打歪）
    t = np.radians(frame_tilt_deg)
    R_frame = np.array([[np.cos(t), 0, np.sin(t)],
                        [0, 1, 0],
                        [-np.sin(t), 0, np.cos(t)]])
    pts_cam = (R_frame @ OBJ.T).T + center           # 四角在相機座標
    uv = (K @ pts_cam.T).T
    uv = uv[:, :2] / uv[:, 2:3]                       # 投影到像素
    if noise_px:
        uv = uv + np.random.default_rng(0).normal(0, noise_px, uv.shape)
    return uv


def report(name, truth, pts):
    p = solve_frame_pose(pts)
    d, bear, elev, tilt = truth
    print(f"\n── {name} ──")
    print(f"          {'真值':>10} {'解出':>10}   誤差")
    print(f"  距離(m)  {d:10.2f} {p['distance']:10.2f}   {abs(d-p['distance']):.3f}")
    print(f"  方位(°)  {bear:10.1f} {p['yaw_aim']:10.1f}   {abs(bear-p['yaw_aim']):.2f}")
    print(f"  俯仰(°)  {elev:10.1f} {p['pitch_aim']:10.1f}   {abs(elev-p['pitch_aim']):.2f}")
    print(f"  傾角(°)  {tilt:10.1f} {p['tilt']:10.1f}   {abs(tilt-p['tilt']):.2f}")
    print(f"  重投影誤差 {p['reproj_err']:.2f} px")


if __name__ == "__main__":
    print("=" * 56)
    print("  四角 solvePnP 自我驗證（真值 → 投影 → 反解）")
    print("=" * 56)

    # 案例 A：正前方 2.5m、正對
    truth = (2.50, 0.0, 0.0, 0.0)
    report("A. 正前方 2.5m 正對", truth, project_truth(*truth))

    # 案例 B：右偏 15°、略仰 10°、距離 3.0m
    truth = (3.00, 15.0, 10.0, 0.0)
    report("B. 右15° 仰10° 距3.0m", truth, project_truth(*truth))

    # 案例 C：網框被打歪 25°（測 tilt 能不能抓到）
    truth = (2.50, -8.0, 5.0, 25.0)
    report("C. 網框被打歪 25°", truth, project_truth(*truth))

    # 案例 D：加入 1.5px 偵測雜訊（模擬真實 YOLO 關鍵點誤差）
    truth = (2.50, 0.0, 0.0, 0.0)
    report("D. 正對 2.5m + 1.5px 雜訊", truth, project_truth(*truth, noise_px=1.5))

    print("\n" + "=" * 56)
    print("  無雜訊時誤差趨近 0 → 幾何與程式正確")
    print("  1.5px 雜訊下距離誤差 = 真實可預期的精度")
    print("=" * 56)
