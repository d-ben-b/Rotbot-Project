# -*- coding: utf-8 -*-
"""筆電鏡頭即時圓孔測距 —— 手拿一個圓，即時算距離 / 方位 / 俯仰 / 傾角。

驗證計劃書的「單目幾何景深估測」的真實版：用真 webcam + 已知圓真尺寸（比賽 Ø40/Ø20cm），
以「表觀直徑 → 距離」（方法A）為主、「橢圓圓錐反投影」（方法B）補傾角。
與機器人共用 rotbot.perception.pose 的公式。

用法：
    python -m tools.webcam_ranger                 # 預設 40cm、FOV 60、鏡頭 0
    python -m tools.webcam_ranger --fov 68 --cam 1

視窗操作：
    1 = 一般孔 40cm    2 = 加分孔 20cm    d = 自訂直徑(cm)
    +/- = 微調 FOV     s = 存一張截圖     q/Esc = 離開
    左側 controls 視窗可調 Canny 邊緣門檻與最小半徑。
"""
from __future__ import annotations

import argparse
import math

import cv2
import numpy as np

from rotbot.perception.pose import distance_from_diameter, pose_from_ellipse

WIN = "Webcam ranger"
CTRL = "controls"


def intrinsics(w: int, h: int, fov_deg: float) -> np.ndarray:
    """由水平 FOV 估相機內參（假設方形像素、主點在中心）。"""
    fx = (w / 2.0) / math.tan(math.radians(fov_deg) / 2.0)
    return np.array([[fx, 0, w / 2.0], [0, fx, h / 2.0], [0, 0, 1.0]], np.float64)


def find_circles(gray: np.ndarray, canny_lo: int, canny_hi: int, min_r: int):
    """回傳通過濾網的橢圓清單 [((cx,cy),(d1,d2),ang), area, fill]，最大者在最後方便挑。"""
    edges = cv2.Canny(gray, canny_lo, canny_hi)
    edges = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=1)
    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for c in contours:
        if len(c) < 20:
            continue
        area = cv2.contourArea(c)
        if area < math.pi * min_r * min_r:
            continue
        ell = cv2.fitEllipse(c)
        (_, _), (d1, d2), _ = ell
        major, minor = max(d1, d2), min(d1, d2)
        if minor < 2 * min_r or minor / major < 0.25:      # 太小或太扁（線狀）→ 丟
            continue
        ell_area = math.pi * (d1 / 2) * (d2 / 2)
        fill = area / (ell_area + 1e-9)
        if not (0.7 <= fill <= 1.3):                        # 輪廓要真的像橢圓
            continue
        out.append((ell, area, fill))
    out.sort(key=lambda t: t[1])                            # 由小到大，最後 = 最大
    return out


class EMA:
    """指數平滑，讓顯示數值不抖。"""
    def __init__(self, a=0.4):
        self.a, self.v = a, None

    def __call__(self, x):
        if x is None or (isinstance(x, float) and math.isnan(x)):
            return self.v
        self.v = x if self.v is None else self.a * x + (1 - self.a) * self.v
        return self.v


def draw(frame, ell, info, real_cm, fov):
    (cx, cy), (d1, d2), ang = ell
    h, w = frame.shape[:2]
    cv2.ellipse(frame, ell, (60, 220, 90), 2, cv2.LINE_AA)
    cv2.circle(frame, (int(cx), int(cy)), 4, (60, 220, 235), -1, cv2.LINE_AA)
    cv2.drawMarker(frame, (w // 2, h // 2), (200, 200, 200), cv2.MARKER_CROSS, 22, 1)
    cv2.line(frame, (w // 2, h // 2), (int(cx), int(cy)), (120, 120, 120), 1, cv2.LINE_AA)
    lines = [
        f"type: {'BONUS 20cm' if abs(real_cm-20)<1 else 'MAIN 40cm' if abs(real_cm-40)<1 else f'{real_cm:.0f}cm'}"
        f"   FOV {fov:.0f}deg",
        f"dist(size)  {info['dist']:6.2f} m   (major {max(d1,d2):.0f}px)",
        f"bearing     {info['bear']:+6.1f} deg  (right +)",
        f"elevation   {info['elev']:+6.1f} deg  (up +)",
        f"tilt(conic) {info['tilt']}   dist(conic) {info['distB']}",
    ]
    for i, t in enumerate(lines):
        y = 28 + i * 26
        cv2.putText(frame, t, (14, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62,
                    (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame, t, (14, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62,
                    (240, 240, 240), 1, cv2.LINE_AA)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cam", type=int, default=0, help="鏡頭索引")
    ap.add_argument("--fov", type=float, default=60.0, help="水平視野角(度)，決定測距準度")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--diam", type=float, default=40.0, help="初始圓真直徑(cm)")
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.cam, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        print(f"[錯誤] 打不開鏡頭 {args.cam}。換 --cam 1 試試，或確認沒有其他程式佔用。")
        return

    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.namedWindow(CTRL, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(CTRL, 360, 140)
    cv2.createTrackbar("canny_lo", CTRL, 60, 500, lambda x: None)
    cv2.createTrackbar("canny_hi", CTRL, 160, 500, lambda x: None)
    cv2.createTrackbar("min_r", CTRL, 18, 200, lambda x: None)

    real_cm, fov = args.diam, args.fov
    em_d, em_b, em_e = EMA(), EMA(), EMA()
    print("操作：1=40cm 2=20cm d=自訂 +/-=FOV s=截圖 q=離開")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("[錯誤] 讀取影格失敗"); break
        h, w = frame.shape[:2]
        K = intrinsics(w, h, fov)
        fx = K[0, 0]
        gray = cv2.GaussianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (5, 5), 0)
        lo = cv2.getTrackbarPos("canny_lo", CTRL)
        hi = cv2.getTrackbarPos("canny_hi", CTRL)
        min_r = max(4, cv2.getTrackbarPos("min_r", CTRL))
        cands = find_circles(gray, lo, hi, min_r)

        if cands:
            for ell, _a, _f in cands[:-1]:                 # 其他候選：淡灰
                cv2.ellipse(frame, ell, (90, 90, 90), 1, cv2.LINE_AA)
            ell = cands[-1][0]                             # 最大者 = 目標
            (cx, cy), (d1, d2), _ = ell
            major = max(d1, d2)
            real_m = real_cm / 100.0
            dist = distance_from_diameter(major, real_m, fx)
            ray = np.linalg.inv(K) @ np.array([cx, cy, 1.0])
            bearing = math.degrees(math.atan2(ray[0], ray[2]))
            elev = math.degrees(math.atan2(-ray[1], ray[2]))
            # 方法B：橢圓圓錐反投影（正對時會退化 → 過濾不合理值）
            tilt_s, distB_s = "--", "--"
            cp = pose_from_ellipse(ell, K, real_m / 2.0)
            if cp is not None and 0.1 < cp.distance < 30 and abs(cp.distance - dist) < 2 * dist:
                tilt_s = f"{cp.tilt:4.0f}deg"
                distB_s = f"{cp.distance:.2f}m"
            info = dict(dist=em_d(dist), bear=em_b(bearing), elev=em_e(elev),
                        tilt=tilt_s, distB=distB_s)
            draw(frame, ell, info, real_cm, fov)
        else:
            cv2.putText(frame, "no circle - 對準一個圓，或調 controls 門檻",
                        (14, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (60, 200, 240), 2, cv2.LINE_AA)

        cv2.imshow(WIN, frame)
        k = cv2.waitKey(1) & 0xFF
        if k in (ord("q"), 27):
            break
        elif k == ord("1"):
            real_cm = 40.0; em_d.v = em_b.v = em_e.v = None
        elif k == ord("2"):
            real_cm = 20.0; em_d.v = em_b.v = em_e.v = None
        elif k == ord("d"):
            try:
                real_cm = float(input("輸入圓的真實直徑(cm)："))
            except ValueError:
                print("忽略：非數字")
        elif k == ord("+") or k == ord("="):
            fov = min(120, fov + 1)
        elif k == ord("-"):
            fov = max(20, fov - 1)
        elif k == ord("s"):
            cv2.imwrite("runs/ranger_shot.png", frame); print("已存 runs/ranger_shot.png")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
