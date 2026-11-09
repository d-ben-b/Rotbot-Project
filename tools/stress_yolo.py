# -*- coding: utf-8 -*-
"""YOLO 壓力測試 —— 比賽掃視姿態（約 6 m）的合成影像，加上實際場地可能有、訓練資料沒有的情況。

訓練資料（sim/scene.py）的孔是「暗底 + 亮色孔緣」畫在隨機暗色背景上，亮孔是孔緣上一整圈紅環。
規則書與官方 Q&A 描述的實物不同：
  * 孔是木板上的開口，後面以木管接網子（規則「九宮格網框平面尺寸標示圖」第 3 點）→ 沒有亮色孔緣；
    相機在下方往上看，看得到木管上緣的內壁。
  * 紅色 LED 燈條設在網框邊緣往內約 5 cm（Q33）→ 從下往上看只看得到上半圈，而且 475 lm 可能過曝成白色。
  * 場地後方與兩側是透明圍布，孔後面可能是亮的；網子有格線；網內有已投進的球；下半場兩隊球色不同。
這支工具把這些情況一項一項加上去，每項跑同一組幾何（同 seed），看 YOLO + 定位哪裡先壞。

幾何：機器人在 dist_x 5.8–6.2 m、左右偏 ±0.3 m（程式不知道）、轉塔停在 ControlLoop 的掃視姿態（±0.1°）、
網板前後左右傾 ±3°。每場隨機亮一個加分孔（前 60 秒一定有一個亮）。

指標（每項條件 N 場）：
  PnP        整板位姿解出的場數
  一般孔     瞄準角誤差 < 0.5° 的一般孔數 / 9N
  加分孔     同上 / 3N
  亮孔對     回報的亮孔是真的亮孔
  亮孔錯     回報了亮孔但不是真的亮孔（策略會投錯孔）
  編號錯     角度誤差 > 1° 的瞄準（多半是孔編號配錯）
  信心中位   YOLO 框信心值的中位數（掉很多 = 進入沒看過的外觀）

用法：
    python -m tools.stress_yolo                       # 全部條件，各 40 場
    python -m tools.stress_yolo --scenes 10 --only 木板 LED
    python -m tools.stress_yolo --weights models/grid_holes_yolo11n.pt --save-examples
輸出：表格印在終端機；--save-examples 時每項條件存一張畫了框的圖到 runs/stress_yolo/。
結果是模擬/合成評估：外觀是依規則描述推測畫的，實物外觀要用實拍確認。
"""
from __future__ import annotations

import argparse
import dataclasses
import sys
import time
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cv2
import numpy as np

from rotbot.config import Config, load_config
from rotbot.decision import frames
from rotbot.perception.localize import localize_holes
from rotbot.pipeline.loop import ControlLoop
from rotbot.sim.scene import SceneGenerator

REPO_ROOT = Path(__file__).resolve().parent.parent
OUT_DIR = REPO_ROOT / "runs" / "stress_yolo"

WOOD = (115, 165, 205)          # BGR，淺色合板
WHITE = (215, 215, 215)
TUBE_DEPTH = 0.10               # 木管深度 (m)，未知，取 10 cm
LED_DEPTH = 0.05                # Q33：網框邊緣往內約 5 cm
BALL_D = 0.07


def _rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


class FieldScene(SceneGenerator):
    """依規則描述畫實際網板：整塊木板 + 開口 + 木管內壁 + 網子 + LED 位置。

    board=None 時沿用 SceneGenerator 原本的畫法（訓練資料的外觀）。
    """

    def __init__(self, cfg: Config, rng, *, board=None, rim=False, tube=False, net=None,
                 interior=(24, 24, 27), lit="ring", red_ball_unlit=False, white_balls=0,
                 randomize=False):
        super().__init__(cfg, rng=rng, randomize=randomize, rim_jitter=False)
        self.board, self.rim, self.tube, self.net = board, rim, tube, net
        self.interior, self.lit, self.red_ball_unlit, self.white_balls = interior, lit, red_ball_unlit, white_balls
        self._R = self._t = None

    def _render_pose(self, R, t, vp, lit_bonus):
        self._R, self._t = R, t
        self._ball_holes = set(int(i) for i in self.rng.choice(9, self.white_balls, replace=False)) \
            if self.white_balls else set()
        return super()._render_pose(R, t, vp, lit_bonus)

    def _ring_px(self, hole, depth: float, d: float | None = None, n: int = 96):
        th = np.linspace(0, 2 * np.pi, n, endpoint=False)
        r = (hole.diameter if d is None else d) / 2
        pts = hole.center_board + np.stack([r * np.cos(th), r * np.sin(th), np.full_like(th, -depth)], 1)
        return self._project(pts, self._R, self._t)[0]

    def _draw_background(self, img):
        if self.board is None:
            return super()._draw_background(img)
        r = self.rng
        img[:] = r.integers(25, 70, size=3).astype(np.uint8)       # 場館：桁架、圍布、人
        for _ in range(8):
            p1 = (int(r.uniform(0, self.W)), int(r.uniform(0, self.H)))
            p2 = (int(r.uniform(0, self.W)), int(r.uniform(0, self.H)))
            cv2.rectangle(img, p1, p2, tuple(int(x) for x in r.integers(10, 160, size=3)), -1)
        # 木板 300×340 cm：孔外緣 ±1.1 m，板寬 ±1.5 m；中排孔心離地 grid_cy → 板底 y = -grid_cy
        f = self.cfg.field
        corners = np.array([[-1.5, -f.grid_cy, 0], [1.5, -f.grid_cy, 0],
                            [1.5, 3.4 - f.grid_cy, 0], [-1.5, 3.4 - f.grid_cy, 0]], float)
        uv = self._project(corners, self._R, self._t)[0]
        board = np.zeros(img.shape[:2], np.uint8)
        cv2.fillConvexPoly(board, uv.astype(np.int32), 255)
        tex = np.full(img.shape, self.board, np.float32)
        tex *= 1 + 0.06 * np.sin(np.linspace(0, 40, self.H))[:, None, None]   # 木紋（橫向）
        tex *= r.uniform(0.85, 1.15)                                         # 場館亮度
        img[board > 0] = np.clip(tex, 0, 255).astype(np.uint8)[board > 0]

    def _draw_hole(self, img, hole):
        if self.board is None:
            return super()._draw_hole(img, hole)
        front = hole.rim_px.astype(np.int32)
        H, W = img.shape[:2]
        m_front = np.zeros((H, W), np.uint8)
        cv2.fillPoly(m_front, [front], 255)
        m_in = m_front.copy()
        if self.tube:                                    # 看得到的木管內壁 = 前開口 − 後開口
            back = np.zeros_like(m_front)
            cv2.fillPoly(back, [self._ring_px(hole, TUBE_DEPTH).astype(np.int32)], 255)
            m_in = cv2.bitwise_and(m_front, back)
            wall = cv2.bitwise_and(m_front, cv2.bitwise_not(back))
            img[wall > 0] = (np.array(self.board) * 0.55).astype(np.uint8)
        img[m_in > 0] = self.interior
        if self.net is not None:                          # 菱形網格（約 4 cm 一格）
            step = max(4, int(round(self.K[0, 0] * 0.04 / max(hole.center_cam[2], 0.1))))
            layer = np.zeros_like(img)
            for k in range(-2 * W, 2 * W, step):
                cv2.line(layer, (k, 0), (k + H, H), self.net, 1)
                cv2.line(layer, (k + H, 0), (k, H), self.net, 1)
            img[m_in > 0] = np.where(layer[m_in > 0] > 0, layer[m_in > 0], img[m_in > 0])
        if self.rim:
            cv2.polylines(img, [front], True, (200, 200, 205), 5, cv2.LINE_AA)
        if hole.is_bonus and not hole.is_lit and self.red_ball_unlit:
            self._ball(img, hole, (40, 40, 200), 0.0, m_in)
        if not hole.is_bonus and hole.idx in self._ball_holes:
            for dx in (-0.08, 0.0, 0.08)[: 1 + hole.idx % 3]:
                self._ball(img, hole, (225, 230, 235), dx, m_in)
        if hole.is_lit:
            self._draw_lit(img, hole, front, m_front, m_in)

    def _ball(self, img, hole, color, dx: float, m_in):
        """網內的球：孔中心偏下（落在網袋底部），在木管後方，只畫穿過開口看得到的部分。"""
        p = hole.center_board + np.array([dx, -hole.diameter * 0.25, -TUBE_DEPTH - 0.05])
        (uv,), (pc,) = self._project(p[None], self._R, self._t)
        r = max(2, int(round(self.K[0, 0] * BALL_D / 2 / pc[2])))
        layer = img.copy()
        cv2.circle(layer, tuple(int(v) for v in uv), r, color, -1, cv2.LINE_AA)
        img[m_in > 0] = layer[m_in > 0]

    def _draw_lit(self, img, hole, front, m_front, m_in):
        if self.lit == "ring":                            # 訓練資料的畫法：孔緣一整圈紅環
            cv2.polylines(img, [front], True, (18, 18, 106), 13, cv2.LINE_AA)
            cv2.polylines(img, [front], True, (40, 40, 235), 8, cv2.LINE_AA)
            return
        if self.lit in ("arc_white", "arc_red", "glow"):
            tint = img.astype(np.float32)                 # 紅光照亮管內與網子
            tint[m_in > 0] = 0.55 * tint[m_in > 0] + 0.45 * np.array([30, 30, 190])
            img[:] = tint.astype(np.uint8)
        if self.lit == "glow":
            return
        ring = self._ring_px(hole, LED_DEPTH)
        vis = np.array([cv2.pointPolygonTest(front.astype(np.float32), (float(x), float(y)), False) > 0
                        for x, y in ring])                # 被木板擋住的下半圈不畫
        core = (225, 225, 255) if self.lit == "arc_white" else (40, 40, 240)
        layer = img.copy()
        pairs = [(a, (a + 1) % len(ring)) for a in range(len(ring)) if vis[a] and vis[(a + 1) % len(ring)]]
        for color, thick in (((30, 30, 200), 9), (core, 3)):
            for a, b in pairs:
                cv2.line(layer, tuple(ring[a].astype(int)), tuple(ring[b].astype(int)), color, thick, cv2.LINE_AA)
        img[m_front > 0] = layer[m_front > 0]


# ── 影像層的干擾 ─────────────────────────────────────────

def sensor(img, rng, sigma=2.0):
    """鏡頭 + sensor：輕微模糊 + 雜訊（實拍影像不會像合成的那麼銳利乾淨）。"""
    out = cv2.GaussianBlur(img, (3, 3), 0).astype(np.float32)
    return np.clip(out + rng.normal(0, sigma, out.shape), 0, 255).astype(np.uint8)


def motion_blur(px):
    def f(img, rng):
        k = np.zeros((px, px), np.float32)
        k[px // 2, :] = 1.0 / px
        return sensor(cv2.filter2D(img, -1, k), rng)
    return f


def defocus(sigma):
    return lambda img, rng: sensor(cv2.GaussianBlur(img, (0, 0), sigma), rng)


def exposure(gain, noise):
    return lambda img, rng: sensor(np.clip(img.astype(np.float32) * gain, 0, 255).astype(np.uint8), rng, noise)


def white_balance(b, r):
    def f(img, rng):
        out = img.astype(np.float32) * np.array([b, 1.0, r])
        return sensor(np.clip(out, 0, 255).astype(np.uint8), rng)
    return f


def glare_band(img, rng):
    """透明圍布或燈具的反光：斜向亮帶。"""
    H, W = img.shape[:2]
    yy, xx = np.mgrid[:H, :W]
    d = (xx * 0.5 + yy - rng.uniform(0.3, 0.8) * (H + 0.5 * W)) / 60.0
    out = img.astype(np.float32) + 170 * np.exp(-d ** 2)[..., None]
    return sensor(np.clip(out, 0, 255).astype(np.uint8), rng)


def occlude(img, rng, sample):
    """對手機器人或人擋住左下兩個一般孔的大半。"""
    pts = np.array([h.center_px for h in sample.holes if h.group == "main" and h.idx in (6, 7)])
    d = np.linalg.norm(pts[0] - pts[1])
    x1, y1 = pts[:, 0].min() - 0.4 * d, pts[:, 1].min() - 0.1 * d
    out = img.copy()
    cv2.rectangle(out, (int(x1), int(y1)), (int(pts[:, 0].max() + 0.1 * d), img.shape[0]), (60, 70, 80), -1)
    return sensor(out, rng)


def undistort_chain(cfg: Config):
    """實機流程：鏡頭畸變（標定值）→ IMX296Camera.undistort（最近鄰）。"""
    cal = cfg.camera_calib
    K_raw, dist = cal.matrix(), np.array(cal.dist, np.float64)
    K_out = cfg.camera.matrix()
    size = (cfg.camera.width, cfg.camera.height)
    gx, gy = np.meshgrid(np.arange(size[0], dtype=np.float32), np.arange(size[1], dtype=np.float32))
    ideal = cv2.undistortPoints(np.stack([gx.ravel(), gy.ravel()], 1)[:, None], K_raw, dist, P=K_out)
    fwd = ideal.reshape(size[1], size[0], 2)
    mx, my = cv2.initUndistortRectifyMap(K_raw, dist, None, K_out, size, cv2.CV_32FC1)
    inv = cv2.convertMaps(mx, my, cv2.CV_16SC2, nninterpolation=True)

    def f(img, rng):
        raw = cv2.remap(img, fwd[..., 0], fwd[..., 1], cv2.INTER_LINEAR)
        return cv2.remap(sensor(raw, rng), inv[0], None, cv2.INTER_NEAREST)
    return f


# ── 條件 ─────────────────────────────────────────────────

# 實物的基本外觀：木板、沒有亮色孔緣、看得到木管內壁、LED 只看得到上半圈（紅色、沒過曝）
REAL = dict(board=WOOD, tube=True, lit="arc_red")


def conditions(cfg: Config) -> list[tuple[str, dict, object, dict]]:
    """(名稱, FieldScene 參數, 影像處理, 幾何參數)。"""
    real_net = dict(REAL, net=(150, 150, 150))
    combo = dict(real_net, lit="arc_white", white_balls=3)
    return [
        ("對照：訓練資料外觀", dict(randomize=True), None, {}),
        ("對照：乾淨（中性孔緣）", dict(), None, {}),
        ("木板、沒有亮色孔緣（LED 整圈）", dict(board=WOOD), sensor, {}),
        ("白色板、沒有亮色孔緣（LED 整圈）", dict(board=WHITE), sensor, {}),
        ("木板 + 木管內壁 + LED 上半圈", REAL, sensor, {}),
        ("木板 + 孔內網子（暗底亮線）", real_net, sensor, {}),
        ("木板 + 孔後面亮（背光）+ 網子", dict(REAL, interior=(175, 175, 170), net=(60, 60, 60)), sensor, {}),
        ("木板 + 網內白球（3 孔）", dict(REAL, white_balls=3), sensor, {}),
        ("LED 上半圈、過曝白芯", dict(REAL, lit="arc_white"), sensor, {}),
        ("LED 只有孔內紅光", dict(REAL, lit="glow"), sensor, {}),
        ("未亮加分孔內有紅球", dict(REAL, red_ball_unlit=True), sensor, {}),
        ("動態模糊 10 px", REAL, motion_blur(10), {}),
        ("動態模糊 18 px", REAL, motion_blur(18), {}),
        ("動態模糊 25 px", REAL, motion_blur(25), {}),
        ("失焦 σ 3 px", REAL, defocus(3), {}),
        ("過暗（×0.3、高增益雜訊）", REAL, exposure(0.3, 8), {}),
        ("過曝（×2）", REAL, exposure(2.0, 2), {}),
        ("色溫偏暖（B×0.7、R×1.25）", REAL, white_balance(0.7, 1.25), {}),
        ("色溫偏冷（B×1.3、R×0.75）", REAL, white_balance(1.3, 0.75), {}),
        ("畸變 → 去畸變（最近鄰）", REAL, undistort_chain(cfg), {}),
        ("反光亮帶", REAL, glare_band, {}),
        ("遮擋左下兩孔", REAL, "occlude", {}),
        ("機器人偏離中心 1 m", REAL, sensor, dict(offset=1.0)),
        ("網框被打歪（左右傾 10°）", REAL, sensor, dict(tilt=10.0)),
        ("組合：木管 + 網子 + 白球 + LED 白芯", combo, sensor, {}),
        ("組合 + 動態模糊 10 px", combo, motion_blur(10), {}),
    ]


def scene_pose(cfg: Config, rng, offset: float | None = None, tilt: float = 3.0):
    """比賽幾何：回傳 board→cam 的 (R, t)。tilt：網板繞垂直軸傾斜的上限 (deg)。"""
    f = cfg.field
    true_cfg = dataclasses.replace(cfg, field=dataclasses.replace(
        f, dist_x=float(rng.uniform(5.8, 6.2)),
        grid_cx=float(offset if offset is not None else rng.uniform(-0.3, 0.3))))
    pitch, yaw = ControlLoop.default_survey_pose(cfg)          # 程式以為機器人在正中、6 m
    pitch += rng.normal(0, 0.1)
    yaw += rng.normal(0, 0.1)
    R_cl, t_cl = frames.camera_extrinsic_at(cfg, pitch, yaw)
    R_bl, t_bl = frames.board_to_launcher(true_cfg)
    R_bl = R_bl @ _rot_y(np.radians(rng.choice([-1, 1]) * rng.uniform(tilt - 3, tilt) if tilt > 3
                                    else rng.uniform(-3, 3))) @ _rot_x(np.radians(rng.uniform(-3, 3)))
    return R_cl.T @ R_bl, R_cl.T @ (t_bl - t_cl)


def evaluate(det, cfg: Config, name, scene_kw, post, geo, n, seed, save: Path | None):
    K = cfg.camera.matrix()
    gen = FieldScene(cfg, np.random.default_rng(seed), **scene_kw)
    geo_rng, img_rng = np.random.default_rng(seed + 1), np.random.default_rng(seed + 2)
    r = dict(pnp=0, main=0, bonus=0, lit_ok=0, lit_bad=0, id_err=0, confs=[], dets=0)
    for i in range(n):
        R, t = scene_pose(cfg, geo_rng, geo.get("offset"), geo.get("tilt", 3.0))
        s = gen.sample_pose(R, t, lit_bonus=int(geo_rng.integers(0, 3)))
        img = s.image
        if post == "occlude":
            img = occlude(img, img_rng, s)
        elif post is not None:
            img = post(img, img_rng)
        dets = det.detect(img)
        r["dets"] += len(dets)
        r["confs"] += [d.confidence for d in dets]
        est = localize_holes(dets, cfg, K)
        r["pnp"] += est.board_pose is not None
        truth = {(h.group, h.idx): h for h in s.holes}
        lit_true = s.viewpoint["lit_bonus_idx"]
        for a in est.aims:
            h = truth.get((a.group, a.idx))
            err = np.hypot(a.bearing - h.bearing, a.elev - h.elev) if h is not None else 99.0
            if err < 0.5:
                r["bonus" if a.is_bonus else "main"] += 1
            elif err > 1.0:
                r["id_err"] += 1
        lit = est.lit_bonus
        if lit is not None:
            ok = lit.idx == lit_true and np.hypot(lit.bearing - truth[("bonus", lit.idx)].bearing,
                                                  lit.elev - truth[("bonus", lit.idx)].elev) < 0.5
            r["lit_ok" if ok else "lit_bad"] += 1
        if save is not None and i == 0:
            vis = img.copy()
            col = {(False, False): (0, 165, 255), (True, False): (0, 200, 0), (True, True): (0, 0, 255)}
            for d in dets:
                x1, y1, x2, y2 = (int(v) for v in d.bbox)
                cv2.rectangle(vis, (x1, y1), (x2, y2), col[(d.is_bonus, d.is_lit)], 2)
                cv2.putText(vis, f"{d.confidence:.2f}", (x1, y1 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            col[(d.is_bonus, d.is_lit)], 2)
            cv2.imwrite(str(save / f"{list_index[name]:02d}.jpg"), vis, [cv2.IMWRITE_JPEG_QUALITY, 90])
    return r


list_index: dict[str, int] = {}


def main():
    ap = argparse.ArgumentParser(description="YOLO 壓力測試（比賽幾何 + 實際場地外觀）")
    ap.add_argument("--weights", default=str(REPO_ROOT / "models" / "grid_holes_yolo11n.pt"))
    ap.add_argument("--scenes", type=int, default=40)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--device", default="0")
    ap.add_argument("--only", nargs="*", help="只跑名稱含這些字的條件")
    ap.add_argument("--save-examples", action="store_true", help="每項存一張畫了框的圖到 runs/stress_yolo/")
    args = ap.parse_args()
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")

    from rotbot.perception.detector import YoloDetector
    cfg = load_config()
    det = YoloDetector(args.weights, conf=args.conf, device=args.device)
    save = None
    if args.save_examples:
        save = OUT_DIR
        save.mkdir(parents=True, exist_ok=True)
    n = args.scenes
    print(f"權重 {Path(args.weights).name}、每項 {n} 場、seed {args.seed}（模擬/合成評估）")
    print(f"{'#':>2} {'條件':<30} {'PnP':>6} {'一般孔':>8} {'加分孔':>8} {'亮孔對':>6} {'亮孔錯':>6} "
          f"{'編號錯':>6} {'框/場':>6} {'信心中位':>8}")
    t0 = time.time()
    for k, (name, kw, post, geo) in enumerate(conditions(cfg)):
        list_index[name] = k
        if args.only and not any(s in name for s in args.only):
            continue
        r = evaluate(det, cfg, name, kw, post, geo, n, args.seed, save)
        conf = np.median(r["confs"]) if r["confs"] else float("nan")
        print(f"{k:>2} {name:<30} {r['pnp']:>3}/{n:<2} {r['main']:>4}/{9 * n:<3} {r['bonus']:>4}/{3 * n:<3} "
              f"{r['lit_ok']:>3}/{n:<2} {r['lit_bad']:>6} {r['id_err']:>6} {r['dets'] / n:>6.1f} {conf:>8.2f}")
    print(f"完成，{time.time() - t0:.0f} s" + (f"；範例圖在 {save}" if save else ""))


if __name__ == "__main__":
    main()
