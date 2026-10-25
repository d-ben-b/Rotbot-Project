# -*- coding: utf-8 -*-
"""相機內參標定 —— 拿棋盤格在相機前移動，自動挑畫面並算出內參。

步驟：
    1. python -m tools.calibrate_camera --make-board
         產生 runs/calibration/checkerboard_A4.pdf。用 A4 印出來（選「實際大小/100%」），
         平貼在硬紙板或墊板上。板子一定要平，彎的會讓結果不準。
    2. python -m tools.calibrate_camera
         用瀏覽器開 http://localhost:8081（別台電腦加 --lan，開 http://<Jetson IP>:8081）。
         把棋盤格拿到相機前，整張都要在畫面內。每換一個位置就停住約半秒，畫面邊框閃綠色代表收到一張。
         要涵蓋畫面的四個角落和中央，而且要有幾張是把板子傾斜著拍的。
    3. 收滿後自動計算，結果印在終端機，並存到 runs/calibration/<時間>/result.yaml。

先把鏡頭的光圈與對焦調好並鎖緊再標定；之後換鏡頭或重新對焦都要重做。
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

# 允許直接以檔案路徑執行（python tools/calibrate_camera.py）：把專案根目錄補進 sys.path
if __package__ in (None, ""):
    import os
    import sys
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rotbot.config import load_config
from rotbot.perception.camera import make_camera
from rotbot.perception.intrinsics import (PATTERN, SQUARE_M, ViewCollector, find_corners,
                                          make_board_pdf, solve_intrinsics)

OUT_ROOT = Path(__file__).resolve().parent.parent / "runs" / "calibration"
MIN_VIEWS = 8
HINTS = {"none": "沒看到完整的棋盤格（整張板子都要在畫面內，不要反光）",
         "far": "太遠了，拿近一點（棋盤格要佔畫面寬度的四分之一以上，約 0.4–0.8 m）",
         "moving": "拿穩，停住不要動…",
         "duplicate": "這個位置收過了，換個位置、距離或傾斜角度",
         "added": "✔ 收到一張"}

PAGE = """<!doctype html><meta charset="utf-8"><title>相機內參標定</title>
<style>
 body{margin:0;padding:12px 16px;background:#16181c;color:#e6e8eb;font:15px system-ui,sans-serif}
 img{max-width:100%;display:block;border:1px solid #333}
 .row{display:flex;flex-wrap:wrap;gap:8px 20px;align-items:center;margin:10px 0}
 .big{font-size:30px;font-weight:600;font-variant-numeric:tabular-nums}
 button{font:inherit;padding:6px 12px;background:#2a2d33;color:inherit;border:1px solid #444;border-radius:6px;cursor:pointer}
 pre{background:#0f1114;padding:12px;border-radius:6px;overflow-x:auto}
 .dim{color:#9aa0a6} .hint{color:#f0c674;min-height:1.4em}
</style>
<div class="row"><div>已收集 <span class="big" id="n">–</span> / <span id="target">–</span> 張</div>
 <div class="dim">畫面涵蓋 <span id="cov">–</span>%</div>
 <button onclick="fetch('/solve')">立即計算（至少 8 張）</button>
 <button onclick="fetch('/reset')">重新開始</button></div>
<div class="hint" id="hint"></div>
<div class="dim" id="missing"></div>
<pre id="result" hidden></pre>
<img id="view" alt="相機畫面：綠色區塊是已經涵蓋到的範圍">
<p class="dim">綠色區塊＝已有角點涵蓋的範圍，要把整個畫面塗滿。每個位置停住約半秒；
要有近有遠，並且有幾張把板子往左右、上下傾斜 20–40 度。</p>
<script>
const $=id=>document.getElementById(id);
async function tick(){try{const s=await (await fetch('/status')).json();
 $('n').textContent=s.n;$('target').textContent=s.target;$('cov').textContent=Math.round(s.coverage*100);
 $('hint').textContent=s.message;
 $('missing').textContent=s.missing.length&&!s.result?'還沒涵蓋到：'+s.missing.join('、'):'';
 $('result').hidden=!s.result;if(s.result)$('result').textContent=s.result;}catch(e){}
 setTimeout(tick,300)}
tick();
let seq=0,url=null;
(async function pump(){for(;;){try{
 const r=await fetch('/frame?after='+seq,{cache:'no-store'});if(!r.ok)throw 0;
 seq=+r.headers.get('X-Seq');const u=URL.createObjectURL(await r.blob());
 $('view').src=u;if(url)URL.revokeObjectURL(url);url=u;
}catch(e){await new Promise(r=>setTimeout(r,800))}}})();
</script>"""


class Session:
    """標定流程的共享狀態（擷取迴圈寫、HTTP 執行緒讀）。"""

    def __init__(self, image_size, target, pattern, square_m, out_dir: Path):
        self.image_size, self.target = image_size, target
        self.pattern, self.square_m, self.out_dir = pattern, square_m, out_dir
        self.cond = threading.Condition()
        self.jpeg, self.seq = None, 0
        self.solve_now = False
        self.reset()

    def reset(self):
        self.collector = ViewCollector(self.image_size, self.target)
        self.verdict, self.result, self.result_text = "none", None, ""
        self.flash_until = 0.0

    def status(self) -> dict:
        c = self.collector
        msg = "標定完成，結果在下方（也印在終端機）。按 Ctrl+C 結束。" if self.result \
            else HINTS[self.verdict]
        return {"n": len(c.views), "target": c.target, "coverage": c.coverage,
                "missing": c.missing_zones(), "message": msg, "result": self.result_text}

    def finish(self) -> None:
        res = solve_intrinsics(self.collector.views, self.image_size, self.pattern, self.square_m)
        from rotbot.perception.imx296 import PIXEL_PITCH_MM
        hfov, vfov = res.true_fov_deg()
        text = (f"重投影誤差 RMS：{res.rms_px:.2f} px（{res.quality()}；用了 {res.n_views} 張）\n"
                f"鏡頭實際視角（含畸變）：水平 {hfov:.1f}°、垂直 {vfov:.1f}°"
                f"（相當於 {res.output_focal() * PIXEL_PITCH_MM:.2f} mm 鏡頭）\n"
                f"畸變在畫面四角造成的位移：{res.max_distortion_px():.1f} px\n\n"
                f"把下面這段貼進（或覆蓋）configs/field.yaml：\n\n"
                f"{res.yaml_block(time.strftime('（%Y-%m-%d 標定）'))}")
        self.out_dir.mkdir(parents=True, exist_ok=True)
        (self.out_dir / "result.yaml").write_text(
            res.yaml_block(time.strftime('（%Y-%m-%d 標定）'))
            + f"# 每張的重投影誤差 (px)：{[round(e, 2) for e in res.per_view_px]}\n",
            encoding="utf-8")
        np.savez(self.out_dir / "corners.npz", views=np.array(self.collector.views),
                 image_size=self.image_size, pattern=self.pattern, square_m=self.square_m)
        self.result, self.result_text = res, text
        print("\n" + "=" * 60 + "\n" + text + f"已存：{self.out_dir / 'result.yaml'}\n" + "=" * 60,
              flush=True)


def draw_preview(frame, corners, session: Session, now: float) -> np.ndarray:
    small = cv2.resize(frame, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
    h, w = small.shape[:2]
    c = session.collector
    gx, gy = c.GRID
    tint = small.copy()
    for j in range(gy):
        for i in range(gx):
            if c.covered[j, i]:
                cv2.rectangle(tint, (i * w // gx, j * h // gy), ((i + 1) * w // gx, (j + 1) * h // gy),
                              (0, 200, 0), -1)
    small = cv2.addWeighted(tint, 0.25, small, 0.75, 0)
    if corners is not None:
        cv2.drawChessboardCorners(small, session.pattern, (corners / 2).astype(np.float32), True)
    if now < session.flash_until:
        cv2.rectangle(small, (0, 0), (w - 1, h - 1), (0, 255, 0), 12)
    return small


def capture_loop(cam, session: Session, stop: threading.Event) -> None:
    saved = 0
    while not stop.is_set():
        frame = cam.read()
        now = time.monotonic()
        corners = None
        if session.result is None:
            corners = find_corners(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), session.pattern)
            session.verdict = session.collector.offer(corners, now)
            if session.verdict == "added":
                session.flash_until = now + 0.6
                session.out_dir.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(session.out_dir / f"view_{saved:02d}.jpg"), frame,
                            [cv2.IMWRITE_JPEG_QUALITY, 95])
                saved += 1
                print(f"  收到第 {len(session.collector.views)} 張"
                      f"（涵蓋 {session.collector.coverage:.0%}）", flush=True)
            enough = len(session.collector.views) >= MIN_VIEWS
            if session.collector.done or (session.solve_now and enough):
                session.finish()
            session.solve_now = False
        ok, enc = cv2.imencode(".jpg", draw_preview(frame, corners, session, now),
                               [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            with session.cond:
                session.jpeg, session.seq = enc.tobytes(), session.seq + 1
                session.cond.notify_all()


def make_handler(session: Session):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):
            pass

        def send(self, body, ctype="text/plain; charset=utf-8", extra=()):
            body = body.encode() if isinstance(body, str) else body
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            for k, v in extra:
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            u = urlparse(self.path)
            if u.path == "/":
                self.send(PAGE, "text/html; charset=utf-8")
            elif u.path == "/status":
                self.send(json.dumps(session.status()), "application/json")
            elif u.path == "/solve":
                session.solve_now = True
                self.send("ok")
            elif u.path == "/reset":
                session.reset()
                self.send("ok")
            elif u.path == "/frame":
                after = int(parse_qs(u.query).get("after", ["0"])[0])
                with session.cond:
                    session.cond.wait_for(lambda: session.seq > after, 2.0)
                    jpg, seq = session.jpeg, session.seq
                if jpg is None:
                    self.send_error(503)
                else:
                    self.send(jpg, "image/jpeg", [("X-Seq", str(seq))])
            else:
                self.send_error(404)
    return Handler


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--make-board", action="store_true", help="只產生可列印的棋盤格 PDF 後結束")
    ap.add_argument("--resolve", nargs="+", metavar="DIR",
                    help="不開相機：把一或多次標定存下的 corners.npz 合併重新計算（給 runs/calibration/ 下的資料夾）")
    ap.add_argument("--views", type=int, default=20, help="要收集幾張（預設 20）")
    ap.add_argument("--square-mm", type=float, default=SQUARE_M * 1000,
                    help="格子邊長 (mm)；不影響內參，只影響板子距離的估計")
    ap.add_argument("--exposure-us", type=float, default=4000, help="標定時的曝光：短一點比較不會手震模糊")
    ap.add_argument("--gain-db", type=float, default=6.0)
    ap.add_argument("--port", type=int, default=8081)
    ap.add_argument("--lan", action="store_true", help="開放給區網內其他電腦（預設只有本機）")
    args = ap.parse_args()

    if args.make_board:
        OUT_ROOT.mkdir(parents=True, exist_ok=True)
        path = OUT_ROOT / "checkerboard_A4.pdf"
        make_board_pdf(str(path), PATTERN, args.square_mm)
        print(f"已產生：{path}\nA4 橫式列印，選「實際大小 / 100%」，印好平貼在硬板上。")
        return

    if args.resolve:
        views, size = [], None
        for d in args.resolve:
            data = np.load(Path(d) / "corners.npz")
            views += list(data["views"])
            size = tuple(int(x) for x in data["image_size"])
        res = solve_intrinsics(views, size, PATTERN, args.square_mm / 1000)
        hfov, vfov = res.true_fov_deg()
        print(f"合併 {len(args.resolve)} 次標定、共 {len(views)} 張 → 採用 {res.n_views} 張，"
              f"RMS {res.rms_px:.2f} px（{res.quality()}）；鏡頭實際視角 {hfov:.1f}°×{vfov:.1f}°\n")
        print(res.yaml_block(time.strftime("（%Y-%m-%d 標定）")))
        return

    cfg = load_config()
    # 標定只需要幾何，不需要準確顏色：關掉色彩矩陣比較快
    hw = dataclasses.replace(cfg.camera_hw, exposure_us=args.exposure_us,
                             gain_db=args.gain_db, ccm=False)
    # camera_calib 清掉：標定要看鏡頭原始（未去畸變）的影像
    cam = make_camera(dataclasses.replace(cfg, camera_hw=hw, camera_calib=None))
    try:
        first = cam.read()
        size = (first.shape[1], first.shape[0])
        session = Session(size, args.views, PATTERN, args.square_mm / 1000,
                          OUT_ROOT / time.strftime("%Y%m%d_%H%M%S"))
        try:
            server = ThreadingHTTPServer(("0.0.0.0" if args.lan else "127.0.0.1", args.port),
                                         make_handler(session))
        except OSError:
            raise SystemExit(f"連接埠 {args.port} 已被占用（標定工具可能已經在跑）")
        server.daemon_threads = True
        threading.Thread(target=server.serve_forever, daemon=True).start()
        print(f"相機 {size[0]}x{size[1]} 已開啟。用瀏覽器開 http://localhost:{args.port}"
              f"   （Ctrl+C 結束）", flush=True)

        stop = threading.Event()
        for s in (signal.SIGINT, signal.SIGTERM):
            signal.signal(s, lambda *_: stop.set())
        capture_loop(cam, session, stop)
        server.shutdown()
        if session.result is None:
            print(f"尚未完成標定（收了 {len(session.collector.views)} 張）。")
    finally:
        cam.release()


if __name__ == "__main__":
    main()
