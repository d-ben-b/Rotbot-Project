# -*- coding: utf-8 -*-
"""手動操控 Web 伺服器（在 Orin 上跑；在電腦上可用韌體模擬器 + 合成相機完整驗證）。

用法：
    # 電腦上（不需任何硬體）：韌體模擬器 + 合成相機，手機/另一台筆電連同一台路由器即可操控
    python -m tools.teleop_web --mcu sil --camera sim
    # Orin：Arduino 接 USB、USB 相機
    python -m tools.teleop_web --mcu /dev/ttyACM0 --camera 0
    # Orin：CSI 相機（GStreamer 管線字串）
    python -m tools.teleop_web --mcu /dev/ttyACM0 --camera "nvarguscamerasrc ! video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! appsink drop=1"
    # 在家快速試手動：自主鎖定縮成 5 秒
    python -m tools.teleop_web --mcu sil --camera sim --auto-s 5

連線方式（詳見 docs/teleop_network.md）：
    鍵盤：任何瀏覽器開 http://<Orin IP>:8080 即可
    搖桿：瀏覽器只在安全連線開放搖桿 → 筆電先 ssh -N -L 8080:127.0.0.1:8080 user@<Orin IP>，
          再開 http://localhost:8080；或用 --tls-cert/--tls-key 走 HTTPS
"""
from __future__ import annotations

import argparse
import socket
import ssl
import sys
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from aiohttp import web

from rotbot.config import load_config
from rotbot.pipeline.state_machine import MatchStateMachine
from rotbot.teleop.server import TeleopServer


def lan_ips() -> list[str]:
    """本機區網 IP（給操作手筆電連線用）。"""
    ips = set()
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ips.add(info[4][0])
    except socket.gaierror:
        pass
    try:                                        # 預設路由那張網卡（UDP connect 不會真的送封包）
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("192.0.2.1", 9))
        ips.add(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    return sorted(ip for ip in ips if not ip.startswith("127."))


class _SimScene:
    """合成相機畫面：固定視角、亮孔固定（不會每幀換）。"""

    def __init__(self, cfg):
        from rotbot.sim.scene import SceneGenerator
        self.gen = SceneGenerator(cfg, randomize=False, rim_jitter=False)

    def render(self):
        return self.gen.sample(lit_bonus=1).image


def make_camera(spec: str, cfg, mcu=None):
    if spec == "none":
        return None
    if spec == "sim":
        if mcu is not None:                         # 數位孿生：轉塔轉到哪，畫面就看到哪
            from rotbot.sim.twin import TwinCamera
            return TwinCamera(cfg, mcu, lit_bonus=1)
        from rotbot.sim.camera import SimCamera
        return SimCamera(cfg.camera, scene=_SimScene(cfg))
    from rotbot.perception.camera import RealCamera
    return RealCamera(cfg.camera, device=int(spec) if spec.isdigit() else spec)


def main():
    ap = argparse.ArgumentParser(description="Rotbot 手動操控 Web 伺服器")
    ap.add_argument("--mcu", default="sil", help="sil / none / COM5 / /dev/ttyACM0 / socket://host:port")
    ap.add_argument("--camera", default="sim", help="sim / none / 相機編號 / GStreamer 管線")
    ap.add_argument("--host", default="0.0.0.0", help="0.0.0.0 = 區網都能連；127.0.0.1 = 只允許 SSH 通道")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--auto-s", type=float, default=60.0, help="強制自主秒數（規則 60）")
    ap.add_argument("--match-s", type=float, default=180.0)
    ap.add_argument("--rpm", type=int, default=2400, help="飛輪預設轉速（6m 設計點）")
    ap.add_argument("--tls-cert", help="HTTPS 憑證（搖桿需要安全連線）")
    ap.add_argument("--tls-key")
    args = ap.parse_args()
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")

    cfg = load_config()
    sil = None
    mcu = None
    if args.mcu != "none":
        from rotbot.actuation.link import SerialMCU
        url = args.mcu
        if args.mcu == "sil":
            from tools.mcu_sil import SilProcess
            sil = SilProcess("full").start()
            url = sil.url
            print(f"[MCU] 韌體模擬器：{sil.banner}")
        mcu = SerialMCU(url).open()
        print(f"[MCU] 已連線 {url}，PING={mcu.ping()}")
        if sil is not None:
            mcu.zero(0.0, 0.0)                     # 模擬器開機姿態視為 (0,0)
            mcu.aim(15.0, 0.0)                     # 轉到大約對準網板的掃視姿態，畫面一開就看得到
    camera = make_camera(args.camera, cfg, mcu)

    server = TeleopServer(mcu=mcu, camera=camera,
                          sm=MatchStateMachine(auto_s=args.auto_s, match_s=args.match_s),
                          rpm_preset=args.rpm)
    ssl_ctx = None
    scheme = "http"
    if args.tls_cert:
        ssl_ctx = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
        ssl_ctx.load_cert_chain(args.tls_cert, args.tls_key)
        scheme = "https"
    print("\n操控頁：")
    print(f"  本機          {scheme}://localhost:{args.port}")
    for ip in lan_ips():
        print(f"  區網          {scheme}://{ip}:{args.port}   ← 同一台路由器下的筆電/手機開這個")
    print("  （搖桿要安全連線：用 SSH 通道開 localhost，或加 --tls-cert/--tls-key）\n")
    try:
        web.run_app(server.build_app(), host=args.host, port=args.port, ssl_context=ssl_ctx,
                    print=None)
    finally:
        if mcu is not None:
            try:
                mcu.stop()
            except Exception:
                pass
            mcu.close()
        if sil is not None:
            sil.stop()


if __name__ == "__main__":
    main()
