# -*- coding: utf-8 -*-
"""手動操控 Web 伺服器：瀏覽器協定（WebSocket）→ 伺服器 → MCU。

1) 假 MCU（快、不需編譯器）：驗證規則鎖定、駕駛權、deadman、斷線停機、急停。
2) 端到端：伺服器 → SerialMCU → 韌體模擬器（同一份 Arduino 原始碼），量指令延遲。
"""
import asyncio
import time

import aiohttp
import pytest
from aiohttp import web

from rotbot.pipeline.state_machine import MatchStateMachine
from rotbot.teleop.server import TeleopServer


class FakeMCU:
    status = None
    rtt_ms: list = []

    def __init__(self):
        self.calls = []

    def jog(self, p, y):
        self.calls.append(("jog", p, y))

    def flywheel(self, rpm):
        self.calls.append(("flywheel", rpm))

    def fire(self):
        self.calls.append(("fire",))

    def stop(self):
        self.calls.append(("stop",))

    def zero(self, p, y):
        self.calls.append(("zero", p, y))


async def _serve(server):
    runner = web.AppRunner(server.build_app())
    await runner.setup()
    site = web.TCPSite(runner, "127.0.0.1", 0)
    await site.start()
    return runner, runner.addresses[0][1]


async def _wait(ws, pred, timeout=3.0):
    end = time.monotonic() + timeout
    while True:
        msg = await asyncio.wait_for(ws.receive_json(), timeout=max(0.01, end - time.monotonic()))
        if pred(msg):
            return msg


def test_teleop_rules_with_fake_mcu():
    async def scenario():
        mcu = FakeMCU()
        server = TeleopServer(mcu=mcu, camera=None, sm=MatchStateMachine(auto_s=0.4, match_s=30))
        runner, port = await _serve(server)
        base = f"http://127.0.0.1:{port}"
        try:
            async with aiohttp.ClientSession() as s:
                r = await s.get(base + "/")
                assert r.status == 200 and "Rotbot 操控台" in await r.text()
                assert (await (await s.get(base + "/api/state")).json())["game"] == "IDLE"

                drv = await s.ws_connect(base + "/ws")
                st = await _wait(drv, lambda m: m["t"] == "state")
                assert st["role"] == "driver"
                view = await s.ws_connect(base + "/ws")
                assert (await _wait(view, lambda m: m["t"] == "state"))["role"] == "viewer"
                await view.send_json({"t": "cmd", "name": "start"})          # 觀看者不能下指令
                assert "只有駕駛" in (await _wait(view, lambda m: m["t"] == "notice"))["msg"]

                await drv.send_json({"t": "cmd", "name": "start"})
                await _wait(drv, lambda m: m["t"] == "state" and m["game"] == "AUTONOMOUS")
                await drv.send_json({"t": "input", "jog": [0, 20], "fly": 2400, "fire": True})
                n = await _wait(drv, lambda m: m["t"] == "notice" and "AI 自主中" in m["msg"])
                assert n and not mcu.calls                                    # 自主期間完全不碰 MCU
                await drv.send_json({"t": "cmd", "name": "manual"})
                assert "還不能切手動" in (await _wait(drv, lambda m: m["t"] == "notice"))["msg"]

                await asyncio.sleep(0.45)                                     # 過了自主鎖定
                await drv.send_json({"t": "cmd", "name": "manual"})
                await _wait(drv, lambda m: m["t"] == "state" and m["game"] == "MANUAL")
                for _ in range(5):
                    await drv.send_json({"t": "input", "jog": [0, 20], "fly": 0, "fire": False})
                    await asyncio.sleep(0.05)
                assert ("jog", 0.0, 20.0) in mcu.calls
                await asyncio.sleep(0.5)                                      # 停止送輸入 → deadman
                assert [c for c in mcu.calls if c[0] == "jog"][-1] == ("jog", 0.0, 0.0)

                await drv.send_json({"t": "input", "jog": [0, 0], "fly": 2400, "fire": True})
                await asyncio.sleep(0.15)
                assert ("flywheel", 2400) in mcu.calls and ("fire",) in mcu.calls

                await view.send_json({"t": "cmd", "name": "estop"})          # 任何人都能急停
                await _wait(drv, lambda m: m["t"] == "state" and m["game"] == "ESTOP")
                n_stop = mcu.calls.count(("stop",))
                assert n_stop >= 1
                await drv.close()                                             # 駕駛斷線 → 再送 STOP
                await asyncio.sleep(0.2)
                assert mcu.calls.count(("stop",)) == n_stop + 1
                await view.close()
        finally:
            await runner.cleanup()

    asyncio.run(scenario())


def test_teleop_end_to_end_with_firmware():
    from tools.mcu_sil import CompilerNotFound, SilProcess, build
    try:
        build("full")
    except CompilerNotFound as e:
        pytest.skip(str(e))
    from rotbot.actuation.link import SerialMCU

    async def scenario(mcu):
        server = TeleopServer(mcu=mcu, camera=None, sm=MatchStateMachine(auto_s=0.3, match_s=60))
        runner, port = await _serve(server)
        try:
            async with aiohttp.ClientSession() as s:
                ws = await s.ws_connect(f"http://127.0.0.1:{port}/ws")
                await ws.send_json({"t": "cmd", "name": "start"})
                await asyncio.sleep(0.4)
                await ws.send_json({"t": "cmd", "name": "manual"})
                await _wait(ws, lambda m: m["t"] == "state" and m["game"] == "MANUAL")
                y0 = mcu.get_status().yaw_deg
                t0 = time.monotonic()
                for _ in range(12):                                           # 約 0.6 s × 20°/s
                    await ws.send_json({"t": "input", "jog": [0, 20], "fly": 2400, "fire": False})
                    await asyncio.sleep(0.05)
                dur = time.monotonic() - t0
                await asyncio.sleep(0.6)                                      # deadman 停住
                y1 = mcu.get_status().yaw_deg
                await asyncio.sleep(0.3)
                y2 = mcu.get_status().yaw_deg
                # 最後一筆輸入後，還會轉到 deadman 觸發（0.3 s）+ 減速，之後必須停住
                assert 20 * dur * 0.7 < y1 - y0 < 20 * (dur + server.deadman_s + 0.25), (dur, y0, y1)
                assert abs(y2 - y1) < 0.01                                    # deadman 後不再轉
                for _ in range(60):                                           # 等 READY（到速 + 穩定）
                    await ws.send_json({"t": "input", "jog": [0, 0], "fly": 2400, "fire": False})
                    if mcu.get_status().ready:
                        break
                    await asyncio.sleep(0.05)
                await ws.send_json({"t": "input", "jog": [0, 0], "fly": 2400, "fire": True})
                await asyncio.sleep(0.8)
                assert mcu.get_status().shots == 1
                await ws.send_json({"t": "cmd", "name": "estop"})
                await asyncio.sleep(0.3)
                assert mcu.get_status().rpm_target == 0
                st = await _wait(ws, lambda m: m["t"] == "state")
                assert st["cmd_ms"] is not None and st["cmd_ms"] < 50       # 瀏覽器指令 → MCU ACK
                await ws.close()
        finally:
            await runner.cleanup()

    with SilProcess("full") as sil:
        with SerialMCU(sil.url) as mcu:
            mcu.zero(0.0, 0.0)
            asyncio.run(scenario(mcu))
