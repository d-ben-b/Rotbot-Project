# -*- coding: utf-8 -*-
"""機器人主程式整合：網頁「開始」→ AI 自主投球（真韌體模擬器）→ 切手動後 AI 立刻放手、手動接管。"""
import asyncio
import time

import aiohttp
import numpy as np
import pytest
from aiohttp import web

from rotbot.config import load_config


async def _wait(ws, pred, timeout=5.0):
    end = time.monotonic() + timeout
    while True:
        msg = await asyncio.wait_for(ws.receive_json(), timeout=max(0.01, end - time.monotonic()))
        if pred(msg):
            return msg


def test_start_autonomous_then_manual_handover():
    from tools.mcu_sil import CompilerNotFound, SilProcess, build
    try:
        build("full")
    except CompilerNotFound as e:
        pytest.skip(str(e))
    from rotbot.actuation.link import SerialMCU
    from rotbot.sim.twin import TwinCamera, TwinDetector, TwinImpactObserver
    from tools.robot_main import build_robot

    cfg = load_config()

    async def scenario(mcu, server, runner):
        app_runner = web.AppRunner(server.build_app())
        await app_runner.setup()
        site = web.TCPSite(app_runner, "127.0.0.1", 0)
        await site.start()
        port = app_runner.addresses[0][1]
        try:
            async with aiohttp.ClientSession() as s:
                ws = await s.ws_connect(f"http://127.0.0.1:{port}/ws")
                await ws.send_json({"t": "cmd", "name": "start"})              # 哨音
                await _wait(ws, lambda m: m["t"] == "state" and m["game"] == "AUTONOMOUS")
                t0 = time.monotonic()
                while mcu.get_status().shots < 2 and time.monotonic() - t0 < 10:  # AI 自己投
                    await ws.send_json({"t": "input", "jog": [0, 30], "fly": 0, "fire": True})
                    await asyncio.sleep(0.1)
                assert mcu.get_status().shots >= 2, runner.errors
                await asyncio.sleep(max(0.0, server.sm.auto_s - server.sm.elapsed) + 0.1)
                await ws.send_json({"t": "cmd", "name": "manual"})             # 第 61 秒起接手
                await _wait(ws, lambda m: m["t"] == "state" and m["game"] == "MANUAL")
                await asyncio.sleep(2.0)                                       # AI 進行中的那發會放手
                shots_at_handover = mcu.get_status().shots
                y0 = mcu.get_status().yaw_deg
                for _ in range(8):                                             # 手動 JOG
                    await ws.send_json({"t": "input", "jog": [0, 20], "fly": 0, "fire": False})
                    await asyncio.sleep(0.05)
                await asyncio.sleep(0.6)
                st = mcu.get_status()
                assert st.shots == shots_at_handover                           # 交接後 AI 沒有再發射
                assert st.yaw_deg - y0 > 3.0                                   # 手動指令有效
                await ws.close()
        finally:
            await app_runner.cleanup()

    with SilProcess("full") as sil, SerialMCU(sil.url) as mcu:
        mcu.zero(0.0, 0.0)
        cam = TwinCamera(cfg, mcu, lit_bonus=1)
        server, runner, shared = build_robot(
            cfg, mcu, cam, TwinDetector(cam, noise_px=0.5), rpm=2400, practice=9,
            observe_impact=TwinImpactObserver(cfg, np.random.default_rng(1)), auto_s=3.0, match_s=60.0)
        runner.start()
        try:
            asyncio.run(scenario(mcu, server, runner))
        finally:
            runner.stop()
            shared.release()
    assert not [e for e in runner.errors if "LinkError" in e], runner.errors
