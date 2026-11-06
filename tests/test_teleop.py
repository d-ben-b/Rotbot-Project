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


def test_teleop_explains_fire_not_ready():
    """FIRE 被拒（NOT_READY）時，頁面顯示缺哪個條件，不是只有錯誤碼。"""
    from rotbot.actuation.link import NakError
    from rotbot.actuation.protocol import E_NOT_READY, F_FLY_OK, F_HOMED, Status

    class NotReadyMCU(FakeMCU):
        def fire(self):
            raise NakError("FIRE", E_NOT_READY)

    async def scenario():
        mcu = NotReadyMCU()
        server = TeleopServer(mcu=mcu, camera=None, sm=MatchStateMachine(auto_s=0.0, match_s=30))
        runner, port = await _serve(server)
        try:
            async with aiohttp.ClientSession() as s:
                ws = await s.ws_connect(f"http://127.0.0.1:{port}/ws")
                await ws.send_json({"t": "cmd", "name": "start"})
                await ws.send_json({"t": "cmd", "name": "manual"})
                await _wait(ws, lambda m: m["t"] == "state" and m["game"] == "MANUAL")
                for flags, rpm_t, want in ((F_FLY_OK, 900, "還沒歸零"), (F_HOMED, 900, "飛輪還沒到速"),
                                           (F_HOMED, 0, "飛輪沒開")):
                    mcu.status = Status(flags, -5.0, 0.0, -1, rpm_t, 0, E_NOT_READY, 0)
                    await ws.send_json({"t": "input", "jog": [0, 0], "fly": rpm_t, "fire": True})
                    n = await _wait(ws, lambda m: m["t"] == "notice" and "不能發射" in m["msg"])
                    assert want in n["msg"] and "NOT_READY" not in n["msg"], n["msg"]
                    assert want in server.last_error
                await ws.close()
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


def test_teleop_restores_zero_after_mcu_reboot():
    """手動中 MCU 重開機：網頁跳警告、自動補回歸零、飛輪轉速恢復；按「重置」後警告消失。"""
    from tools.mcu_sil import CompilerNotFound, SilProcess, build
    try:
        build("full")
    except CompilerNotFound as e:
        pytest.skip(str(e))
    from rotbot.actuation.link import SerialMCU

    async def scenario(mcu):
        server = TeleopServer(mcu=mcu, camera=None, sm=MatchStateMachine(auto_s=0.2, match_s=60))
        runner, port = await _serve(server)
        try:
            async with aiohttp.ClientSession() as s:
                ws = await s.ws_connect(f"http://127.0.0.1:{port}/ws")
                await ws.send_json({"t": "cmd", "name": "start"})
                await asyncio.sleep(0.3)
                await ws.send_json({"t": "cmd", "name": "manual"})
                await _wait(ws, lambda m: m["t"] == "state" and m["game"] == "MANUAL")
                await ws.send_json({"t": "input", "jog": [0, 0], "fly": 2400, "fire": False})
                await asyncio.sleep(0.3)
                mcu.write_raw(b"!RESET\n")
                n = await _wait(ws, lambda m: m["t"] == "notice" and "MCU 重開機" in m["msg"])
                assert "俯仰 12.00°、水平 -5.00°" in n["msg"], n["msg"]
                st = mcu.get_status()
                assert st.homed and (st.pitch_deg, st.yaw_deg) == (12.0, -5.0)
                assert st.rpm_target == 2400
                await _wait(ws, lambda m: m["t"] == "state" and m["mcu_warning"] == n["msg"])
                await ws.send_json({"t": "cmd", "name": "reset"})
                await _wait(ws, lambda m: m["t"] == "state" and m["game"] == "IDLE" and not m["mcu_warning"])
                await ws.close()
        finally:
            await runner.cleanup()

    with SilProcess("full") as sil:
        with SerialMCU(sil.url) as mcu:
            mcu.zero(0.0, 0.0)
            mcu.aim(12.0, -5.0)
            for _ in range(200):
                st = mcu.get_status()
                if not st.moving and (st.pitch_deg, st.yaw_deg) == (12.0, -5.0):
                    break
                time.sleep(0.02)
            asyncio.run(scenario(mcu))


def test_teleop_drive_mixing_and_deadman():
    """手動時方向鍵/左搖桿 → 麥克納姆輪四輪輸出；自主期間不開車；輸入中斷 → 車輪停。"""
    class DriveMCU(FakeMCU):
        def drive(self, *w):
            self.calls.append(("drive", *w))

    async def scenario():
        mcu = DriveMCU()
        server = TeleopServer(mcu=mcu, camera=None, sm=MatchStateMachine(auto_s=0.3, match_s=30))
        runner, port = await _serve(server)
        try:
            async with aiohttp.ClientSession() as s:
                ws = await s.ws_connect(f"http://127.0.0.1:{port}/ws")
                await ws.send_json({"t": "cmd", "name": "start"})
                await _wait(ws, lambda m: m["t"] == "state" and m["game"] == "AUTONOMOUS")
                await ws.send_json({"t": "input", "jog": [0, 0], "drive": [1, 0], "fly": 0})
                await asyncio.sleep(0.1)
                assert not [c for c in mcu.calls if c[0] == "drive"]           # 自主期間不開車
                await asyncio.sleep(0.3)
                await ws.send_json({"t": "cmd", "name": "manual"})
                await _wait(ws, lambda m: m["t"] == "state" and m["game"] == "MANUAL")
                # (前進, 右移, 右轉) → (M1 左前, M2 左後, M3 右前, M4 右後)，麥克納姆輪 X 型
                for drive, want in (([1, 0, 0], (1, 1, 1, 1)), ([0, 1, 0], (1, -1, -1, 1)),
                                    ([0, 0, 1], (1, 1, -1, -1)), ([1, 1, 0], (1, 0, 0, 1))):
                    await ws.send_json({"t": "input", "jog": [0, 0], "drive": drive, "fly": 0})
                    await asyncio.sleep(0.05)
                    assert mcu.calls[-1] == ("drive", *want), mcu.calls[-1]
                await asyncio.sleep(0.5)                                      # 停止送輸入 → deadman
                assert mcu.calls[-1] == ("drive", 0.0, 0.0, 0.0, 0.0)
                await _wait(ws, lambda m: m["t"] == "state" and m["drive"] == [0.0, 0.0, 0.0])   # 網頁也顯示歸零
                await ws.close()
        finally:
            await runner.cleanup()

    asyncio.run(scenario())


def test_chassis_mix_options():
    """一般輪（差速）時橫移無效；strafe_invert 讓橫移反向；輸出超過 1 時等比例縮小。"""
    from rotbot.config import ChassisConfig
    tank = TeleopServer(chassis=ChassisConfig(mecanum=False))
    assert tank.mix(0, 1, 0) == (0, 0, 0, 0)
    assert tank.mix(1, 0, 1) == (1, 1, 0, 0)
    inv = TeleopServer(chassis=ChassisConfig(strafe_invert=True))
    assert inv.mix(0, 1, 0) == (-1, 1, 1, -1)
    assert TeleopServer().mix(1, 1, 1) == (1, 1 / 3, -1 / 3, 1 / 3)
