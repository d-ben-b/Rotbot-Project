# -*- coding: utf-8 -*-
"""Orin 手動操控 Web 伺服器 —— 瀏覽器（鍵盤/搖桿）→ WebSocket → Orin → SerialMCU → Arduino。

網路：操作手筆電 ──(Wi‑Fi/網線)── 路由器（自建區網、不接外網）── Orin：本伺服器 :8080
      規則禁止場館外遙控 → 只在區網內；場館不提供 Wi‑Fi → 自備路由器（見 docs/teleop_network.md）。
路由：
  GET /            控制頁（static/index.html）
  GET /video.mjpg  相機 MJPEG 串流（預估落點標記 + 網板小地圖 + 狀態 HUD）
  GET /api/state   目前狀態 JSON（除錯用）
  WS  /ws          控制：瀏覽器 20 Hz 送輸入、伺服器 10 Hz 推狀態、ping/pong 量延遲
安全：
  * 只有一個「駕駛」連線能下指令（規則：一個遙控器、一人操作）；其他連線只能看（急停除外）。
  * 前 60 秒（AUTONOMOUS 鎖定）一律拒絕手動輸入（規則：自主期間不得遙控）。
  * Deadman：駕駛 deadman_s 沒送輸入 → JOG、車輪歸零；駕駛斷線 → STOP（JOG、飛輪、車輪停）。
    MCU 端車輪另有 DRIVE_TIMEOUT_MS（0.5 s）沒收到 DRIVE 就停。
  * MCU 自己還有看門狗（1 s 沒指令 → 飛輪停），Orin 當掉也安全。
  * MCU 重開機（SerialMCU.reboots 變了）：用重開機前的角度自動補回歸零、重送 AIM 目標與飛輪轉速，
    網頁顯示警告直到按「重置」。
"""
from __future__ import annotations

import asyncio
import collections
import itertools
import json
import statistics
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np
from aiohttp import WSMsgType, web

from ..actuation.protocol import E_NOT_READY
from ..config import ChassisConfig
from ..pipeline.state_machine import GameState, MatchStateMachine

STATIC_DIR = Path(__file__).resolve().parent / "static"
GAME_LABEL = {"IDLE": "待命", "AUTONOMOUS": "AI 自主", "MANUAL": "手動操控",
              "ESTOP": "急停", "FINISHED": "比賽結束"}


class _Client:
    _ids = itertools.count(1)

    def __init__(self, ws: web.WebSocketResponse, peer: str):
        self.id = next(self._ids)
        self.ws, self.peer = ws, peer
        self.role = "viewer"
        self.last_input = 0.0
        self.notice_t: dict[str, float] = {}    # 各類提示上次送出時間（節流用）


class FrameSource:
    """背景執行緒抓相機畫面 → 縮圖 + HUD → JPEG，給 MJPEG 串流用。沒有相機就產生佔位畫面。"""

    def __init__(self, camera=None, width: int = 960, fps: float = 15.0, hud=None):
        self.camera, self.width, self.fps, self.hud = camera, width, fps, hud
        self.jpeg: bytes | None = None
        self.frames = 0
        self._stop = threading.Event()
        self._t: threading.Thread | None = None

    def start(self):
        self._t = threading.Thread(target=self._run, name="frame-source", daemon=True)
        self._t.start()

    def stop(self):
        self._stop.set()
        if self._t:
            self._t.join(timeout=2)

    def _run(self):
        import cv2
        period = 1.0 / self.fps
        while not self._stop.is_set():
            t0 = time.monotonic()
            try:
                if self.camera is None:
                    img = None
                elif hasattr(self.camera, "latest"):          # SharedCamera：不搶自主迴圈的新畫面
                    img = self.camera.latest()
                else:
                    img = self.camera.read()
            except Exception:
                img = None
            if img is None:
                img = np.full((540, 960, 3), 32, np.uint8)
                cv2.putText(img, "NO CAMERA", (330, 280), cv2.FONT_HERSHEY_SIMPLEX, 1.4,
                            (120, 120, 120), 2, cv2.LINE_AA)
            h, w = img.shape[:2]
            scale = 1.0
            if w > self.width:
                scale = self.width / w
                img = cv2.resize(img, (self.width, int(h * scale)), interpolation=cv2.INTER_AREA)
            if self.hud:
                self.hud(img, scale)
            ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                self.jpeg = buf.tobytes()
                self.frames += 1
            self._stop.wait(max(0.0, period - (time.monotonic() - t0)))


class TeleopServer:
    def __init__(self, mcu=None, camera=None, sm: MatchStateMachine | None = None,
                 deadman_s: float = 0.3, rpm_preset: int = 2400, max_dps: float = 60.0,
                 jog_refresh_s: float = 0.2, video_fps: float = 15.0, preview=None, chassis=None):
        self.mcu = mcu                      # SerialMCU（或測試替身）；None = 乾跑
        self.preview = preview              # AimPreview：預估落點（None = 不顯示）
        self.chassis = chassis or ChassisConfig()
        self._K, self._K_tried = None, False
        self.sm = sm or MatchStateMachine()
        self.deadman_s, self.rpm_preset, self.max_dps = deadman_s, rpm_preset, max_dps
        self.jog_refresh_s = jog_refresh_s
        self.clients: set[_Client] = set()
        self.driver: _Client | None = None
        self.last_jog = (0.0, 0.0)
        self.last_jog_t = 0.0
        self.last_drive = (0.0, 0.0, 0.0)   # (前進, 右移, 右轉) -1..1
        self.last_drive_t = 0.0
        self.last_fly = 0
        self.last_error = ""
        self.last_nak = None                # 最近一次 NAK 的錯誤碼（None = 不是 NAK）
        self.mcu_warning = ""
        self._reboots_seen = 0
        self.cmd_ms: collections.deque[float] = collections.deque(maxlen=200)
        self.dry_log: collections.deque[tuple] = collections.deque(maxlen=200)
        self._exec = ThreadPoolExecutor(max_workers=1, thread_name_prefix="mcu")   # MCU 呼叫序列化
        self._tasks: list[asyncio.Task] = []
        self._prev_state = self.sm.state
        self.frames = FrameSource(camera, fps=video_fps, hud=self._draw_hud)

    # ── MCU 呼叫（在執行緒池跑，避免卡住事件迴圈）────────────────
    async def _mcu(self, name: str, *args):
        if self.mcu is None:
            self.dry_log.append((time.monotonic(), name, args))
            return True
        loop = asyncio.get_running_loop()
        t0 = time.perf_counter()
        try:
            await loop.run_in_executor(self._exec, lambda: getattr(self.mcu, name)(*args))
            self.cmd_ms.append((time.perf_counter() - t0) * 1000)
            return True
        except Exception as e:                       # LinkError / NakError
            self.last_error = f"{name}: {e}"
            self.last_nak = getattr(e, "code", None)
            return False

    def _fire_blockers(self) -> str:
        """FIRE 被拒（NOT_READY）時，依最近的 MCU 狀態列出還缺哪些條件（韌體 App::ready()）。"""
        s = getattr(self.mcu, "status", None)
        if s is None:
            return ""
        why = []
        if not s.homed:
            why.append("還沒歸零（按「重置」回待命，再按「歸零」）")
        if s.estop:
            why.append("急停中")
        if s.wdog:
            why.append("看門狗觸發")
        if s.firing:
            why.append("上一球還在推")
        if s.rpm_target <= 0:
            why.append("飛輪沒開")
        elif not s.fly_ok:
            why.append("飛輪還沒到速")
        if s.moving:
            why.append("轉塔還在動")
        return "、".join(why)

    async def _safe_stop(self, why: str):
        self.last_jog, self.last_fly, self.last_drive = (0.0, 0.0), 0, (0.0, 0.0, 0.0)   # STOP 也停車輪
        await self._mcu("stop")
        self.last_error = self.last_error if self.last_error else ""
        self._log(f"STOP（{why}）")

    def _log(self, msg: str):
        self.sm.log.append((self.sm.elapsed, msg))

    # ── 狀態 ─────────────────────────────────────────
    def snapshot(self, client: _Client | None = None) -> dict:
        st = self.sm.update()
        mcu = None
        if self.mcu is not None and getattr(self.mcu, "status", None) is not None:
            s = self.mcu.status
            mcu = dict(flags=s.flags, pitch=s.pitch_deg, yaw=s.yaw_deg, rpm=s.rpm,
                       rpm_target=s.rpm_target, shots=s.shots, ready=s.ready, moving=s.moving,
                       fly_ok=s.fly_ok, estop=s.estop, wdog=s.wdog, homed=s.homed, vbat=s.vbat)
        rtt = list(getattr(self.mcu, "rtt_ms", []))[-50:] if self.mcu is not None else []
        return dict(
            t="state", game=st.value, game_label=GAME_LABEL[st.value],
            elapsed=round(self.sm.elapsed, 2), remaining=round(self.sm.remaining, 2),
            auto_locked=self.sm.autonomous_locked,
            lock_left=round(max(0.0, self.sm.auto_s - self.sm.elapsed), 1) if self.sm.t_start else None,
            role=client.role if client else None, has_driver=self.driver is not None,
            viewers=len(self.clients), mcu_connected=self.mcu is not None, mcu=mcu,
            mcu_rtt_ms=round(statistics.median(rtt), 2) if rtt else None,
            cmd_ms=round(statistics.median(list(self.cmd_ms)[-50:]), 2) if self.cmd_ms else None,
            jog=self.last_jog, fly=self.last_fly, drive=self.last_drive,
            rpm_preset=self.rpm_preset, max_dps=self.max_dps,
            last_error=self.last_error, mcu_warning=self.mcu_warning,
            video_frames=self.frames.frames, preview=self._preview_info())

    # ── 預估落點 ─────────────────────────────────────
    def _intrinsics(self):
        """相機內參（第一次用時取一次）；沒有相機或還沒標定就是 None（只顯示小地圖與文字）。"""
        if not self._K_tried:
            self._K_tried = True
            cam = self.frames.camera
            try:
                self._K = None if cam is None else np.asarray(cam.intrinsics(), float)
            except Exception:
                self._K = None
        return self._K

    def _predict(self):
        """依 MCU 回報的轉塔角度與飛輪目標轉速算預估落點；飛輪沒開時用預設轉速估。"""
        s = getattr(self.mcu, "status", None) if self.mcu is not None else None
        if self.preview is None or s is None:
            return None, False
        fly_on = s.rpm_target > 0
        try:
            pv = self.preview.predict(s.pitch_deg, s.yaw_deg, s.rpm_target if fly_on else self.rpm_preset,
                                      self._intrinsics())
        except Exception:
            return None, fly_on
        return pv, fly_on

    def _preview_info(self) -> dict | None:
        if self.preview is None or getattr(self.mcu, "status", None) is None:
            return None
        pv, fly_on = self._predict()
        if pv is None:
            return dict(ok=False, fly_on=fly_on)
        return dict(ok=True, fly_on=fly_on, hole=pv.hole, label=pv.label, inside=bool(pv.inside),
                    off_cm=[round(float(v), 1) for v in pv.off_cm],
                    z=round(float(pv.z), 3), y=round(float(pv.y), 3), v0=round(float(pv.v0), 2), rpm=pv.rpm)

    def _draw_hud(self, img, scale: float = 1.0):
        import cv2
        h, w = img.shape[:2]
        st = self.sm.state.value
        txt = f"{st}  t={self.sm.elapsed:5.1f}s"
        s = getattr(self.mcu, "status", None) if self.mcu is not None else None
        if s is not None:
            txt += f"  P {s.pitch_deg:6.2f}  Y {s.yaw_deg:6.2f}  {s.rpm_target}rpm"
            if s.ready:
                txt += "  READY"
        if self.preview is not None and s is not None:     # 畫面中央不再畫準星：它是相機正前方，不是落點
            self.preview.draw(img, self._predict()[0], scale)
        cv2.rectangle(img, (0, 0), (w, 26), (0, 0, 0), -1)
        cv2.putText(img, txt, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (230, 230, 230), 1, cv2.LINE_AA)

    async def _send(self, c: _Client, msg: dict):
        try:
            await c.ws.send_str(json.dumps(msg, ensure_ascii=False))
        except (ConnectionResetError, RuntimeError):
            pass

    async def _notice(self, c: _Client, text: str, throttle: float = 0.0, key: str = ""):
        """送提示；throttle>0 時同一 key 在 throttle 秒內只送一次（例如 20 Hz 輸入被拒時）。"""
        now = time.monotonic()
        if throttle and now - c.notice_t.get(key, -1e9) < throttle:
            return
        c.notice_t[key] = now
        await self._send(c, dict(t="notice", msg=text))

    # ── MCU 重開機 ───────────────────────────────────
    async def _check_reboot(self):
        n = getattr(self.mcu, "reboots", 0)
        if n == self._reboots_seen:
            return
        self._reboots_seen = n
        pose = self.mcu.pose_before_reboot
        head = f"MCU 重開機（第 {n} 次）"
        if pose is None:
            msg = f"{head}：重開機前沒有歸零，無法自動補回；回待命、擺好起始姿態後按「歸零」"
        else:
            loop = asyncio.get_running_loop()
            try:                                     # 走同一個執行緒池：補回期間不會插進 JOG
                got = await loop.run_in_executor(self._exec, self.mcu.restore_after_reboot)
            except Exception as e:
                got, msg = None, f"{head}：補回歸零失敗（{e}）"
            else:
                msg = f"{head}：已經重新歸零"
            if got is not None:
                msg = f"{head}：已用重開機前的角度補回歸零（俯仰 {got[0]:.2f}°、水平 {got[1]:.2f}°）"
                if pose[2]:
                    msg += "；當時轉塔在轉，角度可能有誤差"
        self.mcu_warning = msg
        self._log(msg)
        for c in list(self.clients):
            await self._notice(c, msg)

    # ── 背景工作 ─────────────────────────────────────
    async def _broadcaster(self):
        while True:
            await self._check_reboot()
            st = self.sm.update()
            if st != self._prev_state:                       # 進入急停/結束/交還 AI → 停手動動作
                if st in (GameState.ESTOP, GameState.FINISHED, GameState.AUTONOMOUS) \
                        and self._prev_state == GameState.MANUAL:
                    await self._safe_stop(f"狀態 {self._prev_state.value}→{st.value}")
                self._prev_state = st
            for c in list(self.clients):
                await self._send(c, self.snapshot(c))
            await asyncio.sleep(0.1)

    async def _deadman(self):
        while True:
            await asyncio.sleep(0.05)
            if self.sm.state != GameState.MANUAL:
                continue
            stale = self.driver is None or time.monotonic() - self.driver.last_input > self.deadman_s
            if stale and self.last_jog != (0.0, 0.0):
                self.last_jog = (0.0, 0.0)
                await self._mcu("jog", 0.0, 0.0)
                self._log("deadman：駕駛輸入中斷 → JOG 歸零")
            if stale and self.last_drive != (0.0, 0.0, 0.0):
                self.last_drive = (0.0, 0.0, 0.0)
                await self._mcu("drive", 0.0, 0.0, 0.0, 0.0)
                self._log("deadman：駕駛輸入中斷 → 車輪停")

    async def _on_startup(self, app):
        self.frames.start()
        self._tasks = [asyncio.create_task(self._broadcaster()), asyncio.create_task(self._deadman())]

    async def _on_cleanup(self, app):
        for t in self._tasks:
            t.cancel()
        self.frames.stop()
        self._exec.shutdown(wait=False)

    # ── HTTP ─────────────────────────────────────────
    async def index(self, request):
        return web.FileResponse(STATIC_DIR / "index.html")

    async def api_state(self, request):
        return web.json_response(self.snapshot())

    async def video(self, request):
        resp = web.StreamResponse(headers={
            "Content-Type": "multipart/x-mixed-replace; boundary=frame",
            "Cache-Control": "no-cache, no-store"})
        await resp.prepare(request)
        last = -1
        try:
            while True:
                if self.frames.frames != last and self.frames.jpeg:
                    last = self.frames.frames
                    jpg = self.frames.jpeg
                    await resp.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                     + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                await asyncio.sleep(0.02)
        except (ConnectionResetError, asyncio.CancelledError):
            pass
        return resp

    # ── WebSocket ────────────────────────────────────
    async def ws_handler(self, request):
        ws = web.WebSocketResponse(heartbeat=5.0)
        await ws.prepare(request)
        c = _Client(ws, request.remote or "?")
        self.clients.add(c)
        if self.driver is None:
            self.driver, c.role = c, "driver"
        self._log(f"連線 #{c.id} {c.peer}（{c.role}）")
        await self._send(c, self.snapshot(c))
        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    try:
                        await self._on_message(c, json.loads(msg.data))
                    except (ValueError, KeyError, TypeError) as e:
                        await self._notice(c, f"訊息格式錯誤：{e}")
        finally:
            self.clients.discard(c)
            if self.driver is c:
                self.driver = None
                await self._safe_stop(f"駕駛 #{c.id} 斷線")
        return ws

    async def _on_message(self, c: _Client, m: dict):
        t = m.get("t")
        if t == "ping":
            await self._send(c, dict(t="pong", ts=m.get("ts")))
        elif t == "claim":                                    # 取得駕駛權
            if self.driver is None or m.get("force"):
                if self.driver is not None and self.driver is not c:
                    self.driver.role = "viewer"
                    await self._notice(self.driver, "駕駛權已被其他連線取走")
                self.driver, c.role = c, "driver"
                c.last_input = time.monotonic()
        elif t == "cmd":
            await self._on_cmd(c, str(m.get("name")))
        elif t == "input":
            await self._on_input(c, m)

    async def _on_cmd(self, c: _Client, name: str):
        if name == "estop":                                   # 任何連線都能急停
            self.sm.estop()
            await self._safe_stop(f"#{c.id} 按下軟體急停")
            return
        if c is not self.driver:
            await self._notice(c, "只有駕駛能下指令（按「取得控制權」）")
            return
        if name == "start":
            ok = self.sm.start()
            await self._notice(c, "哨音：開始計時，前 60 秒 AI 自主（手動輸入一律拒絕）" if ok
                               else "只有待命狀態能開始")
        elif name == "manual":
            if not self.sm.request_manual():
                left = max(0.0, self.sm.auto_s - self.sm.elapsed) if self.sm.t_start else None
                await self._notice(c, f"還不能切手動：自主鎖定剩 {left:.0f} 秒" if left
                                   else f"目前狀態 {self.sm.state.value} 不能切手動")
            else:
                c.last_input = time.monotonic()
        elif name == "auto":
            if self.sm.request_auto():
                await self._safe_stop("交還 AI")
        elif name == "reset":
            self.sm.reset()
            self.last_error = self.mcu_warning = ""
        elif name == "stop":
            await self._safe_stop(f"#{c.id} 按 STOP")
        elif name == "zero":
            if self.sm.state != GameState.IDLE:
                await self._notice(c, "只能在待命時歸零（擺好起始姿態後按）")
            elif await self._mcu("zero", 0.0, 0.0):
                await self._notice(c, "已把目前姿態設為 (0°, 0°)")
        else:
            await self._notice(c, f"未知指令 {name}")

    async def _on_input(self, c: _Client, m: dict):
        if c is not self.driver:
            return
        c.last_input = time.monotonic()
        st = self.sm.update()
        if st != GameState.MANUAL:
            if st == GameState.AUTONOMOUS and self.sm.autonomous_locked:
                left = self.sm.auto_s - self.sm.elapsed
                await self._notice(c, f"AI 自主中，手動輸入被拒（剩 {left:.0f} 秒）",
                                   throttle=1.0, key="locked")
            return
        jp, jy = (float(v) for v in m.get("jog", (0.0, 0.0)))
        jp = max(-self.max_dps, min(self.max_dps, jp))
        jy = max(-self.max_dps, min(self.max_dps, jy))
        now = time.monotonic()
        if (jp, jy) != self.last_jog or ((jp, jy) != (0.0, 0.0) and now - self.last_jog_t > self.jog_refresh_s):
            self.last_jog, self.last_jog_t = (jp, jy), now
            await self._mcu("jog", jp, jy)
        dv = m.get("drive")
        if dv is not None:
            d = tuple(max(-1.0, min(1.0, float(v))) for v in (list(dv) + [0, 0, 0])[:3])   # (前進, 右移, 右轉)
            if d != self.last_drive or (any(d) and now - self.last_drive_t > self.jog_refresh_s):
                self.last_drive, self.last_drive_t = d, now
                await self._mcu("drive", *self.mix(*d))
        fly = m.get("fly")
        if fly is not None:
            fly = int(max(0, min(4000, int(fly))))
            if fly != self.last_fly:
                self.last_fly = fly
                await self._mcu("flywheel", fly)
        if m.get("fire"):
            if await self._mcu("fire"):
                await self._notice(c, "發射！")
            elif self.last_nak == E_NOT_READY and (why := self._fire_blockers()):
                self.last_error = f"不能發射：{why}"           # 頁面一直顯示 last_error，換成看得懂的原因
                await self._notice(c, self.last_error)
            else:
                await self._notice(c, f"不能發射：{self.last_error}")

    def mix(self, fwd: float, strafe: float, rot: float) -> tuple[float, float, float, float]:
        """（前進, 右移, 右轉）→ 四輪輸出（M1 左前、M2 左後、M3 右前、M4 右後）。

        麥克納姆輪 X 型排列：右移 = 左前、右後往前轉，左後、右前往後轉。一般輪時橫移無效（差速）。
        """
        if not self.chassis.mecanum:
            strafe = 0.0
        elif self.chassis.strafe_invert:
            strafe = -strafe
        w = (fwd + strafe + rot, fwd - strafe + rot, fwd - strafe - rot, fwd + strafe - rot)
        k = max(1.0, *(abs(x) for x in w))
        return tuple(x / k for x in w)

    # ── 組裝 ─────────────────────────────────────────
    def build_app(self) -> web.Application:
        app = web.Application()
        app.router.add_get("/", self.index)
        app.router.add_get("/api/state", self.api_state)
        app.router.add_get("/video.mjpg", self.video)
        app.router.add_get("/ws", self.ws_handler)
        app.on_startup.append(self._on_startup)
        app.on_cleanup.append(self._on_cleanup)
        return app
