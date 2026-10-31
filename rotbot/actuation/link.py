# -*- coding: utf-8 -*-
"""致動器 HAL 實作 —— Mock（模擬）與 SerialMCU（真實 MCU / 韌體模擬器）。

MockArduino：無硬體時的替身，記錄收到的指令並回 ack（閉迴路模擬用，in-process、最快）。
SerialMCU：Orin 端的 MCU 函式庫，走協定 v1（見 protocol.py / docs/mcu_orin_link.md）。
  url 交給 pyserial 的 serial_for_url，所以同一份程式可接：
    "COM5"、"/dev/ttyACM0"          → 真 MCU（NUCLEO-G474RE 的 ST-LINK 虛擬序列埠；Arduino Uno 也可）
    "socket://127.0.0.1:5760"      → 電腦上跑的韌體模擬器（tools/mcu_sil.py，同一份韌體原始碼）
"""
from __future__ import annotations

import collections
import os
import threading
import time

from ..hal import Ack, ActuatorLink, AimCommand
from . import protocol
from .protocol import Event, ProtocolError, Reply, Status, cdeg


class MockArduino(ActuatorLink):
    """模擬下位機：把指令編碼後解碼驗證，記錄歷史，永遠回 ok。"""

    def __init__(self):
        self.history: list[AimCommand] = []

    def send(self, cmd: AimCommand) -> Ack:
        frame = protocol.encode(cmd)
        echoed = protocol.decode(frame)          # 驗證 encode/decode round-trip
        self.history.append(echoed)
        return Ack(ok=True, message="mock-ack")

    def is_connected(self) -> bool:
        return True


ST_LINK_VID = 0x0483      # STMicroelectronics：Nucleo 板上 ST-LINK 的虛擬序列埠


def resets_on_open(url: str) -> bool:
    """開埠時 MCU 會不會重開機（會的話要等 EV BOOT）。

    Arduino Uno（16U2 / CH340）開埠時 DTR 會重置 MCU；NUCLEO 的 ST-LINK 虛擬序列埠不會，
    等 BOOT 只會白等 boot_wait。socket://（韌體模擬器）會模擬開機，視為會。查不到裝置時保守回 True。
    """
    if "://" in url:
        return True
    try:
        from serial.tools import list_ports
        dev = os.path.realpath(url) if url.startswith("/") else url      # /dev/serial/by-id/… 是連結
        for port in list_ports.comports():
            if port.device == dev or port.device == url:
                return port.vid != ST_LINK_VID
    except Exception:
        pass
    return True


class LinkError(RuntimeError):
    """通訊失敗（逾時、斷線）。"""


class NakError(LinkError):
    """MCU 明確拒絕（NAK）。"""

    def __init__(self, cmd: str, code: int):
        super().__init__(f"{cmd} 被拒：{protocol.ERROR_NAMES.get(code, code)} ({code})")
        self.cmd, self.code = cmd, code


class SerialMCU(ActuatorLink):
    """Orin ↔ MCU 連線（協定 v1）。

    * 背景讀取執行緒：解析每行、依 SEQ 交給等待中的請求；事件進佇列 / on_event 回呼。
    * 請求：分配 SEQ → 寫出 → 等回覆；逾時以「同一 SEQ」重送（MCU 去重，FIRE 不會連發）。
    * 背景輪詢 STAT（poll_hz）：同時當心跳餵 MCU 看門狗，並快取最新狀態。
    * Arduino Uno 開埠會被 DTR 重置（bootloader ~1.6 s）→ open() 先等 EV BOOT 再 PING；
      NUCLEO 的 ST-LINK 序列埠不會重置 → 直接 PING（MCU 保留上次的歸零狀態）。
    * 連線中 MCU 重開機（電壓掉、碰到 RESET）：收到 EV BOOT，或狀態從「已歸零」變「未歸零」且不是急停
      → reboots +1，pose_before_reboot 記下重開機前最後一筆角度；restore_after_reboot() 用它補回歸零。
    """

    def __init__(self, url: str, baud: int = 115200, timeout: float = 0.15, retries: int = 3,
                 boot_wait: float = 3.0, poll_hz: float = 10.0, on_event=None):
        self.url, self.baud = url, baud
        self.timeout, self.retries = timeout, retries
        self.boot_wait, self.poll_hz = boot_wait, poll_hz
        self.on_event = on_event
        self._ser = None
        self._wlock = threading.Lock()
        self._seq = 0
        self._waiters: dict[int, list] = {}        # seq -> [threading.Event, Reply|None]
        self._events: collections.deque[tuple[int, Event]] = collections.deque(maxlen=256)
        self._ev_count = 0                          # 本端收到的事件流水號（MCU 重開機也不歸零）
        self._ev_cond = threading.Condition()
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []
        self.status: Status | None = None
        self.status_time = 0.0
        self.booted = False
        self.resets_on_open = True
        self.reboots = 0                            # open() 之後偵測到的 MCU 重開機次數
        self.pose_before_reboot: tuple[float, float, bool] | None = None   # (俯仰, 水平, 當時在轉)
        self._homed_pose: tuple[float, float, bool] | None = None          # 最近一筆已歸零時的角度
        self._last_aim: tuple[float, float] | None = None                  # 最後的 AIM 目標（JOG/STOP 後清掉）
        self._last_fly = 0                                                 # 最後的飛輪指令
        self._reboot_t = -1e9
        self._opened = False
        self.stats = collections.Counter()
        self.rtt_ms: collections.deque[float] = collections.deque(maxlen=2000)

    # ── 連線 ─────────────────────────────────────────
    def open(self, wait_boot: bool = True) -> "SerialMCU":
        import serial  # 局部 import：無硬體環境（CI 模擬）不必安裝 pyserial 也能用 Mock
        self._ser = serial.serial_for_url(self.url, baudrate=self.baud, timeout=0.05)
        self._stop.clear()
        self._opened = False                       # 開埠造成的重開機（Uno、模擬器）不算
        t = threading.Thread(target=self._reader, name="mcu-reader", daemon=True)
        t.start()
        self._threads = [t]
        self.resets_on_open = resets_on_open(self.url)
        if wait_boot and self.resets_on_open:      # since=0：讀取緒啟動後就算（BOOT 可能已先到）
            self.wait_event("BOOT", timeout=self.boot_wait, since=0, raise_on_timeout=False)
        self.ping()                                # 確認雙向通
        self._opened = True
        if self.poll_hz > 0:
            p = threading.Thread(target=self._poller, name="mcu-poller", daemon=True)
            p.start()
            self._threads.append(p)
        return self

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()

    def close(self) -> None:
        self._stop.set()
        for t in self._threads:
            t.join(timeout=1.0)
        self._threads = []
        if self._ser is not None:
            try:
                self._ser.close()
            finally:
                self._ser = None

    def is_connected(self) -> bool:
        return self._ser is not None and not self._stop.is_set()

    # ── 背景執行緒 ───────────────────────────────────
    def _reader(self):
        buf = b""
        while not self._stop.is_set():
            try:
                # 先阻塞讀 1 byte（最多等 timeout），再把已到的全部讀走；
                # 直接 read(256) 會等滿 256 bytes 或逾時 → 每次回覆多 50 ms 延遲
                chunk = self._ser.read(1)
                if chunk and self._ser.in_waiting:
                    chunk += self._ser.read(self._ser.in_waiting)
            except Exception:                     # 斷線（拔 USB / 模擬器關閉）
                self.stats["io_error"] += 1
                break
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if line.strip():
                    self._handle_line(line)
            if len(buf) > 4 * protocol.MAX_LINE:  # 沒有換行的垃圾 → 丟掉
                buf = b""
                self.stats["rx_garbage"] += 1

    def _handle_line(self, line: bytes):
        try:
            msg = protocol.parse_frame(protocol.decode_line(line))
        except ProtocolError:
            self.stats["rx_bad"] += 1
            return
        self.stats["rx"] += 1
        if isinstance(msg, Event):
            if msg.name == "BOOT":
                self.booted = True
                if self._opened:
                    self._note_reboot()
            with self._ev_cond:
                self._ev_count += 1
                self._events.append((self._ev_count, msg))
                self._ev_cond.notify_all()
            if self.on_event:
                try:
                    self.on_event(msg)
                except Exception:
                    pass
            return
        if msg.kind == "ST":
            self._track_homed(msg.status)
            self.status, self.status_time = msg.status, time.monotonic()
        w = self._waiters.get(msg.seq)
        if w is not None:
            w[1] = msg
            w[0].set()

    def _track_homed(self, st: Status):
        if st.homed:
            self._homed_pose = (st.pitch_deg, st.yaw_deg, st.moving)
        elif st.estop:                             # 急停時步進斷電、轉塔可能被推動 → 舊角度不能用
            self._homed_pose = self.pose_before_reboot = None
        elif self.status is not None and self.status.homed and self._opened:
            self._note_reboot()                    # EV BOOT 那行被重開機時的雜訊弄壞也抓得到

    def _note_reboot(self):
        now = time.monotonic()
        if self._homed_pose is None and now - self._reboot_t < 1.0:
            return                                 # 同一次重開機：EV BOOT 和狀態變化各觸發一次
        if self._homed_pose is not None:
            self.pose_before_reboot, self._homed_pose = self._homed_pose, None
        self._reboot_t = now
        self.reboots += 1

    def _poller(self):
        period = 1.0 / self.poll_hz
        while not self._stop.wait(period):
            try:
                self.request("STAT", retries=0)
            except LinkError:
                self.stats["poll_fail"] += 1

    # ── 請求/回覆 ────────────────────────────────────
    def _next_seq(self) -> int:
        with self._wlock:
            self._seq = self._seq % 255 + 1          # 1..255（0 保留給「不去重」）
            return self._seq

    def write_raw(self, data: bytes) -> None:
        """直接寫位元組（測試注入雜訊/壞 CRC 用）。"""
        with self._wlock:
            self._ser.write(data)

    def request(self, cmd: str, *args: int, retries: int | None = None,
                timeout: float | None = None) -> Reply:
        """送一個指令並等回覆；NAK 丟 NakError，逾時（含重送）丟 LinkError。"""
        if self._ser is None:
            raise LinkError("尚未 open()")
        seq = self._next_seq()
        line = protocol.encode_line(cmd, seq, *args)
        retries = self.retries if retries is None else retries
        timeout = self.timeout if timeout is None else timeout
        w = [threading.Event(), None]
        self._waiters[seq] = w
        try:
            for attempt in range(retries + 1):
                t0 = time.perf_counter()
                with self._wlock:
                    self._ser.write(line)
                self.stats["tx"] += 1
                if w[0].wait(timeout):
                    self.rtt_ms.append((time.perf_counter() - t0) * 1000)
                    rep: Reply = w[1]
                    if rep.kind == "NAK":
                        self.stats["nak"] += 1
                        raise NakError(cmd, rep.code)
                    return rep
                self.stats["retry"] += 1
            raise LinkError(f"{cmd} 逾時（{retries + 1} 次 × {timeout * 1000:.0f} ms）")
        finally:
            self._waiters.pop(seq, None)

    # ── 事件 ─────────────────────────────────────────
    def event_mark(self) -> int:
        """目前的事件流水號；配合 wait_event(since=mark) 只等之後才到的事件。"""
        with self._ev_cond:
            return self._ev_count

    def wait_event(self, name: str, timeout: float = 2.0, since: int | None = None,
                   raise_on_timeout: bool = True) -> Event | None:
        """等某名稱的事件；since=event_mark() 的值（省略 = 只等呼叫之後的新事件）。"""
        deadline = time.monotonic() + timeout
        with self._ev_cond:
            mark = self._ev_count if since is None else since
            while True:
                for idx, e in self._events:
                    if idx > mark and e.name == name:
                        return e
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    if raise_on_timeout:
                        raise LinkError(f"等不到事件 {name}（{timeout:.1f} s）")
                    return None
                self._ev_cond.wait(remaining)

    def events(self) -> list[Event]:
        with self._ev_cond:
            return [e for _, e in self._events]

    # ── 高階指令 ─────────────────────────────────────
    def ping(self) -> tuple[int, int, int]:
        """回傳 (協定版本, 韌體版本, MCU 開機毫秒)；協定版本不符丟 LinkError。"""
        proto, fw, up = self.request("PING").pong
        if proto != protocol.PROTO_VERSION:
            raise LinkError(f"協定版本不符：MCU {proto} vs Orin {protocol.PROTO_VERSION}")
        return proto, fw, up

    def get_status(self) -> Status:
        return self.request("STAT").status

    def zero(self, pitch_deg: float = 0.0, yaw_deg: float = 0.0) -> None:
        self.request("ZERO", cdeg(pitch_deg), cdeg(yaw_deg))
        self._last_aim = None                      # MCU 把目標設成目前位置

    def restore_after_reboot(self, timeout: float = 2.0) -> tuple[float, float] | None:
        """MCU 重開機後補回歸零，回傳設定的 (俯仰, 水平)；沒有可用角度或已經歸零回 None。

        步進馬達沒有角度感測，角度只是 MCU 記憶體裡的步數；重開機後步數從 0 開始，轉塔仍停在原處。
        所以 ZERO 設成重開機前的角度，再加上重開機後到現在被 JOG 轉過的步數。
        之後重送重開機前的 AIM 目標與飛輪轉速，進行中的那一發不必等逾時。
        """
        pose = self.pose_before_reboot
        if pose is None:
            return None
        aim, fly = self._last_aim, self._last_fly
        st = self.get_status()
        if st.homed:
            return None
        if st.moving:                              # 未歸零時只有 JOG 能轉 → 先停下來
            self.jog(0.0, 0.0)
            deadline = time.monotonic() + timeout
            while st.moving:
                if time.monotonic() > deadline:
                    raise LinkError(f"轉塔 {timeout:.1f} s 內沒停下，無法補回歸零")
                time.sleep(0.02)
                st = self.get_status()
        p, y = pose[0] + st.pitch_deg, pose[1] + st.yaw_deg
        self.zero(p, y)
        if aim is not None:
            self.aim(*aim)
        if fly:
            self.flywheel(fly)
        return p, y

    def aim(self, pitch_deg: float, yaw_deg: float) -> None:
        self.request("AIM", cdeg(pitch_deg), cdeg(yaw_deg))
        self._last_aim = (pitch_deg, yaw_deg)

    def flywheel(self, rpm: float) -> None:
        self.request("FLY", int(round(rpm)))
        self._last_fly = int(round(rpm))

    def jog(self, pitch_dps: float, yaw_dps: float) -> None:
        self.request("JOG", cdeg(pitch_dps), cdeg(yaw_dps), retries=0)
        self._last_aim = None

    def stop(self) -> None:
        self.request("STOP")
        self._last_aim, self._last_fly = None, 0

    def fire(self) -> None:
        self.request("FIRE")

    def wait_ready(self, timeout: float = 5.0, period: float = 0.02) -> Status:
        """輪詢到 READY（轉塔到位穩定 + 飛輪到速）；逾時丟 LinkError。"""
        deadline = time.monotonic() + timeout
        while True:
            st = self.get_status()
            if st.ready:
                return st
            if st.estop:
                raise LinkError("急停中")
            if time.monotonic() > deadline:
                raise LinkError(f"{timeout:.1f} s 內未 READY（flags=0x{st.flags:02X}）")
            time.sleep(period)

    def send(self, cmd: AimCommand) -> Ack:
        """HAL 介面：一發 = 轉到角度 + 飛輪轉速 [+ 等到位到速 + 推球 + 等 FIRED]。"""
        try:
            self.aim(cmd.pitch_deg, cmd.yaw_deg)
            self.flywheel(cmd.flywheel)
            if not cmd.trigger:
                return Ack(ok=True, message="aimed")
            self.wait_ready()
            mark = self.event_mark()
            self.fire()
            ev = self.wait_event("FIRED", timeout=3.0, since=mark)
            return Ack(ok=True, message=f"fired #{ev.arg}")
        except LinkError as e:
            return Ack(ok=False, message=str(e))


# 舊名稱相容（P5 規劃時叫 SerialArduino）
SerialArduino = SerialMCU
