# -*- coding: utf-8 -*-
"""致動器 HAL 實作 —— Mock（模擬）與 SerialMCU（真實 Arduino / 韌體模擬器）。

MockArduino：無硬體時的替身，記錄收到的指令並回 ack（閉迴路模擬用，in-process、最快）。
SerialMCU：Orin 端的 MCU 函式庫，走協定 v1（見 protocol.py / docs/mcu_orin_link.md）。
  url 交給 pyserial 的 serial_for_url，所以同一份程式可接：
    "COM5"、"/dev/ttyACM0"          → 真 Arduino（USB 序列）
    "socket://127.0.0.1:5760"      → 電腦上跑的韌體模擬器（tools/mcu_sil.py，同一份韌體原始碼）
"""
from __future__ import annotations

import collections
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
    * Arduino Uno 開埠會被 DTR 重置（bootloader ~1.6 s）→ open() 先等 EV BOOT 再 PING。
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
        self.stats = collections.Counter()
        self.rtt_ms: collections.deque[float] = collections.deque(maxlen=2000)

    # ── 連線 ─────────────────────────────────────────
    def open(self, wait_boot: bool = True) -> "SerialMCU":
        import serial  # 局部 import：無硬體環境（CI 模擬）不必安裝 pyserial 也能用 Mock
        self._ser = serial.serial_for_url(self.url, baudrate=self.baud, timeout=0.05)
        self._stop.clear()
        t = threading.Thread(target=self._reader, name="mcu-reader", daemon=True)
        t.start()
        self._threads = [t]
        if wait_boot:                              # since=0：讀取緒啟動後就算（BOOT 可能已先到）
            self.wait_event("BOOT", timeout=self.boot_wait, since=0, raise_on_timeout=False)
        self.ping()                                # 確認雙向通
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
            self.status, self.status_time = msg.status, time.monotonic()
        w = self._waiters.get(msg.seq)
        if w is not None:
            w[1] = msg
            w[0].set()

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

    def aim(self, pitch_deg: float, yaw_deg: float) -> None:
        self.request("AIM", cdeg(pitch_deg), cdeg(yaw_deg))

    def flywheel(self, rpm: float) -> None:
        self.request("FLY", int(round(rpm)))

    def jog(self, pitch_dps: float, yaw_dps: float) -> None:
        self.request("JOG", cdeg(pitch_dps), cdeg(yaw_dps), retries=0)

    def stop(self) -> None:
        self.request("STOP")

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
