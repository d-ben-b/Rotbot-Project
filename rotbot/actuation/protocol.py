# -*- coding: utf-8 -*-
"""Orin ↔ MCU（Arduino）序列協定 v1 —— ASCII 一行一幀 + CRC-8。

幀格式（雙向相同）：
    <BODY>*<CRC>\\n          BODY = TYPE,SEQ[,ARG...]（逗號分隔、只有整數）
                            CRC  = CRC-8/SMBUS（poly 0x07, init 0）對 BODY 的兩位大寫十六進位
例：  AIM,12,2513,-150*34\\n    → 轉塔轉到 俯仰 25.13°、水平 -1.50°

為什麼用 ASCII：隊友可直接在 Arduino 序列埠監控視窗閱讀/手打（MCU 可設定接受無 CRC 的手打行）；
換行自然重新同步；CRC 擋雜訊；全整數（0.01° / rpm）免去 AVR 上的浮點解析。
SEQ 1~255 循環（0=不去重）：MCU 對「同 SEQ 同內容」的重送只回上次的回覆、不重做 →
FIRE 因 ACK 遺失而重送時不會連發兩球。

指令（Orin → MCU）                         回覆
  PING                                     PONG,seq,proto,fw,uptime_ms
  STAT                                     ST,seq,flags,pitch_cdeg,yaw_cdeg,rpm,rpm_target,shots,err,crc_errors,vbat_dv
  AIM,pitch_cdeg,yaw_cdeg   轉到絕對角度      ACK,seq,0 / NAK,seq,code
  FLY,rpm                   飛輪目標轉速(0=停) ACK / NAK
  FIRE                      到位+到速才推一球  ACK（之後送 EV FIRED）/ NAK
  JOG,pitch_cdps,yaw_cdps   手動速度模式       ACK / NAK（需持續有指令，否則看門狗停）
  STOP                      停止 JOG、飛輪停   ACK
  ZERO,pitch_cdeg,yaw_cdeg  宣告目前姿態角度   ACK（AIM 前必須先 ZERO）
事件（MCU → Orin）：EV,evseq,NAME,arg   NAME ∈ BOOT/READY/FIRED/WDOG/ESTOP/FAULT
規格全文見 docs/mcu_orin_link.md；MCU 端實作見 firmware/rotbot_mcu/。
"""
from __future__ import annotations

from dataclasses import dataclass

from ..hal import AimCommand

PROTO_VERSION = 1
MAX_LINE = 64                 # 含 *CC\n 的最大行長（MCU 端緩衝區大小）
TERMINATOR = b"\n"

# NAK 錯誤碼（與 firmware/rotbot_mcu/proto.h 一致）
E_OK = 0
E_FORMAT = 1        # 參數個數/格式錯
E_UNKNOWN = 2       # 未知指令
E_RANGE = 3         # 角度/轉速超出限制
E_NOT_READY = 4     # FIRE 時尚未到位/到速
E_ESTOP = 5         # 急停中
E_BUSY = 6          # 推球進行中
E_NOT_HOMED = 7     # 尚未 ZERO 就 AIM
ERROR_NAMES = {E_OK: "OK", E_FORMAT: "FORMAT", E_UNKNOWN: "UNKNOWN", E_RANGE: "RANGE",
               E_NOT_READY: "NOT_READY", E_ESTOP: "ESTOP", E_BUSY: "BUSY",
               E_NOT_HOMED: "NOT_HOMED"}

# STAT 旗標位元
F_READY, F_MOVING, F_FLY_OK, F_FIRING = 0x01, 0x02, 0x04, 0x08
F_ESTOP, F_WDOG, F_HOMED, F_JOGGING = 0x10, 0x20, 0x40, 0x80


class ProtocolError(ValueError):
    """幀格式或 CRC 錯誤。"""


def crc8(data: bytes) -> int:
    """CRC-8/SMBUS：poly 0x07、init 0x00、不反射；crc8(b"123456789") == 0xF4。"""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def encode_line(*fields) -> bytes:
    """欄位 → 一行幀（含 CRC 與換行）。"""
    body = ",".join(str(f) for f in fields).encode("ascii")
    line = body + b"*%02X\n" % crc8(body)
    if len(line) > MAX_LINE:
        raise ProtocolError(f"幀過長 {len(line)} > {MAX_LINE}: {line!r}")
    return line


def decode_line(line: bytes, require_crc: bool = True) -> list[str]:
    """一行幀 → 欄位清單（驗 CRC）。格式/CRC 錯誤丟 ProtocolError。"""
    s = line.strip()
    if b"*" in s:
        body, _, cs = s.rpartition(b"*")
        try:
            want = int(cs, 16)
        except ValueError as e:
            raise ProtocolError(f"CRC 欄位不是十六進位: {line!r}") from e
        if len(cs) != 2 or crc8(body) != want:
            raise ProtocolError(f"CRC 錯誤: {line!r}")
    elif require_crc:
        raise ProtocolError(f"缺少 CRC: {line!r}")
    else:
        body = s
    try:
        text = body.decode("ascii")
    except UnicodeDecodeError as e:
        raise ProtocolError(f"非 ASCII: {line!r}") from e
    if not text:
        raise ProtocolError("空行")
    return text.split(",")


def cdeg(deg: float) -> int:
    """角度 → 0.01° 整數。"""
    return int(round(deg * 100))


# ══════════════════════════ 回覆解析 ══════════════════════════
@dataclass(frozen=True)
class Status:
    flags: int
    pitch_deg: float
    yaw_deg: float
    rpm: int                  # 量測轉速；MCU 沒有轉速計時為 -1
    rpm_target: int
    shots: int
    err: int                  # 最近一次 NAK 錯誤碼
    crc_errors: int           # MCU 收到 CRC 錯誤的累計數
    vbat: float | None = None # 飛輪電池電壓 (V)；MCU 沒量測時為 None

    ready = property(lambda s: bool(s.flags & F_READY))
    moving = property(lambda s: bool(s.flags & F_MOVING))
    fly_ok = property(lambda s: bool(s.flags & F_FLY_OK))
    firing = property(lambda s: bool(s.flags & F_FIRING))
    estop = property(lambda s: bool(s.flags & F_ESTOP))
    wdog = property(lambda s: bool(s.flags & F_WDOG))
    homed = property(lambda s: bool(s.flags & F_HOMED))
    jogging = property(lambda s: bool(s.flags & F_JOGGING))


@dataclass(frozen=True)
class Event:
    evseq: int
    name: str
    arg: int


@dataclass(frozen=True)
class Reply:
    kind: str                 # ACK / NAK / PONG / ST
    seq: int
    code: int = E_OK          # NAK 錯誤碼
    status: Status | None = None
    pong: tuple[int, int, int] | None = None    # (proto, fw, uptime_ms)


def parse_frame(fields: list[str]) -> Reply | Event:
    """MCU → Orin 的欄位 → Reply 或 Event。"""
    try:
        kind, n = fields[0], [int(x) for x in fields[1:] if x.lstrip("-").isdigit()]
        if kind in ("ACK", "NAK") and len(fields) == 3:
            return Reply(kind, n[0], code=n[1])
        if kind == "PONG" and len(fields) == 5:
            return Reply(kind, n[0], pong=(n[1], n[2], n[3]))
        if kind == "ST" and len(fields) == 11:
            st = Status(flags=n[1], pitch_deg=n[2] / 100, yaw_deg=n[3] / 100, rpm=n[4],
                        rpm_target=n[5], shots=n[6], err=n[7], crc_errors=n[8],
                        vbat=n[9] / 10 if n[9] >= 0 else None)
            return Reply(kind, n[0], status=st)
        if kind == "EV" and len(fields) == 4:
            return Event(int(fields[1]), fields[2], int(fields[3]))
    except (IndexError, ValueError) as e:
        raise ProtocolError(f"無法解析: {fields}") from e
    raise ProtocolError(f"未知/格式錯的回覆: {fields}")


# ══════════════════════════ 相容介面（MockArduino / 舊測試）══════════════════════════
def encode(cmd: AimCommand, seq: int = 1) -> bytes:
    """一發投球指令 → 對應的幀串（AIM + FLY [+ FIRE]）。"""
    out = encode_line("AIM", seq, cdeg(cmd.pitch_deg), cdeg(cmd.yaw_deg))
    out += encode_line("FLY", (seq % 255) + 1, int(round(cmd.flywheel)))
    if cmd.trigger:
        out += encode_line("FIRE", ((seq + 1) % 255) + 1)
    return out


def decode(data: bytes) -> AimCommand:
    """encode 的反向（MCU 端/測試用）：把幀串解回 AimCommand。"""
    pitch = yaw = fly = None
    trigger = False
    for line in data.splitlines():
        if not line.strip():
            continue
        f = decode_line(line)
        if f[0] == "AIM":
            pitch, yaw = int(f[2]) / 100, int(f[3]) / 100
        elif f[0] == "FLY":
            fly = float(f[2])
        elif f[0] == "FIRE":
            trigger = True
        else:
            raise ProtocolError(f"未知的 frame: {line!r}")
    if pitch is None or fly is None:
        raise ProtocolError(f"不完整的投球指令: {data!r}")
    return AimCommand(pitch, yaw, fly, trigger)
