# -*- coding: utf-8 -*-
"""UART 指令協定 —— 編/解碼投球指令 frame（P3 完整實作）。

P3 待辦（見 ROADMAP §P3）：
  * 定義 frame 格式：header + 俯仰/水平/飛輪/觸發 + checksum
  * ack 與逾時處理
本檔目前提供最小文字協定，讓 MockArduino 能先驗證通訊流程。
"""
from __future__ import annotations

from ..hal import AimCommand

TERMINATOR = "\n"


def encode(cmd: AimCommand) -> bytes:
    """把指令編成一行文字 frame（P3 會換成含 checksum 的二進位 frame）。"""
    line = f"AIM,{cmd.pitch_deg:.3f},{cmd.yaw_deg:.3f},{cmd.flywheel:.3f},{int(cmd.trigger)}"
    return (line + TERMINATOR).encode("ascii")


def decode(data: bytes) -> AimCommand:
    """把一行文字 frame 解回指令（下位機端/測試用）。"""
    parts = data.decode("ascii").strip().split(",")
    if not parts or parts[0] != "AIM":
        raise ValueError(f"未知的 frame: {data!r}")
    _, pitch, yaw, flywheel, trigger = parts
    return AimCommand(float(pitch), float(yaw), float(flywheel), bool(int(trigger)))
