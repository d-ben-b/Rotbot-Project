# -*- coding: utf-8 -*-
"""致動器 HAL 實作 —— Mock（模擬）與 Serial（真實）。

MockArduino：無硬體時的替身，記錄收到的指令並回 ack，供 P3 驗證通訊流程。
SerialArduino：P5 硬體到位後接真 Arduino（pyserial）。
"""
from __future__ import annotations

from ..hal import Ack, ActuatorLink, AimCommand
from . import protocol


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


class SerialArduino(ActuatorLink):
    """真實 Arduino（pyserial）。P5 硬體到位後啟用。"""

    def __init__(self, port: str, baud: int = 115200, timeout: float = 0.5):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self._ser = None

    def _ensure_open(self):
        if self._ser is None:
            import serial  # 局部 import：無硬體環境不需安裝 pyserial
            self._ser = serial.Serial(self.port, self.baud, timeout=self.timeout)

    def send(self, cmd: AimCommand) -> Ack:
        self._ensure_open()
        self._ser.write(protocol.encode(cmd))
        resp = self._ser.readline().decode("ascii", errors="replace").strip()
        return Ack(ok=resp.startswith("OK"), message=resp)

    def is_connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def close(self) -> None:
        if self._ser is not None:
            self._ser.close()
            self._ser = None
