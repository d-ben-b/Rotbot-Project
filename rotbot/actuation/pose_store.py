# -*- coding: utf-8 -*-
"""轉塔角度存檔 —— 兩軸斷電後推不動（2026-11-10 硬體組），關機前最後的角度 = 下次開機時的角度。

步進馬達沒有角度感測，MCU 斷電後忘記歸零；Orin 把最後一筆「已歸零」的角度存在檔案裡，
下次開機 MCU 還沒歸零時，SerialMCU.open() 用它自動補回，操作手不必每次擺姿勢按「歸零」。

* SerialMCU 每收到一筆已歸零的狀態就呼叫 put()：只記下最新值、不碰檔案，
  背景執行緒最多每 min_interval_s 寫一次（寫檔 + fsync 可能要幾十 ms，不能卡住讀 MCU 回覆的執行緒）。
* 寫法：暫存檔 → fsync → os.replace。寫到一半斷電時舊檔仍完整；fsync 後才算存好，整台斷電也不會掉。
* 轉動中存的角度標 moving=True：比賽中斷電時，那一筆和實際停下的位置最多差 輪詢週期 × 轉速。
"""
from __future__ import annotations

import json
import os
import threading
import time
from pathlib import Path


class PoseStore:
    def __init__(self, path: str | Path, min_interval_s: float = 0.2, min_change_deg: float = 0.005):
        self.path = Path(path)
        self.min_interval_s, self.min_change_deg = min_interval_s, min_change_deg
        self._cond = threading.Condition()
        self._pending: tuple[float, float, bool] | None = None
        self._written: tuple[float, float, bool] | None = None
        self._busy = False                         # 背景執行緒拿走 _pending、還沒寫完
        self._stop = False
        self.writes = 0
        self.errors: list[str] = []
        self._thread = threading.Thread(target=self._writer, name="pose-store", daemon=True)
        self._thread.start()

    def load(self) -> tuple[float, float, bool] | None:
        """(俯仰, 水平, 存的時候在轉)；沒有檔案或內容壞掉回 None。"""
        try:
            d = json.loads(self.path.read_text(encoding="utf-8"))
            return float(d["pitch_deg"]), float(d["yaw_deg"]), bool(d["moving"])
        except (OSError, ValueError, KeyError, TypeError):
            return None

    def put(self, pitch_deg: float, yaw_deg: float, moving: bool) -> None:
        """記下最新角度（不阻塞）；和上次寫入的差不到 min_change_deg 且轉動狀態相同時不寫。"""
        pose = (float(pitch_deg), float(yaw_deg), bool(moving))
        with self._cond:
            w = self._written if self._pending is None else self._pending
            if (w is not None and w[2] == pose[2] and abs(w[0] - pose[0]) < self.min_change_deg
                    and abs(w[1] - pose[1]) < self.min_change_deg):
                return
            self._pending = pose
            self._cond.notify()

    def flush(self, timeout: float = 2.0) -> bool:
        """等最新一筆寫進檔案；回傳是否寫完。"""
        deadline = time.monotonic() + timeout
        with self._cond:
            while self._pending is not None or self._busy:
                left = deadline - time.monotonic()
                if left <= 0:
                    return False
                self._cond.wait(left)
        return True

    def close(self, timeout: float = 2.0) -> None:
        self.flush(timeout)
        with self._cond:
            self._stop = True
            self._cond.notify_all()
        self._thread.join(timeout)

    def _writer(self):
        last = -1e9
        while True:
            with self._cond:
                while self._pending is None and not self._stop:
                    self._cond.wait()
                if self._pending is None:
                    return
            wait = last + self.min_interval_s - time.monotonic()
            if wait > 0:                           # 限制寫入頻率；等待期間新的值會蓋掉舊的
                time.sleep(wait)
            with self._cond:
                pose, self._pending, self._busy = self._pending, None, True
            try:
                self._write(pose)
                self._written = pose
                self.writes += 1
            except OSError as e:
                self.errors.append(f"{type(e).__name__}: {e}")
            last = time.monotonic()
            with self._cond:
                self._busy = False
                self._cond.notify_all()            # 叫醒等 flush() 的人

    def _write(self, pose: tuple[float, float, bool]) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_name(self.path.name + ".tmp")
        data = dict(pitch_deg=round(pose[0], 3), yaw_deg=round(pose[1], 3), moving=pose[2],
                    saved_at=time.strftime("%Y-%m-%d %H:%M:%S"))
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(data, f)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, self.path)
        if os.name == "posix":                     # 改名本身也要落到磁碟，整台斷電才不會退回舊檔
            fd = os.open(self.path.parent, os.O_RDONLY)
            try:
                os.fsync(fd)
            finally:
                os.close(fd)
