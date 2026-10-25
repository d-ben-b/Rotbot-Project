# -*- coding: utf-8 -*-
"""賽制狀態機 —— 待命 → 前 60 秒全自主 → （操作手選擇）手動或繼續自主 → 結束；急停任何時候都可進入。

規則依據（2026_rule-1.pdf 上/下半場）：
  * 比賽開始前 60 秒須「AI 自主控制」；可用遙控器啟動，但啟動後不得再用任何遙控器/設備操控。
  * 第 61 秒起可選擇繼續自主或切換無線遠端操控。每場 3 分鐘。
→ 本狀態機保證：AUTONOMOUS 期間任何手動輸入都被拒絕；60 秒到才允許切 MANUAL；180 秒到 FINISHED。
時間來源可注入（測試用假時鐘）。
"""
from __future__ import annotations

import time
from enum import Enum


class GameState(Enum):
    IDLE = "IDLE"              # 待命（可做賽前調整，未計時）
    AUTONOMOUS = "AUTONOMOUS"  # AI 自主（前 60 秒強制；60 秒後操作手也可選擇繼續）
    MANUAL = "MANUAL"          # 無線遠端操控（第 61 秒起才允許）
    ESTOP = "ESTOP"            # 急停（軟體層；實體紅色急停按鈕另外直接斷電）
    FINISHED = "FINISHED"      # 3 分鐘到


class MatchStateMachine:
    def __init__(self, auto_s: float = 60.0, match_s: float = 180.0, clock=time.monotonic):
        self.auto_s, self.match_s, self.clock = auto_s, match_s, clock
        self.state = GameState.IDLE
        self.t_start: float | None = None
        self.log: list[tuple[float, str]] = []

    # ── 查詢 ─────────────────────────────────────────
    @property
    def elapsed(self) -> float:
        return 0.0 if self.t_start is None else self.clock() - self.t_start

    @property
    def remaining(self) -> float:
        return max(0.0, self.match_s - self.elapsed) if self.t_start is not None else self.match_s

    @property
    def autonomous_locked(self) -> bool:
        """是否仍在強制自主窗口內（禁止任何手動輸入）。"""
        return self.t_start is not None and self.elapsed < self.auto_s

    @property
    def manual_allowed(self) -> bool:
        return self.update() == GameState.MANUAL

    @property
    def bonus_window_open(self) -> bool:
        """加分孔只在前 60 秒亮（官方 Q34）。"""
        return self.autonomous_locked

    # ── 轉移 ─────────────────────────────────────────
    def _go(self, s: GameState, why: str) -> GameState:
        self.state = s
        self.log.append((self.elapsed, f"{s.value}: {why}"))
        return s

    def update(self) -> GameState:
        """依時間推進（每次查詢前呼叫）。"""
        if self.state in (GameState.AUTONOMOUS, GameState.MANUAL) and self.elapsed >= self.match_s:
            self._go(GameState.FINISHED, "比賽時間到")
        return self.state

    def start(self) -> bool:
        """哨音：開始計時並進入自主。只有 IDLE 能開始。"""
        if self.update() != GameState.IDLE:
            return False
        self.t_start = self.clock()
        self._go(GameState.AUTONOMOUS, "哨音開始，前 60 秒強制自主")
        return True

    def request_manual(self) -> bool:
        """操作手要求切手動：60 秒內一律拒絕。"""
        if self.update() != GameState.AUTONOMOUS or self.autonomous_locked:
            return False
        self._go(GameState.MANUAL, "第 61 秒起切換遠端操控")
        return True

    def request_auto(self) -> bool:
        """手動中交還 AI。"""
        if self.update() != GameState.MANUAL:
            return False
        self._go(GameState.AUTONOMOUS, "交還 AI 自主")
        return True

    def estop(self) -> None:
        self._go(GameState.ESTOP, "急停")

    def reset(self) -> None:
        """急停解除 / 賽後：回待命、計時歸零（重新開賽需再 start）。"""
        self.t_start = None
        self._go(GameState.IDLE, "重置")
