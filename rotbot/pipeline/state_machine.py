# -*- coding: utf-8 -*-
"""賽制狀態機 —— 啟動 → 前 60 秒全自主 → 後半場 → 安全停止（P3 實作）。

P3 待辦（見 ROADMAP §P3）：
  * 狀態：IDLE → AUTONOMOUS(前60s，純機上視覺無無線) → MANUAL → ESTOP
  * 紅色急停、通訊逾時、偵測失敗的轉移與 fallback
本檔目前只定義狀態列舉，佔位。
"""
from __future__ import annotations

from enum import Enum, auto


class GameState(Enum):
    IDLE = auto()          # 待命
    AUTONOMOUS = auto()    # 前 60 秒全自主（純機上視覺、無無線）
    MANUAL = auto()        # 後半場手動
    ESTOP = auto()         # 緊急停止
