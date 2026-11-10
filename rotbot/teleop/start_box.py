# -*- coding: utf-8 -*-
"""起始框（左框、右框）的選擇 —— 決定 AI 的掃視方向、盲投與手動預估落點用的網板位置。

規則：比賽開始前機器人放在左邊或右邊的起始框（下半場兩隊猜拳決定）；AI 不開車，前 60 秒都在起始框裡投。
兩邊的 grid_cx、dist_x 事先量好寫在 field.yaml 的 start_box；網頁待命時切換，不必改檔案、重開程式。
比賽主程式的 make_loop 在按「開始」時讀 .cfg；AimPreview 透過 listeners 跟著換。
"""
from __future__ import annotations

from ..config import Config

SIDES = {"left": "左框", "right": "右框"}


class StartBoxChoice:
    def __init__(self, cfg: Config, side: str | None = None):
        """side：None = 用 field 原本的 grid_cx、dist_x（模擬與評估用的置中 6 m）。"""
        self.base = cfg
        self.listeners: list = []                  # 換框時呼叫 f(新的 cfg)
        self.side: str | None = None
        self.cfg = cfg
        if side is not None:
            self.select(side)

    def select(self, side: str) -> Config:
        if side not in SIDES:
            raise ValueError(f"起始框只能是 {list(SIDES)}，收到 {side!r}")
        self.side, self.cfg = side, self.base.at_start_box(side)
        for f in self.listeners:
            f(self.cfg)
        return self.cfg

    def describe(self) -> str:
        f = self.cfg.field
        if abs(f.grid_cx) < 0.005:
            where = f"網板在正前方 {f.dist_x:.2f} m"
        else:
            where = f"網板中心在{'右' if f.grid_cx > 0 else '左'}邊 {abs(f.grid_cx):.2f} m、前方 {f.dist_x:.2f} m"
        return f"{SIDES.get(self.side, 'field.yaml 的位置')}（{where}）"
