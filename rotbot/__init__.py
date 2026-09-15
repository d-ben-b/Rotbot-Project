# -*- coding: utf-8 -*-
"""Rotbot —— AI 自主棒球投手機器人 軟體套件。

架構：perception（感知）→ decision（決策）→ actuation（致動），
以 hal 抽象層隔離真硬體與 sim 模擬。詳見 ROADMAP.md。
"""
from .config import Config, load_config

__all__ = ["Config", "load_config"]
