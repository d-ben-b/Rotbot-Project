# -*- coding: utf-8 -*-
"""程序化材質 —— 用 numpy/cv2 生成貼圖 PNG（零下載），供 PyBullet loadTexture。

生成並快取於 sim/assets/tex_*.png：木紋網板、深色網子、賽場地板、棒球縫線、金屬。
目的：讓 PyBullet 沙盒不像除錯畫面，更接近真實賽場，同時讓孔在鏡頭裡變暗（縮小 YOLO 域差）。
"""
from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

ASSETS_DIR = Path(__file__).resolve().parent / "assets"


def _save_rgb(name: str, rgb: np.ndarray) -> str:
    ASSETS_DIR.mkdir(parents=True, exist_ok=True)
    path = ASSETS_DIR / name
    cv2.imwrite(str(path), rgb[:, :, ::-1])          # 存成 BGR，讓載入時得回 RGB
    return str(path)


def _wood(size: int = 512) -> np.ndarray:
    rng = np.random.default_rng(1)
    base = np.array([170, 135, 90], np.float32)      # 淺木色 (RGB)
    img = np.tile(base, (size, size, 1))
    y = np.linspace(0, 30 * np.pi, size)
    grain = (np.sin(y)[:, None] * 10 + rng.normal(0, 6, (size, size)))  # 橫向木紋
    img += grain[:, :, None]
    for _ in range(6):                               # 幾條深色紋理帶
        r = int(rng.integers(0, size))
        img[max(0, r - 2):r + 2] *= 0.82
    return np.clip(img, 0, 255).astype(np.uint8)


def _net(size: int = 512) -> np.ndarray:
    """深色網子：暗底 + 細網格線。孔後放這片 → 鏡頭裡孔會變暗（像真網框後方）。"""
    img = np.full((size, size, 3), (34, 38, 42), np.uint8)
    step = size // 26
    for i in range(0, size, step):
        img[i:i + 1, :] = (60, 66, 72)
        img[:, i:i + 1] = (60, 66, 72)
    return img


def _floor(size: int = 512) -> np.ndarray:
    rng = np.random.default_rng(2)
    base = np.array([120, 122, 128], np.float32)     # 灰色場地
    img = np.tile(base, (size, size, 1)) + rng.normal(0, 8, (size, size, 1))
    step = size // 4                                  # 淡淡的地磚縫
    for i in range(0, size, step):
        img[i:i + 2, :] *= 0.85
        img[:, i:i + 2] *= 0.85
    return np.clip(img, 0, 255).astype(np.uint8)


def _metal(size: int = 256) -> np.ndarray:
    rng = np.random.default_rng(3)
    base = np.array([115, 120, 128], np.float32)
    img = np.tile(base, (size, size, 1)) + rng.normal(0, 5, (size, size, 1))
    return np.clip(img, 0, 255).astype(np.uint8)


def _baseball(size: int = 256) -> np.ndarray:
    """白球 + 兩道紅色縫線（球旋轉時可見，賣真實感）。"""
    img = np.full((size, size, 3), (245, 245, 240), np.uint8)
    xs = np.arange(size)
    for cx in (size * 0.30, size * 0.70):
        ys = (cx + 0.16 * size * np.sin(2 * np.pi * xs / size * 2)).astype(int)
        for k in (-1, 0, 1):
            yk = np.clip(ys + k, 0, size - 1)
            img[yk, xs] = (200, 40, 40)
    return img


_CACHE: dict[str, str] = {}


def paths() -> dict[str, str]:
    """回傳 {name: png_path}，第一次呼叫時生成並快取。"""
    if _CACHE:
        return _CACHE
    _CACHE.update(
        wood=_save_rgb("tex_wood.png", _wood()),
        net=_save_rgb("tex_net.png", _net()),
        floor=_save_rgb("tex_floor.png", _floor()),
        metal=_save_rgb("tex_metal.png", _metal()),
        baseball=_save_rgb("tex_baseball.png", _baseball()),
    )
    return _CACHE
