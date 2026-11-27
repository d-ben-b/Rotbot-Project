# -*- coding: utf-8 -*-
"""場地照片擴增（tools/augment_arena_photo.py）：標籤數、類別、範圍。"""
import cv2
import numpy as np

from tools.augment_arena_photo import LIT, PHOTO, PHOTO_Y, sample


def test_samples_have_12_holes_and_at_most_one_lit():
    photo = cv2.imread(str(PHOTO))[PHOTO_Y[0]:PHOTO_Y[1]]
    rng = np.random.default_rng(3)
    full = 0
    for _ in range(6):
        img, labels = sample(photo, rng)
        assert img.shape == (1088, 1456, 3)
        cls = [c for c, *_ in labels]
        assert cls.count(0) <= 9 and len(cls) <= 12 and cls.count(LIT) <= 1   # 近距離時網板邊緣的孔可能出畫面
        full += len(cls) == 12
        for _c, x, y, w, h in labels:
            assert 0 < x < 1 and 0 < y < 1 and 0 < w < 0.2 and 0 < h < 0.2
    assert full >= 3
