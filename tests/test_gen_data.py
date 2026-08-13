# -*- coding: utf-8 -*-
"""合成資料集產生：驗證結構與 YOLO-pose 標籤格式。"""
from rotbot.config import load_config
from tools.gen_synthetic_data import generate_dataset


def test_dataset_structure_and_format(tmp_path):
    cfg = load_config()
    out = generate_dataset(cfg, n=6, out_dir=tmp_path / "ds", val_split=0.5,
                           seed=1, n_preview=2)

    assert (out / "dataset.yaml").exists()
    train_imgs = list((out / "images" / "train").glob("*.png"))
    val_imgs = list((out / "images" / "val").glob("*.png"))
    assert len(train_imgs) == 3 and len(val_imgs) == 3

    # 每個影像都有對應標籤
    for img in train_imgs:
        assert (out / "labels" / "train" / f"{img.stem}.txt").exists()

    # 標籤格式：class + 4(bbox) + 4*3(keypoints) = 17 個 token，座標在 [0,1]
    for txt in (out / "labels" / "train").glob("*.txt"):
        for line in txt.read_text(encoding="utf-8").splitlines():
            toks = line.split()
            assert len(toks) == 17
            assert toks[0] in ("0", "1")
            coords = [float(t) for t in toks[1:5]]
            assert all(0.0 <= c <= 1.0 for c in coords)


def test_yaml_has_kpt_shape(tmp_path):
    cfg = load_config()
    out = generate_dataset(cfg, n=2, out_dir=tmp_path / "ds", val_split=0.5, seed=2)
    text = (out / "dataset.yaml").read_text(encoding="utf-8")
    assert "kpt_shape: [4, 3]" in text
    assert "flip_idx: [1, 0, 3, 2]" in text
