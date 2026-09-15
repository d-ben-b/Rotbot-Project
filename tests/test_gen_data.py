# -*- coding: utf-8 -*-
"""合成資料集產生：驗證結構與 YOLO 偵測標籤格式（圓孔）。"""
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

    for img in train_imgs:
        assert (out / "labels" / "train" / f"{img.stem}.txt").exists()

    # 偵測標籤：class + cx cy w h = 5 個 token，座標在 [0,1]，class 在 {0,1,2}
    for txt in (out / "labels" / "train").glob("*.txt"):
        for line in txt.read_text(encoding="utf-8").splitlines():
            toks = line.split()
            assert len(toks) == 5
            assert toks[0] in ("0", "1", "2")
            assert all(0.0 <= float(t) <= 1.0 for t in toks[1:])


def test_yaml_has_names(tmp_path):
    cfg = load_config()
    out = generate_dataset(cfg, n=2, out_dir=tmp_path / "ds", val_split=0.5, seed=2)
    text = (out / "dataset.yaml").read_text(encoding="utf-8")
    assert "main" in text and "bonus_lit" in text
    assert "kpt_shape" not in text                 # detect 格式，非 pose
