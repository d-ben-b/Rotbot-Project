# -*- coding: utf-8 -*-
"""轉塔角度存檔（PoseStore）：寫入、讀回、壞檔、限制寫入頻率。"""
import time

from rotbot.actuation.pose_store import PoseStore


def test_pose_store_round_trip(tmp_path):
    path = tmp_path / "runs" / "turret_pose.json"
    s = PoseStore(path, min_interval_s=0.0)
    try:
        assert s.load() is None                            # 第一次：沒有存檔
        s.put(12.0, -5.0, False)
        assert s.flush()
        assert s.load() == (12.0, -5.0, False)
        s.put(12.001, -5.0, False)                         # 差不到 0.005°：不寫
        assert s.flush() and s.writes == 1
        s.put(20.0, 3.0, True)
        assert s.flush() and s.load() == (20.0, 3.0, True)
        assert not path.with_name(path.name + ".tmp").exists()
    finally:
        s.close()


def test_pose_store_bad_file_is_ignored(tmp_path):
    path = tmp_path / "turret_pose.json"
    path.write_text("{壞掉", encoding="utf-8")
    s = PoseStore(path)
    try:
        assert s.load() is None
    finally:
        s.close()


def test_pose_store_limits_write_rate_but_keeps_latest(tmp_path):
    """轉動中每 20 ms 一筆：只寫最新的，0.2 s 最多一次；最後停下的那筆一定寫進去。"""
    s = PoseStore(tmp_path / "turret_pose.json", min_interval_s=0.2)
    try:
        t0 = time.monotonic()
        for i in range(25):                                # 0.5 s 內 25 筆
            s.put(float(i), 0.0, True)
            time.sleep(0.02)
        s.put(30.0, 0.0, False)
        assert s.flush()
        assert s.load() == (30.0, 0.0, False)
        assert s.writes <= (time.monotonic() - t0) / 0.2 + 2
    finally:
        s.close()
    assert not s._thread.is_alive()


def test_turret_config_defaults():
    from rotbot.config import load_config
    t = load_config().turret
    assert t.holds_unpowered is True and t.pose_file.endswith(".json")
