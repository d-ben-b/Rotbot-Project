# -*- coding: utf-8 -*-
"""Orin 端 SerialMCU ↔ 韌體（同一份 app.cpp/proto.cpp 在電腦上跑）端到端驗證。

建置模擬器需要 C++ 編譯器（g++/clang++，或 `pip install ziglang`）；沒有就跳過。
兩個韌體變體都跑：full（轉速閉迴路 + 電壓補償 + 推球原點開關）、basic（開迴路 + 固定時間推球）。
"""
import pytest

from tools.mcu_sil import CompilerNotFound, SilProcess, build


@pytest.fixture(scope="module", params=["full", "basic"])
def sil(request):
    try:
        build(request.param)
    except CompilerNotFound as e:
        pytest.skip(str(e))
    with SilProcess(request.param) as proc:
        yield proc


def test_link_suite_against_firmware(sil):
    from tools.mcu_link_test import Suite
    results = Suite(sil.url, sil=True, allow_motion=True, variant=sil.variant,
                    n_ping=50, verbose=False).run()
    failed = [r for r in results if r["status"] == "FAIL"]
    assert not failed, failed
    assert sum(r["status"] == "PASS" for r in results) >= 19


def test_hal_send_fires_one_ball(sil):
    """HAL 介面 send(AimCommand) = 轉到位 + 到速 + 推球 + 等 FIRED。"""
    from rotbot.actuation.link import SerialMCU
    from rotbot.hal import AimCommand
    with SerialMCU(sil.url) as mcu:
        mcu.zero(0, 0)
        ack = mcu.send(AimCommand(pitch_deg=24.0, yaw_deg=3.0, flywheel=2400, trigger=True))
        assert ack.ok, ack.message
        st = mcu.get_status()
        assert abs(st.pitch_deg - 24.0) < 0.05 and abs(st.yaw_deg - 3.0) < 0.05
        assert st.shots == 1


def _settle(mcu, pitch, yaw, timeout=3.0):
    """等轉到位。容許 0.1°：水平每微步 0.1125°，回報角度和指令最多差約 0.06°（同 loop._move_and_settle）。

    要求回報值完全相等的測試用水平 −4.5°：每度 8.889 步時剛好 40 步，可以精確表示。
    """
    import time
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        st = mcu.get_status()
        if not st.moving and abs(st.pitch_deg - pitch) < 0.1 and abs(st.yaw_deg - yaw) < 0.1:
            return st
        time.sleep(0.02)
    raise AssertionError(f"沒轉到 ({pitch}, {yaw})")


def test_reboot_detected_and_zero_restored(sil):
    """比賽中 MCU 重開機：偵測到 → 用重開機前的角度補回歸零（含補回前被 JOG 轉過的量）→ AIM 照常。"""
    import time

    from rotbot.actuation.link import NakError, SerialMCU
    with SerialMCU(sil.url) as mcu:
        mcu.zero(0, 0)
        mcu.aim(12.0, -4.5)
        _settle(mcu, 12.0, -4.5)
        time.sleep(0.25)                                    # 背景輪詢記下停好的角度
        mcu.write_raw(b"!RESET\n")                          # 模擬器重新開機
        end = time.monotonic() + 2.0
        while mcu.reboots == 0 and time.monotonic() < end:
            time.sleep(0.02)
        assert mcu.reboots == 1
        assert mcu.pose_before_reboot[:2] == pytest.approx((12.0, -4.5), abs=0.02)
        with pytest.raises(NakError) as e:                  # 還沒補回：未歸零，MCU 拒絕 AIM
            mcu.aim(10.0, 0.0)
        assert e.value.code == 7
        mcu.jog(0.0, 20.0)                                  # 補回前被轉了一段
        time.sleep(0.3)
        p, y = mcu.restore_after_reboot()
        st = mcu.get_status()
        assert st.homed and (st.pitch_deg, st.yaw_deg) == pytest.approx((p, y), abs=0.02)
        assert p == pytest.approx(12.0, abs=0.02) and 2.0 < y + 4.5 < 8.0   # 20°/s × 約 0.3 s
        mcu.aim(12.0, -4.5)
        _settle(mcu, 12.0, -4.5)
        assert mcu.restore_after_reboot() is None           # 已歸零：不再補
        time.sleep(0.25)
        assert mcu.reboots == 1                             # EV BOOT 和狀態變化只算一次

        mcu.flywheel(1800)                                  # AI 那一發轉到一半時重開機
        mcu.aim(30.0, 10.0)
        time.sleep(0.15)
        mcu.write_raw(b"!RESET\n")
        end = time.monotonic() + 2.0
        while mcu.reboots == 1 and time.monotonic() < end:
            time.sleep(0.02)
        assert mcu.reboots == 2 and mcu.pose_before_reboot[2]   # 記下「當時在轉」
        assert mcu.get_status().rpm_target == 0
        mcu.restore_after_reboot()
        st = _settle(mcu, 30.0, 10.0)                       # 重送的 AIM 讓它繼續轉到目標
        assert st.rpm_target == 1800                        # 飛輪轉速也重送了


def test_power_cycle_restores_zero_from_saved_pose(sil, tmp_path):
    """整台斷電再開（新的模擬器 = MCU 步數從 0 開始）：open() 用存檔的角度自動歸零，不必手動按「歸零」。"""
    import time

    from rotbot.actuation.link import SerialMCU
    from rotbot.actuation.pose_store import PoseStore
    store = PoseStore(tmp_path / "turret_pose.json")
    try:
        with SilProcess(sil.variant) as first:
            with SerialMCU(first.url, pose_store=store) as mcu:
                assert mcu.restored_at_open is None and not mcu.get_status().homed   # 沒有存檔：等手動歸零
                mcu.zero(0, 0)
                mcu.aim(12.0, -4.5)
                _settle(mcu, 12.0, -4.5)
                time.sleep(0.25)                            # 背景輪詢記下停好的角度
        assert store.flush()
        saved = store.load()
        assert saved[:2] == pytest.approx((12.0, -4.5), abs=0.02) and saved[2] is False
        with SilProcess(sil.variant) as second:
            with SerialMCU(second.url, pose_store=store) as mcu:
                assert mcu.restored_at_open[:2] == pytest.approx((12.0, -4.5), abs=0.02)
                st = mcu.get_status()
                assert st.homed and (st.pitch_deg, st.yaw_deg) == pytest.approx((12.0, -4.5), abs=0.02)
                mcu.aim(0.0, 0.0)                           # 不必再歸零就能轉
                _settle(mcu, 0.0, 0.0)
    finally:
        store.close()


def test_estop_keeps_zero_when_turret_holds(sil):
    """兩軸斷電推不動時急停：MCU 清掉「已歸零」但步數沒變 → restore_zero() 用目前步數補回，放開後直接能轉。"""
    import time

    from rotbot.actuation.link import SerialMCU
    with SerialMCU(sil.url, keep_pose_on_estop=True) as mcu:
        mcu.zero(0, 0)
        mcu.aim(12.0, -4.5)
        _settle(mcu, 12.0, -4.5)
        time.sleep(0.25)
        mcu.write_raw(b"!ESTOP 1\n")
        end = time.monotonic() + 2.0
        while mcu.zero_lost == 0 and time.monotonic() < end:
            time.sleep(0.02)
        assert mcu.zero_lost == 1 and mcu.zero_lost_why == "estop" and mcu.reboots == 0
        assert mcu.lost_pose[:2] == pytest.approx((12.0, -4.5), abs=0.02)
        assert mcu.restore_zero() == pytest.approx((12.0, -4.5), abs=0.02)
        st = mcu.get_status()
        assert st.homed and st.estop
        mcu.write_raw(b"!ESTOP 0\n")
        end = time.monotonic() + 2.0
        while mcu.get_status().estop and time.monotonic() < end:
            time.sleep(0.02)
        mcu.aim(10.0, 0.0)
        _settle(mcu, 10.0, 0.0)


def test_drive_full_scaled_to_wheel_max(sil):
    """DRIVE ±1000 縮放成 WHEEL_MAX_PERMILLE（714）：12 V 馬達接 4S 時全速約 12 V。模擬器全速 160 rpm → 約 114。"""
    import time

    from rotbot.actuation.link import SerialMCU
    with SerialMCU(sil.url) as mcu:
        for _ in range(15):
            mcu.drive(1.0, 1.0, -1.0, -1.0)
            time.sleep(0.1)
        rpm = mcu.wheels()
        mcu.stop()
        assert all(100 < r < 125 for r in rpm[:2]) and all(-125 < r < -100 for r in rpm[2:]), rpm


def test_stop_also_stops_wheels(sil):
    """STOP（操控台的停止鍵、駕駛斷線）也要讓車輪停。"""
    import time

    from rotbot.actuation.link import SerialMCU
    with SerialMCU(sil.url) as mcu:
        for _ in range(8):
            mcu.drive(0.5, 0.5, -0.5, -0.5)
            time.sleep(0.1)
        rpm = mcu.wheels()
        assert rpm[0] > 20 and rpm[2] < -20
        mcu.stop()
        time.sleep(0.8)
        assert max(abs(r) for r in mcu.wheels()) < 3.0


def test_encoder_raw_debug(sil):
    """除錯指令 ENC：原始計數跟著轉、A/B 腳位會變。"""
    import time

    from rotbot.actuation.link import SerialMCU
    with SerialMCU(sil.url) as mcu:
        mcu.stop()
        raw0, _ = mcu.encoder_raw()
        end = time.monotonic() + 3.0
        while time.monotonic() < end:                       # 前一項測試的輪子可能還在慢慢停（全套一起跑時偶爾發生）
            time.sleep(0.1)
            again, _ = mcu.encoder_raw()
            if again == raw0:
                break
            raw0 = again
        seen = set()
        for _ in range(6):
            mcu.drive(0.3, 0, 0, 0)
            time.sleep(0.05)
            seen.add(mcu.encoder_raw()[1] & 0b11)
        raw1, _ = mcu.encoder_raw()
        mcu.stop()
        assert raw1[0] != raw0[0] and raw1[1:] == raw0[1:]
        assert len(seen) > 1
