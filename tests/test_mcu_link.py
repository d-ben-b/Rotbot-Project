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
