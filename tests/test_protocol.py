# -*- coding: utf-8 -*-
"""Orin ↔ MCU 協定 v1：CRC、幀編解碼、回覆解析（Python 端；C++ 端由 test_mcu_link 交叉驗證）。"""
import pytest

from rotbot.actuation import protocol as P


def test_crc8_check_value():
    # CRC-8/SMBUS 標準檢查值（與 firmware/rotbot_mcu/proto.cpp 同一演算法）
    assert P.crc8(b"123456789") == 0xF4
    assert P.crc8(b"") == 0


def test_encode_decode_roundtrip():
    line = P.encode_line("AIM", 12, 2513, -150)
    assert line == b"AIM,12,2513,-150*34\n"        # 文件（docs/mcu_orin_link.md）裡的範例幀
    assert P.decode_line(line) == ["AIM", "12", "2513", "-150"]


def test_bad_crc_rejected():
    line = bytearray(P.encode_line("FIRE", 7))
    line[0:4] = b"FIRF"                       # 竄改內容，CRC 不再相符
    with pytest.raises(P.ProtocolError):
        P.decode_line(bytes(line))
    with pytest.raises(P.ProtocolError):
        P.decode_line(b"PING,1\n")            # 預設要求 CRC
    assert P.decode_line(b"PING,1\n", require_crc=False) == ["PING", "1"]


def test_line_length_limit():
    with pytest.raises(P.ProtocolError):
        P.encode_line("AIM", 1, "9" * 80)


def test_parse_replies():
    st = P.parse_frame(P.decode_line(P.encode_line("ST", 5, 0x45, 2513, -150, 2400, 2400, 3, 0, 1, 118)))
    assert st.kind == "ST" and st.seq == 5
    s = st.status
    assert s.ready and s.fly_ok and s.homed and not s.moving
    assert s.pitch_deg == 25.13 and s.yaw_deg == -1.5 and s.rpm == 2400 and s.shots == 3
    assert s.vbat == 11.8
    no_vbat = P.parse_frame(P.decode_line(P.encode_line("ST", 6, 0, 0, 0, -1, 0, 0, 0, 0, -1)))
    assert no_vbat.status.vbat is None and no_vbat.status.rpm == -1
    nak = P.parse_frame(P.decode_line(P.encode_line("NAK", 9, P.E_NOT_READY)))
    assert nak.kind == "NAK" and nak.code == P.E_NOT_READY
    ev = P.parse_frame(P.decode_line(P.encode_line("EV", 4, "FIRED", 3)))
    assert isinstance(ev, P.Event) and ev.name == "FIRED" and ev.arg == 3
    pong = P.parse_frame(P.decode_line(P.encode_line("PONG", 1, 1, 1, 123456)))
    assert pong.pong == (1, 1, 123456)
    with pytest.raises(P.ProtocolError):
        P.parse_frame(["ST", "1", "2"])


def test_resets_on_open_by_usb_vendor(monkeypatch):
    """Uno 類開埠會重置（要等 EV BOOT）；NUCLEO 的 ST-LINK（VID 0x0483）不會；模擬器 socket:// 視為會。"""
    from types import SimpleNamespace

    from serial.tools import list_ports

    from rotbot.actuation.link import ST_LINK_VID, resets_on_open
    ports = [SimpleNamespace(device="COM7", vid=ST_LINK_VID), SimpleNamespace(device="COM5", vid=0x2341)]
    monkeypatch.setattr(list_ports, "comports", lambda: ports)
    assert resets_on_open("COM7") is False
    assert resets_on_open("COM5") is True
    assert resets_on_open("COM99") is True                  # 查不到：保守等 BOOT
    assert resets_on_open("socket://127.0.0.1:5760") is True
