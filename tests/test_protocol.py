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


def test_open_without_reply_explains_and_closes():
    """開埠後 PING 沒回應：錯誤訊息提示查占用 / 按 RESET，並關掉埠（不留讀取緒占著）。"""
    from rotbot.actuation.link import LinkError, SerialMCU
    mcu = SerialMCU("loop://", timeout=0.02, retries=0, boot_wait=0.0)
    with pytest.raises(LinkError, match="RESET"):
        mcu.open()
    assert mcu._ser is None and not mcu._threads


def test_reboot_detected_from_status_without_boot_event():
    """EV BOOT 那行被弄壞時，靠狀態「已歸零 → 未歸零（不是急停）」判斷重開機；急停不算，且舊角度作廢。"""
    from rotbot.actuation.link import SerialMCU
    mcu = SerialMCU("loop://")
    mcu._opened = True

    def st(flags, p_cdeg, y_cdeg):
        mcu._handle_line(P.encode_line("ST", 0, flags, p_cdeg, y_cdeg, 0, 0, 0, 0, 0, -1).strip())

    st(P.F_HOMED, 1500, 300)
    st(0, 0, 0)
    assert mcu.reboots == 1 and mcu.pose_before_reboot == (15.0, 3.0, False)
    st(0, 0, 0)
    assert mcu.reboots == 1                                  # 同一次重開機只算一次
    st(P.F_HOMED, 1500, 300)
    st(P.F_ESTOP, 1500, 300)
    assert mcu.reboots == 1 and mcu.pose_before_reboot is None


def test_estop_keeps_pose_when_turret_holds_unpowered():
    """兩軸斷電推不動（keep_pose_on_estop）：急停不算重開機，記下急停前的角度給 restore_zero()；已歸零的角度交給存檔。"""
    from rotbot.actuation.link import SerialMCU

    class Store:
        def __init__(self):
            self.puts = []

        def put(self, *pose):
            self.puts.append(pose)

    store = Store()
    mcu = SerialMCU("loop://", pose_store=store, keep_pose_on_estop=True)
    mcu._opened = True

    def st(flags, p_cdeg, y_cdeg):
        mcu._handle_line(P.encode_line("ST", 0, flags, p_cdeg, y_cdeg, 0, 0, 0, 0, 0, -1).strip())

    st(P.F_HOMED, 1500, 300)
    assert store.puts == [(15.0, 3.0, False)]
    st(P.F_ESTOP, 1500, 300)
    assert mcu.reboots == 0 and mcu.zero_lost == 1 and mcu.zero_lost_why == "estop"
    assert mcu.lost_pose == (15.0, 3.0, False)
    st(P.F_ESTOP, 1500, 300)
    assert mcu.zero_lost == 1                                # 同一次急停只算一次
    assert store.puts == [(15.0, 3.0, False)]               # 未歸零的狀態不存


def test_wheel_reply_parsed():
    wh = P.parse_frame(P.decode_line(P.encode_line("WH", 7, 482, -15, 0, 1203)))
    assert wh.kind == "WH" and wh.seq == 7 and wh.wheels == (48.2, -1.5, 0.0, 120.3)


def test_encoder_raw_reply_parsed():
    er = P.parse_frame(P.decode_line(P.encode_line("ER", 3, 120, 0, 65535, 7, 0b00000110)))
    assert er.kind == "ER" and er.enc_raw == ((120, 0, 65535, 7), 6)
