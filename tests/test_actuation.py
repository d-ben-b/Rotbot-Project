# -*- coding: utf-8 -*-
from rotbot.actuation import protocol
from rotbot.actuation.link import MockArduino
from rotbot.hal import AimCommand


def test_protocol_roundtrip():
    cmd = AimCommand(pitch_deg=12.5, yaw_deg=-3.2, flywheel=1650.0, trigger=True)
    decoded = protocol.decode(protocol.encode(cmd))
    assert abs(decoded.pitch_deg - cmd.pitch_deg) < 1e-3
    assert abs(decoded.yaw_deg - cmd.yaw_deg) < 1e-3
    assert abs(decoded.flywheel - cmd.flywheel) < 1e-3
    assert decoded.trigger is True


def test_mock_arduino_records_and_acks():
    arduino = MockArduino()
    assert arduino.is_connected()
    ack = arduino.send(AimCommand(10.0, 0.0, 1600.0, True))
    assert ack.ok
    assert len(arduino.history) == 1
