import pytest

from arm_control.protocol import decode_reply, encode_gripper, encode_move, encode_simple


def test_encode_move_uses_centidegrees():
    assert encode_move([0.0, 12.34, -90.0]) == b"MOVE 0 1234 -9000\n"


def test_encode_gripper():
    assert encode_gripper(45.5) == b"GRIP 4550\n"


def test_encode_simple_rejects_unknown():
    assert encode_simple("HOME") == b"HOME\n"
    with pytest.raises(ValueError):
        encode_simple("FLY")


def test_decode_replies():
    assert decode_reply(b"OK\r\n") == ("OK", "")
    assert decode_reply("ERR limit joint 2") == ("ERR", "limit joint 2")
    assert decode_reply("POS 0 1234 -9000") == ("POS", [0.0, 12.34, -90.0])
