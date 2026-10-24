"""Serial protocol shared with firmware/src/main.cpp. See docs/protocol.md.

Commands are ASCII lines terminated by '\\n'. Angles are sent as integer
centidegrees so the firmware never parses floats.
"""

from __future__ import annotations

from typing import Sequence


def encode_move(angles_deg: Sequence[float]) -> bytes:
    return ("MOVE " + " ".join(str(round(a * 100)) for a in angles_deg) + "\n").encode()


def encode_gripper(angle_deg: float) -> bytes:
    return f"GRIP {round(angle_deg * 100)}\n".encode()


def encode_simple(command: str) -> bytes:
    if command not in {"HOME", "STATUS", "STOP", "ENABLE", "DISABLE"}:
        raise ValueError(f"unknown command {command!r}")
    return f"{command}\n".encode()


def decode_reply(line: bytes | str) -> tuple[str, list[float] | str]:
    """Parse a firmware reply.

    'OK' -> ('OK', '')
    'ERR <message>' -> ('ERR', message)
    'POS <cdeg> ...' -> ('POS', [deg, ...])
    """
    text = line.decode() if isinstance(line, bytes) else line
    text = text.strip()
    kind, _, rest = text.partition(" ")
    if kind == "POS":
        return kind, [int(v) / 100 for v in rest.split()]
    if kind in {"OK", "ERR", "BUSY"}:
        return kind, rest
    raise ValueError(f"unrecognised reply {text!r}")


class ArmLink:
    """Thin wrapper around a pyserial port."""

    def __init__(self, port: str, baud: int = 115200, timeout: float = 1.0):
        import serial  # imported here so the rest of the package works without pyserial

        self.ser = serial.Serial(port, baud, timeout=timeout)

    def send(self, payload: bytes):
        self.ser.write(payload)
        return decode_reply(self.ser.readline())

    def move(self, angles_deg):
        return self.send(encode_move(angles_deg))

    def status(self):
        return self.send(encode_simple("STATUS"))

    def close(self):
        self.ser.close()
