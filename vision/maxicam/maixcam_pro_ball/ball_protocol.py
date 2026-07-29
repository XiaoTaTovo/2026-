"""Frozen H-problem ball observation protocol, version 2."""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum

SOF = b"\xa5\x5a"
VERSION = 2
MESSAGE_TYPE = 0x20
PAYLOAD_LENGTH = 8
FRAME_LENGTH = 20
INVALID_POSITION = -32768
UNKNOWN_CAPTURE_AGE_MS = 0xFFFF


class BallReason(IntEnum):
    OK = 0
    NO_BALL = 1
    LOW_CONFIDENCE = 2
    AMBIGUOUS = 3
    OUT_OF_ROI = 4
    CALIBRATION_INVALID = 5
    FRAME_STALE = 6
    CAMERA_ERROR = 7
    MODEL_ERROR = 8
    POSITION_RANGE = 9
    WARMUP = 10


def crc16_modbus(data: bytes | bytearray | memoryview) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc & 0xFFFF


def _check_uint(name: str, value: int, maximum: int) -> None:
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError(f"{name} must be an integer")
    if not 0 <= value <= maximum:
        raise ValueError(f"{name} is out of range")


@dataclass(frozen=True, slots=True)
class BallObservationPacket:
    sequence: int
    tx_uptime_ms: int
    valid: bool
    reason: BallReason
    x_0_1mm: int
    confidence_permille: int
    capture_age_ms: int

    def __post_init__(self) -> None:
        if not isinstance(self.valid, bool):
            raise TypeError("valid must be bool")
        _check_uint("sequence", self.sequence, 0xFF)
        _check_uint("tx_uptime_ms", self.tx_uptime_ms, 0xFFFFFFFF)
        _check_uint("confidence_permille", self.confidence_permille, 1000)
        _check_uint("capture_age_ms", self.capture_age_ms, 0xFFFF)
        if not isinstance(self.reason, BallReason):
            object.__setattr__(self, "reason", BallReason(self.reason))
        if not -32768 <= self.x_0_1mm <= 32767:
            raise ValueError("x_0_1mm must fit int16")
        if self.valid:
            if self.reason != BallReason.OK:
                raise ValueError("a valid packet must use reason OK")
            if self.x_0_1mm == INVALID_POSITION:
                raise ValueError("a valid packet must carry a position")
            if self.capture_age_ms == UNKNOWN_CAPTURE_AGE_MS:
                raise ValueError("a valid packet must carry capture age")
        else:
            if self.reason == BallReason.OK:
                raise ValueError("an invalid packet must carry a failure reason")
            if self.x_0_1mm != INVALID_POSITION:
                raise ValueError("an invalid packet must not carry a position")

    @classmethod
    def invalid(
        cls,
        *,
        sequence: int,
        tx_uptime_ms: int,
        reason: BallReason,
        capture_age_ms: int = UNKNOWN_CAPTURE_AGE_MS,
        confidence_permille: int = 0,
    ) -> BallObservationPacket:
        return cls(
            sequence=sequence,
            tx_uptime_ms=tx_uptime_ms,
            valid=False,
            reason=reason,
            x_0_1mm=INVALID_POSITION,
            confidence_permille=confidence_permille,
            capture_age_ms=capture_age_ms,
        )

    def encode(self) -> bytes:
        body = struct.pack(
            "<BBBBIBBhHH",
            VERSION,
            MESSAGE_TYPE,
            self.sequence,
            PAYLOAD_LENGTH,
            self.tx_uptime_ms,
            int(self.valid),
            int(self.reason),
            self.x_0_1mm,
            self.confidence_permille,
            self.capture_age_ms,
        )
        return SOF + body + struct.pack("<H", crc16_modbus(body))

    @classmethod
    def decode(cls, frame: bytes | bytearray | memoryview) -> BallObservationPacket:
        data = bytes(frame)
        if len(data) != FRAME_LENGTH:
            raise ValueError("ball observation frame must be 20 bytes")
        if data[:2] != SOF:
            raise ValueError("invalid SOF")
        body = data[2:-2]
        received_crc = struct.unpack_from("<H", data, FRAME_LENGTH - 2)[0]
        if crc16_modbus(body) != received_crc:
            raise ValueError("CRC mismatch")
        values = struct.unpack("<BBBBIBBhHH", body)
        version, message_type, sequence, length = values[:4]
        if (version, message_type, length) != (VERSION, MESSAGE_TYPE, PAYLOAD_LENGTH):
            raise ValueError("not a ball observation v2 frame")
        if values[5] not in (0, 1):
            raise ValueError("valid field must be exactly 0 or 1")
        return cls(
            sequence=sequence,
            tx_uptime_ms=values[4],
            valid=bool(values[5]),
            reason=BallReason(values[6]),
            x_0_1mm=values[7],
            confidence_permille=values[8],
            capture_age_ms=values[9],
        )
