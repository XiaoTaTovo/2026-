"""PC-side checks for the MaixCAM-to-STM32 observation frame."""

import unittest

from ball_protocol import (
    FRAME_LENGTH,
    INVALID_POSITION,
    BallObservationPacket,
    BallReason,
)


class BallProtocolTests(unittest.TestCase):
    def test_valid_packet_round_trip(self):
        sent = BallObservationPacket(
            sequence=7,
            tx_uptime_ms=0x12345678,
            valid=True,
            reason=BallReason.OK,
            x_0_1mm=-131,
            confidence_permille=590,
            capture_age_ms=23,
        )
        frame = sent.encode()
        self.assertEqual(len(frame), FRAME_LENGTH)
        self.assertEqual(frame[:6], bytes((0xA5, 0x5A, 2, 0x20, 7, 8)))
        self.assertEqual(BallObservationPacket.decode(frame), sent)

    def test_invalid_packet_uses_position_sentinel(self):
        packet = BallObservationPacket.invalid(
            sequence=8,
            tx_uptime_ms=1234,
            reason=BallReason.OUT_OF_ROI,
        )
        decoded = BallObservationPacket.decode(packet.encode())
        self.assertFalse(decoded.valid)
        self.assertEqual(decoded.x_0_1mm, INVALID_POSITION)
        self.assertEqual(decoded.reason, BallReason.OUT_OF_ROI)

    def test_crc_error_is_rejected(self):
        packet = BallObservationPacket.invalid(
            sequence=1,
            tx_uptime_ms=1,
            reason=BallReason.NO_BALL,
        )
        damaged = bytearray(packet.encode())
        damaged[12] ^= 0x01
        with self.assertRaisesRegex(ValueError, "CRC mismatch"):
            BallObservationPacket.decode(damaged)


if __name__ == "__main__":
    unittest.main()
