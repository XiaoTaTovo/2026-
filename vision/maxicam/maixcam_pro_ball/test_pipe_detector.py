"""PC-side checks for the white-pipe detector."""

import unittest

import cv2
import numpy as np

from pipe_detector import (
    detect_white_pipe,
    pipe_relative_position,
    pipe_roi_bbox,
    relative_to_centered_mm,
)


class PipeDetectorTests(unittest.TestCase):
    def test_maps_relative_position_to_centered_250_mm_coordinate(self):
        self.assertEqual(relative_to_centered_mm(0.0, 250.0), -125.0)
        self.assertEqual(relative_to_centered_mm(0.5, 250.0), 0.0)
        self.assertEqual(relative_to_centered_mm(1.0, 250.0), 125.0)

    def test_detects_long_white_pipe_with_dark_ball(self):
        frame = np.zeros((224, 320, 3), dtype=np.uint8)
        frame[88:136, 24:296] = (230, 230, 230)
        cv2.circle(frame, (160, 112), 9, (45, 45, 45), -1)

        pipe, _, stats = detect_white_pipe(frame)

        self.assertIsNotNone(pipe)
        self.assertGreater(pipe["aspect_ratio"], 3.0)
        self.assertGreaterEqual(stats["fill"], 1)

        midpoint = pipe_relative_position(pipe, (160.0, 112.0))
        self.assertIsNotNone(midpoint)
        self.assertAlmostEqual(midpoint["percent"], 50.0, delta=1.0)

        end_a = pipe_relative_position(pipe, pipe["end_a"])
        self.assertIsNotNone(end_a)
        self.assertAlmostEqual(end_a["percent"], 0.0, delta=0.01)

        self.assertIsNone(pipe_relative_position(pipe, (160.0, 30.0)))

        roi = pipe_roi_bbox(pipe, frame.shape[1], frame.shape[0])
        pipe_x, _, pipe_width, _ = pipe["bbox"]
        self.assertLessEqual(roi[0], pipe_x)
        self.assertGreaterEqual(roi[0] + roi[2], pipe_x + pipe_width - 1)

    def test_rejects_white_square(self):
        frame = np.zeros((224, 320, 3), dtype=np.uint8)
        frame[70:150, 120:200] = (230, 230, 230)

        pipe, _, _ = detect_white_pipe(frame)

        self.assertIsNone(pipe)

    def test_detects_complete_pipe_touching_both_frame_edges(self):
        frame = np.zeros((224, 320, 3), dtype=np.uint8)
        frame[90:120, 0:320] = (230, 230, 230)

        pipe, _, stats = detect_white_pipe(
            frame,
            min_aspect_ratio=8.0,
            min_fill_ratio=0.60,
        )

        self.assertIsNotNone(pipe)
        x, y, width, height = pipe["bbox"]
        self.assertEqual(x, 0)
        self.assertEqual(x + width, 320)
        self.assertGreaterEqual(y, 85)
        self.assertLessEqual(y + height, 125)
        self.assertEqual(stats["fill"], 1)

    def test_detects_pipe_when_one_endpoint_touches_frame_edge(self):
        frame = np.zeros((224, 320, 3), dtype=np.uint8)
        frame[90:120, 0:280] = (230, 230, 230)

        pipe, _, stats = detect_white_pipe(
            frame,
            min_aspect_ratio=8.0,
            min_fill_ratio=0.60,
        )

        self.assertIsNotNone(pipe)
        self.assertEqual(pipe["bbox"][0], 0)
        self.assertEqual(stats["fill"], 1)


if __name__ == "__main__":
    unittest.main()
