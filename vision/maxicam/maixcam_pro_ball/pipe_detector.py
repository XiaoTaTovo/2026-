"""OpenCV helpers for locating a bright, low-saturation pipe."""

from __future__ import annotations

import math
from typing import Any


def detect_white_pipe(
    frame_rgb: Any,
    *,
    min_value: int = 160,
    max_saturation: int = 70,
    min_area_ratio: float = 0.01,
    min_aspect_ratio: float = 3.0,
    min_fill_ratio: float = 0.35,
) -> tuple[dict[str, Any] | None, Any, dict[str, int]]:
    """Return the best elongated white region, its mask, and filter counts."""
    import cv2
    import numpy as np

    if frame_rgb is None or len(frame_rgb.shape) < 2:
        raise ValueError("frame_rgb must be a non-empty RGB image")

    height, width = frame_rgb.shape[:2]
    if width < 2 or height < 2:
        raise ValueError("frame_rgb is too small")

    hsv = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(
        hsv,
        np.array((0, 0, max(0, min(255, int(min_value)))), dtype=np.uint8),
        np.array(
            (179, max(0, min(255, int(max_saturation))), 255),
            dtype=np.uint8,
        ),
    )

    close_width = max(3, round(width * 0.03))
    close_height = max(3, round(height * 0.015))
    close_kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT,
        (close_width, close_height),
    )
    open_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    stats = {
        "raw": len(contours),
        "area": 0,
        "aspect": 0,
        "fill": 0,
    }
    frame_area = float(width * height)
    candidates = []

    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < frame_area * min_area_ratio:
            continue
        stats["area"] += 1

        (center_x, center_y), (rect_width, rect_height), angle = cv2.minAreaRect(
            contour
        )
        long_side = max(float(rect_width), float(rect_height))
        short_side = min(float(rect_width), float(rect_height))
        if short_side < 1.0:
            continue
        aspect_ratio = long_side / short_side
        if aspect_ratio < min_aspect_ratio:
            continue
        stats["aspect"] += 1

        rotated_area = max(1.0, long_side * short_side)
        fill_ratio = min(1.0, area / rotated_area)
        if fill_ratio < min_fill_ratio:
            continue
        stats["fill"] += 1

        x, y, box_width, box_height = cv2.boundingRect(contour)

        axis_angle = float(angle)
        if rect_width < rect_height:
            axis_angle += 90.0
        if axis_angle >= 90.0:
            axis_angle -= 180.0

        angle_radians = math.radians(axis_angle)
        axis_x = math.cos(angle_radians)
        axis_y = math.sin(angle_radians)
        half_length = 0.5 * long_side
        end_a = (
            float(center_x) - axis_x * half_length,
            float(center_y) - axis_y * half_length,
        )
        end_b = (
            float(center_x) + axis_x * half_length,
            float(center_y) + axis_y * half_length,
        )
        if abs(axis_x) >= abs(axis_y):
            should_swap = end_a[0] > end_b[0]
        else:
            should_swap = end_a[1] > end_b[1]
        if should_swap:
            end_a, end_b = end_b, end_a

        ordered_axis_x = (end_b[0] - end_a[0]) / long_side
        ordered_axis_y = (end_b[1] - end_a[1]) / long_side

        candidates.append(
            {
                "bbox": (int(x), int(y), int(box_width), int(box_height)),
                "area": area,
                "aspect_ratio": aspect_ratio,
                "fill_ratio": fill_ratio,
                "angle": axis_angle,
                "center": (float(center_x), float(center_y)),
                "long_side": long_side,
                "short_side": short_side,
                "end_a": end_a,
                "end_b": end_b,
                "axis_unit": (ordered_axis_x, ordered_axis_y),
                "score": area * min(aspect_ratio, 10.0),
            }
        )

    candidates.sort(key=lambda candidate: candidate["score"], reverse=True)
    best = candidates[0] if candidates else None
    return best, mask, stats


def pipe_relative_position(
    pipe: dict[str, Any],
    point: tuple[float, float],
    *,
    width_margin_ratio: float = 0.35,
) -> dict[str, float] | None:
    """Project a point from pipe end A to B if it lies in the pipe ROI."""
    end_a_x, end_a_y = pipe["end_a"]
    end_b_x, end_b_y = pipe["end_b"]
    axis_x = end_b_x - end_a_x
    axis_y = end_b_y - end_a_y
    length_squared = axis_x * axis_x + axis_y * axis_y
    if length_squared < 1.0:
        return None

    relative_x = float(point[0]) - end_a_x
    relative_y = float(point[1]) - end_a_y
    relative = (relative_x * axis_x + relative_y * axis_y) / length_squared
    if relative < 0.0 or relative > 1.0:
        return None

    length = math.sqrt(length_squared)
    cross_distance = abs(relative_x * axis_y - relative_y * axis_x) / length
    half_width = 0.5 * float(pipe["short_side"])
    allowed_cross_distance = half_width * (1.0 + max(0.0, width_margin_ratio))
    if cross_distance > allowed_cross_distance:
        return None

    return {
        "relative": relative,
        "percent": 100.0 * relative,
        "cross_distance": cross_distance,
    }


def pipe_roi_bbox(
    pipe: dict[str, Any],
    frame_width: int,
    frame_height: int,
    *,
    width_margin_ratio: float = 0.35,
) -> tuple[int, int, int, int]:
    """Return a clipped display bbox around the rotated logical pipe ROI."""
    center_x, center_y = pipe["center"]
    axis_x, axis_y = pipe["axis_unit"]
    normal_x, normal_y = -axis_y, axis_x
    half_length = 0.5 * float(pipe["long_side"])
    half_width = (
        0.5
        * float(pipe["short_side"])
        * (1.0 + max(0.0, width_margin_ratio))
    )

    corners = []
    for along_sign in (-1.0, 1.0):
        for across_sign in (-1.0, 1.0):
            corners.append(
                (
                    center_x
                    + along_sign * axis_x * half_length
                    + across_sign * normal_x * half_width,
                    center_y
                    + along_sign * axis_y * half_length
                    + across_sign * normal_y * half_width,
                )
            )

    x0 = max(0, min(frame_width - 1, math.floor(min(p[0] for p in corners))))
    y0 = max(0, min(frame_height - 1, math.floor(min(p[1] for p in corners))))
    x1 = max(x0 + 1, min(frame_width, math.ceil(max(p[0] for p in corners))))
    y1 = max(y0 + 1, min(frame_height, math.ceil(max(p[1] for p in corners))))
    return x0, y0, x1 - x0, y1 - y0


def relative_to_centered_mm(relative: float, pipe_length_mm: float) -> float:
    """Map A=0 and B=1 to a centered physical coordinate."""
    if pipe_length_mm <= 0.0:
        raise ValueError("pipe_length_mm must be positive")
    return (float(relative) - 0.5) * float(pipe_length_mm)
