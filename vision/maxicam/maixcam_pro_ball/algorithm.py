"""Small, testable helpers for YOLO-assisted one-dimensional ball location."""

from __future__ import annotations

import math
from typing import Any


def project_pixel_to_mm(
    point: tuple[float, float],
    negative_end: tuple[float, float],
    positive_end: tuple[float, float],
    rod_length_mm: float,
) -> float:
    """Project a pixel onto the calibrated rod axis and use its center as x=0."""
    axis_x = positive_end[0] - negative_end[0]
    axis_y = positive_end[1] - negative_end[1]
    denominator = axis_x * axis_x + axis_y * axis_y
    if denominator < 1.0 or rod_length_mm <= 0.0:
        raise ValueError("invalid rod calibration")
    relative_x = point[0] - negative_end[0]
    relative_y = point[1] - negative_end[1]
    fraction = (relative_x * axis_x + relative_y * axis_y) / denominator
    return (fraction - 0.5) * rod_length_mm


def normalized_point(
    point: tuple[float, float], width: int, height: int
) -> tuple[float, float]:
    if width < 2 or height < 2:
        raise ValueError("image is too small")
    return point[0] * (width - 1), point[1] * (height - 1)


def refine_ball_center(
    frame_bgr: Any,
    bbox: tuple[int, int, int, int],
    *,
    background_bgr: Any | None = None,
    background_threshold: int = 24,
) -> tuple[tuple[float, float], float, bool]:
    """Refine a YOLO box with background difference/connected components.

    Returns (center_px, refinement_quality_0_to_1, refined). If refinement is
    unreliable, the YOLO box center is returned with refined=False.
    """
    import cv2
    import numpy as np

    x, y, w, h = bbox
    fallback = (x + 0.5 * w, y + 0.5 * h)
    if w < 4 or h < 4 or frame_bgr is None:
        return fallback, 0.0, False

    image_height, image_width = frame_bgr.shape[:2]
    pad = max(2, round(0.20 * max(w, h)))
    x0 = max(0, x - pad)
    y0 = max(0, y - pad)
    x1 = min(image_width, x + w + pad)
    y1 = min(image_height, y + h + pad)
    if x1 - x0 < 5 or y1 - y0 < 5:
        return fallback, 0.0, False

    roi = frame_bgr[y0:y1, x0:x1]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    masks = []
    if background_bgr is not None and background_bgr.shape[:2] == frame_bgr.shape[:2]:
        background_roi = background_bgr[y0:y1, x0:x1]
        background_gray = cv2.cvtColor(background_roi, cv2.COLOR_BGR2GRAY)
        difference = cv2.absdiff(gray, background_gray)
        _, mask = cv2.threshold(
            difference, background_threshold, 255, cv2.THRESH_BINARY
        )
        masks.append(mask)
    else:
        for mode in (cv2.THRESH_BINARY, cv2.THRESH_BINARY_INV):
            _, mask = cv2.threshold(gray, 0, 255, mode | cv2.THRESH_OTSU)
            masks.append(mask)

    kernel = np.ones((3, 3), np.uint8)
    roi_area = float((x1 - x0) * (y1 - y0))
    expected_center = (fallback[0] - x0, fallback[1] - y0)
    best = None
    for mask in masks:
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < max(6.0, 0.02 * w * h) or area > 0.75 * roi_area:
                continue
            bx, by, bw, bh = cv2.boundingRect(contour)
            if bx == 0 or by == 0 or bx + bw >= x1 - x0 or by + bh >= y1 - y0:
                continue
            aspect = bw / max(1.0, float(bh))
            if not 0.45 <= aspect <= 2.20:
                continue
            perimeter = float(cv2.arcLength(contour, True))
            if perimeter <= 0.0:
                continue
            circularity = min(1.0, 4.0 * math.pi * area / (perimeter * perimeter))
            if circularity < 0.30:
                continue
            moments = cv2.moments(contour)
            if moments["m00"] == 0.0:
                continue
            cx = moments["m10"] / moments["m00"]
            cy = moments["m01"] / moments["m00"]
            distance = math.hypot(cx - expected_center[0], cy - expected_center[1])
            distance_scale = max(2.0, 0.5 * math.hypot(w, h))
            proximity = max(0.0, 1.0 - distance / distance_scale)
            fill = min(1.0, area / max(1.0, 0.45 * w * h))
            quality = 0.50 * circularity + 0.35 * proximity + 0.15 * fill
            if best is None or quality > best[0]:
                best = (quality, cx + x0, cy + y0)

    if best is None or best[0] < 0.45:
        return fallback, 0.0, False
    return (best[1], best[2]), min(1.0, best[0]), True
