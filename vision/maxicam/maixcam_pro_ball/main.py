"""MaixCAM Pro entry point for H-problem ball position transmission."""

from __future__ import annotations

import config
import cv2
from algorithm import normalized_point, project_pixel_to_mm, refine_ball_center
from ball_protocol import BallObservationPacket, BallReason
from maix import app, camera, display, err, image, nn, pinmap, time, uart


def create_detector():
    detector_types = {
        "YOLOv5": nn.YOLOv5,
        "YOLOv8": nn.YOLOv8,
        "YOLO11": nn.YOLO11,
    }
    try:
        detector_type = detector_types[config.MODEL_KIND]
    except KeyError as exc:
        raise ValueError(f"unsupported MODEL_KIND: {config.MODEL_KIND}") from exc
    return detector_type(model=config.MODEL_PATH, dual_buff=True)


def open_uart():
    if not config.UART_LEVEL_CONFIRMED:
        raise RuntimeError("confirm MaixCAM Pro/MSPM0 UART voltage before enabling TX")
    mappings = {
        config.UART_RX_PIN: "UART1_RX",
        config.UART_TX_PIN: "UART1_TX",
    }
    for pin, function in mappings.items():
        err.check_raise(
            pinmap.set_pin_function(pin, function),
            f"failed to map {pin} to {function}",
        )
    return uart.UART(config.UART_DEVICE, config.UART_BAUD)


def choose_candidate(objects):
    candidates = [obj for obj in objects if obj.class_id == config.BALL_CLASS_ID]
    candidates.sort(key=lambda obj: obj.score, reverse=True)
    if not candidates:
        return None, BallReason.NO_BALL
    if (
        len(candidates) > 1
        and candidates[0].score - candidates[1].score < config.AMBIGUITY_SCORE_MARGIN
    ):
        return None, BallReason.AMBIGUOUS
    return candidates[0], BallReason.OK


def make_packet(sequence, capture_started_ms, frame, objects, background_bgr):
    now_ms = time.ticks_ms()
    capture_age_ms = min(0xFFFE, max(0, time.ticks_diff(capture_started_ms)))
    candidate, reason = choose_candidate(objects)
    if candidate is None:
        return BallObservationPacket.invalid(
            sequence=sequence,
            tx_uptime_ms=now_ms & 0xFFFFFFFF,
            reason=reason,
            capture_age_ms=capture_age_ms,
        ), None

    bbox = (candidate.x, candidate.y, candidate.w, candidate.h)
    frame_bgr = image.image2cv(frame, ensure_bgr=True, copy=False)
    center, refine_quality, refined = refine_ball_center(
        frame_bgr,
        bbox,
        background_bgr=background_bgr,
        background_threshold=config.BACKGROUND_DIFF_THRESHOLD,
    )
    if not (0.0 <= center[0] < frame.width() and 0.0 <= center[1] < frame.height()):
        return BallObservationPacket.invalid(
            sequence=sequence,
            tx_uptime_ms=now_ms & 0xFFFFFFFF,
            reason=BallReason.OUT_OF_ROI,
            capture_age_ms=capture_age_ms,
        ), center
    if not config.CALIBRATION_CONFIRMED:
        return BallObservationPacket.invalid(
            sequence=sequence,
            tx_uptime_ms=now_ms & 0xFFFFFFFF,
            reason=BallReason.CALIBRATION_INVALID,
            capture_age_ms=capture_age_ms,
            confidence_permille=round(candidate.score * 1000.0),
        ), center

    negative_end = normalized_point(
        config.ROD_NEG_END_NORM, frame.width(), frame.height()
    )
    positive_end = normalized_point(
        config.ROD_POS_END_NORM, frame.width(), frame.height()
    )
    try:
        x_mm = project_pixel_to_mm(
            center, negative_end, positive_end, config.ROD_LENGTH_MM
        )
    except ValueError:
        return BallObservationPacket.invalid(
            sequence=sequence,
            tx_uptime_ms=now_ms & 0xFFFFFFFF,
            reason=BallReason.CALIBRATION_INVALID,
            capture_age_ms=capture_age_ms,
        ), center
    if abs(x_mm) > config.MAX_ABS_POSITION_MM:
        return BallObservationPacket.invalid(
            sequence=sequence,
            tx_uptime_ms=now_ms & 0xFFFFFFFF,
            reason=BallReason.POSITION_RANGE,
            capture_age_ms=capture_age_ms,
        ), center

    confidence_scale = (
        0.70 + 0.30 * refine_quality if refined else config.YOLO_ONLY_CONFIDENCE_SCALE
    )
    confidence = round(candidate.score * confidence_scale * 1000.0)
    if confidence < config.MIN_OUTPUT_CONFIDENCE_PERMILLE:
        return BallObservationPacket.invalid(
            sequence=sequence,
            tx_uptime_ms=now_ms & 0xFFFFFFFF,
            reason=BallReason.LOW_CONFIDENCE,
            capture_age_ms=capture_age_ms,
            confidence_permille=max(0, min(1000, confidence)),
        ), center
    return BallObservationPacket(
        sequence=sequence,
        tx_uptime_ms=now_ms & 0xFFFFFFFF,
        valid=True,
        reason=BallReason.OK,
        x_0_1mm=round(x_mm * 10.0),
        confidence_permille=min(1000, confidence),
        capture_age_ms=capture_age_ms,
    ), center


def main() -> None:
    serial = open_uart()
    detector = create_detector()
    cam = camera.Camera(
        detector.input_width(),
        detector.input_height(),
        detector.input_format(),
        fps=config.CAMERA_FPS,
        buff_num=1,
    )
    cam.exposure(config.EXPOSURE_US)
    cam.gain(config.GAIN)
    cam.skip_frames(config.CAMERA_SKIP_FRAMES)
    screen = display.Display() if config.SHOW_PREVIEW else None
    background_bgr = (
        cv2.imread(config.EMPTY_GROOVE_IMAGE) if config.EMPTY_GROOVE_IMAGE else None
    )
    sequence = 0
    minimum_period_ms = max(1, round(1000 / config.MAX_OUTPUT_HZ))

    while not app.need_exit():
        loop_started_ms = time.ticks_ms()
        capture_started_ms = loop_started_ms
        objects = []
        try:
            frame = cam.read()
        except Exception as exc:  # noqa: BLE001 - keep transmitting invalid frames.
            print(f"camera error: {exc}")
            now_ms = time.ticks_ms()
            packet = BallObservationPacket.invalid(
                sequence=sequence,
                tx_uptime_ms=now_ms & 0xFFFFFFFF,
                reason=BallReason.CAMERA_ERROR,
                capture_age_ms=min(0xFFFE, max(0, time.ticks_diff(capture_started_ms))),
            )
            frame = None
            center = None
        else:
            try:
                objects = detector.detect(
                    frame,
                    conf_th=config.YOLO_CONFIDENCE,
                    iou_th=config.YOLO_IOU,
                )
                packet, center = make_packet(
                    sequence, capture_started_ms, frame, objects, background_bgr
                )
            except Exception as exc:  # noqa: BLE001 - keep transmitting invalid frames.
                print(f"model/processing error: {exc}")
                now_ms = time.ticks_ms()
                packet = BallObservationPacket.invalid(
                    sequence=sequence,
                    tx_uptime_ms=now_ms & 0xFFFFFFFF,
                    reason=BallReason.MODEL_ERROR,
                    capture_age_ms=min(
                        0xFFFE, max(0, time.ticks_diff(capture_started_ms))
                    ),
                )
                center = None

        serial.write(packet.encode())
        if screen is not None and frame is not None:
            for obj in objects:
                frame.draw_rect(obj.x, obj.y, obj.w, obj.h, image.COLOR_RED)
            if center is not None:
                frame.draw_cross(
                    round(center[0]), round(center[1]), image.COLOR_GREEN, size=5
                )
            frame.draw_string(
                2,
                2,
                f"v={int(packet.valid)} r={int(packet.reason)} x={packet.x_0_1mm}",
                image.COLOR_GREEN if packet.valid else image.COLOR_RED,
            )
            screen.show(frame)
        sequence = (sequence + 1) & 0xFF
        elapsed_ms = time.ticks_diff(loop_started_ms)
        if elapsed_ms < minimum_period_ms:
            time.sleep_ms(minimum_period_ms - elapsed_ms)


if __name__ == "__main__":
    main()
