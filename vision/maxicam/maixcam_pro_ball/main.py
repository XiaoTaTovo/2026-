"""YOLO steel-ball position with pipe ROI and touchscreen tuning."""

import cv2

from maix import app, camera, display, err, image, nn, pinmap, time, touchscreen, uart

from ball_protocol import BallObservationPacket, BallReason

from pipe_detector import (
    detect_white_pipe,
    pipe_relative_position,
    pipe_roi_bbox,
    relative_to_centered_mm,
)


BUILD_ID = "yolo-performance-r12"
MODEL_PATH = "/root/models/steel_ball3_int8.mud"
PIPE_LENGTH_MM = 250.0
CONFIDENCE_THRESHOLD = 0.30
IOU_THRESHOLD = 0.45

MANUAL_EXPOSURE_US = 5000
MANUAL_GAIN = 100
EXPOSURE_STEP_US = 1000
EXPOSURE_MIN_US = 1000
EXPOSURE_MAX_US = 30000
PIPE_VALUE_STEP = 5
PIPE_SATURATION_STEP = 5
CONFIDENCE_STEP = 0.05

# Provisional starting values. Tune from real MaixCAM images.
PIPE_MIN_VALUE = 160
PIPE_MAX_SATURATION = 70
PIPE_MIN_AREA_RATIO = 0.01
PIPE_MIN_ASPECT_RATIO = 8.0
PIPE_MIN_FILL_RATIO = 0.60
PIPE_ROI_WIDTH_MARGIN_RATIO = 0.35
PRINT_EVERY_N_FRAMES = 30
PIPE_DETECT_EVERY_N_FRAMES = 10
DISPLAY_EVERY_N_FRAMES = 2
MASK_PREVIEW_TIMEOUT_MS = 5000
BUTTON_HEIGHT = 28

# Existing MaixCAM Pro wiring: A19(TX) -> STM32 RX, with common ground.
UART_DEVICE = "/dev/ttyS1"
UART_RX_PIN = "A18"
UART_TX_PIN = "A19"
UART_BAUD = 115200
UART_LEVEL_CONFIRMED = True


def open_uart():
    if not UART_LEVEL_CONFIRMED:
        raise RuntimeError("confirm MaixCAM Pro/STM32 UART voltage before enabling TX")
    mappings = {
        UART_RX_PIN: "UART1_RX",
        UART_TX_PIN: "UART1_TX",
    }
    for pin, function in mappings.items():
        err.check_raise(
            pinmap.set_pin_function(pin, function),
            f"failed to map {pin} to {function}",
        )
    return uart.UART(UART_DEVICE, UART_BAUD)


def make_uart_packet(
    sequence,
    capture_started_ms,
    pipe,
    objects,
    selected_ball,
    selected_x_mm,
):
    """Build one observation frame from this camera cycle."""
    now_ms = time.ticks_ms()
    capture_age_ms = min(
        0xFFFE,
        max(0, int(time.ticks_diff(capture_started_ms, now_ms))),
    )
    if selected_ball is not None and selected_x_mm is not None:
        obj = selected_ball[0]
        return BallObservationPacket(
            sequence=sequence & 0xFF,
            tx_uptime_ms=now_ms & 0xFFFFFFFF,
            valid=True,
            reason=BallReason.OK,
            x_0_1mm=round(selected_x_mm * 10.0),
            confidence_permille=min(1000, max(0, round(obj.score * 1000.0))),
            capture_age_ms=capture_age_ms,
        )

    if pipe is None:
        reason = BallReason.CALIBRATION_INVALID
    elif objects:
        reason = BallReason.OUT_OF_ROI
    else:
        reason = BallReason.NO_BALL
    return BallObservationPacket.invalid(
        sequence=sequence & 0xFF,
        tx_uptime_ms=now_ms & 0xFFFFFFFF,
        reason=reason,
        capture_age_ms=capture_age_ms,
    )


def _button_rects(frame_width: int, frame_height: int):
    specs = (
        ("exp_down", "E-"),
        ("exp_up", "E+"),
        ("auto", "A"),
        ("value_down", "V-"),
        ("value_up", "V+"),
        ("saturation_down", "S-"),
        ("saturation_up", "S+"),
        ("confidence_down", "C-"),
        ("confidence_up", "C+"),
        ("calibrate", "R"),
        ("view", "M"),
        ("exit", "X"),
    )
    y = frame_height - BUTTON_HEIGHT
    x = 2
    step = max(24, (frame_width - 4) // len(specs))
    button_width = step - 2
    result = {}
    for name, label in specs:
        result[name] = ((x, y + 2, button_width, BUTTON_HEIGHT - 4), label)
        x += step
    return result


def _is_in_rect(x: int, y: int, rect) -> bool:
    return rect[0] <= x < rect[0] + rect[2] and rect[1] <= y < rect[1] + rect[3]


def _map_rect_to_display(frame, screen, rect):
    return image.resize_map_pos(
        frame.width(),
        frame.height(),
        screen.width(),
        screen.height(),
        image.Fit.FIT_CONTAIN,
        *rect,
    )


def _button_action_at(x: int, y: int, display_buttons):
    for name, (rect, _) in display_buttons.items():
        if _is_in_rect(x, y, rect):
            return name
    return None


def _draw_controls(
    frame,
    buttons,
    *,
    auto_exposure: bool,
    exposure_us: int,
    pipe_min_value: int,
    pipe_max_saturation: int,
    confidence_threshold: float,
    fps: float,
    show_mask: bool,
    calibration_locked: bool,
    calibration_requested: bool,
) -> None:
    y = frame.height() - BUTTON_HEIGHT
    frame.draw_rect(
        0,
        y - 16,
        frame.width(),
        BUTTON_HEIGHT + 16,
        image.COLOR_BLACK,
        -1,
    )
    for name, (rect, label) in buttons.items():
        frame.draw_rect(*rect, image.COLOR_WHITE, 1)
        if name == "auto" and auto_exposure:
            color = image.COLOR_GREEN
        elif name == "view" and show_mask:
            color = image.COLOR_GREEN
        elif name == "calibrate" and calibration_locked:
            color = image.COLOR_GREEN
        elif name == "calibrate" and calibration_requested:
            color = image.COLOR_YELLOW
        else:
            color = image.COLOR_WHITE
        frame.draw_string(rect[0] + 5, rect[1] + 7, label, color)

    exposure_text = "AE" if auto_exposure else f"E={exposure_us}"
    frame.draw_string(
        2,
        y - 15,
        f"F={fps:.1f} {exposure_text} V={pipe_min_value} "
        f"S={pipe_max_saturation} C={confidence_threshold:.2f}",
        image.COLOR_WHITE,
    )


def _render_display_frame(
    frame,
    pipe_mask,
    show_mask,
    objects,
    detector,
    pipe,
    selected_ball,
    selected_x_mm,
    buttons,
    auto_exposure,
    exposure_us,
    pipe_min_value,
    pipe_max_saturation,
    confidence_threshold,
    fps,
    calibration_locked,
    calibration_requested,
):
    if show_mask and pipe_mask is not None:
        mask_bgr = cv2.cvtColor(pipe_mask, cv2.COLOR_GRAY2BGR)
        display_frame = image.cv2image(mask_bgr, bgr=True, copy=False)
    else:
        display_frame = frame

    for obj in objects:
        display_frame.draw_rect(
            obj.x,
            obj.y,
            obj.w,
            obj.h,
            color=image.COLOR_RED,
        )
        label = detector.labels[obj.class_id]
        display_frame.draw_string(
            obj.x,
            obj.y,
            f"{label}: {obj.score:.2f}",
            color=image.COLOR_RED,
        )

    if pipe is None:
        display_frame.draw_string(2, 2, "PIPE NOT FOUND", color=image.COLOR_BLUE)
    else:
        x, y, width, height = pipe["bbox"]
        display_frame.draw_rect(x, y, width, height, image.COLOR_BLUE, 2)
        roi_rect = pipe_roi_bbox(
            pipe,
            display_frame.width(),
            display_frame.height(),
            width_margin_ratio=PIPE_ROI_WIDTH_MARGIN_RATIO,
        )
        display_frame.draw_rect(*roi_rect, image.COLOR_YELLOW, 1)
        end_a_x, end_a_y = pipe["end_a"]
        end_b_x, end_b_y = pipe["end_b"]
        display_frame.draw_circle(
            round(end_a_x), round(end_a_y), 3, image.COLOR_BLUE, 2
        )
        display_frame.draw_circle(
            round(end_b_x), round(end_b_y), 3, image.COLOR_YELLOW, 2
        )
        display_frame.draw_string(
            round(end_a_x), max(0, round(end_a_y) - 12), "A", image.COLOR_BLUE
        )
        display_frame.draw_string(
            round(end_b_x), max(0, round(end_b_y) - 12), "B", image.COLOR_YELLOW
        )

    if selected_ball is not None:
        selected_obj, ball_center, ball_position = selected_ball
        display_frame.draw_rect(
            selected_obj.x,
            selected_obj.y,
            selected_obj.w,
            selected_obj.h,
            image.COLOR_GREEN,
            2,
        )
        display_frame.draw_cross(
            round(ball_center[0]),
            round(ball_center[1]),
            image.COLOR_GREEN,
            size=5,
        )
        display_frame.draw_string(
            selected_obj.x,
            min(
                display_frame.height() - BUTTON_HEIGHT - 18,
                selected_obj.y + selected_obj.h,
            ),
            f"X={selected_x_mm:+.1f}mm P={ball_position['percent']:.1f}%",
            color=image.COLOR_GREEN,
        )

    _draw_controls(
        display_frame,
        buttons,
        auto_exposure=auto_exposure,
        exposure_us=exposure_us,
        pipe_min_value=pipe_min_value,
        pipe_max_saturation=pipe_max_saturation,
        confidence_threshold=confidence_threshold,
        fps=fps,
        show_mask=show_mask,
        calibration_locked=calibration_locked,
        calibration_requested=calibration_requested,
    )
    return display_frame


def main() -> None:
    detector = nn.YOLO11(model=MODEL_PATH, dual_buff=True)
    input_width = detector.input_width()
    input_height = detector.input_height()

    cam = camera.Camera(
        input_width,
        input_height,
        detector.input_format(),
    )
    serial = open_uart()
    screen = display.Display()
    try:
        touch = touchscreen.TouchScreen()
    except Exception as exc:
        touch = None
        print(f"touchscreen disabled: {exc}", flush=True)

    sequence = 0
    auto_exposure = True
    exposure_us = MANUAL_EXPOSURE_US
    pipe_min_value = PIPE_MIN_VALUE
    pipe_max_saturation = PIPE_MAX_SATURATION
    confidence_threshold = CONFIDENCE_THRESHOLD
    display_fps = 0.0
    status_started_ms = time.ticks_ms()
    status_frame_count = 0
    touch_was_pressed = False
    touch_start_action = None
    touch_last_x = 0
    touch_last_y = 0
    live_pipe = None
    locked_pipe = None
    calibration_requested = False
    pipe_mask = None
    pipe_stats = {"raw": 0, "area": 0, "aspect": 0, "fill": 0}
    force_pipe_update = True
    show_mask = False
    mask_preview_started_ms = None
    capture_total_ms = 0
    pipe_total_ms = 0
    yolo_total_ms = 0
    draw_total_ms = 0
    display_total_ms = 0
    display_update_count = 0
    pipe_update_count = 0

    print(
        f"ready: build={BUILD_ID} model={MODEL_PATH} "
        f"input={input_width}x{input_height} uart={UART_DEVICE}@{UART_BAUD}",
        flush=True,
    )

    while not app.need_exit():
        first_frame = sequence == 0
        if (
            show_mask
            and mask_preview_started_ms is not None
            and time.ticks_diff(mask_preview_started_ms, time.ticks_ms())
            >= MASK_PREVIEW_TIMEOUT_MS
        ):
            show_mask = False
            mask_preview_started_ms = None
            print("control: mask preview timeout, view=camera", flush=True)
        if first_frame:
            print("[DEBUG-yolo-r6] first camera read begin", flush=True)
        capture_started_ms = time.ticks_ms()
        frame = cam.read()
        capture_total_ms += time.ticks_diff(capture_started_ms, time.ticks_ms())
        if first_frame:
            print("[DEBUG-yolo-r6] first camera read complete", flush=True)
            print("[DEBUG-yolo-r6] first pipe detection begin", flush=True)

        pipe_started_ms = time.ticks_ms()
        update_pipe = locked_pipe is None and (
            force_pipe_update
            or live_pipe is None
            or show_mask
            or sequence % PIPE_DETECT_EVERY_N_FRAMES == 0
        )
        if update_pipe:
            frame_rgb = image.image2cv(frame, ensure_bgr=False, copy=False)
            live_pipe, pipe_mask, pipe_stats = detect_white_pipe(
                frame_rgb,
                min_value=pipe_min_value,
                max_saturation=pipe_max_saturation,
                min_area_ratio=PIPE_MIN_AREA_RATIO,
                min_aspect_ratio=PIPE_MIN_ASPECT_RATIO,
                min_fill_ratio=PIPE_MIN_FILL_RATIO,
            )
            force_pipe_update = False
            pipe_update_count += 1
            pipe_total_ms += time.ticks_diff(pipe_started_ms, time.ticks_ms())
            if calibration_requested and live_pipe is not None:
                # Keep the detected geometry immutable until the user releases it.
                locked_pipe = dict(live_pipe)
                calibration_requested = False
                print("control: calibration locked", flush=True)
        pipe = locked_pipe if locked_pipe is not None else live_pipe
        if first_frame:
            print(
                f"[DEBUG-yolo-r6] first pipe detection complete: "
                f"found={int(pipe is not None)} raw={pipe_stats['raw']}",
                flush=True,
            )
            print("[DEBUG-yolo-r6] first YOLO detection begin", flush=True)

        yolo_started_ms = time.ticks_ms()
        objects = detector.detect(
            frame,
            conf_th=confidence_threshold,
            iou_th=IOU_THRESHOLD,
        )
        yolo_total_ms += time.ticks_diff(yolo_started_ms, time.ticks_ms())
        if first_frame:
            print(
                f"[DEBUG-yolo-r6] first YOLO detection complete: "
                f"objects={len(objects)}",
                flush=True,
            )

        valid_balls = []
        if pipe is not None:
            for obj in objects:
                center = (obj.x + 0.5 * obj.w, obj.y + 0.5 * obj.h)
                position = pipe_relative_position(
                    pipe,
                    center,
                    width_margin_ratio=PIPE_ROI_WIDTH_MARGIN_RATIO,
                )
                if position is not None:
                    valid_balls.append((obj, center, position))
        valid_balls.sort(key=lambda item: item[0].score, reverse=True)
        selected_ball = valid_balls[0] if valid_balls else None
        selected_x_mm = None
        if selected_ball is not None:
            selected_x_mm = relative_to_centered_mm(
                selected_ball[2]["relative"],
                PIPE_LENGTH_MM,
            )

        packet = make_uart_packet(
            sequence,
            capture_started_ms,
            pipe,
            objects,
            selected_ball,
            selected_x_mm,
        )
        serial.write(packet.encode())

        buttons = _button_rects(frame.width(), frame.height())
        display_buttons = {
            name: (_map_rect_to_display(frame, screen, rect), label)
            for name, (rect, label) in buttons.items()
        }
        if touch is None:
            touch_x, touch_y, touch_pressed = 0, 0, False
        else:
            touch_x, touch_y, touch_pressed = touch.read()

        if touch_pressed:
            if not touch_was_pressed:
                touch_start_action = _button_action_at(
                    touch_x,
                    touch_y,
                    display_buttons,
                )
            touch_was_pressed = True
            touch_last_x = touch_x
            touch_last_y = touch_y
        elif touch_was_pressed:
            touch_end_action = _button_action_at(
                touch_last_x,
                touch_last_y,
                display_buttons,
            )
            action = (
                touch_start_action
                if touch_start_action == touch_end_action
                else None
            )
            if action == "exp_down":
                if locked_pipe is not None or calibration_requested:
                    locked_pipe = None
                    calibration_requested = False
                    print("control: calibration unlocked (exposure changed)", flush=True)
                auto_exposure = False
                exposure_us = max(EXPOSURE_MIN_US, exposure_us - EXPOSURE_STEP_US)
                cam.exposure(exposure_us)
                cam.gain(MANUAL_GAIN)
                force_pipe_update = True
                print(f"control: manual exposure={exposure_us}us", flush=True)
            elif action == "exp_up":
                if locked_pipe is not None or calibration_requested:
                    locked_pipe = None
                    calibration_requested = False
                    print("control: calibration unlocked (exposure changed)", flush=True)
                auto_exposure = False
                exposure_us = min(EXPOSURE_MAX_US, exposure_us + EXPOSURE_STEP_US)
                cam.exposure(exposure_us)
                cam.gain(MANUAL_GAIN)
                force_pipe_update = True
                print(f"control: manual exposure={exposure_us}us", flush=True)
            elif action == "auto":
                if locked_pipe is not None or calibration_requested:
                    locked_pipe = None
                    calibration_requested = False
                    print("control: calibration unlocked (exposure changed)", flush=True)
                auto_exposure = True
                cam.exp_mode(camera.AeMode.Auto)
                force_pipe_update = True
                print("control: auto exposure", flush=True)
            elif action == "value_down":
                if locked_pipe is not None or calibration_requested:
                    locked_pipe = None
                    calibration_requested = False
                    print("control: calibration unlocked (threshold changed)", flush=True)
                pipe_min_value = max(0, pipe_min_value - PIPE_VALUE_STEP)
                force_pipe_update = True
            elif action == "value_up":
                if locked_pipe is not None or calibration_requested:
                    locked_pipe = None
                    calibration_requested = False
                    print("control: calibration unlocked (threshold changed)", flush=True)
                pipe_min_value = min(255, pipe_min_value + PIPE_VALUE_STEP)
                force_pipe_update = True
            elif action == "saturation_down":
                if locked_pipe is not None or calibration_requested:
                    locked_pipe = None
                    calibration_requested = False
                    print("control: calibration unlocked (threshold changed)", flush=True)
                pipe_max_saturation = max(
                    0, pipe_max_saturation - PIPE_SATURATION_STEP
                )
                force_pipe_update = True
            elif action == "saturation_up":
                if locked_pipe is not None or calibration_requested:
                    locked_pipe = None
                    calibration_requested = False
                    print("control: calibration unlocked (threshold changed)", flush=True)
                pipe_max_saturation = min(
                    255, pipe_max_saturation + PIPE_SATURATION_STEP
                )
                force_pipe_update = True
            elif action == "confidence_down":
                confidence_threshold = max(
                    0.05, confidence_threshold - CONFIDENCE_STEP
                )
            elif action == "confidence_up":
                confidence_threshold = min(
                    0.95, confidence_threshold + CONFIDENCE_STEP
                )
            elif action == "calibrate":
                if locked_pipe is not None:
                    locked_pipe = None
                    calibration_requested = False
                    force_pipe_update = True
                    print("control: calibration unlocked", flush=True)
                elif calibration_requested:
                    calibration_requested = False
                    print("control: calibration cancelled", flush=True)
                else:
                    calibration_requested = True
                    force_pipe_update = True
                    print("control: calibration requested", flush=True)
            elif action == "view":
                show_mask = not show_mask
                mask_preview_started_ms = time.ticks_ms() if show_mask else None
                if locked_pipe is None:
                    force_pipe_update = True
                print(
                    f"control: view={'mask' if show_mask else 'camera'}",
                    flush=True,
                )
            elif action == "exit":
                app.set_exit_flag(True)
            touch_was_pressed = False
            touch_start_action = None

        display_due = first_frame or sequence % DISPLAY_EVERY_N_FRAMES == 0
        if display_due:
            draw_started_ms = time.ticks_ms()
            display_frame = _render_display_frame(
                frame,
                pipe_mask,
                show_mask,
                objects,
                detector,
                pipe,
                selected_ball,
                selected_x_mm,
                buttons,
                auto_exposure,
                exposure_us,
                pipe_min_value,
                pipe_max_saturation,
                confidence_threshold,
                display_fps,
                locked_pipe is not None,
                calibration_requested,
            )
            draw_total_ms += time.ticks_diff(draw_started_ms, time.ticks_ms())
            if first_frame:
                print("[DEBUG-yolo-r6] first display begin", flush=True)
            display_started_ms = time.ticks_ms()
            screen.show(display_frame)
            display_total_ms += time.ticks_diff(
                display_started_ms,
                time.ticks_ms(),
            )
            display_update_count += 1
        sequence += 1
        status_frame_count += 1
        if first_frame:
            print("[DEBUG-yolo-r6] first display complete", flush=True)
            print("[DEBUG-yolo-r6] first iteration complete", flush=True)

        if status_frame_count >= PRINT_EVERY_N_FRAMES:
            status_now_ms = time.ticks_ms()
            elapsed_ms = time.ticks_diff(status_started_ms, status_now_ms)
            fps = (
                1000.0 * status_frame_count / elapsed_ms if elapsed_ms > 0 else 0.0
            )
            display_fps = fps
            if pipe is None:
                pipe_detail = "found=0"
            else:
                pipe_detail = (
                    f"found=1 bbox={pipe['bbox']} area={pipe['area']:.1f} "
                    f"aspect={pipe['aspect_ratio']:.2f} "
                    f"fill={pipe['fill_ratio']:.2f} angle={pipe['angle']:.1f}"
                )
            if selected_ball is None:
                ball_detail = "found=0"
            else:
                _, ball_center, ball_position = selected_ball
                ball_detail = (
                    f"found=1 center=({ball_center[0]:.1f},{ball_center[1]:.1f}) "
                    f"relative={ball_position['relative']:.4f} "
                    f"percent={ball_position['percent']:.1f} "
                    f"x_mm={selected_x_mm:+.1f}"
                )
            print(
                f"status: seq={sequence} yolo={len(objects)} "
                f"valid={len(valid_balls)} ball={ball_detail} pipe={pipe_detail} "
                f"raw={pipe_stats['raw']} area_ok={pipe_stats['area']} "
                f"aspect_ok={pipe_stats['aspect']} fill_ok={pipe_stats['fill']} "
                f"fps={fps:.1f} ae={int(auto_exposure)} exp={exposure_us}us "
                f"lock={int(locked_pipe is not None)} "
                f"view={'mask' if show_mask else 'camera'} "
                f"value={pipe_min_value} saturation={pipe_max_saturation} "
                f"conf={confidence_threshold:.2f} "
                f"uart_v={int(packet.valid)} uart_r={int(packet.reason)} "
                f"uart_x={packet.x_0_1mm} "
                f"cap={capture_total_ms / status_frame_count:.1f}ms "
                f"pipe={pipe_total_ms / max(1, pipe_update_count):.1f}ms/update "
                f"pipe_n={pipe_update_count} "
                f"yolo_ms={yolo_total_ms / status_frame_count:.1f} "
                f"draw={draw_total_ms / status_frame_count:.1f}ms "
                f"disp={display_total_ms / max(1, display_update_count):.1f}ms/update "
                f"disp_n={display_update_count}",
                flush=True,
            )
            status_started_ms = status_now_ms
            status_frame_count = 0
            capture_total_ms = 0
            pipe_total_ms = 0
            yolo_total_ms = 0
            draw_total_ms = 0
            display_total_ms = 0
            display_update_count = 0
            pipe_update_count = 0

    print(f"loop exited: frames={sequence}", flush=True)


if __name__ == "__main__":
    main()
