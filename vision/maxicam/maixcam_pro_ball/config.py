"""Deployment settings. Verify every item marked UNKNOWN before wiring.

Observed hardware: MaixCAM Pro, MaixPy 4.12.5, camera ID ov_os04a10.
"""

MODEL_KIND = "YOLOv5"  # YOLOv5, YOLOv8, or YOLO11.
MODEL_PATH = "/root/models/steel_ball.mud"
BALL_CLASS_ID = 0
YOLO_CONFIDENCE = 0.45
YOLO_IOU = 0.45
AMBIGUITY_SCORE_MARGIN = 0.08

# MaixCAM Pro UART1 avoids UART0 boot logs. Electrical level is still UNKNOWN.
UART_DEVICE = "/dev/ttyS1"
UART_RX_PIN = "A18"
UART_TX_PIN = "A19"
UART_BAUD = 115200
UART_LEVEL_CONFIRMED = False

# Conventional starting values only; tune one variable at a time on the final lens.
EXPOSURE_US = 5000
GAIN = 100
CAMERA_FPS = 30
CAMERA_SKIP_FRAMES = 30
SHOW_PREVIEW = True

# Normalized rod endpoints: negative side and official +5 cm side.
# Replace from a real empty-groove image, then set CALIBRATION_CONFIRMED True.
ROD_NEG_END_NORM = (0.05, 0.50)
ROD_POS_END_NORM = (0.95, 0.50)
ROD_LENGTH_MM = 250.0
CALIBRATION_CONFIRMED = False

# Optional full-frame empty-groove image. Empty string enables local Otsu fallback.
EMPTY_GROOVE_IMAGE = ""
BACKGROUND_DIFF_THRESHOLD = 24
MIN_OUTPUT_CONFIDENCE_PERMILLE = 500
YOLO_ONLY_CONFIDENCE_SCALE = 0.75
MAX_ABS_POSITION_MM = 130.0
MAX_OUTPUT_HZ = 30
