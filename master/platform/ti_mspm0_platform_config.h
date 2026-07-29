#ifndef H2024_TI_PLATFORM_CONFIG_H
#define H2024_TI_PLATFORM_CONFIG_H

/* Replace these placeholders with measured values before ground testing. */
#define H2024_WHEEL_DIAMETER_MM              (65.0f)//确定是轮值直径
#define H2024_TRACK_WIDTH_MM                 (115.0f)//轮子距离
#define H2024_ENCODER_COUNTS_PER_WHEEL_REV   (724.0f)//实测值，很确定，确定是轮子转一圈的编码器计数
#define H2024_MOTOR_UNITS_PER_MM_S           (1.0f)
#define H2024_MOTOR_BACKEND_TB6612           (1U)
#define H2024_TB6612_SPEED_UNITS_AT_MAX_DUTY (350)
#define H2024_TASK_SPEED_LOOP_ENABLE          (1U)
#define H2024_TASK_SPEED_LOOP_PERIOD_MS       (50U)
#define H2024_TASK_SPEED_KP_MILLI             (250U)
#define H2024_TASK_SPEED_KI_MILLI             (600U)
#define H2024_TASK_SPEED_KD_MILLI             (0U)
#define H2024_TASK_SPEED_LIMIT_PERCENT        (60U)//输出最大占空比
#define H2024_TASK_SPEED_FF_STATIC_MILLI      (6000)
#define H2024_TASK_SPEED_FF_RPM_MILLI         (225U)
#define H2024_TASK_SPEED_DEADBAND_RPM         (2)
/* PROVISIONAL: verify both thresholds with the wheels raised before ground
 * testing. The timeout must cover normal launch friction without masking a
 * disconnected encoder or a mechanically stalled wheel. */
#define H2024_MOTION_WATCHDOG_MIN_TARGET_RPM  (10U)
#define H2024_MOTION_WATCHDOG_TIMEOUT_MS      (600U)
#define H2024_STRAIGHT_HEADING_KP            (4.0f)
#define H2024_IMU_BIAS_DPS                   (-0.45f)
#define H2024_IMU_USE_FIXED_BIAS             (0U)
#define H2024_IMU_CALIBRATION_SAMPLES       (400U)
#define H2024_IMU_YAW_SIGN                   (1)//控制yaw的方向，目前假设向左转yaw增大

/* Reserved for the retained serial motor-board backend. TB6612 task mode
 * ignores these register/PID values and uses the direct adapter below. */
#define H2024_MOTOR_PID_KP                   (40.0f)
#define H2024_MOTOR_PID_KI                   (4.9f)
#define H2024_MOTOR_PID_KD                   (0.0f)
#define H2024_MOTOR_COMMAND_SPACING_MS       (50U)

/* Start line control with P only; tune Ki/Kd after the TB6612 baseline. */
#define H2024_LINE_PID_KP                    (0.025f)//巡线段的pid调控
#define H2024_LINE_PID_KI                    (0.0f)
#define H2024_LINE_PID_KD                    (0.0f)
#define H2024_LINE_PID_INTEGRAL_LIMIT        (5000.0f)

/* 直线终点横线判定；与弧线巡线灵敏度分开调。 */
#define H2024_FINISH_ARM_RATIO               (0.8f)
#define H2024_FINISH_MIN_CONFIDENCE          (1200U)
#define H2024_FINISH_MIN_ACTIVE              (2U)
#define H2024_FINISH_CONSECUTIVE_FRAMES      (2U)

/* D->A rectangle tracking: detect corners/end only in the final 25%. */
#define H2024_LINE_CORNER_MIN_POSITION       (2000)
#define H2024_LINE_CORNER_CONSECUTIVE_FRAMES (2U)
#define H2024_LINE_LOST_CONSECUTIVE_FRAMES   (3U)
#define H2026_LINE_HEADING_KP                 (1.0f)
#define H2026_LINE_HEADING_MAX_CORRECTION     (15.0f)
#define H2026_LINE_FOLLOW_KP                  (0.025f)
#define H2026_LINE_FOLLOW_KI                  (0.0f)
#define H2026_LINE_FOLLOW_KD                  (0.0f)
#define H2026_LINE_FOLLOW_INTEGRAL_LIMIT      (5000.0f)
/* A small forward-only correction lets an edge probe pull the car onto line. */
#define H2026_LINE_SEEK_MAX_CORRECTION_MM_S   (25.0f)
#define H2026_CORNER_MIN_RIGHT_RATIO_PERMILLE (650U)
#define H2026_CORNER_MIN_ACTIVE               (3U)
#define H2026_CORNER_MIN_SPAN                 (3U)
#define H2026_ARC_SENSOR_TO_AXLE_MM           (150.0f)
/* Retained source-level alias for earlier notes and local experiments. */
#define H2026_GRAY_TO_AXLE_MM                 H2026_ARC_SENSOR_TO_AXLE_MM
/* A one-wheel pivot turns around the inner wheel, so the correct corner
 * approach is shorter than the physical gray-row-to-axle distance above. */
#define H2026_CORNER_PIVOT_APPROACH_MM        (50.0f)
#define H2026_CORNER_APPROACH_SPEED_MM_S      (80.0f)
#define H2026_TURN_REACQUIRE_MIN_DEG          (80.0f)
#define H2026_TURN_REACQUIRE_MAX_POSITION     (900)
#define H2026_TURN_REACQUIRE_FRAMES           (2U)

/* H2026 adaptive gray perception. These are RAM-tunable through the task
 * Bluetooth UART; macros remain the power-on defaults. */
#define H2026_GRAY_CENTER_OFFSET               (0)
#define H2026_GRAY_RELATIVE_DELTA               (120U)
#define H2026_GRAY_TRACK_MIN_CONFIDENCE         (250U)
#define H2026_GRAY_TRACK_ENTER_FRAMES           (2U)
#define H2026_GRAY_TRACK_LOST_FRAMES            (3U)
#define H2026_GRAY_TRACK_MAX_ACTIVE             (4U)
#define H2026_GRAY_TRACK_MAX_SPAN               (4U)
#define H2026_GRAY_WIDE_MIN_ACTIVE              (6U)
#define H2026_GRAY_WIDE_MIN_BACKGROUND          (600U)
#define H2026_ARC_LINE_ENTRY_MIN_ANGLE_DEG      (8.0f)
#define H2026_ARC_LINE_ENTRY_FRAMES             (2U)
#define H2026_ARC_LINE_BLEND_MS                 (200U)
#define H2026_DIAGONAL_LINE_ARM_RATIO           (0.75f)
#define H2026_DIAGONAL_LINE_APPROACH_MM         (100.0f)
#define H2026_LINE_CROSS_CENTER_POSITION        (1200)
#define H2026_LINE_CROSS_CAPTURE_WINDOW_MM      (80.0f)
#define H2026_LINE_CROSS_CAPTURE_SPEED_MM_S     (80.0f)
#define H2026_LINE_CROSS_CAPTURE_KP             (0.025f)
#define H2026_LINE_CROSS_CAPTURE_MAX_CORRECTION_MM_S (25.0f)
#define H2026_TURN_INNER_SPEED_RATIO            (0.0f)
#define H2026_TURN_SETTLE_MS                    (300U)
#define H2026_TURN_SETTLE_SPEED_MM_S            (80.0f)
#define H2026_REQUIRED_LINE_SEARCH_MM           (80.0f)
#define H2026_REQUIRED_LINE_SEARCH_SPEED_MM_S   (60.0f)

/* 任务2最新CSV反算：115mm物理轮距对应约153~161mm等效转向轮距。
 * 外环必须显著慢于50ms轮速内环，否则出弯扰动会把目标打成94/12 RPM。 */
#define H2026_ARC_EFFECTIVE_TRACK_WIDTH_MM     (160.0f)
#define H2026_STRAIGHT_HEADING_KP              (1.0f)
#define H2026_STRAIGHT_HEADING_MAX_CORR_MM_S   (25.0f)
#define H2026_ARC_LINE_MAX_CORR_MM_S           (20.0f)
/* Task2-3: the real arc permanently loses the line at 114.5-117.5deg.
 * Arm at 110deg, reject short gaps for 5 control frames, then advance the
 * axle by H2026_ARC_SENSOR_TO_AXLE_MM before handing off to the straight. */
#define H2026_ARC_LINE_EXIT_MIN_ANGLE_DEG       (110.0f)
#define H2026_ARC_LINE_EXIT_LOST_FRAMES         (5U)

/* Official 2026 H bring-up values. 350 mm/s is the measured full-duty
 * software ceiling, so B2 has little timing margin and still needs field
 * validation before claiming the 20 s score threshold. */
#define H2026_B2_STRAIGHT_SPEED_MM_S             (350.0f)
#define H2026_B2_ARC_CENTER_SPEED_MM_S           (300.0f)
#define H2026_BALL_TASK_TRACK_SPEED_MM_S         (240.0f)
#define H2026_FINISH_SENSOR_TO_TEST_POINT_MM     (150.0f)
#define H2026_FINISH_APPROACH_SPEED_MM_S         (180.0f)
#define H2026_TRACK_WHEEL_ACCEL_LIMIT_MM_S2      (1500.0f)

#define H2024_GRAY_SETTLE_US                 (10U)
#define H2024_GRAY_SAMPLES_PER_CHANNEL       (4U)
#define H2024_ADC_TIMEOUT_LOOPS              (100000U)

/* These bounds prevent a failed peripheral from hanging the whole car. */
#define H2024_UART_TX_TIMEOUT_LOOPS          (100000U)
#define H2024_SPI_TIMEOUT_LOOPS              (100000U)
#define H2024_UART_RX_BUFFER_SIZE            (64U)

#endif
