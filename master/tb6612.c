#include "tb6612.h"

#include "encoder.h"
#include "ti_msp_dl_config.h"

#define TB6612_PI_F (3.14159265358979323846f)

static int8_t gLeftCommand;
static int8_t gRightCommand;

static int8_t clamp_percent(int8_t percent)
{
    if (percent > (int8_t) TB6612_MAX_DUTY_PERCENT) {
        return (int8_t) TB6612_MAX_DUTY_PERCENT;
    }
    if (percent < -(int8_t) TB6612_MAX_DUTY_PERCENT) {
        return -(int8_t) TB6612_MAX_DUTY_PERCENT;
    }
    return percent;
}

static uint32_t percent_to_compare(int8_t percent)
{
    uint32_t magnitude = (percent < 0) ? (uint32_t) (-percent) : (uint32_t) percent;
    uint32_t activeTicks = (TB6612_PWM_PERIOD_TICKS * magnitude) / 100U;

    return TB6612_PWM_PERIOD_TICKS - activeTicks;
}//把占空比变成比较寄存器的值
//我们是递减计数，所以类似递增计数，数了多少个数字就用它来算占空比

static void set_pwm(int8_t leftPercent, int8_t rightPercent)
{
    DL_TimerA_setCaptureCompareValue(PWM_TB1_INST,
        percent_to_compare(leftPercent), GPIO_PWM_TB1_C1_IDX);
    DL_TimerA_setCaptureCompareValue(PWM_TB1_INST,
        percent_to_compare(rightPercent), GPIO_PWM_TB1_C2_IDX);
}

static int8_t units_to_percent(int16_t units,
                               int16_t units_at_max_duty)
{
    int32_t scaled;
    int32_t denominator = (units_at_max_duty > 0) ?
        units_at_max_duty : 350;//规定映射

    scaled = (int32_t)units * (int32_t)TB6612_MAX_DUTY_PERCENT;
    if (scaled >= 0) {
        scaled = (scaled + denominator / 2) / denominator;
    } else {
        scaled = (scaled - denominator / 2) / denominator;
    }
    if (scaled > (int32_t)TB6612_MAX_DUTY_PERCENT) {
        scaled = TB6612_MAX_DUTY_PERCENT;
    } else if (scaled < -(int32_t)TB6612_MAX_DUTY_PERCENT) {
        scaled = -(int32_t)TB6612_MAX_DUTY_PERCENT;
    }
    return (int8_t)scaled;
}

static int32_t round_signed(float value)
{
    return (int32_t)(value + ((value >= 0.0f) ? 0.5f : -0.5f));
}

static int32_t speed_to_rpm(const TB6612MotorBoardContext *context,
                            int16_t speed_mm_s)
{
    float circumference_mm = TB6612_PI_F *
        context->speed_loop.wheel_diameter_mm;

    if (circumference_mm <= 0.0f) {
        return 0;
    }
    return round_signed(((float)speed_mm_s * 60.0f) / circumference_mm);
}

static int32_t calculate_rpm(int32_t delta_count,
                             uint32_t counts_per_rev,
                             uint32_t elapsed_ms)
{
    int64_t numerator;
    int64_t denominator;

    if ((counts_per_rev == 0U) || (elapsed_ms == 0U)) {
        return 0;
    }
    numerator = (int64_t)delta_count * 60000;
    denominator = (int64_t)counts_per_rev * elapsed_ms;
    numerator += (numerator >= 0) ? denominator / 2 : -denominator / 2;
    return (int32_t)(numerator / denominator);
}

static int8_t run_speed_pid(const TB6612SpeedLoopConfig *config,
                            int32_t target_rpm,
                            int32_t measured_rpm,
                            uint32_t elapsed_ms,
                            int64_t *integral_milli,
                            int32_t *previous_error)
{
    int32_t error;
    int32_t target_magnitude;
    int64_t proportional;
    int64_t derivative;
    int64_t feedforward;
    int64_t integral_candidate;
    int64_t integral_delta;
    int64_t unclamped_output;
    int64_t output;
    int64_t limit_milli = (int64_t)config->output_limit_percent * 1000;
    int64_t min_output;
    int64_t max_output;

    if (target_rpm == 0) {
        *integral_milli = 0;
        *previous_error = 0;
        return 0;
    }

    error = target_rpm - measured_rpm;
    if ((error <= config->error_deadband_rpm) &&
        (error >= -config->error_deadband_rpm)) {
        error = 0;
    }
    target_magnitude = (target_rpm < 0) ? -target_rpm : target_rpm;
    feedforward = config->feedforward_static_milli +
                  (int64_t)config->feedforward_rpm_milli * target_magnitude;
    if (target_rpm < 0) {
        feedforward = -feedforward;
    }

    proportional = (int64_t)config->kp_milli * error;
    integral_delta = ((int64_t)config->ki_milli * error * elapsed_ms) / 1000;
    integral_candidate = *integral_milli + integral_delta;
    if (integral_candidate > limit_milli) {
        integral_candidate = limit_milli;
    } else if (integral_candidate < -limit_milli) {
        integral_candidate = -limit_milli;
    }
    derivative = ((int64_t)config->kd_milli *
                  (error - *previous_error) * 1000) / elapsed_ms;
    *previous_error = error;

    if (target_rpm > 0) {
        min_output = 0;
        max_output = limit_milli;
    } else {
        min_output = -limit_milli;
        max_output = 0;
    }
    unclamped_output = feedforward + proportional +
                       integral_candidate + derivative;
    if (!(((unclamped_output > max_output) && (integral_delta > 0)) ||
          ((unclamped_output < min_output) && (integral_delta < 0)))) {
        *integral_milli = integral_candidate;
    }

    output = feedforward + proportional + *integral_milli + derivative;
    if (output < min_output) {
        output = min_output;
    } else if (output > max_output) {
        output = max_output;
    }
    output = (output >= 0) ? (output + 500) / 1000 :
                             (output - 500) / 1000;
    return (int8_t)output;
}

static void reset_speed_loop_state(TB6612MotorBoardContext *context)
{
    context->target_left_rpm = 0;
    context->target_right_rpm = 0;
    context->measured_left_rpm = 0;
    context->measured_right_rpm = 0;
    context->left_delta_count = 0;
    context->right_delta_count = 0;
    context->left_previous_error = 0;
    context->right_previous_error = 0;
    context->left_integral_milli = 0;
    context->right_integral_milli = 0;
    context->last_sample_elapsed_ms = 0U;
    context->encoder_sample_ready = false;
    context->speed_filter_ready = false;
}

static void update_speed_loop(TB6612MotorBoardContext *context,
                              uint32_t now_ms,
                              uint32_t elapsed_ms)
{
    int32_t left_count = Encoder_GetLeftCount();
    int32_t right_count = Encoder_GetRightCount();
    int32_t raw_left_rpm;
    int32_t raw_right_rpm;
    int8_t left_output;
    int8_t right_output;

    context->left_delta_count = (int32_t)((uint32_t)left_count -
                                           (uint32_t)context->previous_left_count);
    context->right_delta_count = (int32_t)((uint32_t)right_count -
                                            (uint32_t)context->previous_right_count);
    context->previous_left_count = left_count;
    context->previous_right_count = right_count;
    context->previous_control_ms = now_ms;
    context->last_sample_elapsed_ms = elapsed_ms;

    raw_left_rpm = calculate_rpm(context->left_delta_count,
                                 context->speed_loop.left_counts_per_rev,
                                 elapsed_ms);
    raw_right_rpm = calculate_rpm(context->right_delta_count,
                                  context->speed_loop.right_counts_per_rev,
                                  elapsed_ms);
    if (!context->speed_filter_ready) {
        context->measured_left_rpm = raw_left_rpm;
        context->measured_right_rpm = raw_right_rpm;
        context->speed_filter_ready = true;
    } else {
        context->measured_left_rpm =
            (context->measured_left_rpm + raw_left_rpm) / 2;
        context->measured_right_rpm =
            (context->measured_right_rpm + raw_right_rpm) / 2;
    }

    left_output = run_speed_pid(&context->speed_loop,
                                context->target_left_rpm,
                                context->measured_left_rpm,
                                elapsed_ms,
                                &context->left_integral_milli,
                                &context->left_previous_error);
    right_output = run_speed_pid(&context->speed_loop,
                                 context->target_right_rpm,
                                 context->measured_right_rpm,
                                 elapsed_ms,
                                 &context->right_integral_milli,
                                 &context->right_previous_error);
    TB6612_SetMotors(left_output, right_output);
    context->update_count++;
}
//units_at_max_duty这个参数的意思是最大占空比的时候的这个对应的速度，单位是mm/s,但是现在是开环，也从来没有测试过，以后变成闭环
// 占空比% = 速度指令 / 350 × 80
//现在速度的规定映射是 350 ，对应最大占空比是80
//也是很好理解 速度350对应百分之80，那么要求一个指定速度，对应除以速度350即可再乘以80
//所以想要多少占空比就是这个规定映射乘以对应的百分比即可
static void set_left_direction(int8_t percent)
{//先都清零，clear函数是清零的：确保没有一瞬间都是1，这个叫刹车状态（电机两端短接，强行按住）会有卡顿和异响
    //然后根据我们这个left的值来确定正反转和我们要的前进方向的关系
    DL_GPIO_clearPins(A_PORT, A_PIN_AIN1_PIN | A_PIN_AIN2_PIN);
    if (percent == 0) {
        return;
    }

    if ((percent > 0) == (TB6612_LEFT_FORWARD_IN1_HIGH != 0)) {
        DL_GPIO_setPins(A_PORT, A_PIN_AIN1_PIN);
    } else {
        DL_GPIO_setPins(A_PORT, A_PIN_AIN2_PIN);
    }
}

static void set_right_direction(int8_t percent)
{
    DL_GPIO_clearPins(B_PORT, B_PIN_BIN1_PIN | B_PIN_BIN2_PIN);
    if (percent == 0) {
        return;
    }

    if ((percent > 0) == (TB6612_RIGHT_FORWARD_IN1_HIGH != 0)) {
        DL_GPIO_setPins(B_PORT, B_PIN_BIN1_PIN);
    } else {
        DL_GPIO_setPins(B_PORT, B_PIN_BIN2_PIN);
    }
}

void TB6612_Init(void)
{
    gLeftCommand  = 0;
    gRightCommand = 0;
    set_pwm(0, 0);
    DL_GPIO_clearPins(A_PORT, A_PIN_AIN1_PIN | A_PIN_AIN2_PIN);
    DL_GPIO_clearPins(B_PORT, B_PIN_BIN1_PIN | B_PIN_BIN2_PIN);
    DL_GPIO_clearPins(STBY_PORT, STBY_PIN_STBY_PIN);
    DL_TimerA_startCounter(PWM_TB1_INST);
}

void TB6612_Stop(void)
{
    set_pwm(0, 0);
    DL_GPIO_clearPins(A_PORT, A_PIN_AIN1_PIN | A_PIN_AIN2_PIN);
    DL_GPIO_clearPins(B_PORT, B_PIN_BIN1_PIN | B_PIN_BIN2_PIN);
    DL_GPIO_clearPins(STBY_PORT, STBY_PIN_STBY_PIN);
    gLeftCommand  = 0;
    gRightCommand = 0;
}

void TB6612_SetMotors(int8_t leftPercent, int8_t rightPercent)
{
    leftPercent  = clamp_percent(leftPercent);
    rightPercent = clamp_percent(rightPercent);

    if ((leftPercent == 0) && (rightPercent == 0)) {
        TB6612_Stop();
        return;
    }

    /* Blank both PWM channels before touching direction and STBY. */
    set_pwm(0, 0);
    DL_GPIO_clearPins(STBY_PORT, STBY_PIN_STBY_PIN);
    set_left_direction(leftPercent);
    set_right_direction(rightPercent);
    DL_GPIO_setPins(STBY_PORT, STBY_PIN_STBY_PIN);
    delay_cycles(CPUCLK_FREQ / 1000000U);
    set_pwm(leftPercent, rightPercent);

    gLeftCommand  = leftPercent;
    gRightCommand = rightPercent;
}

int8_t TB6612_GetLeftCommand(void)
{
    return gLeftCommand;
}

int8_t TB6612_GetRightCommand(void)
{
    return gRightCommand;
}

void TB6612_MotorBoardContextInit(TB6612MotorBoardContext *context,
                                  TB6612NowFn now_ms,
                                  void *now_context,
                                  int16_t speed_units_at_max_duty)
{
    if (context == 0) {
        return;
    }
    *context = (TB6612MotorBoardContext){0};
    context->now_ms = now_ms;
    context->now_context = now_context;
    context->speed_units_at_max_duty = speed_units_at_max_duty;
}

bool TB6612_MotorBoardConfigureSpeedLoop(
    TB6612MotorBoardContext *context,
    const TB6612SpeedLoopConfig *config)
{
    CarMotionWatchdogConfig watchdog_config;

    if ((context == 0) || (config == 0) || (context->now_ms == 0) ||
        (config->wheel_diameter_mm <= 0.0f) ||
        (config->left_counts_per_rev == 0U) ||
        (config->right_counts_per_rev == 0U) ||
        (config->control_period_ms == 0U) ||
        (config->output_limit_percent == 0U) ||
        (config->output_limit_percent > TB6612_MAX_DUTY_PERCENT) ||
        (config->motion_watchdog_min_target_rpm == 0U) ||
        (config->motion_watchdog_timeout_ms == 0U)) {
        return false;
    }
    watchdog_config = (CarMotionWatchdogConfig){
        config->motion_watchdog_min_target_rpm,
        config->motion_watchdog_timeout_ms
    };
    if (!CarMotionWatchdog_Init(&context->motion_watchdog,
                                &watchdog_config)) {
        return false;
    }
    context->speed_loop = *config;
    context->speed_loop_enabled = true;
    context->motion_watchdog_enabled = true;
    context->update_count = 0U;
    reset_speed_loop_state(context);
    return true;
}

bool TB6612_MotorBoardGetSpeedLoopConfig(
    const TB6612MotorBoardContext *context,
    TB6612SpeedLoopConfig *config)
{
    if ((context == 0) || (config == 0) ||
        !context->speed_loop_enabled) {
        return false;
    }
    *config = context->speed_loop;
    return true;
}

bool TB6612_MotorBoardUpdateSpeedLoopTuning(
    TB6612MotorBoardContext *context,
    uint32_t kp_milli,
    uint32_t ki_milli,
    uint32_t kd_milli,
    uint8_t output_limit_percent)
{
    if ((context == 0) || !context->speed_loop_enabled ||
        (output_limit_percent == 0U) ||
        (output_limit_percent > TB6612_MAX_DUTY_PERCENT)) {
        return false;
    }

    context->speed_loop.kp_milli = kp_milli;
    context->speed_loop.ki_milli = ki_milli;
    context->speed_loop.kd_milli = kd_milli;
    context->speed_loop.output_limit_percent = output_limit_percent;
    /* Keep the current route target, but remove history from the old tuning.
     * Seed D history with the current error to avoid a one-frame derivative
     * kick when gains are changed while the task is running. */
    context->left_integral_milli = 0;
    context->right_integral_milli = 0;
    context->left_previous_error =
        context->target_left_rpm - context->measured_left_rpm;
    context->right_previous_error =
        context->target_right_rpm - context->measured_right_rpm;
    return true;
}

void TB6612_MotorBoardGetSpeedLoopStatus(
    const TB6612MotorBoardContext *context,
    TB6612SpeedLoopStatus *status)
{
    if (status == 0) {
        return;
    }
    *status = (TB6612SpeedLoopStatus){0};
    if (context == 0) {
        return;
    }
    status->enabled = context->speed_loop_enabled;
    status->target_left_rpm = context->target_left_rpm;
    status->target_right_rpm = context->target_right_rpm;
    status->measured_left_rpm = context->measured_left_rpm;
    status->measured_right_rpm = context->measured_right_rpm;
    status->left_delta_count = context->left_delta_count;
    status->right_delta_count = context->right_delta_count;
    status->left_output_percent = TB6612_GetLeftCommand();
    status->right_output_percent = TB6612_GetRightCommand();
    status->update_count = context->update_count;
    status->sample_elapsed_ms = context->last_sample_elapsed_ms;
}

void TB6612_MotorBoardService(TB6612MotorBoardContext *context)
{
    uint32_t now_ms;
    uint32_t elapsed_ms;

    if ((context == 0) || !context->speed_loop_enabled ||
        (context->now_ms == 0) ||
        ((context->target_left_rpm == 0) &&
         (context->target_right_rpm == 0))) {
        return;
    }

    now_ms = context->now_ms(context->now_context);
    if (!context->encoder_sample_ready) {
        /* A real elapsed interval is required before the first RPM sample. */
        context->previous_left_count = Encoder_GetLeftCount();
        context->previous_right_count = Encoder_GetRightCount();
        context->previous_control_ms = now_ms;
        context->encoder_sample_ready = true;
        return;
    }

    elapsed_ms = (uint32_t)(now_ms - context->previous_control_ms);
    if (elapsed_ms >= context->speed_loop.control_period_ms) {
        update_speed_loop(context, now_ms, elapsed_ms);
    }
}

bool TB6612_MotorBoard_SetWheelSpeeds(int16_t left,
                                      int16_t right,
                                      void *context)
{
    TB6612MotorBoardContext *adapter = (TB6612MotorBoardContext *)context;
    int16_t scale = (adapter == 0) ? 350 :
                    adapter->speed_units_at_max_duty;

    if ((adapter != 0) && adapter->speed_loop_enabled) {
        int32_t old_left_target = adapter->target_left_rpm;
        int32_t old_right_target = adapter->target_right_rpm;

        if ((left == 0) && (right == 0)) {
            reset_speed_loop_state(adapter);
            CarMotionWatchdog_Reset(&adapter->motion_watchdog);
            TB6612_Stop();
            return true;
        }
        adapter->target_left_rpm = speed_to_rpm(adapter, left);
        adapter->target_right_rpm = speed_to_rpm(adapter, right);
        if (((old_left_target < 0) != (adapter->target_left_rpm < 0)) ||
            ((old_left_target == 0) != (adapter->target_left_rpm == 0))) {
            adapter->left_integral_milli = 0;
            adapter->left_previous_error = 0;
        }
        if (((old_right_target < 0) != (adapter->target_right_rpm < 0)) ||
            ((old_right_target == 0) != (adapter->target_right_rpm == 0))) {
            adapter->right_integral_milli = 0;
            adapter->right_previous_error = 0;
        }

        TB6612_MotorBoardService(adapter);
        return true;
    }

    TB6612_SetMotors(units_to_percent(left, scale),
                     units_to_percent(right, scale));
    return true;
}

bool TB6612_MotorBoard_GetEncoder(int16_t *left,
                                  int16_t *right,
                                  uint32_t *timestamp_ms,
                                  void *context)
{
    TB6612MotorBoardContext *adapter = (TB6612MotorBoardContext *)context;
    CarMotionWatchdogResult watchdog_result = {true, true};
    int32_t left_count;
    int32_t right_count;
    uint32_t now_ms;

    if ((left == 0) || (right == 0) || (timestamp_ms == 0) ||
        (adapter == 0) || (adapter->now_ms == 0)) {
        return false;
    }
    left_count = Encoder_GetLeftCount();
    right_count = Encoder_GetRightCount();
    now_ms = adapter->now_ms(adapter->now_context);
    if (adapter->motion_watchdog_enabled) {
        watchdog_result = CarMotionWatchdog_Update(
            &adapter->motion_watchdog,
            adapter->target_left_rpm,
            adapter->target_right_rpm,
            left_count,
            right_count,
            now_ms);
        if (!watchdog_result.left_valid || !watchdog_result.right_valid) {
            reset_speed_loop_state(adapter);
            TB6612_Stop();
            return false;
        }
    }
    *left = (int16_t)left_count;
    *right = (int16_t)right_count;
    *timestamp_ms = now_ms;
    return true;
}
