#include "pitch_status_display.h"

#include <stddef.h>
#include <string.h>

#define DISPLAY_RENDER_PERIOD_MS 250U
#define DISPLAY_RETRY_PERIOD_MS 1000U
#define DISPLAY_COLUMN_OFFSET 2U
#define DISPLAY_FONT_WIDTH 5U
#define DISPLAY_FONT_STRIDE 6U

typedef struct
{
    char character;
    uint8_t columns[DISPLAY_FONT_WIDTH];
} DisplayGlyph;

static const DisplayGlyph display_font[] = {
    {' ', {0x00U, 0x00U, 0x00U, 0x00U, 0x00U}},
    {'-', {0x08U, 0x08U, 0x08U, 0x08U, 0x08U}},
    {'.', {0x00U, 0x60U, 0x60U, 0x00U, 0x00U}},
    {':', {0x00U, 0x36U, 0x36U, 0x00U, 0x00U}},
    {'?', {0x02U, 0x01U, 0x51U, 0x09U, 0x06U}},
    {'0', {0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU}},
    {'1', {0x00U, 0x42U, 0x7FU, 0x40U, 0x00U}},
    {'2', {0x42U, 0x61U, 0x51U, 0x49U, 0x46U}},
    {'3', {0x21U, 0x41U, 0x45U, 0x4BU, 0x31U}},
    {'4', {0x18U, 0x14U, 0x12U, 0x7FU, 0x10U}},
    {'5', {0x27U, 0x45U, 0x45U, 0x45U, 0x39U}},
    {'6', {0x3CU, 0x4AU, 0x49U, 0x49U, 0x30U}},
    {'7', {0x01U, 0x71U, 0x09U, 0x05U, 0x03U}},
    {'8', {0x36U, 0x49U, 0x49U, 0x49U, 0x36U}},
    {'9', {0x06U, 0x49U, 0x49U, 0x29U, 0x1EU}},
    {'A', {0x7EU, 0x11U, 0x11U, 0x11U, 0x7EU}},
    {'B', {0x7FU, 0x49U, 0x49U, 0x49U, 0x36U}},
    {'C', {0x3EU, 0x41U, 0x41U, 0x41U, 0x22U}},
    {'D', {0x7FU, 0x41U, 0x41U, 0x22U, 0x1CU}},
    {'E', {0x7FU, 0x49U, 0x49U, 0x49U, 0x41U}},
    {'F', {0x7FU, 0x09U, 0x09U, 0x09U, 0x01U}},
    {'G', {0x3EU, 0x41U, 0x49U, 0x49U, 0x7AU}},
    {'H', {0x7FU, 0x08U, 0x08U, 0x08U, 0x7FU}},
    {'I', {0x00U, 0x41U, 0x7FU, 0x41U, 0x00U}},
    {'J', {0x20U, 0x40U, 0x41U, 0x3FU, 0x01U}},
    {'K', {0x7FU, 0x08U, 0x14U, 0x22U, 0x41U}},
    {'L', {0x7FU, 0x40U, 0x40U, 0x40U, 0x40U}},
    {'M', {0x7FU, 0x02U, 0x0CU, 0x02U, 0x7FU}},
    {'N', {0x7FU, 0x04U, 0x08U, 0x10U, 0x7FU}},
    {'O', {0x3EU, 0x41U, 0x41U, 0x41U, 0x3EU}},
    {'P', {0x7FU, 0x09U, 0x09U, 0x09U, 0x06U}},
    {'Q', {0x3EU, 0x41U, 0x51U, 0x21U, 0x5EU}},
    {'R', {0x7FU, 0x09U, 0x19U, 0x29U, 0x46U}},
    {'S', {0x46U, 0x49U, 0x49U, 0x49U, 0x31U}},
    {'T', {0x01U, 0x01U, 0x7FU, 0x01U, 0x01U}},
    {'U', {0x3FU, 0x40U, 0x40U, 0x40U, 0x3FU}},
    {'V', {0x1FU, 0x20U, 0x40U, 0x20U, 0x1FU}},
    {'W', {0x3FU, 0x40U, 0x38U, 0x40U, 0x3FU}},
    {'X', {0x63U, 0x14U, 0x08U, 0x14U, 0x63U}},
    {'Y', {0x07U, 0x08U, 0x70U, 0x08U, 0x07U}},
    {'Z', {0x61U, 0x51U, 0x49U, 0x45U, 0x43U}}
};

static bool elapsed(uint32_t now_ms, uint32_t since_ms, uint32_t period_ms)
{
    return (uint32_t)(now_ms - since_ms) >= period_ms;
}

static const uint8_t *find_glyph(char character)
{
    size_t index;

    for (index = 0U;
         index < (sizeof(display_font) / sizeof(display_font[0]));
         ++index)
    {
        if (display_font[index].character == character)
        {
            return display_font[index].columns;
        }
    }
    return find_glyph('?');
}

static void show_string(
    PitchStatusDisplay *display,
    uint8_t page,
    const char *text)
{
    uint8_t x = 0U;

    while ((*text != '\0') &&
           ((uint16_t)x + DISPLAY_FONT_WIDTH <=
            PITCH_STATUS_DISPLAY_WIDTH))
    {
        const uint8_t *glyph = find_glyph(*text);
        uint8_t column;

        for (column = 0U; column < DISPLAY_FONT_WIDTH; ++column)
        {
            display->framebuffer[page][x + column] = glyph[column];
        }
        if ((uint16_t)x + DISPLAY_FONT_WIDTH <
            PITCH_STATUS_DISPLAY_WIDTH)
        {
            display->framebuffer[page][x + DISPLAY_FONT_WIDTH] = 0U;
        }
        if ((PITCH_STATUS_DISPLAY_WIDTH - x) < DISPLAY_FONT_STRIDE)
        {
            break;
        }
        x = (uint8_t)(x + DISPLAY_FONT_STRIDE);
        ++text;
    }
}

static void append_text(char *line, size_t capacity, size_t *length,
                        const char *text)
{
    while ((*text != '\0') && ((*length + 1U) < capacity))
    {
        line[*length] = *text;
        ++(*length);
        ++text;
    }
    line[*length] = '\0';
}

static void append_u8(char *line, size_t capacity, size_t *length,
                      uint8_t value)
{
    if (value >= 10U)
    {
        if ((*length + 1U) < capacity)
        {
            line[(*length)++] = (char)('0' + ((value / 10U) % 10U));
        }
    }
    if ((*length + 1U) < capacity)
    {
        line[(*length)++] = (char)('0' + (value % 10U));
    }
    line[*length] = '\0';
}

static void append_u32(char *line, size_t capacity, size_t *length,
                       uint32_t value)
{
    char reversed[10];
    uint8_t count = 0U;

    do
    {
        reversed[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while ((value != 0U) && (count < sizeof(reversed)));
    while (count > 0U)
    {
        if ((*length + 1U) < capacity)
        {
            line[(*length)++] = reversed[--count];
        }
        else
        {
            break;
        }
    }
    line[*length] = '\0';
}

static void append_pid_value(char *line, size_t capacity, size_t *length,
                             float value)
{
    int32_t milli = (int32_t)(value * 1000.0f +
        ((value >= 0.0f) ? 0.5f : -0.5f));
    uint32_t magnitude;
    uint32_t integer;
    uint32_t fraction;

    if (milli < 0)
    {
        append_text(line, capacity, length, "-");
        magnitude = (uint32_t)(-(milli + 1)) + 1U;
    }
    else
    {
        magnitude = (uint32_t)milli;
    }
    integer = magnitude / 1000U;
    fraction = magnitude % 1000U;
    if (integer != 0U)
    {
        append_u32(line, capacity, length, integer);
    }
    append_text(line, capacity, length, ".");
    if ((*length + 3U) < capacity)
    {
        line[(*length)++] = (char)('0' + ((fraction / 100U) % 10U));
        line[(*length)++] = (char)('0' + ((fraction / 10U) % 10U));
        line[(*length)++] = (char)('0' + (fraction % 10U));
        line[*length] = '\0';
    }
}

static const char *task_state_text(PitchTaskState state)
{
    switch (state)
    {
        case PITCH_TASK_STATE_IDLE: return "IDLE";
        case PITCH_TASK_STATE_WAIT_CAPTURE: return "CAP";
        case PITCH_TASK_STATE_STARTING: return "START";
        case PITCH_TASK_STATE_RUNNING_POSITIVE: return "POS";
        case PITCH_TASK_STATE_RUNNING_NEGATIVE: return "NEG";
        case PITCH_TASK_STATE_HOLDING: return "HOLD";
        case PITCH_TASK_STATE_FAULT: return "FAULT";
        default: return "?";
    }
}

static void render(PitchStatusDisplay *display)
{
    PitchTaskControllerReport task_report;
    PitchAxisVisionConfig vision_config;
    PitchAxisVelocityTestConfig velocity_config;
    bool task_valid;
    bool vision_config_valid;
    bool velocity_config_valid;
    char line[22];
    size_t length;

    memset(display->framebuffer, 0, sizeof(display->framebuffer));
    memset(&task_report, 0, sizeof(task_report));
    task_valid = PitchTaskController_GetReport(
        display->task_controller,
        &task_report);
    vision_config_valid = PitchAxisVisionControl_GetConfig(
        display->vision,
        &vision_config);
    velocity_config_valid = PitchAxisVelocityTest_GetConfig(
        display->velocity,
        &velocity_config);

    length = 0U;
    line[0] = '\0';
    append_text(line, sizeof(line), &length, "TASK:");
    if (task_valid)
    {
        append_u8(line, sizeof(line), &length,
                  (uint8_t)task_report.selected_task);
        append_text(line, sizeof(line), &length, " ");
        append_text(line, sizeof(line), &length,
                    task_state_text(task_report.state));
        append_text(line, sizeof(line), &length, " G");
        append_u8(line, sizeof(line), &length, task_report.pid_profile);
    }
    else
    {
        append_text(line, sizeof(line), &length, "?");
    }
    show_string(display, 0U, line);

    length = 0U;
    line[0] = '\0';
    append_text(line, sizeof(line), &length, "KEY:");
    append_text(line, sizeof(line), &length,
                display->buttons.key1_pressed ? "0" : "1");
    append_text(line, sizeof(line), &length,
                display->buttons.key2_pressed ? "0" : "1");
    append_text(line, sizeof(line), &length,
                display->buttons.key3_pressed ? "0" : "1");
    append_text(line, sizeof(line), &length,
                display->buttons.key4_pressed ? "0" : "1");
    show_string(display, 2U, line);

    length = 0U;
    line[0] = '\0';
    append_text(line, sizeof(line), &length, "P:");
    if (vision_config_valid)
    {
        append_pid_value(line, sizeof(line), &length,
                         vision_config.kp_rpm_per_mm);
        append_text(line, sizeof(line), &length, " I:");
        append_pid_value(line, sizeof(line), &length,
                         vision_config.ki_rpm_per_mm_s);
        append_text(line, sizeof(line), &length, " D:");
        append_pid_value(line, sizeof(line), &length,
                         vision_config.kd_rpm_per_mm_s);
    }
    else
    {
        append_text(line, sizeof(line), &length, "? I:? D:?");
    }
    show_string(display, 4U, line);

    length = 0U;
    line[0] = '\0';
    append_text(line, sizeof(line), &length, "S:");
    if (velocity_config_valid)
    {
        append_u32(line, sizeof(line), &length,
                   velocity_config.automatic_tilt_scale_um_per_outer_rpm);
        append_text(line, sizeof(line), &length, " L:");
        append_u32(line, sizeof(line), &length,
                   velocity_config.automatic_tilt_limit_um);
    }
    else
    {
        append_text(line, sizeof(line), &length, "? L:?");
    }
    show_string(display, 6U, line);
}

static bool buttons_equal(PitchAxisVelocityTestButtons left,
                          PitchAxisVelocityTestButtons right)
{
    return (left.key1_pressed == right.key1_pressed) &&
        (left.key2_pressed == right.key2_pressed) &&
        (left.key3_pressed == right.key3_pressed) &&
        (left.key4_pressed == right.key4_pressed);
}

static bool transfer_finished(const PitchStatusDisplay *display)
{
    return HAL_I2C_GetState(display->i2c) == HAL_I2C_STATE_READY;
}

static bool transfer_ok(const PitchStatusDisplay *display)
{
    return transfer_finished(display) &&
           (HAL_I2C_GetError(display->i2c) == HAL_I2C_ERROR_NONE);
}

static bool start_transfer(PitchStatusDisplay *display, uint16_t length)
{
    return HAL_I2C_Master_Transmit_IT(
               display->i2c,
               (uint16_t)display->address_7bit << 1U,
               display->tx_buffer,
               length) == HAL_OK;
}

static void enter_retry(PitchStatusDisplay *display, uint32_t now_ms)
{
    if ((HAL_I2C_GetError(display->i2c) & HAL_I2C_ERROR_AF) != 0U)
    {
        display->address_7bit =
            (display->address_7bit == 0x3CU) ? 0x3DU : 0x3CU;
    }
    display->initialized = false;
    display->first_frame_written = false;
    display->retry_since_ms = now_ms;
    display->error_count++;
    display->state = PITCH_STATUS_DISPLAY_STATE_RETRY_WAIT;
}

bool PitchStatusDisplay_Init(
    PitchStatusDisplay *display,
    I2C_HandleTypeDef *i2c,
    uint8_t address_7bit,
    PitchTaskController *task_controller,
    PitchAxisSelfTest *self_test,
    PitchAxisVisionControl *vision,
    PitchAxisVelocityTest *velocity,
    uint32_t now_ms)
{
    if ((display == NULL) || (i2c == NULL) ||
        (task_controller == NULL) || (self_test == NULL) ||
        (vision == NULL) || (velocity == NULL) ||
        (address_7bit < 0x08U) ||
        (address_7bit > 0x77U))
    {
        return false;
    }
    memset(display, 0, sizeof(*display));
    display->i2c = i2c;
    display->task_controller = task_controller;
    display->self_test = self_test;
    display->vision = vision;
    display->velocity = velocity;
    display->address_7bit = address_7bit;
    display->last_render_ms = now_ms - DISPLAY_RENDER_PERIOD_MS;
    display->retry_since_ms = now_ms;
    display->render_pending = true;
    display->state = PITCH_STATUS_DISPLAY_STATE_RETRY_WAIT;
    return true;
}

void PitchStatusDisplay_Service(
    PitchStatusDisplay *display,
    PitchAxisVelocityTestButtons buttons,
    uint32_t now_ms)
{
    static const uint8_t init_commands[] = {
        0xAEU, 0xD5U, 0x80U, 0xA8U, 0x3FU, 0xD3U, 0x00U, 0x40U,
        0xA1U, 0xC8U, 0xDAU, 0x12U, 0x81U, 0xCFU, 0xD9U, 0xF1U,
        0xDBU, 0x30U, 0xA4U, 0xA6U, 0x2EU, 0x20U, 0x02U, 0x8DU, 0x14U
    };

    if ((display == NULL) ||
        (display->state == PITCH_STATUS_DISPLAY_STATE_UNINITIALIZED))
    {
        return;
    }
    if (!buttons_equal(display->buttons, buttons))
    {
        display->buttons = buttons;
        display->render_pending = true;
    }

    switch (display->state)
    {
        case PITCH_STATUS_DISPLAY_STATE_START_INIT:
            if (HAL_I2C_GetState(display->i2c) != HAL_I2C_STATE_READY)
            {
                return;
            }
            display->tx_buffer[0] = 0x00U;
            memcpy(&display->tx_buffer[1], init_commands,
                   sizeof(init_commands));
            if (start_transfer(
                    display,
                    (uint16_t)(sizeof(init_commands) + 1U)))
            {
                display->state = PITCH_STATUS_DISPLAY_STATE_WAIT_INIT;
            }
            break;

        case PITCH_STATUS_DISPLAY_STATE_WAIT_INIT:
            if (!transfer_finished(display))
            {
                return;
            }
            if (!transfer_ok(display))
            {
                enter_retry(display, now_ms);
                return;
            }
            render(display);
            display->last_render_ms = now_ms;
            display->render_pending = false;
            display->tx_page = 0U;
            display->state = PITCH_STATUS_DISPLAY_STATE_START_CURSOR;
            break;

        case PITCH_STATUS_DISPLAY_STATE_READY:
            if (display->render_pending ||
                elapsed(now_ms, display->last_render_ms,
                        DISPLAY_RENDER_PERIOD_MS))
            {
                render(display);
                display->last_render_ms = now_ms;
                display->render_pending = false;
                display->tx_page = 0U;
                display->state = PITCH_STATUS_DISPLAY_STATE_START_CURSOR;
            }
            break;

        case PITCH_STATUS_DISPLAY_STATE_START_CURSOR:
            if (display->tx_page >= PITCH_STATUS_DISPLAY_PAGE_COUNT)
            {
                if (!display->first_frame_written)
                {
                    display->first_frame_written = true;
                    display->state =
                        PITCH_STATUS_DISPLAY_STATE_START_DISPLAY_ON;
                }
                else
                {
                    display->state = PITCH_STATUS_DISPLAY_STATE_READY;
                }
                return;
            }
            if (HAL_I2C_GetState(display->i2c) != HAL_I2C_STATE_READY)
            {
                return;
            }
            display->tx_buffer[0] = 0x00U;
            display->tx_buffer[1] = (uint8_t)(0xB0U | display->tx_page);
            display->tx_buffer[2] =
                (uint8_t)(DISPLAY_COLUMN_OFFSET & 0x0FU);
            display->tx_buffer[3] =
                (uint8_t)(0x10U | (DISPLAY_COLUMN_OFFSET >> 4U));
            if (start_transfer(display, 4U))
            {
                display->state = PITCH_STATUS_DISPLAY_STATE_WAIT_CURSOR;
            }
            break;

        case PITCH_STATUS_DISPLAY_STATE_WAIT_CURSOR:
            if (!transfer_finished(display))
            {
                return;
            }
            if (!transfer_ok(display))
            {
                enter_retry(display, now_ms);
                return;
            }
            display->tx_buffer[0] = 0x40U;
            memcpy(&display->tx_buffer[1],
                   display->framebuffer[display->tx_page],
                   PITCH_STATUS_DISPLAY_WIDTH);
            if (start_transfer(
                    display,
                    PITCH_STATUS_DISPLAY_WIDTH + 1U))
            {
                display->state = PITCH_STATUS_DISPLAY_STATE_WAIT_PAGE;
            }
            break;

        case PITCH_STATUS_DISPLAY_STATE_WAIT_PAGE:
            if (!transfer_finished(display))
            {
                return;
            }
            if (!transfer_ok(display))
            {
                enter_retry(display, now_ms);
                return;
            }
            display->tx_page++;
            display->state = PITCH_STATUS_DISPLAY_STATE_START_CURSOR;
            break;

        case PITCH_STATUS_DISPLAY_STATE_START_DISPLAY_ON:
            if (HAL_I2C_GetState(display->i2c) != HAL_I2C_STATE_READY)
            {
                return;
            }
            display->tx_buffer[0] = 0x00U;
            display->tx_buffer[1] = 0xAFU;
            if (start_transfer(display, 2U))
            {
                display->state =
                    PITCH_STATUS_DISPLAY_STATE_WAIT_DISPLAY_ON;
            }
            break;

        case PITCH_STATUS_DISPLAY_STATE_WAIT_DISPLAY_ON:
            if (!transfer_finished(display))
            {
                return;
            }
            if (!transfer_ok(display))
            {
                enter_retry(display, now_ms);
                return;
            }
            display->initialized = true;
            display->state = PITCH_STATUS_DISPLAY_STATE_READY;
            break;

        case PITCH_STATUS_DISPLAY_STATE_RETRY_WAIT:
            if (elapsed(now_ms, display->retry_since_ms,
                        DISPLAY_RETRY_PERIOD_MS))
            {
                display->state = PITCH_STATUS_DISPLAY_STATE_START_INIT;
            }
            break;

        default:
            break;
    }
}
