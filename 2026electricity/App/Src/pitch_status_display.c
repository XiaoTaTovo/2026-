#include "pitch_status_display.h"

#include <stddef.h>
#include <string.h>

#define DISPLAY_RENDER_PERIOD_MS 250U
#define DISPLAY_RETRY_PERIOD_MS 1000U
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

static void append_position(char *line, size_t capacity, size_t *length,
                            int16_t position_0_1mm)
{
    uint16_t magnitude;
    char reversed[5];
    uint8_t count = 0U;

    if (position_0_1mm < 0)
    {
        append_text(line, capacity, length, "-");
        magnitude = (uint16_t)(-(int32_t)position_0_1mm);
    }
    else
    {
        magnitude = (uint16_t)position_0_1mm;
    }
    do
    {
        reversed[count++] = (char)('0' + ((magnitude / 10U) % 10U));
        magnitude = (uint16_t)(magnitude / 10U);
    } while ((magnitude != 0U) && (count < sizeof(reversed)));
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
    append_text(line, capacity, length, ".");
    if ((*length + 1U) < capacity)
    {
        uint16_t absolute = (uint16_t)(position_0_1mm < 0 ?
            -(int32_t)position_0_1mm : position_0_1mm);
        line[(*length)++] = (char)('0' + (absolute % 10U));
        line[*length] = '\0';
    }
}

static const char *task_state_text(PitchTaskState state)
{
    switch (state)
    {
        case PITCH_TASK_STATE_IDLE: return "IDLE";
        case PITCH_TASK_STATE_WAIT_CAPTURE: return "CAPTURE";
        case PITCH_TASK_STATE_STARTING: return "START";
        case PITCH_TASK_STATE_RUNNING_POSITIVE: return "TO POS";
        case PITCH_TASK_STATE_RUNNING_NEGATIVE: return "TO NEG";
        case PITCH_TASK_STATE_HOLDING: return "HOLD";
        case PITCH_TASK_STATE_FAULT: return "FAULT";
        default: return "UNKNOWN";
    }
}

static const char *vision_state_text(const PitchAxisVisionReport *report)
{
    if ((report == NULL) || !report->observation_present)
    {
        return "WAIT";
    }
    if ((report->state == PITCH_VISION_STATE_TRACKING) &&
        report->observation_fresh)
    {
        return "OK";
    }
    switch (report->state)
    {
        case PITCH_VISION_STATE_REJECT_LOW_CONFIDENCE: return "LOW";
        case PITCH_VISION_STATE_REJECT_STALE: return "OLD";
        case PITCH_VISION_STATE_REJECT_INVALID: return "BAD";
        default: return "WAIT";
    }
}

static void render(PitchStatusDisplay *display)
{
    PitchTaskControllerReport task_report;
    PitchAxisVisionReport vision_report;
    PitchAxisSelfTestState self_test_state;
    bool task_valid;
    bool vision_valid;
    char line[22];
    size_t length;

    memset(display->framebuffer, 0, sizeof(display->framebuffer));
    memset(&task_report, 0, sizeof(task_report));
    memset(&vision_report, 0, sizeof(vision_report));
    task_valid = PitchTaskController_GetReport(
        display->task_controller,
        &task_report);
    vision_valid = PitchAxisVisionControl_GetReport(
        display->vision,
        &vision_report);
    self_test_state = PitchAxisSelfTest_GetState(display->self_test);

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
    }
    else
    {
        append_text(line, sizeof(line), &length, "?");
    }
    show_string(display, 0U, line);

    length = 0U;
    line[0] = '\0';
    append_text(line, sizeof(line), &length, "MOTOR1000:");
    if (self_test_state == PITCH_AXIS_SELF_TEST_STATE_COMM_PASS)
    {
        append_text(line, sizeof(line), &length, "PASS");
    }
    else if (self_test_state == PITCH_AXIS_SELF_TEST_STATE_FAILED)
    {
        append_text(line, sizeof(line), &length, "FAIL");
    }
    else
    {
        append_text(line, sizeof(line), &length, "RUN");
    }
    show_string(display, 1U, line);

    length = 0U;
    line[0] = '\0';
    append_text(line, sizeof(line), &length, "X:");
    if (vision_valid && vision_report.observation_present)
    {
        append_position(line, sizeof(line), &length,
                        vision_report.observation.x_0_1mm);
    }
    else
    {
        append_text(line, sizeof(line), &length, "---");
    }
    append_text(line, sizeof(line), &length, " A:");
    append_text(line, sizeof(line), &length,
                task_valid && task_report.automatic_armed ? "ON" : "OFF");
    append_text(line, sizeof(line), &length, " V:");
    append_text(line, sizeof(line), &length,
                vision_valid ? vision_state_text(&vision_report) : "WAIT");
    show_string(display, 2U, line);

    length = 0U;
    line[0] = '\0';
    append_text(line, sizeof(line), &length, "T6 FIXED:");
    if (task_valid && task_report.captured_position_valid)
    {
        append_text(line, sizeof(line), &length, "YES ");
        append_position(line, sizeof(line), &length,
                        task_report.captured_position_0_1mm);
    }
    else
    {
        append_text(line, sizeof(line), &length, "NO");
    }
    show_string(display, 3U, line);
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
    display->initialized = false;
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
    uint32_t now_ms)
{
    if ((display == NULL) || (i2c == NULL) ||
        (task_controller == NULL) || (self_test == NULL) ||
        (vision == NULL) || (address_7bit < 0x08U) ||
        (address_7bit > 0x77U))
    {
        return false;
    }
    memset(display, 0, sizeof(*display));
    display->i2c = i2c;
    display->task_controller = task_controller;
    display->self_test = self_test;
    display->vision = vision;
    display->address_7bit = address_7bit;
    display->last_render_ms = now_ms - DISPLAY_RENDER_PERIOD_MS;
    display->state = PITCH_STATUS_DISPLAY_STATE_START_INIT;
    return true;
}

void PitchStatusDisplay_Service(
    PitchStatusDisplay *display,
    uint32_t now_ms)
{
    static const uint8_t init_commands[] = {
        0xAEU, 0xD5U, 0x80U, 0xA8U, 0x3FU, 0xD3U, 0x00U, 0x40U,
        0xA1U, 0xC8U, 0xDAU, 0x12U, 0x81U, 0xCFU, 0xD9U, 0xF1U,
        0xDBU, 0x30U, 0xA4U, 0xA6U, 0x8DU, 0x14U, 0x20U, 0x02U,
        0xAFU
    };

    if ((display == NULL) ||
        (display->state == PITCH_STATUS_DISPLAY_STATE_UNINITIALIZED))
    {
        return;
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
            display->initialized = true;
            render(display);
            display->last_render_ms = now_ms;
            display->tx_page = 0U;
            display->state = PITCH_STATUS_DISPLAY_STATE_START_CURSOR;
            break;

        case PITCH_STATUS_DISPLAY_STATE_READY:
            if (elapsed(now_ms, display->last_render_ms,
                        DISPLAY_RENDER_PERIOD_MS))
            {
                render(display);
                display->last_render_ms = now_ms;
                display->tx_page = 0U;
                display->state = PITCH_STATUS_DISPLAY_STATE_START_CURSOR;
            }
            break;

        case PITCH_STATUS_DISPLAY_STATE_START_CURSOR:
            if (display->tx_page >= PITCH_STATUS_DISPLAY_PAGE_COUNT)
            {
                display->state = PITCH_STATUS_DISPLAY_STATE_READY;
                return;
            }
            if (HAL_I2C_GetState(display->i2c) != HAL_I2C_STATE_READY)
            {
                return;
            }
            display->tx_buffer[0] = 0x00U;
            display->tx_buffer[1] = (uint8_t)(0xB0U | display->tx_page);
            display->tx_buffer[2] = 0x00U;
            display->tx_buffer[3] = 0x10U;
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
