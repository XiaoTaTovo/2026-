#include "app/gray_adc_debug.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "OLED.h"
#include "drivers/button.h"
#include "drivers/gray_array.h"
#include "platform/ti_mspm0_platform.h"
#include "ti_msp_dl_config.h"

#define GRAY_CALIBRATION_FRAMES (16U)
#define GRAY_WHITE_THRESHOLD    (350U)
#define GRAY_BLACK_THRESHOLD    (650U)
#define GRAY_BUTTON_POLL_MS      (10U)
#define GRAY_DISPLAY_PERIOD_MS   (100U)

static GrayArray gGrayDebugArray;
static OLED_Status gGrayDebugOledStatus = OLED_STATUS_ERROR_NOT_INITIALIZED;
static uint32_t gGrayDebugFrame;

typedef enum {
    GRAY_CAL_WHITE_WAIT = 0,
    GRAY_CAL_BLACK_WAIT,
    GRAY_CAL_RUNNING,
    GRAY_CAL_ERROR,
} GrayCalibrationState;

typedef enum {
    GRAY_CAL_ERROR_NONE = 0,
    GRAY_CAL_ERROR_ADC,
    GRAY_CAL_ERROR_SPAN,
} GrayCalibrationError;

typedef enum {
    GRAY_DISPLAY_NORMALIZED = 0,
    GRAY_DISPLAY_WHITE_CAL,
    GRAY_DISPLAY_BLACK_CAL,
    GRAY_DISPLAY_RAW,
} GrayDebugDisplayPage;

static void GrayAdcDebug_DelayMs(uint32_t ms)
{
    while (ms > 0U) {
        delay_cycles(CPUCLK_FREQ / 1000U);
        ms--;
    }
}

static bool ReadKey1(void *context)
{
    (void)context;
    return TiMspm0Platform_ReadKey1Level();
}

static bool ReadKey2(void *context)
{
    (void)context;
    return TiMspm0Platform_ReadKey2Level();
}

static bool ReadKey3(void *context)
{
    (void)context;
    return TiMspm0Platform_ReadKey3Level();
}

static OLED_Status DisplayCalibrationMessage(const char *message,
                                             const char *detail,
                                             const char *extra)
{
    char line[22];
    OLED_Status status = OLED_Clear();

    if (status != OLED_STATUS_OK) {
        return status;
    }
    status = OLED_ShowString(0U, 1U, message);
    if (status != OLED_STATUS_OK) {
        return status;
    }
    status = OLED_ShowString(0U, 3U, detail);
    if (status != OLED_STATUS_OK) {
        return status;
    }
    if (extra != 0) {
        status = OLED_ShowString(0U, 5U, extra);
        if (status != OLED_STATUS_OK) {
            return status;
        }
    }
    (void)snprintf(line, sizeof(line), "KEYH:%u%u%u",
                   TiMspm0Platform_ReadKey1Level() ? 1U : 0U,
                   TiMspm0Platform_ReadKey2Level() ? 1U : 0U,
                   TiMspm0Platform_ReadKey3Level() ? 1U : 0U);
    status = OLED_ShowString(0U, 7U, line);
    if (status != OLED_STATUS_OK) {
        return status;
    }
    return OLED_Update();
}

static char GrayClassificationChar(
    const GrayArrayClassification *classification, uint8_t channel)
{
    uint8_t bit;

    if ((classification == 0) || (channel >= GRAY_ARRAY_CHANNELS)) {
        return '?';
    }
    bit = (uint8_t)(1U << channel);
    if ((classification->black_mask & bit) != 0U) {
        return 'B';
    }
    if ((classification->white_mask & bit) != 0U) {
        return 'W';
    }
    return '?';
}

static OLED_Status DisplayGrayValues(
    const GrayArray *gray,
    bool adc_ok,
    uint32_t frame,
    const GrayArrayClassification *classification)
{
    char line[22];
    OLED_Status status = OLED_Clear();

    if (status != OLED_STATUS_OK) {
        return status;
    }
    status = OLED_ShowString(0U, 0U, "GRAY ADC NORM");
    if (status != OLED_STATUS_OK) {
        return status;
    }
    for (uint8_t row = 0U; row < 4U; row++) {
        uint8_t first = (uint8_t)(row * 2U);

        (void)snprintf(line, sizeof(line), "%u:%4u %u:%4u",
                       (uint8_t)(first + 1U), gray->latest.normalized[first],
                       (uint8_t)(first + 2U),
                       gray->latest.normalized[first + 1U]);
        status = OLED_ShowString(0U, (uint8_t)(row + 1U), line);
        if (status != OLED_STATUS_OK) {
            return status;
        }
    }
    (void)snprintf(line, sizeof(line),
                   adc_ok ? "ADC OK F:%lu" : "ADC ERR F:%lu",
                   (unsigned long)frame);
    status = OLED_ShowString(0U, 5U, line);
    if (status != OLED_STATUS_OK) {
        return status;
    }
    (void)snprintf(line, sizeof(line), "1:%c 2:%c 3:%c 4:%c",
                   GrayClassificationChar(classification, 0U),
                   GrayClassificationChar(classification, 1U),
                   GrayClassificationChar(classification, 2U),
                   GrayClassificationChar(classification, 3U));
    status = OLED_ShowString(0U, 6U, line);
    if (status != OLED_STATUS_OK) {
        return status;
    }
    (void)snprintf(line, sizeof(line), "5:%c 6:%c 7:%c 8:%c",
                   GrayClassificationChar(classification, 4U),
                   GrayClassificationChar(classification, 5U),
                   GrayClassificationChar(classification, 6U),
                   GrayClassificationChar(classification, 7U));
    status = OLED_ShowString(0U, 7U, line);
    if (status != OLED_STATUS_OK) {
        return status;
    }
    return OLED_Update();
}

static OLED_Status DisplayCalibrationValues(
    const uint16_t values[GRAY_ARRAY_CHANNELS],
    const char *title)
{
    char line[22];
    OLED_Status status = OLED_Clear();

    if (status != OLED_STATUS_OK) {
        return status;
    }
    status = OLED_ShowString(0U, 0U, title);
    if (status != OLED_STATUS_OK) {
        return status;
    }
    for (uint8_t row = 0U; row < 4U; row++) {
        uint8_t first = (uint8_t)(row * 2U);

        (void)snprintf(line, sizeof(line), "%u:%4u %u:%4u",
                       (uint8_t)(first + 1U), values[first],
                       (uint8_t)(first + 2U), values[first + 1U]);
        status = OLED_ShowString(0U, (uint8_t)(row + 1U), line);
        if (status != OLED_STATUS_OK) {
            return status;
        }
    }
    status = OLED_ShowString(0U, 5U, "MID=W RIGHT=B");
    if (status != OLED_STATUS_OK) {
        return status;
    }
    status = OLED_ShowString(0U, 6U, "LEFT=NORM");
    if (status != OLED_STATUS_OK) {
        return status;
    }
    (void)snprintf(line, sizeof(line), "KEYH:%u%u%u",
                   TiMspm0Platform_ReadKey1Level() ? 1U : 0U,
                   TiMspm0Platform_ReadKey2Level() ? 1U : 0U,
                   TiMspm0Platform_ReadKey3Level() ? 1U : 0U);
    status = OLED_ShowString(0U, 7U, line);
    if (status != OLED_STATUS_OK) {
        return status;
    }
    return OLED_Update();
}

static OLED_Status DisplayRawValues(const GrayArray *gray)
{
    char line[22];
    OLED_Status status = OLED_Clear();

    if (status != OLED_STATUS_OK) {
        return status;
    }
    status = OLED_ShowString(0U, 0U, "GRAY ADC RAW");
    if (status != OLED_STATUS_OK) {
        return status;
    }
    for (uint8_t row = 0U; row < 4U; row++) {
        uint8_t first = (uint8_t)(row * 2U);

        (void)snprintf(line, sizeof(line), "%u:%4u %u:%4u",
                       (uint8_t)(first + 1U), gray->raw[first],
                       (uint8_t)(first + 2U), gray->raw[first + 1U]);
        status = OLED_ShowString(0U, (uint8_t)(row + 1U), line);
        if (status != OLED_STATUS_OK) {
            return status;
        }
    }
    status = OLED_ShowString(0U, 5U, "LEFT=NORM");
    if (status != OLED_STATUS_OK) {
        return status;
    }
    status = OLED_ShowString(0U, 6U, "W~3000 B~111");
    if (status != OLED_STATUS_OK) {
        return status;
    }
    (void)snprintf(line, sizeof(line), "KEYH:%u%u%u",
                   TiMspm0Platform_ReadKey1Level() ? 1U : 0U,
                   TiMspm0Platform_ReadKey2Level() ? 1U : 0U,
                   TiMspm0Platform_ReadKey3Level() ? 1U : 0U);
    status = OLED_ShowString(0U, 7U, line);
    if (status != OLED_STATUS_OK) {
        return status;
    }
    return OLED_Update();
}

static bool CaptureCalibrationSurface(
    GrayArray *gray,
    uint16_t output[GRAY_ARRAY_CHANNELS],
    const char *label)
{
    uint32_t sums[GRAY_ARRAY_CHANNELS] = {0U};

    for (uint8_t sample = 0U; sample < GRAY_CALIBRATION_FRAMES; sample++) {
        if (!GrayArray_Read(gray, TiMspm0Platform_Millis())) {
            return false;
        }
        for (uint8_t channel = 0U; channel < GRAY_ARRAY_CHANNELS; channel++) {
            sums[channel] += gray->raw[channel];
        }
        (void)DisplayCalibrationMessage(label, "SAMPLING...", 0);
        GrayAdcDebug_DelayMs(10U);
    }
    for (uint8_t channel = 0U; channel < GRAY_ARRAY_CHANNELS; channel++) {
        output[channel] = (uint16_t)(sums[channel] / GRAY_CALIBRATION_FRAMES);
    }
    return true;
}

void GrayAdcDebug_Run(void)
{
    CarFirmwareConfig config;
    OLED_Config oled_config;
    Button key1;
    Button key2;
    Button key3;
    GrayCalibrationState state = GRAY_CAL_WHITE_WAIT;
    GrayCalibrationError calibration_error = GRAY_CAL_ERROR_NONE;
    uint16_t white[GRAY_ARRAY_CHANNELS] = {0U};
    uint16_t black[GRAY_ARRAY_CHANNELS] = {0U};
    uint8_t bad_channel = 0U;
    int32_t bad_span = 0;
    uint32_t last_display_ms = 0U;
    bool force_display = true;
    GrayDebugDisplayPage display_page = GRAY_DISPLAY_NORMALIZED;

    TiMspm0Platform_Init();
    if (TiMspm0Platform_BuildConfig(&config, H2024_MODE_ITEM_1) != CAR_OK) {
        while (1) {
            __WFI();
        }
    }
    GrayArray_Init(&gGrayDebugArray, &config.gray);
    Button_Init(&key1, ReadKey1, 0, true, 30U);
    Button_Init(&key2, ReadKey2, 0, true, 30U);
    Button_Init(&key3, ReadKey3, 0, true, 30U);

    oled_config = OLED_MakeSSD1306Config(OLED_DEFAULT_ADDR_7BIT);
    gGrayDebugOledStatus = OLED_Init(&oled_config);
    if (gGrayDebugOledStatus == OLED_STATUS_ERROR_I2C_ADDRESS_NACK) {
        oled_config = OLED_MakeSSD1306Config(0x3DU);
        gGrayDebugOledStatus = OLED_Init(&oled_config);
    }

    while (1) {
        uint32_t now_ms = TiMspm0Platform_Millis();
        bool adc_ok = GrayArray_Read(
            &gGrayDebugArray, now_ms);
        GrayArrayClassification classification = {0U};
        bool classification_ok = GrayArray_ClassifyLatest(
            &gGrayDebugArray, GRAY_WHITE_THRESHOLD, GRAY_BLACK_THRESHOLD,
            &classification);
        bool key1_pressed;
        bool key2_pressed;
        bool key3_pressed;

        gGrayDebugFrame++;
        Button_Update(&key1, now_ms);
        Button_Update(&key2, now_ms);
        Button_Update(&key3, now_ms);
        key1_pressed = Button_TakePressedEvent(&key1);
        key2_pressed = Button_TakePressedEvent(&key2);
        key3_pressed = Button_TakePressedEvent(&key3);

        if ((state == GRAY_CAL_WHITE_WAIT) && key1_pressed) {
            if (CaptureCalibrationSurface(
                    &gGrayDebugArray, white, "WHITE")) {
                state = GRAY_CAL_BLACK_WAIT;
                calibration_error = GRAY_CAL_ERROR_NONE;
            } else {
                state = GRAY_CAL_ERROR;
                calibration_error = GRAY_CAL_ERROR_ADC;
            }
            force_display = true;
        } else if ((state == GRAY_CAL_BLACK_WAIT) && key2_pressed) {
            if (!CaptureCalibrationSurface(
                    &gGrayDebugArray, black, "BLACK")) {
                state = GRAY_CAL_ERROR;
                calibration_error = GRAY_CAL_ERROR_ADC;
            } else if (GrayArray_SetCalibration(
                           &gGrayDebugArray, black, white)) {
                state = GRAY_CAL_RUNNING;
                calibration_error = GRAY_CAL_ERROR_NONE;
                display_page = GRAY_DISPLAY_NORMALIZED;
            } else {
                state = GRAY_CAL_ERROR;
                calibration_error = GRAY_CAL_ERROR_SPAN;
                for (uint8_t channel = 0U;
                     channel < GRAY_ARRAY_CHANNELS; channel++) {
                    int32_t span =
                        (int32_t)white[channel] - (int32_t)black[channel];
                    if ((span > -20) && (span < 20)) {
                        bad_channel = channel;
                        bad_span = span;
                        break;
                    }
                }
            }
            force_display = true;
        } else if ((state == GRAY_CAL_ERROR) && key3_pressed) {
            state = GRAY_CAL_WHITE_WAIT;
            calibration_error = GRAY_CAL_ERROR_NONE;
            display_page = GRAY_DISPLAY_NORMALIZED;
            force_display = true;
        } else if (state == GRAY_CAL_RUNNING) {
            if (key1_pressed) {
                display_page = GRAY_DISPLAY_WHITE_CAL;
                force_display = true;
            } else if (key2_pressed) {
                display_page = GRAY_DISPLAY_BLACK_CAL;
                force_display = true;
            } else if (key3_pressed) {
                display_page = (display_page == GRAY_DISPLAY_NORMALIZED) ?
                    GRAY_DISPLAY_RAW : GRAY_DISPLAY_NORMALIZED;
                force_display = true;
            }
        }
        now_ms = TiMspm0Platform_Millis();
        if (OLED_IsInitialized() &&
            (force_display ||
             ((uint32_t)(now_ms - last_display_ms) >=
              GRAY_DISPLAY_PERIOD_MS))) {
            force_display = false;
            last_display_ms = now_ms;
            if (state == GRAY_CAL_WHITE_WAIT) {
                gGrayDebugOledStatus = DisplayCalibrationMessage(
                    "PUT WHITE", "MID KEY1", 0);
            } else if (state == GRAY_CAL_BLACK_WAIT) {
                gGrayDebugOledStatus = DisplayCalibrationMessage(
                    "PUT BLACK", "RIGHT KEY2", 0);
            } else if (state == GRAY_CAL_ERROR) {
                char error_detail[22] = {0};

                if (calibration_error == GRAY_CAL_ERROR_SPAN) {
                    (void)snprintf(error_detail, sizeof(error_detail),
                                   "CH:%u DIFF:%ld",
                                   (unsigned)(bad_channel + 1U),
                                   (long)bad_span);
                } else {
                    (void)snprintf(error_detail, sizeof(error_detail),
                                   "ADC:%s", adc_ok ? "NOW OK" : "FAILED");
                }
                gGrayDebugOledStatus = DisplayCalibrationMessage(
                    calibration_error == GRAY_CAL_ERROR_SPAN ?
                    "CAL SPAN ERROR" : "ADC READ ERROR",
                    error_detail, "LEFT KEY3 RETRY");
            } else if (display_page == GRAY_DISPLAY_WHITE_CAL) {
                gGrayDebugOledStatus = DisplayCalibrationValues(
                    white, "CAL WHITE RAW");
            } else if (display_page == GRAY_DISPLAY_BLACK_CAL) {
                gGrayDebugOledStatus = DisplayCalibrationValues(
                    black, "CAL BLACK RAW");
            } else if (display_page == GRAY_DISPLAY_RAW) {
                gGrayDebugOledStatus = DisplayRawValues(&gGrayDebugArray);
            } else {
                gGrayDebugOledStatus = DisplayGrayValues(
                    &gGrayDebugArray, adc_ok, gGrayDebugFrame,
                    classification_ok ? &classification : 0);
            }
        }
        GrayAdcDebug_DelayMs(GRAY_BUTTON_POLL_MS);
    }
}
