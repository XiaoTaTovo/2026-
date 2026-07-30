#include "OLED.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ti_msp_dl_config.h"

#define OLED_I2C_ERRATA_DELAY_CYCLES       (16U)
#define OLED_I2C_BASE_TIMEOUT_CYCLES       (200000U)
#define OLED_I2C_TIMEOUT_CYCLES_PER_BYTE   (2000U)
#define OLED_CONTROL_BYTE_COMMAND          (0x00U)
#define OLED_CONTROL_BYTE_DATA             (0x40U)
#define OLED_DATA_CHUNK_SIZE               (16U)
#define OLED_FONT_WIDTH                    (5U)
#define OLED_FONT_STRIDE                   (6U)

typedef struct {
    char ch;
    uint8_t column[OLED_FONT_WIDTH];
} OLED_Glyph5x7;

static OLED_Config gConfig;
static bool gInitialized;
static uint8_t gBuffer[OLED_PAGE_COUNT][OLED_WIDTH];

static const OLED_Glyph5x7 gFont5x7[] = {
    {' ', {0x00, 0x00, 0x00, 0x00, 0x00}},
    {'!', {0x00, 0x00, 0x5F, 0x00, 0x00}},
    {'-', {0x08, 0x08, 0x08, 0x08, 0x08}},
    {'.', {0x00, 0x60, 0x60, 0x00, 0x00}},
    {'/', {0x20, 0x10, 0x08, 0x04, 0x02}},
    {':', {0x00, 0x36, 0x36, 0x00, 0x00}},
    {'?', {0x02, 0x01, 0x51, 0x09, 0x06}},
    {'0', {0x3E, 0x51, 0x49, 0x45, 0x3E}},
    {'1', {0x00, 0x42, 0x7F, 0x40, 0x00}},
    {'2', {0x42, 0x61, 0x51, 0x49, 0x46}},
    {'3', {0x21, 0x41, 0x45, 0x4B, 0x31}},
    {'4', {0x18, 0x14, 0x12, 0x7F, 0x10}},
    {'5', {0x27, 0x45, 0x45, 0x45, 0x39}},
    {'6', {0x3C, 0x4A, 0x49, 0x49, 0x30}},
    {'7', {0x01, 0x71, 0x09, 0x05, 0x03}},
    {'8', {0x36, 0x49, 0x49, 0x49, 0x36}},
    {'9', {0x06, 0x49, 0x49, 0x29, 0x1E}},
    {'A', {0x7E, 0x11, 0x11, 0x11, 0x7E}},
    {'B', {0x7F, 0x49, 0x49, 0x49, 0x36}},
    {'C', {0x3E, 0x41, 0x41, 0x41, 0x22}},
    {'D', {0x7F, 0x41, 0x41, 0x22, 0x1C}},
    {'E', {0x7F, 0x49, 0x49, 0x49, 0x41}},
    {'F', {0x7F, 0x09, 0x09, 0x09, 0x01}},
    {'G', {0x3E, 0x41, 0x49, 0x49, 0x7A}},
    {'H', {0x7F, 0x08, 0x08, 0x08, 0x7F}},
    {'I', {0x00, 0x41, 0x7F, 0x41, 0x00}},
    {'J', {0x20, 0x40, 0x41, 0x3F, 0x01}},
    {'K', {0x7F, 0x08, 0x14, 0x22, 0x41}},
    {'L', {0x7F, 0x40, 0x40, 0x40, 0x40}},
    {'M', {0x7F, 0x02, 0x0C, 0x02, 0x7F}},
    {'N', {0x7F, 0x04, 0x08, 0x10, 0x7F}},
    {'O', {0x3E, 0x41, 0x41, 0x41, 0x3E}},
    {'P', {0x7F, 0x09, 0x09, 0x09, 0x06}},
    {'Q', {0x3E, 0x41, 0x51, 0x21, 0x5E}},
    {'R', {0x7F, 0x09, 0x19, 0x29, 0x46}},
    {'S', {0x46, 0x49, 0x49, 0x49, 0x31}},
    {'T', {0x01, 0x01, 0x7F, 0x01, 0x01}},
    {'U', {0x3F, 0x40, 0x40, 0x40, 0x3F}},
    {'V', {0x1F, 0x20, 0x40, 0x20, 0x1F}},
    {'W', {0x3F, 0x40, 0x38, 0x40, 0x3F}},
    {'X', {0x63, 0x14, 0x08, 0x14, 0x63}},
    {'Y', {0x07, 0x08, 0x70, 0x08, 0x07}},
    {'Z', {0x61, 0x51, 0x49, 0x45, 0x43}},
};

static bool OLED_IsValidAddress(uint8_t address7bit)
{
    return ((address7bit >= 0x08U) && (address7bit <= 0x77U));
}

static bool OLED_IsValidConfig(const OLED_Config *config)
{
    if (config == NULL) {
        return false;
    }

    return OLED_IsValidAddress(config->address7bit) &&
           (config->width == OLED_WIDTH) &&
           (config->height == OLED_HEIGHT) &&
           (config->columnOffset <= 2U);
}

static OLED_Status OLED_StatusFromI2C(uint32_t controllerStatus)
{
    if ((controllerStatus & DL_I2C_CONTROLLER_STATUS_ERROR) == 0U) {
        return OLED_STATUS_OK;
    }

    if ((controllerStatus & DL_I2C_CONTROLLER_STATUS_ADDR_ACK) == 0U) {
        return OLED_STATUS_ERROR_I2C_ADDRESS_NACK;
    }

    if ((controllerStatus & DL_I2C_CONTROLLER_STATUS_DATA_ACK) == 0U) {
        return OLED_STATUS_ERROR_I2C_DATA_NACK;
    }

    return OLED_STATUS_ERROR_I2C_BUS;
}

static OLED_Status OLED_WaitIdle(void)
{
    uint32_t timeout = OLED_I2C_BASE_TIMEOUT_CYCLES;

    while ((DL_I2C_getControllerStatus(I2C_OLED_INST) &
               DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        if (timeout == 0U) {
            return OLED_STATUS_ERROR_I2C_TIMEOUT;
        }
        timeout--;
    }

    return OLED_STATUS_OK;
}

static OLED_Status OLED_I2C_WriteRaw(
    uint8_t address7bit, const uint8_t *data, uint16_t length)
{
    OLED_Status status;
    uint16_t written = 0U;
    uint32_t timeout = OLED_I2C_BASE_TIMEOUT_CYCLES +
                       ((uint32_t) length * OLED_I2C_TIMEOUT_CYCLES_PER_BYTE);

    if (!OLED_IsValidAddress(address7bit)) {
        return OLED_STATUS_ERROR_INVALID_CONFIG;
    }

    if ((data == NULL) && (length > 0U)) {
        return OLED_STATUS_ERROR_NULL;
    }

    status = OLED_WaitIdle();
    if (status != OLED_STATUS_OK) {
        return status;
    }

    DL_I2C_flushControllerTXFIFO(I2C_OLED_INST);
    if (length > 0U) {
        written =
            DL_I2C_fillControllerTXFIFO(I2C_OLED_INST, data, length);
    }

    DL_I2C_startControllerTransfer(
        I2C_OLED_INST, address7bit, DL_I2C_CONTROLLER_DIRECTION_TX, length);
    delay_cycles(OLED_I2C_ERRATA_DELAY_CYCLES);

    while ((DL_I2C_getControllerStatus(I2C_OLED_INST) &
               DL_I2C_CONTROLLER_STATUS_BUSY) != 0U) {
        uint32_t controllerStatus = DL_I2C_getControllerStatus(I2C_OLED_INST);

        status = OLED_StatusFromI2C(controllerStatus);
        if (status != OLED_STATUS_OK) {
            DL_I2C_resetControllerTransfer(I2C_OLED_INST);
            return status;
        }

        if (written < length) {
            written += DL_I2C_fillControllerTXFIFO(
                I2C_OLED_INST, &data[written], (uint16_t) (length - written));
        }

        if (timeout == 0U) {
            DL_I2C_resetControllerTransfer(I2C_OLED_INST);
            return OLED_STATUS_ERROR_I2C_TIMEOUT;
        }
        timeout--;
    }

    status = OLED_StatusFromI2C(DL_I2C_getControllerStatus(I2C_OLED_INST));
    if (status != OLED_STATUS_OK) {
        DL_I2C_resetControllerTransfer(I2C_OLED_INST);
    }

    return status;
}

static OLED_Status OLED_WriteCommands(const uint8_t *commands, uint16_t count)
{
    uint8_t packet[32U];

    if ((commands == NULL) || (count == 0U)) {
        return OLED_STATUS_ERROR_NULL;
    }

    if ((count + 1U) > sizeof(packet)) {
        return OLED_STATUS_ERROR_OUT_OF_RANGE;
    }

    packet[0] = OLED_CONTROL_BYTE_COMMAND;
    memcpy(&packet[1], commands, count);
    return OLED_I2C_WriteRaw(gConfig.address7bit, packet, (uint16_t) (count + 1U));
}

static OLED_Status OLED_WriteData(const uint8_t *data, uint16_t count)
{
    uint8_t packet[OLED_DATA_CHUNK_SIZE + 1U];
    uint16_t offset = 0U;

    if ((data == NULL) || (count == 0U)) {
        return OLED_STATUS_ERROR_NULL;
    }

    while (offset < count) {
        uint16_t chunk = (uint16_t) (count - offset);
        OLED_Status status;

        if (chunk > OLED_DATA_CHUNK_SIZE) {
            chunk = OLED_DATA_CHUNK_SIZE;
        }

        packet[0] = OLED_CONTROL_BYTE_DATA;
        memcpy(&packet[1], &data[offset], chunk);

        status = OLED_I2C_WriteRaw(
            gConfig.address7bit, packet, (uint16_t) (chunk + 1U));
        if (status != OLED_STATUS_OK) {
            return status;
        }

        offset = (uint16_t) (offset + chunk);
    }

    return OLED_STATUS_OK;
}

static OLED_Status OLED_SetCursor(uint8_t page, uint8_t x)
{
    uint8_t column = (uint8_t) (x + gConfig.columnOffset);
    const uint8_t commands[] = {
        (uint8_t) (0xB0U | page),
        (uint8_t) (0x00U | (column & 0x0FU)),
        (uint8_t) (0x10U | ((column >> 4U) & 0x0FU)),
    };

    if ((page >= OLED_PAGE_COUNT) || (x >= OLED_WIDTH)) {
        return OLED_STATUS_ERROR_OUT_OF_RANGE;
    }

    return OLED_WriteCommands(commands, (uint16_t) sizeof(commands));
}

static const uint8_t *OLED_FindGlyph(char ch)
{
    uint32_t i;

    /* 调试屏优先保证英文状态和数字；小写字母统一映射成大写。 */
    if ((ch >= 'a') && (ch <= 'z')) {
        ch = (char) (ch - ('a' - 'A'));
    }

    for (i = 0U; i < (sizeof(gFont5x7) / sizeof(gFont5x7[0])); i++) {
        if (gFont5x7[i].ch == ch) {
            return gFont5x7[i].column;
        }
    }

    return OLED_FindGlyph('?');
}

static OLED_Status OLED_RequireInitialized(void)
{
    return gInitialized ? OLED_STATUS_OK : OLED_STATUS_ERROR_NOT_INITIALIZED;
}

OLED_Config OLED_MakeSSD1306Config(uint8_t address7bit)
{
    OLED_Config config = {
        .address7bit = address7bit,
        .model = OLED_MODEL_SSD1306_128X64,
        .width = OLED_WIDTH,
        .height = OLED_HEIGHT,
        .columnOffset = 0U,
    };

    return config;
}

OLED_Config OLED_MakeSH1106Config(uint8_t address7bit)
{
    OLED_Config config = {
        .address7bit = address7bit,
        .model = OLED_MODEL_SH1106_128X64,
        .width = OLED_WIDTH,
        .height = OLED_HEIGHT,
        .columnOffset = 2U,
    };

    return config;
}

OLED_Status OLED_ProbeAddress(uint8_t address7bit)
{
    if (!OLED_IsValidAddress(address7bit)) {
        return OLED_STATUS_ERROR_INVALID_CONFIG;
    }

    return OLED_I2C_WriteRaw(address7bit, NULL, 0U);
}

OLED_Status OLED_Init(const OLED_Config *config)
{
    static const uint8_t ssd1306Init[] = {
        0xAE,       /* display off */
        0xD5, 0x80, /* clock divide */
        0xA8, 0x3F, /* multiplex 1/64 */
        0xD3, 0x00, /* display offset */
        0x40,       /* start line */
        0xA1,       /* segment remap */
        0xC8,       /* COM scan direction */
        0xDA, 0x12, /* COM pins */
        0x81, 0xCF, /* contrast */
        0xD9, 0xF1, /* pre-charge */
        0xDB, 0x30, /* VCOMH */
        0xA4,       /* display follows RAM */
        0xA6,       /* normal display */
        0x8D, 0x14, /* charge pump */
        0x20, 0x02, /* page addressing mode */
        0xAF,       /* display on */
    };

    static const uint8_t sh1106Init[] = {
        0xAE,       /* display off */
        0xD5, 0x80,
        0xA8, 0x3F,
        0xD3, 0x00,
        0x40,
        0xA1,
        0xC8,
        0xDA, 0x12,
        0x81, 0xCF,
        0xD9, 0xF1,
        0xDB, 0x40,
        0xA4,
        0xA6,
        0xAF,
    };

    OLED_Status status;

    if (!OLED_IsValidConfig(config)) {
        return OLED_STATUS_ERROR_INVALID_CONFIG;
    }

    if (config->model == OLED_MODEL_UNKNOWN) {
        return OLED_STATUS_ERROR_UNSUPPORTED_MODEL;
    }

    gConfig = *config;
    gInitialized = false;

    if (config->model == OLED_MODEL_SSD1306_128X64) {
        status = OLED_WriteCommands(ssd1306Init, (uint16_t) sizeof(ssd1306Init));
    } else if (config->model == OLED_MODEL_SH1106_128X64) {
        status = OLED_WriteCommands(sh1106Init, (uint16_t) sizeof(sh1106Init));
    } else {
        return OLED_STATUS_ERROR_UNSUPPORTED_MODEL;
    }

    if (status != OLED_STATUS_OK) {
        return status;
    }

    gInitialized = true;
    status = OLED_Clear();
    if (status != OLED_STATUS_OK) {
        return status;
    }

    return OLED_Update();
}

bool OLED_IsInitialized(void)
{
    return gInitialized;
}

OLED_Status OLED_Clear(void)
{
    memset(gBuffer, 0, sizeof(gBuffer));
    return OLED_STATUS_OK;
}

OLED_Status OLED_Fill(bool on)
{
    memset(gBuffer, on ? 0xFF : 0x00, sizeof(gBuffer));
    return OLED_STATUS_OK;
}

OLED_Status OLED_Update(void)
{
    return OLED_UpdatePages(0U, OLED_PAGE_COUNT);
}

OLED_Status OLED_UpdatePages(uint8_t firstPage, uint8_t pageCount)
{
    uint8_t page;
    OLED_Status status = OLED_RequireInitialized();

    if (status != OLED_STATUS_OK) {
        return status;
    }

    if ((firstPage >= OLED_PAGE_COUNT) ||
        ((uint16_t) firstPage + pageCount > OLED_PAGE_COUNT) ||
        (pageCount == 0U)) {
        return OLED_STATUS_ERROR_OUT_OF_RANGE;
    }

    for (page = firstPage; page < (uint8_t) (firstPage + pageCount); page++) {
        status = OLED_SetCursor(page, 0U);
        if (status != OLED_STATUS_OK) {
            return status;
        }

        status = OLED_WriteData(gBuffer[page], OLED_WIDTH);
        if (status != OLED_STATUS_OK) {
            return status;
        }
    }

    return OLED_STATUS_OK;
}

OLED_Status OLED_SetPixel(uint8_t x, uint8_t y, bool on)
{
    uint8_t mask;

    if ((x >= OLED_WIDTH) || (y >= OLED_HEIGHT)) {
        return OLED_STATUS_ERROR_OUT_OF_RANGE;
    }

    mask = (uint8_t) (1U << (y & 0x07U));
    if (on) {
        gBuffer[y >> 3U][x] |= mask;
    } else {
        gBuffer[y >> 3U][x] &= (uint8_t) ~mask;
    }

    return OLED_STATUS_OK;
}

OLED_Status OLED_ShowChar(uint8_t x, uint8_t page, char ch)
{
    const uint8_t *glyph;
    uint8_t column;

    if ((page >= OLED_PAGE_COUNT) || (x >= OLED_WIDTH)) {
        return OLED_STATUS_ERROR_OUT_OF_RANGE;
    }

    glyph = OLED_FindGlyph(ch);
    for (column = 0U; column < OLED_FONT_WIDTH; column++) {
        if ((x + column) < OLED_WIDTH) {
            gBuffer[page][x + column] = glyph[column];
        }
    }

    if ((x + OLED_FONT_WIDTH) < OLED_WIDTH) {
        gBuffer[page][x + OLED_FONT_WIDTH] = 0x00U;
    }

    return OLED_STATUS_OK;
}

OLED_Status OLED_ShowString(uint8_t x, uint8_t page, const char *text)
{
    uint8_t cursor = x;

    if (text == NULL) {
        return OLED_STATUS_ERROR_NULL;
    }

    if ((page >= OLED_PAGE_COUNT) || (x >= OLED_WIDTH)) {
        return OLED_STATUS_ERROR_OUT_OF_RANGE;
    }

    while ((*text != '\0') && (cursor < OLED_WIDTH)) {
        OLED_Status status = OLED_ShowChar(cursor, page, *text);
        if (status != OLED_STATUS_OK) {
            return status;
        }

        if ((OLED_WIDTH - cursor) < OLED_FONT_STRIDE) {
            break;
        }
        cursor = (uint8_t) (cursor + OLED_FONT_STRIDE);
        text++;
    }

    return OLED_STATUS_OK;
}

OLED_Status OLED_RenderDebugPage(const OLED_DebugPage *page)
{
    char line[22];

    if (page == NULL) {
        return OLED_STATUS_ERROR_NULL;
    }

    OLED_Clear();

    (void) snprintf(line, sizeof(line), "TASK:%s",
        (page->taskState != NULL) ? page->taskState : "NULL");
    OLED_ShowString(0U, 0U, line);

    (void) snprintf(line, sizeof(line), "F:%4u R:%4u",
        page->frontDistanceMm, page->rightDistanceMm);
    OLED_ShowString(0U, 1U, line);

    (void) snprintf(line, sizeof(line), "L:%4u", page->leftDistanceMm);
    OLED_ShowString(0U, 2U, line);

    (void) snprintf(line, sizeof(line), "M:%d/%d",
        page->leftMotorSpeed, page->rightMotorSpeed);
    OLED_ShowString(0U, 3U, line);

    (void) snprintf(line, sizeof(line), "ERR:%04X", page->errorCode);
    OLED_ShowString(0U, 4U, line);

    return OLED_Update();
}
