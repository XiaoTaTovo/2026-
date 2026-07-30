#ifndef TI_CAR_USERS_DRIVERS_OLED_OLED_H_
#define TI_CAR_USERS_DRIVERS_OLED_OLED_H_

#include <stdbool.h>
#include <stdint.h>

#define OLED_WIDTH             (128U)
#define OLED_HEIGHT            (64U)
#define OLED_PAGE_COUNT        (8U)
#define OLED_DEFAULT_ADDR_7BIT (0x3CU)

typedef enum {
    OLED_MODEL_UNKNOWN = 0,
    OLED_MODEL_SSD1306_128X64,
    OLED_MODEL_SH1106_128X64,
} OLED_Model;

typedef enum {
    OLED_STATUS_OK = 0,
    OLED_STATUS_ERROR_NULL,
    OLED_STATUS_ERROR_INVALID_CONFIG,
    OLED_STATUS_ERROR_UNSUPPORTED_MODEL,
    OLED_STATUS_ERROR_NOT_INITIALIZED,
    OLED_STATUS_ERROR_OUT_OF_RANGE,
    OLED_STATUS_ERROR_I2C_BUSY,
    OLED_STATUS_ERROR_I2C_TIMEOUT,
    OLED_STATUS_ERROR_I2C_ADDRESS_NACK,
    OLED_STATUS_ERROR_I2C_DATA_NACK,
    OLED_STATUS_ERROR_I2C_BUS,
} OLED_Status;

typedef struct {
    uint8_t address7bit;
    OLED_Model model;
    uint8_t width;
    uint8_t height;
    uint8_t columnOffset;
} OLED_Config;

typedef struct {
    const char *taskState;
    uint16_t frontDistanceMm;
    uint16_t rightDistanceMm;
    uint16_t leftDistanceMm;
    int16_t leftMotorSpeed;
    int16_t rightMotorSpeed;
    uint16_t errorCode;
} OLED_DebugPage;

OLED_Config OLED_MakeSSD1306Config(uint8_t address7bit);
OLED_Config OLED_MakeSH1106Config(uint8_t address7bit);

/* 用零长度 I2C 传输探测 7-bit 地址是否应答，适合上电后做 0x3C/0x3D 扫描。 */
OLED_Status OLED_ProbeAddress(uint8_t address7bit);

/* 必须先显式传入型号配置；未确认型号时不要传 OLED_MODEL_UNKNOWN。 */
OLED_Status OLED_Init(const OLED_Config *config);
bool OLED_IsInitialized(void);

OLED_Status OLED_Clear(void);
OLED_Status OLED_Fill(bool on);
OLED_Status OLED_Update(void);
OLED_Status OLED_UpdatePages(uint8_t firstPage, uint8_t pageCount);

OLED_Status OLED_SetPixel(uint8_t x, uint8_t y, bool on);
OLED_Status OLED_ShowChar(uint8_t x, uint8_t page, char ch);
OLED_Status OLED_ShowString(uint8_t x, uint8_t page, const char *text);
OLED_Status OLED_RenderDebugPage(const OLED_DebugPage *page);

#endif /* TI_CAR_USERS_DRIVERS_OLED_OLED_H_ */
