#ifndef TI_MSPM0_PLATFORM_H
#define TI_MSPM0_PLATFORM_H

#include <stdbool.h>
#include <stdint.h>

#include "firmware.h"

/*
 * SysConfig owns these fixed resources:
 *
 * TB6612 PWM: PB12/PB4, TIMA0, 20 kHz
 * TB6612 direction/STBY: PA14/PA15, PA16/PA17, PA28
 * encoder A/B: PA7/PA22 and PA30/PA31
 * ICM42688 SPI/CS: PB9/PB8/PB7 and PB0
 * GRAY mux/ADC: PA24/PA25/PA26, PA27; GRAY EN/ERR: PB10/PB11
 * RED ADC: PA18/PA21/PB17/PB18/PB19 on ADC1, PB24/PB25/PB20 on ADC0
 * OLED I2C0: PA0/PA1
 * KEY1/KEY2/KEY3: PB23/PB26/PB27; KEY1 start/stop, KEY2 calibration
 * buzzer: PB5
 * tuning/telemetry UART3: PB2/PB3, 115200 8N1
 */

typedef struct {
    uint32_t imu_spi_timeouts;
    uint32_t gray_adc_timeouts;
    uint32_t gray_adc_isr_completions;
    uint32_t gray_adc_poll_completions;
    uint32_t red_adc_timeouts;
    uint32_t red_adc_poll_completions;
} TiMspm0PlatformDiagnostics;

void TiMspm0Platform_Init(void);
void TiMspm0Platform_OnSysTick(void);
uint32_t TiMspm0Platform_Millis(void);
bool TiMspm0Platform_ReadKey1Level(void);
bool TiMspm0Platform_ReadKey2Level(void);
bool TiMspm0Platform_ReadKey3Level(void);
void TiMspm0Platform_GetDiagnostics(TiMspm0PlatformDiagnostics *diagnostics);
CarStatus TiMspm0Platform_BuildConfig(CarFirmwareConfig *config,
                                      H2026Mode mode);

#endif
