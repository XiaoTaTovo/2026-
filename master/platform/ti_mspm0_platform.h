#ifndef H2024_TI_MSPM0_PLATFORM_H
#define H2024_TI_MSPM0_PLATFORM_H

#include <stdbool.h>
#include <stdint.h>

#include "firmware.h"

/*
 * Required SysConfig instance and pin names:
 *
 * UART_MOTOR: UART1, TX PA8, RX PA9, 115200 8N1
 * SPI_IMU: SPI1, POCI PB7, PICO PB8, SCLK PB9, mode confirmed on hardware
 * ADC_GRAY: ADC0 MEM0, OUT PA27, 12 bit, conversion-complete interrupt
 * GPIO_GRAY: AD0 PA24, AD1 PA25, AD2 PA26
 * GPIO_IMU: CS PB0, initialized high
 * GPIO_KEYS: KEY1 PB23, KEY2 PB26, KEY3 PB27, inputs with pull-ups
 * GPIO_BUZZER: BUZZER PB5, initialized low
 * SYSTICK: 1 ms interrupt
 */

typedef struct {
    uint32_t motor_rx_bytes;
    uint32_t motor_rx_overflows;
    uint32_t motor_tx_timeouts;
    uint32_t imu_spi_timeouts;
    uint32_t gray_adc_timeouts;
} TiMspm0PlatformDiagnostics;

void TiMspm0Platform_Init(void);
void TiMspm0Platform_OnSysTick(void);
uint32_t TiMspm0Platform_Millis(void);
bool TiMspm0Platform_ReadKey1Level(void);
bool TiMspm0Platform_ReadKey2Level(void);
bool TiMspm0Platform_ReadKey3Level(void);
void TiMspm0Platform_PollMotorRx(CarFirmware *firmware);
void TiMspm0Platform_ServiceMotorBackend(void);
void TiMspm0Platform_GetDiagnostics(TiMspm0PlatformDiagnostics *diagnostics);
CarStatus TiMspm0Platform_BuildConfig(CarFirmwareConfig *config,
                                      H2024Mode mode);

#endif
