#ifndef TEST_TI_MSP_DL_CONFIG_H
#define TEST_TI_MSP_DL_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t unused;
} TestGpioRegs;

typedef struct {
    uint32_t unused;
} TestTimerARegs;

typedef struct {
    uint32_t unused;
} TestUartRegs;

extern TestGpioRegs g_test_gpio_a;
extern TestGpioRegs g_test_gpio_b;
extern TestTimerARegs g_test_timer_a;
extern TestUartRegs g_test_uart;

#define CPUCLK_FREQ (32000000U)

#define A_PORT (&g_test_gpio_a)
#define A_PIN_AIN1_PIN (1U << 14)
#define A_PIN_AIN2_PIN (1U << 15)

#define B_PORT (&g_test_gpio_a)
#define B_PIN_BIN1_PIN (1U << 16)
#define B_PIN_BIN2_PIN (1U << 17)

#define STBY_PORT (&g_test_gpio_a)
#define STBY_PIN_STBY_PIN (1U << 28)

#define PWM_TB1_INST (&g_test_timer_a)
#define GPIO_PWM_TB1_C1_IDX (1U)
#define GPIO_PWM_TB1_C2_IDX (2U)

#define UART_VOFA_INST (&g_test_uart)
#define DL_UART_MAIN_INTERRUPT_TX (1U)

void DL_TimerA_setCaptureCompareValue(TestTimerARegs *timer,
                                      uint32_t value,
                                      uint32_t index);
void DL_TimerA_startCounter(TestTimerARegs *timer);
void DL_GPIO_clearPins(TestGpioRegs *gpio, uint32_t pins);
void DL_GPIO_setPins(TestGpioRegs *gpio, uint32_t pins);
bool DL_UART_Main_isTXFIFOFull(TestUartRegs *uart);
void DL_UART_Main_transmitData(TestUartRegs *uart, uint8_t value);
void DL_UART_Main_disableInterrupt(TestUartRegs *uart, uint32_t interrupt);
void DL_UART_Main_enableInterrupt(TestUartRegs *uart, uint32_t interrupt);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __enable_irq(void);
void __DMB(void);
void delay_cycles(uint32_t cycles);

#endif
