#include "encoder.h"

#include <stdbool.h>

#include "ti_msp_dl_config.h"

volatile int32_t gEncoderLeftCount;
volatile int32_t gEncoderRightCount;

static int32_t gLeftDeltaPrevious;
static int32_t gRightDeltaPrevious;

static int32_t count_delta(int32_t current, int32_t *previous)
{
    int32_t old = *previous;
    *previous = current;

    /* Unsigned subtraction keeps counter wraparound behavior defined. */
    return (int32_t) ((uint32_t) current - (uint32_t) old);
}

void Encoder_Init(void)
{
    NVIC_DisableIRQ(ENC_A_INT_IRQN);
    gEncoderLeftCount   = 0;
    gEncoderRightCount  = 0;
    gLeftDeltaPrevious  = 0;
    gRightDeltaPrevious = 0;

    DL_GPIO_clearInterruptStatus(
        ENC_A_PORT, ENC_A_PIN_AL_PIN | ENC_A_PIN_AR_PIN);
    NVIC_ClearPendingIRQ(ENC_A_INT_IRQN);
    NVIC_EnableIRQ(ENC_A_INT_IRQN);
}

void Encoder_ResetCounts(void)
{
    uint32_t interruptState = __get_PRIMASK();

    __disable_irq();
    gEncoderLeftCount   = 0;
    gEncoderRightCount  = 0;
    gLeftDeltaPrevious  = 0;
    gRightDeltaPrevious = 0;
    if (interruptState == 0U) {
        __enable_irq();
    }
}

int32_t Encoder_GetLeftCount(void)
{
    return gEncoderLeftCount;
}

int32_t Encoder_GetRightCount(void)
{
    return gEncoderRightCount;
}

int32_t Encoder_GetLeftDelta(void)
{
    return count_delta(gEncoderLeftCount, &gLeftDeltaPrevious);
}

int32_t Encoder_GetRightDelta(void)
{
    return count_delta(gEncoderRightCount, &gRightDeltaPrevious);
}

void GROUP1_IRQHandler(void)
{
    uint32_t pending = DL_GPIO_getEnabledInterruptStatus(
        ENC_A_PORT, ENC_A_PIN_AL_PIN | ENC_A_PIN_AR_PIN);

    if ((pending & ENC_A_PIN_AL_PIN) != 0U) {
        bool aHigh;
        bool bHigh;

        DL_GPIO_clearInterruptStatus(ENC_A_PORT, ENC_A_PIN_AL_PIN);
        aHigh = DL_GPIO_readPins(ENC_A_PORT, ENC_A_PIN_AL_PIN) != 0U;
        bHigh = DL_GPIO_readPins(ENC_B_PORT, ENC_B_PIN_BL_PIN) != 0U;
        gEncoderLeftCount += (aHigh != bHigh) ? ENC_LEFT_SIGN : -ENC_LEFT_SIGN;
    }

    if ((pending & ENC_A_PIN_AR_PIN) != 0U) {
        bool aHigh;
        bool bHigh;

        DL_GPIO_clearInterruptStatus(ENC_A_PORT, ENC_A_PIN_AR_PIN);
        aHigh = DL_GPIO_readPins(ENC_A_PORT, ENC_A_PIN_AR_PIN) != 0U;
        bHigh = DL_GPIO_readPins(ENC_B_PORT, ENC_B_PIN_BR_PIN) != 0U;
        gEncoderRightCount += (aHigh != bHigh) ? ENC_RIGHT_SIGN : -ENC_RIGHT_SIGN;
    }
}
