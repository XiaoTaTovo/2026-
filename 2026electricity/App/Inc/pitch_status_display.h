#ifndef PITCH_STATUS_DISPLAY_H
#define PITCH_STATUS_DISPLAY_H

#include <stdbool.h>
#include <stdint.h>

#include "pitch_axis_self_test.h"
#include "pitch_axis_vision_control.h"
#include "pitch_task_controller.h"
#include "stm32f4xx_hal.h"

#define PITCH_STATUS_DISPLAY_WIDTH 128U
#define PITCH_STATUS_DISPLAY_PAGE_COUNT 8U
#define PITCH_STATUS_DISPLAY_DEFAULT_ADDRESS_7BIT 0x3CU

typedef enum
{
    PITCH_STATUS_DISPLAY_STATE_UNINITIALIZED = 0,
    PITCH_STATUS_DISPLAY_STATE_START_INIT,
    PITCH_STATUS_DISPLAY_STATE_WAIT_INIT,
    PITCH_STATUS_DISPLAY_STATE_READY,
    PITCH_STATUS_DISPLAY_STATE_START_CURSOR,
    PITCH_STATUS_DISPLAY_STATE_WAIT_CURSOR,
    PITCH_STATUS_DISPLAY_STATE_WAIT_PAGE,
    PITCH_STATUS_DISPLAY_STATE_START_DISPLAY_ON,
    PITCH_STATUS_DISPLAY_STATE_WAIT_DISPLAY_ON,
    PITCH_STATUS_DISPLAY_STATE_RETRY_WAIT
} PitchStatusDisplayState;

typedef struct
{
    I2C_HandleTypeDef *i2c;
    PitchTaskController *task_controller;
    PitchAxisSelfTest *self_test;
    PitchAxisVisionControl *vision;
    PitchAxisVelocityTest *velocity;
    uint8_t address_7bit;
    uint8_t framebuffer[PITCH_STATUS_DISPLAY_PAGE_COUNT]
                       [PITCH_STATUS_DISPLAY_WIDTH];
    uint8_t tx_buffer[PITCH_STATUS_DISPLAY_WIDTH + 1U];
    PitchAxisVelocityTestButtons buttons;
    PitchStatusDisplayState state;
    uint8_t tx_page;
    uint32_t last_render_ms;
    uint32_t retry_since_ms;
    uint32_t error_count;
    bool render_pending;
    bool first_frame_written;
    bool initialized;
} PitchStatusDisplay;

bool PitchStatusDisplay_Init(
    PitchStatusDisplay *display,
    I2C_HandleTypeDef *i2c,
    uint8_t address_7bit,
    PitchTaskController *task_controller,
    PitchAxisSelfTest *self_test,
    PitchAxisVisionControl *vision,
    PitchAxisVelocityTest *velocity,
    uint32_t now_ms);

/* Cooperative entry point. Every I2C transfer is interrupt driven. */
void PitchStatusDisplay_Service(
    PitchStatusDisplay *display,
    PitchAxisVelocityTestButtons buttons,
    uint32_t now_ms);

#endif
