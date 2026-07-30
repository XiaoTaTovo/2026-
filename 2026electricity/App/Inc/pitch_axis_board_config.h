#ifndef PITCH_AXIS_BOARD_CONFIG_H
#define PITCH_AXIS_BOARD_CONFIG_H

#include "i2c.h"

/*
 * Temporary bench route: AS5600 -> I2C1, PB8(SCL)/PB9(SDA).
 * Final PCB route: change only this macro to (&hi2c3), PA8(SCL)/PC9(SDA).
 */
#define PITCH_AXIS_ANGLE_I2C_HANDLE (&hi2c1)

#endif
