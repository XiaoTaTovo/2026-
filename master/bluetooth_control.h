#ifndef BLUETOOTH_CONTROL_H
#define BLUETOOTH_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    BLUETOOTH_MODE_STOP = 0,
    BLUETOOTH_MODE_OPEN_LOOP = 1,
    BLUETOOTH_MODE_SPEED_LOOP = 2
} BluetoothControlMode;

typedef enum {
    BLUETOOTH_MOTION_STOP = 0,
    BLUETOOTH_MOTION_FORWARD = 1,
    BLUETOOTH_MOTION_BACKWARD = 2,
    BLUETOOTH_MOTION_LEFT = 3,
    BLUETOOTH_MOTION_RIGHT = 4,
    BLUETOOTH_MOTION_DIRECT = 5
} BluetoothMotion;

typedef struct {
    BluetoothControlMode mode;
    BluetoothMotion motion;
    int32_t targetLeftRpm;
    int32_t targetRightRpm;
    int32_t measuredLeftRpm;
    int32_t measuredRightRpm;
    int8_t leftCommandPercent;
    int8_t rightCommandPercent;
    int32_t leftDeltaCount;
    int32_t rightDeltaCount;
    int32_t leftTotalCount;
    int32_t rightTotalCount;
    uint32_t leftCountsPerRev;
    uint32_t rightCountsPerRev;
    uint32_t kpMilli;
    uint32_t kiMilli;
    uint32_t kdMilli;
    uint8_t speedLimitPercent;
    uint8_t dutyPercent;
    uint32_t rxCount;
    uint32_t errorCount;
    uint32_t failsafeCount;
} BluetoothControlStatus;

void BluetoothControl_Init(void);
void BluetoothControl_PushRxFromIsr(uint8_t byte);
bool BluetoothControl_ProcessPending(uint32_t nowMs);
void BluetoothControl_Update(uint32_t nowMs);
bool BluetoothControl_CheckFailsafe(uint32_t nowMs);
void BluetoothControl_GetStatus(BluetoothControlStatus *status);

#endif
