#ifndef DRIVE_PORT_H
#define DRIVE_PORT_H

#include <stdbool.h>
#include <stdint.h>

#include "car_types.h"

typedef bool (*CarDriveSetWheelSpeedsFn)(int16_t left_mm_s,
                                         int16_t right_mm_s,
                                         void *context);
typedef bool (*CarDriveReadEncoderFn)(CarEncoderSample *sample,
                                      void *context);
typedef void (*CarDriveStopFn)(void *context);
typedef void (*CarDriveServiceFn)(void *context);

typedef struct {
    CarDriveSetWheelSpeedsFn set_wheel_speeds;
    CarDriveReadEncoderFn read_encoder;
    CarDriveStopFn stop;
    CarDriveServiceFn service;
    void *context;
} CarDrivePort;

#endif
