#ifndef ICM42688_H
#define ICM42688_H

#include <stdbool.h>
#include <stdint.h>

typedef uint8_t (*Icm42688TransferFn)(uint8_t value, void *context);
typedef void (*Icm42688ChipSelectFn)(bool active, void *context);
typedef void (*Icm42688DelayMsFn)(uint32_t delay_ms, void *context);

//spi协议是全双工的，你发送一个字节同时也收到一个字节

typedef struct {
    Icm42688TransferFn transfer;//收发一个字节
    Icm42688ChipSelectFn chip_select;//拉高拉低
    Icm42688DelayMsFn delay_ms;//延时
    void *context;
} Icm42688Port;

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    uint32_t timestamp_ms;
    bool valid;
} Icm42688Sample;

typedef struct {
    Icm42688Port port;
    float gyro_lsb_per_dps;
    uint8_t who_am_i;
    uint32_t read_errors;
    bool initialized;
} Icm42688;

void Icm42688_InitObject(Icm42688 *imu, const Icm42688Port *port);
bool Icm42688_Initialize(Icm42688 *imu);
bool Icm42688_ReadSample(Icm42688 *imu,
                         uint32_t now_ms,
                         Icm42688Sample *sample);

#endif

//icm42688是一个惯性测量单元（IMU）传感器，包含加速度计和陀螺仪。
//加速度计测量加速度；陀螺仪测量角速度。通过读取这些数据，可以推算出物体的运动状态和方向。
