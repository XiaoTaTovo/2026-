#ifndef GRAY_ARRAY_H
#define GRAY_ARRAY_H

#include <stdbool.h>
#include <stdint.h>

#include "car_types.h"

#define GRAY_ARRAY_CHANNELS (8U)
#define GRAY_ARRAY_FULL_SCALE (1000U)

typedef bool (*GraySelectChannelFn)(uint8_t channel, void *context);
//声明一个函数指针，可以用类型名GraySelectChannelFn来声明一个函数指针变量
//指向一个传入为uint8_t类型的参数和void*类型的参数，返回值为bool类型的函数
typedef bool (*GrayReadAdcFn)(uint16_t *value, void *context);
typedef void (*GrayDelayUsFn)(uint32_t delay_us, void *context);

typedef struct {
    GraySelectChannelFn select_channel;
    GrayReadAdcFn read_adc;
    GrayDelayUsFn delay_us;
    void *context;
    uint16_t settle_us;
    uint8_t samples_per_channel;
} GrayArrayPort;

typedef struct {
    GrayArrayPort port;
    uint16_t black[GRAY_ARRAY_CHANNELS];
    uint16_t white[GRAY_ARRAY_CHANNELS];
    uint16_t raw[GRAY_ARRAY_CHANNELS];
    CarGraySample latest;
    uint32_t read_errors;
    bool calibrated;
} GrayArray;

typedef struct {
    uint8_t black_mask;
    uint8_t white_mask;
    uint8_t unknown_mask;
} GrayArrayClassification;

void GrayArray_Init(GrayArray *array, const GrayArrayPort *port);
bool GrayArray_SetCalibration(GrayArray *array,
                              const uint16_t black[GRAY_ARRAY_CHANNELS],
                              const uint16_t white[GRAY_ARRAY_CHANNELS]);
bool GrayArray_Read(GrayArray *array, uint32_t now_ms);
bool GrayArray_GetLatest(const GrayArray *array, CarGraySample *sample);
bool GrayArray_ClassifyLatest(const GrayArray *array,
                              uint16_t white_threshold,
                              uint16_t black_threshold,
                              GrayArrayClassification *classification);

#endif
