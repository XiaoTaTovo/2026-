#ifndef H2024_BUZZER_H
#define H2024_BUZZER_H

#include <stdbool.h>
#include <stdint.h>

#include "car_types.h"

typedef void (*BuzzerSetFn)(bool enabled, void *context);

typedef struct {
    BuzzerSetFn set_output;
    void *context;
    uint8_t pulses_remaining;
    uint16_t on_ms;
    uint16_t off_ms;
    uint32_t deadline_ms;
    bool output_on;
    bool active;
} Buzzer;

void Buzzer_Init(Buzzer *buzzer, BuzzerSetFn set_output, void *context);
void Buzzer_PlayCue(Buzzer *buzzer, CarCue cue, uint32_t now_ms);
void Buzzer_Update(Buzzer *buzzer, uint32_t now_ms);
void Buzzer_Stop(Buzzer *buzzer);

#endif
