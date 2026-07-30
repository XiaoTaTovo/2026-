#include "drivers/buzzer.h"

static void Buzzer_Start(Buzzer *buzzer,
                         uint8_t pulses,
                         uint16_t on_ms,
                         uint16_t off_ms,
                         uint32_t now_ms)
{
    if ((buzzer == 0) || (buzzer->set_output == 0) || (pulses == 0U)) {
        return;
    }
    buzzer->pulses_remaining = pulses;
    buzzer->on_ms = on_ms;
    buzzer->off_ms = off_ms;
    buzzer->output_on = true;
    buzzer->active = true;
    buzzer->deadline_ms = now_ms + on_ms;
    buzzer->set_output(true, buzzer->context);
}

void Buzzer_Init(Buzzer *buzzer, BuzzerSetFn set_output, void *context)
{
    if (buzzer == 0) {
        return;
    }
    *buzzer = (Buzzer){0};
    buzzer->set_output = set_output;
    buzzer->context = context;
    if (set_output != 0) {
        set_output(false, context);
    }
}

void Buzzer_PlayCue(Buzzer *buzzer, CarCue cue, uint32_t now_ms)
{
    switch (cue) {
        case CAR_CUE_START:
            Buzzer_Start(buzzer, 1U, 120U, 80U, now_ms);
            break;
        case CAR_CUE_CHECKPOINT:
            Buzzer_Start(buzzer, 1U, 80U, 60U, now_ms);
            break;
        case CAR_CUE_FINISH:
            Buzzer_Start(buzzer, 2U, 150U, 100U, now_ms);
            break;
        case CAR_CUE_FAULT:
            Buzzer_Start(buzzer, 4U, 80U, 80U, now_ms);
            break;
        case CAR_CUE_NONE:
        default:
            break;
    }
}

void Buzzer_Update(Buzzer *buzzer, uint32_t now_ms)
{
    if ((buzzer == 0) || !buzzer->active ||
        ((int32_t)(now_ms - buzzer->deadline_ms) < 0)) {
        return;
    }
    if (buzzer->output_on) {
        buzzer->set_output(false, buzzer->context);
        buzzer->output_on = false;
        buzzer->pulses_remaining--;
        if (buzzer->pulses_remaining == 0U) {
            buzzer->active = false;
        } else {
            buzzer->deadline_ms = now_ms + buzzer->off_ms;
        }
    } else {
        buzzer->set_output(true, buzzer->context);
        buzzer->output_on = true;
        buzzer->deadline_ms = now_ms + buzzer->on_ms;
    }
}

void Buzzer_Stop(Buzzer *buzzer)
{
    if (buzzer == 0) {
        return;
    }
    buzzer->active = false;
    buzzer->output_on = false;
    if (buzzer->set_output != 0) {
        buzzer->set_output(false, buzzer->context);
    }
}
