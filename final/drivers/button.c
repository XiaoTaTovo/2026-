#include "drivers/button.h"

void Button_Init(Button *button,
                 ButtonReadFn read_level,
                 void *context,
                 bool active_low,
                 uint16_t debounce_ms)
{
    if (button == 0) {
        return;
    }
    *button = (Button){0};
    button->read_level = read_level;
    button->context = context;
    button->active_low = active_low;
    button->debounce_ms = debounce_ms;
}

void Button_Update(Button *button, uint32_t now_ms)
{
    bool level;
    bool pressed;

    if ((button == 0) || (button->read_level == 0)) {
        return;
    }//保护：button是否为空；读取电平的函数指针是否为空
    level = button->read_level(button->context);
    pressed = button->active_low ? !level : level;
    //level高电平时是true，低电平时是false
    if (!button->initialized) {
        button->stable_pressed = pressed;
        button->candidate_pressed = pressed;
        button->candidate_since_ms = now_ms;
        button->initialized = true;
        return;
    }
    if (pressed != button->candidate_pressed) {
        button->candidate_pressed = pressed;
        button->candidate_since_ms = now_ms;
        return;
    }
    if ((pressed != button->stable_pressed) &&
        ((uint32_t)(now_ms - button->candidate_since_ms) >= button->debounce_ms)) {
        button->stable_pressed = pressed;
        if (pressed) {
            button->pressed_event = true;
        }
    }
}

bool Button_TakePressedEvent(Button *button)
{
    bool event;

    if (button == 0) {
        return false;
    }
    event = button->pressed_event;
    button->pressed_event = false;
    return event;
}
//读后清零，不然虽然成功检测到按下，但是会多次触发