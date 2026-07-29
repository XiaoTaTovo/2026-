#ifndef H2024_BUTTON_H
#define H2024_BUTTON_H

#include <stdbool.h>
#include <stdint.h>

typedef bool (*ButtonReadFn)(void *context);
//bool (*ButtonReadFn)(void *context);定义一个函数指针，指向一个传入为void*类型的参数，返回值为bool类型的函数
//
typedef struct {
    ButtonReadFn read_level;
    void *context;
    uint16_t debounce_ms;
    bool active_low;
    bool stable_pressed;//稳定
    bool candidate_pressed;//候选
    bool pressed_event;
    uint32_t candidate_since_ms;
    bool initialized;
} Button;

void Button_Init(Button *button,
                 ButtonReadFn read_level,
                 void *context,
                 bool active_low,
                 uint16_t debounce_ms);
void Button_Update(Button *button, uint32_t now_ms);
bool Button_TakePressedEvent(Button *button);

#endif

//新读到的电平 - 候选 -稳定