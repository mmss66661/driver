#ifndef BUTTONS_H_
#define BUTTONS_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    BUTTON_MODE = 0,
    BUTTON_UP,
    BUTTON_DOWN,
    BUTTON_COUNT
} Button_Id;

void Buttons_init(void);
void Buttons_process(uint32_t nowMs);
bool Buttons_isDown(Button_Id id);
bool Buttons_wasPressed(Button_Id id);

#endif
