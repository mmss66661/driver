#include "buttons.h"

#include "ti_msp_dl_config.h"

#define BUTTON_DEBOUNCE_MS (20U)

typedef struct {
    bool stableDown;
    bool sampledDown;
    bool pressedEvent;
    uint32_t changedAt;
} Button_State;

static Button_State gButtons[BUTTON_COUNT];
static const uint32_t gButtonPins[BUTTON_COUNT] = {
    BUTTONS_MODE_PIN, BUTTONS_UP_PIN, BUTTONS_DOWN_PIN
};

static bool readDown(Button_Id id)
{
    /* Buttons use pull-ups and are active low. */
    return DL_GPIO_readPins(BUTTONS_PORT, gButtonPins[id]) == 0U;
}

void Buttons_init(void)
{
    uint8_t i;
    for (i = 0U; i < (uint8_t) BUTTON_COUNT; i++) {
        bool down = readDown((Button_Id) i);
        gButtons[i].stableDown = down;
        gButtons[i].sampledDown = down;
        gButtons[i].pressedEvent = false;
        gButtons[i].changedAt = 0U;
    }
}

void Buttons_process(uint32_t nowMs)
{
    uint8_t i;
    for (i = 0U; i < (uint8_t) BUTTON_COUNT; i++) {
        bool down = readDown((Button_Id) i);
        if (down != gButtons[i].sampledDown) {
            gButtons[i].sampledDown = down;
            gButtons[i].changedAt = nowMs;
        } else if ((down != gButtons[i].stableDown) &&
                   ((nowMs - gButtons[i].changedAt) >= BUTTON_DEBOUNCE_MS)) {
            gButtons[i].stableDown = down;
            if (down) {
                gButtons[i].pressedEvent = true;
            }
        }
    }
}

bool Buttons_isDown(Button_Id id)
{
    return ((uint8_t) id < (uint8_t) BUTTON_COUNT) ?
        gButtons[id].stableDown : false;
}

bool Buttons_wasPressed(Button_Id id)
{
    bool result;
    if ((uint8_t) id >= (uint8_t) BUTTON_COUNT) {
        return false;
    }
    result = gButtons[id].pressedEvent;
    gButtons[id].pressedEvent = false;
    return result;
}
