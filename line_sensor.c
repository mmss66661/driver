#include "line_sensor.h"

#include "ti_msp_dl_config.h"

static const uint32_t gLinePins[LINE_SENSOR_COUNT] = {
    LINE_SENSOR_LINE0_PIN, LINE_SENSOR_LINE1_PIN,
    LINE_SENSOR_LINE2_PIN, LINE_SENSOR_LINE3_PIN,
    LINE_SENSOR_LINE4_PIN, LINE_SENSOR_LINE5_PIN,
    LINE_SENSOR_LINE6_PIN, LINE_SENSOR_LINE7_PIN,
    LINE_SENSOR_LINE8_PIN, LINE_SENSOR_LINE9_PIN,
    LINE_SENSOR_LINE10_PIN, LINE_SENSOR_LINE11_PIN
};

void LineSensor_init(void)
{
    /* Pin mux and input mode are initialized by SYSCFG_DL_init(). */
}

uint16_t LineSensor_readRaw(void)
{
    uint32_t portValue = DL_GPIO_readPins(LINE_SENSOR_PORT, UINT32_MAX);
    uint16_t result = 0U;
    uint8_t i;

    for (i = 0U; i < LINE_SENSOR_COUNT; i++) {
        if ((portValue & gLinePins[i]) != 0U) {
            result |= (uint16_t) (1U << i);
        }
    }
    return result;
}

uint8_t LineSensor_read(uint8_t index)
{
    if (index >= LINE_SENSOR_COUNT) {
        return 0U;
    }
    return (DL_GPIO_readPins(LINE_SENSOR_PORT, gLinePins[index]) != 0U) ?
        1U : 0U;
}

uint8_t LineSensor_countBlack(void)
{
    uint16_t value = LineSensor_readRaw();
    uint8_t count = 0U;

    while (value != 0U) {
        count = (uint8_t) (count + (uint8_t) (value & 1U));
        value >>= 1U;
    }
    return count;
}
