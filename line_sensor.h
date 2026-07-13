#ifndef LINE_SENSOR_H_
#define LINE_SENSOR_H_

#include <stdint.h>

#define LINE_SENSOR_COUNT (12U)

void LineSensor_init(void);
uint16_t LineSensor_readRaw(void);
uint8_t LineSensor_read(uint8_t index);
uint8_t LineSensor_countBlack(void);

#endif
