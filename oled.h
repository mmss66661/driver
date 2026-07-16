#ifndef OLED_H_
#define OLED_H_

#include <stdbool.h>
#include <stdint.h>

bool OLED_Init(void);
bool OLED_IsPresent(void);
void OLED_Clear(void);
void OLED_ShowString(uint8_t x, uint8_t y, const char *text);
bool OLED_Refresh(void);
void OLED_Process(void);

#endif
