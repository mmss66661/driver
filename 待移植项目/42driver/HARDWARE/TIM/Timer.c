#include "Timer.h"

uint16_t Count = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim2)
	{
		if(++Count == 250)
		{
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
			Count = 0;
		}
	}
}






