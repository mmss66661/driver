#include "ADC_power.h"

uint16_t GetADC_Power(void)
{
	uint16_t ADC_value;
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 30);
	if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
	{
		ADC_value = HAL_ADC_GetValue(&hadc1);
	}
	return ADC_value;
}






