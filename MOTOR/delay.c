#include "stm32f7xx.h"
#include "delay.h"

volatile uint16_t delaytime;

void Delay_Init(void)
{
	 HAL_TIM_Base_Start(&htim6);
	__HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE);
}

void Delay_ms(uint16_t nums)
{
	uint16_t start = delaytime;
	while ((uint16_t)(delaytime - start) <= nums) {}
}
