#include "stm32f7xx.h"
#include "delay.h"

 uint16_t delaytime;

void Delay_Init(void)
{
	 HAL_TIM_Base_Start(&htim6);//¿ªÊ¼
	__HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE);
}

//void Delay_us(uint16_t nus)
//{
//	delaytime=0;
//	while(delaytime<=nus){}
//}

void Delay_ms(uint16_t nums)
{
	delaytime=0;
	while(delaytime<=nums){}
}
