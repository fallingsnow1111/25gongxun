#include "main.h"
#include "usart.h"
#include "QR_code.h"
#include "circe.h"
#include "GO-M8010-6.h"
#include "postion_control.h"
#include "motor.h"

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART3)
    {
		MY_UART3_IRQHandler();

    }
	if(huart->Instance==UART7)
	{
		MY_UART7_IRQHandler(Size);
	}	
}

