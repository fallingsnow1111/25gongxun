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
		UART3_RxEventHandler(Size);

    }
	if(huart->Instance==UART7)
	{
		MY_UART7_IRQHandler(Size);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == UART7)
	{
		u7_debug_error_count++;
		u7_debug_last_error = huart->ErrorCode;
		UART7_RxRestart();
	}
	if(huart->Instance == USART3)
	{
		UART3_ErrorHandler();
	}
}

