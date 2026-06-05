#include "QR_code.h"
#include "usart.h"
#include "tjc_usart_hmi.h"

int length,i = 0;
static uint8_t RxBuffer1[128] = {0};//接收数组

// 调试：原始字节捕获，方便 Keil 断点查看
volatile uint8_t  raw_buf[32] = {0};
volatile uint8_t  raw_idx = 0;
volatile uint32_t raw_total = 0;

volatile int TheNumber = 0;

struct COLOR_ID one;
struct COLOR_ID two;
int first_code = 0;
int	second_code = 0;

void U5_send(unsigned char data)
{
	unsigned short Usart5_Time=0;
	UART5->TDR=data;
	while((UART5->ISR&0X40)==0)//等待发送结束
	{
		Usart5_Time++;
		if(Usart5_Time>65534)
		{
			break;
		}
	}
    __HAL_UART_CLEAR_OREFLAG(&huart5);    
}

void QR_sense_init()
{
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_RXNE);
    one.firse = 0;
    one.second = 0;
    one.thrid = 0;
    two.firse = 0;
    two.second = 0;
    two.thrid = 0;
}


void UART5_IRQHandler(void)
{
	unsigned char res;
	if(UART5->ISR&(1<<5))
	{
		res=UART5->RDR;

		// 调试：捕获原始字节
		raw_buf[raw_idx & 0x1F] = res;
		raw_idx++;
		raw_total++;

		if(res == 0x48 && length == 0)
		{
			length = 1;
		}
		else if(res == 0x45 && length == 1)
		{
			length = 2;
		}
		else if(res == 0X41 && length == 2)
		{
			length = 3;
		}		
		else if(res == 0X44 && length == 3)
		{
			length = 4;
            i = 0;
		}
		else if(length == 4 && res != 0X0D)
		{
            RxBuffer1[i] = res;
            i++;
            if(i>=7)
            {
                one.firse = RxBuffer1[0] - 0x30;
                one.second = RxBuffer1[1] - 0x30;
                one.thrid = RxBuffer1[2] - 0x30;
                two.firse = RxBuffer1[4] - 0x30;
                two.second = RxBuffer1[5] - 0x30;
                two.thrid = RxBuffer1[6] - 0x30;
                first_code = one.firse*100+one.second*10+one.thrid;
                second_code = two.firse*100+two.second*10+two.thrid;
                length = 0;
                RxBuffer1[0] = 0;
                RxBuffer1[1] = 0;
                RxBuffer1[2] = 0;
                RxBuffer1[3] = 0;
                RxBuffer1[4] = 0;
                RxBuffer1[5] = 0;
                RxBuffer1[6] = 0;           
            }
		}
		else
		{
			length = 0;
			RxBuffer1[0] = 0;
			RxBuffer1[1] = 0;
            RxBuffer1[2] = 0;
            RxBuffer1[3] = 0;
            RxBuffer1[4] = 0;
            RxBuffer1[5] = 0;
            RxBuffer1[6] = 0;
		}
	}
  __HAL_UART_CLEAR_OREFLAG(&huart5);    
}

void HMI_SEND(void)
{		
		char str[30];
    	sprintf(str,"%d%d%d+%d%d%d",one.firse,one.second,one.thrid,two.firse,two.second,two.thrid);
		tjc_send_txt("t0", "txt", str);

}

#include <stdarg.h>
void vofa_printf(char* fmt, ...)
{
    uint8_t i, j;          // 定义计数器变量
    va_list ap;            // 可变参数列表
    va_start(ap, fmt);     // 初始化可变参数
    uint8_t USART_TX_BUF[100];
    // 将格式化后的字符串存入 USART_TX_BUF 缓冲区
    vsprintf((char*)USART_TX_BUF, fmt, ap);
    
    va_end(ap);            // 清理可变参数
    
    i = strlen((const char*)USART_TX_BUF);  // 计算待发送数据的长度
    
    // 循环发送每一个字节
    for (j = 0; j < i; j++) {
        U5_send(USART_TX_BUF[j]);  // 调用串口发送函数（例如 USART5）
    }
}

