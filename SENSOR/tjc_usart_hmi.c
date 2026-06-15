/**
使用注意事项:
    1.将 tjc_usart_hmi.c 和 tjc_usart_hmi.h 加入工程
    2.在需要使用的头文件中加入 #include "tjc_usart_hmi.h"
    3.使用前请将 HAL_UART_Transmit_IT() 替换成对应单片机的串口发送字节函数
*/

#include "main.h"
#include <stdio.h>
#include "tjc_usart_hmi.h"
#include "QR_code.h"

typedef struct
{
    uint16_t Head;
    uint16_t Tail;
    uint16_t Length;
    uint8_t  Ring_data[RINGBUFFER_LEN];
}RingBuffer_t;

RingBuffer_t ringBuffer;	// 定义一个环形缓冲区
uint8_t RxBuffer[1];

/********************************************************
功能名称:   TCJ_RXData_Init()
日期:       2024.09.18
功能:       初始化串口接收
输入参数:   无
返回值:     无
修改记录:
**********************************************************/
void TCJ_RXData_Init(void)
{
	initRingBuffer();		// 初始化环形缓冲区
	HAL_UART_Receive_IT(&TJC_UART, RxBuffer, 1);	// 打开串口接收中断
}

/********************************************************
功能名称:   intToStr
日期:       2024.09.18
功能:       将整数转换为字符串
输入参数:   要转换的整数, 存放结果的字符串
返回值:     无
修改记录:
**********************************************************/
void intToStr(int num, char* str) {
    int i = 0;
    int isNegative = 0;

    // 处理负数
    if (num < 0) {
        isNegative = 1;
        num = -num;
    }

    // 提取每一位数字
    do {
        str[i++] = (num % 10) + '0';
        num /= 10;
    } while (num);

    // 如果是负数，添加负号
    if (isNegative) {
        str[i++] = '-';
    }

    // 添加字符串终止符
    str[i] = '\0';

    // 反转字符串
    int start = 0;
    int end = i - 1;
    while (start < end) {
        char temp = str[start];
        str[start] = str[end];
        str[end] = temp;
        start++;
        end--;
    }
    return ;
}

/********************************************************
功能名称:   uart_send_char
日期:       2024.09.18
功能:       串口发送单个字符
输入参数:   要发送的单个字符
返回值:     无
修改记录:
**********************************************************/
void uart_send_char(char ch)
{
	uint8_t ch2 = (uint8_t)ch;
    // 清除忙标志位，等待不忙时再发送下一个字符
	//while(__HAL_UART_GET_FLAG(&TJC_UART, UART_FLAG_TXE) == RESET);	// 等待发送为空
    // 发送单个字符
//	HAL_UART_Transmit_IT(&TJC_UART, &ch2, 1);
	U5_send(ch2);
	return;
}

void uart_send_string(char* str)
{
    // 当前字符非字符串结尾 并且 字符串首地址非空
    while(*str!=0&&str!=0)
    {
        // 发送字符串首地址中的字符，之后地址指针自增
        uart_send_char(*str++);
    }
	return;
}

/********************************************************
功能名称:   tjc_send_string
日期:       2024.09.18
功能:       陶晶驰屏发送字符串命令格式
输入参数:   要发送的字符串
返回值:     无
示例:       tjc_send_val("n0", "val", 100); 发送结果为 n0.val=100
修改记录:
**********************************************************/
void tjc_send_string(char* str)
{
    while(*str!=0&&str!=0)
    {
        uart_send_char(*str++);
    }
	uart_send_char(0xff);
	uart_send_char(0xff);
	uart_send_char(0xff);
	return;
}

/********************************************************
功能名称:   tjc_send_txt
日期:       2024.09.18
功能:       发送文本到串口屏
输入参数:   对象名, 属性名, 文本内容
返回值:     无
示例:       tjc_send_txt("t0", "txt", "ABC"); 发送结果为 t0.txt="ABC"
修改记录:
**********************************************************/
void tjc_send_txt(char* objname, char* attribute, char* txt)
{
    uart_send_string(objname);
    uart_send_char('.');
    uart_send_string(attribute);
    uart_send_string("=\"");
    uart_send_string(txt);
    uart_send_char('\"');
	uart_send_char(0xff);
	uart_send_char(0xff);
	uart_send_char(0xff);
	return;
}

/********************************************************
功能名称:   tjc_send_val
日期:       2024.09.18
功能:       发送数值到串口屏
输入参数:   对象名, 属性名, 数值
返回值:     无
修改记录:
**********************************************************/
void tjc_send_val(char* objname, char* attribute, int val)
{
	// 拼接字符串, 例如 n0.val=123
    uart_send_string(objname);
    uart_send_char('.');
    uart_send_string(attribute);
    uart_send_char('=');
    // C语言中val的取值范围是(-2147483648 ~ 2147483647), 最长为-2147483648, 加上结束符\0一共12个字符
    char txt[12]="";
    intToStr(val, txt);
    uart_send_string(txt);
	uart_send_char(0xff);
	uart_send_char(0xff);
	uart_send_char(0xff);
	return;
}

/********************************************************
功能名称:   tjc_send_nstring
日期:       2024.09.18
功能:       发送指定长度的字符串
输入参数:   要发送的字符串, 字符串长度
返回值:     无
修改记录:
**********************************************************/
void tjc_send_nstring(char* str, unsigned char str_length)
{
    for (int var = 0; var < str_length; ++var)
    {
        uart_send_char(*str++);
    }
	uart_send_char(0xff);
	uart_send_char(0xff);
	uart_send_char(0xff);
	return;
}

// 指定数据格式和接收端，接收目标
/********************************************************
功能名称:   HAL_UART_RxCpltCallback
日期:       2022.10.08
功能:       串口接收中断, 将接收到的数据写入环形缓冲区
输入参数:
返回值:     void
修改记录:
**********************************************************/
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart->Instance == TJC_UART.Instance)	// 判断是哪个串口触发的中断
//	{
//		write1ByteToRingBuffer(RxBuffer[0]);
//		HAL_UART_Receive_IT(&TJC_UART,RxBuffer,1);		// 重新使能串口接收中断
//	}
//	return;
//}

/********************************************************
功能名称:   initRingBuffer
日期:       2022.10.08
功能:       初始化环形缓冲区
输入参数:
返回值:     void
修改记录:
**********************************************************/
void initRingBuffer(void)
{
	// 初始化缓冲区信息
	ringBuffer.Head = 0;
	ringBuffer.Tail = 0;
	ringBuffer.Length = 0;
	return;
}

/********************************************************
功能名称:   write1ByteToRingBuffer
日期:       2022.10.08
功能:       向环形缓冲区写入数据
输入参数:   要写入的1字节数据
返回值:     void
修改记录:
**********************************************************/
void write1ByteToRingBuffer(uint8_t data)
{
	if(ringBuffer.Length >= RINGBUFFER_LEN) // 判断缓冲区是否已满
	{
	return ;
	}
	ringBuffer.Ring_data[ringBuffer.Tail]=data;
	ringBuffer.Tail = (ringBuffer.Tail+1)%RINGBUFFER_LEN;// 防止越界访问
	ringBuffer.Length++;
	return ;
}

/********************************************************
功能名称:   deleteRingBuffer
作者:
日期:       2022.10.08
功能:       删除串口缓冲区指定长度的数据
输入参数:   要删除的长度
返回值:     void
修改记录:
**********************************************************/
void deleteRingBuffer(uint16_t size)
{
	if(size >= ringBuffer.Length)
	{
	    initRingBuffer();
	    return;
	}
	for(int i = 0; i < size; i++)
	{
		ringBuffer.Head = (ringBuffer.Head+1)%RINGBUFFER_LEN;// 防止越界访问
		ringBuffer.Length--;
		return;
	}
}

/********************************************************
功能名称:   read1ByteFromRingBuffer
作者:
日期:       2022.10.08
功能:       从串口缓冲区读取1字节数据
输入参数:   position: 读取的位置
返回值:     对应位置的数据(1字节)
修改记录:
**********************************************************/
uint8_t read1ByteFromRingBuffer(uint16_t position)
{
	uint16_t realPosition = (ringBuffer.Head + position) % RINGBUFFER_LEN;
	return ringBuffer.Ring_data[realPosition];
}

/********************************************************
功能名称:   getRingBufferLength
作者:
日期:       2022.10.08
功能:       获取当前缓冲区中的数据量
输入参数:
返回值:     当前缓冲区中的数据量
修改记录:
**********************************************************/
uint16_t getRingBufferLength()
{
	return ringBuffer.Length;
}

/********************************************************
功能名称:   isRingBufferOverflow
作者:
日期:       2022.10.08
功能:       判断环形缓冲区是否溢出
输入参数:
返回值:     0: 环形缓冲区已满, 1: 环形缓冲区未满
修改记录:
**********************************************************/
uint8_t isRingBufferOverflow()
{
	return ringBuffer.Length < RINGBUFFER_LEN;
}
