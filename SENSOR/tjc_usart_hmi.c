/**
使用注意事项:
    1.将 tjc_usart_hmi.c 和 tjc_usart_hmi.h 加入工程
    2.在需要使用的头文件中加入 #include "tjc_usart_hmi.h"
    3.使用前请将 HAL_UART_Transmit_IT() 替换成对应单片机的串口发送字节函数
*/

#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
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

#define HMI_TASK_STACK       384
#define HMI_TASK_PRIORITY    2
#define HMI_QUEUE_LEN        16
#define HMI_TEXT_MAX         40
#define HMI_LOG_LINE_COUNT   3

typedef enum
{
    HMI_MSG_TEXT = 0,
    HMI_MSG_LOG
} HMI_MSG_TYPE;

typedef struct
{
    HMI_MSG_TYPE type;
    char obj[3];
    char text[HMI_TEXT_MAX + 1];
} HMI_MSG;

TaskHandle_t hmi_task_Handle;
volatile uint32_t hmi_debug_drop_count = 0;

static QueueHandle_t hmi_queue = NULL;
static char tjc_print_line[HMI_TEXT_MAX + 1];
static uint8_t tjc_print_len = 0;
static char hmi_log_lines[HMI_LOG_LINE_COUNT][HMI_TEXT_MAX + 1];
static char hmi_log_objs[HMI_LOG_LINE_COUNT][3] = {"t6", "t7", "t8"};

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

static void HMI_CopyString(char* dst, uint8_t dst_size, const char* src)
{
    uint8_t i = 0;

    if(dst_size == 0) {
        return;
    }

    if(src == NULL) {
        dst[0] = '\0';
        return;
    }

    while(src[i] != '\0' && i < (dst_size - 1)) {
        dst[i] = src[i];
        i++;
    }
    dst[i] = '\0';
}

static void HMI_SendTextNow(char* obj, char* text)
{
    tjc_send_txt(obj, "txt", text);
}

static void HMI_LogRefreshNow(void)
{
    uint8_t i;

    for(i = 0; i < HMI_LOG_LINE_COUNT; i++) {
        HMI_SendTextNow(hmi_log_objs[i], hmi_log_lines[i]);
    }
}

static void HMI_LogPushNow(char* line)
{
    uint8_t i;

    for(i = 0; i < (HMI_LOG_LINE_COUNT - 1); i++) {
        strcpy(hmi_log_lines[i], hmi_log_lines[i + 1]);
    }

    HMI_CopyString(hmi_log_lines[HMI_LOG_LINE_COUNT - 1], sizeof(hmi_log_lines[0]), line);
    HMI_LogRefreshNow();
}

static uint8_t HMI_PostText(char* obj, char* text)
{
    HMI_MSG msg;

    msg.type = HMI_MSG_TEXT;
    HMI_CopyString(msg.obj, sizeof(msg.obj), obj);
    HMI_CopyString(msg.text, sizeof(msg.text), text);

    if(hmi_queue == NULL) {
        HMI_SendTextNow(msg.obj, msg.text);
        return 1;
    }

    if(xQueueSend(hmi_queue, &msg, 0) != pdPASS) {
        hmi_debug_drop_count++;
        return 0;
    }

    return 1;
}

static uint8_t HMI_PostLog(char* text)
{
    HMI_MSG msg;

    msg.type = HMI_MSG_LOG;
    msg.obj[0] = '\0';
    HMI_CopyString(msg.text, sizeof(msg.text), text);

    if(hmi_queue == NULL) {
        HMI_LogPushNow(msg.text);
        return 1;
    }

    if(xQueueSend(hmi_queue, &msg, 0) != pdPASS) {
        hmi_debug_drop_count++;
        return 0;
    }

    return 1;
}

static void HMI_FormatAndPostLog(char* level, char* fmt, va_list ap)
{
    char body[HMI_TEXT_MAX + 1];
    char line[HMI_TEXT_MAX + 1];

    vsnprintf(body, sizeof(body), fmt, ap);
    snprintf(line, sizeof(line), "[%s] %s", level, body);
    HMI_PostLog(line);
}

static void HMI_Task(void *pvParameters)
{
    HMI_MSG msg;

    (void)pvParameters;

    while(1) {
        if(xQueueReceive(hmi_queue, &msg, portMAX_DELAY) == pdPASS) {
            if(msg.type == HMI_MSG_LOG) {
                HMI_LogPushNow(msg.text);
            }
            else {
                HMI_SendTextNow(msg.obj, msg.text);
            }
        }
    }
}

void HMI_Task_Create(void)
{
    if(hmi_queue == NULL) {
        hmi_queue = xQueueCreate(HMI_QUEUE_LEN, sizeof(HMI_MSG));
    }

    if(hmi_queue == NULL) {
        hmi_debug_drop_count++;
        return;
    }

    if(hmi_task_Handle == NULL) {
        xTaskCreate(HMI_Task, "HMI_Task", HMI_TASK_STACK, NULL, HMI_TASK_PRIORITY, &hmi_task_Handle);
    }
}

void HMI_InitScreen(void)
{
    HMI_PostText("t0", "000+000");
    HMI_PostText("t1", "---sys status / logs---");
    HMI_PostText("t2", "SYS:BOOT ERR:NONE");
    HMI_PostText("t3", "VIS:----");
    HMI_PostText("t4", "CHS:----");
    HMI_PostText("t5", "ARM:----");
    HMI_PostText("t6", "");
    HMI_PostText("t7", "");
    HMI_PostText("t8", "");
}

void HMI_SetQR(int first, int second)
{
    char str[HMI_TEXT_MAX + 1];

    snprintf(str, sizeof(str), "%03d+%03d", first, second);
    HMI_PostText("t0", str);
}

void HMI_SetSys(char* mode, char* err)
{
    char str[HMI_TEXT_MAX + 1];

    snprintf(str, sizeof(str), "SYS:%s ERR:%s", mode, err);
    HMI_PostText("t2", str);
}

void HMI_SetVisionText(char* text)
{
    char str[HMI_TEXT_MAX + 1];

    snprintf(str, sizeof(str), "VIS:%s", text);
    HMI_PostText("t3", str);
}

void HMI_SetChassisText(char* text)
{
    char str[HMI_TEXT_MAX + 1];

    snprintf(str, sizeof(str), "CHS:%s", text);
    HMI_PostText("t4", str);
}

void HMI_SetArmText(char* text)
{
    char str[HMI_TEXT_MAX + 1];

    snprintf(str, sizeof(str), "ARM:%s", text);
    HMI_PostText("t5", str);
}

void HMI_LogInfo(char* fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    HMI_FormatAndPostLog("I", fmt, ap);
    va_end(ap);
}

void HMI_LogWarn(char* fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    HMI_FormatAndPostLog("W", fmt, ap);
    va_end(ap);
}

void HMI_LogError(char* fmt, ...)
{
    va_list ap;

    HMI_SetSys("ERROR", "CHECK");
    va_start(ap, fmt);
    HMI_FormatAndPostLog("E", fmt, ap);
    va_end(ap);
}

/********************************************************
功能名称:   TJC_PrintChar
日期:       2026.07.08
功能:       printf重定向到串口屏t2-t6日志区, 一行刷新一次
输入参数:   ch: printf输出的单个字符
返回值:     无
修改记录:
**********************************************************/
void TJC_PrintChar(char ch)
{
    if(ch == '\r') {
        return;
    }

    if(ch == '\n') {
        tjc_print_line[tjc_print_len] = '\0';
        HMI_PostLog(tjc_print_line);
        tjc_print_len = 0;
        return;
    }

    if(tjc_print_len < HMI_TEXT_MAX) {
        tjc_print_line[tjc_print_len++] = ch;
    }
    else {
        tjc_print_line[tjc_print_len] = '\0';
        HMI_PostLog(tjc_print_line);
        tjc_print_line[0] = ch;
        tjc_print_len = 1;
    }
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
