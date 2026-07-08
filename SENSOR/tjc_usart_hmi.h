#ifndef __TJCUSARTHMI_H__
#define __TJCUSARTHMI_H__

#include <stdio.h>
#include "main.h"
/**
    陶晶驰串口屏幕通信
*/

#define TJC_UART huart5
#define TJC_UART_INS USART5
extern UART_HandleTypeDef huart5;

void TCJ_Init(void);
void intToStr(int num, char* str);
void tjc_send_string(char* str);
void uart_send_string(char* str);
void tjc_send_txt(char* objname, char* attribute, char* txt);
void tjc_send_val(char* objname, char* attribute, int val);
void tjc_send_nstring(char* str, unsigned char str_length);
void TJC_PrintChar(char ch);
void HMI_Task_Create(void);
void HMI_InitScreen(void);
void HMI_SetQR(int first, int second);
void HMI_SetSys(char* mode, char* err);
void HMI_SetVisionText(char* text);
void HMI_SetChassisText(char* text);
void HMI_SetArmText(char* text);
void HMI_LogInfo(char* fmt, ...);
void HMI_LogWarn(char* fmt, ...);
void HMI_LogError(char* fmt, ...);
void initRingBuffer(void);
void write1ByteToRingBuffer(uint8_t data);
void deleteRingBuffer(uint16_t size);
uint16_t getRingBufferLength(void);
uint8_t read1ByteFromRingBuffer(uint16_t position);

#define RINGBUFFER_LEN	(500)     // 环形缓冲区最大字节数 500

#define usize getRingBufferLength()
#define code_c() initRingBuffer()
#define udelete(x) deleteRingBuffer(x)
#define u(x) read1ByteFromRingBuffer(x)

extern uint8_t RxBuffer[1];
extern uint32_t msTicks;

#endif
