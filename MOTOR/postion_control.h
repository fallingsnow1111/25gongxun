#ifndef __POSTION_CONTROL_H
#define __POSTION_CONTROL_H

#include "sys.h"
#include "TIME.h"
#include "tim.h"

#define FINISH_MOVE     1
#define NO_FINISH_MOVE  0
void uart7WriteBuf(uint8_t *buf, uint8_t len);

extern struct POSTION Z_POSTION;
extern volatile uint8_t u7_debug_buf[8];
extern volatile uint16_t u7_debug_size;
struct POSTION
{
	int NOW;
    int TARGE;
    volatile int CHANGE;
    volatile int BIT;
};

void POSTION_init(void);
void postion_send(uint8_t id,int position);
void Z_SetHeight(int high);
void Y_SetLength(int length);
void Read_Z_position(void);
void uart7WriteBuf(uint8_t *buf, uint8_t len);
void u7RXdat_dispose(uint8_t* data);
void u7_speed_send(uint8_t id,int speed);
void Motor_Height_Ss_Stop(char id);
void MY_UART7_IRQHandler(uint16_t size);
void UART7_RxRestart(void);

uint32_t Get_Y_position(void);
void Read_Y_position(void);
void YZ_SetZero(char id);

#endif

