#ifndef __POSTION_CONTROL_H
#define __POSTION_CONTROL_H

#include "sys.h"
#include "TIME.h"
#include "tim.h"

#define FINISH_MOVE     1
#define NO_FINISH_MOVE  0
void uart7WriteBuf(uint8_t *buf, uint8_t len);
static void U7_send(unsigned char data);

extern struct POSTION Z_POSTION;
struct POSTION
{
	  int NOW;
    int TARGE;
    int CHANGE;
    int BIT;
};

void POSTION_init(void);
void postion_send(uint8_t id,int position);
void Z_SetHeight(int high);
void Y_SetLength(int length);
void Read_Z_position(void);
static void U7_send(unsigned char data);
void uart7WriteBuf(uint8_t *buf, uint8_t len);
void u7RXdat_dispose(uint8_t* data);
void u7_speed_send(uint8_t id,int speed);
void Motor_Height_Ss_Stop(char id);
void MY_UART7_IRQHandler(uint8_t size);

uint32_t Get_Y_position(void);
void Read_Y_position(void);
void YZ_SetZero(char id);

#endif

