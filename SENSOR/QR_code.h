#ifndef __QR_code_H
#define __QR_code_H

#include "struct_typedef.h"
#include "string.h"

#define FRAME_HEADER 0x55             //帧头
#define CMD_SERVO_MOVE 0x03           //舵机移动指令
#define CMD_ACTION_GROUP_RUN 0x06     //运行动作组指令
#define CMD_ACTION_GROUP_STOP 0x07    //停止动作组指令
#define CMD_ACTION_GROUP_SPEED 0x0B   //设置动作组运行速度

void QR_sense_init(void);
void U5_send(unsigned char data);
void MY_UART5_IRQHandler(void);
extern int first_code,second_code ;
void vofa_printf(char* fmt, ...);


extern struct COLOR_ID one;
extern struct COLOR_ID two;
void HMI_SEND(void);

struct COLOR_ID
{
	int firse;
	int second;
  int thrid;
};
#endif

