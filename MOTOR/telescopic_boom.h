#ifndef __TELESCOPIC_BOOM_H
#define __TELESCOPIC_BOOM_H

#include "main.h"
#include "pid.h"
#include "usart.h"
#define  	telescopic_usart huart7//todo
#define 	telescopic_UART_INS UART7
#define  	telescopic_id    2//todo


#ifndef	ABS
#define ABS(x) ({ typeof(x) _x = (x); _x < 0 ? -_x : _x; })
#endif

#ifndef _LIMIT_MIN
#define _LIMIT_MIN(x, min) ((x) < (min) ? (min) : (x))
#endif

void Telescopic_Init(void);
void Telescopic_Send_Speed(int speed);
void Telescopic_Send_Position(int position);
void Telescopic_control_pid(int Difference_var);
void Telescopic_set_pid(float kp,float ki,float kd);
void Read_Telescopic_position(void);
void Telescopic_Stop(void);

#endif
