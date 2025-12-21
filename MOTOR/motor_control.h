#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "main.h"


extern unsigned char Calibration_Complete;	
extern unsigned char Calibration_Complete_turn;	
extern unsigned char W_Gray_openmv ;

float FMy_Abs(float temp);
void Move_To_Target_area(float x,float y,float angle,int imu_able,MODE_POSITION mode);
void Move_To_Target_Postion(float vy,float vx,float w,char mode);
void motor_read_coordination_all(void);

#endif
