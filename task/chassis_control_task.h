#ifndef __CHASSIS_CONTROL_TASK_H
#define __CHASSIS_CONTROL_TASK_H

#include "main.h"
#include "cmsis_os.h"

void Get_Chassis_c_output(float *x_output, float *y_output, float *w_output);
void Chassis_Control_Task_Create(void);
void  Set_chassis_able(ABLE_T Odometer);
void chassis_control_init(void);
void Obimeter_SetZero(void);

#endif

