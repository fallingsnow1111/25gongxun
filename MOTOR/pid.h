#ifndef __PID_H
#define __PID_H

#include "struct_typedef.h"
#include "main.h"

extern struct PIDstruct pid_coordination;
extern struct PIDstruct Pid_Circle_Positioning;

float _ABS(float a, float b);
int MAX_Limit(int a,float max);

void PID_Init(struct PIDstruct *pid,float kp,float ki,float kd,int outputmax,float outputmin);
void PID_Init_Integral_separation(struct PIDstructIntegralSeparation *pid,float kp,float ki,float kd,int outputmax,float outputmin,float IntegralSeparationThreshold);
void positional_PID(struct P_pid_obj *obj, struct PID *pid);
float PID_Compute(struct PIDstruct *pid, float setpoint, float current_value);
float PID_Compute_Integral_separation(PIDstructIntegralSeparation *pid, float setpoint, float current_value);

#endif
