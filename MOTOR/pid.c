#include "pid.h"

struct PIDstruct Gyro_Pid;
struct PIDstruct pid_coordination;
struct PIDstruct Pid_Circle_Positioning;

//求绝对值
float _ABS(float a, float b) {
    return (a > b) ? (a - b) : (b - a);
}
int MAX_Limit(int a,float max)
{
	max=_ABS(max,0);
	if(a>0&&a>max)
	{
		a=max;
	}
	if(a<0&&a<(-max))
	{
		a=-max;
	}
	return a;
}

//PID初始化
void PID_Init(struct PIDstruct *pid,float kp,float ki,float kd,int outputmax,float outputmin)
{
	pid->kp=kp;
	pid->ki=ki;
	pid->kd=kd;
	pid->integralMax=100;
	pid->outputMax=outputmax;
	pid->outputMin=outputmin;
	pid->integral=0;
	pid->previousError=0;
}
//PID+积分分离初始化
void PID_Init_Integral_separation(struct PIDstructIntegralSeparation *pid,float kp,float ki,float kd,int outputmax,float outputmin,float IntegralSeparationThreshold)
{
		PID_Init(&pid->PID_basic,kp,ki,kd,outputmax,outputmin);
		pid->IntegralSeparationThreshold=IntegralSeparationThreshold;
}
////增量式PID
////带抗积分饱和
//void incremental_PID (struct I_pid_obj *motor, struct PID *pid)
//{
//	motor->last_bias = motor->bias;
//	motor->bias = motor->target - motor->encoder;
//	motor->integral += motor->bias;
//	if (pid->integralMax) {
//		if (motor->integral > pid->integralMax)
//			motor->integral = pid->integralMax;
//		if (motor->integral < -pid->integralMax)
//			motor->integral = -pid->integralMax;
//	}
//	motor->speed = pid->kp * motor->bias + pid->ki * motor->integral + pid->kd * (motor->bias - motor->last_bias);
//}

//位置式PID
float PID_Compute(struct PIDstruct *pid, float setpoint, float current_value) {
    // 计算当前误差
    float error = setpoint - current_value;
    // 计算比例项
    float proportional = pid->kp * error;
    // 积分项: 累积误差并限制积分值
    pid->integral += error;
    if (pid->integral > pid->integralMax) {
        pid->integral = pid->integralMax;
    } else if (pid->integral < -pid->integralMax) {
        pid->integral = -pid->integralMax;
    }
    float integral = pid->ki * pid->integral;
    // 微分项: 计算误差变化率
    float derivative = pid->kd * (error - pid->previousError);
    // 计算PID总输出
    float output = proportional + integral + derivative;
    // 更新历史误差
    pid->previousError = error;
    // 输出限幅: 限制PID控制器的输出范围
    if (output > pid->outputMax) {
        output = pid->outputMax;
    } else if (output < pid->outputMin) {
        output = pid->outputMin;
    }
    return output;
}
//位置式PID+积分分离
float PID_Compute_Integral_separation(PIDstructIntegralSeparation *pid, float setpoint, float current_value)
{
		static char C=0;//是否积分分离的标志位
	    // 计算当前误差
    float error = setpoint - current_value;
		//判断是否积分分离
		if(__fabs(error)<=pid->IntegralSeparationThreshold)
		{
			C=1;
		}else{C=0;}
    // 计算比例项
    float proportional = pid->PID_basic.kp * error;
    // 积分项: 累积误差并限制积分值
    pid->PID_basic.integral += error;
    if (pid->PID_basic.integral > pid->PID_basic.integralMax) {
        pid->PID_basic.integral = pid->PID_basic.integralMax;
    } else if (pid->PID_basic.integral < -pid->PID_basic.integralMax) {
        pid->PID_basic.integral = -pid->PID_basic.integralMax;
    }
    float integral = pid->PID_basic.ki * pid->PID_basic.integral;
    // 微分项: 计算误差变化率
    float derivative = pid->PID_basic.kd * (error - pid->PID_basic.previousError);
    // 计算PID总输出
    float output = proportional + C*integral + derivative;
    // 更新历史误差
    pid->PID_basic.previousError = error;
    // 输出限幅: 限制PID控制器的输出范围
    if (output > pid->PID_basic.outputMax) {
        output = pid->PID_basic.outputMax;
    } else if (output < pid->PID_basic.outputMin) {
        output = pid->PID_basic.outputMin;
    }
    return output;
}

