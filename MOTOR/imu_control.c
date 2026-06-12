#include "imu_control.h"
#include "motor_control.h"
#include "motor.h"
#include "IMU.h"
#include "pid.h"
#include "tjc_usart_hmi.h"

#define usmart
static int Compensating_corners=4;

struct IMU_RUNDATA inu_run;
struct IMU_RUNDATA inu_turn;


void Gyro_Init(void)	//陀螺仪初始化
{
	PID_Init(&Gyro_Pid, 2.6, 0.0, 0.5, 150, -150);// Kd=0.5 抑制超调
	imu_receive_init();//开启串口2接收陀螺仪信息
}

float getAngleZ(float yaw,float my_angel) //获取相对角度
{
	float temp;
	temp = yaw - my_angel;
	if(yaw != my_angel)
	{
		if(temp > 180)
			temp -= 360;
		else if(temp<-180)
			temp += 360;
	}
	return temp;
}

float getAngleZ_avg(float my_angel)
{
	float temp;
	temp = getAngleZ(imu.yaw,my_angel);
	return temp;
}

// 角度归一化函数（无状态版本，避免多调用方共享 static flag）
float normalize_angle(float angle) {
    while (angle > 180.0f)  angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

// 偏航校准函数，用于保持机器人或无人机的偏航角度稳定
void Direction_Calibration(int target_angle)
{
	char str[20];
	float current_val=normalize_angle(imu.yaw);float w_output;
		if(_ABS(FMy_Abs(current_val),FMy_Abs(target_angle))>0.6f)
		{
			w_output = PID_Compute(&Gyro_Pid, target_angle, current_val); // PID 计算;
		    Motor_setspeed(0,0 ,-w_output);
		}
		else{
			Motor_setspeed(0,0,0);
			Delay_ms(6);
		}
}

 float Direction_Calibration_turn(float tar_angle) {
    float current_val = normalize_angle(imu.yaw);
    // 将目标角旋转到离当前角最近的等效角（走最短路径）
    float tar = tar_angle;
    while (tar - current_val > 180.0f)  tar -= 360.0f;
    while (tar - current_val < -180.0f) tar += 360.0f;
    float w_output = PID_Compute(&Gyro_Pid, tar, current_val);
    return -w_output;
}
