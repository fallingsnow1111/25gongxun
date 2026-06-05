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
	PID_Init(&Gyro_Pid,2.6,0,0,150,-150);//对陀螺仪的PID初始化
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

// 角度归一化函数
float normalize_angle(float angle) {
    static int flag=0;//记录偏航角突变次数
	float Last_Yaw=inu_run.LAST_ANGEL;
	if(angle-Last_Yaw<-180.0f)//如果存在角度突变
	{
		flag++;
	}
	else if(angle-Last_Yaw>180.0f)
	{
		flag--;
	}
 
    inu_run.LAST_ANGEL = angle;

	if(inu_run.SPEED==0)//如果速度为0
	{
		inu_run.SPEED=1;
		flag=0;
	}

    return angle+ flag*360.0f;
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
    // 输入检查
    float current_val = normalize_angle(imu.yaw);  // 当前角度归一化
    float w_output = PID_Compute(&Gyro_Pid, tar_angle, current_val);  // PID 计算
    // 小输出时设置死区（防止电机抖动）
    if (__fabs(w_output)<2) 
	{
        w_output = (w_output > 0) ? 1.0f : -1.0f;
    }
    return -w_output;  // 反向控制
}
