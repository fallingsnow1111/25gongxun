#include "imu_control.h"
#include "motor_control.h"
#include "motor.h"
#include "IMU.h"
#include "pid.h"
#include "tjc_usart_hmi.h"

static int Compensating_corners=4;

/* 底盘航向 PID：先调 P 改善响应，再用 D 抑制转弯末端超调，I 当前不用。 */
#define GYRO_PID_KP          2.1f  /* 航向比例增益，增大后转向更快，但过大容易振荡。 */
#define GYRO_PID_KI          0.0f   /* 航向积分增益，当前保持为 0。 */
#define GYRO_PID_KD          0.5f   /* 航向微分增益，增大可抑制超调，但过大响应会变钝。 */
#define GYRO_PID_OUTPUT_MAX  120.0f /* 航向 PID 最大输出。 */
#define GYRO_PID_OUTPUT_MIN -120.0f /* 航向 PID 最小输出。 */

struct IMU_RUNDATA inu_run;
struct IMU_RUNDATA inu_turn;


void Gyro_Init(void)	//陀螺仪初始化
{
	PID_Init(&Gyro_Pid, GYRO_PID_KP, GYRO_PID_KI, GYRO_PID_KD,
			 GYRO_PID_OUTPUT_MAX, GYRO_PID_OUTPUT_MIN);
	IMU_Receive_Init();//开启串口2接收陀螺仪信息(环形DMA)
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

/* 补偿 IMU 累积漂移：用实测 yaw 与期望角度的差值修正目标角度。
   expected_current: 当前位置"应该"是什么角度 (上一段转弯的目标)
   target_angle:     要转到的目标角度
   返回 target_angle + drift_error，使机器人物理上转到正确的方向。 */
float Yaw_DriftCorrect(float expected_current, float target_angle)
{
#if YAW_DRIFT_COMP_ENABLE
    float current = normalize_angle(imu.yaw);
    float expected = normalize_angle(expected_current);
    float error = current - expected;

    while (error > 180.0f)  error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    return target_angle + error;
#else
    (void)expected_current;
    return target_angle;
#endif
}
