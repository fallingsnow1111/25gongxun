#include "motor_control.h"
#include "motor.h"
#include "pid.h"
#include "imu_control.h"
#include <stdio.h>
#include "math.h"
#include "IMU.h"
#include "TIME.h"
#include "IMU.h"
#include "chassis_control_task.h"

unsigned char Calibration_Complete = 1;	
unsigned char Calibration_Complete_turn = 1;	
unsigned char W_Gray_openmv = 1;

#define ratio_of_pulse_distance_y  1.44928f       //脉冲数与距离的比值
#define ratio_of_pulse_distance_x  1.51515f
#define ratio_of_pulse_angle  (float)(280/83.50)//脉冲数与距离的比值89.59
#define MOVE_TO_TARGET_TIMEOUT_MS 10000U

float FMy_Abs(float temp)
{
	if(temp<0)return -temp;
	else return temp;
}

void motor_read_coordination_all(void)
{
	for (uint8_t i = 1; i <= 4; i++)
	{
		motor_read_coordination(i);
		Delay_ms(1);
	}
}

//Odometer is turned on by default
void Move_To_Target_area(float x,float y,float angle,int imu_able,MODE_POSITION mode)
{	
	uint32_t start_tick;

	car.imu_modeable=(ABLE_T)imu_able;
	car.Odometer_able=enable;
	Set_chassis_able(car.Odometer_able);
	//taskENTER_CRITICAL();
	car.target_x=(x*ratio_of_pulse_distance_x);
	car.target_y=(y*ratio_of_pulse_distance_y);
	car.target_w=angle;
	if(mode==Relative_Position)
	{
		Motor_SetZero();
		Imu_setZero();
	}
	//taskEXIT_CRITICAL();
	MOTOR_ACTIONFALG=Incomplete;
	start_tick = HAL_GetTick();
	while (MOTOR_ACTIONFALG!=finish)
	{
		if ((HAL_GetTick() - start_tick) > MOVE_TO_TARGET_TIMEOUT_MS)
		{
			break;
		}
		Delay_ms(2);
	}
	MOTOR_ACTIONFALG=Incomplete;
}

void Move_To_Target_Postion(float vy,float vx,float w,char mode)//旋转,厘米为单位
{
	uint32_t start_tick;

	// uint8_t _COUNT=150;
	// uint8_t count=0;
	motor_data_reset();
	inu_turn.ANGEL = w;///设置目标角度
	//Motor_setposition(ratio_of_pulse_distance_y*vy,ratio_of_pulse_distance_x*vx,ratio_of_pulse_angle*w,(MODE_POSITION)mode);
	MOTOR_ACTIONFALG=Incomplete;
	start_tick = HAL_GetTick();
	while(MOTOR_ACTIONFALG!=finish)
	{
		if ((HAL_GetTick() - start_tick) > MOVE_TO_TARGET_TIMEOUT_MS)
		{
			break;
		}
		Delay_ms(20);
		motor_read_stateflag(1);
		//vofa_printf("MOTOR_ACTIONFALG:%d\n",MOTOR_ACTIONFALG);
	}
	MOTOR_ACTIONFALG=Incomplete;
	Motor_Rxdata_SetSero();
}



