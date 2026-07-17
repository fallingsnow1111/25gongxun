#include "catch.h"
#include "circe.h"
#include "delay.h"
#include "GO-M8010-6.h"
#include "postion_control.h"
#include "telescopic_boom.h"
#include "tim.h"
#include <math.h>

/* 圆盘机、色环识别和色环区回收使用的夹爪开合参数。 */
void claw_move_1(int action)
{
	switch(action)
	{
		case open:
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 70);
			break;
		case close:
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 150);
			break;
		default:
			break;
	}
}

/* 从物料仓取料时使用的夹爪开合参数。 */
void claw_move_2(int action)
{
	switch(action)
	{
		case open:
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 115);
			break;
		case close:
			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 150);
			break;
		default:
			break;
	}
}

/*
 * 极坐标定位备用方案：关节角负责横向修正，伸缩臂负责纵向修正。
 * 当前完整流程不调用，保留给圆盘机极坐标抓取实验使用。
 */
void Catch_TELESCOPIC_Polar_coordinates(char color, float car_angle)
{
	const float angle_max = -140.0f;
	const float angle_min = -220.0f;
	const float delta_angle_max = 3.0f;
	int x_y_change[2];
	int color_flag = 0xFF;
	float s_r = 0.0f;
	float delta_angle = 0.0f;
	uint16_t count = 0;

	Set_Circle_Center(117, 115);
	x_y_change[0] = Get_X_Change_yuanpanji();
	x_y_change[1] = Get_Y_Change_yuanpanji();

	while(1)
	{
		send_NX(color);
		Delay_ms(6);
		if(count >= 1667)
			break;
		count++;

		x_y_change[0] = Get_X_Change_yuanpanji();
		x_y_change[1] = Get_Y_Change_yuanpanji();
		color_flag = Get_data_action_flag();
		if(color_flag != color)
			continue;

		if(x_y_change[0] != 0xFF && x_y_change[1] != 0xFF)
		{
			if(fabsf((float)x_y_change[0]) <= 2.0f &&
			   fabsf((float)x_y_change[1]) <= 8.0f)
			{
				for(char i = 0; i < 4; i++)
				{
					Delay_ms(6);
					Telescopic_Send_Speed(0);
				}
				break;
			}

			delta_angle = -(0.075f * x_y_change[0]) + 0.05f;
			if(fabsf(delta_angle) >= delta_angle_max)
				delta_angle = delta_angle >= 0.0f ? delta_angle_max : -delta_angle_max;

			if(fabsf((float)x_y_change[0]) < 8.0f ||
			   fabsf((float)x_y_change[1]) < 8.0f)
				delta_angle = delta_angle > 0.0f ? 0.15f : -0.15f;
			else if(fabsf((float)x_y_change[0]) <= 3.0f)
				delta_angle = 0.0f;

			car_angle += delta_angle;
			if(car_angle >= angle_max)
				car_angle = angle_max;
			else if(car_angle <= angle_min)
				car_angle = angle_min;

			s_r = (float)x_y_change[1];
			Telescopic_control_pid((int)s_r);
			M8010_SetAngle((int)car_angle);
		}
		else
		{
			Delay_ms(5);
			Telescopic_Send_Speed(0);
		}
	}
}
