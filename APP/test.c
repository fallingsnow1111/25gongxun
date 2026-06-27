#include "test.h"
#include "tjc_usart_hmi.h"
#include "warehouse_app.h"
#include "action_control.h"
#include "motor.h"
#include "motor_control.h"
#include "imu_control.h"
#include "IMU.h"
#include "QR_code.h"
#include "user.h"
#include "chassis_control_task.h"
#include <math.h>

static void Chassis_Test_Stop(uint32_t stop_time_ms)
{
	Motor_setspeed(0, 0, 0);
	vTaskDelay(pdMS_TO_TICKS(stop_time_ms));
}

void Chassis_Test_Run_With_Imu(float vy, float vx, float target_angle, uint32_t run_time_ms)
{
	uint32_t start_tick = HAL_GetTick();

	Imu_setZero();
	vTaskDelay(pdMS_TO_TICKS(100));

	while ((HAL_GetTick() - start_tick) < run_time_ms)
	{
		float vw = Direction_Calibration_turn(target_angle);
		Motor_setspeed(vy, vx, vw);
		vTaskDelay(pdMS_TO_TICKS(20));
	}

	Chassis_Test_Stop(400);
}

void Chassis_Test_Rotate_With_Imu(float target_angle, uint32_t timeout_ms)
{
	uint32_t start_tick = HAL_GetTick();

	Imu_setZero();
	vTaskDelay(pdMS_TO_TICKS(100));

	while ((HAL_GetTick() - start_tick) < timeout_ms)
	{
		float current_angle = normalize_angle(imu.yaw);
		float angle_error = getAngleZ(current_angle, target_angle);

		if (fabsf(angle_error) <= 2.0f)
		{
			break;
		}

		Motor_setspeed(0, 0, Direction_Calibration_turn(target_angle));
		vTaskDelay(pdMS_TO_TICKS(20));
	}

	Chassis_Test_Stop(500);
}

void User_function_final(void)
{
	vTaskDelay(pdMS_TO_TICKS(10));
	Set_chassis_able(enable);
	Move_To_Target_area(-110,1400,0,enable,Relative_Position);
	while(first_code == 0 && second_code == 0)
	{
		Move_To_Target_area(0,0,0,enable,Relative_Position);
	}
	Init_Warehouse(1);
	HMI_SEND();

	Move_To_Target_area(0,-400,0,enable,Relative_Position);
	Move_To_Target_area(-1810,0,0,enable,Relative_Position);
	Move_To_Target_area(0,0,180,enable,Relative_Position);

	Set_chassis_able(unable);
	Set_chassis_able(enable);

	Move_To_Target_area(-30,-850,0,enable,Relative_Position);
	Move_To_Target_area(-850,0,0,enable,Relative_Position);
	action_set_in_user(CIRCLE_ACTION);
	Move_To_Target_area(0,0,-90,enable,Relative_Position);

	Set_chassis_able(unable);
	Set_chassis_able(enable);

	Move_To_Target_area(-1790,0,0,enable,Relative_Position);
	Move_To_Target_area(0,0,180,enable,Relative_Position);

	Init_Warehouse(2);
	Move_To_Target_area(0,-900,0,enable,Relative_Position);
	Move_To_Target_area(-950,0,0,enable,Relative_Position);
	Move_To_Target_area(0,0,-90,enable,Relative_Position);

	Set_chassis_able(unable);
	Set_chassis_able(enable);

	Move_To_Target_area(-30,-850,0,enable,Relative_Position);
	Move_To_Target_area(-900,0,0,enable,Relative_Position);
	Move_To_Target_area(0,0,-90,enable,Relative_Position);

	Set_chassis_able(unable);
	Set_chassis_able(enable);

	Move_To_Target_area(-1790,0,0,enable,Relative_Position);
	Move_To_Target_area(0,0,180,enable,Relative_Position);

	Move_To_Target_area(0,1100,0,enable,Relative_Position);
	Move_To_Target_area(70,0,0,enable,Relative_Position);
}

void QR_Code_Test(void)
{
	vTaskDelay(pdMS_TO_TICKS(500));

	tjc_send_txt("t0", "txt", "WAIT QR...");
	vTaskDelay(pdMS_TO_TICKS(200));

	while (1)
	{
		if (first_code != 0 || second_code != 0)
		{
			HMI_SEND();
			first_code = 0;
			second_code = 0;
		}

		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

void Motor_Periodic_Feedback_Test(void)
{
	Set_chassis_able(unable);
	Motor_TimedReturn_Stop();
	vTaskDelay(pdMS_TO_TICKS(30));
	Motor_setspeed(0, 0, 0);
	vTaskDelay(pdMS_TO_TICKS(50));

	Motor_Rxdata_SetSero();
	Motor_FeedbackState_Reset();
	Motor_TimedReturn_Init();

	while (1)
	{
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

void Route_Test(void)
{
	Set_chassis_able(enable);
	vTaskDelay(pdMS_TO_TICKS(500));

	Move_To_Target_area(140, 0, 0, enable, Relative_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(0, 1600, 0, enable, Relative_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(0, -500, 0, enable, Relative_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(0, 0, 90, enable, Relative_Position);
	vTaskDelay(pdMS_TO_TICKS(200));

	Move_To_Target_area(0, 1750, 0, enable, Relative_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(0, 0, 90, enable, Relative_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(0, -750, 0, enable, Relative_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(0, 0, -90, enable, Relative_Position);
	vTaskDelay(pdMS_TO_TICKS(200));

	Move_To_Target_area(0, -800, 0, enable, Relative_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(0, 0, 90, enable, Relative_Position);
	vTaskDelay(pdMS_TO_TICKS(200));

	Move_To_Target_area(0, 1650, 0, enable, Relative_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(0, 0, 90, enable, Relative_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(0, 900, 0, enable, Relative_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(-150, 100, 0, enable, Relative_Position);
}

static uint8_t App_Test_Scan_QR(float base_x, float base_y)
{
	first_code = 0;
	second_code = 0;

	const float adj[][2] = {{0,0},{-40,0},{-40, 20}, {-40, -20}};
	const uint8_t n = 4;
	const uint32_t wait_ms = 1000;

	for(uint8_t i = 0; i < n; i++)
	{
		Move_To_Target_area(base_x + adj[i][0], base_y + adj[i][1], 0, enable, Absolute_Position);
		uint32_t t = HAL_GetTick();
		while ((first_code == 0 && second_code == 0) && (HAL_GetTick() - t) < wait_ms)
		{
			vTaskDelay(pdMS_TO_TICKS(50));
		}

		if(first_code != 0 || second_code != 0)
		{
			HMI_SEND();
			vTaskDelay(pdMS_TO_TICKS(300));
			return 1;
		}
	}

	return 0;
}

void Route_Test_ABS(void)
{
	Set_chassis_able(enable);
	vTaskDelay(pdMS_TO_TICKS(500));

	Move_To_Target_area(0, 0, 0, enable, Relative_Position);
	vTaskDelay(pdMS_TO_TICKS(300));

	Move_To_Target_area(140, 0, 0, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));

	App_Test_Scan_QR(150, 750);

	Move_To_Target_area(140, 1600, 0, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(140, 1100, 0, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(140, 1100, 90, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));

	Move_To_Target_area(140, 2850, 90, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(140, 2850, 180, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(140, 2050, 180, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(140, 2050, 90, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));

	Move_To_Target_area(140, 1250, 90, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(140, 1250, 180, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));

	Move_To_Target_area(140, 2950, 180, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(140, 2950, 270, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(140, 3900, 270, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(-85, 4020, 270, enable, Absolute_Position);
}
