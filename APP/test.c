#include "test.h"
#include "hmi_task.h"
#include "tjc_usart_hmi.h"
#include "warehouse_app.h"
#include "action_control.h"
#include "motor.h"
#include "motor_control.h"
#include "imu_control.h"
#include "IMU.h"
#include "QR_code.h"
#include "circe.h"
#include "user.h"
#include "chassis_control_task.h"
#include "GO-M8010-6.h"
#include "catch.h"
#include "postion_control.h"
#include <math.h>

#define TEST_CHASSIS_SPEED_FINE  5.0f
#define TEST_CHASSIS_SPEED_LOW   80.0f

#define RING_CENTER_X            122
#define RING_CENTER_Y            133
#define RING_LOCATE_DEADZONE     5
#define RING_LOCATE_STABLE_COUNT 5
#define RING_LOCATE_TIMEOUT_MS   6000

#define RING_SWITCH_RAMP_TICKS   30
#define RING_SWITCH_10CM_TICKS   34
#define RING_SWITCH_20CM_TICKS   68

static int Test_AbsInt(int value)
{
	return (value < 0) ? -value : value;
}

static float Test_SignSpeed(int value, float speed)
{
	if(value > 0) return speed;
	if(value < 0) return -speed;
	return 0.0f;
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

void Vision_Parse_Test(void)
{
	VISION_TARGET_T target;
	uint32_t last_tick = 0;

	vTaskDelay(pdMS_TO_TICKS(500));

	HMI_InitScreen();
	HMI_SetSys("VISTEST", "NONE");
	HMI_SetVisionText("MATERIAL");
	HMI_SetChassisText("IDLE");
	HMI_SetArmText("IDLE");
	HMI_LogInfo("vision parse test");

	USART6_readdata_SeetZero();
	Vision_LED_On();
	vTaskDelay(pdMS_TO_TICKS(300));

	Vision_StartMaterial();

	while(1)
	{
		if((HAL_GetTick() - last_tick) >= 200)
		{
			last_tick = HAL_GetTick();

			HMI_LogInfo("mode=%d cnt=%d", vision_last_mode, vision_last_count);

			if(Vision_GetMaterialTarget(RED, &target))
			{
				HMI_LogInfo("M R %03d,%03d", target.x, target.y);
			}

			if(Vision_GetMaterialTarget(GREEN, &target))
			{
				HMI_LogInfo("M G %03d,%03d", target.x, target.y);
			}

			if(Vision_GetMaterialTarget(BLUE, &target))
			{
				HMI_LogInfo("M B %03d,%03d", target.x, target.y);
			}

			HMI_LogInfo("old %d dx=%d dy=%d",
			            Get_data_action_flag(),
			            Get_X_Change(),
			            Get_Y_Change());
		}

		vTaskDelay(pdMS_TO_TICKS(20));
	}
}

void Joint_Chassis_Feedback_Test(void)
{
	uint32_t start_tick;
	uint32_t last_log_tick = 0;

	vTaskDelay(pdMS_TO_TICKS(500));

	HMI_InitScreen();
	HMI_SetSys("JCTEST", "NONE");
	HMI_SetVisionText("IDLE");
	HMI_SetChassisText("IDLE");
	HMI_SetArmText("INIT");
	HMI_LogInfo("joint chassis test");

	Set_chassis_able(unable);
	Motor_setspeed(0, 0, 0);
	vTaskDelay(pdMS_TO_TICKS(200));

	HMI_SetArmText("M -30");
	HMI_LogInfo("m8010 to -30");
	M8010_SetAngle(-30);
	HMI_LogInfo("m8010 pos %lu", (unsigned long)M8010_ShowRealPostion());
	vTaskDelay(pdMS_TO_TICKS(300));

	HMI_SetArmText("M 0");
	HMI_LogInfo("m8010 to 0");
	M8010_SetAngle(0);
	HMI_LogInfo("m8010 pos %lu", (unsigned long)M8010_ShowRealPostion());
	vTaskDelay(pdMS_TO_TICKS(300));

	motor1.actual_angle = 0;
	motor2.actual_angle = 0;
	motor3.actual_angle = 0;
	motor4.actual_angle = 0;
	car.actual_y = 0;
	car.actual_x = 0;
	car.actual_w = 0;
	Motor_Rxdata_SetSero();

	HMI_SetArmText("DONE");
	HMI_SetChassisText("RUN");
	HMI_LogInfo("chassis run");

	start_tick = HAL_GetTick();
	while((HAL_GetTick() - start_tick) < 1200)
	{
		Motor_setspeed(20, 0, 0);
		Motor_Action_Calculate_actual(&car.actual_y, &car.actual_x, &car.actual_w);

		if((HAL_GetTick() - last_log_tick) >= 200)
		{
			last_log_tick = HAL_GetTick();
			HMI_LogInfo("m12 %d,%d", (int)motor1.actual_angle, (int)motor2.actual_angle);
			HMI_LogInfo("m34 %d,%d", (int)motor3.actual_angle, (int)motor4.actual_angle);
			HMI_LogInfo("car y%d x%d w%d", (int)car.actual_y, (int)car.actual_x, (int)car.actual_w);
		}

		vTaskDelay(pdMS_TO_TICKS(20));
	}

	Motor_setspeed(0, 0, 0);
	HMI_SetChassisText("STOP");
	HMI_LogInfo("chassis stop");

	while(1)
	{
		Motor_Action_Calculate_actual(&car.actual_y, &car.actual_x, &car.actual_w);
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}

static uint8_t Ring_LocateOne(uint8_t cls, char *name)
{
	VISION_TARGET_T target;
	uint32_t start_tick = HAL_GetTick();
	uint32_t last_log_tick = 0;
	uint8_t stable_count = 0;

	HMI_SetVisionText(name);
	HMI_LogInfo("loc %s", name);

	while((HAL_GetTick() - start_tick) < RING_LOCATE_TIMEOUT_MS)
	{
		int err_x;
		int err_y;
		float vx = 0.0f;
		float vy = 0.0f;

		if(Vision_GetRingTarget(cls, &target) == 0)
		{
			Motor_setspeed(0, 0, 0);
			stable_count = 0;

			if((HAL_GetTick() - last_log_tick) >= 300)
			{
				last_log_tick = HAL_GetTick();
				HMI_LogWarn("%s lost", name);
			}

			vTaskDelay(pdMS_TO_TICKS(20));
			continue;
		}

		err_x = RING_CENTER_X - (int)target.x;
		err_y = RING_CENTER_Y - (int)target.y;

		if(Test_AbsInt(err_x) <= RING_LOCATE_DEADZONE &&
		   Test_AbsInt(err_y) <= RING_LOCATE_DEADZONE)
		{
			Motor_setspeed(0, 0, 0);
			stable_count++;

			if(stable_count >= RING_LOCATE_STABLE_COUNT)
			{
				HMI_LogInfo("%s ok %03d,%03d", name, target.x, target.y);
				return 1;
			}
		}
		else
		{
			stable_count = 0;

			/* Camera Y error maps to chassis X; camera X error maps to chassis Y. */
			if(Test_AbsInt(err_y) > RING_LOCATE_DEADZONE)
			{
				vx = Test_SignSpeed(-err_y, TEST_CHASSIS_SPEED_FINE);
			}

			if(Test_AbsInt(err_x) > RING_LOCATE_DEADZONE)
			{
				vy = Test_SignSpeed(err_x, TEST_CHASSIS_SPEED_FINE);
			}

			Chassis_OpenLoop_SetSpeed(vx, vy, 0);
		}

		if((HAL_GetTick() - last_log_tick) >= 300)
		{
			last_log_tick = HAL_GetTick();
			HMI_LogInfo("%s e%d,%d", name, err_x, err_y);
		}

		vTaskDelay(pdMS_TO_TICKS(20));
	}

	Motor_setspeed(0, 0, 0);
	HMI_LogError("%s timeout", name);
	return 0;
}

static void Ring_SwitchY(float vy, uint16_t hold_ticks)
{
	HMI_SetChassisText("SWITCH");
	Chassis_MoveOnce(0, vy, 0, hold_ticks, RING_SWITCH_RAMP_TICKS);
	HMI_SetChassisText("STOP");
	vTaskDelay(pdMS_TO_TICKS(200));
}

void Ring_Location_Test(void)
{
	vTaskDelay(pdMS_TO_TICKS(500));

	HMI_InitScreen();
	HMI_SetSys("RING", "TEST");
	HMI_SetVisionText("INIT");
	HMI_SetChassisText("STOP");
	HMI_SetArmText("INIT");
	HMI_LogInfo("ring locate test");

	Set_chassis_able(unable);
	Motor_setspeed(0, 0, 0);
	vTaskDelay(pdMS_TO_TICKS(100));

	Imu_setZero();
	vTaskDelay(pdMS_TO_TICKS(200));

	USART6_readdata_SeetZero();
	Set_Circle_Center(RING_CENTER_X, RING_CENTER_Y);
	Vision_LED_On();
	Vision_StartRing();
	vTaskDelay(pdMS_TO_TICKS(300));

	claw_move_2(open);
	vTaskDelay(pdMS_TO_TICKS(200));
	M8010_SetAngle(PUT_AND_CATCH_ANGLE);
	vTaskDelay(pdMS_TO_TICKS(300));
	Z_SetHeight(60);
	vTaskDelay(pdMS_TO_TICKS(300));
	HMI_SetArmText("READY");

	while(1)
	{
		Ring_LocateOne(GREEN, "GREEN");
		Ring_SwitchY(TEST_CHASSIS_SPEED_LOW, RING_SWITCH_10CM_TICKS);

		Ring_LocateOne(BLUE, "BLUE");
		Ring_SwitchY(-TEST_CHASSIS_SPEED_LOW, RING_SWITCH_20CM_TICKS);

		Ring_LocateOne(RED, "RED");
		Ring_SwitchY(TEST_CHASSIS_SPEED_LOW, RING_SWITCH_10CM_TICKS);

		Motor_setspeed(0, 0, 0);
		HMI_SetVisionText("WAIT");
		HMI_LogInfo("round done");
		vTaskDelay(pdMS_TO_TICKS(2000));
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
