#include "test.h"
#include "task_flow.h"
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
#include "GO-M8010-6.h"
#include "catch.h"
#include "postion_control.h"
#include "tim.h"
#include <math.h>
#include <stdio.h>

/* 底盘速度档位：修改速度后，所有对应距离都需要重新标定。 */

/* 加入角速度环前的转向稳态基线测试参数。 */
#define TURN_BASELINE_TIMEOUT_MS       4000U  /* 单次转向最大允许时间。 */
#define TURN_BASELINE_SETTLE_MS         300U  /* 到位停车后等待机械惯性衰减。 */
#define TURN_BASELINE_SAMPLE_MS        1000U  /* 每个目标角度的稳态采样时长。 */
#define TURN_BASELINE_SAMPLE_PERIOD_MS   20U  /* 稳态采样周期。 */

/* 电机旋转速度指令与车身实际角速度的映射测试参数。 */
#define TURN_RATE_RAMP_STEPS             20U  /* 升降速分段数。 */
#define TURN_RATE_RAMP_PERIOD_MS           5U  /* 每个升降速分段的持续时间。 */
#define TURN_RATE_WARMUP_MS              300U  /* 达到目标RPM后的稳定等待时间。 */
#define TURN_RATE_SAMPLE_MS             1000U  /* 每档角速度采样时间。 */
#define TURN_RATE_SAMPLE_PERIOD_MS        20U  /* 角速度采样周期。 */
#define TURN_RATE_PHOTO_WAIT_MS         2000U  /* 每档停车并显示结果后的拍照时间。 */

/* 粗加工区色环定位参数。 */
#define RING_CENTER_X            122     /* 色环在画面中的目标中心 X 坐标。 */
#define RING_CENTER_Y            133     /* 色环在画面中的目标中心 Y 坐标。 */

/* 色环切换参数。 */
#define RING_SWITCH_SPEED         50.0f  /* 色环之间固定移动的速度。 */
#define RING_SWITCH_DISTANCE_CM   15.0f  /* 相邻两个色环的中心距离，单位 cm。 */

/*
 * 函数简介：底盘连续原地转向误差测试。
 * 测试流程：依次转到-90°、-180°、90°和0°，停车500ms后读取角度，打印后等待1秒。
 * 输出内容：目标角度、最终角度、静态误差和单次转向耗时。
 */
void Chassis_Turn_Error_Test(void)
{
	static const float target_angle[] = {-90.0f, -180.0f, 90.0f, 0.0f};
	float final_yaw;
	float final_error;
	uint32_t start_tick;
	uint32_t turn_time;

	vTaskDelay(pdMS_TO_TICKS(500));
	HMI_InitScreen();
	HMI_LogInfo("turn error test");

	Motor_setspeed(0, 0, 0);
	vTaskDelay(pdMS_TO_TICKS(100));
	Imu_setZero();
	vTaskDelay(pdMS_TO_TICKS(200));

	for(uint8_t i = 0; i < 4; i++)
	{
		start_tick = HAL_GetTick();
		Chassis_TurnToAngle(target_angle[i], 4000U);
		turn_time = HAL_GetTick() - start_tick;

		/* 停车500ms后读取静态误差，避免把车身惯性计入到位误差。 */
		vTaskDelay(pdMS_TO_TICKS(500));
		final_yaw = normalize_angle(imu.yaw);
		final_error = getAngleZ(final_yaw, target_angle[i]);

		HMI_LogInfo("T%.1f Y%.2f E%.2f %lums",
					target_angle[i], final_yaw, final_error,
					(unsigned long)turn_time);
		vTaskDelay(pdMS_TO_TICKS(1000));
	}

	Motor_setspeed(0, 0, 0);
	HMI_LogInfo("turn test done");
}

/*
 * 函数简介：加入角速度环前的转向稳态基线测试。
 * 测试流程：依次执行0°到-90°、0°、90°、0°的双向转向；到位后采样1秒。
 * 输出内容：平均角度误差Eavg、最大角度误差Emax、平均和最大残余角速度。
 * 说明：该测试只评价停车后的稳态性能，不反映转向过程中的动态角速度响应。
 */
void Chassis_AngularRate_Baseline_Test(void)
{
	static const float target_angle[] = {-90.0f, 0.0f, 90.0f, 0.0f};
	char chassis_text[32];

	vTaskDelay(pdMS_TO_TICKS(500));
	HMI_InitScreen();
	HMI_SetSys("TURN", "BASE");
	HMI_SetVisionText("GYRO BASE");
	HMI_LogInfo("gyro baseline start");

	Motor_setspeed(0, 0, 0);
	vTaskDelay(pdMS_TO_TICKS(100));
	Imu_setZero();
	vTaskDelay(pdMS_TO_TICKS(200));

	for(uint8_t i = 0; i < 4; i++)
	{
		uint32_t turn_start = HAL_GetTick();
		uint32_t sample_start;
		uint32_t turn_time;
		uint16_t sample_count = 0;
		float error_sum = 0.0f;
		float error_abs_max = 0.0f;
		float rate_abs_sum = 0.0f;
		float rate_abs_max = 0.0f;
		float final_yaw;
		float error_average;
		float rate_abs_average;

		Chassis_TurnToAngle(target_angle[i], TURN_BASELINE_TIMEOUT_MS);
		turn_time = HAL_GetTick() - turn_start;
		vTaskDelay(pdMS_TO_TICKS(TURN_BASELINE_SETTLE_MS));

		sample_start = HAL_GetTick();
		while((HAL_GetTick() - sample_start) < TURN_BASELINE_SAMPLE_MS)
		{
			float yaw = normalize_angle(imu.yaw);
			float error = getAngleZ(yaw, target_angle[i]);
			float error_abs = fabsf(error);
			float rate_abs = fabsf(imu.angular_rate);

			error_sum += error;
			rate_abs_sum += rate_abs;
			if(error_abs > error_abs_max)
				error_abs_max = error_abs;
			if(rate_abs > rate_abs_max)
				rate_abs_max = rate_abs;
			sample_count++;

			if((sample_count % 5U) == 0U)
			{
				snprintf(chassis_text, sizeof(chassis_text),
						 "T%.0f Y%.2f W%.2f",
						 target_angle[i], yaw, imu.angular_rate);
				HMI_SetChassisText(chassis_text);
			}

			vTaskDelay(pdMS_TO_TICKS(TURN_BASELINE_SAMPLE_PERIOD_MS));
		}

		if(sample_count == 0U)
			continue;

		final_yaw = normalize_angle(imu.yaw);
		error_average = error_sum / (float)sample_count;
		rate_abs_average = rate_abs_sum / (float)sample_count;

		HMI_LogInfo("T%.0f Y%.2f %lums", target_angle[i], final_yaw,
					(unsigned long)turn_time);
		HMI_LogInfo("Eavg%.2f Emax%.2f", error_average, error_abs_max);
		HMI_LogInfo("Wavg%.2f Wmax%.2f", rate_abs_average, rate_abs_max);
		vTaskDelay(pdMS_TO_TICKS(1000));
	}

	Motor_setspeed(0, 0, 0);
	HMI_SetChassisText("BASE DONE");
	HMI_LogInfo("gyro baseline done");
}

/*
 * 函数简介：测试专用的旋转速度平滑切换。
 * 参数说明：start_speed为起始RPM，end_speed为目标RPM。
 * 说明：仅用于RPM-角速度映射测试，避免速度指令突变引起车身冲击。
 */
static void Test_SetTurnSpeedSmooth(float start_speed, float end_speed)
{
	for(uint8_t step = 1; step <= TURN_RATE_RAMP_STEPS; step++)
	{
		float ratio = (float)step / (float)TURN_RATE_RAMP_STEPS;
		float speed = start_speed + (end_speed - start_speed) * ratio;

		Motor_setspeed(0, 0, speed);
		vTaskDelay(pdMS_TO_TICKS(TURN_RATE_RAMP_PERIOD_MS));
	}
}

/*
 * 函数简介：标定电机旋转RPM指令与车身实际角速度的映射关系。
 * 测试流程：补测正负60、80、100RPM，稳定后采样1秒，再停车显示2秒。
 * 输出内容：IMU平均角速度Wavg、航向变化反算角速度Yrate，以及角速度最小/最大值。
 * 用途：确定角速度内环的对象增益、正负方向一致性和可执行最小速度。
 */
void Chassis_TurnRate_Map_Test(void)
{
	static const float command_rpm[] = {
		60.0f, 80.0f, 100.0f,
		-60.0f, -80.0f, -100.0f
	};
	char chassis_text[32];

	vTaskDelay(pdMS_TO_TICKS(500));
	HMI_InitScreen();
	HMI_SetSys("TURN", "RATE MAP");
	HMI_SetVisionText("RPM TO DPS");
	HMI_LogInfo("turn rate map start");

	Motor_setspeed(0, 0, 0);
	vTaskDelay(pdMS_TO_TICKS(200));
	Imu_setZero();
	vTaskDelay(pdMS_TO_TICKS(200));

	for(uint8_t i = 0; i < (sizeof(command_rpm) / sizeof(command_rpm[0])); i++)
	{
		uint32_t sample_start;
		uint32_t sample_elapsed;
		uint16_t sample_count = 0;
		float rate_sum = 0.0f;
		float rate_min = 10000.0f;
		float rate_max = -10000.0f;
		float sample_start_yaw;
		float sample_end_yaw;
		float rate_average;
		float yaw_rate_average;

		HMI_LogInfo("cmd %.0f rpm", command_rpm[i]);
		Test_SetTurnSpeedSmooth(0.0f, command_rpm[i]);
		vTaskDelay(pdMS_TO_TICKS(TURN_RATE_WARMUP_MS));

		sample_start_yaw = normalize_angle(imu.yaw);
		sample_start = HAL_GetTick();
		while((HAL_GetTick() - sample_start) < TURN_RATE_SAMPLE_MS)
		{
			float rate = imu.angular_rate;

			rate_sum += rate;
			if(rate < rate_min)
				rate_min = rate;
			if(rate > rate_max)
				rate_max = rate;
			sample_count++;

			if((sample_count % 5U) == 0U)
			{
				snprintf(chassis_text, sizeof(chassis_text),
						 "RPM%.0f W%.2f", command_rpm[i], rate);
				HMI_SetChassisText(chassis_text);
			}

			vTaskDelay(pdMS_TO_TICKS(TURN_RATE_SAMPLE_PERIOD_MS));
		}

		sample_elapsed = HAL_GetTick() - sample_start;
		sample_end_yaw = normalize_angle(imu.yaw);
		Test_SetTurnSpeedSmooth(command_rpm[i], 0.0f);
		Motor_setspeed(0, 0, 0);

		if(sample_count == 0U || sample_elapsed == 0U)
			continue;

		rate_average = rate_sum / (float)sample_count;
		yaw_rate_average = getAngleZ(sample_end_yaw, sample_start_yaw) *
						   1000.0f / (float)sample_elapsed;

		HMI_LogInfo("C%.0f Wavg%.2f", command_rpm[i], rate_average);
		HMI_LogInfo("Yrate%.2f %lums", yaw_rate_average,
					(unsigned long)sample_elapsed);
		HMI_LogInfo("Wmin%.2f max%.2f", rate_min, rate_max);
		vTaskDelay(pdMS_TO_TICKS(TURN_RATE_PHOTO_WAIT_MS));
	}

	Motor_setspeed(0, 0, 0);
	HMI_SetChassisText("RATE MAP DONE");
	HMI_LogInfo("turn rate map done");
}

#define TURN_LOW_RATE_WARMUP_MS         1000U
#define TURN_LOW_RATE_SAMPLE_MS        10000U
#define TURN_LOW_RATE_SAMPLE_PERIOD_MS    20U
#define TURN_LOW_RATE_PHOTO_WAIT_MS     2000U

/*
 * 依次测试正反向0.1~1.0RPM，每档采样10秒。
 * 屏幕输出指令RPM、IMU平均角速度、Yaw反算角速度及角速度范围。
 */
void Chassis_LowSpeed_Linearity_Test(void)
{
	static const float command_rpm[] = {
		 0.1f, -0.1f,  0.2f, -0.2f,  0.3f, -0.3f,
		 0.5f, -0.5f,  0.7f, -0.7f,  1.0f, -1.0f
	};
	char chassis_text[32];

	vTaskDelay(pdMS_TO_TICKS(500));
	HMI_InitScreen();
	HMI_SetSys("TURN", "LOW RPM");
	HMI_SetVisionText("0.1RPM TEST");
	HMI_LogInfo("scale10 cfg=%d", motor_speed_scale10_ready);

	Motor_setspeed_fine(0, 0, 0);
	vTaskDelay(pdMS_TO_TICKS(200));
	Imu_setZero();
	vTaskDelay(pdMS_TO_TICKS(200));

	for(uint8_t i = 0; i < (sizeof(command_rpm) / sizeof(command_rpm[0])); i++)
	{
		uint32_t sample_start;
		uint32_t sample_elapsed;
		uint16_t sample_count = 0;
		float rate_sum = 0.0f;
		float rate_min = 10000.0f;
		float rate_max = -10000.0f;
		float start_yaw;
		float end_yaw;
		float rate_average;
		float yaw_rate_average;

		snprintf(chassis_text, sizeof(chassis_text),
				 "LOW %.1fRPM", command_rpm[i]);
		HMI_SetChassisText(chassis_text);
		HMI_LogInfo("cmd %.1f rpm", command_rpm[i]);

		Motor_setspeed_fine(0, 0, command_rpm[i]);
		vTaskDelay(pdMS_TO_TICKS(TURN_LOW_RATE_WARMUP_MS));

		start_yaw = normalize_angle(imu.yaw);
		sample_start = HAL_GetTick();
		while((HAL_GetTick() - sample_start) < TURN_LOW_RATE_SAMPLE_MS)
		{
			float rate = imu.angular_rate;

			rate_sum += rate;
			if(rate < rate_min) rate_min = rate;
			if(rate > rate_max) rate_max = rate;
			sample_count++;
			vTaskDelay(pdMS_TO_TICKS(TURN_LOW_RATE_SAMPLE_PERIOD_MS));
		}

		sample_elapsed = HAL_GetTick() - sample_start;
		end_yaw = normalize_angle(imu.yaw);
		Motor_setspeed_fine(0, 0, 0);

		if(sample_count != 0U && sample_elapsed != 0U)
		{
			rate_average = rate_sum / (float)sample_count;
			yaw_rate_average = getAngleZ(end_yaw, start_yaw) *
							   1000.0f / (float)sample_elapsed;

			HMI_LogInfo("C%.1f Wavg%.3f", command_rpm[i], rate_average);
			HMI_LogInfo("Yrate%.3f %lums", yaw_rate_average,
						(unsigned long)sample_elapsed);
			HMI_LogInfo("Wmin%.3f max%.3f", rate_min, rate_max);
		}

		vTaskDelay(pdMS_TO_TICKS(TURN_LOW_RATE_PHOTO_WAIT_MS));
	}

	Motor_setspeed_fine(0, 0, 0);
	HMI_SetChassisText("LOW RPM DONE");
	HMI_LogInfo("low rpm test done");
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

void Ring_Warehouse_Clearance_Test(void)
{
	vTaskDelay(pdMS_TO_TICKS(500));
	HMI_InitScreen();
	HMI_SetSys("ARM", "WH TEST");
	HMI_SetVisionText("OFF");
	HMI_SetChassisText("STOP");
	HMI_SetArmText("INIT");
	HMI_LogInfo("warehouse 1 2 3 test");

	Motor_setspeed(0, 0, 0);
	Init_Warehouse(1);

	for(uint8_t warehouse_index = 0; warehouse_index < 3; warehouse_index++)
	{
		char *name = warehouse_index == 0 ? "WH1" :
		             warehouse_index == 1 ? "WH2" : "WH3";

		HMI_SetArmText(name);
		if(TaskFlow_PlaceFromWarehouseIndex(warehouse_index, name,
										 CIRCLE_PLACE_HEIGHT) == 0)
			break;

		if(warehouse_index < 2)
		{
			HMI_SetArmText("REMOVE ITEM");
			HMI_LogInfo("remove placed item");
			vTaskDelay(pdMS_TO_TICKS(3000));
		}
	}

	HMI_SetArmText("DONE");
	HMI_SetSys("ARM", "DONE");
	HMI_LogInfo("warehouse test done");

	while(1)
	{
		vTaskDelay(pdMS_TO_TICKS(200));
	}
}

/*
 * 函数简介：色环视觉定位与色环间固定距离切换测试。
 * 测试流程：循环定位绿、蓝、红色环，并按已标定距离在相邻色环之间移动。
 * 观察重点：像素误差收敛、航向保持、低速微调抖动和色环切换距离。
 */
void Ring_Location_Test(void)
{
	vTaskDelay(pdMS_TO_TICKS(500));

	HMI_InitScreen();
	HMI_SetSys("RING", "TEST");
	HMI_SetVisionText("INIT");
	HMI_SetChassisText("STOP");
	HMI_SetArmText("INIT");
	HMI_LogInfo("ring locate test");

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
		TaskFlow_RingLocateOne(GREEN, "GREEN", 0);
		TaskFlow_RingSwitchY(RING_SWITCH_SPEED, 0, RING_SWITCH_DISTANCE_CM);

		TaskFlow_RingLocateOne(BLUE, "BLUE", 0);
		TaskFlow_RingSwitchY(-RING_SWITCH_SPEED, 0, RING_SWITCH_DISTANCE_CM * 2.0f);

		TaskFlow_RingLocateOne(RED, "RED", 0);
		TaskFlow_RingSwitchY(RING_SWITCH_SPEED, 0, RING_SWITCH_DISTANCE_CM);

		Motor_setspeed(0, 0, 0);
		HMI_SetVisionText("WAIT");
		HMI_LogInfo("round done");
		vTaskDelay(pdMS_TO_TICKS(2000));
	}
}

/*
 * 函数简介：夹爪重新安装后的舵机位置校正测试。
 * 测试流程：CCR=50、夹爪1张开位、夹爪2张开位、CCR=50、闭合位，每步保持1秒并循环。
 */
void Claw_Calibration_Test(void)
{
	HMI_InitScreen();
	HMI_SetSys("CLAW", "CAL");
	HMI_SetChassisText("STOP");
	HMI_SetVisionText("OFF");

	while(1)
	{
		HMI_SetArmText("CCR 50");
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 50);
		vTaskDelay(pdMS_TO_TICKS(1000));

		HMI_SetArmText("CLAW1 OPEN");
		claw_move_1(open);
		vTaskDelay(pdMS_TO_TICKS(1000));

		HMI_SetArmText("CLAW2 OPEN");
		claw_move_2(open);
		vTaskDelay(pdMS_TO_TICKS(1000));

		HMI_SetArmText("CCR 50");
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 50);
		vTaskDelay(pdMS_TO_TICKS(1000));

		HMI_SetArmText("CLOSE");
		claw_move_1(close);
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}
