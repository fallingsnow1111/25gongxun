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
#include "task_flow.h"
#include "tim.h"
#include <math.h>
#include <stdio.h>

/* 底盘速度档位：修改速度后，所有对应距离都需要重新标定。 */

/* 加入角速度环前的转向稳态基线测试参数。 */
#define TURN_BASELINE_TIMEOUT_MS       5000U  /* 单次转向及二次确认的总时间。 */
#define TURN_BASELINE_SETTLE_MS         300U  /* 到位停车后等待机械惯性衰减。 */
#define TURN_BASELINE_SAMPLE_MS        1000U  /* 每个目标角度的稳态采样时长。 */
#define TURN_BASELINE_SAMPLE_PERIOD_MS   20U  /* 稳态采样周期。 */

/* IMU 静止稳定性测试参数。 */
#define IMU_STATIC_BOOT_WAIT_MS        2000U  /* 上电后等待 IMU 初始零偏稳定。 */
#define IMU_STATIC_ZERO_WAIT_MS         200U  /* 发送归零命令后等待其生效。 */
#define IMU_STATIC_SAMPLE_MS          60000U  /* 静止采样总时长。 */
#define IMU_STATIC_PERIOD_MS             20U  /* 静止采样周期。 */
#define IMU_STATIC_CHECKPOINT_MS       10000U  /* 每隔 10 秒保存一次航向漂移。 */

/* IMU 上电归零后直行测试参数。 */
#define IMU_STRAIGHT_BOOT_WAIT_MS       2000U  /* 上电后先等待 IMU 初始零偏稳定。 */
#define IMU_STRAIGHT_ZERO_WAIT_MS        200U  /* 发送归零命令后等待其生效。 */
#define IMU_STRAIGHT_SPEED             160.0f  /* 直行测试速度。 */
#define IMU_STRAIGHT_DISTANCE_CM       100.0f  /* 直行测试距离，单位 cm。 */

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
		Chassis_TurnToAngle(target_angle[i], TURN_BASELINE_TIMEOUT_MS);
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
 * 函数简介：测试机器人完全静止时 IMU 航向和角速度的稳态误差。
 * 测试流程：停止底盘，上电等待 2 秒后归零，再等待 200ms，以 20ms 周期采样 60 秒。
 * 输出内容：航向末值、平均漂移、最大绝对漂移、峰峰值，以及角速度统计量。
 * 注意事项：测试期间不要触碰机器人，也不要运行机械臂和底盘。
 */
void IMU_Static_Stability_Test(void)
{
	TickType_t last_wake;
	uint32_t sample_count = 0U;
	float reference_yaw;
	float yaw_error = 0.0f;
	float yaw_error_sum = 0.0f;
	float yaw_error_min = 0.0f;
	float yaw_error_max = 0.0f;
	float yaw_error_abs_max = 0.0f;
	float rate_sum = 0.0f;
	float rate_min = 0.0f;
	float rate_max = 0.0f;
	float yaw_checkpoint[6] = {0.0f};
	uint8_t checkpoint_index = 0U;

	vTaskDelay(pdMS_TO_TICKS(500));
	HMI_InitScreen();
	HMI_SetSys("IMU", "STATIC");
	HMI_SetChassisText("KEEP STILL");
	HMI_LogInfo("imu boot wait 2s");

	Motor_setspeed(0, 0, 0);
	vTaskDelay(pdMS_TO_TICKS(IMU_STATIC_BOOT_WAIT_MS));
	Imu_setZero();
	HMI_LogInfo("imu zero");
	vTaskDelay(pdMS_TO_TICKS(IMU_STATIC_ZERO_WAIT_MS));

	reference_yaw = normalize_angle(imu.yaw);
	HMI_LogInfo("Yref %.3f", reference_yaw);
	HMI_LogInfo("sampling 60s");

	last_wake = xTaskGetTickCount();
	while(sample_count < (IMU_STATIC_SAMPLE_MS / IMU_STATIC_PERIOD_MS))
	{
		float yaw_now = normalize_angle(imu.yaw);
		float rate_now = imu.angular_rate;
		float yaw_abs;

		yaw_error = normalize_angle(yaw_now - reference_yaw);
		yaw_abs = fabsf(yaw_error);

		if(sample_count == 0U)
		{
			yaw_error_min = yaw_error;
			yaw_error_max = yaw_error;
			rate_min = rate_now;
			rate_max = rate_now;
		}
		else
		{
			if(yaw_error < yaw_error_min)
				yaw_error_min = yaw_error;
			if(yaw_error > yaw_error_max)
				yaw_error_max = yaw_error;
			if(rate_now < rate_min)
				rate_min = rate_now;
			if(rate_now > rate_max)
				rate_max = rate_now;
		}

		if(yaw_abs > yaw_error_abs_max)
			yaw_error_abs_max = yaw_abs;

		yaw_error_sum += yaw_error;
		rate_sum += rate_now;
		sample_count++;

		if((sample_count % (IMU_STATIC_CHECKPOINT_MS / IMU_STATIC_PERIOD_MS)) == 0U &&
		   checkpoint_index < 6U)
		{
			yaw_checkpoint[checkpoint_index] = yaw_error;
			checkpoint_index++;
		}

		vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(IMU_STATIC_PERIOD_MS));
	}

	Motor_setspeed(0, 0, 0);
	HMI_SetChassisText("IMU TEST DONE");
	HMI_LogInfo("Y10%.3f Y20%.3f", yaw_checkpoint[0], yaw_checkpoint[1]);
	HMI_LogInfo("Y30%.3f Y40%.3f", yaw_checkpoint[2], yaw_checkpoint[3]);
	HMI_LogInfo("Y50%.3f Y60%.3f", yaw_checkpoint[4], yaw_checkpoint[5]);
	vTaskDelay(pdMS_TO_TICKS(10000));

	HMI_LogInfo("Ye%.3f Ya%.3f P%.3f A%.3f", yaw_error,
				yaw_error_sum / (float)sample_count,
				yaw_error_max - yaw_error_min, yaw_error_abs_max);
	HMI_LogInfo("Slope%.4f Wavg%.4f", yaw_error / 60.0f,
				rate_sum / (float)sample_count);
	HMI_LogInfo("Wmin%.3f Wmax%.3f", rate_min, rate_max);

	while(1)
		vTaskDelay(pdMS_TO_TICKS(200));
}

/* IMU 多次归零漂移测试参数。 */
#define IMU_DRIFT_INTERVAL_MS   10000U  /* 每段归零间隔 10 秒。 */
#define IMU_DRIFT_PERIOD_MS        20U  /* 采样周期。 */
#define IMU_DRIFT_INTERVALS         6U  /* 共 6 段，总计 60 秒。 */
#define IMU_DRIFT_BOOT_WAIT_MS   2000U  /* 上电后等待 IMU 稳定。 */
#define IMU_DRIFT_ZERO_WAIT_MS    200U  /* 归零后等待生效。 */

/*
 * 函数简介：每隔 10 秒归零一次，统计每段的 Yaw 漂移量和平均角速率。
 * 测试流程：上电等 2 秒 → 归零 → 每 10 秒记录漂移并重新归零 → 共 6 段。
 * 输出内容：每段漂移角度、平均角速率，以及 6 段汇总均值。
 */
void IMU_Drift_Rezero_Test(void)
{
	float interval_drift[IMU_DRIFT_INTERVALS];
	float interval_rate_avg[IMU_DRIFT_INTERVALS];
	uint32_t samples_per_interval = IMU_DRIFT_INTERVAL_MS / IMU_DRIFT_PERIOD_MS;
	TickType_t last_wake;

	vTaskDelay(pdMS_TO_TICKS(500));
	HMI_InitScreen();
	HMI_SetSys("IMU", "DRIFT REZERO");
	HMI_SetChassisText("KEEP STILL");
	HMI_LogInfo("drift rezero 6x10s");

	Motor_setspeed(0, 0, 0);
	vTaskDelay(pdMS_TO_TICKS(IMU_DRIFT_BOOT_WAIT_MS));
	Imu_setZero();
	vTaskDelay(pdMS_TO_TICKS(IMU_DRIFT_ZERO_WAIT_MS));

	for (uint8_t n = 0; n < IMU_DRIFT_INTERVALS; n++)
	{
		float rate_sum = 0.0f;
		uint32_t cnt;

		last_wake = xTaskGetTickCount();
		for (cnt = 0; cnt < samples_per_interval; cnt++)
		{
			rate_sum += imu.angular_rate;
			vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(IMU_DRIFT_PERIOD_MS));
		}

		interval_drift[n]    = normalize_angle(imu.yaw);
		interval_rate_avg[n] = rate_sum / (float)cnt;

		HMI_LogInfo("#%d drift%.3f Wavg%.3f",
					n + 1, interval_drift[n], interval_rate_avg[n]);

		/* 最后一段不归零，避免覆盖最终漂移值。 */
		if (n < IMU_DRIFT_INTERVALS - 1)
		{
			Imu_setZero();
			vTaskDelay(pdMS_TO_TICKS(IMU_DRIFT_ZERO_WAIT_MS));
		}
	}

	Motor_setspeed(0, 0, 0);
	HMI_SetChassisText("TEST DONE");
	{
		float drift_abs_sum = 0.0f;
		float rate_avg_sum  = 0.0f;
		for (uint8_t i = 0; i < IMU_DRIFT_INTERVALS; i++)
		{
			drift_abs_sum += fabsf(interval_drift[i]);
			rate_avg_sum  += interval_rate_avg[i];
		}
		HMI_LogInfo("AVG |drift|%.3f Wavg%.3f",
					drift_abs_sum / (float)IMU_DRIFT_INTERVALS,
					rate_avg_sum  / (float)IMU_DRIFT_INTERVALS);
	}

	while (1)
		vTaskDelay(pdMS_TO_TICKS(200));
}

/*
 * 函数简介：验证新 IMU 上电稳定后归零，锁定零度航向直行是否仍会跑偏。
 * 测试流程：上电等待 2 秒，归零并等待 200ms 生效，然后锁定 0 度前进 100cm。
 * 观察重点：起步是否扭动、直线是否弯曲，以及停车后的航向误差。
 */
void IMU_Stable_Straight_Test(void)
{
	const float target_yaw = 0.0f;
	float final_yaw;
	float final_error;
	uint32_t move_start;
	uint32_t move_time;

	HMI_InitScreen();
	HMI_SetSys("IMU", "STRAIGHT");
	HMI_SetChassisText("IMU WAIT 2S");
	HMI_LogInfo("imu boot wait 2s");

	Motor_setspeed(0, 0, 0);
	vTaskDelay(pdMS_TO_TICKS(IMU_STRAIGHT_BOOT_WAIT_MS));
	Imu_setZero();
	HMI_LogInfo("imu zero");
	vTaskDelay(pdMS_TO_TICKS(IMU_STRAIGHT_ZERO_WAIT_MS));

	HMI_SetChassisText("STRAIGHT RUN");
	HMI_LogInfo("target yaw %.3f", target_yaw);

	move_start = HAL_GetTick();
	Chassis_MoveByDistance(0.0f, IMU_STRAIGHT_SPEED,
						  target_yaw, IMU_STRAIGHT_DISTANCE_CM);
	move_time = HAL_GetTick() - move_start;

	Motor_setspeed(0, 0, 0);
	vTaskDelay(pdMS_TO_TICKS(500));
	final_yaw = normalize_angle(imu.yaw);
	final_error = getAngleZ(final_yaw, target_yaw);

	HMI_SetChassisText("STRAIGHT DONE");
	HMI_LogInfo("T%.3f Y%.3f", target_yaw, final_yaw);
	HMI_LogInfo("E%.3f %lums", final_error, (unsigned long)move_time);
	HMI_LogInfo("measure path");

	while(1)
		vTaskDelay(pdMS_TO_TICKS(200));
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

#define TURN_INTEGRAL_CMD_RPM           40.0f
#define TURN_INTEGRAL_RATE_GAIN          1.066f
#define TURN_INTEGRAL_RAMP_STEPS         20U
#define TURN_INTEGRAL_RAMP_PERIOD_MS      5U
#define TURN_INTEGRAL_PERIOD_MS           5U
#define TURN_INTEGRAL_TIMEOUT_MS       5000U
#define TURN_INTEGRAL_PHOTO_WAIT_MS    2000U

/*
 * 依次测试正反向0.1~1.0RPM，每档采样10秒。
 * 屏幕输出指令RPM、IMU平均角速度、Yaw反算角速度及角速度范围。
 */
void Chassis_LowSpeed_Linearity_Test(void)
{
	static const float command_rpm[] = {
		 1.0f, -1.0f
	};
	char chassis_text[32];

	vTaskDelay(pdMS_TO_TICKS(500));
	HMI_InitScreen();
	HMI_SetSys("TURN", "LOW RPM");
	HMI_SetVisionText("1RPM MIN TEST");
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

static float Test_IntegralTurnRamp(float start_speed, float end_speed, uint8_t send_cmd)
{
	float calc_delta = 0.0f;

	for(uint8_t step = 1; step <= TURN_INTEGRAL_RAMP_STEPS; step++)
	{
		float ratio = (float)step / (float)TURN_INTEGRAL_RAMP_STEPS;
		float speed = start_speed + (end_speed - start_speed) * ratio;

		if(send_cmd != 0U)
			Motor_setspeed_fine(0, 0, speed);

		/* 当前底盘方向：正 vw 对应 yaw 减小，负 vw 对应 yaw 增大。 */
		calc_delta += (-speed) * TURN_INTEGRAL_RATE_GAIN *
					  (float)TURN_INTEGRAL_RAMP_PERIOD_MS / 1000.0f;
		if(send_cmd != 0U)
			vTaskDelay(pdMS_TO_TICKS(TURN_INTEGRAL_RAMP_PERIOD_MS));
	}

	return calc_delta;
}

static void Test_IntegralTurnOnce(float target_delta, float abs_cmd_rpm)
{
	TickType_t last_wake;
	CHASSIS_ODOM_T odom;
	uint32_t start_tick;
	uint32_t last_tick;
	uint32_t now_tick;
	float start_yaw;
	float end_yaw;
	float cmd_rpm;
	float calc_delta = 0.0f;
	float brake_delta;
	float imu_delta;
	float error;

	cmd_rpm = (target_delta < 0.0f) ? fabsf(abs_cmd_rpm) : -fabsf(abs_cmd_rpm);

	Chassis_OdomResetSegment();
	start_yaw = normalize_angle(imu.yaw);
	start_tick = HAL_GetTick();

	HMI_SetChassisText("INT TURN RUN");
	HMI_LogInfo("I turn %.0f C%.0f", target_delta, cmd_rpm);

	calc_delta += Test_IntegralTurnRamp(0.0f, cmd_rpm, 1U);
	last_tick = HAL_GetTick();
	brake_delta = Test_IntegralTurnRamp(cmd_rpm, 0.0f, 0U);
	last_wake = xTaskGetTickCount();

	while((HAL_GetTick() - start_tick) < TURN_INTEGRAL_TIMEOUT_MS)
	{
		now_tick = HAL_GetTick();
		calc_delta += (-cmd_rpm) * TURN_INTEGRAL_RATE_GAIN *
					  (float)(now_tick - last_tick) / 1000.0f;
		last_tick = now_tick;

		if(fabsf(calc_delta + brake_delta) >= fabsf(target_delta))
			break;

		Motor_setspeed_fine(0, 0, cmd_rpm);
		vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TURN_INTEGRAL_PERIOD_MS));
	}

	calc_delta += Test_IntegralTurnRamp(cmd_rpm, 0.0f, 1U);
	Motor_setspeed_fine(0, 0, 0);
	vTaskDelay(pdMS_TO_TICKS(300));

	end_yaw = normalize_angle(imu.yaw);
	imu_delta = getAngleZ(end_yaw, start_yaw);
	error = imu_delta - calc_delta;
	Chassis_OdomGetSegment(&odom);

	HMI_SetChassisText("INT TURN DONE");
	HMI_LogInfo("I%.1f Y%.2f E%.2f", calc_delta, imu_delta, error);
	HMI_LogInfo("O%ld T%lums", (long)odom.w,
				(unsigned long)(HAL_GetTick() - start_tick));
	vTaskDelay(pdMS_TO_TICKS(TURN_INTEGRAL_PHOTO_WAIT_MS));
}

/*
 * 积分转弯测试：只按发送的转向速度和时间积分估算转角，
 * 停车后再和 IMU 实测角度对比，用来判断“不开角度闭环转弯”是否可用。
 */
void Chassis_Integral_Turn_Test(void)
{
	static const float target_delta[] = {-90.0f, -90.0f, 90.0f, 90.0f};

	vTaskDelay(pdMS_TO_TICKS(500));
	HMI_InitScreen();
	HMI_SetSys("TURN", "INTEGRAL");
	HMI_SetVisionText("C40 INT");
	HMI_LogInfo("integral turn test");

	Motor_setspeed_fine(0, 0, 0);
	vTaskDelay(pdMS_TO_TICKS(200));
	Imu_setZero();
	vTaskDelay(pdMS_TO_TICKS(200));

	for(uint8_t i = 0; i < (sizeof(target_delta) / sizeof(target_delta[0])); i++)
	{
		Test_IntegralTurnOnce(target_delta[i], TURN_INTEGRAL_CMD_RPM);
	}

	Motor_setspeed_fine(0, 0, 0);
	HMI_SetChassisText("INT TEST DONE");
	HMI_LogInfo("integral test done");
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
										 CIRCLE_PLACE_HEIGHT, 0, 0) == 0)
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
 * 固定仓位取料并放到指定姿态，仅用于验证关节角和放置精度。
 * 本测试不伸长到旋转避障长度，运行前必须确认机构允许直接旋转。
 */
static uint8_t Ring_TestPlaceFromWarehouse(uint8_t warehouse_index,
										 int16_t warehouse_angle,
										 int16_t place_angle,
										 uint16_t place_length)
{
	if(warehouse_index > 2)
		return 0;

	claw_move_2(open);
	Z_SetHeight(CIRCLE_SAFE_HEIGHT);
	M8010_SetAngle(warehouse_angle);
	Y_SetLength(CIRCLE_WAREHOUSE_LENGTH);
	Z_SetHeight(CIRCLE_WAREHOUSE_HEIGHT);
	claw_move_2(close);
	vTaskDelay(pdMS_TO_TICKS(100));

	Z_SetHeight(CIRCLE_SAFE_HEIGHT);
	M8010_SetAngle(place_angle);
	Y_SetLength(place_length);
	Z_SetHeight(CIRCLE_PLACE_HEIGHT);
	claw_move_2(open);
	vTaskDelay(pdMS_TO_TICKS(100));
	Z_SetHeight(CIRCLE_DETECT_HEIGHT);

	return 1;
}

/*
 * 上电姿态初始化后定位一次绿环，再固定按 1、2、3 号仓位放置。
 * 忽略二维码和物料颜色：1号放红位、2号放绿位、3号放蓝位。
 */
void Ring_LocateOne_Place123_Test(void)
{
	static const int16_t warehouse_angle[3] = {
		FIRST_WAREHOUSE,
		SECOND_WAREHOUSE,
		THIRD_WAREHOUSE
	};
	static const int16_t place_angle[3] = {
		RED_PLACE_ANGLE, GREEN_PLACE_ANGLE, BLUE_PLACE_ANGLE
	};
	static const uint16_t place_length[3] = {
		RED_PLACE_LENGTH, GREEN_PLACE_LENGTH, BLUE_PLACE_LENGTH
	};
	static char * const place_name[3] = {"WH1 RED", "WH2 GREEN", "WH3 BLUE"};
	float target_heading;

	if(Flow_ArmPoseInit() == 0)
		return;

	Motor_setspeed(0, 0, 0);
	target_heading = imu.yaw;
	USART6_readdata_SeetZero();
	Set_Circle_Center(RING_CENTER_X, RING_CENTER_Y);
	Vision_LED_On();
	Vision_StartRing();
	vTaskDelay(pdMS_TO_TICKS(300));
	Circle_PrepareDetectPose();

	if(TaskFlow_RingLocateOne(GREEN, "GREEN", target_heading) == 0)
	{
		Motor_setspeed(0, 0, 0);
		Vision_Stop();
		Vision_LED_Off();
		HMI_SetSys("PLACE TEST", "LOC ERR");
		HMI_SetArmText("STOP");
		HMI_LogError("green locate fail");
		return;
	}

	Motor_setspeed(0, 0, 0);
	Vision_Stop();
	Vision_LED_Off();
	HMI_SetVisionText("OFF");

	for(uint8_t i = 0; i < 3; i++)
	{
		HMI_SetArmText(place_name[i]);
		HMI_LogInfo("%s A%d L%d", place_name[i], place_angle[i], place_length[i]);
		if(Ring_TestPlaceFromWarehouse(i, warehouse_angle[i],
										 place_angle[i], place_length[i]) == 0)
		{
			HMI_SetSys("PLACE TEST", "ARM ERR");
			HMI_SetArmText("STOP");
			return;
		}
	}

	HMI_SetSys("PLACE TEST", "DONE");
	HMI_SetArmText("DONE");
	HMI_LogInfo("place 1 2 3 done");
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
 * 函数简介：独立运行主流程使用的机械臂姿态初始化。
 * 调参位置：APP/task_flow.c 文件顶部的 ARM_INIT_* 参数。
 */
void Arm_Pose_Init_Test(void)
{
	(void)Flow_ArmPoseInit();

	while(1)
	{
		vTaskDelay(pdMS_TO_TICKS(200));
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
