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
#include "GO-M8010-6.h"
#include "catch.h"
#include "postion_control.h"
#include <math.h>
#include <stdio.h>

/* 底盘速度档位：修改速度后，所有对应距离都需要重新标定。 */
#define TEST_CHASSIS_SPEED_FINE   5.0f   /* 视觉闭环微调速度。 */
#define TEST_CHASSIS_SPEED_LOW   80.0f   /* 相邻色环之间固定移动的低速。 */
#define TEST_CHASSIS_SPEED_MID  140.0f   /* 当前开环跑图使用的中速。 */

/* 软件脉冲里程计标定：1 tick 等于 5ms，增大保持周期会走得更远。 */
#define ODOM_TEST_SPEED          140.0f  /* 本次标定使用的前进速度。 */
#define ODOM_TEST_HOLD_TICKS     150     /* 配合 80 tick 加减速，保持总距离仍接近原来的 65cm。 */
#define ODOM_TEST_RAMP_TICKS      100     /* 加速和减速各 400ms，接近原驱动器内部加减速手感。 */

/* 粗加工区色环定位参数。 */
#define RING_CENTER_X            122     /* 色环在画面中的目标中心 X 坐标。 */
#define RING_CENTER_Y            133     /* 色环在画面中的目标中心 Y 坐标。 */
#define RING_LOCATE_DEADZONE       5     /* X/Y 允许的像素误差。 */
#define RING_LOCATE_STABLE_COUNT   3     /* 连续满足误差要求多少帧才算定位完成。 */
#define RING_ROUTE_HEADING      -180.0f  /* 色环定位时底盘保持的航向角。 */

/* 色环切换参数：1 tick 等于 5ms，增大保持周期会走得更远。 */
#define RING_SWITCH_RAMP_TICKS    30     /* 每次切换的加速和减速周期数。 */
#define RING_SWITCH_10CM_TICKS    37     /* 相邻两个色环之间的标定保持周期。 */
#define RING_SWITCH_20CM_TICKS    81     /* 跨越两个色环间距的标定保持周期。 */
#define RING_SWITCH_TURN_TIMEOUT 4000U   /* 移动前航向精调的最大时间，单位 ms。 */
#define RING_SWITCH_YAW_ERROR      0.2f  /* 允许开始固定平移的航向误差，单位度。 */

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

static void Test_ShowQRNow(void)
{
	HMI_SEND();
}

static char *Test_ColorName(uint8_t color)
{
	switch(color)
	{
		case RED:   return "RED";
		case GREEN: return "GREEN";
		case BLUE:  return "BLUE";
		default:    return "BAD";
	}
}

static void Test_ShowYawFixed(void)
{
	char text[24];

	snprintf(text, sizeof(text), "YAW:%.2f", normalize_angle(imu.yaw));
	HMI_SetChassisText(text);
}

void Chassis_Odom_Calibration_Test(void)
{
	vTaskDelay(pdMS_TO_TICKS(500));
	HMI_InitScreen();
	HMI_LogInfo("odom test start");

	Motor_setspeed(0, 0, 0);
	vTaskDelay(pdMS_TO_TICKS(100));
	Imu_setZero();
	vTaskDelay(pdMS_TO_TICKS(200));

	Chassis_MoveOnce(-ODOM_TEST_SPEED, 0, 0,
					 ODOM_TEST_HOLD_TICKS,
					 ODOM_TEST_RAMP_TICKS);
	Motor_setspeed(0, 0, 0);
	HMI_LogInfo("odom test done");
}

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
	}

	Motor_setspeed(0, 0, 0);
	HMI_LogInfo("turn test done");
}

static void Test_FineTuneHeading(float target_angle)
{
	HMI_SetChassisText("ALIGN");
	while(Chassis_FineTuneAngle(target_angle, RING_SWITCH_TURN_TIMEOUT) == 0)
	{
		HMI_LogWarn("align retry");
	}
	Motor_setspeed(0, 0, 0);
	Test_ShowYawFixed();
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

static uint8_t Ring_LocateOne(uint8_t cls, char *name, float target_angle)
{
	VISION_TARGET_T target;
	uint32_t last_log_tick = 0;
	uint8_t stable_count = 0;

	HMI_SetVisionText(name);
	HMI_LogInfo("loc %s", name);
	Test_FineTuneHeading(target_angle);

	while(1)
	{
		int err_x;
		int err_y;
		float vx = 0.0f;
		float vy = 0.0f;

		if(Vision_GetRingTarget(cls, &target) == 0)
		{
			HMI_SetPixelError(0, 0, 0);
			Motor_setspeed(0, 0, 0);
			stable_count = 0;

			if((HAL_GetTick() - last_log_tick) >= 300)
			{
				last_log_tick = HAL_GetTick();
				Test_ShowYawFixed();
				HMI_LogWarn("%s lost", name);
			}

			vTaskDelay(pdMS_TO_TICKS(20));
			continue;
		}

		err_x = RING_CENTER_X - (int)target.x;
		err_y = RING_CENTER_Y - (int)target.y;
		HMI_SetPixelError(err_x, err_y, 1);

		if(Test_AbsInt(err_x) <= RING_LOCATE_DEADZONE &&
		   Test_AbsInt(err_y) <= RING_LOCATE_DEADZONE)
		{
			Motor_setspeed(0, 0, 0);
			stable_count++;

			if(stable_count >= RING_LOCATE_STABLE_COUNT)
			{
				Motor_setspeed(0, 0, 0);
				vTaskDelay(pdMS_TO_TICKS(100));
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

			Chassis_OpenLoop_SetTranslation(vx, vy, target_angle);
		}

		if((HAL_GetTick() - last_log_tick) >= 300)
		{
			last_log_tick = HAL_GetTick();
			Test_ShowYawFixed();
			HMI_LogInfo("%s e%d,%d", name, err_x, err_y);
		}

		vTaskDelay(pdMS_TO_TICKS(20));
	}

}

static void Ring_SwitchY(float vy, float target_angle, uint16_t hold_ticks)
{
	HMI_SetChassisText("SWITCH");
	Chassis_MoveOnce(0, vy, target_angle, hold_ticks, RING_SWITCH_RAMP_TICKS);
	HMI_SetChassisText("STOP");
	vTaskDelay(pdMS_TO_TICKS(200));
}

static int8_t Ring_Position(uint8_t color)
{
	switch(color)
	{
		case RED:   return 0;
		case GREEN: return 1;
		case BLUE:  return 2;
		default:    return -1;
	}
}

static void Ring_CorrectHeading(float target_angle)
{
	float yaw_error;

	do
	{
		HMI_SetChassisText("ALIGN");
		Chassis_FineTuneAngle(target_angle, RING_SWITCH_TURN_TIMEOUT);
		yaw_error = fabsf(getAngleZ(normalize_angle(imu.yaw), target_angle));
		Test_ShowYawFixed();

		if(yaw_error > RING_SWITCH_YAW_ERROR)
			HMI_LogWarn("yaw err %.2f", yaw_error);
	}
	while(yaw_error > RING_SWITCH_YAW_ERROR);

	Motor_setspeed(0, 0, 0);
	HMI_SetChassisText("ALIGNED");
	vTaskDelay(pdMS_TO_TICKS(200));
}

static uint8_t Ring_SwitchToTarget(uint8_t current_color, uint8_t target_color,
								 float target_angle)
{
	int8_t current_position = Ring_Position(current_color);
	int8_t target_position = Ring_Position(target_color);
	int8_t distance;
	uint16_t hold_ticks;

	if(current_position < 0 || target_position < 0)
		return 0;

	distance = target_position - current_position;
	if(distance == 0)
		return 1;

	hold_ticks = Test_AbsInt(distance) == 1 ?
				 RING_SWITCH_10CM_TICKS : RING_SWITCH_20CM_TICKS;
	HMI_SetVisionText(Test_ColorName(target_color));
	HMI_LogInfo("switch %s", Test_ColorName(target_color));
	Ring_CorrectHeading(target_angle);
	Ring_SwitchY(distance > 0 ? TEST_CHASSIS_SPEED_LOW : -TEST_CHASSIS_SPEED_LOW,
				 target_angle,
				 hold_ticks);
	return 1;
}

static uint8_t Ring_PlaceFromWarehouseIndex(uint8_t warehouse_index, char *name)
{
	if(warehouse_index > 2)
	{
		HMI_LogError("bad wh %d", warehouse_index + 1);
		return 0;
	}

	HMI_SetArmText("TAKE WH");
	HMI_LogInfo("take %s wh%d", name, warehouse_index + 1);

	HMI_SetArmText("PUT RING");
	if(Circle_PlaceFromWarehouse(warehouse_index) == 0)
		return 0;

	HMI_LogInfo("put %s ok", name);
	return 1;
}

static uint8_t Ring_PlaceFromWarehouse(uint8_t color)
{
	uint8_t warehouse_index = Get_Warehouse_index_from_color(color);

	return Ring_PlaceFromWarehouseIndex(warehouse_index, Test_ColorName(color));
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
		if(Ring_PlaceFromWarehouseIndex(warehouse_index, name) == 0)
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
		Ring_LocateOne(GREEN, "GREEN", 0);
		Ring_SwitchY(TEST_CHASSIS_SPEED_LOW, 0, RING_SWITCH_10CM_TICKS);

		Ring_LocateOne(BLUE, "BLUE", 0);
		Ring_SwitchY(-TEST_CHASSIS_SPEED_LOW, 0, RING_SWITCH_20CM_TICKS);

		Ring_LocateOne(RED, "RED", 0);
		Ring_SwitchY(TEST_CHASSIS_SPEED_LOW, 0, RING_SWITCH_10CM_TICKS);

		Motor_setspeed(0, 0, 0);
		HMI_SetVisionText("WAIT");
		HMI_LogInfo("round done");
		vTaskDelay(pdMS_TO_TICKS(2000));
	}
}

/* 圆盘机物料定位参数。 */
#define YPJ_CENTER_X              115     /* 物料在画面中的目标中心 X 坐标。 */
#define YPJ_CENTER_Y              115     /* 物料在画面中的目标中心 Y 坐标。 */
#define YPJ_TRACK_ROI              80     /* 以目标中心为原点的方形跟踪区域半宽，单位像素。 */
#define YPJ_LOCATE_DEADZONE         8     /* 物料中心允许的像素误差。 */
#define YPJ_LOCATE_STABLE_COUNT     3     /* 连续满足误差要求多少帧才算定位完成。 */

/* 二维码搜索和路径 3：1 tick 等于 5ms，增大保持周期会走得更远。 */
#define YPJ_QR_SCAN_2CM_TICKS     120     /* 每次二维码小范围搜索移动的保持周期。 */
#define YPJ_BACK_20CM_TICKS        50     /* 圆盘机作业完成后第一段倒车的保持周期。 */
#define YPJ_BACK_20CM_RAMP_TICKS   10     /* 第一段倒车的加速和减速周期。 */
#define YPJ_BACK_LONG_TICKS       330     /* 圆盘机到粗加工区长距离倒车的保持周期。 */
#define YPJ_BACK_LONG_RAMP_TICKS   30     /* 长距离倒车的加速和减速周期。 */
#define YPJ_FIRST_TURN_ANGLE      -90.0f  /* 第一处顺时针转向的目标航向角。 */
#define YPJ_FINAL_TURN_ANGLE     -180.0f  /* 到达粗加工区后的目标航向角。 */
#define YPJ_TURN_TIMEOUT_MS      4000U    /* 每次常规转向的最大时间，单位 ms。 */
#define YPJ_TURN_SETTLE_MS        200U    /* 最后一次角度校正前的稳定等待时间。 */

/* 路径 1：启停区到二维码位置。1 tick 等于 5ms。 */
#define PATH1_LEFT_HOLD_TICKS      20     /* 起步左移的匀速保持周期。 */
#define PATH1_LEFT_RAMP_TICKS      10     /* 起步左移的加速和减速周期。 */
#define PATH1_FORWARD_HOLD_TICKS  143     /* 前进到二维码位置的匀速保持周期。 */
#define PATH1_FORWARD_RAMP_TICKS   30     /* 前进到二维码位置的加速和减速周期。 */

/* 路径 2：二维码位置到圆盘机。1 tick 等于 5ms。 */
#define PATH2_FORWARD_HOLD_TICKS  133     /* 前进到圆盘机的匀速保持周期。 */
#define PATH2_FORWARD_RAMP_TICKS   30     /* 前进到圆盘机的加速和减速周期。 */

volatile uint8_t ypj_debug_stage = 0;
volatile uint8_t ypj_debug_color = 0;
volatile uint8_t ypj_debug_target_valid = 0;
volatile uint8_t ypj_debug_target_x = 0;
volatile uint8_t ypj_debug_target_y = 0;
volatile uint32_t ypj_debug_frame_count = 0;

static void CatchLocatedMaterialToWarehouse(uint8_t color, uint16_t catch_height)
{
	uint8_t warehouse_index;
	int warehouse_angle;
	uint32_t z_finish_before;
	uint32_t z_event_before;
	uint32_t z_error_before;
	uint32_t z_restart_fail_before;
	char z_status[32];

	HMI_LogInfo("z catch");
	z_finish_before = u7_debug_z_finish_count;
	z_event_before = u7_debug_rx_event_count;
	z_error_before = u7_debug_error_count;
	z_restart_fail_before = u7_debug_rx_restart_fail;
	Z_SetHeight(catch_height);
	snprintf(z_status, sizeof(z_status), "%lums F%lu C%lu E%lu R%lu TX%u",
			 (unsigned long)z_debug_last_wait_ms,
			 (unsigned long)(u7_debug_z_finish_count - z_finish_before),
			 (unsigned long)(u7_debug_rx_event_count - z_event_before),
			 (unsigned long)(u7_debug_error_count - z_error_before),
			 (unsigned long)(u7_debug_rx_restart_fail - z_restart_fail_before),
			 (unsigned int)u7_debug_last_tx_status);
	if(z_debug_last_timeout)
	{
		HMI_LogError("z to %s", z_status);
		HMI_SetSys("ZTO", z_status);
	}
	else
	{
		HMI_LogInfo("z ok %s", z_status);
		HMI_SetSys("ZOK", z_status);
	}

	HMI_LogInfo("claw close");
	claw_move_1(close);
	vTaskDelay(pdMS_TO_TICKS(170));

	HMI_LogInfo("z up");
	Z_SetHeight(0);

	HMI_LogInfo("y home");
	Y_SetLength(Y_LENGHT_WAREHOUSE);

	warehouse_index = Get_Warehouse_index_from_color(color);
	warehouse_angle = Get_Warehouse_Angle(warehouse_index);
	HMI_LogInfo("turn wh %d", warehouse_index);
	M8010_SetAngle(warehouse_angle);

	HMI_LogInfo("z put");
	Z_SetHeight(PUT_HOUSE_HEIGHT);

	HMI_LogInfo("claw open");
	claw_move_1(open);
	vTaskDelay(pdMS_TO_TICKS(150));

	HMI_LogInfo("z safe");
	Z_SetHeight(0);
}

static uint8_t App_Test_QRWait(uint32_t wait_ms)
{
	uint32_t start_tick = HAL_GetTick();

	while((HAL_GetTick() - start_tick) < wait_ms)
	{
		if(first_code != 0 || second_code != 0)
		{
			return 1;
		}

		vTaskDelay(pdMS_TO_TICKS(20));
	}

	return 0;
}

static uint8_t App_Test_QRMove(float vx, float vy, uint16_t move_ticks)
{
	for(uint16_t i = 0; i < move_ticks; i++)
	{
		if(first_code != 0 || second_code != 0)
		{
			Motor_setspeed(0, 0, 0);
			return 1;
		}

		Chassis_OpenLoop_SetSpeed(vx, vy, 0);
		vTaskDelay(pdMS_TO_TICKS(5));
	}

	Motor_setspeed(0, 0, 0);
	return (first_code != 0 || second_code != 0);
}

static void App_Test_WaitQR_OpenLoop(void)
{
	first_code = 0;
	second_code = 0;
	HMI_SetVisionText("WAIT QR");
	HMI_LogInfo("wait qr");

	while(1)
	{
		if(App_Test_QRWait(1000)) break;

		/* Search 2 cm around the nominal QR position, returning to center each time. */
		if(App_Test_QRMove(TEST_CHASSIS_SPEED_FINE, 0, YPJ_QR_SCAN_2CM_TICKS)) break;
		if(App_Test_QRWait(1000)) break;
		if(App_Test_QRMove(-TEST_CHASSIS_SPEED_FINE, 0, YPJ_QR_SCAN_2CM_TICKS)) break;

		if(App_Test_QRMove(-TEST_CHASSIS_SPEED_FINE, 0, YPJ_QR_SCAN_2CM_TICKS)) break;
		if(App_Test_QRWait(1000)) break;
		if(App_Test_QRMove(TEST_CHASSIS_SPEED_FINE, 0, YPJ_QR_SCAN_2CM_TICKS)) break;

		if(App_Test_QRMove(0, TEST_CHASSIS_SPEED_FINE, YPJ_QR_SCAN_2CM_TICKS)) break;
		if(App_Test_QRWait(1000)) break;
		if(App_Test_QRMove(0, -TEST_CHASSIS_SPEED_FINE, YPJ_QR_SCAN_2CM_TICKS)) break;

		if(App_Test_QRMove(0, -TEST_CHASSIS_SPEED_FINE, YPJ_QR_SCAN_2CM_TICKS)) break;
		if(App_Test_QRWait(1000)) break;
		if(App_Test_QRMove(0, TEST_CHASSIS_SPEED_FINE, YPJ_QR_SCAN_2CM_TICKS)) break;
	}

	Motor_setspeed(0, 0, 0);
	Test_ShowQRNow();
	HMI_SetVisionText("QR OK");
	HMI_LogInfo("qr %03d+%03d", first_code, second_code);
}

static void Yuanpanji_LocateOpenLoop(uint8_t cls, char *name, float target_angle)
{
	VISION_TARGET_T target;
	uint32_t last_frame_count = vision_frame_count;
	uint8_t stable_count = 0;

	HMI_SetVisionText(name);
	HMI_LogInfo("wait %s", name);
	ypj_debug_stage = 11;
	Test_FineTuneHeading(target_angle);

	while(1)
	{
		int err_x;
		int err_y;
		float vx = 0.0f;
		float vy = 0.0f;

		if(vision_frame_count == last_frame_count)
		{
			ypj_debug_stage = 12;
			vTaskDelay(pdMS_TO_TICKS(10));
			continue;
		}

		last_frame_count = vision_frame_count;
		ypj_debug_frame_count = vision_frame_count;

		if(Vision_GetMaterialTarget(cls, &target) == 0)
		{
			HMI_SetPixelError(0, 0, 0);
			ypj_debug_stage = 13;
			ypj_debug_target_valid = 0;
			stable_count = 0;
			Motor_setspeed(0, 0, 0);
			vTaskDelay(pdMS_TO_TICKS(20));
			continue;
		}

		ypj_debug_target_valid = 1;
		ypj_debug_target_x = target.x;
		ypj_debug_target_y = target.y;
		err_x = YPJ_CENTER_X - (int)target.x;
		err_y = YPJ_CENTER_Y - (int)target.y;
		HMI_SetPixelError(err_x, err_y, 1);

		if(Test_AbsInt(err_x) > YPJ_TRACK_ROI ||
		   Test_AbsInt(err_y) > YPJ_TRACK_ROI)
		{
			ypj_debug_stage = 14;
			stable_count = 0;
			Motor_setspeed(0, 0, 0);
			vTaskDelay(pdMS_TO_TICKS(20));
			continue;
		}

		if(Test_AbsInt(err_x) <= YPJ_LOCATE_DEADZONE &&
		   Test_AbsInt(err_y) <= YPJ_LOCATE_DEADZONE)
		{
			ypj_debug_stage = 15;
			Motor_setspeed(0, 0, 0);
			stable_count++;

			if(stable_count >= YPJ_LOCATE_STABLE_COUNT)
			{
				ypj_debug_stage = 20;
				Motor_setspeed(0, 0, 0);
				vTaskDelay(pdMS_TO_TICKS(100));
				HMI_LogInfo("%s located", name);
				return;
			}
		}
		else
		{
			stable_count = 0;

			if(Test_AbsInt(err_y) > YPJ_LOCATE_DEADZONE)
			{
				vx = Test_SignSpeed(-err_y, TEST_CHASSIS_SPEED_FINE);
			}

			if(Test_AbsInt(err_x) > YPJ_LOCATE_DEADZONE)
			{
				vy = Test_SignSpeed(err_x, TEST_CHASSIS_SPEED_FINE);
			}

			ypj_debug_stage = 18;
			Chassis_OpenLoop_SetTranslation(vx, vy, target_angle);
		}

		vTaskDelay(pdMS_TO_TICKS(20));
	}
}

static uint8_t flow_color_data[3];
static uint8_t flow_catch_count;
static uint8_t flow_ring_place_count;
static uint8_t flow_ring_recover_count;
static uint8_t flow_current_ring;

void Route_Path1_StartToQR(void)
{
	HMI_LogInfo("path1 start");
	Motor_setspeed(0, 0, 0);
	vTaskDelay(pdMS_TO_TICKS(100));
	Imu_setZero();
	vTaskDelay(pdMS_TO_TICKS(200));
	Chassis_MoveOnce(TEST_CHASSIS_SPEED_MID, 0, 0,
					 PATH1_LEFT_HOLD_TICKS, PATH1_LEFT_RAMP_TICKS);
	Chassis_MoveOnce(0, TEST_CHASSIS_SPEED_MID, 0,
					 PATH1_FORWARD_HOLD_TICKS, PATH1_FORWARD_RAMP_TICKS);
	Motor_setspeed(0, 0, 0);
}

void Flow_QRRecognize(void)
{
	App_Test_WaitQR_OpenLoop();
	Init_Warehouse(1);
	flow_color_data[0] = (uint8_t)one.firse;
	flow_color_data[1] = (uint8_t)one.second;
	flow_color_data[2] = (uint8_t)one.thrid;
	HMI_LogInfo("seq %s %s %s", Test_ColorName(flow_color_data[0]),
				Test_ColorName(flow_color_data[1]), Test_ColorName(flow_color_data[2]));
}

void Route_Path2_QRToTurntable(void)
{
	HMI_LogInfo("path2 start");
	Chassis_MoveOnce(0, TEST_CHASSIS_SPEED_MID, 0,
					 PATH2_FORWARD_HOLD_TICKS, PATH2_FORWARD_RAMP_TICKS);
	Motor_setspeed(0, 0, 0);
}

void Flow_TurntableCatch(void)
{
	USART6_readdata_SeetZero();
	Set_Circle_Center(YPJ_CENTER_X, YPJ_CENTER_Y);
	Vision_LED_On();
	Vision_StartMaterial();
	vTaskDelay(pdMS_TO_TICKS(300));
	Yuanpanji_PrepareDetectPose();
	vTaskDelay(pdMS_TO_TICKS(500));

	for(uint8_t i = 0; i < 3; i++)
	{
		uint8_t color = flow_color_data[i];
		ypj_debug_color = color;
		if(color < RED || color > BLUE)
		{
			HMI_LogError("bad color %d", color);
			break;
		}

		Yuanpanji_LocateOpenLoop(color, Test_ColorName(color), 0.0f);
		Motor_setspeed(0, 0, 0);
		ypj_debug_stage = 31;
		CatchLocatedMaterialToWarehouse(color, YUAN_PAN_HEIGHT);
		ypj_debug_stage = 32;
		flow_catch_count++;
		HMI_LogInfo("catch %s ok", Test_ColorName(color));

		if(i < 2)
		{
			ypj_debug_stage = 40;
			Yuanpanji_PrepareDetectPose();
			ypj_debug_stage = 44;
			vTaskDelay(pdMS_TO_TICKS(300));
		}
	}

	Vision_Stop();
	Vision_LED_Off();
	Motor_setspeed(0, 0, 0);
	Z_SetHeight(0);
}

void Route_Path3_TurntableToProcessing(void)
{
	if(flow_catch_count < 3)
		return;

	HMI_LogInfo("path3 start");
	Y_SetLength(0);
	M8010_SetAngle(0);
	Chassis_MoveOnce(0, -TEST_CHASSIS_SPEED_MID, 0,
					 YPJ_BACK_20CM_TICKS, YPJ_BACK_20CM_RAMP_TICKS);
	Chassis_TurnToAngle(YPJ_FIRST_TURN_ANGLE, YPJ_TURN_TIMEOUT_MS);
	Chassis_MoveOnce(0, -TEST_CHASSIS_SPEED_MID, -90.0f,
					 YPJ_BACK_LONG_TICKS, YPJ_BACK_LONG_RAMP_TICKS);
	Chassis_TurnToAngle(YPJ_FINAL_TURN_ANGLE, YPJ_TURN_TIMEOUT_MS);
	vTaskDelay(pdMS_TO_TICKS(YPJ_TURN_SETTLE_MS));
	Chassis_TurnToAngle(YPJ_FINAL_TURN_ANGLE, YPJ_TURN_TIMEOUT_MS);
	Test_ShowYawFixed();
}

void Flow_ProcessingArea(void)
{
	if(flow_catch_count < 3)
	{
		HMI_LogError("caught %d", flow_catch_count);
		return;
	}

	USART6_readdata_SeetZero();
	Set_Circle_Center(RING_CENTER_X, RING_CENTER_Y);
	Vision_LED_On();
	Vision_StartRing();
	vTaskDelay(pdMS_TO_TICKS(300));
	Circle_PrepareDetectPose();

	if(Ring_LocateOne(GREEN, "GREEN", RING_ROUTE_HEADING) != 0)
	{
		for(uint8_t i = 0; i < 3; i++)
		{
			uint8_t color = flow_color_data[i];
			if(color != flow_current_ring)
			{
				if(Ring_SwitchToTarget(flow_current_ring, color, RING_ROUTE_HEADING) == 0)
					break;
				if(Ring_LocateOne(color, Test_ColorName(color), RING_ROUTE_HEADING) == 0)
					break;
				flow_current_ring = color;
			}
			if(Ring_PlaceFromWarehouse(color) == 0)
				break;
			flow_ring_place_count++;
			if(i < 2)
				Circle_PrepareDetectPose();
		}
	}

	if(flow_ring_place_count < 3)
	{
		HMI_LogError("placed %d", flow_ring_place_count);
		Vision_Stop();
		Vision_LED_Off();
		return;
	}

	HMI_LogInfo("all placed");
	USART6_readdata_SeetZero();
	Set_Circle_Center(YPJ_CENTER_X, YPJ_CENTER_Y);
	Vision_StartMaterial();
	vTaskDelay(pdMS_TO_TICKS(300));

	for(uint8_t i = 0; i < 3; i++)
	{
		uint8_t color = flow_color_data[i];
		if(color != flow_current_ring)
		{
			if(Ring_SwitchToTarget(flow_current_ring, color, RING_ROUTE_HEADING) == 0)
				break;
			flow_current_ring = color;
		}

		Circle_PrepareMaterialCatchPose();
		Yuanpanji_LocateOpenLoop(color, Test_ColorName(color), RING_ROUTE_HEADING);
		Motor_setspeed(0, 0, 0);
		CatchLocatedMaterialToWarehouse(color, CIRCLE_PLACE_HEIGHT);
		flow_ring_recover_count++;
		HMI_LogInfo("recover %s ok", Test_ColorName(color));
	}

	Vision_Stop();
	Vision_LED_Off();
	Motor_setspeed(0, 0, 0);
	Z_SetHeight(CIRCLE_SAFE_HEIGHT);
	Y_SetLength(0);
	M8010_SetAngle(0);
	ypj_debug_stage = 100;
	if(flow_ring_recover_count >= 3)
		HMI_LogInfo("all recovered");
	else
		HMI_LogError("recovered %d", flow_ring_recover_count);
}

void Flow_RunCurrent(void)
{
	flow_catch_count = 0;
	flow_ring_place_count = 0;
	flow_ring_recover_count = 0;
	flow_current_ring = GREEN;
	Route_Path1_StartToQR();
	Flow_QRRecognize();
	Route_Path2_QRToTurntable();
	Flow_TurntableCatch();
	Route_Path3_TurntableToProcessing();
	Flow_ProcessingArea();
}

void Yuanpanji_OpenLoop_Catch_Test(void)
{
	vTaskDelay(pdMS_TO_TICKS(500));
	HMI_InitScreen();
	HMI_LogInfo("open loop flow");
	Flow_RunCurrent();
	while(1)
		vTaskDelay(pdMS_TO_TICKS(200));
}
