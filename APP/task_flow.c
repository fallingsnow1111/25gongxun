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
#include <math.h>
#include <stdio.h>

/* 上电机械臂姿态初始化参数，调试机械零点时只修改这里。 */
#define ARM_INIT_Z_OFFSET             (-78)  /* Z轴从上电临时零点上升的距离。 */
#define ARM_INIT_Y_OFFSET             (-44)  /* Y轴从上电临时零点缩回的距离。 */
#define ARM_INIT_JOINT_ANGLE             34  /* 关节转到该角度后设置为新零点。 */
#define ARM_INIT_JOINT_SETTLE_MS       1000U /* 关节转动后的机械稳定时间。 */
#define ARM_INIT_ZERO_INTERVAL_MS        20U /* UART7两条清零命令之间的间隔。 */

/* 底盘速度档位：修改速度后，所有对应距离都需要重新标定。 */
#define TEST_CHASSIS_SPEED_FINE   5.0f   /* 视觉闭环微调速度。 */
#define VISION_LOCATE_KP          0.25f  /* 像素误差到平移速度的比例系数。 */
#define VISION_LOCATE_SPEED_MIN   2.0f   /* 超出死区后的最低调整速度。 */
#define VISION_LOCATE_SPEED_MAX  20.0f   /* 视觉定位允许的最大调整速度。 */

/* 粗加工区色环定位参数。 */
#define RING_CENTER_X            115     /* 色环在画面中的目标中心 X 坐标。 */
#define RING_CENTER_Y            122     /* 色环在画面中的目标中心 Y 坐标。 */
#define RING_LOCATE_DEADZONE       2     /* X/Y 允许的像素误差。 */
#define RING_LOCATE_STABLE_COUNT   3     /* 连续满足误差要求多少帧才算定位完成。 */
#define RING_ROUTE_HEADING      -180.0f  /* 色环定位时底盘保持的航向角。 */

/* 色环放置定位模式：定位一次放3件，或每放1件都重新定位。 */
#define RING_PLACE_MODE_LOCATE_ONE   1U /* 先定位绿色环一次，再按固定间距连续放3件。 */
#define RING_PLACE_MODE_LOCATE_EACH  3U /* 切换到目标环后，每件都重新视觉定位。 */
#define RING_PLACE_MODE              RING_PLACE_MODE_LOCATE_EACH /* 当前调试模式。 */

/* 色环切换参数。 */
#define RING_SWITCH_SPEED         50.0f  /* 色环之间固定移动的速度。 */
#define RING_SWITCH_DISTANCE_CM   15.0f  /* 相邻两个色环的中心距离，单位 cm。 */
#define RING_SWITCH_TURN_TIMEOUT 4000U   /* 移动前航向精调的最大时间，单位 ms。 */
#define RING_SWITCH_YAW_ERROR      0.1f  /* 色环定位与切换允许的航向误差，单位度。 */

/* 圆盘机物料定位参数。 */
#define YPJ_CENTER_X              116     /* 物料在画面中的目标中心 X 坐标。 */
#define YPJ_CENTER_Y              125     /* 物料在画面中的目标中心 Y 坐标。 */
#define YPJ_TRACK_ROI              80     /* 以目标中心为原点的方形跟踪区域半宽，单位像素。 */
#define YPJ_LOCATE_DEADZONE         5     /* 物料中心允许的像素误差。 */
#define YPJ_LOCATE_STABLE_COUNT     3     /* 连续满足误差要求多少帧才算定位完成。 */

/* 二维码小范围搜索。 */
#define YPJ_QR_SCAN_DISTANCE_CM   2.0f    /* 每次二维码小范围搜索的固定移动距离。 */
#define YPJ_TURN_TIMEOUT_MS      4000U    /* 每次常规转向的最大时间，单位 ms。 */

/* 路径 1：启停区到二维码区域。 */
#define PATH1_LEFT_SPEED          80.0f   /* 第一段左移速度。 */
#define PATH1_LEFT_DISTANCE_CM    18.0f   /* 第一段左移距离，单位 cm。 */
#define PATH1_FORWARD_SPEED      160.0f   /* 前进到二维码区域的速度。 */
#define PATH1_FORWARD_DISTANCE_CM 63.0f   /* 前进到二维码区域的距离，单位 cm。 */

/* 路径 2：二维码区域到圆盘机。 */
#define PATH2_FORWARD_SPEED      200.0f   /* 扫码完成后的前进速度。 */
#define PATH2_FORWARD_DISTANCE_CM 80.0f   /* 二维码区域到圆盘机的距离，单位 cm。 */
#define PATH2_ACCEL_TICKS        100U     /* 加速 500ms，1 tick = 5ms。 */
#define PATH2_DECEL_TICKS        200U     /* 减速 750ms，降低停车冲击。 */
#define PATH2_RIGHT_SPEED          20.0f   /* 到达圆盘机后的低速右移速度。 */
#define PATH2_RIGHT_DISTANCE_CM    5.0f   /* 到达圆盘机后的右移距离，单位 cm。 */

/* 路径 3：圆盘机到粗加工区。 */
#define PATH3_BACK_SPEED          80.0f   /* 圆盘机夹取完成后的倒车速度。 */
#define PATH3_BACK_DISTANCE_CM    36.0f   /* 第一段倒车距离，单位 cm。 */
#define PATH3_LEFT_SPEED          20.0f   /* 转弯前左移避线速度。 */
#define PATH3_LEFT_DISTANCE_CM     5.0f   /* 转弯前左移避线距离，单位 cm。 */
#define PATH3_FIRST_ANGLE        -90.0f   /* 第一次顺时针转90°后的目标航向。 */
#define PATH3_LONG_BACK_SPEED    200.0f   /* 到粗加工区的长距离倒车速度。 */
#define PATH3_LONG_BACK_CM       176.0f   /* 到粗加工区的长距离倒车距离，单位 cm。 */
#define PATH3_FINAL_ANGLE       -180.0f   /* 进入粗加工区后的目标航向。 */

/* 路径 4：粗加工区物料全部回收后的路线。 */
#define PATH4_BACK_SPEED         200.0f   /* 离开粗加工区的倒车速度。 */
#define PATH4_BACK_DISTANCE_CM   85.0f    /* 离开粗加工区的倒车距离，单位 cm。 */
#define PATH4_BACK_ACCEL_TICKS   100U     /* 倒车加速 500ms，1 tick = 5ms。 */
#define PATH4_BACK_DECEL_TICKS   150U     /* 倒车减速 750ms，降低停车冲击。 */
#define PATH4_LEFT_SPEED          20.0f    /* 转弯前左移避线速度。 */
#define PATH4_LEFT_DISTANCE_CM     5.0f    /* 转弯前左移避线距离，单位 cm。 */
#define PATH4_TURN_ANGLE        -270.0f   /* 从-180°继续顺时针90°后的连续世界航向。 */
#define PATH4_FORWARD_SPEED     -200.0f   /* 前往暂存区的移动速度。 */
#define PATH4_FORWARD_DISTANCE_CM 86.0f   /* 前往暂存区的距离，单位 cm。 */

/* 路径 5：第一轮暂存区完成后返回圆盘机。 */
#define PATH5_BACK_SPEED          200.0f  /* 离开暂存区的倒车速度。 */
#define PATH5_BACK_DISTANCE_CM     90.0f  /* 离开暂存区的倒车距离，单位 cm。 */
#define PATH5_TURN_ANGLE         -360.0f  /* 从-270°继续顺时针90°后的连续世界航向。 */
#define PATH5_FINAL_BACK_SPEED    160.0f  /* 转向后驶向圆盘机的倒车速度。 */
#define PATH5_FINAL_BACK_CM        52.0f  /* 转向后驶向圆盘机的倒车距离，单位 cm。 */

/* 路径 6：第二轮暂存区完成后返回启停区。 */
#define PATH6_BACK_SPEED          200.0f  /* 离开第二轮暂存区的倒车速度。 */
#define PATH6_BACK_DISTANCE_CM     90.0f  /* 从绿环离开暂存区的基准倒车距离，单位 cm。 */
#define PATH6_BACK_ACCEL_TICKS    100U    /* 倒车加速 500ms，1 tick = 5ms。 */
#define PATH6_BACK_DECEL_TICKS    150U    /* 倒车减速 750ms，降低停车冲击。 */
#define PATH6_PRETURN_LEFT_SPEED   20.0f   /* 转弯前左移避线速度。 */
#define PATH6_PRETURN_LEFT_CM       5.0f   /* 转弯前左移避线距离，单位 cm。 */
#define PATH6_TURN_ANGLE         -360.0f  /* 返航前顺时针转向后的目标航向。 */
#define PATH6_LONG_BACK_SPEED     200.0f  /* 朝启停区长距离倒车的速度。 */
#define PATH6_LONG_BACK_CM        190.0f  /* 朝启停区长距离倒车的距离，单位 cm。 */
#define PATH6_RIGHT_SPEED          80.0f  /* 最后右移对准启停区的速度。 */
#define PATH6_RIGHT_DISTANCE_CM    14.0f  /* 最后右移对准启停区的距离，单位 cm。 */

static int Flow_AbsInt(int value)
{
	return (value < 0) ? -value : value;
}

static float Flow_PixelErrorSpeed(int error, int deadzone)
{
	float speed;

	if(Flow_AbsInt(error) <= deadzone)
		return 0.0f;

	speed = (float)error * VISION_LOCATE_KP;
	if(speed > VISION_LOCATE_SPEED_MAX)
		speed = VISION_LOCATE_SPEED_MAX;
	else if(speed < -VISION_LOCATE_SPEED_MAX)
		speed = -VISION_LOCATE_SPEED_MAX;

	if(speed > 0.0f && speed < VISION_LOCATE_SPEED_MIN)
		speed = VISION_LOCATE_SPEED_MIN;
	else if(speed < 0.0f && speed > -VISION_LOCATE_SPEED_MIN)
		speed = -VISION_LOCATE_SPEED_MIN;

	return speed;
}

static void Flow_ShowQRNow(void)
{
	HMI_SEND();
}

static char *Flow_ColorName(uint8_t color)
{
	switch(color)
	{
		case RED:   return "RED";
		case GREEN: return "GREEN";
		case BLUE:  return "BLUE";
		default:    return "BAD";
	}
}

static void Flow_ShowYawFixed(void)
{
	char text[24];

	snprintf(text, sizeof(text), "YAW:%.2f", normalize_angle(imu.yaw));
	HMI_SetChassisText(text);
}

static void Flow_FineTuneHeading(float target_angle)
{
	HMI_SetChassisText("ALIGN");
	while(Chassis_FineTuneAngle(target_angle, RING_SWITCH_TURN_TIMEOUT) == 0)
	{
		HMI_LogWarn("align retry");
	}
	Motor_setspeed(0, 0, 0);
	Flow_ShowYawFixed();
}

uint8_t TaskFlow_RingLocateOne(uint8_t cls, char *name, float target_angle)
{
	VISION_TARGET_T target;
	uint32_t last_log_tick = 0;
	uint8_t stable_count = 0;

	HMI_SetVisionText(name);
	HMI_LogInfo("loc %s", name);
	Flow_FineTuneHeading(target_angle);
	Chassis_WorldBeginSegment();

	while(1)
	{
		int err_x;
		int err_y;
		float yaw_error;
		float vx = 0.0f;
		float vy = 0.0f;

		yaw_error = fabsf(getAngleZ(normalize_angle(imu.yaw), target_angle));
		if(yaw_error > RING_SWITCH_YAW_ERROR)
		{
			Motor_setspeed(0, 0, 0);
			Chassis_WorldCommitSegment(target_angle);
			HMI_SetChassisText("ALIGN");
			Chassis_FineTuneAngle(target_angle, RING_SWITCH_TURN_TIMEOUT);
			Chassis_WorldBeginSegment();
			stable_count = 0;
			continue;
		}

		if(Vision_GetRingTarget(cls, &target) == 0)
		{
			HMI_SetPixelError(0, 0, 0);
			Motor_setspeed(0, 0, 0);
			stable_count = 0;

			if((HAL_GetTick() - last_log_tick) >= 300)
			{
				last_log_tick = HAL_GetTick();
				Flow_ShowYawFixed();
				HMI_LogWarn("%s lost", name);
			}

			vTaskDelay(pdMS_TO_TICKS(20));
			continue;
		}

		err_x = RING_CENTER_X - (int)target.x;
		err_y = RING_CENTER_Y - (int)target.y;
		HMI_SetPixelError(err_x, err_y, 1);

		if(Flow_AbsInt(err_x) <= RING_LOCATE_DEADZONE &&
		   Flow_AbsInt(err_y) <= RING_LOCATE_DEADZONE)
		{
			Motor_setspeed(0, 0, 0);
			stable_count++;

			if(stable_count >= RING_LOCATE_STABLE_COUNT)
			{
				Motor_setspeed(0, 0, 0);
				vTaskDelay(pdMS_TO_TICKS(100));
				Chassis_WorldCommitSegment(target_angle);
				HMI_LogInfo("%s ok %03d,%03d", name, target.x, target.y);
				return 1;
			}
		}
		else
		{
			stable_count = 0;

			/* Camera Y error maps to chassis X; camera X error maps to chassis Y. */
			vx = Flow_PixelErrorSpeed(-err_y, RING_LOCATE_DEADZONE);
			vy = Flow_PixelErrorSpeed(err_x, RING_LOCATE_DEADZONE);

			Chassis_OpenLoop_SetTranslation(vx, vy, target_angle);
		}

		if((HAL_GetTick() - last_log_tick) >= 300)
		{
			last_log_tick = HAL_GetTick();
			Flow_ShowYawFixed();
			HMI_LogInfo("%s e%d,%d", name, err_x, err_y);
		}

		vTaskDelay(pdMS_TO_TICKS(20));
	}

}

void TaskFlow_RingSwitchY(float vy, float target_angle, float distance_cm)
{
	HMI_SetChassisText("SWITCH");
	Chassis_MoveByDistanceSmoothYaw(0, vy, target_angle, distance_cm);
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
		Flow_ShowYawFixed();

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
	float move_distance_cm;

	if(current_position < 0 || target_position < 0)
		return 0;

	distance = target_position - current_position;
	if(distance == 0)
		return 1;

	move_distance_cm = (float)Flow_AbsInt(distance) * RING_SWITCH_DISTANCE_CM;
	HMI_SetVisionText(Flow_ColorName(target_color));
	HMI_LogInfo("switch %s", Flow_ColorName(target_color));
	Ring_CorrectHeading(target_angle);
	TaskFlow_RingSwitchY(distance > 0 ? RING_SWITCH_SPEED : -RING_SWITCH_SPEED,
				 target_angle,
				 move_distance_cm);
	return 1;
}

uint8_t TaskFlow_PlaceFromWarehouseIndex(uint8_t warehouse_index, char *name,
											 uint16_t place_height)
{
	if(warehouse_index > 2)
	{
		HMI_LogError("bad wh %d", warehouse_index + 1);
		return 0;
	}

	HMI_SetArmText("TAKE WH");
	HMI_LogInfo("take %s wh%d", name, warehouse_index + 1);

	HMI_SetArmText("PUT RING");
	if(Circle_PlaceFromWarehouseAtHeight(warehouse_index, place_height) == 0)
		return 0;

	HMI_LogInfo("put %s ok", name);
	return 1;
}

static uint8_t Ring_PlaceFromWarehouse(uint8_t color, uint16_t place_height)
{
	uint8_t warehouse_index = Get_Warehouse_index_from_color(color);

	return TaskFlow_PlaceFromWarehouseIndex(warehouse_index, Flow_ColorName(color),
										 place_height);
}

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
	claw_move_2(open);
	vTaskDelay(pdMS_TO_TICKS(150));

	HMI_LogInfo("z safe");
	Z_SetHeight(0);
	claw_move_1(open);
}

static uint8_t Flow_QRWait(uint32_t wait_ms)
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

static uint8_t Flow_QRMove(float vx, float vy, float distance_cm)
{
	Chassis_MoveByDistance(vx, vy, 0.0f, distance_cm);
	return (first_code != 0 || second_code != 0);
}

static void Flow_WaitQR_OpenLoop(void)
{
	first_code = 0;
	second_code = 0;
	HMI_SetVisionText("WAIT QR");
	HMI_LogInfo("wait qr");

	while(1)
	{
		if(Flow_QRWait(1000)) break;

		/* 围绕二维码标称位置按 2 cm 搜索，每个方向检查后回到中心。 */
		if(Flow_QRMove(TEST_CHASSIS_SPEED_FINE, 0, YPJ_QR_SCAN_DISTANCE_CM)) break;
		if(Flow_QRWait(1000)) break;
		if(Flow_QRMove(-TEST_CHASSIS_SPEED_FINE, 0, YPJ_QR_SCAN_DISTANCE_CM)) break;

		if(Flow_QRMove(-TEST_CHASSIS_SPEED_FINE, 0, YPJ_QR_SCAN_DISTANCE_CM)) break;
		if(Flow_QRWait(1000)) break;
		if(Flow_QRMove(TEST_CHASSIS_SPEED_FINE, 0, YPJ_QR_SCAN_DISTANCE_CM)) break;

		if(Flow_QRMove(0, TEST_CHASSIS_SPEED_FINE, YPJ_QR_SCAN_DISTANCE_CM)) break;
		if(Flow_QRWait(1000)) break;
		if(Flow_QRMove(0, -TEST_CHASSIS_SPEED_FINE, YPJ_QR_SCAN_DISTANCE_CM)) break;

		if(Flow_QRMove(0, -TEST_CHASSIS_SPEED_FINE, YPJ_QR_SCAN_DISTANCE_CM)) break;
		if(Flow_QRWait(1000)) break;
		if(Flow_QRMove(0, TEST_CHASSIS_SPEED_FINE, YPJ_QR_SCAN_DISTANCE_CM)) break;
	}

	Motor_setspeed(0, 0, 0);
	Flow_ShowQRNow();
	HMI_SetVisionText("QR OK");
	HMI_LogInfo("qr %03d+%03d", first_code, second_code);
}

static void Yuanpanji_LocateOpenLoop(uint8_t cls, char *name, float target_angle,
									 uint8_t adjust_heading)
{
	VISION_TARGET_T target;
	uint32_t last_frame_count = vision_frame_count;
	uint8_t stable_count = 0;

	HMI_SetVisionText(name);
	HMI_LogInfo("wait %s", name);
	ypj_debug_stage = 11;
	if(adjust_heading)
		Flow_FineTuneHeading(target_angle);
	Chassis_WorldBeginSegment();

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

		if(Flow_AbsInt(err_x) > YPJ_TRACK_ROI ||
		   Flow_AbsInt(err_y) > YPJ_TRACK_ROI)
		{
			ypj_debug_stage = 14;
			stable_count = 0;
			Motor_setspeed(0, 0, 0);
			vTaskDelay(pdMS_TO_TICKS(20));
			continue;
		}

		if(Flow_AbsInt(err_x) <= YPJ_LOCATE_DEADZONE &&
		   Flow_AbsInt(err_y) <= YPJ_LOCATE_DEADZONE)
		{
			ypj_debug_stage = 15;
			Motor_setspeed(0, 0, 0);
			stable_count++;

			if(stable_count >= YPJ_LOCATE_STABLE_COUNT)
			{
				ypj_debug_stage = 20;
				Motor_setspeed(0, 0, 0);
				vTaskDelay(pdMS_TO_TICKS(100));
				Chassis_WorldCommitSegment(target_angle);
				HMI_LogInfo("%s located", name);
				return;
			}
		}
		else
		{
			stable_count = 0;

			vx = Flow_PixelErrorSpeed(-err_y, YPJ_LOCATE_DEADZONE);
			vy = Flow_PixelErrorSpeed(err_x, YPJ_LOCATE_DEADZONE);

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

/* 绿环为路径基准；最后停在红环少走15cm，停在蓝环多走15cm。 */
static float Ring_CompensatedDistance(float green_base_cm)
{
	int8_t current_position = Ring_Position(flow_current_ring);
	int8_t green_position = Ring_Position(GREEN);

	if(current_position < 0 || green_position < 0)
		return green_base_cm;

	return green_base_cm + (float)(current_position - green_position) *
						   RING_SWITCH_DISTANCE_CM;
}

static void Flow_LoadRound(uint8_t round)
{
	Init_Warehouse(round);
	if(round == 2)
	{
		flow_color_data[0] = (uint8_t)two.firse;
		flow_color_data[1] = (uint8_t)two.second;
		flow_color_data[2] = (uint8_t)two.thrid;
	}
	else
	{
		flow_color_data[0] = (uint8_t)one.firse;
		flow_color_data[1] = (uint8_t)one.second;
		flow_color_data[2] = (uint8_t)one.thrid;
	}

	HMI_LogInfo("R%d %s %s %s", round,
				 Flow_ColorName(flow_color_data[0]),
				 Flow_ColorName(flow_color_data[1]),
				 Flow_ColorName(flow_color_data[2]));
}

void Route_Path1_StartToQR(void)
{
	HMI_LogInfo("path1 start");
	Motor_setspeed(0, 0, 0);
	vTaskDelay(pdMS_TO_TICKS(100));
	Imu_setZero();
	vTaskDelay(pdMS_TO_TICKS(200));
	Chassis_WorldPoseReset(0.0f, 0.0f, 0.0f);
	Chassis_MoveByDistance(PATH1_LEFT_SPEED, 0, 0,
					   PATH1_LEFT_DISTANCE_CM);
	Chassis_MoveByDistance(0, PATH1_FORWARD_SPEED, 0,
					   PATH1_FORWARD_DISTANCE_CM);
	Motor_setspeed(0, 0, 0);
}

void Flow_QRRecognize(void)
{
	Flow_WaitQR_OpenLoop();
	Flow_LoadRound(1);
}

void Route_Path2_QRToTurntable(void)
{
	HMI_LogInfo("path2 start");
	Chassis_MoveByDistanceRamp(0, PATH2_FORWARD_SPEED, 0,
						   PATH2_FORWARD_DISTANCE_CM,
						   PATH2_ACCEL_TICKS,
						   PATH2_DECEL_TICKS);
	Chassis_MoveByPulse(-PATH2_RIGHT_SPEED, 0, 0,
					(int64_t)(PATH2_RIGHT_DISTANCE_CM *
							  CHASSIS_LATERAL_PULSE_PER_CM + 0.5f),
					0U);
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

		Yuanpanji_LocateOpenLoop(color, Flow_ColorName(color), 0.0f, 0);
		Motor_setspeed(0, 0, 0);
		ypj_debug_stage = 31;
		CatchLocatedMaterialToWarehouse(color, YUAN_PAN_HEIGHT);
		ypj_debug_stage = 32;
		flow_catch_count++;
		HMI_LogInfo("catch %s ok", Flow_ColorName(color));

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
	Chassis_MoveByDistance(0, -PATH3_BACK_SPEED, 0,
					   PATH3_BACK_DISTANCE_CM);
	Chassis_MoveByPulse(PATH3_LEFT_SPEED, 0, 0,
					(int64_t)(PATH3_LEFT_DISTANCE_CM *
							  CHASSIS_LATERAL_PULSE_PER_CM + 0.5f),
					0U);
	Chassis_TurnToAngle(PATH3_FIRST_ANGLE, YPJ_TURN_TIMEOUT_MS);
	Chassis_MoveByDistance(0, -PATH3_LONG_BACK_SPEED, PATH3_FIRST_ANGLE,
					   PATH3_LONG_BACK_CM);
	Chassis_TurnToAngle(PATH3_FINAL_ANGLE, YPJ_TURN_TIMEOUT_MS);
	Flow_ShowYawFixed();
}

static void Flow_RingArea(float target_heading, RING_WORK_LAYER_T work_layer,
						  uint8_t recover_after_place)
{
	uint16_t work_height = work_layer == RING_WORK_LAYER_SECOND ?
						   CIRCLE_SECOND_LAYER_HEIGHT : CIRCLE_PLACE_HEIGHT;
	uint8_t locate_existing_material =
		(work_layer == RING_WORK_LAYER_SECOND && recover_after_place == 0);
	uint8_t initial_located;

	if(flow_catch_count < 3)
	{
		HMI_LogError("caught %d", flow_catch_count);
		return;
	}

	flow_ring_place_count = 0;
	flow_ring_recover_count = 0;
	flow_current_ring = GREEN;

	USART6_readdata_SeetZero();
	Vision_LED_On();
	if(locate_existing_material)
	{
		Set_Circle_Center(YPJ_CENTER_X, YPJ_CENTER_Y);
		Vision_StartMaterial();
		vTaskDelay(pdMS_TO_TICKS(300));
		Circle_PrepareMaterialCatchPose();
		Yuanpanji_LocateOpenLoop(GREEN, "GREEN", target_heading, 1);
		initial_located = 1;
	}
	else
	{
		Set_Circle_Center(RING_CENTER_X, RING_CENTER_Y);
		Vision_StartRing();
		vTaskDelay(pdMS_TO_TICKS(300));
		Circle_PrepareDetectPose();
		initial_located = TaskFlow_RingLocateOne(GREEN, "GREEN", target_heading);
	}

	if(initial_located != 0)
	{
		for(uint8_t i = 0; i < 3; i++)
		{
			uint8_t color = flow_color_data[i];
			if(color != flow_current_ring)
			{
				if(Ring_SwitchToTarget(flow_current_ring, color, target_heading) == 0)
					break;
				if(locate_existing_material)
				{
					Yuanpanji_LocateOpenLoop(color, Flow_ColorName(color), target_heading, 1);
				}
				else if(RING_PLACE_MODE == RING_PLACE_MODE_LOCATE_EACH &&
						TaskFlow_RingLocateOne(color, Flow_ColorName(color), target_heading) == 0)
				{
					break;
				}
				flow_current_ring = color;
			}
			if(Ring_PlaceFromWarehouse(color, work_height) == 0)
				break;
			flow_ring_place_count++;
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
	if(recover_after_place == 0)
	{
		Vision_Stop();
		Vision_LED_Off();
		Motor_setspeed(0, 0, 0);
		Z_SetHeight(CIRCLE_SAFE_HEIGHT);
		Y_SetLength(0);
		M8010_SetAngle(0);
		HMI_LogInfo("area done");
		return;
	}

	USART6_readdata_SeetZero();
	Set_Circle_Center(YPJ_CENTER_X, YPJ_CENTER_Y);
	Vision_StartMaterial();
	vTaskDelay(pdMS_TO_TICKS(300));

	for(uint8_t i = 0; i < 3; i++)
	{
		uint8_t color = flow_color_data[i];
		if(color != flow_current_ring)
		{
			if(Ring_SwitchToTarget(flow_current_ring, color, target_heading) == 0)
				break;
			flow_current_ring = color;
		}

		Circle_PrepareMaterialCatchPose();
		Yuanpanji_LocateOpenLoop(color, Flow_ColorName(color), target_heading, 1);
		Motor_setspeed(0, 0, 0);
		CatchLocatedMaterialToWarehouse(color, work_height);
		flow_ring_recover_count++;
		HMI_LogInfo("recover %s ok", Flow_ColorName(color));
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

void Flow_ProcessingArea(void)
{
	HMI_LogInfo("processing area");
	Flow_RingArea(RING_ROUTE_HEADING, RING_WORK_LAYER_FIRST, 1);
}

void Flow_StorageArea(RING_WORK_LAYER_T work_layer)
{
	HMI_LogInfo("storage area");
	Flow_RingArea(PATH4_TURN_ANGLE, work_layer, 0);
}

void Route_Path4_ProcessingToNext(void)
{
	float compensated_back_cm;

	if(flow_ring_recover_count < 3)
		return;

	compensated_back_cm = Ring_CompensatedDistance(PATH4_BACK_DISTANCE_CM);

	HMI_LogInfo("path4 start");
	HMI_LogInfo("back %.0fcm", compensated_back_cm);
	Chassis_MoveByDistanceRamp(0, -PATH4_BACK_SPEED, PATH3_FINAL_ANGLE,
						   compensated_back_cm,
						   PATH4_BACK_ACCEL_TICKS,
						   PATH4_BACK_DECEL_TICKS);
	Chassis_MoveByPulse(PATH4_LEFT_SPEED, 0, PATH3_FINAL_ANGLE,
					(int64_t)(PATH4_LEFT_DISTANCE_CM *
							  CHASSIS_LATERAL_PULSE_PER_CM + 0.5f),
					0U);
	Chassis_TurnToAngle(PATH4_TURN_ANGLE, YPJ_TURN_TIMEOUT_MS);
	Chassis_MoveByDistance(0, PATH4_FORWARD_SPEED, PATH4_TURN_ANGLE,
					   PATH4_FORWARD_DISTANCE_CM);
	Motor_setspeed(0, 0, 0);
	HMI_LogInfo("path4 done");
}

void Route_Path5_StorageToTurntable(void)
{
	float compensated_back_cm;

	if(flow_ring_place_count < 3)
		return;

	compensated_back_cm = Ring_CompensatedDistance(PATH5_BACK_DISTANCE_CM);
	HMI_LogInfo("path5 start");
	HMI_LogInfo("back %.0fcm", compensated_back_cm);
	Chassis_MoveByDistance(0, -PATH5_BACK_SPEED, PATH4_TURN_ANGLE,
					   compensated_back_cm);
	Chassis_TurnToAngle(PATH5_TURN_ANGLE, YPJ_TURN_TIMEOUT_MS);
	Chassis_MoveByDistance(0, -PATH5_FINAL_BACK_SPEED, PATH5_TURN_ANGLE,
					   PATH5_FINAL_BACK_CM);
	Motor_setspeed(0, 0, 0);
	HMI_LogInfo("round2 ready");
}

void Route_Path6_StorageToHome(void)
{
	float compensated_back_cm;

	if(flow_ring_place_count < 3)
		return;

	compensated_back_cm = Ring_CompensatedDistance(PATH6_BACK_DISTANCE_CM);
	HMI_LogInfo("path6 home");
	HMI_LogInfo("back %.0fcm", compensated_back_cm);
	Chassis_MoveByDistanceRamp(0, -PATH6_BACK_SPEED, PATH4_TURN_ANGLE,
						   compensated_back_cm,
						   PATH6_BACK_ACCEL_TICKS,
						   PATH6_BACK_DECEL_TICKS);
	Chassis_MoveByPulse(PATH6_PRETURN_LEFT_SPEED, 0, PATH4_TURN_ANGLE,
					(int64_t)(PATH6_PRETURN_LEFT_CM *
							  CHASSIS_LATERAL_PULSE_PER_CM + 0.5f),
					0U);
	Chassis_TurnToAngle(PATH6_TURN_ANGLE, YPJ_TURN_TIMEOUT_MS);
	Chassis_MoveByDistance(0, -PATH6_LONG_BACK_SPEED, PATH6_TURN_ANGLE,
					   PATH6_LONG_BACK_CM);
	Chassis_MoveByDistance(-PATH6_RIGHT_SPEED, 0, PATH6_TURN_ANGLE,
					   PATH6_RIGHT_DISTANCE_CM);
	Motor_setspeed(0, 0, 0);
	HMI_SetChassisText("HOME");
	HMI_LogInfo("flow done");
}

uint8_t Flow_ArmPoseInit(void)
{
	vTaskDelay(pdMS_TO_TICKS(500));
	HMI_InitScreen();
	HMI_SetSys("ARM INIT", "RUN");
	HMI_SetChassisText("STOP");
	HMI_SetVisionText("OFF");

	HMI_SetArmText("Z INIT");
	HMI_LogInfo("init z %d", ARM_INIT_Z_OFFSET);
	if(!Z_InitMoveSigned(ARM_INIT_Z_OFFSET))
	{
		HMI_SetSys("ARM INIT", "Z TIMEOUT");
		HMI_SetArmText("Z ERROR");
		HMI_LogError("init z timeout");
		return 0;
	}

	HMI_SetArmText("Y INIT");
	HMI_LogInfo("init y %d", ARM_INIT_Y_OFFSET);
	if(!Y_InitMoveSigned(ARM_INIT_Y_OFFSET))
	{
		HMI_SetSys("ARM INIT", "Y TIMEOUT");
		HMI_SetArmText("Y ERROR");
		HMI_LogError("init y timeout");
		return 0;
	}

	HMI_SetArmText("JOINT INIT");
	HMI_LogInfo("joint %d deg", ARM_INIT_JOINT_ANGLE);
	M8010_SetAngle(ARM_INIT_JOINT_ANGLE);
	vTaskDelay(pdMS_TO_TICKS(ARM_INIT_JOINT_SETTLE_MS));

	HMI_SetArmText("SET ZERO");
	M8010_Set_Zero();
	YZ_SetZero(1);
	vTaskDelay(pdMS_TO_TICKS(ARM_INIT_ZERO_INTERVAL_MS));
	YZ_SetZero(2);
	vTaskDelay(pdMS_TO_TICKS(ARM_INIT_ZERO_INTERVAL_MS));

	HMI_SetSys("ARM INIT", "DONE");
	HMI_SetArmText("ZERO DONE");
	HMI_LogInfo("arm pose init done");
	return 1;
}

void Flow_RunCurrent(void)
{
	if(!Flow_ArmPoseInit())
		return;

	/* 完整流程入口负责一次性的显示初始化，阶段函数仍可独立调试。 */
	vTaskDelay(pdMS_TO_TICKS(500));
	HMI_InitScreen();
	HMI_LogInfo("task flow start");
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
	if(flow_ring_recover_count < 3)
		return;
	Route_Path4_ProcessingToNext();
	Flow_StorageArea(RING_WORK_LAYER_FIRST);
	if(flow_ring_place_count < 3)
		return;
	Route_Path5_StorageToTurntable();

	flow_catch_count = 0;
	Flow_LoadRound(2);
	Flow_TurntableCatch();
	Route_Path3_TurntableToProcessing();
	Flow_ProcessingArea();
	if(flow_ring_recover_count < 3)
		return;
	Route_Path4_ProcessingToNext();
	Flow_StorageArea(RING_WORK_LAYER_SECOND);
	if(flow_ring_place_count < 3)
		return;
	Route_Path6_StorageToHome();
}

