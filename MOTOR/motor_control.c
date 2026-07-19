#include "motor_control.h"
#include "delay.h"
#include "motor.h"
#include "pid.h"
#include "imu_control.h"
#include <stdio.h>
#include "math.h"
#include "TIME.h"
#include "IMU.h"
#include "hmi_task.h"

volatile uint32_t chassis_period_overrun_count = 0;
volatile uint32_t chassis_period_max_ms = 0;

#define CHASSIS_TURN_THRESHOLD     0.1f /* 常规转向的到位误差，单位度。 */
#define CHASSIS_TURN_SETTLE_COUNT    10U /* 角度连续约50ms满足阈值才算到位。 */
#define CHASSIS_TURN_MIN_SPEED      1.0f /* 双环最小输出，实车测试后再确定机械有效下限。 */
#define CHASSIS_TURN_ACCEL_DELTA    2.0f /* 原地转向每5ms最多增加2RPM。 */
#define CHASSIS_TURN_DECEL_DELTA    4.0f /* 原地转向每5ms最多减少4RPM。 */
#define CHASSIS_TURN_ANGLE_KP       3.0f /* 角度外环增益：角度误差转换为目标角速度。 */
#define CHASSIS_TURN_RATE_MAX     150.0f /* 角度外环最大目标角速度，单位度每秒。 */
#define CHASSIS_TURN_RATE_GAIN      1.055f /* 实测电机RPM到车身角速度的换算系数。 */
#define CHASSIS_TURN_RATE_KP        0.20f /* 角速度内环比例增益，输出单位为RPM。 */
#define CHASSIS_TURN_RATE_KI        0.15f /* 角速度内环积分增益，仅补偿稳态偏差。 */
#define CHASSIS_TURN_RATE_I_LIMIT  30.0f /* 角速度积分限幅，防止转向饱和时积分累积。 */
#define CHASSIS_TURN_RATE_FILTER    0.25f /* 角速度低通滤波系数，减小IMU瞬时波动。 */
#define CHASSIS_TURN_RPM_MAX      150.0f /* 原地转向允许输出的最大电机RPM。 */
#define CHASSIS_MOVE_YAW_THRESHOLD  0.1f /* 低速平移时允许的航向误差，单位度。 */
#define CHASSIS_MOVE_YAW_MAX_SPEED  2.0f /* 低速平移时最大航向修正速度。 */
#define CHASSIS_MOVE_YAW_DELTA      0.5f /* 低速平移每5ms最大航向速度变化。 */

#define CHASSIS_HEADING_NONE        0U
#define CHASSIS_HEADING_NORMAL      1U
#define CHASSIS_HEADING_SMOOTH      2U

#define OPEN_LOOP_PERIOD_MS 5 /* 开环速度更新周期，修改后所有路径周期参数都要重新标定。 */
#define MOTOR_PULSE_READ_SETTLE_MS 20U /* 停车后等待驱动器完成响应，再读取实际位置。 */
#define PI_F 3.1415926f			// 正弦加减速计算用圆周率

/*
 *函数简介: 底盘开环速度设置
 *参数说明: vx_world     世界坐标系横向速度, 向左为正
 *参数说明: vy_world     世界坐标系纵向速度, 向前为正
 *参数说明: target_angle 期望Yaw角度, 单位: 度
 *返回类型: 无
 *备注: 根据当前IMU角度将世界坐标速度转换为车体坐标速度
 *备注: 同时使用IMU航向PID计算旋转速度, 用于行进中保持目标航向
 *备注: 本函数只按速度控制底盘, 不读取编码器位置
 */
/* speed_frame_angle locks the translation frame; target_angle controls yaw. */
static void Chassis_WaitPeriod(TickType_t *last_wake, TickType_t *last_cycle)
{
	TickType_t now;
	uint32_t period_ms;

	vTaskDelayUntil(last_wake, pdMS_TO_TICKS(OPEN_LOOP_PERIOD_MS));
	now = xTaskGetTickCount();
	period_ms = (uint32_t)((now - *last_cycle) * portTICK_PERIOD_MS);
	*last_cycle = now;

	if(period_ms > chassis_period_max_ms)
		chassis_period_max_ms = period_ms;
	if(period_ms > OPEN_LOOP_PERIOD_MS)
		chassis_period_overrun_count++;
}

static void Chassis_LogSegmentOdom(void)
{
	CHASSIS_ODOM_T odom;

	Chassis_OdomGetSegment(&odom);
	HMI_LogInfo("P X%ld Y%ld %lums", (long)odom.x, (long)odom.y,
				(unsigned long)odom.move_time_ms);
}

/* 段内只累计软件脉冲，段结束后统一换算为毫米世界坐标。 */
void Chassis_WorldBeginSegment(void)
{
	Chassis_OdomResetSegment();
}

void Chassis_WorldCommitSegment(float segment_yaw)
{
	CHASSIS_ODOM_T odom;
	float body_x_mm;
	float body_y_mm;
	float yaw_rad;
	float world_dx_mm;
	float world_dy_mm;

	Chassis_OdomGetSegment(&odom);
	body_x_mm = (float)odom.x * 10.0f / CHASSIS_LATERAL_PULSE_PER_CM;
	body_y_mm = (float)odom.y * 10.0f / CHASSIS_LONGITUDINAL_PULSE_PER_CM;
	yaw_rad = segment_yaw * PI_F / 180.0f;
	world_dx_mm = body_x_mm * cosf(yaw_rad) + body_y_mm * sinf(yaw_rad);
	world_dy_mm = -body_x_mm * sinf(yaw_rad) + body_y_mm * cosf(yaw_rad);

	taskENTER_CRITICAL();
	car.actual_x += world_dx_mm;
	car.actual_y += world_dy_mm;
	taskEXIT_CRITICAL();

	Chassis_OdomResetSegment();
}

static uint8_t Chassis_BeginSegment(int32_t actual_start[4])
{
	HMI_SetMotorCompare(0, 0, 0, 0, 0);
	if(Motor_ReadPulseSnapshot(actual_start) == 0)
	{
		HMI_LogWarn("motor pulse start lost");
		return 0;
	}
	return 1;
}

static void Chassis_EndSegment(const int32_t actual_start[4], uint8_t start_valid)
{
	int32_t actual_end[4];
	int64_t delta[4];
	int64_t actual_x;
	int64_t actual_y;
	int64_t software_x;
	int64_t software_y;
	CHASSIS_ODOM_T odom;

	if(start_valid == 0 || Motor_ReadPulseSnapshot(actual_end) == 0)
	{
		HMI_SetMotorCompare(0, 0, 0, 0, 0);
		HMI_LogWarn("motor pulse end lost");
		return;
	}

	for(uint8_t i = 0; i < 4; i++)
		delta[i] = (int64_t)actual_end[i] - actual_start[i];
	actual_x = (-delta[0] + delta[1] + delta[2] - delta[3]) / 4;
	actual_y = -(delta[0] + delta[1] - delta[2] - delta[3]) / 4;

	Chassis_OdomGetSegment(&odom);
	software_x = odom.x * 65536LL / 60000LL;
	software_y = odom.y * 65536LL / 60000LL;
	HMI_SetMotorCompare((int32_t)actual_x, (int32_t)actual_y,
						(int32_t)(actual_x - software_x),
						(int32_t)(actual_y - software_y), 1);
}

void Chassis_OpenLoop_SetSpeedFrame(float vx_world, float vy_world,
                                    float speed_frame_angle, float target_angle)
{
	float yaw = normalize_angle(imu.yaw);
	float yaw_err = speed_frame_angle - yaw;

	while(yaw_err > 180.0f) yaw_err -= 360.0f;
	while(yaw_err < -180.0f) yaw_err += 360.0f;

	float rad = yaw_err * PI_F / 180.0f;

	float vx_body =  vx_world * cosf(rad) + vy_world * sinf(rad);
	float vy_body = -vx_world * sinf(rad) + vy_world * cosf(rad);

	float vw = Direction_Calibration_turn(target_angle);

	Motor_setspeed(-vy_body, vx_body, vw);
}

void Chassis_OpenLoop_SetSpeed(float vx_world, float vy_world, float target_angle)
{
	Chassis_OpenLoop_SetSpeedFrame(vx_world, vy_world, target_angle, target_angle);
}

/* Translate in the selected world frame without applying yaw correction. */
void Chassis_OpenLoop_SetTranslation(float vx_world, float vy_world,
                                     float speed_frame_angle)
{
	float yaw = normalize_angle(imu.yaw);
	float yaw_err = speed_frame_angle - yaw;
	float rad;
	float vx_body;
	float vy_body;

	while(yaw_err > 180.0f) yaw_err -= 360.0f;
	while(yaw_err < -180.0f) yaw_err += 360.0f;

	rad = yaw_err * PI_F / 180.0f;
	vx_body =  vx_world * cosf(rad) + vy_world * sinf(rad);
	vy_body = -vx_world * sinf(rad) + vy_world * cosf(rad);

	Motor_setspeed(-vy_body, vx_body, 0);
}

/*
 *函数简介: 底盘正弦加减速
 *参数说明: vx1          起始横向速度, 向左为正
 *参数说明: vy1          起始纵向速度, 向前为正
 *参数说明: vx2          目标横向速度, 向左为正
 *参数说明: vy2          目标纵向速度, 向前为正
 *参数说明: target_angle 期望Yaw角度, 单位: 度
 *参数说明: ramp_ticks   加减速控制周期数
 *返回类型: 无
 *备注: 实际加减速时间 = ramp_ticks * OPEN_LOOP_PERIOD_MS
 *备注: 采用正弦曲线插值, 减小起步和停车时的打滑
 */
static void Chassis_OpenLoop_SetSmoothHeading(float vx_world, float vy_world,
										  float target_angle,
										  float *last_vw)
{
	float yaw = normalize_angle(imu.yaw);
	float yaw_err = target_angle - yaw;
	float angle_error;
	float rad;
	float vx_body;
	float vy_body;
	float desired_vw;
	float speed_delta;

	while(yaw_err > 180.0f) yaw_err -= 360.0f;
	while(yaw_err < -180.0f) yaw_err += 360.0f;
	rad = yaw_err * PI_F / 180.0f;
	vx_body = vx_world * cosf(rad) + vy_world * sinf(rad);
	vy_body = -vx_world * sinf(rad) + vy_world * cosf(rad);

	angle_error = getAngleZ(yaw, target_angle);
	if(fabsf(angle_error) <= CHASSIS_MOVE_YAW_THRESHOLD)
	{
		desired_vw = 0.0f;
	}
	else
	{
		desired_vw = Direction_Calibration_turn(target_angle);
		desired_vw = desired_vw >= 0.0f ?
					 CHASSIS_MOVE_YAW_MAX_SPEED :
					-CHASSIS_MOVE_YAW_MAX_SPEED;
	}

	speed_delta = desired_vw - *last_vw;
	if(speed_delta > CHASSIS_MOVE_YAW_DELTA)
		speed_delta = CHASSIS_MOVE_YAW_DELTA;
	else if(speed_delta < -CHASSIS_MOVE_YAW_DELTA)
		speed_delta = -CHASSIS_MOVE_YAW_DELTA;
	*last_vw += speed_delta;

	Motor_setspeed(-vy_body, vx_body, *last_vw);
}

static void Chassis_ApplyMoveSpeed(float vx, float vy, float target_angle,
								  uint8_t heading_control,
								  float *last_vw)
{
	if(heading_control == CHASSIS_HEADING_NORMAL)
		Chassis_OpenLoop_SetSpeed(vx, vy, target_angle);
	else if(heading_control == CHASSIS_HEADING_SMOOTH)
		Chassis_OpenLoop_SetSmoothHeading(vx, vy, target_angle, last_vw);
	else
		Chassis_OpenLoop_SetTranslation(vx, vy, target_angle);
}

static void Chassis_SINAccel(float vx1, float vy1, float vx2, float vy2,
							 float target_angle, uint16_t ramp_ticks,
							 TickType_t *last_wake, TickType_t *last_cycle,
							 uint8_t heading_control, float *last_vw)
{
	if(ramp_ticks == 0) return;

	for(uint16_t i = 0; i < ramp_ticks; i++)
	{
		float k = 0.5f - 0.5f * cosf(PI_F * i / ramp_ticks);
		float vx = vx1 + (vx2 - vx1) * k;
		float vy = vy1 + (vy2 - vy1) * k;

		Chassis_ApplyMoveSpeed(vx, vy, target_angle,
						   heading_control, last_vw);
		Chassis_WaitPeriod(last_wake, last_cycle);
	}
}

/*
 *函数简介: 底盘开环单段移动
 *参数说明: vx           匀速段横向速度, 向左为正
 *参数说明: vy           匀速段纵向速度, 向前为正
 *参数说明: target_angle 期望Yaw角度, 单位: 度
 *参数说明: hold_ticks   匀速保持控制周期数
 *参数说明: ramp_ticks   加速/减速控制周期数
 *返回类型: 无
 *备注: 移动流程为 正弦加速 -> 匀速保持 -> 正弦减速 -> 停车
 *备注: 实际匀速时间 = hold_ticks * OPEN_LOOP_PERIOD_MS
 *备注: 本函数为开环距离控制, 距离由速度和时间标定决定
 */
static int64_t Chassis_AbsPulse(int64_t value)
{
	return value < 0 ? -value : value;
}

static int64_t Chassis_GetMovePulse(const CHASSIS_ODOM_T *odom,
									float vx, float vy)
{
	return fabsf(vx) >= fabsf(vy) ?
		   Chassis_AbsPulse(odom->x) : Chassis_AbsPulse(odom->y);
}

static int64_t Chassis_EstimateDecelPulse(float vx, float vy,
									  uint16_t ramp_ticks)
{
	float axis_speed = fabsf(vx) >= fabsf(vy) ? fabsf(vx) : fabsf(vy);
	int64_t pulse = 0;

	for(uint16_t i = 0; i < ramp_ticks; i++)
	{
		float k = 0.5f + 0.5f * cosf(PI_F * i / ramp_ticks);
		pulse += (int32_t)(axis_speed * k) * OPEN_LOOP_PERIOD_MS;
	}
	return pulse;
}

static void Chassis_MoveByPulseInternal(float vx, float vy, float target_angle,
									   int64_t target_pulse,
									   uint16_t accel_ticks,
									   uint16_t decel_ticks,
									   uint8_t heading_control)
{
	TickType_t last_wake;
	TickType_t last_cycle;
	int64_t decel_pulse;
	CHASSIS_ODOM_T odom;
	int32_t actual_start[4];
	uint8_t actual_start_valid;
	float last_vw = 0.0f;

	if(target_pulse <= 0 || (vx == 0.0f && vy == 0.0f))
	{
		Motor_setspeed(0, 0, 0);
		return;
	}

	actual_start_valid = Chassis_BeginSegment(actual_start);
	Chassis_WorldBeginSegment();
	last_wake = xTaskGetTickCount();
	last_cycle = last_wake;
	decel_pulse = Chassis_EstimateDecelPulse(vx, vy, decel_ticks);
	Chassis_SINAccel(0, 0, vx, vy, target_angle, accel_ticks,
					 &last_wake, &last_cycle, heading_control, &last_vw);

	while(1)
	{
		Chassis_OdomGetSegment(&odom);
		if(Chassis_GetMovePulse(&odom, vx, vy) + decel_pulse >= target_pulse)
			break;

		Chassis_ApplyMoveSpeed(vx, vy, target_angle,
						   heading_control, &last_vw);
		Chassis_WaitPeriod(&last_wake, &last_cycle);
	}

	Chassis_SINAccel(vx, vy, 0, 0, target_angle, decel_ticks,
					 &last_wake, &last_cycle, heading_control, &last_vw);
	Motor_setspeed(0, 0, 0);
	vTaskDelay(pdMS_TO_TICKS(MOTOR_PULSE_READ_SETTLE_MS));
	Chassis_EndSegment(actual_start, actual_start_valid);
	Chassis_LogSegmentOdom();
	Chassis_WorldCommitSegment(target_angle);
	vTaskDelay(pdMS_TO_TICKS(50));
}

void Chassis_MoveByPulse(float vx, float vy, float target_angle,
						 int64_t target_pulse, uint16_t ramp_ticks)
{
	Chassis_MoveByPulseInternal(vx, vy, target_angle, target_pulse,
								ramp_ticks, ramp_ticks,
								CHASSIS_HEADING_NORMAL);
}

void Chassis_MoveByDistanceRamp(float vx, float vy, float target_angle,
								float distance_cm,
								uint16_t accel_ticks,
								uint16_t decel_ticks)
{
	float pulse_per_cm;
	int64_t target_pulse;

	if(distance_cm <= 0.0f || (vx == 0.0f && vy == 0.0f))
	{
		Motor_setspeed(0, 0, 0);
		return;
	}

	if(vx != 0.0f && vy != 0.0f)
	{
		HMI_LogError("distance axis error");
		Motor_setspeed(0, 0, 0);
		return;
	}

	pulse_per_cm = vx != 0.0f ?
				   CHASSIS_LATERAL_PULSE_PER_CM :
				   CHASSIS_LONGITUDINAL_PULSE_PER_CM;
	target_pulse = (int64_t)(distance_cm * pulse_per_cm + 0.5f);

	Chassis_MoveByPulseInternal(vx, vy, target_angle, target_pulse,
								accel_ticks, decel_ticks,
								CHASSIS_HEADING_NORMAL);
}

/*
 * 单轴直线距离接口：vx 控制左右，vy 控制前后，distance_cm 始终传正数。
 * 当加速和减速均为 100 tick（500ms）时，完整加减速所需的最小行程为：
 *   速度 20  约需  4cm；
 *   速度 40  约需  8cm；
 *   速度 80  约需 16cm；
 *   速度 160 约需 32cm。
 * 建议速度档位：5cm 用 20，10cm 用 40，20cm 用 80，40cm 以上可用 160。
 * 最大速度近似公式：speed_max = distance_cm * pulse_per_cm / (ramp_ticks * 5ms)。
 * 传入速度超过该上限时，完整加减速距离会超过目标距离。
 * 视觉微调继续使用低速速度控制，不调用本距离接口。
 */
void Chassis_MoveByDistance(float vx, float vy, float target_angle,
						   float distance_cm)
{
	float pulse_per_cm;
	int64_t target_pulse;
	uint16_t ramp_ticks;

	if(distance_cm <= 0.0f || (vx == 0.0f && vy == 0.0f))
	{
		Motor_setspeed(0, 0, 0);
		return;
	}

	/* 只允许单轴直线，避免把未标定的斜行套用到直线系数。 */
	if(vx != 0.0f && vy != 0.0f)
	{
		HMI_LogError("distance axis error");
		Motor_setspeed(0, 0, 0);
		return;
	}

	pulse_per_cm = vx != 0.0f ?
				   CHASSIS_LATERAL_PULSE_PER_CM :
				   CHASSIS_LONGITUDINAL_PULSE_PER_CM;
	ramp_ticks = distance_cm >= CHASSIS_LONG_ROUTE_MIN_CM ?
				 CHASSIS_LONG_ROUTE_RAMP_TICKS :
				 CHASSIS_SHORT_ROUTE_RAMP_TICKS;
	target_pulse = (int64_t)(distance_cm * pulse_per_cm + 0.5f);

	Chassis_MoveByPulse(vx, vy, target_angle, target_pulse, ramp_ticks);
}

void Chassis_MoveByDistanceSmoothYaw(float vx, float vy, float target_angle,
									float distance_cm)
{
	float pulse_per_cm;
	int64_t target_pulse;
	uint16_t ramp_ticks;

	if(distance_cm <= 0.0f || (vx == 0.0f && vy == 0.0f) ||
	   (vx != 0.0f && vy != 0.0f))
	{
		Motor_setspeed(0, 0, 0);
		return;
	}

	pulse_per_cm = vx != 0.0f ?
				   CHASSIS_LATERAL_PULSE_PER_CM :
				   CHASSIS_LONGITUDINAL_PULSE_PER_CM;
	ramp_ticks = distance_cm >= CHASSIS_LONG_ROUTE_MIN_CM ?
				 CHASSIS_LONG_ROUTE_RAMP_TICKS :
				 CHASSIS_SHORT_ROUTE_RAMP_TICKS;
	target_pulse = (int64_t)(distance_cm * pulse_per_cm + 0.5f);

	Chassis_MoveByPulseInternal(vx, vy, target_angle, target_pulse,
								ramp_ticks, ramp_ticks,
								CHASSIS_HEADING_SMOOTH);
}

/*
 *函数简介: 底盘原地转向到指定Yaw角度
 *参数说明: target_angle 目标Yaw角度, 单位: 度
 *参数说明: timeout_ms   超时时间, 单位: ms
 *返回类型: 1表示在超时前到位，0表示超时
 *备注: 使用IMU角度PID计算旋转速度, 连续多次进入角度误差范围后停车
 */
static uint8_t Chassis_TurnToAngleOnce(float target_angle, uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();
    uint8_t settle_count = 0;
	float turn_speed = 0.0f;
	float last_turn_speed = 0.0f;
	float speed_delta;
	float delta_limit;
	float rate_integral = 0.0f;
	float filtered_rate = -imu.angular_rate;
	TickType_t last_wake = xTaskGetTickCount();
	TickType_t last_cycle = last_wake;

    while ((HAL_GetTick() - start_tick) < timeout_ms)
    {
        float current_angle = normalize_angle(imu.yaw);
        float angle_error = getAngleZ(current_angle, target_angle);
        float measured_rate = -imu.angular_rate;

        filtered_rate += CHASSIS_TURN_RATE_FILTER *
                         (measured_rate - filtered_rate);

        if (fabsf(angle_error) <= CHASSIS_TURN_THRESHOLD)
        {
			settle_count++;
			last_turn_speed = 0.0f;
            Motor_setspeed_fine(0, 0, 0);
            if (settle_count >= CHASSIS_TURN_SETTLE_COUNT)
            {
                Motor_setspeed_fine(0, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(100));
                return 1U;
            }

			Chassis_WaitPeriod(&last_wake, &last_cycle);
            continue;
        }
		else
		{
			settle_count = 0;
		}
		{
			float target_rate = angle_error * CHASSIS_TURN_ANGLE_KP;
			float rate_error;

			if(target_rate > CHASSIS_TURN_RATE_MAX)
				target_rate = CHASSIS_TURN_RATE_MAX;
			else if(target_rate < -CHASSIS_TURN_RATE_MAX)
				target_rate = -CHASSIS_TURN_RATE_MAX;

			rate_error = target_rate - filtered_rate;
			rate_integral += rate_error * (OPEN_LOOP_PERIOD_MS / 1000.0f);
			if(rate_integral > CHASSIS_TURN_RATE_I_LIMIT)
				rate_integral = CHASSIS_TURN_RATE_I_LIMIT;
			else if(rate_integral < -CHASSIS_TURN_RATE_I_LIMIT)
				rate_integral = -CHASSIS_TURN_RATE_I_LIMIT;

			turn_speed = target_rate / CHASSIS_TURN_RATE_GAIN +
						 CHASSIS_TURN_RATE_KP * rate_error +
						 CHASSIS_TURN_RATE_KI * rate_integral;
			if(turn_speed > CHASSIS_TURN_RPM_MAX)
				turn_speed = CHASSIS_TURN_RPM_MAX;
			else if(turn_speed < -CHASSIS_TURN_RPM_MAX)
				turn_speed = -CHASSIS_TURN_RPM_MAX;
		}
		delta_limit = (turn_speed * last_turn_speed >= 0.0f &&
					   fabsf(turn_speed) > fabsf(last_turn_speed)) ?
					  CHASSIS_TURN_ACCEL_DELTA : CHASSIS_TURN_DECEL_DELTA;
		speed_delta = turn_speed - last_turn_speed;
		if(speed_delta > delta_limit)
			speed_delta = delta_limit;
		else if(speed_delta < -delta_limit)
			speed_delta = -delta_limit;
		turn_speed = last_turn_speed + speed_delta;

		if(fabsf(turn_speed) < CHASSIS_TURN_MIN_SPEED)
		{
			turn_speed = angle_error > 0.0f ?
						 CHASSIS_TURN_MIN_SPEED : -CHASSIS_TURN_MIN_SPEED;
		}

		last_turn_speed = turn_speed;
		Motor_setspeed_fine(0, 0, turn_speed);
		Chassis_WaitPeriod(&last_wake, &last_cycle);
    }

    Motor_setspeed_fine(0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    return 0U;
}

/* 色环姿态微调复用原地转向双环，返回1表示最终角度满足停车阈值。 */
uint8_t Chassis_TurnToAngle(float target_angle, uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();
    uint32_t elapsed_ms;
    uint32_t remaining_ms;
    float angle_error;

    if (Chassis_TurnToAngleOnce(target_angle, timeout_ms) == 0U)
    {
        return 0U;
    }

    /* Stop first, then verify the settled IMU angle before leaving the turn. */
    angle_error = getAngleZ(normalize_angle(imu.yaw), target_angle);
    if (fabsf(angle_error) <= CHASSIS_TURN_THRESHOLD)
    {
        return 1U;
    }

    elapsed_ms = HAL_GetTick() - start_tick;
    if (elapsed_ms >= timeout_ms)
    {
        return 0U;
    }

    remaining_ms = timeout_ms - elapsed_ms;
    return Chassis_TurnToAngleOnce(target_angle, remaining_ms);
}

uint8_t Chassis_FineTuneAngle(float target_angle, uint32_t timeout_ms)
{
    float angle_error;

    Chassis_TurnToAngle(target_angle, timeout_ms);
    angle_error = getAngleZ(normalize_angle(imu.yaw), target_angle);

    return (fabsf(angle_error) <= CHASSIS_TURN_THRESHOLD) ? 1U : 0U;
}

/*
 *函数简介: 底盘开环边走边转单段移动
 *参数说明: vx          匀速段横向速度, 向左为正
 *参数说明: vy          匀速段纵向速度, 向前为正
 *参数说明: start_angle 起始目标Yaw角度, 单位: 度
 *参数说明: end_angle   结束目标Yaw角度, 单位: 度
 *参数说明: hold_ticks  匀速保持控制周期数
 *参数说明: ramp_ticks  加速/减速控制周期数
 *返回类型: 无
 *备注: 移动速度按正弦曲线加减速, 目标角度按时间线性插值
 */
void Chassis_MoveTurnOnce(float vx, float vy, float start_angle, float end_angle,
                          uint16_t hold_ticks, uint16_t ramp_ticks)
{
    uint16_t total_ticks = hold_ticks + ramp_ticks * 2;
    float angle_delta = end_angle - start_angle;
	TickType_t last_wake = xTaskGetTickCount();
	TickType_t last_cycle = last_wake;

    while (angle_delta > 180.0f) angle_delta -= 360.0f;
    while (angle_delta < -180.0f) angle_delta += 360.0f;

    if (total_ticks == 0)
    {
        return;
    }

    for (uint16_t i = 0; i < total_ticks; i++)
    {
        float speed_k = 1.0f;
        float progress = 0.0f;
        float target_angle;
        float vx_now;
        float vy_now;

        if (ramp_ticks > 0 && i < ramp_ticks)
        {
            speed_k = 0.5f - 0.5f * cosf(PI_F * i / ramp_ticks);
        }
        else if (ramp_ticks > 0 && i >= (ramp_ticks + hold_ticks))
        {
            uint16_t decel_i = i - ramp_ticks - hold_ticks;
            speed_k = 0.5f + 0.5f * cosf(PI_F * decel_i / ramp_ticks);
        }

        if (total_ticks > 1)
        {
            progress = (float)i / (float)(total_ticks - 1);
        }

        target_angle = start_angle + angle_delta * progress;
        vx_now = vx * speed_k;
        vy_now = vy * speed_k;

        Chassis_OpenLoop_SetSpeed(vx_now, vy_now, target_angle);
		Chassis_WaitPeriod(&last_wake, &last_cycle);
    }

    Motor_setspeed(0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
}

/*
 *函数简介: 底盘开环保持指定速度和航向
 *参数说明: vx           世界坐标系横向速度, 向左为正
 *参数说明: vy           世界坐标系纵向速度, 向前为正
 *参数说明: target_angle 期望Yaw角度, 单位: 度
 *参数说明: hold_ticks   保持控制周期数
 *返回类型: 无
 *备注: 实际保持时间 = hold_ticks * OPEN_LOOP_PERIOD_MS
 *备注: 本函数不会停车, 用于多段路径之间连续衔接
 */
void Chassis_HoldSpeedAngle(float vx, float vy, float target_angle, uint16_t hold_ticks)
{
	TickType_t last_wake = xTaskGetTickCount();
	TickType_t last_cycle = last_wake;

    for (uint16_t i = 0; i < hold_ticks; i++)
    {
        Chassis_OpenLoop_SetSpeed(vx, vy, target_angle);
		Chassis_WaitPeriod(&last_wake, &last_cycle);
    }
}

/*
 *函数简介: 底盘开环平滑切换速度和航向
 *参数说明: vx1          起始横向速度, 向左为正
 *参数说明: vy1          起始纵向速度, 向前为正
 *参数说明: angle1       起始目标Yaw角度, 单位: 度
 *参数说明: vx2          结束横向速度, 向左为正
 *参数说明: vy2          结束纵向速度, 向前为正
 *参数说明: angle2       结束目标Yaw角度, 单位: 度
 *参数说明: blend_ticks  过渡控制周期数
 *返回类型: 无
 *备注: 实际过渡时间 = blend_ticks * OPEN_LOOP_PERIOD_MS
 *备注: 目标角度按最短路径插值, 速度按正弦曲线插值
 *备注: 本函数不会停车, 适合漂移过弯和连续跑图
 */
void Chassis_BlendSpeedAngle(float vx1, float vy1, float angle1,
                             float vx2, float vy2, float angle2,
                             uint16_t blend_ticks)
{
    if (blend_ticks == 0) return;

    float angle_delta = angle2 - angle1;
	TickType_t last_wake = xTaskGetTickCount();
	TickType_t last_cycle = last_wake;

    while (angle_delta > 180.0f) angle_delta -= 360.0f;
    while (angle_delta < -180.0f) angle_delta += 360.0f;

    for (uint16_t i = 0; i < blend_ticks; i++)
    {
        float k = 0.5f - 0.5f * cosf(PI_F * i / blend_ticks);

        float vx = vx1 + (vx2 - vx1) * k;
        float vy = vy1 + (vy2 - vy1) * k;
        float target_angle = angle1 + angle_delta * k;

        Chassis_OpenLoop_SetSpeed(vx, vy, target_angle);
		Chassis_WaitPeriod(&last_wake, &last_cycle);
    }
}

void Chassis_DriftStraightTurn(float vx_world, float vy_world,
                               float speed_frame_angle,
                               float start_angle, float end_angle,
                               uint16_t turn_ticks)
{
    if (turn_ticks == 0) return;

    float angle_delta = end_angle - start_angle;
	TickType_t last_wake = xTaskGetTickCount();
	TickType_t last_cycle = last_wake;

    while (angle_delta > 180.0f) angle_delta -= 360.0f;
    while (angle_delta < -180.0f) angle_delta += 360.0f;

    for (uint16_t i = 0; i < turn_ticks; i++)
    {
        float t = (float)(i + 1) / (float)turn_ticks;
        float k = 0.5f - 0.5f * cosf(PI_F * t);
        float target_angle = start_angle + angle_delta * k;

        Chassis_OpenLoop_SetSpeedFrame(vx_world, vy_world,
                                       speed_frame_angle,
                                       target_angle);
		Chassis_WaitPeriod(&last_wake, &last_cycle);
    }
}

float FMy_Abs(float temp)
{
	if(temp<0)return -temp;
	else return temp;
}
