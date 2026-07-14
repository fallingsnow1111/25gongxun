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

unsigned char Calibration_Complete = 1;	
unsigned char Calibration_Complete_turn = 1;	
unsigned char W_Gray_openmv = 1;
volatile uint32_t move_to_target_last_wait_ms = 0;
volatile uint32_t move_to_target_timeout_count = 0;
volatile uint8_t move_to_target_last_timeout = 0;
volatile uint32_t chassis_period_overrun_count = 0;
volatile uint32_t chassis_period_max_ms = 0;

#define ratio_of_pulse_distance_y  1.44928f       //脉冲数与距离的比值
#define ratio_of_pulse_distance_x  1.51515f
#define ratio_of_pulse_angle  (float)(280/83.50)//脉冲数与距离的比值89.59
#define MOVE_TO_TARGET_TIMEOUT_MS 10000U
#define CHASSIS_TURN_THRESHOLD      0.1f /* 常规转向的到位误差，单位度。 */
#define CHASSIS_TURN_SETTLE_COUNT    10U /* 常规转向连续约50ms满足误差才算到位。 */
#define CHASSIS_TURN_MIN_SPEED      2.0f /* 原地转向克服电机死区的最小速度。 */
#define CHASSIS_TURN_ACCEL_DELTA    2.0f /* 原地转向每5ms最多增加2RPM。 */
#define CHASSIS_TURN_DECEL_DELTA    4.0f /* 原地转向每5ms最多减少4RPM。 */
#define CHASSIS_FINE_TURN_THRESHOLD 0.2f /* 航向精调的到位误差，单位度。 */
#define CHASSIS_FINE_TURN_SPEED     1.0f /* 航向精调专用的固定旋转速度。 */

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
static void Chassis_SINAccel(float vx1, float vy1, float vx2, float vy2,
							 float target_angle, uint16_t ramp_ticks,
							 TickType_t *last_wake, TickType_t *last_cycle)
{
	if(ramp_ticks == 0) return;

	for(uint16_t i = 0; i < ramp_ticks; i++)
	{
		float k = 0.5f - 0.5f * cosf(PI_F * i / ramp_ticks);
		float vx = vx1 + (vx2 - vx1) * k;
		float vy = vy1 + (vy2 - vy1) * k;

		Chassis_OpenLoop_SetSpeed(vx, vy, target_angle);
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
void Chassis_MoveOnce(float vx, float vy,float target_angle, uint16_t hold_ticks, uint16_t ramp_ticks)
{
	TickType_t last_wake;
	TickType_t last_cycle;
	int32_t actual_start[4];
	uint8_t actual_start_valid;

	actual_start_valid = Chassis_BeginSegment(actual_start);
	Chassis_OdomResetSegment();
	last_wake = xTaskGetTickCount();
	last_cycle = last_wake;
	Chassis_SINAccel(0, 0, vx, vy, target_angle, ramp_ticks,
					 &last_wake, &last_cycle);

	for(uint16_t i = 0; i < hold_ticks; i++)
	{
		Chassis_OpenLoop_SetSpeed(vx, vy, target_angle);
		Chassis_WaitPeriod(&last_wake, &last_cycle);
	}

	Chassis_SINAccel(vx, vy, 0, 0, target_angle, ramp_ticks,
					 &last_wake, &last_cycle);

	Motor_setspeed(0,0,0);
	vTaskDelay(pdMS_TO_TICKS(MOTOR_PULSE_READ_SETTLE_MS));
	Chassis_EndSegment(actual_start, actual_start_valid);
	Chassis_LogSegmentOdom();
	vTaskDelay(pdMS_TO_TICKS(50));
}

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

void Chassis_MoveByPulse(float vx, float vy, float target_angle,
						 int64_t target_pulse, uint16_t ramp_ticks)
{
	TickType_t last_wake;
	TickType_t last_cycle;
	int64_t decel_pulse;
	CHASSIS_ODOM_T odom;
	int32_t actual_start[4];
	uint8_t actual_start_valid;

	if(target_pulse <= 0 || (vx == 0.0f && vy == 0.0f))
	{
		Motor_setspeed(0, 0, 0);
		return;
	}

	actual_start_valid = Chassis_BeginSegment(actual_start);
	Chassis_OdomResetSegment();
	last_wake = xTaskGetTickCount();
	last_cycle = last_wake;
	decel_pulse = Chassis_EstimateDecelPulse(vx, vy, ramp_ticks);
	Chassis_SINAccel(0, 0, vx, vy, target_angle, ramp_ticks,
					 &last_wake, &last_cycle);

	while(1)
	{
		Chassis_OdomGetSegment(&odom);
		if(Chassis_GetMovePulse(&odom, vx, vy) + decel_pulse >= target_pulse)
			break;

		Chassis_OpenLoop_SetSpeed(vx, vy, target_angle);
		Chassis_WaitPeriod(&last_wake, &last_cycle);
	}

	Chassis_SINAccel(vx, vy, 0, 0, target_angle, ramp_ticks,
					 &last_wake, &last_cycle);
	Motor_setspeed(0, 0, 0);
	vTaskDelay(pdMS_TO_TICKS(MOTOR_PULSE_READ_SETTLE_MS));
	Chassis_EndSegment(actual_start, actual_start_valid);
	Chassis_LogSegmentOdom();
	vTaskDelay(pdMS_TO_TICKS(50));
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

/*
 *函数简介: 底盘原地转向到指定Yaw角度
 *参数说明: target_angle 目标Yaw角度, 单位: 度
 *参数说明: timeout_ms   超时时间, 单位: ms
 *返回类型: 无
 *备注: 使用IMU角度PID计算旋转速度, 连续多次进入角度误差范围后停车
 */
void Chassis_TurnToAngle(float target_angle, uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();
    uint8_t settle_count = 0;
	float turn_speed = 0.0f;
	float last_turn_speed = 0.0f;
	float speed_delta;
	float delta_limit;
	TickType_t last_wake = xTaskGetTickCount();
	TickType_t last_cycle = last_wake;

    while ((HAL_GetTick() - start_tick) < timeout_ms)
    {
        float current_angle = normalize_angle(imu.yaw);
        float angle_error = getAngleZ(current_angle, target_angle);

        if (fabsf(angle_error) <= CHASSIS_TURN_THRESHOLD)
        {
            settle_count++;
			last_turn_speed = 0.0f;
            Motor_setspeed(0, 0, 0);
            if (settle_count >= CHASSIS_TURN_SETTLE_COUNT)
            {
                break;
            }

			Chassis_WaitPeriod(&last_wake, &last_cycle);
            continue;
        }
        else
        {
            settle_count = 0;
        }

		turn_speed = Direction_Calibration_turn(target_angle);
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
		Motor_setspeed(0, 0, turn_speed);
		Chassis_WaitPeriod(&last_wake, &last_cycle);
    }

    Motor_setspeed(0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
}

/* Fine heading correction with a fixed minimum executable motor speed. */
uint8_t Chassis_FineTuneAngle(float target_angle, uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();
    uint8_t settle_count = 0;
	TickType_t last_wake = xTaskGetTickCount();
	TickType_t last_cycle = last_wake;

    while ((HAL_GetTick() - start_tick) < timeout_ms)
    {
        float current_angle = normalize_angle(imu.yaw);
        float angle_error = getAngleZ(current_angle, target_angle);

        if (fabsf(angle_error) <= CHASSIS_FINE_TURN_THRESHOLD)
        {
            Motor_setspeed(0, 0, 0);
            settle_count++;
            if (settle_count >= 3)
            {
                vTaskDelay(pdMS_TO_TICKS(100));
                return 1;
            }
        }
        else
        {
            settle_count = 0;
            Motor_setspeed(0, 0,
                           angle_error > 0.0f ?
                           CHASSIS_FINE_TURN_SPEED : -CHASSIS_FINE_TURN_SPEED);
        }

		Chassis_WaitPeriod(&last_wake, &last_cycle);
    }

    Motor_setspeed(0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    return 0;
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

void motor_read_coordination_all(void)
{
	for (uint8_t i = 1; i <= 4; i++)
	{
		motor_read_coordination(i);
		Delay_ms(3);
	}
}

//Odometer is turned on by default
void Move_To_Target_area(float x,float y,float angle,int imu_able,MODE_POSITION mode)
{	
	uint32_t start_tick;

	car.imu_modeable=(ABLE_T)imu_able;
	car.Odometer_able=enable;

	// 相对模式就清零编码器和imu
	if(mode==Relative_Position)
	{
		Motor_SetZero();
		Imu_setZero();
		Delay_ms(200);	//等imu稳定
	}

	// 绝对模式什么都不做
	car.target_x=(x*ratio_of_pulse_distance_x);
	car.target_y=(-y*ratio_of_pulse_distance_y);
	car.target_w=angle;
	MOTOR_ACTIONFALG=Incomplete;
	start_tick = HAL_GetTick();
	while (MOTOR_ACTIONFALG!=finish)
	{
		if ((HAL_GetTick() - start_tick) > MOVE_TO_TARGET_TIMEOUT_MS)
		{
			break;
		}
		vTaskDelay(pdMS_TO_TICKS(2));
	}

	move_to_target_last_wait_ms = HAL_GetTick() - start_tick;
	move_to_target_last_timeout = (MOTOR_ACTIONFALG != finish);
	if(move_to_target_last_timeout)
	{
		move_to_target_timeout_count++;
	}
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



