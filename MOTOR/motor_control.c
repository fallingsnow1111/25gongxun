#include "motor_control.h"
#include "delay.h"
#include "motor.h"
#include "pid.h"
#include "imu_control.h"
#include <stdio.h>
#include "math.h"
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

#define OPEN_LOOP_PERIOD_MS 5	// 开环速度控制周期, 单位: ms
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
void Chassis_OpenLoop_SetSpeed(float vx_world, float vy_world, float target_angle)
{
	float yaw = normalize_angle(imu.yaw);
	float yaw_err = yaw - target_angle;

	while(yaw_err > 180.0f) yaw_err -= 360.0f;
	while(yaw_err < -180.0f) yaw_err += 360.0f;

	float rad = yaw_err * PI_F / 180.0f;

	float vx_body =  vx_world * cosf(rad) + vy_world * sinf(rad);
	float vy_body = -vx_world * sinf(rad) + vy_world * cosf(rad);

	float vw = Direction_Calibration_turn(target_angle);

	Motor_setspeed(-vy_body, vx_body, vw);
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
static void Chassis_SINAccel(float vx1, float vy1, float vx2, float vy2, float target_angle, uint16_t ramp_ticks)
{
	if(ramp_ticks == 0) return;

	for(uint16_t i = 0; i < ramp_ticks; i++)
	{
		float k = 0.5f - 0.5f * cosf(PI_F * i / ramp_ticks);
		float vx = vx1 + (vx2 - vx1) * k;
		float vy = vy1 + (vy2 - vy1) * k;

		Chassis_OpenLoop_SetSpeed(vx, vy, target_angle);
		vTaskDelay(pdMS_TO_TICKS(OPEN_LOOP_PERIOD_MS));
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
	Chassis_SINAccel(0, 0, vx, vy, target_angle, ramp_ticks);

	for(uint16_t i = 0; i < hold_ticks; i++)
	{
		Chassis_OpenLoop_SetSpeed(vx, vy, target_angle);
		vTaskDelay(pdMS_TO_TICKS(OPEN_LOOP_PERIOD_MS));
	}

	Chassis_SINAccel(vx, vy, 0, 0, target_angle, ramp_ticks);

	Motor_setspeed(0,0,0);
	vTaskDelay(pdMS_TO_TICKS(50));
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

    while ((HAL_GetTick() - start_tick) < timeout_ms)
    {
        float current_angle = normalize_angle(imu.yaw);
        float angle_error = getAngleZ(current_angle, target_angle);

        if (fabsf(angle_error) <= 0.2f)
        {
            settle_count++;
            if (settle_count >= 3)
            {
                break;
            }
        }
        else
        {
            settle_count = 0;
        }

        Motor_setspeed(0, 0, Direction_Calibration_turn(target_angle));
        vTaskDelay(pdMS_TO_TICKS(OPEN_LOOP_PERIOD_MS));
    }

    Motor_setspeed(0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
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
        vTaskDelay(pdMS_TO_TICKS(OPEN_LOOP_PERIOD_MS));
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
    for (uint16_t i = 0; i < hold_ticks; i++)
    {
        Chassis_OpenLoop_SetSpeed(vx, vy, target_angle);
        vTaskDelay(pdMS_TO_TICKS(OPEN_LOOP_PERIOD_MS));
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

    while (angle_delta > 180.0f) angle_delta -= 360.0f;
    while (angle_delta < -180.0f) angle_delta += 360.0f;

    for (uint16_t i = 0; i < blend_ticks; i++)
    {
        float k = 0.5f - 0.5f * cosf(PI_F * i / blend_ticks);

        float vx = vx1 + (vx2 - vx1) * k;
        float vy = vy1 + (vy2 - vy1) * k;
        float target_angle = angle1 + angle_delta * k;

        Chassis_OpenLoop_SetSpeed(vx, vy, target_angle);
        vTaskDelay(pdMS_TO_TICKS(OPEN_LOOP_PERIOD_MS));
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
		Delay_ms(1);
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
		Set_chassis_able(unable);
		Motor_SetZero();
		Imu_setZero();
		Delay_ms(200);	//等imu稳定
	}

	// 绝对模式什么都不做
	car.target_x=(x*ratio_of_pulse_distance_x);
	car.target_y=(-y*ratio_of_pulse_distance_y);
	car.target_w=angle;
	Set_chassis_able(car.Odometer_able);
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



