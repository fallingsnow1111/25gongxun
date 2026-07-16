#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "main.h"

/* 开环直线距离标定系数，单位：软件脉冲/cm。 */
#define CHASSIS_LONGITUDINAL_PULSE_PER_CM 2478.5f /* 前进、后退共用。 */
#define CHASSIS_LATERAL_PULSE_PER_CM      2560.5f /* 左移、右移共用。 */

/* 距离接口自动选择的正弦加减速周期，1 tick = 5ms。 */
#define CHASSIS_LONG_ROUTE_MIN_CM          40.0f  /* 40cm 以上使用长加减速。 */
#define CHASSIS_SHORT_ROUTE_RAMP_TICKS     80U    /* 40cm 以下，加减速各 400ms。 */
#define CHASSIS_LONG_ROUTE_RAMP_TICKS     100U    /* 40cm 以上，加减速各 500ms。 */

float FMy_Abs(float temp);

void Chassis_OpenLoop_SetSpeed(float vx_world, float vy_world, float target_angle);
void Chassis_OpenLoop_SetSpeedFrame(float vx_world, float vy_world,
                                    float speed_frame_angle, float target_angle);
void Chassis_OpenLoop_SetTranslation(float vx_world, float vy_world,
                                     float speed_frame_angle);
void Chassis_MoveByPulse(float vx, float vy, float target_angle,
						 int64_t target_pulse, uint16_t ramp_ticks);
void Chassis_MoveByDistance(float vx, float vy, float target_angle,
						   float distance_cm);
void Chassis_MoveByDistanceSmoothYaw(float vx, float vy, float target_angle,
									float distance_cm);
void Chassis_WorldBeginSegment(void);
void Chassis_WorldCommitSegment(float segment_yaw);
void Chassis_TurnToAngle(float target_angle, uint32_t timeout_ms);
uint8_t Chassis_FineTuneAngle(float target_angle, uint32_t timeout_ms);
void Chassis_MoveTurnOnce(float vx, float vy, float start_angle, float end_angle,
                          uint16_t hold_ticks, uint16_t ramp_ticks);
void Chassis_HoldSpeedAngle(float vx, float vy, float target_angle, uint16_t hold_ticks);
void Chassis_BlendSpeedAngle(float vx1, float vy1, float angle1,
                             float vx2, float vy2, float angle2,
                             uint16_t blend_ticks);
void Chassis_DriftStraightTurn(float vx_world, float vy_world,
                               float speed_frame_angle,
                               float start_angle, float end_angle,
                               uint16_t turn_ticks);
extern volatile uint32_t chassis_period_overrun_count;
extern volatile uint32_t chassis_period_max_ms;

#endif
