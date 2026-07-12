#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "main.h"


extern unsigned char Calibration_Complete;	
extern unsigned char Calibration_Complete_turn;	
extern unsigned char W_Gray_openmv ;

float FMy_Abs(float temp);
void Move_To_Target_area(float x,float y,float angle,int imu_able,MODE_POSITION mode);
void Move_To_Target_Postion(float vy,float vx,float w,char mode);
void motor_read_coordination_all(void);

void Chassis_OpenLoop_SetSpeed(float vx_world, float vy_world, float target_angle);
void Chassis_OpenLoop_SetSpeedFrame(float vx_world, float vy_world,
                                    float speed_frame_angle, float target_angle);
void Chassis_MoveOnce(float vx, float vy, float target_angle, uint16_t hold_ticks, uint16_t ramp_ticks);
void Chassis_TurnToAngle(float target_angle, uint32_t timeout_ms);
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

#endif
