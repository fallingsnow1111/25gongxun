#ifndef __IMU_control_H
#define __IMU_control_H

#include "struct_typedef.h"
#include "string.h"

extern struct IMU_RUNDATA inu_run;
extern struct IMU_RUNDATA inu_turn;
extern struct PIDstruct Gyro_Pid;

/*
 * Yaw_DriftCorrect one-key switch:
 * 1: enable soft compensation, keep the real relative turn angle.
 * 0: disable soft compensation, turn to target_angle directly.
 */
#define YAW_DRIFT_COMP_ENABLE 0U

struct IMU_RUNDATA
{
	float ANGEL;
	float LAST_ANGEL;
	float SPEED;
};
void Gyro_Init(void);	
void Direction_Calibration(int target_angle);
float Direction_Calibration_turn(float tar_angle);
float getAngleZ_avg(float my_angel);
float getAngleZ(float yaw,float my_angel); 
float normalize_angle(float angle);
float Yaw_DriftCorrect(float expected_current, float target_angle);
#endif

