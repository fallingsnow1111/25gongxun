#ifndef __TEST_H
#define __TEST_H

#include "main.h"

void User_function_final(void);
void QR_Code_Test(void);
void Motor_Periodic_Feedback_Test(void);
void Route_Test(void);
void Route_Test_ABS(void);
void Chassis_Test_Run_With_Imu(float vy, float vx, float target_angle, uint32_t run_time_ms);
void Chassis_Test_Rotate_With_Imu(float target_angle, uint32_t timeout_ms);

#endif
