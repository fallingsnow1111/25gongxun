#ifndef __TEST_H
#define __TEST_H

#include "main.h"

void QR_Code_Test(void);
void Vision_Parse_Test(void);

/* 底盘里程、转向精度和角速度映射测试。 */
void Chassis_Turn_Error_Test(void);
void Chassis_AngularRate_Baseline_Test(void);
void Chassis_TurnRate_Map_Test(void);
void Chassis_LowSpeed_Linearity_Test(void);
void Chassis_Integral_Turn_Test(void);
void IMU_Static_Stability_Test(void);
void IMU_Drift_Rezero_Test(void);
void IMU_Stable_Straight_Test(void);

/* 机械臂仓位间隙与色环视觉定位测试。 */
void Ring_Warehouse_Clearance_Test(void);
void Ring_Location_Test(void);
void Ring_LocateOne_Place123_Test(void);
void Yuanpanji_Warehouse1_RingPlaceHeight_Test(void);

/* 夹爪重新安装后的舵机位置循环校正测试。 */
void Claw_Calibration_Test(void);

/* 上电机械臂姿态与三轴零点初始化测试。 */
void Arm_Pose_Init_Test(void);

#endif
