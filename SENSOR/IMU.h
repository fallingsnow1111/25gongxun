#ifndef __IMU_H
#define __IMU_H

#include <stdint.h>
#include "usart.h"

extern struct Imu imu;
extern unsigned char mpu_flash;

struct Imu
{
    float yaw;
    float roll;
    float pitch;
    float angular_rate;   // Z轴角速率 (deg/s)
};

void IMU_Receive_Init(void);
void IMU_Process(void);
void Imu_setZero(void);
void Imu_unlock_register(void);
void Imu_setset_baudrate_115200(void);
void Imu_setsave_settings(void);
void Imu_set500hz(void);

#endif
