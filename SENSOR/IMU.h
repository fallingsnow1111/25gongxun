#ifndef __IMU_H
#define __IMU_H

#include "struct_typedef.h"
#include "string.h"

#define BUFFER_SIZE  128

extern struct Imu imu;
extern unsigned char mpu_flash;

struct Imu
{
	float yaw;
	float roll;
	float pitch;
};

void Imu_setZero(void);
void imu_receive_init(void);
void Imu_set500hz(void);
void Imu_unlock_register(void);
void Imu_setset_baudrate_115200(void);
void Imu_setsave_settings(void);
void MY_USART2_IRQHandler(uint8_t Size);

#endif

