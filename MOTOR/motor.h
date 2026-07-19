#ifndef __MOTOR_H
#define __MOTOR_H

#include "sys.h"
#include "TIME.h"
#include "tim.h"

#define run_mode_none					0
#define run_mode_gyro					1

#define postion_mode					1
#define speed_mode				     	2

#define RXdat_maxsize 128

extern volatile char MOTOR_ACTIONFALG;
extern volatile struct  CHECK_FLAG_t  motor_check;

extern struct DATA	GLOBAL_DATA;

extern struct MOTO_DATA motor1;
extern struct MOTO_DATA motor2;
extern struct MOTO_DATA motor3;
extern struct MOTO_DATA motor4;
extern volatile uint32_t motor3_rx_probe;
extern volatile uint16_t u3_debug_size;
extern volatile uint32_t u3_debug_rx_count;
extern volatile uint32_t u3_debug_rx_restart_count;
extern volatile uint32_t u3_debug_rx_restart_fail;
extern volatile uint32_t u3_debug_error_count;
extern volatile uint32_t u3_debug_last_error;
extern volatile uint8_t u3_debug_buf[8];


struct DATA
{
	float Y_SPEED;
	float W_SPEED;
	float X_SPEED;
    float CHANHE_SPEED;
    int CHANHE_SAGE;
	unsigned int MOVE_TIME;
};

struct MOTO_DATA {
	//  float speed; // 0.01
	float target_angle;
	float actual_angle;
};

/* Software odometer unit: signed RPM multiplied by elapsed milliseconds. */
typedef struct
{
	int64_t wheel[4];
	int64_t x;
	int64_t y;
	int64_t w;
	uint32_t move_time_ms;
} CHASSIS_ODOM_T;

extern volatile uint32_t chassis_odom_tx_fail_count;
extern volatile int32_t motor_actual_pulse[4];
extern volatile uint8_t motor_speed_scale10_ready;
extern volatile int16_t motor_debug_cmd_rpm_x10[4];


void MOTOR_Init(void);                                                // 电机初始化
uint8_t Motor_EnableSpeedScale10(void);                               // 上电启用0.1RPM通信单位
void Motor_Send_Speed_together(int16_t LB,int16_t LF,int16_t RF,int16_t RB); // 协议速度单位为0.1RPM
void Motor_Send_Postion_together(int LB,int LF,int RF,int RB,char mode); // 打包四电机位置命令

void USART3_RXdata_processing(uint8_t* data,uint8_t size);          // UART3接收数据处理

HAL_StatusTypeDef Send_motor_together(void);                         // 多电机同步触发

void Motor_MakeZeroPiont(void);                                      // 电机归零
void Motor_SetZeroPiont(void);                                       // 设置电机零点

void Motor_setspeed_in_tim(void);
void motor_setspeed_chassis(float vy, float vx, float vw);

void Motor_Action_Calculate_target(float vy,float vx,float vw);
void Motor_Action_Calculate_actual(volatile float *actual_y,volatile float *actual_x,volatile float *actual_w);
void Motor_setspeed(float vy,float vx,float vw);//普通跑图使用整数RPM
void Motor_setspeed_fine(float vy,float vx,float vw);//转向精调使用0.1RPM
void Motor_setposition(float vy,float vx,float vw,char mode);//���õ��λ��
void motor_data_reset(void);

void motor_read_stateflag(uint8_t motor_id);//���͵����־λ

void motor_read_coordination(uint8_t motor_id);//��ȡ���ʵʱλ��
void Motor_coordination_Calculate(int X,int Y);//��������н���
void send_postion_data_switch(void);
void send_speed_data_switch(void);
void Motor_SetZero(void);//�������꣬����
HAL_StatusTypeDef uart3WriteBuf(uint8_t *buf, uint8_t len);
void Chassis_OdomResetSegment(void);
void Chassis_OdomGetSegment(CHASSIS_ODOM_T *odom);
void Chassis_OdomGetTotal(CHASSIS_ODOM_T *odom);
void Chassis_OdomGetSegmentSnapshot(CHASSIS_ODOM_T *odom);
void Chassis_WorldPoseReset(float x_mm, float y_mm, float yaw_deg);
uint8_t Motor_ReadPulseSnapshot(int32_t pulse[4]);
void Motor_Rxdata_SetSero(void);
void UART3_RxRestart(void);
void UART3_RxEventHandler(uint16_t size);
void UART3_ErrorHandler(void);
void MY_UART3_IRQHandler(void);

#endif

