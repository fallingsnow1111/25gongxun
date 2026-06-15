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

extern uint8_t RXdat[RXdat_maxsize];

extern volatile char MOTOR_ACTIONFALG;
extern volatile struct  CHECK_FLAG_t  motor_check;

extern struct DATA	GLOBAL_DATA;

extern struct MOTO_DATA motor1;
extern struct MOTO_DATA motor2;
extern struct MOTO_DATA motor3;
extern struct MOTO_DATA motor4;
extern volatile uint32_t motor3_rx_probe;


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


void MOTOR_Init(void);                                                // 电机初始化
void Motor_Send_Speed_together(float LB,float LF,float RF,float RB); // 打包四电机速度命令
void Motor_Send_Postion_together(int LB,int LF,int RF,int RB,char mode); // 打包四电机位置命令

void USART3_RXdata_processing(uint8_t* data,uint8_t size);          // UART3接收数据处理

void Send_motor_together(void);                                      // 多电机同步触发

void Motor_MakeZeroPiont(void);                                      // 电机归零
void Motor_SetZeroPiont(void);                                       // 设置电机零点

void Motor_setspeed_in_tim(void);
void motor_setspeed_chassis(float vy, float vx, float vw);

void Motor_Action_Calculate_target(float vy,float vx,float vw);
void Motor_Action_Calculate_actual(volatile float *actual_y,volatile float *actual_x,volatile float *actual_w);
void Motor_setspeed(float vy,float vx,float vw);//�趨������ٶ�
void Motor_setposition(float vy,float vx,float vw,char mode);//���õ��λ��
void motor_data_reset(void);

void motor_read_stateflag(uint8_t motor_id);//���͵����־λ

void motor_read_coordination(uint8_t motor_id);//��ȡ���ʵʱλ��
void Motor_coordination_Calculate(int X,int Y);//��������н���
void send_postion_data_switch(void);
void send_speed_data_switch(void);
void Motor_SetZero(void);//�������꣬����
void uart3WriteBuf(uint8_t *buf, uint8_t len);
void Motor_Rxdata_SetSero(void);
void MY_UART3_IRQHandler(void);

#endif

