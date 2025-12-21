#include "circe.h"
#include "usart.h"
#include "tim.h"
#include "pid.h"
#include "motor_control.h"
#include "motor.h"
#include "imu_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stm32f7xx_it.h"


int x_zhong=103;
int y_zhong=102;
int change_x = 0xFF;
int change_y = 0xFF;
int change_x_yuanpanji=0xFF;
int	change_y_yuanpanji=0xFF;
//static int x_zhong=103;
//static int y_zhong=102;
//static int change_x = 0xFF;
//static int change_y = 0xFF;
static int xy_actual[3];
static int xy_aim[2];

static float change_k=0.0f;

struct PIDstruct pid_change_y;
struct PIDstruct pid_change_x;

uint8_t USART6_readdata[128];
uint8_t USART6_senddata[128];

int USART6_stage = 0;
static char U6_pitoner=0;

int COLOR_DATA = 0;

void U6_Init(void)
{
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);
}

void U6_send(unsigned char data)
{
	unsigned short Usart6_Time=0;
	USART6->TDR=data;
	while((USART6->ISR&0X40)==0)//�ȴ����ͽ���
	{
		Usart6_Time++;
		if(Usart6_Time>65534)
		{
			break;
		}
	}
    __HAL_UART_CLEAR_OREFLAG(&huart6);    
}
void uart6WriteBuf(uint8_t *buf, uint8_t len)
{
	unsigned char i;
	for(i = 0; i < len; i++)
		U6_send(USART6_senddata[i]);
}
void send_NX(int order)
{
    switch(order)
    {
        case 0:
            U6_send(0x30);
            break;
        case RED:
            U6_send(0x31);
            break;
        case GREEN:
            U6_send(0x32);
            break;
        case BLUE:
            U6_send(0x33);
            break;
        case RED_CIRCLE:
            U6_send(0x34);
            break;
        case GREEN_CIRCLE:
            U6_send(0x35);
            break;
        case BLUE_CIRCLE:
            U6_send(0x36);
            break;
        case UP_GREEN:
            U6_send(0x37);
			break;
		case ALL_COLOR:
            U6_send(0x38);
            break;
        case 9:
            U6_send(0x39);
            break;
        default:
            break;
    }
}

void u6_data_process(uint8_t* data)
{
    switch (data[1])
    {
        case 0x5A:
        {
            xy_actual[0]=data[2];
            xy_actual[1]=data[3];
            xy_actual[2]=data[4];
            //vofa_printf("x:%d y:%d\n", data[3],data[4]);
            circle_color_data_deal();
        }break;
        case 0x51:
        {
            change_k = (float)(((data[3]<<8)|data[4])*0.001); // 0.01
        }break;
    default:
        break;
    }
}

void USART6_IRQHandler(void)
{
	unsigned char res;	
	if(USART6->ISR&(1<<5))	//?����?��?��y?Y
	{
        res = USART6->RDR;
		if(res==0x55&&USART6_stage==0)
        {
            USART6_stage=1;
            U6_pitoner=0;
        }
        if(USART6_stage==1)
        {
            USART6_readdata[U6_pitoner++]=res;
            if(res==0xAA)
            {
                USART6_stage=0;
                U6_pitoner=0;
                u6_data_process(USART6_readdata);
                // vofa_printf("U6_readdata:%02X %02X %02X %02X %02X %02X %02X %02X\n", 
                //             USART6_readdata[0], USART6_readdata[1], USART6_readdata[2], 
                //             USART6_readdata[3], USART6_readdata[4], USART6_readdata[5], 
                //             USART6_readdata[6], USART6_readdata[7]);
            }
        }
	}
    __HAL_UART_CLEAR_OREFLAG(&huart6);    
}

void circle_color_data_deal(void)
{
    COLOR_DATA = USART6_readdata[0];
    if(COLOR_DATA != 0x30&&USART6_readdata[1]!=0&&USART6_readdata[2]!=0)
    {			
			xy_aim[0]=x_zhong;
			xy_aim[1]=y_zhong;
			//Բ�̻���λ������ϵ
			change_x_yuanpanji =(xy_actual[1] - x_zhong);
            change_y_yuanpanji =(xy_actual[2] - y_zhong);
			
			//Բ����λ������ϵ
			change_x =-(xy_actual[1] - x_zhong);
            change_y =-(xy_actual[2] - y_zhong);
    }
		else
		{
			change_x = 0xFF;
            change_y = 0xFF;
            change_x_yuanpanji=0xFF;
            change_y_yuanpanji=0xFF;
		}
}
//�������ĵ�
void Set_Circle_Center(int x_z,int y_z)
{
	x_zhong=x_z;
	y_zhong=y_z;
}

int Get_data_action_flag(void)
{
    return xy_actual[0];
}

int Get_x_actual(void)
{
	return	xy_actual[1];
}
int Get_y_actual(void)
{
	return	xy_actual[2];
}

int *Get_xy_aim(void)
{
	return	xy_aim;
}

float Get_change_k(void)
{
    return change_k;
}

int Get_X_Change(void)
{
	return change_x;
}
int Get_Y_Change(void)
{
	return change_y;
}

int Get_X_Change_yuanpanji(void)
{
	return change_x_yuanpanji;
}
int Get_Y_Change_yuanpanji(void)
{
	return change_y_yuanpanji;
}

void USART6_readdata_SeetZero(void)
{
	memset(USART6_readdata,0,128);
}


