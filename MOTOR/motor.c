#include "motor.h"
#include "pid.h"
#include "usart.h"
#include <string.h>
#include "dma.h"
#include "stm32f7xx_hal.h"
#include "delay.h"
#include "tjc_usart_hmi.h"

volatile char MOTOR_ACTIONFALG=Incomplete;

extern DMA_HandleTypeDef hdma_usart2_rx;

static uint8_t LB_send[15];
static uint8_t LF_send[15];
static uint8_t RB_send[15];
static uint8_t RF_send[15];

struct DATA	GLOBAL_DATA;
struct MOTO_DATA motor1;
struct MOTO_DATA motor2;
struct MOTO_DATA motor3;
struct MOTO_DATA motor4; 
MOTOR_SPEED_t car_setspeed;

int motor_mode = speed_mode;
int postion_bit = finish;

//static uint8_t u7RxBuffer[128]; // ���ջ�����
static uint8_t RXdat[RXdat_maxsize]={0};
char RXdat_piont=0;
uint8_t u3_Txdata;

void uart3WriteBuf(uint8_t *buf, uint8_t len)
{
	HAL_UART_Transmit_DMA(&huart3,buf,len);
}

void MOTOR_Init(void)//�����ʼ��?
{
	car_setspeed.x_setpeed = 0.0f;
	car_setspeed.y_setpeed = 0.0f;
	car_setspeed.w_setpeed = 0.0f;
	motor1.target_angle = 0;
	motor1.actual_angle = 0;
	motor2.target_angle = 0;
	motor2.actual_angle = 0;
	motor3.target_angle = 0;
	motor3.actual_angle = 0;
	motor4.target_angle = 0;
	motor4.actual_angle = 0;
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);//��������3�Ľ���
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3,RXdat,RXdat_maxsize);//����DMAת��
}

/**********************************************************************************************************
*�� �� ��: Motor_Send_Speed_together
*����˵��: ����ٶ����ݷ���?(���ö��ͬ��ͨ�?)
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Motor_Send_Speed_together(float LB,float LF,float RF,float RB)//�Ƕ��ͬ��?
{
	static uint8_t* LB_speedptr = LB_send;
    static uint8_t* LF_speedptr = LF_send;
    static uint8_t* RB_speedptr = RB_send;
    static uint8_t* RF_speedptr = RF_send;
////////////////////////     1          2              3           4///////
    uint8_t* temp[4] = {LB_speedptr, LF_speedptr, RF_speedptr, RB_speedptr};
    int tempspeed = 0;
    for (uint8_t i = 0; i < 4; i++)
    {
			uint8_t var=i;
        switch(i+1)
				{
					case 1:
						tempspeed=LB;break;
					case 2:
						tempspeed=LF;break;
					case 3:
						tempspeed=RF;break;
					case 4:
						tempspeed=RB;break;
					default:
						tempspeed=0;
						break;
				}
                temp[i][0] = var+1;
				temp[i][1] = 0xF6;

				if(__fabs(tempspeed)>255)
				{
					
				}

				if(tempspeed>0)
				{
						temp[i][2] = 0x00;
						temp[i][3] = (tempspeed >>8) & 0xFF;
						temp[i][4] = (tempspeed & 0xFF);
				}
				else
				{
						tempspeed = -tempspeed;
						temp[i][2] = 0x01;
						temp[i][3] = (tempspeed >>8) & 0xFF;
						temp[i][4] = (tempspeed & 0x00FF);
				}
				temp[i][5] = 0xC8;
				temp[i][6] = 0x00;
				temp[i][7] = 0x6B;
    }
}

void Send_motor_together(void)//����ͬ��ָ��
{
	uint8_t data[4];
	data[0]=0x00;
	data[1]=0xFF;
	data[2]=0x66;
	data[3]=0x6B;
	uart3WriteBuf((uint8_t*)data,4);
}

//向电机发送命令：读取电机实时位置
void motor_read_coordination(uint8_t motor_id)
{
		uint8_t sendmotor_coordination_data[3];
		sendmotor_coordination_data[0]=motor_id;
		sendmotor_coordination_data[1]=0x36;
		sendmotor_coordination_data[2]=0x6B;
		uart3WriteBuf(sendmotor_coordination_data,3);
}

/**********************************************************************************************************
*�� �� ��: Motor_Send_Postion_together
*����˵��: ���λ�����ݷ���?(���ö��ͬ��ͨ�?)
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Motor_Send_Postion_together(int LB, int LF, int RF, int RB, char mode) // ���ͬ��?
{
    static uint8_t* LB_positionptr = LB_send;
    static uint8_t* LF_positionptr = LF_send;
    static uint8_t* RB_positionptr = RB_send;
    static uint8_t* RF_positionptr = RF_send;
////////////////////////     1               2                3                4////////
    uint8_t* temp[4] = {LB_positionptr, LF_positionptr, RF_positionptr, RB_positionptr};
    int temppostion = 0;

    for (uint8_t i = 0; i < 4; i++)
    {
				uint8_t var=i;
        switch(i+1)
				{
					case 1:
						temppostion=LB;break;
					case 2:
						temppostion=LF;break;
					case 3:
						temppostion=RF;break;
					case 4:
						temppostion=RB;break;
					default:
						temppostion=0;
						break;
				}
        temp[i][0] = var+1;
        temp[i][1] = 0xFD;
        if (temppostion > 0)
        {
            temp[i][2] = 0x00;
        }
        else
        {
            temppostion = -temppostion;
            temp[i][2] = 0x01;
        }
        temp[i][3] = 0x2E; // �ٶ�ԭ��0x05
        temp[i][4] = 0xE0;

        temp[i][5] = 0xAE; // ���ٶ�ԭ��1A

        temp[i][6] = (uint8_t)((temppostion * 14) >> 24);
        temp[i][7] = (uint8_t)((temppostion * 14) >> 16);
        temp[i][8] = (uint8_t)((temppostion * 14) >> 8);
        temp[i][9] = (uint8_t)(temppostion * 14);

        temp[i][10] = (uint8_t)mode;
        temp[i][11] = 0x01;
        temp[i][12] = 0x6B;
    }
}

void send_speed_data_switch(void)
{
    uart3WriteBuf(LB_send,8);
	Delay_ms(4);
	//vTaskDelay(pdMS_TO_TICKS(5));
    uart3WriteBuf(LF_send,8);
	Delay_ms(4);
	//vTaskDelay(pdMS_TO_TICKS(5));
    uart3WriteBuf(RF_send,8);
	Delay_ms(4);
	//vTaskDelay(pdMS_TO_TICKS(5));
    uart3WriteBuf(RB_send,8);
	Delay_ms(4);
	//vTaskDelay(pdMS_TO_TICKS(5));
}

void send_postion_data_switch(void)
{
    uart3WriteBuf(LB_send,13);
    Delay_ms(4);
    uart3WriteBuf(LF_send,13);
    Delay_ms(4);
    uart3WriteBuf(RB_send,13);
    Delay_ms(4);
    uart3WriteBuf(RF_send,13);
	Delay_ms(4);
}
 void Motor_Action_Calculate_target(float vy,float vx,float vw)
{	
	__disable_irq();
	motor1.target_angle =  vw + vy - vx; // 1号电�?
	motor2.target_angle =  vw + vy + vx; // 2号电�?
	motor3.target_angle =  vw - vy + vx; // 3号电�?
	motor4.target_angle =  vw - vy - vx; // 4号电�?
	__enable_irq();
}

void Motor_Action_Calculate_actual(volatile float *actual_y,volatile float *actual_x,volatile float *actual_w)
{
    // 调整符号，匹配逆解�?
    *actual_w = ( motor1.actual_angle + motor2.actual_angle + motor3.actual_angle + motor4.actual_angle) * 0.25f;
    *actual_y = ( motor1.actual_angle + motor2.actual_angle - motor3.actual_angle - motor4.actual_angle) * 0.25f;
    *actual_x = (-motor1.actual_angle + motor2.actual_angle + motor3.actual_angle - motor4.actual_angle) * 0.25f;
}

void Motor_setposition(float vy,float vx,float vw,char mode)
{
	Motor_Action_Calculate_target(vy,vx,vw);
	Motor_Send_Postion_together(motor1.target_angle,motor2.target_angle,motor3.target_angle,motor4.target_angle,mode);
	send_postion_data_switch();
	Send_motor_together();
}

//不用多机同步
void Motor_setspeed(float vy, float vx, float vw) // �趨������ٶ�?
{
    Motor_Action_Calculate_target(vy, vx, vw);  // ������Ŀ���ٶ�
	Motor_Send_Speed_together(motor1.target_angle, motor2.target_angle, motor3.target_angle, motor4.target_angle);
	send_speed_data_switch();
	// ����������ɺ������������
	//Send_motor_together();//����ͬ��ָ��
}

void motor_setspeed_chassis(float vy, float vx, float vw) // ���ͨ�����趨������ٶ�
{
	car_setspeed.y_setpeed = vy;
	car_setspeed.x_setpeed = vx;
	car_setspeed.w_setpeed = vw;
}

//use Motor_setspeed_in_tim() to set speed in tim for 10ms
void Motor_setspeed_in_tim(void)
{
	Motor_setspeed(car_setspeed.y_setpeed, car_setspeed.x_setpeed, car_setspeed.w_setpeed);
}

void motor_data_reset(void)
{
	//__disable_irq();
	motor1.target_angle = 0;
    motor2.target_angle = 0;
    motor3.target_angle = 0;
    motor4.target_angle = 0;
	motor1.actual_angle=0;
    motor2.actual_angle = 0;
    motor3.actual_angle = 0;
    motor4.actual_angle = 0;
	GLOBAL_DATA.W_SPEED = 0;
	GLOBAL_DATA.Y_SPEED = 0;
	GLOBAL_DATA.X_SPEED = 0;
    GLOBAL_DATA.CHANHE_SPEED = 0;
	//__enable_irq();
}


void Motor_SetZero(void)//�������꣬����
{
	uint8_t TXdata[4];
	TXdata[1]=0x0A;
	TXdata[2]=0x6D;
	TXdata[3]=0x6B;
	for(uint16_t i=0x01;i<=4;i++)
	{
		TXdata[0]=i;
		uart3WriteBuf((uint8_t*)TXdata,4);
		Delay_ms(6);
	}
	//Delay_ms(10);
}

void Motor_SetZeroPiont(void)//�������λ��?
{
	uint8_t TXdata[5];
	TXdata[1]=0x93;
	TXdata[2]=0x88;
	TXdata[3]=0x01;
	TXdata[4]=0x6B;
	for(uint16_t i=0x01;i<=4;i++)
	{
		TXdata[0]=i;
		uart3WriteBuf((uint8_t*)TXdata,5);
		Delay_ms(5);
	}
}

void Motor_ReadZeroPiontFlag(uint8_t id)
{
	uint8_t TXdata[3];
	TXdata[0]=id;
	TXdata[1]=0x3B;
	TXdata[2]=0x6B;
	uart3WriteBuf((uint8_t*)TXdata,3);
}

void Motor_MakeZeroPiont(void)//��������
{
	int count=0;
	uint8_t TXdata[5];
	TXdata[1]=0x9A;
	TXdata[2]=0x00;
	TXdata[3]=0x00;
	TXdata[4]=0x6B;
	for(uint16_t i=0x01;i<=4;i++)
	{
		TXdata[0]=i;
		uart3WriteBuf((uint8_t*)TXdata,5);
		Delay_ms(5);
	}
	Send_motor_together();
	
	while(!(RXdat[2]&0x01))
	{
		Delay_ms(5);
		Motor_ReadZeroPiontFlag(1);
		count++;
		if(count>=50)
		{
			break;
		}
	}
}

void motor_read_stateflag(uint8_t motor_id)//���ͼ���־λ��ָ��
{
	uint8_t Send_motor_state[3];
	Send_motor_state[0]=motor_id;
	Send_motor_state[1]=0x3A;
	Send_motor_state[2]=0x6B;
	uart3WriteBuf(Send_motor_state,3);
}


void Motor_Rxdata_SetSero(void)
{
	memset(RXdat,0,(RXdat_maxsize)*sizeof(uint8_t));
}

static uint8_t rxbuff1[128];
static uint8_t rxbuff2[128];
extern volatile struct  CHECK_FLAG_t  motor_check;
//读取电机实际角度的函数
uint8_t U3_data_processing(uint8_t* data,uint8_t len)
{
	if(data[len-1] != 0x6B)
	{
		return 0;
	}
	
	if(len==8)
	{
		switch (data[0])  // 0x01-0x04
		{
		case 0x01: // LB
			
			motor1.actual_angle = (data[3] << 8*3) | (data[4] << 8*2) | (data[5] << 8) | data[6];
			motor1.actual_angle = (motor1.actual_angle*360)/65536; // 0-65536 -> 0-360
			if(data[2]==0x01)
			{
				motor1.actual_angle=-motor1.actual_angle;
			}
			motor_check.flag_finish=motor_check.flag_finish | (1<<0); // 设置完成标志
			break;
		case 0x02: // LF
			motor2.actual_angle = (data[3] << 8*3) | (data[4] << 8*2) | (data[5] << 8) | data[6];
			motor2.actual_angle = (motor2.actual_angle*360)/65536; // 0-65536 -> 0-360
			if(data[2]==0x01)
			{
				motor2.actual_angle=-motor2.actual_angle;
			}
			motor_check.flag_finish=motor_check.flag_finish | (1<<1); // 设置完成标志
			break;
		case 0x03: // RF
			motor3.actual_angle = (data[3] << 8*3) | (data[4] << 8*2) | (data[5] << 8) | data[6];
			motor3.actual_angle = (motor3.actual_angle*360)/65536; // 0-65536 -> 0-360
			if(data[2]==0x01)
			{
				motor3.actual_angle=-motor3.actual_angle;
			}
			motor_check.flag_finish=motor_check.flag_finish | (1<<2); // 设置完成标志
			break;
		case 0x04: // RB
			motor4.actual_angle = (data[3] << 8*3) | (data[4] << 8*2) | (data[5] << 8) | data[6];
			motor4.actual_angle = (motor4.actual_angle*360)/65536; // 0-65536 -> 0-360
			if(data[2]==0x01)
			{
				motor4.actual_angle=-motor4.actual_angle;
			}
			motor_check.flag_finish=motor_check.flag_finish | (1<<3); // 设置完成标志
			break;
		default:
			return 0;
			//break;
		}
	}
	//data[1] == 0xF6?
	else if (len==4)
	{
		if(data[0]==1&&data[3]==0x6B)
		{
			if(data[1]==0xF6&&data[2]==0x02)
			{
				motor_check.flag_ready=finish;
			}
		}
	}
}

void MOTOR_FINISHFLAGEXAM(uint8_t *RXdat) // 接收完成标志检查
{
    uint8_t buff_len = 0;
    static uint8_t* rxbuff = NULL;
    static uint8_t rxbuff_flag = 0;
    
    // 停止DMA接收
    if(HAL_UART_DMAStop(&huart3) != HAL_OK) {
        // 错误处理
        return;
    }
    
    // 禁用中断以确保数据一致
    __disable_irq();
    
    // 正确获取DMA接收计数器
    if(huart3.hdmarx != NULL) {
        buff_len = RXdat_maxsize - __HAL_DMA_GET_COUNTER(huart3.hdmarx);
    }
    
    // 检查RXdat指针有效性和数据长度
    if(buff_len > 0 && RXdat != NULL) {
        // 双缓冲区切换，取反表示缓冲区切换
        if(rxbuff_flag == 0) {
            rxbuff = rxbuff1;
            rxbuff_flag = ~rxbuff_flag;
        } else {
            rxbuff = rxbuff2;
            rxbuff_flag = ~rxbuff_flag;
        }
        // 复制数据到缓冲区
        memcpy(rxbuff, RXdat, buff_len);
        // 恢复中断
        __enable_irq();
        // 处理接收到的数据（在中断恢复后进行，避免阻塞时间过长）
        U3_data_processing(rxbuff, buff_len);
    } else {
        // 恢复中断（如果没有数据或指针无效）
        __enable_irq();
    }
}

void MY_UART3_IRQHandler(void)
{
	MOTOR_FINISHFLAGEXAM(RXdat);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3, RXdat, RXdat_maxsize);
}



