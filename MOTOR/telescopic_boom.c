#include "telescopic_boom.h"
#include "postion_control.h"

static struct PIDstruct Telescopic_pid;
struct POSTION Telescopic_POSTION;

void Telescopic_Enable(void)
{
	uint8_t data[6];
	data[0]=(uint8_t)telescopic_id;
	data[1]=0xF3;
	data[2]=0xAB;
	data[3]=0x01;
	data[4]=0x00;
	data[5]=0x6B;
	uart7WriteBuf(data,6);
}

void Telescopic_Init(void)
{
	Telescopic_POSTION.CHANGE=0;
	Telescopic_POSTION.NOW=0;
	Telescopic_POSTION.TARGE=0;
	PID_Init(&Telescopic_pid,3.7,0,0,55,-55);
	//todo:���ڳ�ʼ��
	__HAL_UART_ENABLE_IT(&telescopic_usart, UART_IT_RXNE);
	//Telescopic_Enable();
}

/**********************************************************************************************************
*�� �� ��: Telescopic_Send_Speed
*����˵��: ����ٶ����ݷ���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Telescopic_Send_Speed(int speed)
{
	 Read_Y_position();
	 Delay_ms(3);
	 int32_t Y_position = -Get_Y_position();
	 static uint32_t Max_Y_position = 140000; // 伸缩臂的最大脉冲
	 static uint32_t Min_Y_position = 2000;
	if(Y_position>=Max_Y_position)
	{
		u7_speed_send(0x02,10);
	}
	else if(Y_position<=(Min_Y_position))
	{
		u7_speed_send(0x02,-10);
	}
	else if(Y_position>(Min_Y_position) && Y_position<Max_Y_position){
		u7_speed_send(0x02,-speed);
	}
	
}

/**********************************************************************************************************
*�� �� ��: Telescopic_Send_Speed
*����˵��: ����ٶ����ݷ���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Telescopic_Stop(void)
{
	Motor_Height_Ss_Stop(2);
}

/**********************************************************************************************************
*�� �� ��: Telescopic_Send_Position
*����˵��: ����ٶ����ݷ���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Telescopic_Send_Position(int position)
{
    // ��������֡����ʼ����
	Telescopic_POSTION.TARGE=-position*(360*8.90/94.2);
	postion_send(0x02,Telescopic_POSTION.TARGE);
	Telescopic_POSTION.BIT=Incomplete;
	Delay_ms(__fabs(position)*1);
	
}

void Telescopic_set_pid(float kp,float ki,float kd)
{
	Telescopic_pid.kp=kp;
	Telescopic_pid.ki=ki;
	Telescopic_pid.kd=kd;
}

void Telescopic_control_pid(int Difference_var)
{
	float output=0;
	//compute the PID output
	output=PID_Compute(&Telescopic_pid,0,Difference_var);
	//minimize the output
	if(output<0)
	{
		output=_LIMIT_MIN(__fabs(output),3);
	}
	else
	{
		output=-_LIMIT_MIN(__fabs(output),3);
	}
	//send the output to the motor
	Delay_ms(8);
	Telescopic_Send_Speed(output);
}

void Read_Telescopic_position(void)
{
	uint8_t data[3];
	data[0]=0x02;
	data[1]=0x3A;
	data[2]=0x6B;
	uart7WriteBuf(data,3);
}

