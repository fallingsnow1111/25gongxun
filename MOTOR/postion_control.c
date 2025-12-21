#include "postion_control.h"
#include "pid.h"
#include "usart.h"
#include "main.h"
static uint8_t USART7_senddata[128];

struct POSTION Z_POSTION;
extern struct POSTION Telescopic_POSTION;

int postion_redstage_Z = 0;
int postion_redstage = 0;

void uart7WriteBuf(uint8_t *buf, uint8_t len)
{
	// unsigned char i;
	// for(i = 0; i < len; i++){
	// 	U7_send(USART7_senddata[i]);
	// }
	HAL_UART_Transmit(&huart7, (uint8_t*)buf, len, HAL_MAX_DELAY);

}
static uint8_t u7RXdat[8];
void POSTION_init(void)
{
    Z_POSTION.NOW = 0;
    Z_POSTION.TARGE = 0;
    Z_POSTION.CHANGE = 0;
		Z_POSTION.BIT = FINISH_MOVE;
    __HAL_UART_ENABLE_IT(&huart7, UART_IT_RXNE);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart7, u7RXdat, 8);
}

void u7_speed_send(uint8_t id,int speed)
{
    USART7_senddata[0] = 0x02;
    USART7_senddata[1] = 0xF6;
    if(speed>0)
    {
        USART7_senddata[2] = 0x00;
        USART7_senddata[3] = 0x00;
        USART7_senddata[4] = speed;
    }
    else
    {
        speed = -speed;
        USART7_senddata[2] = 0x01;
        USART7_senddata[3] = 0x00;
        USART7_senddata[4] = speed;
    }
    USART7_senddata[5] = 0xC8;//���ٶ�
    USART7_senddata[6] = 0x00;
    USART7_senddata[7] = 0x6B;
		
		//todo:���ͺ���
   uart7WriteBuf(USART7_senddata,8);
}

void postion_send(uint8_t id,int position)
{
    //Z_POSTION.CHANGE = Z_POSTION.TARGE - Z_POSTION.NOW;  // ����Ŀ��λ���뵱ǰ���λ�õĲ��?
        // ��������֡����ʼ����
		static uint8_t accel=0;//加速度
		USART7_senddata[0] = id;
		if(id==1)
		{
			accel=0xFD;
		}
		else if(id==2)
		{
			accel=0XFB;
		}
        USART7_senddata[1] = 0xFD;
        if (position > 0)
        {
            USART7_senddata[2] = 0x00;
        }
        else
        {
            position = -position;
            USART7_senddata[2] = 0x01;
        }
        USART7_senddata[3] = 0x2E; // �ٶ�ԭ��0x0E
        USART7_senddata[4] = 0xFF;

        USART7_senddata[5] = accel; // ���ٶ�ԭ��F7

        USART7_senddata[6] = (uint8_t)((position ) >> 24);
        USART7_senddata[7] = (uint8_t)((position ) >> 16);
        USART7_senddata[8] = (uint8_t)((position ) >> 8);
        USART7_senddata[9] = (uint8_t)(position );

        USART7_senddata[10] = (uint8_t)0x01;
        USART7_senddata[11] = 0x00;
        USART7_senddata[12] = 0x6B;
        // ��������
        uart7WriteBuf(USART7_senddata, 13);  // �����ݷ��͵�����7	
}

 void Z_SetHeight(int high)
{		
	Z_POSTION.TARGE =(2000/106.5)*high*14;
	//�ж���εĸ߶��Ƿ����ϴ�һ�����ǵĻ�ֱ�ӷ���??
	if(__fabs(Z_POSTION.NOW-high)<=0.4)
	{
		return;
	}
	postion_send(0x01,Z_POSTION.TARGE);
	Z_POSTION.BIT=Incomplete;
	while(Z_POSTION.BIT != finish);
	Z_POSTION.BIT=Incomplete;
	Z_POSTION.NOW=Z_POSTION.TARGE;
}

void Read_Y_position(void)
{
	uint8_t senddata[3];
	senddata[0]=0x02;
	senddata[1]=0x36;
	senddata[2]=0x6B;
	uart7WriteBuf(senddata,3);
}

#define GEAR_RATIO (360.0 * 8.90 / 94.2)  // ���崫���ȳ���
#define POSITION_TOLERANCE 1.0            // λ���ݲ�
#define MAX_TIMEOUT_MS  300

void Y_SetLength(int position) 
{
    Telescopic_POSTION.TARGE = -position * GEAR_RATIO;
    // ���Ŀ��λ���뵱ǰλ�ýӽ�����ֱ�ӷ���??
    // if (__fabs(Telescopic_POSTION.NOW - Telescopic_POSTION.TARGE) <= POSITION_TOLERANCE) {
    //     return ;
    // }
    postion_send(0x02,Telescopic_POSTION.TARGE);
	Telescopic_POSTION.BIT=Incomplete;
    // �ȴ��˶���ɣ�����ʱ���
    uint32_t timeout = 0;
    while ((Telescopic_POSTION.BIT != finish) && (timeout < MAX_TIMEOUT_MS)) {
        Delay_ms(1);
        timeout++;
    }
    Telescopic_POSTION.BIT=Incomplete;	
    // ���µ�ǰλ��
    Telescopic_POSTION.NOW = Telescopic_POSTION.TARGE;
}

//�������ͣ�?,�����ۺ͸߶ȵ����??
void Motor_Height_Ss_Stop(char id)
{
	uint8_t senddata[5];
	senddata[0]=(uint8_t)id;
	senddata[1]=0xFE;
	senddata[2]=0x98;
	senddata[3]=0x00;
	senddata[4]=0x6B;
	uart7WriteBuf(senddata,5);
}
//id为1是升降机，2为伸缩臂
void YZ_SetZero(char id)
{
	uint8_t senddata[4];
	senddata[0]=(uint8_t)id;
	senddata[1]=0x0A;
	senddata[2]=0x6D;
	senddata[3]=0x6B;
	uart7WriteBuf(senddata,4);
}

void Read_Z_position(void)
{
	uint8_t senddata[3];
	senddata[0]=0x01;
	senddata[1]=0x3A;
	senddata[2]=0x6B;
	uart7WriteBuf(senddata,3);
}

// �ò��ˣ���ʱ����
// ʹ�ù̶����ݰ����ȣ�����Э�鶨��Ϊ4�ֽڣ�
#define PACKET_LENGTH 4

void u7RXdat_dispose(uint8_t* data)
{
	//position_control
	if(data[0]==0x01){
				{
					if(data[1]==0xFD&&data[3]==0x6B)
					{
						if(data[2]==0x9F)
						{
							Z_POSTION.BIT=finish;//�����ɶ���
							Z_POSTION.NOW = Z_POSTION.TARGE;
						}
					}
				}
		}
	//Telescopic_POSTION
	if(data[0]==0x02){
			{
				if(data[1]==0xFD&&data[3]==0x6B)
				{
					if(data[2]==0x9F)
					{
						Telescopic_POSTION.BIT=finish;//�����ɶ���
						Telescopic_POSTION.NOW=Telescopic_POSTION.TARGE;
					}
				}
			}
	}
}
static uint32_t temp_position;

uint32_t Get_Y_position(void)
{
	return temp_position;
}

void u7RXdat_dispose_1(uint8_t* data)
{
	
	//Telescopic_POSTION
	if(data[0]==0x02){
			{
				if(data[1]==0x36&&data[7]==0x6B)
				{
					temp_position =  (( data[3] << 24) | ( data[4] << 16)  | ( data[5] << 8) | data[6]);

					if(data[2]==0x01)
					{
						temp_position = -temp_position;	
					}
					else if(data[2]==0x00)
					{
						temp_position = temp_position;
					}
				}
			}
	}
}

void MY_UART7_IRQHandler(uint8_t size)
{	
	if(size == 4)// ������������
	{	
		//vofa_printf("u7RXdat:%02X %02X %02X %02X\n", u7RXdat[0], u7RXdat[1], u7RXdat[2], u7RXdat[3]);
		u7RXdat_dispose(u7RXdat);
	}
	else if(size == 8) // ������������
	{
		//vofa_printf("u7RXdat:%02X %02X %02X %02X %02X %02X %02X %02X\n", u7RXdat[0], u7RXdat[1], u7RXdat[2], u7RXdat[3], u7RXdat[4], u7RXdat[5], u7RXdat[6], u7RXdat[7]);
		u7RXdat_dispose_1(u7RXdat);
	}
	HAL_UARTEx_ReceiveToIdle_DMA(&huart7, u7RXdat, 128);
}


