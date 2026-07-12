#include "postion_control.h"
#include "pid.h"
#include "usart.h"
#include "main.h"

#define U7_RX_BUF_LEN 8
#define Z_MAX_TIMEOUT_MS 4000U
static uint8_t u7RXdat[U7_RX_BUF_LEN];
static uint8_t USART7_senddata[128];
static uint8_t u7_stream_frame[U7_RX_BUF_LEN];

volatile uint8_t u7_debug_buf[U7_RX_BUF_LEN];
volatile uint16_t u7_debug_size = 0;

struct POSTION Z_POSTION;
extern struct POSTION Telescopic_POSTION;

int postion_redstage_Z = 0;
int postion_redstage = 0;

void uart7WriteBuf(uint8_t *buf, uint8_t len)
{
	HAL_UART_Transmit(&huart7, (uint8_t*)buf, len, HAL_MAX_DELAY);
}

void UART7_RxRestart(void)
{
	HAL_UART_DMAStop(&huart7);
	__HAL_UART_CLEAR_FLAG(&huart7, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
	__HAL_UART_SEND_REQ(&huart7, UART_RXDATA_FLUSH_REQUEST);
	huart7.ErrorCode = HAL_UART_ERROR_NONE;
	huart7.RxState = HAL_UART_STATE_READY;
	huart7.ReceptionType = HAL_UART_RECEPTION_STANDARD;

	if(HAL_UARTEx_ReceiveToIdle_DMA(&huart7, u7RXdat, U7_RX_BUF_LEN) == HAL_OK)
	{
		__HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);
	}
}

void POSTION_init(void)
{
    Z_POSTION.NOW = 0;
    Z_POSTION.TARGE = 0;
    Z_POSTION.CHANGE = 0;
	Z_POSTION.BIT = FINISH_MOVE;	// 1 运动完成  0 运动中
	UART7_RxRestart();
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
    USART7_senddata[5] = 0xC8;//加速度
    USART7_senddata[6] = 0x00;
    USART7_senddata[7] = 0x6B;
		
		//todo:发送函数
   uart7WriteBuf(USART7_senddata,8);
}

void postion_send(uint8_t id,int position)
{
    // 计算目标与当前位置差值
        // 设置数据帧并初始化
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
        USART7_senddata[3] = 0x2E; // 速度原值0x0E
        USART7_senddata[4] = 0xFF;

        USART7_senddata[5] = accel; // 加速度原值F7

        USART7_senddata[6] = (uint8_t)((position ) >> 24);
        USART7_senddata[7] = (uint8_t)((position ) >> 16);
        USART7_senddata[8] = (uint8_t)((position ) >> 8);
        USART7_senddata[9] = (uint8_t)(position );

        USART7_senddata[10] = (uint8_t)0x01;
        USART7_senddata[11] = 0x00;
        USART7_senddata[12] = 0x6B;
        // 发送数据
        uart7WriteBuf(USART7_senddata, 13);  // 将数据发送到串口7
}

 void Z_SetHeight(int high)
{	
	uint32_t t = 0;

	Z_POSTION.TARGE =(2000/106.5)*high*14;
		// 若目标高度与当前接近则直接返回
	if(__fabs(Z_POSTION.NOW-high)<=0.4)
	{
		return;
	}
	Z_POSTION.BIT=Incomplete;
	postion_send(0x01,Z_POSTION.TARGE);
	while(Z_POSTION.BIT != finish && t < Z_MAX_TIMEOUT_MS)
	{
		vTaskDelay(pdMS_TO_TICKS(1));
		t++;
	}

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

#define GEAR_RATIO (360.0 * 8.90 / 94.2)  // 传动比常数
#define POSITION_TOLERANCE 1.0            // 位置容差
#define MAX_TIMEOUT_MS  300

void Y_SetLength(int position) 
{
	if(position < 0) 	position = 0;
	if(position > 80) 	position = 80;

    Telescopic_POSTION.TARGE = -position * GEAR_RATIO;
    // 若目标与当前位置接近则直接返回
    // if (__fabs(Telescopic_POSTION.NOW - Telescopic_POSTION.TARGE) <= POSITION_TOLERANCE) {
    //     return ;
    // }
	Telescopic_POSTION.BIT=Incomplete;
    postion_send(0x02,Telescopic_POSTION.TARGE);
    // 等待运动完成，带超时保护
    uint32_t timeout = 0;
    while ((Telescopic_POSTION.BIT != finish) && (timeout < MAX_TIMEOUT_MS)) {
        vTaskDelay(pdMS_TO_TICKS(1));
        timeout++;
    }
    // 更新当前位置
    Telescopic_POSTION.NOW = Telescopic_POSTION.TARGE;
}

// 急停：伸缩臂和高度电机
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

// 用不了，暂时不用
// 使用固定数据包长度，接收协议定义为4字节
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
							Z_POSTION.BIT=finish;// 到位完成
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
						Telescopic_POSTION.BIT=finish;// 到位完成
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

static void U7_Process_8Byte_Frame(uint8_t* data)
{
	if(data[7] != 0x6B)
	{
		return;
	}

	u7RXdat_dispose_1(data);
}

static void U7_Parse_Stream_Byte(uint8_t byte)
{
	for(uint8_t i = 0; i < U7_RX_BUF_LEN - 1; i++)
	{
		u7_stream_frame[i] = u7_stream_frame[i + 1];
	}
	u7_stream_frame[U7_RX_BUF_LEN - 1] = byte;

	if((u7_stream_frame[0] == 0x01 || u7_stream_frame[0] == 0x02) &&
	   u7_stream_frame[1] == 0x36 &&
	   u7_stream_frame[7] == 0x6B)
	{
		U7_Process_8Byte_Frame(u7_stream_frame);
	}

	if((u7_stream_frame[4] == 0x01 || u7_stream_frame[4] == 0x02) &&
	   u7_stream_frame[5] == 0xFD &&
	   u7_stream_frame[6] == 0x9F &&
	   u7_stream_frame[7] == 0x6B)
	{
		u7RXdat_dispose(&u7_stream_frame[4]);
	}
}

void MY_UART7_IRQHandler(uint16_t size)
{
	uint16_t rx_len = size;
	if(rx_len > U7_RX_BUF_LEN)
	{
		rx_len = U7_RX_BUF_LEN;
	}

	u7_debug_size = size;
	for(uint8_t i = 0; i < U7_RX_BUF_LEN; i++)
	{
		u7_debug_buf[i] = u7RXdat[i];
	}

	if(size == 4)// 到位反馈(4字节)
	{	
		//vofa_printf("u7RXdat:%02X %02X %02X %02X\n", u7RXdat[0], u7RXdat[1], u7RXdat[2], u7RXdat[3]);
		u7RXdat_dispose(u7RXdat);
	}
	else if(size == 8) // 位置读取反馈(8字节)
	{
		//vofa_printf("u7RXdat:%02X %02X %02X %02X %02X %02X %02X %02X\n", u7RXdat[0], u7RXdat[1], u7RXdat[2], u7RXdat[3], u7RXdat[4], u7RXdat[5], u7RXdat[6], u7RXdat[7]);
		u7RXdat_dispose_1(u7RXdat);
	}
	for(uint16_t i = 0; i < rx_len; i++)
	{
		U7_Parse_Stream_Byte(u7RXdat[i]);
	}
	HAL_UARTEx_ReceiveToIdle_DMA(&huart7, u7RXdat, U7_RX_BUF_LEN);
	__HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);
}


