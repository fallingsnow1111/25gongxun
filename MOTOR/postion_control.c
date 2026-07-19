#include "postion_control.h"
#include "pid.h"
#include "usart.h"
#include "main.h"

#define U7_RX_BUF_LEN 8
#define U7_DMA_RX_LEN 4
#define Z_MAX_TIMEOUT_MS 4000U
#define UART7_RX_SETTLE_MS 20U
#define UART7_TX_RETRY_DELAY_MS 5U
static uint8_t u7RXdat[U7_RX_BUF_LEN];
static uint8_t USART7_senddata[128];
static uint8_t u7_stream_frame[U7_RX_BUF_LEN];

volatile uint8_t u7_debug_buf[U7_RX_BUF_LEN];
volatile uint16_t u7_debug_size = 0;
volatile uint32_t u7_debug_rx_restart_count = 0;
volatile uint32_t u7_debug_rx_restart_fail = 0;
volatile uint32_t u7_debug_rx_event_count = 0;
volatile uint32_t u7_debug_error_count = 0;
volatile uint32_t u7_debug_last_error = 0;
volatile uint32_t u7_debug_tx_count = 0;
volatile uint32_t u7_debug_tx_retry_count = 0;
volatile uint8_t u7_debug_last_tx_status = 0xFF;
volatile uint8_t u7_debug_last_tx_len = 0;
volatile uint8_t u7_debug_last_tx_buf[13];
volatile uint32_t u7_debug_z_finish_count = 0;
volatile uint8_t z_debug_last_timeout = 0;
volatile uint32_t z_debug_last_wait_ms = 0;

struct POSTION Z_POSTION;
extern struct POSTION Telescopic_POSTION;

int postion_redstage_Z = 0;
int postion_redstage = 0;

uint8_t uart7WriteBuf(uint8_t *buf, uint8_t len)
{
	uint8_t copy_len = len;

	if(copy_len > sizeof(u7_debug_last_tx_buf))
	{
		copy_len = sizeof(u7_debug_last_tx_buf);
	}
	for(uint8_t i = 0; i < sizeof(u7_debug_last_tx_buf); i++)
	{
		u7_debug_last_tx_buf[i] = (i < copy_len) ? buf[i] : 0;
	}
	u7_debug_last_tx_len = len;
	u7_debug_tx_count++;
	u7_debug_last_tx_status = (uint8_t)HAL_UART_Transmit(&huart7, buf, len, HAL_MAX_DELAY);
	return u7_debug_last_tx_status;
}

void UART7_RxRestart(void)
{
	HAL_UART_AbortReceive(&huart7);
	__HAL_UART_DISABLE_IT(&huart7, UART_IT_RXNE);
	__HAL_UART_CLEAR_FLAG(&huart7, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
	__HAL_UART_SEND_REQ(&huart7, UART_RXDATA_FLUSH_REQUEST);
	huart7.ErrorCode = HAL_UART_ERROR_NONE;
	huart7.RxState = HAL_UART_STATE_READY;
	huart7.ReceptionType = HAL_UART_RECEPTION_STANDARD;

	if(HAL_UARTEx_ReceiveToIdle_DMA(&huart7, u7RXdat, U7_DMA_RX_LEN) == HAL_OK)
	{
		u7_debug_rx_restart_count++;
		__HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);
	}
	else
	{
		u7_debug_rx_restart_fail++;
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

#define POSITION_MODE_RELATIVE 0x00U
#define POSITION_MODE_ABSOLUTE 0x01U

static void postion_send_mode(uint8_t id, int position, uint8_t mode)
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

        USART7_senddata[10] = mode;
        USART7_senddata[11] = 0x00;
        USART7_senddata[12] = 0x6B;
        // 发送数据
        uart7WriteBuf(USART7_senddata, 13);  // 将数据发送到串口7
}

static uint8_t position_init_send(uint8_t id, int target)
{
	postion_send_mode(id, target, POSITION_MODE_RELATIVE);
	if(u7_debug_last_tx_status == HAL_OK)
	{
		return 1U;
	}

	u7_debug_tx_retry_count++;
	vTaskDelay(pdMS_TO_TICKS(UART7_TX_RETRY_DELAY_MS));
	UART7_RxRestart();
	vTaskDelay(pdMS_TO_TICKS(UART7_RX_SETTLE_MS));
	postion_send_mode(id, target, POSITION_MODE_RELATIVE);

	return (u7_debug_last_tx_status == HAL_OK);
}

void postion_send(uint8_t id,int position)
{
	postion_send_mode(id, position, POSITION_MODE_ABSOLUTE);
}

 void Z_SetHeight(int high)
{
	uint32_t start_tick;

	if(high < 0) high = 0;
	if(high > 125) high = 125;   // 这里按你的安全最大高度改

	Z_POSTION.TARGE =(2000/106.5)*high*14;
		// 若目标高度与当前接近则直接返回
	if(__fabs(Z_POSTION.NOW-Z_POSTION.TARGE)<=0.4)
	{
		z_debug_last_timeout = 0;
		z_debug_last_wait_ms = 0;
		return;
	}
	UART7_RxRestart();
	Z_POSTION.BIT=Incomplete;
	postion_send(0x01,Z_POSTION.TARGE);
	start_tick = HAL_GetTick();
	while(Z_POSTION.BIT != finish &&
		  (HAL_GetTick() - start_tick) < Z_MAX_TIMEOUT_MS)
	{
		vTaskDelay(pdMS_TO_TICKS(1));
	}

	z_debug_last_wait_ms = HAL_GetTick() - start_tick;
	z_debug_last_timeout = (Z_POSTION.BIT != finish);
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
#define INIT_MOVE_TIMEOUT_MS 4000U

/*
 * 上电零点标定专用：允许使用负方向偏移，不经过正常动作的行程限位。
 * 返回1表示收到到位帧，返回0表示等待超时。
 */
uint8_t Z_InitMoveSigned(int height)
{
	uint32_t start_tick;

	Z_POSTION.TARGE = (int)((2000.0f / 106.5f) * height * 14.0f);
	UART7_RxRestart();
	vTaskDelay(pdMS_TO_TICKS(UART7_RX_SETTLE_MS));
	Z_POSTION.BIT = Incomplete;
	if(!position_init_send(0x01, Z_POSTION.TARGE))
	{
		z_debug_last_wait_ms = 0;
		z_debug_last_timeout = 1;
		return 0;
	}
	start_tick = HAL_GetTick();

	while(Z_POSTION.BIT != finish &&
		  (HAL_GetTick() - start_tick) < INIT_MOVE_TIMEOUT_MS)
	{
		vTaskDelay(pdMS_TO_TICKS(1));
	}

	z_debug_last_wait_ms = HAL_GetTick() - start_tick;
	z_debug_last_timeout = (Z_POSTION.BIT != finish);
	return (Z_POSTION.BIT == finish);
}

uint8_t Y_InitMoveSigned(int length)
{
	uint32_t start_tick;

	Telescopic_POSTION.TARGE = (int)(-length * GEAR_RATIO);
	UART7_RxRestart();
	vTaskDelay(pdMS_TO_TICKS(UART7_RX_SETTLE_MS));
	Telescopic_POSTION.BIT = Incomplete;
	if(!position_init_send(0x02, Telescopic_POSTION.TARGE))
	{
		return 0;
	}
	start_tick = HAL_GetTick();

	while(Telescopic_POSTION.BIT != finish &&
		  (HAL_GetTick() - start_tick) < INIT_MOVE_TIMEOUT_MS)
	{
		vTaskDelay(pdMS_TO_TICKS(1));
	}

	return (Telescopic_POSTION.BIT == finish);
}

void Y_SetLength(int position) 
{
	if(position < 0) 	position = 0;
	if(position > 120) 	position = 120;

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

	/* 硬件当前位置清零后，同步软件坐标，避免后续绝对位置继续使用旧值。 */
	if(id == 1)
	{
		Z_POSTION.NOW = 0;
		Z_POSTION.TARGE = 0;
		Z_POSTION.CHANGE = 0;
		Z_POSTION.BIT = finish;
	}
	else if(id == 2)
	{
		Telescopic_POSTION.NOW = 0;
		Telescopic_POSTION.TARGE = 0;
		Telescopic_POSTION.CHANGE = 0;
		Telescopic_POSTION.BIT = finish;
	}
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
							u7_debug_z_finish_count++;
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
	u7_debug_rx_event_count++;
	if(rx_len > U7_RX_BUF_LEN)
	{
		rx_len = U7_RX_BUF_LEN;
	}

	u7_debug_size = size;
	for(uint8_t i = 0; i < U7_RX_BUF_LEN; i++)
	{
		u7_debug_buf[i] = u7RXdat[i];
	}

	for(uint16_t i = 0; i < rx_len; i++)
	{
		U7_Parse_Stream_Byte(u7RXdat[i]);
	}
	HAL_UARTEx_ReceiveToIdle_DMA(&huart7, u7RXdat, U7_DMA_RX_LEN);
	__HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);
}


