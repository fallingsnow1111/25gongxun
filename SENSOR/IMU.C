#include "IMU.h"
#include "stdio.h"
#include "sys.h"
 #include "usart.h"

//配置命令数组
//解锁寄存器 FF AA 69 88 B5
//Z轴角度归零FF AA 76 00 00
//设置200HZ输出 FF AA 03 0B 00
//设置波特率115200 FF AA 04 06 00
//保存 FF AA 00 00 00
//重启 FF AA 00 FF 00
uint8_t unlock_register[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
uint8_t reset_z_axis[] = {0xFF, 0xAA, 0x76, 0x00, 0x00};
uint8_t set_output_200Hz[] = {0xFF, 0xAA, 0x03, 0x0B, 0x00};
uint8_t set_baudrate_115200[] = {0xFF, 0xAA, 0x04, 0x06, 0x00};
uint8_t save_settings[] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
uint8_t restart_device[] = {0xFF, 0xAA, 0x00, 0xFF, 0x00};

static uint8_t imu_rx_buf[BUFFER_SIZE] = {0};
static uint8_t imu_rx_len = 0;
//static uint8_t imu_rx_singledata = 0;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern struct IMU_RUNDATA inu_run;
unsigned char mpu_flash;
struct Imu imu;
void U2_send(unsigned char data)
{
	unsigned short Usart2_Time=0;
	USART2->TDR=data;
	while((USART2->ISR&0X40)==0)//�ȴ����ͽ���
	{
		Usart2_Time++;
		if(Usart2_Time>65534)
		{
			break;
		}
	}
    __HAL_UART_CLEAR_OREFLAG(&huart2);
}

void uart2WriteBuf(uint8_t *buf, uint8_t len)
{
	unsigned char i;
	for(i = 0; i < len; i++)
		U2_send(buf[i]);
}
void imu_receive_init(void)
{
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart2,imu_rx_buf,BUFFER_SIZE);
}

void Imu_setZero(void)
{
	uart2WriteBuf(reset_z_axis,5);
	inu_run.LAST_ANGEL=0;
	inu_run.SPEED=0;
}
void Imu_unlock_register(void)
{
    uart2WriteBuf(unlock_register,5);
}

void Imu_setset_baudrate_115200(void)
{
    uart2WriteBuf(set_baudrate_115200,5);
}

void Imu_setsave_settings(void)
{
    uart2WriteBuf(save_settings,5);
}
void Imu_set500hz(void)
{
    uart2WriteBuf(set_output_200Hz,5);
}

void USART2_IRQHandler(void)
{
	uint32_t flag_idle = 0;

	flag_idle = __HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE);
	if((flag_idle != RESET))
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart2);

		HAL_UART_DMAStop(&huart2);
		uint32_t temp = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
		imu_rx_len = BUFFER_SIZE - temp;

		if(imu_rx_buf[0] == 0x55)
		{
			uint8_t sum = 0;
			for(int i=11;i<21;i++)
				sum+=imu_rx_buf[i];
			if(sum == imu_rx_buf[21])
			{
				if(imu_rx_buf[12] == 0x53)
				{
					imu.yaw = (float)(0.9*(180.0*(short)((imu_rx_buf[18]<<8)|imu_rx_buf[17])/32768.0)+0.1*imu.yaw);
//					if(imu.yaw>360) imu.yaw -= 360;
//					else if(imu.yaw<0) imu.yaw += 360;
//					printf("imu :%f  %f  %f\r\n",imu.roll,imu.pitch,imu.yaw);
					mpu_flash=~mpu_flash;
				}
			}
		}
		imu_rx_len = 0;
		memset(imu_rx_buf,0,imu_rx_len);
	}
	HAL_UART_Receive_DMA(&huart2,imu_rx_buf,BUFFER_SIZE);
	HAL_UART_IRQHandler(&huart2);
}

// void USART2_IRQHandler(void)
// {

// 	HAL_UART_Receive_DMA(&huart2,&i,u,1);
// 	HAL_UART_IRQHandler(&huart2);
// }

