#include "usart.h"
#include "GO-M8010-6.h"
#include "crc_ccitt.h"
#include "stdio.h"
#include "gpio.h"
#include "sys.h"
#include "delay.h"
#include <string.h>
#include "pid.h"
#include <math.h>
#define SATURATE(_IN, _MIN, _MAX) {\
 if (_IN < _MIN)\
 _IN = _MIN;\
 else if (_IN > _MAX)\
 _IN = _MAX;\
 } 

static struct PIDstruct M8010_pid; 
 
MOTOR_recv *rData;
MOTOR_send cmd; 
struct ANGLE M8010;
uint8_t m8010_rx_buf[20] = {0};
uint8_t m8010_tx_buf[17] = {0xFE,0xEE,0x00,0x00,0x00,0x53,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x00,0x5C,0x36};
uint8_t m8010_rx_len,lenth = 0;
uint32_t M8010_real_postion,M8010_init_postion = 0;
uint8_t m8010_hight_bit,m8010_low_bit = 0;

void M8010_init(void)
{
		PID_Init(&M8010_pid,1,0.5,0,20,-20);
    __HAL_UART_ENABLE_IT(&huart8, UART_IT_RXNE);
}
void read_init_postion(void)
{
    cmd.id=0; 			//���������ָ��ṹ�帳ֵ
	cmd.mode=0;
	cmd.T=0;
	cmd.W=10;
	cmd.Pos=0 ;
	cmd.K_P=0.3;
	cmd.K_W=0.1;
    modify_data(&cmd);
    SET_485_DE_UP();
    SET_485_RE_UP();
    HAL_UART_Transmit(&huart8, (uint8_t *)&cmd, sizeof((&cmd)->motor_send_data), 10);
    SET_485_RE_DOWN();
	SET_485_DE_DOWN();
	
    vTaskDelay(pdMS_TO_TICKS(5));
    M8010_init_postion = M8010_real_postion;
}

int modify_data(MOTOR_send *motor_s)
{
    motor_s->hex_len = 17;
    motor_s->motor_send_data.head[0] = 0xFE;
    motor_s->motor_send_data.head[1] = 0xEE;
	
//		SATURATE(motor_s->id,   0,    15);
//		SATURATE(motor_s->mode, 0,    7);
		SATURATE(motor_s->K_P,  0.0f,   25.599f);
		SATURATE(motor_s->K_W,  0.0f,   25.599f);
		SATURATE(motor_s->T,   -127.99f,  127.99f);
		SATURATE(motor_s->W,   -804.00f,  804.00f);
		SATURATE(motor_s->Pos, -411774.0f,  411774.0f);

    motor_s->motor_send_data.mode.id   = motor_s->id;
    motor_s->motor_send_data.mode.status  = motor_s->mode;
    motor_s->motor_send_data.comd.k_pos  = motor_s->K_P/25.6f*32768;
    motor_s->motor_send_data.comd.k_spd  = motor_s->K_W/25.6f*32768;
    motor_s->motor_send_data.comd.pos_des  = motor_s->Pos/6.2832f*32768 + M8010_init_postion;
    motor_s->motor_send_data.comd.spd_des  = motor_s->W/6.2832f*256;
    motor_s->motor_send_data.comd.tor_des  = motor_s->T*256;
    motor_s->motor_send_data.CRC16 = crc_ccitt(0, (uint8_t *)&motor_s->motor_send_data, 15);
    return 0;
}


void M8010_send(int position) {
    const float tolerance = 1.0f;         // 允许1度误差
    const uint32_t timeout_ms = 5000;     // 5秒超时
    const uint32_t uart_timeout = 50;     // 单次UART发送超时50ms
    uint32_t start_time = HAL_GetTick();
    HAL_StatusTypeDef uart_ret;

    cmd.id = 0;
    cmd.mode = 1;
    cmd.T = 0;
    cmd.W = 0;
    cmd.K_P = 1.0;
    cmd.K_W = 0.15;

    M8010.TARGE = position;
    M8010.CHANGE = M8010.TARGE - M8010.NOW;

    while((__fabs(M8010.CHANGE) > tolerance))
    {
        if(HAL_GetTick() - start_time > timeout_ms) {
            break;
        }

        int step = 0;
        if(__fabs(M8010.CHANGE) >= 10.0f) {
            step = (M8010.CHANGE > 0) ? 10 : -10;
        } else {
            step = (M8010.CHANGE > 0) ? 1 : -1;
        }

        M8010.NOW += step;

        if(M8010.NOW > 400.0f || M8010.NOW < (-50)) {
            M8010.NOW = (M8010.NOW > 0) ? 280.0f : -50;
        }
        M8010.CHANGE = M8010.TARGE - M8010.NOW;

        cmd.Pos = (float)M8010.NOW / 180.0f * 3.1415926f * 6.33f;
        modify_data(&cmd);

        SET_485_DE_UP();
        SET_485_RE_UP();
        HAL_Delay(1);

        uart_ret = HAL_UART_Transmit(&huart8, (uint8_t *)&cmd.motor_send_data,
                                     sizeof(cmd.motor_send_data), uart_timeout);
        HAL_Delay(1);
        SET_485_RE_DOWN();
        SET_485_DE_DOWN();

        if(uart_ret != HAL_OK) {
            // UART发送失败（超时/BUSY/ERROR），退出循环
            break;
        }

        HAL_Delay(10);
    }
}

// 软件零点设置函数
void M8010_Set_Zero(void) {
    M8010.NOW = 0;
    M8010.TARGE = 0;
    M8010.CHANGE = 0;
    read_init_postion();
}


void M8010_SetAngle(int tar_angle)
{
		static int change_var=0;
		change_var=__fabs((tar_angle - M8010.NOW));
		M8010_send(tar_angle);
		vTaskDelay(pdMS_TO_TICKS((change_var)*2));
}

float M8010_Control_pid(float differnt,float Initial_value)
{
	float output=0;
	//����PID
	output=PID_Compute(&M8010_pid,0,differnt);
	//������Сֵ�޷�
	output=_LIMIT_MIN(output,3);
	//������
	M8010_SetAngle(output+Initial_value);
	return output+Initial_value;
}

uint32_t M8010_ShowRealPostion(void)
{
	return M8010_real_postion;
}

void UART8_IRQHandler(void)
 {
    unsigned char res;	
	if(UART8->ISR&(1<<5))	//?����?��?��y?Y
	{
		res=UART8->RDR; 
		if(res == 0xFD && lenth == 0)
		{
			lenth = 1;
            m8010_rx_buf[0] = res;
		}
		else if(res == 0xEE && lenth == 1)
		{
			lenth = 2;
            m8010_rx_buf[1] = res;
            m8010_rx_len = 2;
		}
		else if(lenth == 2 && m8010_rx_len<16)
		{
            m8010_rx_buf[m8010_rx_len] = res;
            m8010_rx_len++;
            if(m8010_rx_len == 16)
            {
                uint16_t sum = 0;
                sum = crc_ccitt(0,m8010_rx_buf,14);
                m8010_low_bit = (sum&0XFF);
                m8010_hight_bit = (sum>>8);
                if(m8010_rx_buf[14] == m8010_low_bit && m8010_rx_buf[15] == m8010_hight_bit)
                {
                    M8010_real_postion = (m8010_rx_buf[10] << 24) | (m8010_rx_buf[9]  << 16) | (m8010_rx_buf[8]  << 8) | m8010_rx_buf[7] ;
				}
                lenth = 0;
                memset(m8010_rx_buf,0,20);
            }
		}
        else
        {
            lenth = 0;
            memset(m8010_rx_buf,0,20);
        }		
	}
   __HAL_UART_CLEAR_OREFLAG(&huart8);    
}

