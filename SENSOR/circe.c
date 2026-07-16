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

int x_zhong=121;
int y_zhong=115;
int change_x = 0xFF;
int change_y = 0xFF;
int change_x_yuanpanji=0xFF;
int	change_y_yuanpanji=0xFF;
static int xy_actual[3];
static int xy_aim[2];

static float change_k=0.0f;

struct PIDstruct pid_change_y;
struct PIDstruct pid_change_x;

uint8_t USART6_readdata[128];
uint8_t USART6_senddata[128];

int USART6_stage = 0;
static uint16_t U6_pitoner=0;

int COLOR_DATA = 0;

volatile VISION_TARGET_T vision_material[4];
volatile VISION_TARGET_T vision_ring[4];
volatile uint8_t vision_last_mode = 0;
volatile uint8_t vision_last_count = 0;
volatile uint32_t vision_frame_count = 0;
volatile uint8_t vision_request_mode = 0;
volatile uint8_t vision_request_cls = 0;

/* MaixCam multi-target frame:
 * 55 5B mode count cls1 x1 y1 cls2 x2 y2 cls3 x3 y3 AA
 */
#define VISION_FRAME_MULTI    0x5B

static void Vision_ClearSelected(void);

void Vision_Receive_Init(void)
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

void Vision_SetMode(uint8_t mode)
{
	if(mode > VISION_MODE_RING)
	{
		return;
	}

	vision_request_mode = mode;
	vision_request_cls = 0;

	if(mode == VISION_MODE_IDLE)
	{
		Vision_ClearSelected();
	}

	U6_send((unsigned char)('0' + mode));
}

void Vision_StartMaterial(void)
{
	Vision_SetMode(VISION_MODE_MATERIAL);
}

void Vision_StartRing(void)
{
	Vision_SetMode(VISION_MODE_RING);
}

void Vision_Stop(void)
{
	Vision_SetMode(VISION_MODE_IDLE);
}

static void Vision_ClearSelected(void)
{
	xy_actual[0] = 0;
	xy_actual[1] = 0;
	xy_actual[2] = 0;
	change_x = 0xFF;
	change_y = 0xFF;
	change_x_yuanpanji = 0xFF;
	change_y_yuanpanji = 0xFF;
	COLOR_DATA = 0;
}

/* Keep old high-level color orders, but MaixCam only receives two detect modes.
 * The selected color is picked locally from the returned three-target cache.
 */
static uint8_t Vision_OrderToModeCls(int order, uint8_t *mode, uint8_t *cls)
{
	switch(order)
	{
		case RED:
		case GREEN:
		case BLUE:
			*mode = VISION_MODE_MATERIAL;
			*cls = (uint8_t)order;
			return 1;
		case RED_CIRCLE:
		case GREEN_CIRCLE:
		case BLUE_CIRCLE:
			*mode = VISION_MODE_RING;
			*cls = (uint8_t)(order - RED_CIRCLE + 1);
			return 1;
		case UP_GREEN:
			*mode = VISION_MODE_MATERIAL;
			*cls = GREEN;
			return 1;
		case ALL_COLOR:
			*mode = VISION_MODE_MATERIAL;
			*cls = 0;
			return 1;
		case 0:
			*mode = VISION_MODE_IDLE;
			*cls = 0;
			return 1;
		default:
			return 0;
	}
}

/* Convert MaixCam class id back to the old COLOR_DATA value used by arm code. */
static int Vision_ModeClsToOrder(uint8_t mode, uint8_t cls)
{
	if(mode == VISION_MODE_RING)
	{
		return (int)(cls + RED_CIRCLE - 1);
	}
	return cls;
}

/* Select the cache table for current vision mode. */
static volatile VISION_TARGET_T *Vision_GetTable(uint8_t mode)
{
	if(mode == VISION_MODE_RING)
	{
		return vision_ring;
	}
	return vision_material;
}

/* Refresh old single-target outputs from the new multi-target cache. */
static void Vision_UpdateSelectedFromCache(void)
{
	uint8_t cls;
	uint8_t selected_cls = 0;
	volatile VISION_TARGET_T *table;
	volatile VISION_TARGET_T *target = 0;

	if(vision_request_mode == VISION_MODE_IDLE)
	{
		Vision_ClearSelected();
		return;
	}

	table = Vision_GetTable(vision_request_mode);

	if(vision_request_cls >= 1 && vision_request_cls <= 3)
	{
		if(table[vision_request_cls].valid)
		{
			target = &table[vision_request_cls];
			selected_cls = vision_request_cls;
		}
	}
	else
	{
		for(cls = 1; cls <= 3; cls++)
		{
			if(table[cls].valid)
			{
				target = &table[cls];
				selected_cls = cls;
				break;
			}
		}
	}

	if(target == 0)
	{
		Vision_ClearSelected();
		return;
	}

	xy_actual[0] = Vision_ModeClsToOrder(vision_request_mode, selected_cls);
	xy_actual[1] = target->x;
	xy_actual[2] = target->y;
	COLOR_DATA = xy_actual[0];
	circle_color_data_deal();
}

/* Legacy API: old code passes one color order, but MaixCam only receives mode.
 * New code should call Vision_StartMaterial/Vision_StartRing and read cache.
 */
void send_NX(int order)
{
	uint8_t mode;
	uint8_t cls;

	if(Vision_OrderToModeCls(order, &mode, &cls) == 0)
	{
		return;
	}

	vision_request_mode = mode;
	vision_request_cls = cls;
	U6_send((unsigned char)('0' + mode));
}

/* Manual LED control is independent from detect mode.
 * MaixCam also turns LED on automatically in mode 1/2 and off in mode 0.
 */
void Vision_SetLED(uint8_t enable)
{
	U6_send(enable ? 'L' : 'l');
}

void Vision_LED_On(void)
{
	Vision_SetLED(1);
}

void Vision_LED_Off(void)
{
	Vision_SetLED(0);
}

/* Clear all cached targets before storing a new frame. */
static void vision_clear_table(volatile VISION_TARGET_T *table)
{
	uint8_t i;
	for(i = 0; i < 4; i++)
	{
		table[i].valid = 0;
		table[i].x = 0;
		table[i].y = 0;
	}
}

/* Decode one complete 0x5B frame and cache every detected color. */
static void u6_multi_target_process(uint8_t *data)
{
	uint8_t mode = data[2];
	uint8_t count = data[3];
	uint8_t i;
	volatile VISION_TARGET_T *table;

	if(mode != VISION_MODE_MATERIAL && mode != VISION_MODE_RING)
	{
		return;
	}

	if(count > 3)
	{
		count = 3;
	}

	table = Vision_GetTable(mode);
	vision_clear_table(table);
	vision_last_mode = mode;
	vision_last_count = count;
	vision_frame_count++;

	for(i = 0; i < count; i++)
	{
		uint8_t cls = data[4 + i * 3];
		if(cls < 1 || cls > 3)
		{
			continue;
		}
		table[cls].valid = 1;
		table[cls].x = data[5 + i * 3];
		table[cls].y = data[6 + i * 3];
	}

	if(mode == vision_request_mode)
	{
		Vision_UpdateSelectedFromCache();
	}
}

/* Binary coordinates may equal 0xAA, so multi-target frames cannot end on
 * "first AA seen". Use frame type and target count to calculate frame length.
 */
static uint8_t U6_IsFrameReady(void)
{
	uint16_t expect_len;

	if(U6_pitoner < 2)
	{
		return 0;
	}

	switch(USART6_readdata[1])
	{
		case VISION_FRAME_MULTI:
			if(U6_pitoner < 4)
			{
				return 0;
			}
			if(USART6_readdata[3] > 3)
			{
				USART6_stage = 0;
				U6_pitoner = 0;
				return 0;
			}
			expect_len = (uint16_t)(5 + USART6_readdata[3] * 3);
			break;
		case 0x5A:
		case 0x51:
			expect_len = 6;
			break;
		default:
			return (USART6_readdata[U6_pitoner - 1] == 0xAA);
	}

	return (U6_pitoner >= expect_len);
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
        case VISION_FRAME_MULTI:
        {
            u6_multi_target_process(data);
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
	if(USART6->ISR&(1<<5))	// RXNE: 接收寄存器非空
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
            if(U6_pitoner >= sizeof(USART6_readdata))
            {
                USART6_stage=0;
                U6_pitoner=0;
            }
            if(U6_IsFrameReady())
            {
                if(USART6_readdata[U6_pitoner - 1] == 0xAA)
                {
                    u6_data_process(USART6_readdata);
                }
                USART6_stage=0;
                U6_pitoner=0;
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
    COLOR_DATA = xy_actual[0];
    if(COLOR_DATA != 0&&xy_actual[1]!=0&&xy_actual[2]!=0)
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

uint8_t Vision_GetTarget(uint8_t mode, uint8_t cls, VISION_TARGET_T *target)
{
	volatile VISION_TARGET_T *table;

	if(target == 0 || cls < 1 || cls > 3)
	{
		return 0;
	}

	table = Vision_GetTable(mode);
	target->valid = table[cls].valid;
	target->x = table[cls].x;
	target->y = table[cls].y;
	return target->valid;
}

uint8_t Vision_GetMaterialTarget(uint8_t cls, VISION_TARGET_T *target)
{
	return Vision_GetTarget(VISION_MODE_MATERIAL, cls, target);
}

uint8_t Vision_GetRingTarget(uint8_t cls, VISION_TARGET_T *target)
{
	return Vision_GetTarget(VISION_MODE_RING, cls, target);
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
	vision_clear_table(vision_material);
	vision_clear_table(vision_ring);
	vision_last_mode = 0;
	vision_last_count = 0;
	Vision_ClearSelected();
}
