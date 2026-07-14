#ifndef __CIRCLE_H
#define __CIRCLE_H

#include "struct_typedef.h"
#define RED 1
#define GREEN 2
#define BLUE 3

#define RED_CIRCLE 4
#define GREEN_CIRCLE 5
#define BLUE_CIRCLE 6

#define UP_GREEN 7
#define ALL_COLOR 8

/* MaixCam mode command: STM32 sends ASCII '0'/'1'/'2'. */
#define VISION_MODE_IDLE      0
#define VISION_MODE_MATERIAL  1
#define VISION_MODE_RING      2

/* One cached target returned by MaixCam multi-target frame. */
typedef struct
{
	uint8_t valid;
	uint8_t x;
	uint8_t y;
} VISION_TARGET_T;


extern int change_x,change_y,x_zhong,y_zhong ;
extern int COLOR_DATA ;
extern uint8_t USART6_senddata[128];

/* Index 1/2/3 maps to red/green/blue. Index 0 is unused. */
extern volatile VISION_TARGET_T vision_material[4];
extern volatile VISION_TARGET_T vision_ring[4];
extern volatile uint8_t vision_last_mode;
extern volatile uint8_t vision_last_count;
extern volatile uint32_t vision_frame_count;

void U6_Init(void);
void circle_color_data_deal(void);
void U6_send(unsigned char data);
void uart6WriteBuf(uint8_t *buf, uint8_t len);

/* Preferred mode API: commands MaixCam to detect all colors in one mode. */
void Vision_SetMode(uint8_t mode);
void Vision_StartMaterial(void);
void Vision_StartRing(void);
void Vision_Stop(void);

/* Legacy color-order API. Keep for old code; new code should use Vision_SetMode. */
void send_NX(int order);

/* Manual fill-light control: sends 'L' or 'l' to MaixCam. */
void Vision_LED_On(void);
void Vision_LED_Off(void);
void Vision_SetLED(uint8_t enable);
void USART6_readdata_SeetZero(void);

/* Read one cached target from material/ring table. cls: 1=red, 2=green, 3=blue. */
uint8_t Vision_GetTarget(uint8_t mode, uint8_t cls, VISION_TARGET_T *target);
uint8_t Vision_GetMaterialTarget(uint8_t cls, VISION_TARGET_T *target);
uint8_t Vision_GetRingTarget(uint8_t cls, VISION_TARGET_T *target);

int Get_X_Change(void);
int Get_Y_Change(void);
void Set_Circle_Center(int x_z,int y_z);
int Get_data_action_flag(void);
int Get_x_actual(void);
int Get_y_actual(void);
int* Get_xy_aim(void);
int Get_X_Change_yuanpanji(void);
int Get_Y_Change_yuanpanji(void);
float Get_change_k(void);
void MY_USART6_IRQHandler(void);
#endif

