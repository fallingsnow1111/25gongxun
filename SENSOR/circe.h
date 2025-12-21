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


extern int change_x,change_y,x_zhong,y_zhong ;
extern int COLOR_DATA ;
extern uint8_t USART6_senddata[128];

void U6_Init(void);
void circle_color_data_deal(void);
void U6_send(unsigned char data);
void uart6WriteBuf(uint8_t *buf, uint8_t len);
void send_NX(int order);
void USART6_readdata_SeetZero(void);

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

