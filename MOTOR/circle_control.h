#ifndef __CIRCLE_control_H
#define __CIRCLE_control_H

#include "struct_typedef.h"
#include "string.h"

#define CENTER_SPEED_EDGE  1.8                       // 速度模式定位值范围
#define CENTER_SPEED_EDGE_YUANPANJI 1.5              // 圆盘机定位值
#define Preliminary_round  0                         // 初赛模式
#define Final_round        1                         // 决赛模式

inline int SATU(float _IN, float _AIM);

void openmv_Init(void);
void openmv_Data_Reset(void);
void W_Gray_Calibration_openmv_x(void);
void W_Gray_Calibration_openmv_y(void);
void W_Gray_Calibration_openmv(void);
void locate(void);
void locate_2(void);
void locate_3(void);
void Circle_Position_Center(char color,char times);            // 定位圆盘中心(位置模式)
//void Circle_Position_Center_SPEED(char color);               // 定位圆盘中心(速度模式) - 已弃用
void Circle_Position_Center_SPEED_yuanpanji(char color);
void Circle_Position_Center_SPEED(char color);
void Circle_Position_Center_SPEED_with_w(char aimcolor,char fuzucolor,char mode);
void Positioning_yuanpanji(char color);
void Positioning_grand(char color);
#endif

