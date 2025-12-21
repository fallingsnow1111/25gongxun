#ifndef __CIRCLE_control_H
#define __CIRCLE_control_H

#include "struct_typedef.h"
#include "string.h"

#define CENTER_SPEED_EDGE  1.8  //�ٶ�ģʽ�����ֵ�ķ�Χ
#define CENTER_SPEED_EDGE_YUANPANJI 1.5//Բ�̻��Ķ�λ
#define Preliminary_round  0    //����ģʽ
#define Final_round        1    //����ģʽ

inline int SATU(float _IN, float _AIM);

void openmv_Init(void);
void openmv_Data_Reset(void);
void W_Gray_Calibration_openmv_x(void);
void W_Gray_Calibration_openmv_y(void);
void W_Gray_Calibration_openmv(void);
void locate(void);
void locate_2(void);
void locate_3(void);
void Circle_Position_Center(char color,char times);//��λԲ������(λ��ģʽ)
//void Circle_Position_Center_SPEED(char color);//��λԲ������(�ٶ�ģʽ)
void Circle_Position_Center_SPEED_yuanpanji(char color);
void Circle_Position_Center_SPEED(char color);
void Circle_Position_Center_SPEED_with_w(char aimcolor,char fuzucolor,char mode);
void Positioning_yuanpanji(char color);
void Positioning_grand(char color);
#endif

