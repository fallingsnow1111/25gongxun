#ifndef __USER_H
#define __USER_H

#include "main.h"

int Find_Color_Position(char color_code);
void Take_Material_Processing_Area(char times);
void Put_Material_Processing_Area(char times,char layer,char mode,char area);
void Take_Material_Processing_Area_ProMax(char times);//times为第几次的2维码
void PUT_material_on_yuanpanji(char times);
#define jing_area 0  //精加工区域
#define cu_area 1   //粗加工区域

#endif
