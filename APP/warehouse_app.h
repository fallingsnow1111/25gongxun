#ifndef __WAREHOUSE_APP_H
#define __WAREHOUSE_APP_H

#include "main.h"
void Init_Warehouse(char times);
void Set_Warehouse_Color(int warehouse_index, uint8_t color);
int Get_Warehouse_Color(int warehouse_index);
void Set_Warehouse_Angle(int warehouse_index, int angle);
int Get_Warehouse_Angle(int warehouse_index);
uint8_t Get_Warehouse_index_from_color(int color);
//取走仓库的角度
int GET_Warehouse_Angle_take(uint8_t warehouse_index);

#endif

