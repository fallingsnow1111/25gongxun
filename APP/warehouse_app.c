#include "warehouse_app.h"
#include "catch.h"


WASEHOUSE_T car_warehouse;

void Init_Warehouse(char times)
{
    switch (times)
    {
        case 1://第1次
        {
            car_warehouse.firse.color = one.firse;   // 第一个仓库颜色初始化
            car_warehouse.second.color = one.second;  // 第二个仓库颜色初始化
            car_warehouse.thrid.color=one.thrid;     //第3个仓库颜色初始化
        }break;
        case 2://第2次
        {
            car_warehouse.firse.color = two.firse;   // 第一个仓库颜色初始化
            car_warehouse.second.color = two.second;  // 第二个仓库颜色初始化
            car_warehouse.thrid.color=two.thrid;     //第3个仓库颜色初始化
        }break;
        default:
            break;
    }
    car_warehouse.firse.angle = FIRST_WAREHOUSE; // 第一个仓库角度初始化
    car_warehouse.second.angle = SECOND_WAREHOUSE; // 第二个仓库角度初始化
    car_warehouse.thrid.angle = THIRD_WAREHOUSE; // 第三个仓库角度初始化
}

void Set_Warehouse_Color(int warehouse_index, uint8_t color)
{
    switch (warehouse_index) {
        case 0:
            car_warehouse.firse.color = color; // 设置第一个仓库颜色
            break;
        case 1:
            car_warehouse.second.color = color; // 设置第二个仓库颜色
            break;
        case 2:
            car_warehouse.thrid.color = color; // 设置第三个仓库颜色
            break;
        default:
            break; // 无效的仓库索引
    }
}

int Get_Warehouse_Color(int warehouse_index)
{
    switch (warehouse_index) {
        case 0:
            return car_warehouse.firse.color; // 获取第一个仓库颜色
        case 1:
            return car_warehouse.second.color; // 获取第二个仓库颜色
        case 2:
            return car_warehouse.thrid.color; // 获取第三个仓库颜色
        default:
            return -1; // 无效的仓库索引
    }
}

uint8_t Get_Warehouse_index_from_color(int color)
{
    char index=0;
    if(car_warehouse.firse.color==color)
    {
        index=0;
    }
    else if (car_warehouse.second.color==color)
    {
        index=1;
    }
    else if (car_warehouse.thrid.color==color)
    {
        index=2;
    }
    else
    {
        index=-1;
    }
    return index;
}

void Set_Warehouse_Angle(int warehouse_index, int angle)
{
    switch (warehouse_index) {
        case 0:
            car_warehouse.firse.angle = angle; // 设置第一个仓库角度
            break;
        case 1:
            car_warehouse.second.angle = angle; // 设置第二个仓库角度
            break;
        case 2:
            car_warehouse.thrid.angle = angle; // 设置第三个仓库角度
            break;
        default:
            break; // 无效的仓库索引
    }
}

int Get_Warehouse_Angle(int warehouse_index)
{
    switch (warehouse_index) {
        case 0:
            return car_warehouse.firse.angle; // 获取第一个仓库角度
        case 1:
            return car_warehouse.second.angle; // 获取第二个仓库角度
        case 2:
            return car_warehouse.thrid.angle; // 获取第三个仓库角度
        default:
            return -1; // 无效的仓库索引
    }
}

//取走仓库的角度
int GET_Warehouse_Angle_take(uint8_t warehouse_index)
{
     switch (warehouse_index) {
        case 0:
            return car_warehouse.firse.angle+360; // 获取第一个仓库角度
        case 1:
            return car_warehouse.second.angle+360; // 获取第二个仓库角度
        case 2:
            return car_warehouse.thrid.angle+360; // 获取第三个仓库角度
        default:
            return -1; // 无效的仓库索引
    }
}











