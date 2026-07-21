#include "warehouse_app.h"
#include "catch.h"

WASEHOUSE_T car_warehouse;

/* 根据第一轮或第二轮二维码建立物料颜色与三个仓位的映射。 */
void Init_Warehouse(char times)
{
	if(times == 1)
	{
		car_warehouse.firse.color = one.firse;
		car_warehouse.second.color = one.second;
		car_warehouse.thrid.color = one.thrid;
	}
	else if(times == 2)
	{
		car_warehouse.firse.color = two.firse;
		car_warehouse.second.color = two.second;
		car_warehouse.thrid.color = two.thrid;
	}

	car_warehouse.firse.angle = FIRST_WAREHOUSE;
	car_warehouse.second.angle = SECOND_WAREHOUSE;
	car_warehouse.thrid.angle = THIRD_WAREHOUSE;
}

int Get_Warehouse_Angle(int warehouse_index)
{
	switch(warehouse_index)
	{
		case 0: return car_warehouse.firse.angle;
		case 1: return car_warehouse.second.angle;
		case 2: return car_warehouse.thrid.angle;
		default: return -1;
	}
}

uint8_t Get_Warehouse_index_from_color(int color)
{
	if(car_warehouse.firse.color == color)
		return 0;
	if(car_warehouse.second.color == color)
		return 1;
	if(car_warehouse.thrid.color == color)
		return 2;
	return 0xFF;
}
