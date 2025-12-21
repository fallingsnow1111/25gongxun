#include "project.h"
#include "gpio.h"
#include "motor_control.h"
#include "QR_code.h"
#include "hmi.h"
#include "catch.h"
#include "usart.h"
#include "motor.h"
#include "pid.h"
int task_code = 0;//二位码序号

//void turn_case(int angle)//left turn
//{
//	GLOBAL_DATA.CHANHE_SAGE = 0;//关闭激光
//	//Move_To_Target_line_x2(0,20,0,angle);
//	//Move_To_Target_line_y1(30,0,0,angle);
//  Move_To_Target_angle(40,0,110,angle+93,2000);//转弯90 (30,0,40,angle+93,2500)//40 0 110 93 2000
//}
//void turn_case_right(int angle)//left turn
//{
//	GLOBAL_DATA.CHANHE_SAGE = 0;//关闭激光
//	//Move_To_Target_line_x2(0,20,0,angle);
//	//Move_To_Target_line_y1(30,0,0,angle);
//  Move_To_Target_angle(-60,0,-90,angle-93,650);//转弯90
//}


