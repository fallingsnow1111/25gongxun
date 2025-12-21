#include "catch.h"
#include "QR_code.h"
#include "tim.h"
#include "postion_control.h"
#include "usart.h"
#include "motor_control.h"
#include "motor_control.h"
#include "circe.h"
#include "motor.h"
#include "circle_control.h"
#include "GO-M8010-6.h"
#include <math.h>
#include "main.h"
#include "telescopic_boom.h"
#include <math.h>
#include "fast_math.h"
#include "action_control.h"
#include "warehouse_app.h"
#include "main_task.h"
#include "action_sets.h"

struct User_parameter_t car_lift; 

///夹爪控制
void claw_move(int action)//圆盘   下压到最低不用这个
{   
    switch(action)
    {
        case 1:
            __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,120);//OPEN
            break;
        case 2:
            __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,198);//初赛145，决赛157
            break;
         default:
            break;
    }
}
void claw_move_1(int action_1)//放置   应对下压碰到物料
{
    switch(action_1)
    {
        case 1:
            __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,170);//OPEN//120
            break;
        case 2:
            __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,198);//初赛145，决赛157
            break;
         default:
            break;
    }
}

void claw_move_2(int action_2)//应对爪子转圈碰到物料的问题//用这个张开
{
    switch(action_2)
    {
        case 1:
            __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,165);//OPEN//120
            break;
        case 2:
            __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,198);//初赛145，决赛157
            break;
         default:
            break;
    }
}

void Put_on_house(int color)
{
	Z_SetHeight(0);//先拉到最高
	Y_SetLength(Y_HEIGHT_WAREHOUSE);//将伸缩臂收回
	//转到指定颜色的物料仓
	uint8_t warehouse_index=Get_Warehouse_index_from_color(color);
	int angle=Get_Warehouse_Angle(warehouse_index);
	M8010_SetAngle(angle);
	// switch (color)
	// {
	// 	case red:
	// 	{
	// 		M8010_SetAngle(RED_WAREHOUSE);
	// 	}break;
	// 	case green:
	// 	{
	// 		M8010_SetAngle(GREEN_WAREHOUSE);
	// 	}break;
	// 	case blue:
	// 	{
	// 		M8010_SetAngle(BLUE_WAREHOUSE);
	// 	}break;
	// }
	//放置物料
	Z_SetHeight(65);
	claw_move_2(open);
	Delay_ms(75);
	Z_SetHeight(10);
	
}

///圆盘机抓取
void catch_yuan_pan_ji(int color)
{	
	Z_SetHeight(10);//先拉高
	claw_move(open);//张开爪子
    Z_SetHeight(95);//下降到圆盘机的物料的高度,todo:下降为零（暂定为0）
	claw_move_2(close);
	Delay_ms(170);//等待夹爪闭合
	//while(car.action_set.finish_flag!=finish);
	Put_on_house(color);
}

///地面抓取（预测决赛）
void catch_material_grand(int color)
{	
	claw_move(open);//张开爪子
	Delay_ms(80);
    Z_SetHeight(160);//下降到圆盘机的物料的高度,todo:下降为零（暂定为0）
	claw_move_2(close);
	//vTaskDelay(pdMS_TO_TICKS(170));
	Delay_ms(400);
	Put_on_house(color);
}


//第几次抓取圆盘机——伸缩臂XY
void Catch_Material_YUAN_PAN_JI_with_TELESCOPIC(int times)//抓取物料，参数为 1 or 2
{	
	struct PIDstruct X_coordination_pid;
	PID_Init(&X_coordination_pid,0.65,0.02,0,5,-5);
	static int color_data[3];
	Set_Circle_Center(0,0);//设置中心点，暂定为0,0
	int x_zhong=Get_X_Change();
	int y_zhong=Get_Y_Change();
	float x_output=0;
	M8010_SetAngle(PUT_AND_CATCH_ANGLE);
	Delay_ms(190);
	switch(times)
	{
		case 1:
		{
			color_data[0]=one.firse;
			color_data[1]=one.second;
			color_data[2]=one.thrid;
		}break;
		case 2:
		{
			color_data[0]=two.firse;
			color_data[1]=two.second;
			color_data[2]=two.thrid;
		}break;
	}
		
		for(char i=0;i<3;i++)
		{
			send_NX(color_data[i]);
			while(1)
			{
					if(x_zhong!=0xFF&&y_zhong!=0xFF)
					{
						Telescopic_control_pid(y_zhong);//伸缩臂来负责y坐标
						x_output=PID_Compute(&X_coordination_pid,0,x_zhong);//底盘只负责x坐标
						if(__fabs(x_output)<2)
						{
							if(x_output>0){x_output=1.3f;}
							else if(x_output<0){x_output=-1.3f;}
						}
						Motor_setspeed(x_output,0,0);
						Delay_ms(15);
						
						if(((x_zhong)<=1)&&((y_zhong)<=1))
						{
							break;
						}
					}
			}
			//跳出循坏则证明定位到了
			catch_yuan_pan_ji(color_data[i]);//抓取并放置
			//如果是最后一次抓取完则不用回到PUT_AND_CATCH_ANGLE
			if(i<2)
			{
				M8010_SetAngle(PUT_AND_CATCH_ANGLE);
			}
			else
			{
				//M8010_SetAngle(PUT_HOUSE_ANGLE);
			}
		}
}

//极坐标
void Catch_TELESCOPIC_Polar_coordinates(char color,float car_angle)
{
	//the angle restrictions of M8010
	static int ANGLE_MAX=220,ANGLE_MIN=140;
	static float	DETA_ANGLE_MAX=3;
	int x_y_change[2],color_flag=0xFF;
	Set_Circle_Center(117,115);
	x_y_change[0]=Get_X_Change_yuanpanji();
	x_y_change[1]=Get_Y_Change_yuanpanji();
	float s_r=0,deta_angle=0;
	uint16_t count=0;
	while (1) 
	{
		send_NX(color);  // transmit the color data
		Delay_ms(6);    // delay 10ms
		if(count>=1667)//如果超过10s则直接退出
		{
			break;
		}
		count++;
		// get the current coordinates
		x_y_change[0] = Get_X_Change_yuanpanji();
		x_y_change[1] = Get_Y_Change_yuanpanji();
		color_flag=Get_data_action_flag();
		if(color_flag!=color)
		{
			continue;
		}
		// judge whether the coordinates are valid
		if (x_y_change[0] != 0xFF && x_y_change[1] != 0xFF) {
			// if the coordinates approach the target center 
			if (__fabs(x_y_change[0]) <= 2 && __fabs(x_y_change[1]) <= 2) {
				for (char i = 0; i < 4; i++)
				{
					Delay_ms(6);
					Telescopic_Send_Speed(0);  // 伸缩臂停止
				}
				break;  // 退出循环
			}

			// use the kx^3 formula to calculate the angle increment,the k is 0.00001
			deta_angle = -(0.075 * x_y_change[0])+0.05;  //the value of deta_angle is negative
			//limit the angle increment to a maximum value
			if(__fabs(deta_angle)>=DETA_ANGLE_MAX)
			{
				if(deta_angle>=0)
				{
					deta_angle=DETA_ANGLE_MAX;
				}
				else{deta_angle=-DETA_ANGLE_MAX;}
			} 
			if(__fabs(x_y_change[0])<8 || __fabs(x_y_change[1])<8)
			{
				if(deta_angle>0)
				{
					deta_angle=0.15;
				}
				else
				{
					deta_angle=-0.15;
				}
			}
			else if(__fabs(x_y_change[0])<=3)
			{
				deta_angle=0;
			}
			//vofa_printf("deta_angle:%f\n",deta_angle);
			car_angle += deta_angle;
			// the limit of car_angle from 140° to 220°
			if (car_angle >= ANGLE_MAX){
				car_angle = ANGLE_MAX;
			} else if (car_angle <= ANGLE_MIN) {
				car_angle = ANGLE_MIN;
			}
			// adjust the telescopic amount based on the Y deviation
			s_r = -x_y_change[1];  // Y deviation → telescopic height
			Telescopic_control_pid((int)s_r);  // control the telescopic boom with PID
			M8010_SetAngle(car_angle);    // set the new angle for the GO-M8010-6
		}
		else{
			//if the coordinates are invalid, stop the telescopic boom
			//for (char i = 0; i < 3; i++) {
				Delay_ms(5);
				Telescopic_Send_Speed(0);  // stop the telescopic boom
			//}
		}
  }
}


//catch yupanji with polar_coordination
void Catch_Material_YUAN_PAN_JI_with_TELESCOPIC_Polar_coordinates(int times)//抓取物料，参数为 1 or 2
{
	static int color_data[3];
	//M8010_SetAngle(PUT_AND_CATCH_ANGLE);
	//等待圆盘机动作组完成 
	Wait_other_task_finish(Get_action_task_notify()); // 等待动作组完成
	switch(times)
	{
		case 1:
		{
			color_data[0]=one.firse;
			color_data[1]=one.second;
			color_data[2]=one.thrid;
		}break;
		case 2:
		{
			color_data[0]=two.firse;
			color_data[1]=two.second;
			color_data[2]=two.thrid;
		}break;
	}
		
		for(char i=0;i<3;i++)
		{
			Y_SetLength(85);//make the paw among 85mm to arrive the center of yuanpanji
			Catch_TELESCOPIC_Polar_coordinates(color_data[i],PUT_AND_CATCH_ANGLE);
			Delay_ms(3);
			if (i<=1)
			{
				send_NX(color_data[i+1]);//send the next color data
			}
			catch_yuan_pan_ji(color_data[i]);
			//M8010 doesn't move PUT_AND_CATCH_ANGLE if it's the last time to catch
			if(i<2)
			{
				M8010_SetAngle(PUT_AND_CATCH_ANGLE);
			}
		}
}

///catch the Processing area
void catch_half_stage(int color)
{		
	uint8_t warehouse_index=Get_Warehouse_index_from_color(color);
	car_lift.lift_angle=Get_Warehouse_Angle(warehouse_index);
    switch(color)
    {
        case RED:
					//car_lift.lift_angle=RED_WAREHOUSE;
					car_lift.change_angle=RED_PUT_AREA_ANGLE;
					car_lift.length=RED_PUT_AREA_HEIGHT;
            break;
        case GREEN:
					//car_lift.lift_angle=GREEN_WAREHOUSE;
					car_lift.change_angle=GREEN_PUT_AREA_ANGLE;
					car_lift.length=GREEN_PUT_AREA_HEIGHT;
            break;
        case BLUE:
					//car_lift.lift_angle=BLUE_WAREHOUSE;
					car_lift.change_angle=BLUE_PUT_AREA_ANGLE;
					car_lift.length=BLUE_PUT_AREA_HEIGHT;
            break;
         default:
            break;
    }
		M8010_SetAngle(car_lift.change_angle);//revolve the M8010
		Y_SetLength(car_lift.length);
		claw_move_2(open);
		Z_SetHeight(170);//识别完成，下降到决赛抓取高度
		claw_move(close);
		Delay_ms(120);
		Z_SetHeight(50);//抓取完成，上升到安全高度
		Y_SetLength(Y_HEIGHT_WAREHOUSE);
		M8010_SetAngle(car_lift.lift_angle);
		Z_SetHeight(65);
		claw_move_1(open);
		Delay_ms(100);
		Z_SetHeight(40);
		//M8010_SetAngle(PUT_AND_CATCH_ANGLE);
}

///放置物料到目标地
void Put_Layer(int layer,int color, int mode)//layer(1 or 2)，color(red,green,blue),mode(P_round or F_round)
{	
	//decision the height of Z axis based on the layer
	uint8_t Z_H=0;
	uint8_t warehouse_index=Get_Warehouse_index_from_color(color);
	car_lift.lift_angle=GET_Warehouse_Angle_take(warehouse_index);//出仓库用这个角度
  switch(color)
    {
        case RED:
            //car.need_color=red;
				    //car_lift.lift_angle=RED_WAREHOUSE;
					car_lift.change_angle=RED_PUT_AREA_ANGLE;
					car_lift.length=RED_PUT_AREA_HEIGHT;
            break;
        case GREEN:
            //car.need_color=green;
					//car_lift.lift_angle=GREEN_WAREHOUSE;
					car_lift.change_angle=GREEN_PUT_AREA_ANGLE;
					car_lift.length=GREEN_PUT_AREA_HEIGHT;
            break;
        case BLUE:
            //car.need_color=blue;
					//car_lift.lift_angle=BLUE_WAREHOUSE;
					car_lift.change_angle=BLUE_PUT_AREA_ANGLE;
					car_lift.length=BLUE_PUT_AREA_HEIGHT;
            break;
         default:
            break;
    }
		
	switch (layer)
		{
			case 1://first layer
			{		
				Z_H=LOW_Z_HEIGHT;
			}break;
			case 2://second layer
			{
				Z_H=HIGH_Z_HEIGHT;
			}break;
		}
		Y_SetLength(Y_HEIGHT_WAREHOUSE);//Unique length
		// claw_move_2(open);
		M8010_SetAngle(car_lift.lift_angle);//There are 3 angles here, corresponding to the red, green and blue warehouses
		claw_move_1(open);//应对下压碰到物料
		Delay_ms(80);
		Z_SetHeight(75);//decline to the height of the warehouse
		claw_move(close);
		Delay_ms(80);
		Z_SetHeight(50);
		if(layer==2)
		{
			Y_SetLength(2);
		}
		M8010_SetAngle(car_lift.change_angle);
		Y_SetLength(car_lift.length);
		if (mode==P_round)
		{
			Z_SetHeight(Z_H);
		}
		//Todo:wait until F_round,to do the value of Z height in the final_round
		else if(mode==F_round)	
		{
			Z_SetHeight(129);
		}
		claw_move_1(open);
		Delay_ms(80);
		Z_SetHeight(40);
		
}


///加工区抓取(决赛)
void catch_half_stage_ProMax(int color,int position)
{		
		Z_SetHeight(0);
		Move_To_Target_Postion(position,0,0,1);
    switch(color)
    {
        case 1://red
            car_lift.need_color=(red);
            break;
        case 2://green
            car_lift.need_color=(green);
            break;
        case 3://blue
            car_lift.need_color=(blue);
            break;
         default:
            break;
    }
		//Delay_ms(150);
		claw_move_2(1);
		Z_SetHeight(130);//识别完成，下降到抓取高度
		claw_move(close);
		Delay_ms(120);
		Z_SetHeight(0);//抓取完成，上升到安全高度
		M8010_SetAngle(PUT_HOUSE_ANGLE);//旋转云台
		Delay_ms(100);
		Z_SetHeight(15);//下降到放置高度初赛20,决赛15
		//claw_move_1(1);
		claw_move_2(1);
		Delay_ms(80);
		Z_SetHeight(0);
		M8010_SetAngle(PUT_AND_CATCH_ANGLE);
}

void remove_material(int color)
{
	Z_SetHeight(50);
	claw_move_2(open);
	Y_SetLength(Y_HEIGHT_WAREHOUSE);
	int index=Get_Warehouse_index_from_color(color);
	M8010_SetAngle(GET_Warehouse_Angle_take(index));
	Z_SetHeight(75);
	claw_move(close);
	Delay_ms(80);
	Z_SetHeight(50);
}

///圆盘机放置(预测决赛)，xiandian
void put_yuan_pan_ji(int color)
{	
	Set_Circle_Center(205,175);
	send_NX(color);
	remove_material(color);//取出仓库的物料
	M8010_SetAngle(220);
	Z_SetHeight(75);
	int x_piont=0xFF,y_piont=0xFF;
	do
	{
		//send_NX(color);
		x_piont=Get_X_Change();
		y_piont=Get_Y_Change();
		vTaskDelay(20);
		if(__fabs(x_piont)<=3 && __fabs(x_piont)<=3)
		{
			break;
		}
	} while (1);
	M8010_SetAngle(175);
	Delay_ms(100);
	claw_move_1(open);
	Delay_ms(80);
	Z_SetHeight(40);
}


void Catch_material_on_grand(char times)
{
	static int color_data[3];
	int grand_color[3][2] = {{-1, -1}, {-1, -1}, {-1, -1}};
	int flag=-1; 
	Z_SetHeight(40);//先拉高
	//M8010_SetAngle(PUT_AND_CATCH_ANGLE);
	//等待圆盘机动作组完成 
	//Wait_other_task_finish(Get_action_task_notify()); // 等待动作组完成
	switch(times)
	{
		case 1:
		{
			color_data[0]=one.firse;
			color_data[1]=one.second;
			color_data[2]=one.thrid;
		}break;
		case 2:
		{
			color_data[0]=two.firse;
			color_data[1]=two.second;
			color_data[2]=two.thrid;
		}break;
	}
	for(char i=0;i<3;i++)
	{
		M8010_SetAngle(PUT_AND_CATCH_ANGLE);
		Catch_TELESCOPIC_Polar_coordinates(color_data[i],PUT_AND_CATCH_ANGLE);
		catch_material_grand(color_data[i]);
  	}
	

}
