#include "circle_control.h"
#include "motor_control.h"
#include "motor.h"
#include "IMU.h"
#include "pid.h"
#include "circe.h"
#include "main_task.h"

static inline int SATU(float _IN, float _AIM)
{
    if (_IN < 0)
        return -_AIM;
    else if (_IN > 0)
        return _AIM;
    return 0;
}

int times = 0;
void openmv_Init(void)
{
	U6_Init();//初始化串口6
	PID_Init(&Pid_Circle_Positioning,0.65,0.02,0,10,-10);//初始化圆环定位的PID
}

#define Y_DIVISOR (float)10
#define X_DIVISOR (float)9.5
#define COUNT  1

//定位圆的中心,color为定位的圆的圆心，times为定位多少次一般为2次（用位置模式来定位）
void Circle_Position_Center(char color,char times)
{	
	char count=0;
	int val_x[10]={0};
	int val_y[10]={0};
	int sum_x=0;
	int sum_y=0;
	float x_average,y_average;
	if(color==7||color==ALL_COLOR)//upgreen or ALL_COLOR
	{
		Set_Circle_Center(88,86);
	}
		else
	{
		Set_Circle_Center(117,127);
	}
	send_NX(color);
	M8010_SetAngle(PUT_AND_CATCH_ANGLE);
	Delay_ms(50);
	if(color!=ALL_COLOR)
	{Z_SetHeight(-40);}
	for(char i=0;i<times;i++)
	{
		count=0;
		Delay_ms(20);
		while(count<COUNT)
		{
			send_NX(color);
			Delay_ms(2);
			circle_color_data_deal();			
			if(Get_X_Change()!=0xFF && Get_Y_Change()!=0xFF)
			{
				val_x[count]=Get_X_Change();
				val_y[count]=Get_Y_Change();
				sum_x=sum_x+val_x[count];
				sum_y=sum_y+val_y[count];
				count++;
			}
		}
		x_average=sum_x/COUNT;
		y_average=sum_y/COUNT;
		Move_To_Target_Postion((float)y_average / Y_DIVISOR, (float)x_average / X_DIVISOR, 0,0);
		sum_x=0;
		sum_y=0;
	}
	Z_SetHeight(0);
}


void Circle_Position_Center_SPEED_yuanpanji(char color) {
    int x_average ;
    int y_average ;
    float x_valspeed, y_valspeed;
		//设置中心点
    Set_Circle_Center(118, 130);
		//将视觉读取的坐标的地址赋值
	x_average = Get_X_Change();
	y_average = Get_Y_Change();
	
    send_NX(color);
    M8010_SetAngle(PUT_AND_CATCH_ANGLE);
    Z_SetHeight(-20);
	
    // 等待初始条件
//    char temp_flag = 1;
//    while (temp_flag) {
//        if ((*Get_X_Change()) != 0xFF && (*Get_Y_Change())!= 0xFF) {
//            temp_flag = 0;
//            break;
//        }
//    }
//    Delay_ms(1000);

    while (1) {
        Delay_ms(10);
        if (x_average != 0xFF && y_average != 0xFF) {
            // PID计算
            y_valspeed = PID_Compute(&Pid_Circle_Positioning, y_average, 0);
            x_valspeed = PID_Compute(&Pid_Circle_Positioning, x_average, 0);

            // 限制速度范围
            if (__fabs(y_valspeed) < CENTER_SPEED_EDGE_YUANPANJI) {
                y_valspeed = SATU(y_valspeed, CENTER_SPEED_EDGE_YUANPANJI);
            }
            if (__fabs(x_valspeed) < CENTER_SPEED_EDGE_YUANPANJI) {
                x_valspeed = SATU(x_valspeed, CENTER_SPEED_EDGE_YUANPANJI);
            }

            Motor_setspeed(y_valspeed, x_valspeed, 0);
        }

        // 退出条件
        if ( __fabs(x_average) <= 1 && __fabs(y_average) <= 1) {
            Motor_setspeed(0, 0, 0);
            break;
        }
    }
    Z_SetHeight(0);
}

//定位圆的中心,color为定位的圆的圆心，用速度模式来定位
void Circle_Position_Center_SPEED(char color)//mode为Preliminary_round(初赛) or Final_round(决赛)
{	
	float x_average=Get_X_Change(),y_average=Get_Y_Change();
	float x_valspeed,y_valspeed;
	if(color==7)//upgreen or ALL_COLOR
	{
		Set_Circle_Center(114,123);
	}
	else if(color==GREEN_CIRCLE)
	{
		Set_Circle_Center(113,123);
	}
	send_NX(color);

	//   M8010_SetAngle(PUT_AND_CATCH_ANGLE);
	//   Y_SetLength(50);//must step
	//   Z_SetHeight(100);
	Wait_other_task_finish(Get_action_task_notify()); // 等待动作组完成
	Delay_ms(50);//等待摄像头稳定下来
	while(1)
	{
		x_average=change_x;
		y_average=change_y;
		if((x_average!=0xFF)&&(y_average!=0xFF))
		{
			//进行PID计算
			y_valspeed=PID_Compute(&Pid_Circle_Positioning,y_average,0);
			x_valspeed=PID_Compute(&Pid_Circle_Positioning,x_average,0);
			//对输出值进行筛选
			if(__fabs(y_valspeed)<CENTER_SPEED_EDGE)
			{
				y_valspeed=SATU(y_valspeed,1.0f);
			}
			if(__fabs(x_valspeed)<CENTER_SPEED_EDGE)
			{
				x_valspeed=SATU(x_valspeed,1.0f);
			}
			//最后输出
			//Delay_ms(10);
			Delay_ms(10);
			Motor_setspeed(x_valspeed,y_valspeed,0);//进行PID计算
		}
		if((__fabs(x_average)==0)&&(__fabs(y_average)==0))
		{
			for (char i = 0; i < 4; i++)
			{
				Delay_ms(5);
				Motor_setspeed(0,0,0);
			}
			break;
		}
	}
  Z_SetHeight(40);
}


//定位圆的中心,color为定位的圆的圆心，用速度模式来定位
void Circle_Position_Center_SPEED_with_w(char aimcolor,char fuzucolor,char mode)//mode为Preliminary_round(初赛) or Final_round(决赛)
{	
	uint8_t now_color;
	float x_valspeed,y_valspeed,w_valspeed,K;
	int two_color_center[2];
	int aim_color_center [2];
	char flag=0,flag1=0;
	Set_Circle_Center(117,123);
	claw_move(open);
	M8010_SetAngle(PUT_AND_CATCH_ANGLE);
	Y_SetLength(40);//must step
	switch(mode)
	{ 
		case Preliminary_round:
		{
			if(aimcolor!=ALL_COLOR)	
	    	{
				Z_SetHeight(100);
			}
			else if(aimcolor==ALL_COLOR)
			{
				Z_SetHeight(0);
			}
		}break;
		case Final_round:
		{
			if(aimcolor==ALL_COLOR)
			{
		        Set_Circle_Center(119,120);
			}
			Z_SetHeight(0);
		}break;
	}
	while(1)
	{
		switch (flag)
		{
			case 0 ://检测辅助圆
			{
				send_NX(fuzucolor);
				Delay_ms(4);
				now_color=Get_data_action_flag();
				if(now_color==fuzucolor)
				{
					two_color_center[0]=Get_X_Change();
					two_color_center[1]=Get_Y_Change();
					if (two_color_center[0]!=0xFF && two_color_center[1]!=0xFF)
					{
						flag=1;
					}	
				}
			}break;
			case 1 ://检测目标圆
			{
				send_NX(aimcolor);
				Delay_ms(4);
				now_color=Get_data_action_flag();
				if(now_color==aimcolor)
				{
					aim_color_center[0]=Get_X_Change();
					aim_color_center[1]=Get_Y_Change();
					if (aim_color_center[0]!=0xFF && aim_color_center[1]!=0xFF)
					{
						flag=2;
					}	
				}
			}break;
			case 2 ://计算PID
			{
				K = (float)((float)(aim_color_center[1]-two_color_center[1])/
				            (float)(aim_color_center[0]-two_color_center[0])-0.001f);
				w_valspeed = K *60; // 10为比例系数
				//进行PID计算
				y_valspeed=PID_Compute(&Pid_Circle_Positioning,aim_color_center[1],0);
				x_valspeed=PID_Compute(&Pid_Circle_Positioning,aim_color_center[0],0);
				//对输出值进行筛选
				if(__fabs(y_valspeed)<CENTER_SPEED_EDGE)
				{
					y_valspeed=SATU(y_valspeed,1);
				}
				if(__fabs(x_valspeed)<CENTER_SPEED_EDGE)
				{
					x_valspeed=SATU(x_valspeed,1);
				}
				if(__fabs(w_valspeed)<CENTER_SPEED_EDGE)
				{
					w_valspeed=SATU(w_valspeed,1);
				}
				else if(__fabs(w_valspeed)>=10)
				{
					if (w_valspeed>0)
					{
						w_valspeed=10;
					}
					else
					{
						w_valspeed=-10;
					}
				}
				//最后输出
				Motor_setspeed(x_valspeed,y_valspeed,w_valspeed);//进行PID计算
				Delay_ms(4);
				// if(w_valspeed==0)
				// {
				// 	flag=1;//证明已校正不需要检测辅助圆
				// }
				// else
				// {
				// 	flag=0;//证明未校正继续检测辅助圆
				// }
				flag=0;
				if((__fabs(aim_color_center[0])+(__fabs(aim_color_center[1]))<=1)&&__fabs(K)<=0.002)
				{
					flag1=1;
					for(char i=0;i<3;i++)
					{
						Delay_ms(6);
						Motor_setspeed(0,0,0);
					}
					break;
				}
			}break;
			default:
				break;
		}
		if (flag1==1)
		{
			break;
		}
		
		Delay_ms(5);
	}
  Z_SetHeight(0);
}

//预测决赛（定位圆盘机）
void Positioning_yuanpanji(char color)
{
	float x_average=Get_X_Change(),y_average=Get_Y_Change();
	float x_valspeed,y_valspeed;
	send_NX(color);
	//M8010_SetAngle(PUT_AND_CATCH_ANGLE);
	Wait_other_task_finish(Get_action_task_notify()); // 等待动作组完成
	if(color==ALL_COLOR)
	{
		Set_Circle_Center(119,124);
	}
	Motor_SetZero();
	while(1)
	{
		motor_read_coordination_all();
		Motor_Action_Calculate_actual(&car.actual_y, &car.actual_x, &car.actual_w);
		x_average=change_x;
		y_average=change_y;
		if((x_average!=0xFF)&&(y_average!=0xFF))
		{
			//进行PID计算
			y_valspeed=PID_Compute(&Pid_Circle_Positioning,y_average,0);
			x_valspeed=PID_Compute(&Pid_Circle_Positioning,x_average,0);
			//对输出值进行筛选
			if(__fabs(y_valspeed)<CENTER_SPEED_EDGE)
			{
				y_valspeed=SATU(y_valspeed,1.0f);
			}
			if(__fabs(x_valspeed)<CENTER_SPEED_EDGE)
			{
				x_valspeed=SATU(x_valspeed,1.0f);
			}
			//最后输出
			Delay_ms(10);
			Motor_setspeed(x_valspeed,y_valspeed,0);//进行PID计算
		}
		if(__fabs(car.actual_y)>100||__fabs(car.actual_x)>100)
		{
			x_valspeed = 0;
			y_valspeed = 0;
		}
		if((__fabs(x_average)==0)&&(__fabs(y_average)==0))
		{
			for (char i = 0; i < 4; i++)
			{
				Delay_ms(5);
				Motor_setspeed(0,0,0);
			}
			break;
		}
	}
}
//定位地面的物料（预测决赛）
void Positioning_grand(char color)
{
	float x_average=Get_X_Change(),y_average=Get_Y_Change();
	float x_valspeed,y_valspeed;
	send_NX(color);
	M8010_SetAngle(PUT_AND_CATCH_ANGLE);
	Y_SetLength(60);
	Z_SetHeight(130);
	//Wait_other_task_finish(Get_action_task_notify()); // 等待动作组完成
	if(color==ALL_COLOR)
	{
		Set_Circle_Center(119,124);
	}
	while(1)
	{
		x_average=change_x;
		y_average=change_y;
		if((x_average!=0xFF)&&(y_average!=0xFF))
		{
			//进行PID计算
			y_valspeed=PID_Compute(&Pid_Circle_Positioning,y_average,0);
			x_valspeed=PID_Compute(&Pid_Circle_Positioning,x_average,0);
			//对输出值进行筛选
			if(__fabs(y_valspeed)<CENTER_SPEED_EDGE)
			{
				y_valspeed=SATU(y_valspeed,1.0f);
			}
			if(__fabs(x_valspeed)<CENTER_SPEED_EDGE)
			{
				x_valspeed=SATU(x_valspeed,1.0f);
			}
			//最后输出
			Delay_ms(10);
			Motor_setspeed(x_valspeed,y_valspeed,0);//进行PID计算
		}
		if((__fabs(x_average)==0)&&(__fabs(y_average)==0))
		{
			for (char i = 0; i < 4; i++)
			{
				Delay_ms(5);
				Motor_setspeed(0,0,0);
			}
			break;
		}
	}
}
