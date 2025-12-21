#include "user.h"
#include "main_task.h"

void Put_Material_Processing_Area(char times,char layer,char mode,char area)//参数为第几次
{
	//定位完之后
	char color,temp=0;
	while(temp<3)
	{
		switch (times)
		{
			case 1:
			{
						switch (temp)
						{
							case 0:
							{
								color=one.firse;
							}break;
							case 1:
							{
								color=one.second;
							}break;
							case 2:
							{
								color=one.thrid;
							}break;
						}
			}break;
			case 2:
			{
							switch (temp)
							{
								case 0:
								{
									color=two.firse;
								}break;
								case 1:
								{
									color=two.second;
								}break;
								case 2:
								{
									color=two.thrid;
								}break;
							}
			}break;
		}
		Put_Layer(layer,color,mode);//放物料
		temp++;
		}             
	Y_SetLength(0);//将伸缩臂收回
	// if(area==jing_area && times==1)
	// {
	// 	M8010_SetAngle(0);
	// }
}

void Take_Material_Processing_Area(char times)
{

	char color,temp=0;
	while(temp<3)
	{
		switch (times)
		{
			case 1:
			{
						switch (temp)
						{
							case 0:
							{
								color=one.firse;
							}break;
							case 1:
							{
								color=one.second;
							}break;
							case 2:
							{
								color=one.thrid;
							}break;
						}
			}break;
			case 2:
			{
							switch (temp)
							{
								case 0:
								{
									color=two.firse;
								}break;
								case 1:
								{
									color=two.second;
								}break;
								case 2:
								{
									color=two.thrid;
								}break;
							}
			}break;
		}
		
		catch_half_stage(color);
		temp++;
	}	
		claw_move_1(1);
		//Delay_ms(100);
		//M8010_SetAngle(0);//旋转电机
}


int Find_Color_Position(char color_code)
{
    //send_NX(color_code);
    Delay_ms(10);
    // Default values indicating invalid position
    int x_average ;
    int y_average ;
 
    int x_change = Get_X_Change();
    int y_change = Get_Y_Change();
    // Check if sensor data is valid (non-0xFF)
    if (x_change != 0xFF && y_change != 0xFF) {
        x_average = x_change;
        y_average = y_change;
    }
    // Return 1 if valid coordinates are found
    return (x_average != 0xFF && y_average != 0xFF) ? 1 : 0;
}

void Take_Material_Processing_Area_ProMax(char times) {
    static int color_data[3];
	M8010_SetAngle(PUT_AND_CATCH_ANGLE);
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
			Y_SetLength(20);//make the paw among 85mm to arrive the center of yuanpanji
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

//预测决赛（放到圆盘机上）
void PUT_material_on_yuanpanji(char times)
{
	char color,temp=0;
	while(temp<3)
	{
		switch (times)
		{
			case 1:
			{
				switch (temp)
				{
					case 0:
					{
						color=one.firse;
					}break;
					case 1:
					{
						color=one.second;
					}break;
					case 2:
					{
						color=one.thrid;
					}break;
				}
			}break;
			case 2:
			{
				switch (temp)
				{
					case 0:
					{
						color=two.firse;
					}break;
					case 1:
					{
						color=two.second;
					}break;
					case 2:
					{
						color=two.thrid;
					}break;
				}
			}break;
		}
		put_yuan_pan_ji(color);
		temp++;
	}
}





