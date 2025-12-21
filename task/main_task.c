#include "main_task.h"
#include "tjc_usart_hmi.h"
#include "warehouse_app.h"
#include "action_sets.h"

/*坐标
  y
  |
  |__ __x
  开关
*/

TaskHandle_t main_task_Handle;

#define MAIN_TASK_STACK 512//任务栈
#define MAIN_TASK_PRIORITY 5//任务优先级

void Wait_other_task_finish(uint32_t tar_TaskNotify)
{
	xTaskNotifyWait(0, tar_TaskNotify, NULL, portMAX_DELAY); // 等待其他任务完成
	vTaskDelay(pdMS_TO_TICKS(1));               // 让出CPU，确保调度器切换
}

void User_function()
{	
	//Delay_ms(10);
	// one.firse=1;
	// one.second = 2;
	// one.thrid = 3;
	// two.firse=3;
	// two.second = 2;
	// two.thrid = 1;
	//vTaskDelay(pdMS_TO_TICKS(10));
	Set_chassis_able(enable);
	action_set_in_user(ACITON_INIT);//动作组的初始化
	// Move_To_Target_area(-110,110,0,enable,Relative_Position);//y方向前进80cm
	Move_To_Target_area(-120,650,0,enable,Relative_Position);//y方向前进80cm
	while(first_code == 0 && second_code == 0)//如果没扫到码
	{
		Move_To_Target_area(0,0,0,enable,Relative_Position);
	}
	Wait_other_task_finish(Get_action_task_notify());
	Init_Warehouse(1);//初始化仓库
	HMI_SEND();//屏幕显示二维码
	action_set_in_user(YUANPANJI_ACTION);//准备圆盘机的动作组
	Move_To_Target_area(0,800,0,enable,Relative_Position);//y方向前进165cm
	//
	//yuanpanji
	Set_chassis_able(unable);
	Catch_Material_YUAN_PAN_JI_with_TELESCOPIC_Polar_coordinates(1);
	Set_chassis_able(enable);
	//
	//Move_To_Target_area(-50,0,0,enable,Relative_Position);//x方向上右移50cm
	Move_To_Target_area(0,-430,0,enable,Relative_Position);//x方向上右移50cm
	Move_To_Target_area(-1810,0,0,enable,Relative_Position);//转90度
	action_set_in_user(CIRCLE_ACTION);//准备圆环的动作组
	Move_To_Target_area(0,0,180,enable,Relative_Position);

	Set_chassis_able(unable);//不使能陀螺仪和里程计
	Circle_Position_Center_SPEED(GREEN_CIRCLE);
	Put_Material_Processing_Area(1,1,P_round,cu_area);//抓取物料
	Take_Material_Processing_Area(1);
	Set_chassis_able(enable);//使能陀螺仪和里程计

	Move_To_Target_area(0,-850,0,enable,Relative_Position);//x方向上前进60cm
	Move_To_Target_area(-930,0,0,enable,Relative_Position);//转回原来角度
	action_set_in_user(CIRCLE_ACTION);//准备圆环的动作组
	Move_To_Target_area(0,0,-90,enable,Relative_Position);//转90度

	Set_chassis_able(unable);//不使能陀螺仪和里程计
	Circle_Position_Center_SPEED(GREEN_CIRCLE);
	Put_Material_Processing_Area(1,1,P_round,jing_area);//抓取物料
	Set_chassis_able(enable);//使能陀螺仪和里程计
	//第2圈
	Init_Warehouse(2);//初始化仓库
	Move_To_Target_area(0,-950,0,enable,Relative_Position);//y方向前进80cm
	Move_To_Target_area(-493,0,0,enable,Relative_Position);//
	action_set_in_user(YUANPANJI_ACTION);//准备圆盘机的动作组
	Move_To_Target_area(0,0,-90,enable,Relative_Position);//转-90度
	
	Set_chassis_able(unable);//不使能陀螺仪和里程计
	Catch_Material_YUAN_PAN_JI_with_TELESCOPIC_Polar_coordinates(2);
	Set_chassis_able(enable);//使能陀螺仪和里程计

	Move_To_Target_area(0,-420,0,enable,Relative_Position);//x方向上右移50cm
	Move_To_Target_area(-1780,0,0,enable,Relative_Position);//转90度
	action_set_in_user(CIRCLE_ACTION);//准备圆环的动作组
	Move_To_Target_area(0,0,180,enable,Relative_Position);

	Set_chassis_able(unable);//不使能陀螺仪和里程计
	Circle_Position_Center_SPEED(GREEN_CIRCLE);
	Put_Material_Processing_Area(2,1,P_round,cu_area);//抓取物料
	Take_Material_Processing_Area(2);
	Set_chassis_able(enable);//使能陀螺仪和里程计

	Move_To_Target_area(0,-850,0,enable,Relative_Position);//x方向上前进60cm
	Move_To_Target_area(-900,0,0,enable,Relative_Position);//转回原来角度
	action_set_in_user(CIRCLE_ACTION);//准备圆环的动作组
	Move_To_Target_area(0,0,-90,enable,Relative_Position);//转90度

	Set_chassis_able(unable);//不使能陀螺仪和里程计
	Circle_Position_Center_SPEED(7);
	Put_Material_Processing_Area(2,2,P_round,jing_area);//抓取物料
	Set_chassis_able(enable);//使能陀螺仪和里程计

	Move_To_Target_area(-1790,0,0,enable,Relative_Position);//转90度
	Move_To_Target_area(0,-1100,0,enable,Relative_Position);//转90度
	Move_To_Target_area(-160,0,0,enable,Relative_Position);//转90度
}

void text_fuction()
{
	Set_chassis_able(unable);
	Motor_setspeed(0,0,50);
//	one.firse=1;
//	 one.second = 2;
//	 one.thrid = 3;
//	 two.firse=3;
//	 two.second = 2;
//	two.thrid = 1;
//	Set_chassis_able(enable);
//	// Move_To_Target_area(-110,110,0,enable,Relative_Position);//y方向前进80cm
//	Move_To_Target_area(-120,650,0,enable,Relative_Position);//y方向前进80cm
//	Move_To_Target_area(0,800,0,enable,Relative_Position);//y方向前进165cm

//	Move_To_Target_area(0,-400,0,enable,Relative_Position);//x方向上右移50cm
//	Move_To_Target_area(-1810,0,0,enable,Relative_Position);//转90度

//	Move_To_Target_area(0,-850,0,enable,Relative_Position);//x方向上前进60cm
//	Move_To_Target_area(-930,0,0,enable,Relative_Position);//转回原来角度
//	//第2圈
//	Move_To_Target_area(0,-950,0,enable,Relative_Position);//y方向前进80cm
//	Move_To_Target_area(-493,0,0,enable,Relative_Position);//
//	
//	Move_To_Target_area(0,-420,0,enable,Relative_Position);//x方向上右移50cm
//	Move_To_Target_area(-1780,0,0,enable,Relative_Position);//转90度

//	Move_To_Target_area(0,-850,0,enable,Relative_Position);//x方向上前进60cm
//	Move_To_Target_area(-900,0,0,enable,Relative_Position);//转回原来角度

//	Move_To_Target_area(-1790,0,0,enable,Relative_Position);//转90度
//	Move_To_Target_area(0,-1100,0,enable,Relative_Position);//转90度
//	Move_To_Target_area(-160,0,0,enable,Relative_Position);//转90度
//	HMI_SEND();
	
//	Z_SetHeight(100);
//	Y_SetLength(50);
//	while(1)
//	{
//		send_NX(5);
//		Delay_ms(6);
//	}
}

void User_function_final()
{	
	vTaskDelay(pdMS_TO_TICKS(10));
	Set_chassis_able(enable);
	// Move_To_Target_area(-110,110,0,enable,Relative_Position);//y方向前进80cm
	Move_To_Target_area(-110,1400,0,enable,Relative_Position);//y方向前进80cm
	while(first_code == 0 && second_code == 0)//如果没扫到码
	{
		Move_To_Target_area(0,0,0,enable,Relative_Position);
	}
	Init_Warehouse(1);//初始化仓库
	HMI_SEND();//屏幕显示二维码

	//
	Move_To_Target_area(0,-400,0,enable,Relative_Position);//x方向上右移50cm
	Move_To_Target_area(-1810,0,0,enable,Relative_Position);//转90度
	//action_set_in_user(CIRCLE_ACTION);//准备圆环的动作组
	Move_To_Target_area(0,0,180,enable,Relative_Position);

	Set_chassis_able(unable);//不使能陀螺仪和里程计
	//Circle_Position_Center_SPEED(GREEN_CIRCLE,P_round);这一行替代成识别物料
	//Take_Material_Processing_Area(1);//抓决赛物料
	Set_chassis_able(enable);//使能陀螺仪和里程计

	Move_To_Target_area(-30,-850,0,enable,Relative_Position);//x方向上前进60cm
	Move_To_Target_area(-850,0,0,enable,Relative_Position);//转回原来角度
	action_set_in_user(CIRCLE_ACTION);//准备圆环的动作组
	Move_To_Target_area(0,0,-90,enable,Relative_Position);//转90度

	Set_chassis_able(unable);//不使能陀螺仪和里程计
	//Circle_Position_Center_SPEED(GREEN_CIRCLE,P_round);
	//Put_Material_Processing_Area(1,1,P_round,jing_area);//放置物料
	//Take_Material_Processing_Area(1);//抓决赛物料
	Set_chassis_able(enable);//使能陀螺仪和里程计
	
	Move_To_Target_area(-1790,0,0,enable,Relative_Position);//走到圆盘机
	Move_To_Target_area(0,0,180,enable,Relative_Position);
	//PUT_Material_YUAN_PAN_JI(1);
	
	//第2圈
	Init_Warehouse(2);//初始化仓库
	Move_To_Target_area(0,-900,0,enable,Relative_Position);
	Move_To_Target_area(-950,0,0,enable,Relative_Position);
	Move_To_Target_area(0,0,-90,enable,Relative_Position);//转-90度
	
	Set_chassis_able(unable);//不使能陀螺仪和里程计
	//Circle_Position_Center_SPEED(GREEN_CIRCLE,P_round);这一行换为识别物料
	//Take_Material_Processing_Area(1);//抓决赛物料
	Set_chassis_able(enable);//使能陀螺仪和里程计
	
	Move_To_Target_area(-30,-850,0,enable,Relative_Position);//x方向上前进60cm
	Move_To_Target_area(-900,0,0,enable,Relative_Position);//转回原来角度
	//action_set_in_user(CIRCLE_ACTION);//准备圆环的动作组
	Move_To_Target_area(0,0,-90,enable,Relative_Position);//转90度

	Set_chassis_able(unable);//不使能陀螺仪和里程计
	//Circle_Position_Center_SPEED(GREEN_CIRCLE,P_round);
	//Put_Material_Processing_Area(1,1,P_round,jing_area);//放置物料
	//Take_Material_Processing_Area(1);//抓决赛物料
	Set_chassis_able(enable);//使能陀螺仪和里程计
	
	Move_To_Target_area(-1790,0,0,enable,Relative_Position);//走到圆盘机
	Move_To_Target_area(0,0,180,enable,Relative_Position);
	//PUT_Material_YUAN_PAN_JI(2);
	
	
	Move_To_Target_area(0,1100,0,enable,Relative_Position);//转90度
	Move_To_Target_area(70,0,0,enable,Relative_Position);//转90度	
}

void Main_Task(void *pvParameters)
{

    
	text_fuction();
	//User_function_final();
	//User_function();

	
}

void Main_Task_create(void)
{
	xTaskCreate(Main_Task,"Main_Task",MAIN_TASK_STACK,NULL,MAIN_TASK_PRIORITY,&main_task_Handle);
}

