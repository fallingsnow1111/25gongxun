#include "main_task.h"
#include "tjc_usart_hmi.h"
#include "warehouse_app.h"
#include "action_sets.h"
#include "motor.h"
#include "imu_control.h"
#include "IMU.h"
#include "postion_control.h"
#include "GO-M8010-6.h"
#include "QR_code.h"
#include <math.h>

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

static void Chassis_Test_Stop(uint32_t stop_time_ms)
{
	Motor_setspeed(0, 0, 0);
	vTaskDelay(pdMS_TO_TICKS(stop_time_ms));
}

static void Chassis_Test_Run_With_Imu(float vy, float vx, float target_angle, uint32_t run_time_ms)
{
	uint32_t start_tick = HAL_GetTick();

	Imu_setZero();
	vTaskDelay(pdMS_TO_TICKS(100));

	while ((HAL_GetTick() - start_tick) < run_time_ms)
	{
		float vw = Direction_Calibration_turn(target_angle);
		Motor_setspeed(vy, vx, vw);
		vTaskDelay(pdMS_TO_TICKS(20));
	}

	Chassis_Test_Stop(400);
}

static void Chassis_Test_Rotate_With_Imu(float target_angle, uint32_t timeout_ms)
{
	uint32_t start_tick = HAL_GetTick();

	Imu_setZero();
	vTaskDelay(pdMS_TO_TICKS(100));

	while ((HAL_GetTick() - start_tick) < timeout_ms)
	{
		float current_angle = normalize_angle(imu.yaw);
		float angle_error = getAngleZ(current_angle, target_angle);

		if (fabsf(angle_error) <= 2.0f)
		{
			break;
		}

		Motor_setspeed(0, 0, Direction_Calibration_turn(target_angle));
		vTaskDelay(pdMS_TO_TICKS(20));
	}

	Chassis_Test_Stop(500);
}

static void Claw_Test_Demo(void)
{
	M8010_SetAngle(PUT_AND_CATCH_ANGLE);
	vTaskDelay(pdMS_TO_TICKS(200));
	Y_SetLength(85);
	vTaskDelay(pdMS_TO_TICKS(400));

	Z_SetHeight(10);
	vTaskDelay(pdMS_TO_TICKS(400));
	claw_move_2(open);
	vTaskDelay(pdMS_TO_TICKS(400));

	Z_SetHeight(95);
	vTaskDelay(pdMS_TO_TICKS(700));
	claw_move_2(close);
	vTaskDelay(pdMS_TO_TICKS(700));

	Z_SetHeight(10);
	vTaskDelay(pdMS_TO_TICKS(600));
	claw_move_2(open);
	vTaskDelay(pdMS_TO_TICKS(400));

	Y_SetLength(20);
	vTaskDelay(pdMS_TO_TICKS(400));
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

// M8010 关节电机测试：循环摆动
static void M8010_Joint_Test(void)
{
	vTaskDelay(pdMS_TO_TICKS(500));

	M8010_Set_Zero();                        // 软件零点
	vTaskDelay(pdMS_TO_TICKS(500));

	// 往正方向走 90 度
	M8010_SetAngle(90);
	vTaskDelay(pdMS_TO_TICKS(1000));

	// 回零点
	M8010_SetAngle(0);
	vTaskDelay(pdMS_TO_TICKS(1000));

	// 往负方向走 90 度
	M8010_SetAngle(-90);
	vTaskDelay(pdMS_TO_TICKS(1000));

	// 回零点
	M8010_SetAngle(0);
	vTaskDelay(pdMS_TO_TICKS(1000));

	// 走 PUT_AND_CATCH_ANGLE（抓取位姿）
	M8010_SetAngle(PUT_AND_CATCH_ANGLE);
	vTaskDelay(pdMS_TO_TICKS(2000));

	// 回零点
	M8010_SetAngle(0);
}

// 二维码识别 + 串口屏显示测试
static void QR_Code_Test(void)
{
	vTaskDelay(pdMS_TO_TICKS(500));

	// 初始化 HMI 显示
	tjc_send_txt("t0", "txt", "WAIT QR...");
	vTaskDelay(pdMS_TO_TICKS(200));

	while (1)
	{
		// 检查是否扫到了二维码
		if (first_code != 0 || second_code != 0)
		{
			// 处理数据并发送到串口屏
			HMI_SEND();

			// 复位，等待下一次扫码
			first_code = 0;
			second_code = 0;
		}

		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

// 第一圈跑图测试
static void Route_Test(void)
{
	Set_chassis_able(enable);
	vTaskDelay(pdMS_TO_TICKS(500));
	
	//到达原料区
	Move_To_Target_area(140, 0, 0, enable, Relative_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(0, 1600, 0, enable, Relative_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(0, -500, 0, enable, Relative_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(0, 0, 90, enable, Relative_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	
	//到达粗加工区
	Move_To_Target_area(0, 1750, 0, enable, Relative_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(0, 0, 90, enable, Relative_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(0, -750, 0, enable, Relative_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(0, 0, -90, enable, Relative_Position);
	vTaskDelay(pdMS_TO_TICKS(200));

	//到达暂存区
	Move_To_Target_area(0, -800, 0, enable, Relative_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(0, 0, 90, enable, Relative_Position);
	vTaskDelay(pdMS_TO_TICKS(200));

	// 回家
	Move_To_Target_area(0, 1650, 0, enable, Relative_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(0, 0, 90, enable, Relative_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(0, 900, 0, enable, Relative_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(-150, 100, 0, enable, Relative_Position);
}

// 绝对模式跑图测试
static void Route_Test_ABS(void)
{
	Set_chassis_able(enable);
	vTaskDelay(pdMS_TO_TICKS(500));

	Move_To_Target_area(0, 0, 0, enable, Relative_Position);  // 归零一次建立原点
	vTaskDelay(pdMS_TO_TICKS(300));
	// 之后全部用 Absolute_Position
	
	//到达原料区
	Move_To_Target_area(140, 0, 0, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(140, 1600, 0, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(140, 1100, 0, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(140, 1100, 90, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	
	//到达粗加工区
	Move_To_Target_area(140, 2850, 90, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(140, 2850, 180, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(140, 2100, 180, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(140, 2100, 90, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));

	//到达暂存区
	Move_To_Target_area(140, 1300, 90, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(140, 1300, 180, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));

	// 回家
	Move_To_Target_area(140, 3000, 180, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(140, 3000, 270, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(140, 3900, 270, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(-75, 4050, 270, enable, Absolute_Position);
}
void Main_Task(void *pvParameters)
{
	QR_sense_init();
	Route_Test_ABS();

	while (1) {
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}

void Main_Task_create(void)
{
	xTaskCreate(Main_Task,"Main_Task",MAIN_TASK_STACK,NULL,MAIN_TASK_PRIORITY,&main_task_Handle);
}

