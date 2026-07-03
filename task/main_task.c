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
#include "catch.h"
#include "user.h"
#include "circe.h"    // 顶部 include 区加这行
#include <math.h>
#include "pid.h"

TaskHandle_t main_task_Handle;

#define MAIN_TASK_STACK 512//任务栈
#define MAIN_TASK_PRIORITY 5//任务优先级

void Wait_other_task_finish(uint32_t tar_TaskNotify)
{
	xTaskNotifyWait(0, tar_TaskNotify, NULL, portMAX_DELAY); // 等待其他任务完成
	vTaskDelay(pdMS_TO_TICKS(1));               // 让出CPU，确保调度器切换
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

// 相对位置模式跑图测试
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

// 扫码逻辑
static uint8_t Scan_QR(float base_x, float base_y)
{
	first_code = 0;
	second_code = 0;

	const float adj[][2] = {{0,0},{-40,0},{-40, 20}, {-40, -20}};
	const uint8_t n = 4;
	// 超时时间
	const uint32_t wait_ms = 1000;

	// 四次尝试
	for(uint8_t i = 0; i< n; i++)
	{
		Move_To_Target_area(base_x + adj[i][0], base_y + adj[i][1], 0, enable, Absolute_Position);
		uint32_t t = HAL_GetTick();
		while ((first_code == 0 && second_code == 0) && (HAL_GetTick() - t) < wait_ms)
		{
			vTaskDelay(pdMS_TO_TICKS(50));
		}

		if(first_code != 0 || second_code != 0)
		{
			HMI_SEND();
			vTaskDelay(pdMS_TO_TICKS(300));
			return 1;
		}
	}

	return 0;
}

// 绝对模式跑图测试
static void Route_Test_ABS(void)
{
	Set_chassis_able(enable);
	vTaskDelay(pdMS_TO_TICKS(500));

	Move_To_Target_area(0, 0, 0, enable, Relative_Position);  // 归零一次建立原点
	vTaskDelay(pdMS_TO_TICKS(300));
	// 之后全部用 Absolute_Position
	
	//扫码
	Move_To_Target_area(140, 0, 0, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));

	Scan_QR(150, 750);

	// 原料区
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
	Move_To_Target_area(140, 2050, 180, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(140, 2050, 90, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));

	//到达暂存区
	Move_To_Target_area(140, 1250, 90, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(140, 1250, 180, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));

	// 回家
	Move_To_Target_area(140, 2950, 180, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(140, 2950, 270, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(140, 3900, 270, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(-85, 4020, 270, enable, Absolute_Position);
}

// 圆盘机静态抓取测试
void yuan_pan_catch(void)
{
	const int colors[] = {one.firse, one.second, one.thrid};

	vTaskDelay(pdMS_TO_TICKS(500));
	claw_move_2(open);
	M8010_SetAngle(PUT_AND_CATCH_ANGLE);
	Y_SetLength(YUAN_PAN_LENGHT);
	vTaskDelay(pdMS_TO_TICKS(800));

	// 识别高度
	Z_SetHeight(YUAN_PAN_DETECT_HEIGHT);
	vTaskDelay(pdMS_TO_TICKS(300));

	for(int i = 0; i < 3; i++)
	{
		// 识别等到在范围内
		while(1)
		{
			USART6_readdata_SeetZero();
			send_NX(colors[i]);
			vTaskDelay(pdMS_TO_TICKS(300));

			int x = Get_X_Change();
			int y = Get_Y_Change();

			if(x != 0xFF && y != 0xFF
				&& Get_data_action_flag() == colors[i]
				&& abs(x) < 15 && abs(y) < 25)
				break;
		}


		// 抓取
		Z_SetHeight(YUAN_PAN_HEIGHT);
		claw_move_2(close);
		vTaskDelay(pdMS_TO_TICKS(100));
		Z_SetHeight(0);

		// 放仓
		Y_SetLength(Y_LENGHT_WAREHOUSE);
		uint8_t idx = Get_Warehouse_index_from_color(colors[i]);
		M8010_SetAngle(Get_Warehouse_Angle(idx));
		vTaskDelay(pdMS_TO_TICKS(100));
		Z_SetHeight(PUT_HOUSE_HEIGHT);
		vTaskDelay(pdMS_TO_TICKS(100));
		claw_move_2(open);
		vTaskDelay(pdMS_TO_TICKS(100));
		Z_SetHeight(0);
		vTaskDelay(pdMS_TO_TICKS(100));

		// 前两个物料抓完回抓取姿态
		if(i < 2)
		{
			Y_SetLength(YUAN_PAN_LENGHT);
			M8010_SetAngle(PUT_AND_CATCH_ANGLE);
			claw_move_2(open);
			vTaskDelay(pdMS_TO_TICKS(300));
			Z_SetHeight(YUAN_PAN_DETECT_HEIGHT);
		}

	}

	M8010_SetAngle(0);
	claw_move_2(open);
	vTaskDelay(pdMS_TO_TICKS(800));
}

static float Circle_Test_MinSpeed(float speed, float min_speed)
{
    if (speed > 0) return min_speed;
    if (speed < 0) return -min_speed;
    return 0;
}

int ring_x_average;
int ring_y_average;
float ring_step_x;
float ring_step_y;

static void Ring_Move_Adjust_Test(float base_x, float base_y, float base_yaw)
{
    const uint32_t timeout_ms = 15000;
    const float K_X = 1.0f;        // 每 1 像素偏差修正多少 mm，先小一点
    const float K_Y = 1.0f;
    const float MIN_STEP = 12.0f;
    const float MAX_STEP = 15.0f;  // 单次最大修正距离，避免跳太猛

    float target_x = base_x;
    float target_y = base_y;
    const float target_yaw = base_yaw;
    uint32_t start_tick;

    Set_chassis_able(enable);
    vTaskDelay(pdMS_TO_TICKS(500));

    M8010_SetAngle(PUT_AND_CATCH_ANGLE);
    Y_SetLength(30);
    Z_SetHeight(110);
    vTaskDelay(pdMS_TO_TICKS(500));

    USART6_readdata_SeetZero();
    Set_Circle_Center(122, 114);
    send_NX(GREEN_CIRCLE);
    vTaskDelay(pdMS_TO_TICKS(300));

    start_tick = HAL_GetTick();

    while ((HAL_GetTick() - start_tick) < timeout_ms)
    {
        send_NX(GREEN_CIRCLE);
        vTaskDelay(pdMS_TO_TICKS(100));

        ring_x_average = Get_X_Change();
        ring_y_average = Get_Y_Change();

        if (Get_data_action_flag() != GREEN_CIRCLE)
        {
            continue;
        }

        if (ring_x_average == 0xFF || ring_y_average == 0xFF)
        {
            continue;
        }

		if (__fabs(ring_x_average) <= 3 && __fabs(ring_y_average) <= 3)
		{
			Motor_setspeed(0, 0, 0);
			vTaskDelay(pdMS_TO_TICKS(100));

			Move_To_Target_area(target_x, target_y, target_yaw, enable, Absolute_Position);

			Motor_setspeed(0, 0, 0);
			break;
		}

        float step_x = 0.0f;
        float step_y = 0.0f;

        if (__fabs(ring_y_average) > 2) {
            step_x = -((float)ring_y_average) * K_X;
        }

        if (__fabs(ring_x_average) > 2) {
            step_y = ((float)ring_x_average) * K_Y;
        }

        if (step_x > MAX_STEP) step_x = MAX_STEP;
        if (step_x < -MAX_STEP) step_x = -MAX_STEP;
        if (step_y > MAX_STEP) step_y = MAX_STEP;
        if (step_y < -MAX_STEP) step_y = -MAX_STEP;

        if (step_x != 0.0f && __fabs(step_x) < MIN_STEP) {
            step_x = (step_x > 0) ? MIN_STEP : -MIN_STEP;
        }

        if (step_y != 0.0f && __fabs(step_y) < MIN_STEP) {
            step_y = (step_y > 0) ? MIN_STEP : -MIN_STEP;
        }

        ring_step_x = step_x;
        ring_step_y = step_y;

        target_x += step_x;
        target_y += step_y;
        Move_To_Target_area(target_x, target_y, target_yaw, enable, Absolute_Position);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    Motor_setspeed(0, 0, 0);
    Z_SetHeight(0);
    vTaskDelay(pdMS_TO_TICKS(300));
}

void Scan_Catch(void)
{
	Set_chassis_able(enable);
	vTaskDelay(pdMS_TO_TICKS(500));

	Move_To_Target_area(0,0,0,enable,Relative_Position);
	Scan_QR(150, 750);
	Init_Warehouse(1);
	HMI_SEND();

	Move_To_Target_area(122, 1450, 0, enable, Absolute_Position);

	yuan_pan_catch();

	vTaskDelay(pdMS_TO_TICKS(500));

	Move_To_Target_area(140, 1100, 0, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(140, 1080, 90, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(140, 2850, 90, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(140, 2850, 180, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));

	vTaskDelay(pdMS_TO_TICKS(200));
}

void first_round(void)
{
	Scan_Catch();

	Ring_Move_Adjust_Test(140.0f, 2850.0f, 180.0f);
	vTaskDelay(pdMS_TO_TICKS(100));

	// Warehouse -> processing area.
	Put_Material_Processing_Area(1, 1, P_round, cu_area);
	vTaskDelay(pdMS_TO_TICKS(300));

	// Processing area -> warehouse.
	Take_Material_Processing_Area(1);
	vTaskDelay(pdMS_TO_TICKS(300));

	// Raise arm before moving.
	Z_SetHeight(0);
	Y_SetLength(0);
	vTaskDelay(pdMS_TO_TICKS(300));

	Move_To_Target_area(140, 2050, 180, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(140, 2050, 90, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(140, 1250, 90, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));

	Ring_Move_Adjust_Test(140.0f, 1250.0f, 90.0f);
	vTaskDelay(pdMS_TO_TICKS(100));

	// Warehouse -> temporary storage, first layer.
	Put_Material_Processing_Area(1, 1, P_round, cu_area);

	Z_SetHeight(0);
	Y_SetLength(0);
	claw_move_2(open);
	vTaskDelay(pdMS_TO_TICKS(300));

	Move_To_Target_area(140, 1250, 180, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(140, 2950, 180, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(140, 2950, 270, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(140, 3900, 270, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
	Move_To_Target_area(-85, 4020, 270, enable, Absolute_Position);
	vTaskDelay(pdMS_TO_TICKS(200));
}

static void OpenLoop_Chassis_Test(void)
{
    Set_chassis_able(unable);

    Motor_setspeed(0, 0, 0);
    Delay_ms(100);

    Imu_setZero();
    Delay_ms(200);

	// 入弯直线
	Chassis_BlendSpeedAngle(0, 0, 0, 0, 140, 0, 30);
	Chassis_HoldSpeedAngle(0, 140, 0, 210);

	// 右转拐角：不要在拐角内直接打满 -90，先只转到 -75
	Chassis_BlendSpeedAngle(0, 140, 0, 60, 45, -75, 90);
	Chassis_HoldSpeedAngle(60, 45, -75, 10);

	// 出弯：先保持左避让，同时把角度补到 -90
	Chassis_BlendSpeedAngle(60, 45, -75, 35, 100, -90, 70);

	// 完全出弯后再正常前进
	Chassis_BlendSpeedAngle(35, 100, -90, 0, 140, -90, 50);
	Chassis_HoldSpeedAngle(0, 140, -90, 210);

    Motor_setspeed(0, 0, 0);

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void Arm_Catch_Action_Test(void)
{
	Set_chassis_able(unable);
	Motor_setspeed(0, 0, 0);
	vTaskDelay(pdMS_TO_TICKS(500));

	claw_move_2(open);
	vTaskDelay(pdMS_TO_TICKS(300));

	M8010_SetAngle(PUT_AND_CATCH_ANGLE);
	vTaskDelay(pdMS_TO_TICKS(300));

	Y_SetLength(30);
	vTaskDelay(pdMS_TO_TICKS(300));

	Z_SetHeight(60);
	vTaskDelay(pdMS_TO_TICKS(300));

	claw_move_2(close);
	vTaskDelay(pdMS_TO_TICKS(300));

	Z_SetHeight(0);
	vTaskDelay(pdMS_TO_TICKS(300));

	Y_SetLength(0);
	vTaskDelay(pdMS_TO_TICKS(300));

	M8010_SetAngle(0);

	while (1)
	{
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

void Main_Task(void *pvParameters)
{
	first_round();

	while (1)
	{
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

void Main_Task_create(void)
{
	xTaskCreate(Main_Task,"Main_Task",MAIN_TASK_STACK,NULL,MAIN_TASK_PRIORITY,&main_task_Handle);
}

