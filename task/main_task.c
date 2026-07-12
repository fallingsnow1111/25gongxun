#include "main_task.h"
#include "hmi_task.h"
#include "warehouse_app.h"
#include "action_sets.h"
#include "motor.h"
#include "motor_control.h"
#include "imu_control.h"
#include "IMU.h"
#include "postion_control.h"
#include "GO-M8010-6.h"
#include "QR_code.h"
#include "catch.h"
#include "user.h"
#include "circe.h"    // 顶部 include 区加这行
#include "test.h"
#include <math.h>
#include "pid.h"

TaskHandle_t main_task_Handle;

#define MAIN_TASK_STACK 512		//任务栈
#define MAIN_TASK_PRIORITY 5	//任务优先级

#define CHASSIS_SPEED_FINE  5.0f    // 超低速，视觉精定位
#define CHASSIS_SPEED_LOW   80.0f   // 低速，小范围移动
#define CHASSIS_SPEED_MID   140.0f  // 中速，漂移/普通过弯
#define CHASSIS_SPEED_HIGH  200.0f  // 高速，长直线

#define DRIFT_MID_ENTER_RAMP_TICKS  30
#define DRIFT_MID_ENTER_HOLD_TICKS  210
#define DRIFT_MID_TURN_END_ANGLE    (-60.0f)
#define DRIFT_MID_TURN_TICKS        100
#define DRIFT_MID_EXIT_HOLD_TICKS   210

void Wait_other_task_finish(uint32_t tar_TaskNotify)
{
	xTaskNotifyWait(0, tar_TaskNotify, NULL, portMAX_DELAY); // 等待其他任务完成
	vTaskDelay(pdMS_TO_TICKS(1));               // 让出CPU，确保调度器切换
}

// 二维码识别 + 串口屏显示测试
static void QR_Code_Test(void)
{
	uint32_t wait_count = 0;
	uint32_t scan_count = 0;
	uint32_t last_log_tick = 0;

	vTaskDelay(pdMS_TO_TICKS(500));

	HMI_InitScreen();
	HMI_SetSys("QRTEST", "NONE");
	HMI_SetVisionText("WAIT QR");
	HMI_SetChassisText("IDLE");
	HMI_SetArmText("IDLE");
	vTaskDelay(pdMS_TO_TICKS(200));
	HMI_LogInfo("qr hmi test start");
	last_log_tick = HAL_GetTick();

	while (1)
	{
		if (first_code != 0 || second_code != 0)
		{
			int code_a = first_code;
			int code_b = second_code;

			HMI_SEND();

			scan_count++;
			HMI_SetSys("SCAN", "NONE");
			HMI_SetVisionText("QR OK");
			HMI_LogInfo("scan %lu:%03d+%03d",
			            (unsigned long)scan_count,
			            code_a,
			            code_b);

			first_code = 0;
			second_code = 0;
			last_log_tick = HAL_GetTick();
		}
		else if((HAL_GetTick() - last_log_tick) >= 200)
		{
			wait_count++;
			HMI_SetSys("QRWAIT", "NONE");
			HMI_SetVisionText("WAIT QR");
			HMI_LogInfo("wait qr %lu", (unsigned long)wait_count);
			last_log_tick = HAL_GetTick();
		}

		vTaskDelay(pdMS_TO_TICKS(50));
	}
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
	USART6_readdata_SeetZero();
	Vision_StartMaterial();

	for(int i = 0; i < 3; i++)
	{
		// 识别等到在范围内
		while(1)
		{
			VISION_TARGET_T target;
			int x;
			int y;

			if(Vision_GetMaterialTarget((uint8_t)colors[i], &target) == 0)
			{
				vTaskDelay(pdMS_TO_TICKS(50));
				continue;
			}

			x = -((int)target.x - x_zhong);
			y = -((int)target.y - y_zhong);

			if(abs(x) < 15 && abs(y) < 25)
				break;

			vTaskDelay(pdMS_TO_TICKS(50));
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
	Chassis_BlendSpeedAngle(0, 0, 0, 0, CHASSIS_SPEED_MID, 0, DRIFT_MID_ENTER_RAMP_TICKS);
	Chassis_HoldSpeedAngle(0, CHASSIS_SPEED_MID, 0, DRIFT_MID_ENTER_HOLD_TICKS);
	
	// Drift: keep the 0 deg path direction and leave a small yaw margin.
	Chassis_DriftStraightTurn(0, CHASSIS_SPEED_MID, 0, 0, DRIFT_MID_TURN_END_ANGLE, DRIFT_MID_TURN_TICKS);

	// After drift, switch path direction to -90 deg.
	Chassis_HoldSpeedAngle(0, CHASSIS_SPEED_MID, -90, DRIFT_MID_EXIT_HOLD_TICKS);

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
	Ring_Location_Test();

	while(1)
	{
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

void Main_Task_create(void)
{
	xTaskCreate(Main_Task,"Main_Task",MAIN_TASK_STACK,NULL,MAIN_TASK_PRIORITY,&main_task_Handle);
}

