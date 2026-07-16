#include "start_task.h"
#include "imu_task.h"
#include "hmi_task.h"

TaskHandle_t Start_Task_Handle;

#define START_TASK_STACK 256//任务栈
#define START_TASK_PRIORITY 5//���ȼ�

void Init_Task_Create(void)
{
    ///////初始化////////
	Delay_Init();
	QR_sense_init();		// 扫码模块初始化
	HMI_InitScreen();		// 串口屏内容初始化
	MOTOR_Init();
	Gyro_Init();
	POSTION_init();			// 机械臂Z轴升降Y轴伸缩初始化
	Vision_Receive_Init();	// 初始化 MaixCam 串口接收
	M8010_init();
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	Telescopic_Init();		// 伸缩臂PID初始化
    //////初始化完成//////
    Delay_ms(2);
	read_init_postion();
}

void Start_Task(void*pvParameters)
{
	HMI_Task_Create();
	Init_Task_Create();
	IMU_Task_Create();          // IMU 5ms 解析任务
	Main_Task_create();
	vTaskDelete(Start_Task_Handle);
}

void Start_Task_Create(void)
{
	xTaskCreate(Start_Task,"Start_Task",START_TASK_STACK,NULL,START_TASK_PRIORITY,&Start_Task_Handle);
}
