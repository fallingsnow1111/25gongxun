#include "start_task.h"

TaskHandle_t Start_Task_Handle;

#define START_TASK_STACK 128//�����ջ
#define START_TASK_PRIORITY 5//���ȼ�



void Start_Task(void*pvParameters)
{
	Init_Task_Create();
	//Imu_task_create();
	Main_Task_create();
	Chassis_Control_Task_Create();
	//Catch_yuanpanji_Task_create();
	vTaskDelete(Start_Task_Handle);
}

void Start_Task_Create(void)
{
	xTaskCreate(Start_Task,"Start_Task",START_TASK_STACK,NULL,START_TASK_PRIORITY,&Start_Task_Handle);
}
