#include "start_task.h"

TaskHandle_t Start_Task_Handle;

#define START_TASK_STACK 128//�����ջ
#define START_TASK_PRIORITY 5//���ȼ�

void Init_Task_Create(void)
{
//	 usmart_init(108);
    ///////初始化////////
	Delay_Init();
	//HAL_TIM_Base_Start_IT(&htim6);//开始
	QR_sense_init();
	MOTOR_Init();
    Gyro_Init();
//    POSTION_init();
//    openm_Init();
//    M8010_init();
//	 	Telescopic_Init();
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
    //////初始化完成//////
    Delay_ms(2);
//    read_init_postion();
     //HMI_SEND();
	chassis_control_init();
}


void Start_Task(void*pvParameters)
{
	Init_Task_Create();
	//底盘任务要在main_task前创建
	Chassis_Control_Task_Create();
	Main_Task_create();
	//Catch_yuanpanji_Task_create();
	vTaskDelete(Start_Task_Handle);
}

void Start_Task_Create(void)
{
	xTaskCreate(Start_Task,"Start_Task",START_TASK_STACK,NULL,START_TASK_PRIORITY,&Start_Task_Handle);
}
