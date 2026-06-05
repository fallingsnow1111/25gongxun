#include "init_task.h"



void Init_Task_Create(void)
{
//	 usmart_init(108);
    ///////初始化////////
		Delay_Init();
		//HAL_TIM_Base_Start_IT(&htim6);//开始
//		QR_sense_init();
		MOTOR_Init();
    Gyro_Init();
//    POSTION_init();
//    openm_Init();
//    M8010_init();
//		Telescopic_Init();
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
    //////初始化完成//////
    Delay_ms(2);
//    read_init_postion();
//    HMI_SEND();
		chassis_control_init();
}
