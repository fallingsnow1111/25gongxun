#include "imu_task.h"
#include "imu_control.h"
TaskHandle_t imu_task_Handle;

#define IMU_TASK_STACK 256//任务栈
#define IMU_TASK_PRIORITY 7//任务优先级
extern volatile float imu_w_c_output; // 角速度输出

void Imu_Task(void *pvParameters)
{
	while(1){
		//imu_w_c_output=Direction_Calibration_turn(car.target_w);
		//vofa_printf("imu.yaw:%4f\n",imu.yaw);
		vTaskDelay(1000);
	}
}

void Imu_task_create(void)
{
	xTaskCreate(Imu_Task,"Imu_task",IMU_TASK_STACK,NULL,IMU_TASK_PRIORITY,&imu_task_Handle);
}
