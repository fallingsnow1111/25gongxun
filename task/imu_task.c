#include "imu_task.h"
#include "IMU.h"

#define IMU_TASK_STACK     128
#define IMU_TASK_PRIORITY  7    // 高于底盘(6), 低于关键ISR

static TaskHandle_t IMU_Task_Handle;

static void IMU_Task(void *pvParameters)
{
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(5);

    while (1)
    {
        vTaskDelayUntil(&last_wake, period);
        IMU_Process();
    }
}

void IMU_Task_Create(void)
{
    xTaskCreate(IMU_Task, "IMU_Task", IMU_TASK_STACK,
                NULL, IMU_TASK_PRIORITY, &IMU_Task_Handle);
}
