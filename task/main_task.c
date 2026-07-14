#include "main_task.h"
#include "test.h"

TaskHandle_t main_task_Handle;

#define MAIN_TASK_STACK    512
#define MAIN_TASK_PRIORITY 5

void Wait_other_task_finish(uint32_t tar_TaskNotify)
{
    xTaskNotifyWait(0, tar_TaskNotify, NULL, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(1));
}

static void Main_Task(void *pvParameters)
{
    (void)pvParameters;

	Chassis_Turn_Error_Test();
    // Yuanpanji_OpenLoop_Catch_Test();

    while(1)
        vTaskDelay(pdMS_TO_TICKS(200));
} 

void Main_Task_create(void)
{
    xTaskCreate(Main_Task, "Main_Task", MAIN_TASK_STACK, NULL,
                MAIN_TASK_PRIORITY, &main_task_Handle);
}
