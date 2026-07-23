#include "main_task.h"
#include "test.h"

TaskHandle_t main_task_Handle;

#define MAIN_TASK_STACK    512
#define MAIN_TASK_PRIORITY 5

static void Main_Task(void *pvParameters)
{
    (void)pvParameters;

	Flow_RunCurrent();

	
    while(1)
        vTaskDelay(pdMS_TO_TICKS(200));
} 

void Main_Task_create(void)
{
    xTaskCreate(Main_Task, "Main_Task", MAIN_TASK_STACK, NULL,
                MAIN_TASK_PRIORITY, &main_task_Handle);
}
