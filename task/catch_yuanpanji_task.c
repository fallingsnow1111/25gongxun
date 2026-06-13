#include "catch_yuanpanji_task.h"
#include "catch.h"

TaskHandle_t catch_yuanpanji_task_Handle;

#define CATCH_YUANPANJI_TASK_STACK 256//任务栈
#define CATCH_YUANPANJI_TASK_PRIORITY 7//任务优先级

static uint8_t color;
static int M8010_angle;

void Set_catch_yuanpanji_able(ABLE_T a)
{
    if (catch_yuanpanji_task_Handle == NULL) return;
	taskENTER_CRITICAL(); // 进入临界区（关闭中断）
	if (a == enable) {
        vTaskResume(catch_yuanpanji_task_Handle);
    }
    else if (a == unable) {
        vTaskSuspend(catch_yuanpanji_task_Handle);
    }
	taskEXIT_CRITICAL(); // 退出临界区
    
}

void catch_positioning_from_yuanpanji(uint8_t _color,int _M8010_angle)
{
    color=_color;
    M8010_angle=_M8010_angle;
    Set_catch_yuanpanji_able(enable);
}

void Catch_yuanpanji_Task(void *pvParameters)
{
	while(1){
        Catch_TELESCOPIC_Polar_coordinates(color,M8010_angle);
		vTaskDelay(100);
	}
}

void Catch_yuanpanji_Task_create(void)
{
	xTaskCreate(Catch_yuanpanji_Task,"Catch_yuanpanji_Task",CATCH_YUANPANJI_TASK_STACK,NULL,CATCH_YUANPANJI_TASK_PRIORITY,&catch_yuanpanji_task_Handle);
    Set_catch_yuanpanji_able(unable);
}


