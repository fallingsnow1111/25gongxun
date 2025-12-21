#include "action_sets.h"
#include "action_control.h"

extern TaskHandle_t main_task_Handle;
TaskHandle_t action_sets_task_Handle;

#define ACTION_SETS_TASK_STACK  256//����ջ
#define ACTION_SETS_TASK_PRIORITY   5//�������ȼ�

#define ACTION_TASK_NOTIFY_FINISH_BIT (1 << 0)  // 位0表示动作组任务完成

uint32_t Get_action_task_notify(void)
{
	return ACTION_TASK_NOTIFY_FINISH_BIT; // 获取任务通知值
}

void Set_Action_task(ABLE_T a)
{
	if (action_sets_task_Handle == NULL) return;
	taskENTER_CRITICAL(); // 进入临界区（关闭中断）
	if (a == enable) {
        vTaskResume(action_sets_task_Handle);
    }
    else if (a == unable) {
        vTaskSuspend(action_sets_task_Handle);
    }
	taskEXIT_CRITICAL(); // 退出临界区
}

eTaskState Take_action_task_state(void)
{
	return eTaskGetState(action_sets_task_Handle);
}

void Action_sets_Task(void *pvParameters)
{
	while(1){
		car.action_set.finish_flag=action_set_in_task(car.action_set.set_action);
		if(car.action_set.finish_flag==finish)
		{
			taskENTER_CRITICAL();
            if (main_task_Handle != NULL) {
                BaseType_t notify_status = xTaskNotify(
                    main_task_Handle,
                    ACTION_TASK_NOTIFY_FINISH_BIT,
                    eSetBits
                );
                if (notify_status != pdPASS) {
                    //vofa_printf("Notify main task failed!\n");
                }
            }
            taskEXIT_CRITICAL();
			//vofa_printf("action_finish\n");
			TaskHandle_t temp_handle = action_sets_task_Handle;
            action_sets_task_Handle = NULL;  // 先置空
            vTaskDelete(temp_handle);        // 再删除
		}	
		//vTaskDelay(600);
	}
}
//创建的时候就先挂起
void Action_sets_task_create(void)
{
	xTaskCreate(Action_sets_Task,"Action_sets_Task",ACTION_SETS_TASK_STACK,NULL,ACTION_SETS_TASK_PRIORITY ,&action_sets_task_Handle);
}
