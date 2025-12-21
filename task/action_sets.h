#ifndef __ACTION_SETS_H
#define __ACTION_SETS_H

#include "main.h"
#include "cmsis_os.h"


void Set_Action_task(ABLE_T a);
void Action_sets_task_create(void);
eTaskState Take_action_task_state(void);
uint32_t Get_action_task_notify(void);

#endif
