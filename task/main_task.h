#ifndef __MAIN_TASK_H
#define __MAIN_TASK_H

#include "main.h"
#include "cmsis_os.h"
#include "start_task.h"
#include "catch.h"
#include "action_control.h"

void Main_Task_create(void);
void Wait_other_task_finish(uint32_t tar_TaskNotify);

#endif
