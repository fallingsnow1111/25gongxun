#ifndef __START_TASK_H
#define __START_TASK_H

#include "main.h"
#include "cmsis_os.h"
#include "main_task.h"
#include "imu_task.h"
#include "init_task.h"
#include "chassis_control_task.h"
#include "action_sets.h"
#include "catch_yuanpanji_task.h"

void Start_Task_Create(void);

#endif
