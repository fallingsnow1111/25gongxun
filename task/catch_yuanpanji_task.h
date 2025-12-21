#ifndef __CATCH_YUANPANJI_TASK_H
#define __CATCH_YUANPANJI_TASK_H

#include "main.h"
#include "cmsis_os.h"

void Catch_yuanpanji_Task_create(void);
void Set_catch_yuanpanji_able(ABLE_T a);
void catch_positioning_from_yuanpanji(uint8_t _color,int _M8010_angle);


#endif

