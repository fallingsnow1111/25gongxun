#ifndef __ACTION_CONTROL_H
#define __ACTION_CONTROL_H

#include "main.h"

void action_set_in_user(ACTION_TOTAL_SET car_tar_action);
uint8_t action_set_in_task(ACTION_TOTAL_SET car_tar_action);
void Yuanpanji_PrepareDetectPose(void);
void Circle_PrepareDetectPose(void);
void Circle_PrepareMaterialCatchPose(void);
uint8_t Circle_PlaceFromWarehouseAtHeight(uint8_t warehouse_index, uint16_t place_height);

#endif
