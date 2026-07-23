#ifndef __ACTION_CONTROL_H
#define __ACTION_CONTROL_H

#include "main.h"

void Yuanpanji_PrepareDetectPose(void);
void Circle_PrepareDetectPose(void);
void Circle_PrepareMaterialCatchPose(void);
uint8_t Circle_PlaceFromWarehouseAtHeight(uint8_t warehouse_index, uint16_t place_height,
                                            uint8_t need_clear, uint8_t need_cw);
uint8_t Circle_PlaceFromWarehouseAtPose(uint8_t warehouse_index, uint16_t place_height,
										int16_t place_angle, uint16_t place_length,
										uint8_t need_clear, uint8_t need_cw);

#endif
