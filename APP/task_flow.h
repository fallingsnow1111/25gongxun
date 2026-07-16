#ifndef __TASK_FLOW_H
#define __TASK_FLOW_H

#include "main.h"

typedef enum {
	RING_WORK_LAYER_FIRST = 1,
	RING_WORK_LAYER_SECOND = 2
} RING_WORK_LAYER_T;

/* 可供孤立测试复用的已验证单步流程。 */
uint8_t TaskFlow_RingLocateOne(uint8_t color, char *name, float target_angle);
void TaskFlow_RingSwitchY(float vy, float target_angle, float distance_cm);
uint8_t TaskFlow_PlaceFromWarehouseIndex(uint8_t warehouse_index, char *name,
										 uint16_t place_height);

/* 比赛流程各阶段，可独立调用进行路径调试。 */
void Route_Path1_StartToQR(void);
void Flow_QRRecognize(void);
void Route_Path2_QRToTurntable(void);
void Flow_TurntableCatch(void);
void Route_Path3_TurntableToProcessing(void);
void Flow_ProcessingArea(void);
void Route_Path4_ProcessingToNext(void);
void Flow_StorageArea(RING_WORK_LAYER_T work_layer);
void Route_Path5_StorageToTurntable(void);
void Route_Path6_StorageToHome(void);
void Flow_RunCurrent(void);

extern volatile uint8_t ypj_debug_stage;
extern volatile uint8_t ypj_debug_color;
extern volatile uint8_t ypj_debug_target_valid;
extern volatile uint8_t ypj_debug_target_x;
extern volatile uint8_t ypj_debug_target_y;
extern volatile uint32_t ypj_debug_frame_count;

#endif
