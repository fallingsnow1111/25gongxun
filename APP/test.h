#ifndef __TEST_H
#define __TEST_H

#include "main.h"

typedef enum {
	RING_WORK_LAYER_FIRST = 1,
	RING_WORK_LAYER_SECOND = 2
} RING_WORK_LAYER_T;

void QR_Code_Test(void);
void Vision_Parse_Test(void);
void Chassis_Odom_Calibration_Test(void);
void Chassis_Turn_Error_Test(void);
void Ring_Warehouse_Clearance_Test(void);
void Ring_Location_Test(void);

void Yuanpanji_OpenLoop_Catch_Test(void);
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
