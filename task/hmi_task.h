#ifndef __HMI_TASK_H__
#define __HMI_TASK_H__

#include <stdint.h>

extern volatile uint32_t hmi_debug_drop_count;

void HMI_Task_Create(void);
void HMI_InitScreen(void);
void HMI_SetQR(int first, int second);
void HMI_SetSys(char* mode, char* err);
void HMI_SetVisionText(char* text);
void HMI_SetChassisText(char* text);
void HMI_SetArmText(char* text);
void HMI_SetArmFault(char* text);
void HMI_ClearArmFault(void);
void HMI_SetPixelError(int err_x, int err_y, uint8_t valid);
void HMI_SetMotorCompare(int32_t actual_x, int32_t actual_y,
						 int32_t error_x, int32_t error_y, uint8_t valid);
void HMI_LogInfo(char* fmt, ...);
void HMI_LogWarn(char* fmt, ...);
void HMI_LogError(char* fmt, ...);
void TJC_PrintChar(char ch);

#endif
