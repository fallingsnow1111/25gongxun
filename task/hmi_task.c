#include "hmi_task.h"
#include "tjc_usart_hmi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "motor.h"
#include "motor_control.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#define HMI_TASK_STACK       384
#define HMI_TASK_PRIORITY    4
#define HMI_QUEUE_LEN        32
#define HMI_TEXT_MAX         40
#define HMI_LOG_LINE_COUNT   3
#define HMI_STATUS_WAIT_MS   20
#define HMI_FIXED_REFRESH_MS 100

typedef enum
{
    HMI_MSG_TEXT = 0,
    HMI_MSG_LOG
} HMI_MSG_TYPE;

typedef struct
{
    HMI_MSG_TYPE type;
    char obj[3];
    char text[HMI_TEXT_MAX + 1];
} HMI_MSG;

TaskHandle_t hmi_task_Handle;
volatile uint32_t hmi_debug_drop_count = 0;

static QueueHandle_t hmi_queue = NULL;
static char tjc_print_line[HMI_TEXT_MAX + 1];
static uint8_t tjc_print_len = 0;
static volatile uint8_t hmi_qr_locked = 0;
static char hmi_qr_text[HMI_TEXT_MAX + 1];
static char hmi_log_lines[HMI_LOG_LINE_COUNT][HMI_TEXT_MAX + 1];
static char hmi_log_objs[HMI_LOG_LINE_COUNT][3] = {"t6", "t7", "t8"};
static volatile int hmi_pixel_error_x;
static volatile int hmi_pixel_error_y;
static volatile uint8_t hmi_pixel_valid;
static volatile int32_t hmi_actual_x;
static volatile int32_t hmi_actual_y;
static volatile int32_t hmi_actual_error_x;
static volatile int32_t hmi_actual_error_y;
static volatile uint8_t hmi_actual_valid;

static void HMI_CopyString(char* dst, uint8_t dst_size, const char* src)
{
    uint8_t i = 0;

    if(dst_size == 0) {
        return;
    }

    if(src == NULL) {
        dst[0] = '\0';
        return;
    }

    while(src[i] != '\0' && i < (dst_size - 1)) {
        dst[i] = src[i];
        i++;
    }
    dst[i] = '\0';
}

static void HMI_SendTextNow(char* obj, char* text)
{
    tjc_send_txt(obj, "txt", text);
}

static void HMI_LogRefreshNow(void)
{
    uint8_t i;

    for(i = 0; i < HMI_LOG_LINE_COUNT; i++) {
        HMI_SendTextNow(hmi_log_objs[i], hmi_log_lines[i]);
    }
}

static void HMI_LogPushNow(char* line)
{
    uint8_t i;

    for(i = 0; i < (HMI_LOG_LINE_COUNT - 1); i++) {
        strcpy(hmi_log_lines[i], hmi_log_lines[i + 1]);
    }

    HMI_CopyString(hmi_log_lines[HMI_LOG_LINE_COUNT - 1], sizeof(hmi_log_lines[0]), line);
    HMI_LogRefreshNow();
}

static void HMI_FixedRefreshNow(void)
{
	CHASSIS_ODOM_T odom;
	char text[HMI_TEXT_MAX + 1];
	float pose_x;
	float pose_y;
	float pose_yaw;

	Chassis_OdomGetSegmentSnapshot(&odom);
	pose_x = car.actual_x;
	pose_y = car.actual_y;
	pose_yaw = car.actual_w;

	snprintf(text, sizeof(text), "POS A%.1f X%.0f Y%.0f",
			 pose_yaw, pose_x, pose_y);
	HMI_SendTextNow("t2", text);

	snprintf(text, sizeof(text), "SW X%ld Y%ld W%ld",
			 (long)odom.x, (long)odom.y, (long)odom.w);
	HMI_SendTextNow("t3", text);

	if(hmi_pixel_valid)
		snprintf(text, sizeof(text), "PIX X%d Y%d",
				 hmi_pixel_error_x, hmi_pixel_error_y);
	else
		snprintf(text, sizeof(text), "PER O%lu M%lu TX%lu T%lums",
				 (unsigned long)chassis_period_overrun_count,
				 (unsigned long)chassis_period_max_ms,
				 (unsigned long)chassis_odom_tx_fail_count,
				 (unsigned long)odom.move_time_ms);
	HMI_SendTextNow("t4", text);

	if(hmi_actual_valid)
		snprintf(text, sizeof(text), "ACT X%ld Y%ld EX%ld EY%ld",
				 (long)hmi_actual_x, (long)hmi_actual_y,
				 (long)hmi_actual_error_x, (long)hmi_actual_error_y);
	else
		snprintf(text, sizeof(text), "ACT WAIT");
	HMI_SendTextNow("t5", text);
}

static uint8_t HMI_PostText(char* obj, char* text)
{
    HMI_MSG msg;

    msg.type = HMI_MSG_TEXT;
    HMI_CopyString(msg.obj, sizeof(msg.obj), obj);
    HMI_CopyString(msg.text, sizeof(msg.text), text);

    if(hmi_queue == NULL) {
        HMI_SendTextNow(msg.obj, msg.text);
        return 1;
    }

    if(xQueueSend(hmi_queue, &msg, pdMS_TO_TICKS(HMI_STATUS_WAIT_MS)) != pdPASS) {
        hmi_debug_drop_count++;
        return 0;
    }

    return 1;
}

static uint8_t HMI_PostLog(char* text)
{
    HMI_MSG msg;

    msg.type = HMI_MSG_LOG;
    msg.obj[0] = '\0';
    HMI_CopyString(msg.text, sizeof(msg.text), text);

    if(hmi_queue == NULL) {
        HMI_LogPushNow(msg.text);
        return 1;
    }

    if(xQueueSend(hmi_queue, &msg, 0) != pdPASS) {
        hmi_debug_drop_count++;
        return 0;
    }

    return 1;
}

static void HMI_FormatAndPostLog(char* level, char* fmt, va_list ap)
{
    char body[HMI_TEXT_MAX + 1];
    char line[HMI_TEXT_MAX + 1];

    vsnprintf(body, sizeof(body), fmt, ap);
    snprintf(line, sizeof(line), "[%s] %s", level, body);
    HMI_PostLog(line);
}

static void HMI_Task(void *pvParameters)
{
    HMI_MSG msg;
	uint32_t last_refresh = 0;

    (void)pvParameters;

    while(1) {
        if(xQueueReceive(hmi_queue, &msg, pdMS_TO_TICKS(20)) == pdPASS) {
            if(msg.type == HMI_MSG_LOG) {
                HMI_LogPushNow(msg.text);
            }
            else {
                if(hmi_qr_locked && strcmp(msg.obj, "t0") == 0) {
                    HMI_SendTextNow(msg.obj, hmi_qr_text);
                }
                else {
                    HMI_SendTextNow(msg.obj, msg.text);
                }
            }
        }

		if((HAL_GetTick() - last_refresh) >= HMI_FIXED_REFRESH_MS) {
			last_refresh = HAL_GetTick();
			HMI_FixedRefreshNow();
		}
    }
}

void HMI_Task_Create(void)
{
    if(hmi_queue == NULL) {
        hmi_queue = xQueueCreate(HMI_QUEUE_LEN, sizeof(HMI_MSG));
    }

    if(hmi_queue == NULL) {
        hmi_debug_drop_count++;
        return;
    }

    if(hmi_task_Handle == NULL) {
        xTaskCreate(HMI_Task, "HMI_Task", HMI_TASK_STACK, NULL, HMI_TASK_PRIORITY, &hmi_task_Handle);
    }
}

void HMI_InitScreen(void)
{
    if(hmi_qr_locked == 0) {
        HMI_PostText("t0", "000+000");
    }
    HMI_PostText("t1", "---chassis monitor---");
    HMI_PostText("t2", "POS A0 X0 Y0");
    HMI_PostText("t3", "SW X0 Y0 W0");
    HMI_PostText("t4", "PER O0 M0 TX0 T0ms");
    HMI_PostText("t5", "ACT WAIT");
    HMI_PostText("t6", "");
    HMI_PostText("t7", "");
    HMI_PostText("t8", "");
}

void HMI_SetQR(int first, int second)
{
    char str[HMI_TEXT_MAX + 1];

	if(hmi_qr_locked || (first == 0 && second == 0)) {
		return;
	}

    snprintf(str, sizeof(str), "%03d+%03d", first, second);
    HMI_CopyString(hmi_qr_text, sizeof(hmi_qr_text), str);
    hmi_qr_locked = 1;
    HMI_PostText("t0", hmi_qr_text);
}

void HMI_SetSys(char* mode, char* err)
{
	(void)mode;
	(void)err;
}

void HMI_SetVisionText(char* text)
{
	(void)text;
}

void HMI_SetChassisText(char* text)
{
	(void)text;
}

void HMI_SetArmText(char* text)
{
	(void)text;
}

void HMI_SetPixelError(int err_x, int err_y, uint8_t valid)
{
	hmi_pixel_error_x = err_x;
	hmi_pixel_error_y = err_y;
	hmi_pixel_valid = valid;
}

void HMI_SetMotorCompare(int32_t actual_x, int32_t actual_y,
						 int32_t error_x, int32_t error_y, uint8_t valid)
{
	hmi_actual_x = actual_x;
	hmi_actual_y = actual_y;
	hmi_actual_error_x = error_x;
	hmi_actual_error_y = error_y;
	hmi_actual_valid = valid;
}

void HMI_LogInfo(char* fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    HMI_FormatAndPostLog("I", fmt, ap);
    va_end(ap);
}

void HMI_LogWarn(char* fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    HMI_FormatAndPostLog("W", fmt, ap);
    va_end(ap);
}

void HMI_LogError(char* fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    HMI_FormatAndPostLog("E", fmt, ap);
    va_end(ap);
}

void TJC_PrintChar(char ch)
{
    if(ch == '\r') {
        return;
    }

    if(ch == '\n') {
        tjc_print_line[tjc_print_len] = '\0';
        HMI_PostLog(tjc_print_line);
        tjc_print_len = 0;
        return;
    }

    if(tjc_print_len < HMI_TEXT_MAX) {
        tjc_print_line[tjc_print_len++] = ch;
    }
    else {
        tjc_print_line[tjc_print_len] = '\0';
        HMI_PostLog(tjc_print_line);
        tjc_print_line[0] = ch;
        tjc_print_len = 1;
    }
}
