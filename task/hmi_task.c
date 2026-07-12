#include "hmi_task.h"
#include "tjc_usart_hmi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#define HMI_TASK_STACK       384
#define HMI_TASK_PRIORITY    2
#define HMI_QUEUE_LEN        16
#define HMI_TEXT_MAX         40
#define HMI_LOG_LINE_COUNT   3

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
static char hmi_log_lines[HMI_LOG_LINE_COUNT][HMI_TEXT_MAX + 1];
static char hmi_log_objs[HMI_LOG_LINE_COUNT][3] = {"t6", "t7", "t8"};

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

    if(xQueueSend(hmi_queue, &msg, 0) != pdPASS) {
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

    (void)pvParameters;

    while(1) {
        if(xQueueReceive(hmi_queue, &msg, portMAX_DELAY) == pdPASS) {
            if(msg.type == HMI_MSG_LOG) {
                HMI_LogPushNow(msg.text);
            }
            else {
                HMI_SendTextNow(msg.obj, msg.text);
            }
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
    HMI_PostText("t0", "000+000");
    HMI_PostText("t1", "---sys status / logs---");
    HMI_PostText("t2", "SYS:BOOT ERR:NONE");
    HMI_PostText("t3", "VIS:----");
    HMI_PostText("t4", "CHS:----");
    HMI_PostText("t5", "ARM:----");
    HMI_PostText("t6", "");
    HMI_PostText("t7", "");
    HMI_PostText("t8", "");
}

void HMI_SetQR(int first, int second)
{
    char str[HMI_TEXT_MAX + 1];

    snprintf(str, sizeof(str), "%03d+%03d", first, second);
    HMI_PostText("t0", str);
}

void HMI_SetSys(char* mode, char* err)
{
    char str[HMI_TEXT_MAX + 1];

    snprintf(str, sizeof(str), "SYS:%s ERR:%s", mode, err);
    HMI_PostText("t2", str);
}

void HMI_SetVisionText(char* text)
{
    char str[HMI_TEXT_MAX + 1];

    snprintf(str, sizeof(str), "VIS:%s", text);
    HMI_PostText("t3", str);
}

void HMI_SetChassisText(char* text)
{
    char str[HMI_TEXT_MAX + 1];

    snprintf(str, sizeof(str), "CHS:%s", text);
    HMI_PostText("t4", str);
}

void HMI_SetArmText(char* text)
{
    char str[HMI_TEXT_MAX + 1];

    snprintf(str, sizeof(str), "ARM:%s", text);
    HMI_PostText("t5", str);
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

    HMI_SetSys("ERROR", "CHECK");
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
