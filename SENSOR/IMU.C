#include "IMU.h"
#include "imu_control.h"
#include "stdio.h"
#include "sys.h"
#include "usart.h"

#define RING_BUFFER_SIZE 256
#define IMU_FRAME_SIZE   11      // HWT101 11字节标准帧

// 配置命令数组
uint8_t unlock_register[]       = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
uint8_t reset_z_axis[]          = {0xFF, 0xAA, 0x76, 0x00, 0x00};
uint8_t set_output_200Hz[]      = {0xFF, 0xAA, 0x03, 0x0B, 0x00};
uint8_t set_baudrate_115200[]   = {0xFF, 0xAA, 0x04, 0x06, 0x00};
uint8_t save_settings[]         = {0xFF, 0xAA, 0x00, 0x00, 0x00};
uint8_t restart_device[]        = {0xFF, 0xAA, 0x00, 0xFF, 0x00};

static uint8_t imu_buffer[RING_BUFFER_SIZE];
static uint16_t read_index = 0;
unsigned char mpu_flash;
struct Imu imu;

// ---- UART 发送 ----
void U2_send(unsigned char data)
{
    unsigned short Usart2_Time = 0;
    USART2->TDR = data;
    while ((USART2->ISR & 0X40) == 0)
    {
        Usart2_Time++;
        if (Usart2_Time > 65534) break;
    }
    __HAL_UART_CLEAR_OREFLAG(&huart2);
}

void uart2WriteBuf(uint8_t *buf, uint8_t len)
{
    unsigned char i;
    for (i = 0; i < len; i++)
        U2_send(buf[i]);
}

// ---- 环形 buffer 接口 ----
static uint16_t RingBuffer_GetCount(uint16_t write_idx)
{
    if (write_idx >= read_index)
        return write_idx - read_index;
    return RING_BUFFER_SIZE - read_index + write_idx;
}

static uint8_t RingBuffer_Peek(uint16_t offset)
{
    return imu_buffer[(read_index + offset) % RING_BUFFER_SIZE];
}

void IMU_Receive_Init(void)
{
    read_index = 0;

    __HAL_UART_CLEAR_OREFLAG(&huart2);
    __HAL_UART_CLEAR_IDLEFLAG(&huart2);

    if (HAL_UART_Receive_DMA(&huart2, imu_buffer, RING_BUFFER_SIZE) != HAL_OK)
        Error_Handler();

    // 仅开启错误中断；帧解析放到 IMU_Process 周期任务中
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_ERR);
}

void IMU_Process(void)
{
    uint16_t write_idx = RING_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);

    while (RingBuffer_GetCount(write_idx) >= IMU_FRAME_SIZE)
    {
        // 校验帧头 0x55
        if (RingBuffer_Peek(0) != 0x55)
        {
            read_index = (read_index + 1) % RING_BUFFER_SIZE;
            continue;
        }

        uint8_t frame_type = RingBuffer_Peek(1);

        // 仅处理角度帧(0x53)和角速度帧(0x52)
        if (frame_type != 0x53 && frame_type != 0x52)
        {
            read_index = (read_index + 1) % RING_BUFFER_SIZE;
            continue;
        }

        // 校验和 (字节0-9累加 vs 字节10)
        uint8_t sum = 0;
        for (int j = 0; j < 10; j++)
            sum += RingBuffer_Peek(j);
        if (sum != RingBuffer_Peek(10))
        {
            read_index = (read_index + 1) % RING_BUFFER_SIZE;
            continue;
        }

        // 提取Z轴数据 (字节6-7, int16, 低字节在前)
        uint8_t low  = RingBuffer_Peek(6);
        uint8_t high = RingBuffer_Peek(7);
        int16_t raw  = (int16_t)(((uint16_t)high << 8) | low);

        if (frame_type == 0x53)    // 角度帧
        {
            float new_yaw = (float)raw / 32768.0f * 180.0f;
            imu.yaw = 0.9f * new_yaw + 0.1f * imu.yaw;
            mpu_flash = ~mpu_flash;
        }
        else if (frame_type == 0x52)   // 角速度帧
        {
            imu.angular_rate = 2000.0f * raw / 32768.0f;   // deg/s
        }

        read_index = (read_index + IMU_FRAME_SIZE) % RING_BUFFER_SIZE;
    }
}

// ---- 配置命令（接口不变） ----
void Imu_setZero(void)
{
    uart2WriteBuf(reset_z_axis, 5);
    inu_run.LAST_ANGEL = 0;
    inu_run.SPEED = 0;
}

void Imu_unlock_register(void)
{
    uart2WriteBuf(unlock_register, 5);
}

void Imu_setset_baudrate_115200(void)
{
    uart2WriteBuf(set_baudrate_115200, 5);
}

void Imu_setsave_settings(void)
{
    uart2WriteBuf(save_settings, 5);
}

void Imu_set500hz(void)
{
    uart2WriteBuf(set_output_200Hz, 5);
}

// ---- USART2 中断（仅错误处理） ----
void USART2_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE))
        __HAL_UART_CLEAR_OREFLAG(&huart2);

    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_NE))
        __HAL_UART_CLEAR_NEFLAG(&huart2);

    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_FE))
        __HAL_UART_CLEAR_FEFLAG(&huart2);
}
