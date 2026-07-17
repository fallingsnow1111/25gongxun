#ifndef __MOTOR_CONTORL_H
#define __MOTOR_CONTORL_H

#include <stdint.h>
#include "main.h" // stm32 hal
#include "ris_protocol.h"
#pragma pack(1)

#define PUT_HOUSE_ANGLE       -7   /* 旧流程放入物料仓时使用的关节角。 */
#define PUT_AND_CATCH_ANGLE -182   /* 圆盘机夹取、色环识别和放置使用的安全关节角。 */

#ifndef _LIMIT_MIN
#define _LIMIT_MIN(x, min) ((x) < (min) ? (min) : (x))
#endif

void M8010_Set_Zero(void);
void M8010_send(int position);
void M8010_init(void);
void read_init_postion(void);
void M8010_SetAngle(int tar_angle);
uint32_t M8010_ShowRealPostion(void);
float M8010_Control_pid(float differnt,float Initial_value);
void MY_UART8_IRQHandler(void);

struct ANGLE
{
	double NOW;
    double TARGE;
    double CHANGE;
};

typedef union
{
    int32_t     L;
    uint8_t     u8[4];
    uint16_t    u16[2];
    uint32_t    u32;
    float       F;
} COMData32;

typedef struct
{
    // 接收: 数据包头
    unsigned char start[2]; // 包头 [0x3E, 0x9A]
    unsigned char motorID;  // 电机ID  0,1,2,3 ...  0xBB 表示广播至所有电机(广播时无返回)
    unsigned char reserved; // 保留
} COMHead;

typedef struct
{ // 接收: 4字节一组 自然序号排列
    // 接收: 数据
    uint8_t mode;       // 当前关节模式
    uint8_t ReadBit;    // 电机控制参数修改 是否成功位
    int8_t Temp;        // 电机当前平均温度
    uint8_t MError;     // 电机错误标识

    COMData32 Read;     // 读取的当前电机的控制参数
    int16_t T;          // 当前实际电机扭矩 (7+8格式)

    int16_t W;          // 当前实际电机速度(转速) (8+7格式)
    float LW;           // 当前实际电机速度(转速)

    int16_t W2;         // 当前实际关节速度(转速) (8+7格式)
    float LW2;          // 当前实际关节速度(转速)

    int16_t Acc;        // 电机转子加速度 (15+0格式, 数值较小)
    int16_t OutAcc;     // 输出轴加速度 (12+3格式, 数值较大)

    int32_t Pos;        // 当前电机位置(零点可设), 上电默认为关节或电机编码器0为基准
    int32_t Pos2;       // 关节编码器位置(仅多圈型号)

    int16_t gyro[3];    // 预留 板载6轴传感器数据
    int16_t acc[3];

    // 预留 板载传感器数据
    int16_t Fgyro[3];
    int16_t Facc[3];
    int16_t Fmag[3];
    uint8_t Ftemp;      // 8位温度传感器  7位:-28~100度  1位:0.5度分辨率

    int16_t Force16;    // 足端压力传感器(16位精度)
    int8_t Force8;      // 足端压力传感器(8位精度)

    uint8_t FError;     // 足端传感器错误标识

    int8_t Res[1];      // 通讯 保留字节

} ServoComdV3; // 接收数据包的包头到CRC 78字节(4+70+4)

typedef struct
{
    uint8_t head[2];    // 包头         2Byte
    RIS_Mode_t mode;    // 控制模式     1Byte
    RIS_Fbk_t   fbk;    // 反馈数据结构 11Byte
    uint16_t  CRC16;    // CRC          2Byte
} MotorData_t;  // 接收数据包

typedef struct
{
    uint8_t none[8];            // 预留

} LowHzMotorCmd;

typedef struct
{                               // 发送: 4字节一组 自然序号排列
                                // 发送: 数据
    uint8_t mode;               // 关节模式选择
    uint8_t ModifyBit;          // 电机控制参数修改位
    uint8_t ReadBit;            // 电机控制参数读取位
    uint8_t reserved;           // 保留

    COMData32 Modify;           // 电机参数修改 控制参数
    // 实际给FOC的指令值为:
    // K_P*delta_Pos + K_W*delta_W + T
    int16_t T;                  // 电机关节的前馈扭矩(负载扭矩) x256 (7+8格式)
    int16_t W;                  // 电机关节速度 前馈速度 x128 (8+7格式)
    int32_t Pos;                // 电机关节位置 x16384/6.2832 (14位), 上电默认电机或关节编码器0为基准

    int16_t K_P;                // 关节刚度系数 x2048 (4+11格式)
    int16_t K_W;                // 关节速度系数 x1024 (5+10格式)

    uint8_t LowHzMotorCmdIndex; // 预留
    uint8_t LowHzMotorCmdByte;  // 预留

    COMData32 Res[1];           // 通讯 保留字节 可用于实现一些通讯需求

} MasterComdV3; // 发送数据包的包头到CRC 34字节

typedef struct
{
    // 发送: 控制数据包
    uint8_t head[2];    // 包头         2Byte
    RIS_Mode_t mode;    // 控制模式     1Byte
    RIS_Comd_t comd;    // 控制数据结构 12Byte
    uint16_t   CRC16;   // CRC          2Byte
} ControlData_t;     // 发送控制数据包

#pragma pack()

typedef struct
{
    // 发送格式结构体
    ControlData_t motor_send_data;   // 发送控制数据结构体
    int hex_len;                        // 发送的16进制数据数组长度, 34
    long long send_time;                // 发送该数据的时间, 微秒(us)
    // 需要发送的控制参数
    unsigned short id;                  // 电机ID 0表示全部电机
    unsigned short mode;                // 0:待机, 5:阻尼转动, 10:闭环FOC控制
    // 实际给FOC的指令值为:
    // K_P*delta_Pos + K_W*delta_W + T
    float T;                            // 电机关节的前馈扭矩(负载扭矩) (Nm)
    float W;                            // 电机关节速度 前馈速度 (rad/s)
    float Pos;                          // 电机关节位置 (rad)
    float K_P;                          // 关节刚度系数
    float K_W;                          // 关节速度系数
    COMData32 Res;                    // 通讯 保留字节 可用于实现一些通讯需求
} MOTOR_send;

typedef struct
{
    // 接收数据结构体
    MotorData_t motor_recv_data;    // 接收数据解析结构体, 见motor_msg.h
    int hex_len;                        // 接收的16进制数据数组长度, 78
    long long resv_time;                // 接收该数据的时间, 微秒(us)
    int correct;                        // 接收数据是否正确 1:正确 0:错误
    // 解析出的电机数据
    unsigned char motor_id;             // 电机ID
    unsigned char mode;                 // 0:待机, 5:阻尼转动, 10:闭环FOC控制
    int Temp;                           // 温度
    unsigned char MError;               // 电机错误
    float T;                            // 当前实际电机扭矩
			float W;														// speed 当前实际电机速度
    float Pos;                          // 当前电机位置(零点可设), 上电默认电机或关节编码器0为基准
			float footForce;												// 足端压力传感器 12bit (0-4095)

} MOTOR_recv;



uint32_t crc32_core(uint32_t *ptr, uint32_t len);
int modify_data(MOTOR_send *motor_s);
int extract_data(MOTOR_recv *motor_r);
HAL_StatusTypeDef SERVO_Send_recv(MOTOR_send *pData, MOTOR_recv *rData);




#endif
