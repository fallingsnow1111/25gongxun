/**
 * @file ris_protocol.h
 * @author Yichao Zhang (unitree@qq.com)
 * @brief Go-M8010-6 关节电机 串口通讯指令集
 * @version 0.1
 * @date 2022-03-04
 *
 * @copyright Copyright (c) unitree robotics .co.ltd. 2022
 */

#ifndef __RIS_PROTOCOL_H
#define __RIS_PROTOCOL_H

#include <stdint.h>

#pragma pack(1)

/**
 * @brief 接收模式控制信息
 *
 */
typedef struct
{
    uint8_t id     :4;      // 电机ID: 0,1...,13,14  15表示所有电机广播(此时无返回)
    uint8_t status :3;      // 运行模式: 0.待机 1.FOC闭环 2.编码器校准 3.刹车
    uint8_t none   :1;      // 保留位
} RIS_Mode_t;   // 接收模式 1Byte

/**
 * @brief 接收状态控制信息
 *
 */
typedef struct
{
    int16_t tor_des;        // 电机关节前馈扭矩   unit: N.m      (q8)
    int16_t spd_des;        // 电机关节前馈速度   unit: rad/s    (q8)
    int32_t pos_des;        // 电机关节目标位置   unit: rad      (q15)
    int16_t k_pos;          // 电机关节刚度系数   unit: -1.0-1.0 (q15)
    int16_t k_spd;          // 电机关节阻尼系数   unit: -1.0-1.0 (q15)

} RIS_Comd_t;   // 控制参数 12Byte

/**
 * @brief 接收状态反馈信息
 *
 */
typedef struct
{
    int16_t  torque;        // 实际关节前馈扭矩   unit: N.m     (q8)
    int16_t  speed;         // 实际关节前馈速度   unit: rad/s   (q8)
    int32_t  pos;           // 实际关节目标位置   unit: rad     (q15)
    int8_t   temp;          // 电机温度: -128~127°C
    uint8_t  MError :3;     // 电机错误标识: 0.正常 1.过热 2.过流 3.过压 4.磁编码异常 5-7.保留
    uint16_t force  :12;    // 足端压力传感器 12bit (0-4095)
    uint8_t  none   :1;     // 保留位
} RIS_Fbk_t;   // 状态反馈 11Byte


#pragma pack()

#endif

/*
 * Actuator Communication Reduced Instruction Set
 * Unitree robotics (c) .Co.Ltd. 2022 All Rights Reserved.
 */
