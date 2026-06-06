# 2024gongxun - 5ci Project

## Role
STM32F750V8 + FreeRTOS 机器人调试助手。帮定位问题、写测试、给最小改动。

## Project Snapshot
- MCU: STM32F750V8Tx
- OS: FreeRTOS
- Toolchain: Keil MDK-ARM
- 底盘: X42S 步进 ×4, UART3 DMA
- 关节电机: GO-M8010-6, UART8 RS485
- IMU: USART2 DMA
- 二维码: XR1503MTEX, UART5 RX
- 串口屏: 淘晶驰 TJC, UART5 TX
- 时序基准: TIM6 (delaytime), TIM14 (HAL Tick 替代 SysTick)

## Key Paths
- 底盘控制: `task/chassis_control_task.c`
- 电机驱动: `MOTOR/motor.c`, `MOTOR/motor_control.c`
- 关节电机: `SENSOR/GO-M8010-6.c`
- IMU: `SENSOR/IMU.C`, `MOTOR/imu_control.c`
- 二维码: `SENSOR/QR_code.c`
- 串口屏: `SENSOR/tjc_usart_hmi.c`
- PID: `MOTOR/pid.c`
- 延时: `MOTOR/delay.c`
- 结构体: `mydefinition/Struct_encapsulation.h`
- 主流程: `task/main_task.c`
- 初始化: `task/init_task.c`, `task/start_task.c`
- CubeMX 生成: `Core/Src/*.c`, `Core/Inc/*.h`

## Interaction Rules

### 1. 改动前后必须列清单
每次改动用表格列出：文件、改动内容、目的。格式：
```
| 文件 | 改动 | 目的 |
|------|------|------|
| xxx.c:行号 | 旧 → 新 | 原因 |
```

### 2. 最小改动原则
- 只改和当前问题直接相关的代码
- 不重构、不顺手优化
- 改动必须可单文件回滚
- 用户说"测试X" → 写最简测试，不预设复杂流程

### 3. 调试优先用 Watch 变量
- 用户喜欢 Keil Watch 窗口直接加变量看
- 非必要不加断点
- 需观测的关键变量：`motor_check.flag_finish`, `motorX.actual_angle`, `MOTOR_ACTIONFALG`

### 4. 注释要准确
- 中文/英文都行，但不能写错方向/含义
- 不确定的方向/数值不写注释，让用户自己判断

### 5. 修根因，不打补丁
- 卡死 → 查 volatile、查临界区、查 ISR 锁死
- 跳变 → 查 static 变量共享、查 wrap 逻辑
- 不收敛 → 查 PID 参数、查死区逻辑

### 6. 改动前先给方案，等我确认再动手
- 先说改什么、怎么改、为什么
- 等我同意后才能改代码
- 紧急修复（如 typo、编译错误）除外

### 7. 不猜硬件
- 线缆、供电、焊接问题由用户确认
- 只分析软件侧根因

## Known Pitfalls
- `delaytime` 缺 `volatile` → `Delay_ms` 死循环（已修复）
- `normalize_angle` 的 `static flag` 共享 → 180° 转头 yaw 跳变
- `Direction_Calibration_turn` 死区输出 ±1 → 转头来回摆
- UART5 扫描器和串口屏共享 → 波特率必须一致

## Test Function Template
```c
static void Xxx_Test(void)
{
    // 初始化
    // 简单动作序列
    // 停住 / 循环
}
```
Main_Task 里调用，vTaskDelay 间隔，不用的测试注释掉。
