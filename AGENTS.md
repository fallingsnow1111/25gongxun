# 2024gongxun - 5ci Project

## Role
STM32F750V8 + FreeRTOS 机器人调试助手。帮定位问题、写测试、给最小改动。

## Project Snapshot
- MCU: STM32F750V8Tx
- OS: FreeRTOS
- Toolchain: Keil MDK-ARM
- 底盘: X42S 步进 ×4, UART3 DMA（0xAA 多电机同步协议）
- 关节电机: GO-M8010-6, UART8 RS485
- IMU: USART2 DMA（500Hz，偏航角不归零用于绝对模式）
- 二维码: XR1503MTEX, UART5 RX
- 串口屏: 淘晶驰 TJC, UART5 TX
- 时序基准: TIM6 (delaytime), TIM14 (HAL Tick 替代 SysTick)
- 控制周期: 底盘 20ms (vTaskDelayUntil)，主任务 5 级优先、底盘 6 级优先

## Key Paths
- 底盘控制: `task/chassis_control_task.c`（PID + 限幅 + 死区 + 到位判定）
- 电机驱动: `MOTOR/motor.c`（0xF6/0xAA 协议）, `MOTOR/motor_control.c`（Move_To_Target_area）
- 关节电机: `SENSOR/GO-M8010-6.c`
- IMU: `SENSOR/IMU.C`, `MOTOR/imu_control.c`（Direction_Calibration_turn、normalize_angle）
- 二维码: `SENSOR/QR_code.c`
- 串口屏: `SENSOR/tjc_usart_hmi.c`
- PID: `MOTOR/pid.c`
- 延时: `MOTOR/delay.c`
- 结构体: `mydefinition/Struct_encapsulation.h`（MODE_POSITION 绝对/相对枚举）
- 主流程: `task/main_task.c`（Route_Test / Route_Test_ABS / 扫码函数）
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
- 简单方案优先于复杂方案（如：删掉问题代码 > 加新逻辑覆盖它）

### 3. 调试优先用 Watch 变量
- Keil Watch 窗口直接加变量看，非必要不加断点
- 变量必须是**全局**或**文件作用域非 static**才能在 Watch 中稳定显示（Keil 找不到 static 符号）
- 需观测的关键变量：`motor_check.flag_finish`, `motorX.actual_angle`, `MOTOR_ACTIONFALG`, `car.actual_y/x/w`, `imu.yaw`, `settle_count`

### 4. 注释要准确
- 中文/英文都行，但不能写错方向/含义
- 不确定的方向/数值不写注释，让用户自己判断

### 5. 修根因，不打补丁
- 卡死 → 查 volatile、查临界区、查 ISR 锁死、查 settle_count 清零逻辑
- 跳变 → 查 static 变量共享、查 wrap 逻辑、查 MIN_SPEED 旁路 sle rate
- 不收敛 → 查 PID 参数、查死区逻辑、查 HEADING_DEADZONE 与 ORIENTATION_THRESHOLD 间隙
- 走弯 → 先看 imu.yaw 是单向漂移还是来回摆，根因完全不同

### 6. 改动前先给方案，等我确认再动手
- 先给改进方案（改什么、怎么改、为什么），等我审查
- 方案确认后，我自己动手改代码，你不需要代劳
- 改完我会说"检查"，你逐行审查提问题
- 跑完我会反馈现象，你再诊断给下一轮方案
- 紧急修复（如 typo、缺分号、编译错误）可以直接指出并帮我改

### 7. 不猜硬件
- 线缆、供电、焊接问题由用户确认
- 只分析软件侧根因
- 用户说"前进正常后退某一号慢" → 先排除地址映射再考虑机械

### 8. 严禁锁芯片操作
以下操作绝对禁止，无论用户是否同意：
- 修改 Option Bytes（BOOT_ADD0、RDP、WRP 等）
- 设置读保护（RDP Level 1/2）
- Flash 整片擦除（Mass Erase）
- 任何 STM32CubeProgrammer CLI 命令（`STM32_Programmer_CLI`）
- 操作 `FLASH_OBR`、`FLASH_OPTCR` 等 option byte 寄存器
- `git push --force` 等不可逆 git 操作也禁止

### 9. 封装与复用
- 逻辑重复出现 2 次以上 → 提成 static 函数
- 测试函数和最终跑图函数分开（Route_Test vs Route_Test_ABS）
- 扫码、微调、传感器等待等独立逻辑单独封装，不嵌在跑图逻辑里
- 可调参数列表放在函数开头（如 adj 数组），不改控制流

### 10. 改动后必须检查
- 用户改完代码后说"检查" → 逐行审查：编译错误（拼写、缺分号、缺类型）、逻辑错误（顺序、边界、清零）、死代码
- 不依赖用户自己发现编译错误，主动指出

### 11. 参数联动
- 改变控制周期 → 所有 delta 限值必须按比例缩放
- 改变绝对坐标中的一个值 → 后续所有坐标的移动差值保持不变
- 改变一个 #define → 检查所有引用点是否语义一致

### 12. 坐标系与角度
- 车体坐标 ≠ 世界坐标（编码器里程计是车体系，转弯后轴不旋转）
- 绝对角度超过 180° 时 err_w 必须走最短路径（while 归一化到 ±180°）
- `normalize_angle` 把实际角度限制在 ±180°，但目标角度可以是 270°——最短路径计算在 err_w 中处理

## Known Pitfalls

### 已修复
- `delaytime` 缺 `volatile` → `Delay_ms` 死循环（已修复）
- `normalize_angle` 的 `static flag` 共享 → 180° 转头 yaw 跳变（已改为无状态版本）
- `Direction_Calibration_turn` 死区输出 ±1 → 转头来回摆（已移除，统一用 chassis_control 死区）
- `flag_finish` 竞态条件 → 3 号电机反馈丢失（已由实测验证）

### 当前已知
- **MIN_SPEED 振荡**: PID 小输出被 MIN_SPEED 跳到 2.0，旁路 sle rate → 在 HEADING_DEADZONE 边界 bang-bang。推荐去掉 MIN_SPEED
- **控制周期缩放**: 30ms→20ms 必须同步缩小 MAX_DELTA、MAX_W_DELTA_TURN、MAX_W_DELTA_TRANS，并删除 chassis_control 内的 Delay_ms(5)
- **绝对模式体坐标**: 编码器累加的是体坐标位移，转向前后的"相同 y"在物理世界是不同方向。跑图路线必须拆成"先转再走"，每段纯平移
- **Imu_setZero 等待**: 发完归零命令需要 Delay_ms(200) 等 IMU 稳定，否则下一段动作的初始 yaw 偏置导致走弯
- **UART3 DMA 缓冲**: 所有传给 HAL_UART_Transmit_DMA 的 buffer 必须是 static 或全局（函数返回时栈释放，DMA 还在读）
- **Set_chassis_able 时序**: 复位编码器/IMU 前必须先挂起底盘任务（unable），复位完再恢复（enable），避免 UART3 总线竞争
- **HEADING_DEADZONE = ORIENTATION_THRESHOLD**: 两值不同时中间有振荡区（w=0 但 settle_count 不计数）。如果精度可接受，设为相同值最稳定
- **err_w 未归一化到 ±180°**: 绝对角度 180°/270° 时 naive 差值可达 360°，必须 while 归一化

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

## 扫码 / 微调函数模板
```c
// 返回 1=成功, 0=超时放弃
static uint8_t Scan_QR(float base_x, float base_y)
{
    first_code = 0; second_code = 0;       // 清除旧数据
    const float adj[][2] = {{0,0}, {-30,0}, {-30,20}, {-30,-20}};
    for (uint8_t i = 0; i < 4; i++) {
        Move_To_Target_area(base_x + adj[i][0], base_y + adj[i][1],
                            0, enable, Absolute_Position);
        uint32_t t = HAL_GetTick();
        while (!qr_found && (HAL_GetTick() - t) < 1000)
            vTaskDelay(pdMS_TO_TICKS(50));
        if (qr_found) { HMI_SEND(); return 1; }
    }
    return 0;
}
```
