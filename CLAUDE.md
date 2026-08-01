# 2024gongxun - 5ci Project

## Role
STM32F750V8 + FreeRTOS 机器人调试助手。帮定位问题、写测试、给最小改动。

## Project Snapshot
- MCU: STM32F750V8Tx
- OS: FreeRTOS
- Toolchain: Keil MDK-ARM
- 底盘: X42S 步进 ×4, UART3 DMA（0xAA 多电机同步协议）
- 关节电机: GO-M8010-6, UART8 RS485
- IMU: HWT101, USART2 DMA（11字节帧, 200Hz 输出, 5ms IMU_Process 周期任务解析）
- 二维码: XR1503MTEX, UART5 RX
- 串口屏: 淘晶驰 TJC, UART5 TX
- 视觉: MaixCam Pro + OpenCV（色环/圆盘机定位），代码在 `Vision Project\gongxun_cv\raspberry_pi\`
- 时序基准: TIM6 (delaytime), TIM14 (HAL Tick 替代 SysTick)
- 控制周期: 底盘开环 5ms (OPEN_LOOP_PERIOD_MS, vTaskDelayUntil)；任务优先级 IMU 7 / Main 5 / Start 5 / HMI 4，无独立底盘任务

## Key Paths
- 底盘控制（开环）: `MOTOR/motor_control.c`（Chassis_MoveByDistance / Chassis_TurnToAngle / Chassis_OpenLoop_SetSpeed / Chassis_BlendSpeedAngle）
- 电机驱动: `MOTOR/motor.c`（0xAA 多机速度包 + 同步触发, uart3WriteBuf 5ms 超时保护）
- 机械臂 Z/Y: `MOTOR/postion_control.c`（POSTION_init / Z_SetHeight / Y_SetLength，UART7）
- 关节电机: `SENSOR/GO-M8010-6.c`（UART8）
- IMU: `SENSOR/IMU.C`, `MOTOR/imu_control.c`（Direction_Calibration_turn、normalize_angle、Yaw_DriftCorrect）
- 二维码: `SENSOR/QR_code.c`
- 串口屏: `SENSOR/tjc_usart_hmi.c`
- 视觉: `Vision Project\gongxun_cv\raspberry_pi\color_line_det.py`（OpenCV + MaixCam Pro）
- PID: `MOTOR/pid.c`
- 延时: `MOTOR/delay.c`（TIM6 delaytime）
- 结构体: `mydefinition/Struct_encapsulation.h`（MODE_POSITION 绝对/相对枚举）
- 主流程: `task/main_task.c` → `APP/task_flow.c`（Flow_RunCurrent 跑图）；测试: `APP/test.c`
- 初始化: `task/start_task.c`（Init_Task_Create，POSTION_init 等）
- 经验文档: `调试经验汇总.md`（根目录，工程问题分类速查）
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
- 需观测的关键变量：`motor_check.flag_finish`, `motor_actual_pulse[4]`, `motor_debug_cmd_rpm_x10[4]`, `chassis_period_overrun_count`, `chassis_odom_tx_fail_count`, `imu.yaw`, `imu.angular_rate`, `car.actual_x/y/w`, `Z_POSTION.BIT`（完整清单见 `调试经验汇总.md` 附录 B）

### 4. 注释要准确
- 中文/英文都行，但不能写错方向/含义
- 不确定的方向/数值不写注释，让用户自己判断

### 5. 修根因，不打补丁
- 卡死 → 查 volatile、查临界区、查 ISR 锁死、查 Z 到位等待（Z_MAX_TIMEOUT_MS 超时）、查脉冲快照 flag_finish
- 跳变 → 查 static 变量共享、查 wrap 逻辑
- 不收敛/转不到位 → 查 Gyro_Pid 参数（Kp/Kd）、查转向阈值与 CHASSIS_TURN_MIN_SPEED、查角度最短路径归一化
- 走弯 → 先看 imu.yaw 是单向漂移还是来回摆：漂移 → Imu_setZero 后 200ms 未等 / 需 Yaw_DriftCorrect；摆动 → Kd 或机械

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
- 测试函数和跑图函数分开（`APP/test.c` 测试 vs `APP/task_flow.c` Flow_RunCurrent 跑图）
- 扫码、微调、传感器等待等独立逻辑单独封装，不嵌在跑图逻辑里
- 可调参数列表放在函数开头（如 adj 数组），不改控制流

### 10. 改动后必须检查
- 用户改完代码后说"检查" → 逐行审查：编译错误（拼写、缺分号、缺类型）、逻辑错误（顺序、边界、清零）、死代码
- 不依赖用户自己发现编译错误，主动指出

### 11. 参数联动
- 距离接口（`Chassis_MoveByDistance`）是软件脉冲积分闭环：`PULSE_PER_CM` 是机械常数，改速度**不影响到位距离**，不需要重标定；短距离才需要降速档（加减速占满行程，5cm→20、10cm→40、20cm→80、40cm+→160）
- 时间标定接口（`Chassis_HoldSpeedAngle`/`Chassis_BlendSpeedAngle`/`Chassis_MoveTurnOnce`/`Chassis_DriftStraightTurn`）按 tick×5ms 跑，**改速度/周期要按档重新调参**
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
- `Direction_Calibration_turn` 死区输出 ±1 → 转头来回摆（已移除，统一走 Gyro_Pid）
- `flag_finish` 竞态条件 → 3 号电机反馈丢失（已由实测验证）

### 当前已知
- **UART3 带宽瓶颈**: `Motor_setspeed` 每次 41 字节（0xAA 37B + 同步触发 4B），@115200 ≈3.6ms，5ms 周期内几乎占满 → 控制周期不能压到 2ms
- **距离接口是脉冲闭环**: `Chassis_MoveByDistance` 用 `target_pulse = distance_cm × PULSE_PER_CM`，边跑边比对软件积分脉冲（`Chassis_OdomGetSegment`）提前减速。`PULSE_PER_CM` 是机械常数，**改速度不需要重标定**；短距离用低速度档让加减速放得下（5cm→20、10cm→40、20cm→80、40cm+→160）
- **Imu_setZero 等待**: 发完归零命令需 200ms 再读 `imu.yaw`（测试参数已固化 `IMU_STRAIGHT_ZERO_WAIT_MS=200`），否则直行走弯
- **UART3 DMA 缓冲**: 所有传给 DMA 的 buffer 必须 static 或全局（0xAA 37B all_send 等），不能放 DTCM
- **角度最短路径**: `Direction_Calibration_turn`/`getAngleZ`/`BlendSpeedAngle` 内部已 while 归一化到 ±180°；**新增角度计算必须同样归一化**（270° 与 -90° 是同一方向）
- **UART7 机械臂**: `Z_SetHeight` 阻塞等到位帧，带 4000ms 超时；上电后等 500ms 再 `POSTION_init` 防冷启动污染 DMA
- **世界坐标 vs 体坐标**: 段内软件脉冲积分，段末 `Chassis_WorldCommitSegment` 旋转变换算世界坐标；跑图拆"先转再走"

## Test Function Template
```c
static void Xxx_Test(void)
{
    // 初始化
    // 简单动作序列
    // 停住 / 循环
}
```
测试函数放 `APP/test.c`，跑图函数放 `APP/task_flow.c`（Flow_RunCurrent 调用），vTaskDelay 间隔，不用的测试注释掉。

## 视觉定位 / 微调函数模板（当前）
```c
// 色环/物料视觉定位：像素误差 → 平移速度，连续稳定帧才算完成。参考 TaskFlow_RingLocateOne。
// 参数：VISION_LOCATE_KP=0.25、SPEED_MIN=2.0、SPEED_MAX=15.0、LOCATE_DEADZONE、LOCATE_STABLE_COUNT（APP/task_flow.c 顶部）
static uint8_t Vision_Locate(float vx, float vy, float target_angle)
{
    // dx = 目标中心 - 识别中心;  vx = clamp(-dx * VISION_LOCATE_KP, ±SPEED_MAX)，低于 SPEED_MIN 用 SPEED_MIN
    // Chassis_MoveByDistance(vx, vy, target_angle, 0.5f) 小步接近
    // 连续 LOCATE_STABLE_COUNT 帧进死区 → return 1;  超时 RING_LOCATE_TIMEOUT_MS → return 0
    return 0;
}
```
