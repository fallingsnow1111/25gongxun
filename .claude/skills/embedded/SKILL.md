---
name: embedded
description: STM32F750V8 FreeRTOS 机器人固件（开环底盘）：审查清单 + 调试流程。按总线冲突/DMA/竞态/时序/控制环/任务调度六大类组织，覆盖 0xAA 电机协议、UART3 带宽、IMU 时序、转向调参、UART7 机械臂等待。详细案例见根目录 调试经验汇总.md。
---

# STM32F750V8 嵌入式审查 & 调试（开环底盘）

> 当前底盘为**开环速度 + 5ms 周期 + IMU 航向保持**方案，已非 PID 位置闭环。看到症状先查下表定位大类，再看对应章节。完整案例/参数表见根目录 `调试经验汇总.md`。

| 症状 | 根因类型 | 详细章节 |
|------|---------|---------|
| 单轮偶尔慢半拍 | ① 总线写路径 | §1 UART3 协议 |
| 发送超时丢帧（`chassis_odom_tx_fail_count`↑） | ① | §1 |
| 5ms 周期超跑（`chassis_period_overrun_count`↑） | ①/⑤ | §1 + §5 |
| UART3 偶发乱帧 | ② DMA 缓冲 | §2 |
| 崩溃静默挂死 | ② 内存/栈 | §2 |
| 脉冲快照超时 "pulse start lost" | ③ 竞态 | §3 |
| Z 卡住（`z_debug_last_timeout`） | ③/④ 时序 | §3 + §4 |
| IMU 归零后直行走弯 | ④ 异步等待 | §4 |
| 转向绕远路/转圈 | ⑤ 角度最短路径 | §5 |
| 转向过头/不到位 | ⑤ 阈值/settle | §5 |

---

## 〇、当前结构速查（2026-08 核实）

- 控制周期 `OPEN_LOOP_PERIOD_MS=5ms`（`MOTOR/motor_control.c:38`），开环函数在主流程任务内顺序调用，**无独立底盘任务**。
- 任务：IMU 7/128 · Main 5/512 · Start 5/256 · HMI 4/384（`task/*.c`）。
- 电机：`Motor_setspeed(vy,vx,vw)` → 0xAA 37B 多机速度包 + `00 FF 66 6B` 同步触发（`MOTOR/motor.c`）；`Motor_setspeed_fine` 保留 0.1RPM 转向精调。
- 发送保护：`uart3WriteBuf` 等 `gState==READY`，超 5ms 返回 `HAL_TIMEOUT`（丢帧计 `chassis_odom_tx_fail_count`）。
- 移动：正弦加减速 → 匀速 → 正弦减速 → 停车；距离接口 `Chassis_MoveByDistance`（脉冲系数换算）。
- 转向：`Chassis_TurnToAngle`，阈值 0.1°、settle 7 次、加减速 slew 2.0/4.0 RPM、MIN_SPEED 2.0。
- 航向：`Direction_Calibration_turn` → `Gyro_Pid`（Kp2.1 Ki0 Kd0.5 ±120），内部最短路径归一化；转弯前用 `Yaw_DriftCorrect` 补偿 IMU 漂移。
- 里程计：软件脉冲积分 + 段末世界坐标旋转（`Chassis_WorldCommitSegment`），写处 `taskENTER_CRITICAL`。
- 机械臂：UART7 DMA+IDLE，`Z_SetHeight` 阻塞等到位帧（4000ms 超时），上电等 500ms 再 `POSTION_init`。

---

## §1 UART3 总线 & 带宽（①）

- **带宽瓶颈**：每次 `Motor_setspeed` = 41B（37+4），@115200 ≈3.6ms，5ms 周期几乎占满 → **周期不能压到 2ms**。
- 多写路径并存：`Motor_setspeed`（跑图）、`Motor_ReadPulseSnapshot`（4×3ms 逐电机读脉冲）。读快照只在段前/段后调用，不要在 5ms 节奏内做。
- `uart3WriteBuf` 超时即丢帧；机械臂动作时不要跑 `Motor_setspeed`（动作前 `Motor_setspeed(0,0,0)` 停车）。
- 单轮慢半拍：该电机 0xF6 帧迟到/丢失 → 切 0xAA 多机同步（当前默认）后消失即 TX 路径问题。
- 关键帧可考虑直接寄存器轮询发送（不依赖 `HAL_GetTick()`）。

## §2 DMA 缓冲 & 栈/内存（②）

- 所有传给 DMA 的 buffer 必须 `static`/全局，不能放 DTCM：`all_send[37]`、`data[4]`、`sendmotor_coordination_data[3]`、`USART7_senddata[128]`。
- 栈现状：Main 512 / Start 256 / HMI 384 / IMU 128；初始化调链深时栈至少 256（旧坑：Start 128 → 256）。
- 四个 Fault Handler 是空 `while(1)` → 崩溃静默。建议加 HardFault 跳板保存 PC（见 `调试经验汇总.md`）。
- 崩溃分析：LR 判 PSP/MSP，SP 第 7 字（偏移 0x18）= PC；CFSR：NOCP=FPU、UNDEFINSTR=栈破坏、DIVBYZERO=浮点除零、IMPRECISERR=缓冲溢出。

## §3 竞态 & 共享状态（③）

- `motor_check.flag_finish`：UART3 解析路径置位、`Motor_ReadPulseSnapshot` 清零等 0x0F；任一电机丢帧则快照超时 → 重试/放弃并打日志。必须 `volatile`。
- `Z_POSTION.BIT` / `Telescopic_POSTION.BIT`：UART7 解析写、`Z_SetHeight` 阻塞读，单写单读，别加第二个写者。
- `car.actual_x/y/w` 世界坐标：写处 `taskENTER_CRITICAL`，读用 `Chassis_OdomGetSegmentSnapshot`。
- `motor1..4.target_angle`：`Motor_Action_Calculate_target` 已用 `__disable_irq()` 包裹。
- 遗留：`MOTOR_ACTIONFALG` 只剩初始化赋值，不再驱动流程，别依赖它。

## §4 异步命令 & 外设时序（④）

- `Imu_setZero()` 后必须等 200ms 再读 `imu.yaw`（测试参数 `IMU_STRAIGHT_ZERO_WAIT_MS=200`），否则直行走弯。
- UART7 冷启动：上电等 `UART7_COLD_BOOT_WAIT_MS=500ms` 再 `POSTION_init`，防机械臂驱动器串口活动污染 DMA。
- `Z_SetHeight` 阻塞等到位帧，超时 `Z_MAX_TIMEOUT_MS=4000ms` 放弃；看 `z_debug_last_timeout` / `z_debug_last_wait_ms` 确认是否超时。
- 接收进坏状态：`UART7_RxRestart` / `UART3_RxRestart`（Abort → 清 ORE → 复位 RxState → 重新 ReceiveToIdle_DMA）。
- 同一 UART 只能用一种接收机制（RXNE **或** DMA+IDLE），不能同时开。

## §5 控制环 & 角度/周期（⑤）

- **角度最短路径**：`Direction_Calibration_turn` / `getAngleZ` / `MoveTurnOnce` / `BlendSpeedAngle` 内部已 while 归一化 ±180°；**新增角度计算必须同样归一化**（270° 与 -90° 同一方向，naive 差值 360° → 转圈）。
- 转向到位：阈值 0.1°、settle 7 次（≈50ms）；`CHASSIS_TURN_MIN_SPEED=2.0` 只用于转向。
- 5ms 周期超跑：`chassis_period_overrun_count` / `chassis_period_max_ms`；说明调用点放不进 5ms 节奏。
- 距离接口是**软件脉冲积分闭环**：`Chassis_MoveByDistance` 用 `target_pulse = distance_cm × PULSE_PER_CM`，边跑边比对 `Chassis_OdomGetSegment` 积分脉冲提前减速。`PULSE_PER_CM` 是机械常数，**改速度不需要重标定**；短距离用低速度档让加减速放得下（5cm→20、10cm→40、20cm→80、40cm+→160）。
- 时间标定接口（`Chassis_HoldSpeedAngle`/`Chassis_BlendSpeedAngle`/`Chassis_MoveTurnOnce`/`Chassis_DriftStraightTurn`）按 tick×5ms 跑，改速度/周期要按档重新调参。
- 起步/停车打滑：用 `Chassis_SINAccel` 正弦加减速。
- 航向漂移补偿：`Yaw_DriftCorrect(expected_current, target_angle)` 每次转向前调用，期望角=上一段目标角。

## §6 任务 & FreeRTOS（⑥）

- IMU_Task prio 7 高于主流程 5：主流程里长阻塞（如 Z 等待 4s）不压 IMU 解析。
- 无独立底盘任务：开环底盘在主流程内顺序执行，旧"底盘任务抢占"结构不存在。
- 同优先级任务互抢 DMA TX：`uart3WriteBuf` 自带 gState 等待 + 5ms 超时。

---

## 调试分诊（Phase 0）

1. `chassis_odom_tx_fail_count` — UART3 发送失败？
2. `chassis_period_overrun_count` / `chassis_period_max_ms` — 5ms 周期超跑？
3. `motor_actual_pulse[4]` — 单轮滞后/丢失？
4. `imu.yaw` / `imu.angular_rate` — 静止稳定 ±0.2°？运动中漂移/摆动？
5. `car.actual_x/y/w` — 世界坐标漂移？
6. 转向：`Chassis_TurnToAngle` 返回值、`angle_error` — 到位与否。

## 报告格式

```
## Diagnosis
Fault/Symptom: [description]
Root cause: [identified cause]  Confidence: HIGH / MEDIUM / LOW

## Fix
File: [path:line]
Change: [what to change]

## Next Check
[What to observe after fix]
```

## 详细参考

- 完整案例表（当前问题 + 旧闭环历史教训）、PID 参数表、Watch 变量清单：**根目录 `调试经验汇总.md`**
