# Code Review: 代码优化待办清单

> 审查日期: 2026-06-03
> 项目: 2024gongxun - 5ci (STM32F750V8 + FreeRTOS)
> 范围: 全部应用层代码，不含 HAL/CMSIS/FreeRTOS 内核

---

## 🔴 严重 (CRITICAL)

### 1. `configENABLE_FPU = 0` — FPU 未启用

**文件:** `Core/Inc/FreeRTOSConfig.h:55`

**问题:**
Cortex-M7 有硬件 FPU，但 FreeRTOS 配置中 `configENABLE_FPU` 设为 0。
项目中 PID 计算、电机运动学解算 (`Motor_Action_Calculate_actual`)、IMU 角度处理全部使用 `float`。
如果 Keil 编译器开了硬件 FPU (`--fpu=fpv5-sp-d16`)，FreeRTOS 任务切换时**不会保存/恢复 FPU 寄存器 (S0-S31)**，
导致 Chassis_Control_Task、IMU_Task、Main_Task 之间的浮点计算结果互相破坏。

**修复:**
```c
#define configENABLE_FPU  1
```
修改后验证：看 Keil 工程 Target → Floating Point Hardware 是否选 "Single Precision"，
确保编译器 flag 和 RTOS 配置一致。

---

### 2. 四个 Fault Handler 均为空死循环

**文件:** `Core/Src/stm32f7xx_it.c:83-153`

**问题:**
`HardFault_Handler`、`MemManage_Handler`、`BusFault_Handler`、`UsageFault_Handler` 全部是空的 `while(1)` 循环。
一旦发生 crash（总线错误、非法内存访问、除零等），调试器只能看到一个死循环，无法知道崩溃原因。

**修复 (最小改动):** HardFault_Handler 中保存 stacked PC 和 LR 到全局变量：
```c
// stm32f7xx_it.c
static uint32_t _fault_pc, _fault_lr, _fault_sp;

void HardFault_Handler(void)
{
    __asm volatile(
        "TST LR, #4          \n"
        "ITE EQ              \n"
        "MRSEQ R0, MSP       \n"
        "MRSNE R0, PSP       \n"
        "LDR R1, [R0, #24]   \n"  // stacked PC
        "LDR R2, [R0, #20]   \n"  // stacked LR
        : : : "r0", "r1", "r2"
    );
    while(1) { __NOP(); }
}
```
配合 Keil 的 Peripherals → Core Peripherals → Fault Reports 查看 CFSR/HFSR 寄存器。

---

### 3. `Error_Handler()` 无诊断信息

**文件:** `Core/Src/main.c:248-257`

**问题:**
`Error_Handler()` 只做了 `__disable_irq()` + `while(1)`。
HAL 初始化失败时（时钟、Flash、PWR 等）无法知道哪个外设出的问题。

**修复:**
```c
void Error_Handler(void)
{
    __disable_irq();
    __BKPT(0);  // Keil 调试器自动在此断点停下
    while(1) {}
}
```

---

### 4. `motor_check.flag_finish` 非原子操作 — 竞态条件（已实测验证）

**文件:**
- `MOTOR/motor.c:406-433` (ISR 侧：U3_process_single_frame 中 |= 置位)
- `task/chassis_control_task.c:93-166` (任务侧：先读后清)

**问题:**
ISR 中的 `motor_check.flag_finish |= (1 << 0)` 是读-改-写，非原子操作。
任务侧在第 93 行读 `if (flag_finish < 0x0F)`，到第 166 行 `flag_finish = 0` 之间，
ISR 写入的新回复 bit 会被清零 —— **3 号电机反馈丢失，已实测验证**。

**时序:**
```
[电机1回复] [电机2回复]          [电机3回复]    [电机4回复]
     |           |                   |              |
     v           v                   v              v
 flag=0x01 → flag=0x03 → ... →  flag=0x07 → ... → flag=0x0F
                                                     |
                                              chassis_control 读到 0x0F ✓
                                                     |
                                              开始 PID 计算...
                                                     |
                                   ┌── 3号电机下一个周期的回复到达! ──┐
                                   │  flag |= 0x04                   │
                                   └─────────────────────────────────┘
                                                     |
                                              flag_finish = 0  ← 把3号的bit也清了!
```

**修复方案 (已在 memory 中记录):**
```c
void chassis_control(void)
{
    motor_read_coordination_all();

    // 原子快照：在读到的同时清零
    uint8_t flag_snapshot;
    taskENTER_CRITICAL();
    flag_snapshot = motor_check.flag_finish;
    motor_check.flag_finish = 0;
    taskEXIT_CRITICAL();

    if (flag_snapshot < 0x0F) {
        Motor_setspeed(0, 0, 0);
        Delay_ms(6);
        return;
    }
    // ... PID 计算、速度发送 ...
    // 末尾不再清零
}
```

**注意:** 参考 memory `flag-finish-race-condition`，已在 `GX_0226_skill.md:53` 记录。

---

## 🟠 高 (HIGH)

### 5. 控制循环中大量 `Delay_ms()` 忙等待

**文件:**
- `task/chassis_control_task.c:96,163`
- `MOTOR/motor.c:198-208` (`send_speed_data_switch`)
- `MOTOR/motor.c:213-220` (`send_postion_data_switch`)
- `MOTOR/motor.c:296-301` (`Motor_SetZero`)

**问题:**
`Delay_ms()` 是 CPU 空转忙等 (`while(delaytime<=nums){}`)。
`Chassis_Control_Task` (优先级 6) 在一个周期内忙等约 30ms（读坐标 4ms + 速度发送 16ms + 控制内 10ms）。
高优先级任务忙等期间**饿死所有低优先级任务**（Main_Task 优先级 5）。

**修复:**
在 FreeRTOS 任务上下文中用 `vTaskDelay(pdMS_TO_TICKS(n))` 替换：
```c
// 旧: Delay_ms(4);
// 新: vTaskDelay(pdMS_TO_TICKS(4));
```
**注意:** ISR / 临界区内不能这样改。`motor_read_coordination_all()` 内部 1ms 延时量太小（< tick），
如果 tick 是 1kHz，保持 `Delay_ms(1)` 合理（只是仍会忙等 1ms）。

---

### 6. 控制周期不精确

**文件:** `task/chassis_control_task.c:169-175`

**问题:**
```c
void Chassis_Control_Task(void *pvParameters)
{
    while(1) {
        chassis_control();       // 执行时间不定 (30~50ms)
        vTaskDelay(60);          // 相对延时
    }
}
```
使用 `vTaskDelay(60)` + 可变执行时间 → 实际周期约 90~110ms，远超标称的 20ms。

**修复:**
```c
void Chassis_Control_Task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while(1) {
        chassis_control();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
    }
}
```
同时需要 `FreeRTOSConfig.h` 中 `INCLUDE_vTaskDelayUntil` 改为 `1`。

---

### 7. `MOTOR_FINISHFLAGEXAM` 在 ISR 内关全局中断

**文件:** `MOTOR/motor.c:490,510,515`

**问题:**
```c
__disable_irq();              // 关闭所有中断
// memcpy + 数据处理
__enable_irq();
```
`__disable_irq()` 关闭**所有**中断（包括 TIM6 的 `delaytime` 计数、TIM14 的 HAL Tick）。
处理时间过长会导致 FreeRTOS 时基漂移和 `Delay_ms()` 不准。

**修复:**
DMA 停止后 `buff_len` 已确定，`memcpy` 不需要关中断。
去掉 `__disable_irq()` / `__enable_irq()`，或改用 `taskENTER_CRITICAL_FROM_ISR()` / `taskEXIT_CRITICAL_FROM_ISR()`（仅关闭受 FreeRTOS 管理的中断）。

---

### 8. `Start_Task` 创建顺序不当

**文件:** `task/start_task.c:10-17`

**问题:**
`Main_Task` (优先级 5) 先于 `Chassis_Control_Task` (优先级 6) 创建。
Main_Task 一运行就调用 `Move_To_Target_area()`，该函数等待 `MOTOR_ACTIONFALG == finish`，
但这个标志由 `chassis_control()` 设置。如果 Main_Task 先被调度，它会一直等到 Chassis_Control_Task 完成动作。
在优先级 5 < 6 的情况下，Chassis_Control_Task 就绪后会抢占 Main_Task，所以实际不会死锁，
但 Main_Task 的第一条移动指令可能在 Chassis_Control_Task 就绪前就开始执行 → 没有底盘控制的空移动。

**修复:**
```c
Chassis_Control_Task_Create();  // 先创建高优先级底盘控制
Main_Task_create();              // 再创建主流程
```

---

## 🟡 中 (MEDIUM)

### 9. 底盘 PID 纯 P 控制，无 I/D

**文件:** `task/chassis_control_task.c:37-38`

**问题:**
```c
PID_Init(&chassis_pid_y, 0.280f, 0, 0, 350.0f, -350.0f);  // Ki=0, Kd=0
PID_Init(&chassis_pid_x, 0.280f, 0, 0, 350.0f, -350.0f);  // Ki=0, Kd=0
```
纯 P 控制存在稳态误差。对需要精确定位的机器人底盘，缺少积分项意味着**永远无法完全消除位置静差**。

**修复:**
```c
PID_Init(&chassis_pid_y, 0.280f, 0.001f, 0.0f, 350.0f, -350.0f);
PID_Init(&chassis_pid_x, 0.280f, 0.001f, 0.0f, 350.0f, -350.0f);
```
Ki 值从 0.001 开始调，逐步增大直到消除静差但不产生过冲。
已有 `PID_Compute_Integral_separation()` 函数可用（接近目标时启用积分）。

---

### 10. `delaytime` 缺少 `volatile`

**文件:** `MOTOR/delay.c:4`

**问题:**
```c
uint16_t delaytime;  // ISR 中写入，Delay_ms() 中轮询读取
```
`delaytime` 在 TIM6 ISR (`main.c:238`) 中递增，在 `Delay_ms()` 中轮询。
非 volatile 时编译器可能把值缓存在寄存器中，导致死循环。

**修复:**
```c
volatile uint16_t delaytime;
```

---

### 11. `delaytime` 在 `Delay_ms` 中清零存在微小竞争

**文件:** `MOTOR/delay.c:19-22`

**问题:**
```c
void Delay_ms(uint16_t nums)
{
    delaytime = 0;              // 写
    while(delaytime <= nums){}  // 读（ISR 在后台递增）
}
```
两个任务同时调用 `Delay_ms` 会互相清零 `delaytime`（全局变量）。

**修复:**
```c
void Delay_ms(uint16_t nums)
{
    uint16_t start = delaytime;
    while((uint16_t)(delaytime - start) <= nums) {}
}
```
用差值比较代替归零，消除多任务并发问题。

---

### 12. `main.c:35` — 未初始化的遗留全局变量

**文件:** `Core/Src/main.c:35`

**问题:**
```c
int x, y, z, a = 0;  // 只有 a 被初始化, x/y/z 未初始化
```
这四个变量未被任何代码使用，属于遗留代码。

**修复:** 删除该行。

---

### 13. IMU_Task 占着最高优先级空转

**文件:** `task/imu_task.c:9-16`

**问题:**
IMU_Task 优先级 7（系统最高），但只是一个空循环：
```c
while(1) {
    vTaskDelay(1000);  // 什么都不做
}
```
它可能抢占任何其他任务（包括 Chassis_Control_Task）。

**修复:**
暂时不用就 `vTaskSuspend(NULL)` 或不创建。
或者把 IMU 数据读取放入此任务（读取 `imu.yaw` 并做归一化），合理利用这个优先级。

---

### 14. 重复 include

**文件:**
- `MOTOR/imu_control.c:7` — `#include "IMU.h"` 重复（第 4 行已有）
- `MOTOR/motor_control.c:9` — `#include "IMU.h"` 重复（第 7 行已有）

**修复:** 删除重复的 include。

---

### 15. `check()` 函数——`static _check` 永远只增不减

**文件:** `task/chassis_control_task.c:69-86`

**问题:**
`static uint16_t _check` 只在特定条件下递增，从不重置也不被外界读取（`vofa_printf` 已注释掉）。
每次调用 `chassis_control()` 都会执行 `check()` → 空耗 CPU。

**修复:** 不需要就删除该函数和调用点。

---

### 16. `main.h` 中 include 了所有用户模块

**文件:** `Core/Inc/main.h:39-60`

**问题:**
`main.h` 被 HAL 生成的文件 include（如 `stm32f7xx_it.c`、`dma.c`），
但这些文件不需要 `QR_code.h`、`GO-M8010-6.h`、`usmart.h` 等。
每次修改任一用户模块头文件 → 所有 HAL 生成文件触发重编译。

**修复:**
用户模块的 include 应该放在各自的 `.c` 文件中，而不是在 `main.h` 的 `USER CODE BEGIN ET` 区块。

---

### 17. 注释掉的大量死代码

**位置:**
- `Core/Src/main.c:105-132` — 旧初始化流程
- `task/main_task.c` — `User_function()` / `User_function_final()` 被注释
- `Core/Src/stm32f7xx_it.c` — USART1/USART2/UART5/USART6/UART8 的 IRQHandler 被注释
- `task/chassis_control_task.c:164` — `motor_check.flag_ready = Incomplete` 被注释

**影响:** 增加阅读负担，版本迭代时容易混淆哪些是"暂时不用"哪些是"已废弃"。

**修复:** 确认已废弃的代码直接删除（git 历史可恢复）。暂时不用的标注 TODO 和原因。

---

## 🔵 低 (LOW)

### 18. 拼写错误

| 文件 | 当前 | 应为 |
|------|------|------|
| `motor.h` / `motor.c` | `MOTOR_ACTIONFALG` | `MOTOR_ACTION_FLAG` |
| `postion_control.c` / `.h` | `postion` | `position` |
| `motor_control.c:49` | `Obimeter_SetZero` | `Odometer_SetZero` |
| `Struct_encapsulation.h:34-37` | `firse`, `thrid` | `first`, `third` |
| `Struct_encapsulation.h:25` | `ACITON_INIT` | `ACTION_INIT` |
| `Struct_encapsulation.h:28` | `wasehouse` | `warehouse` |
| `Struct_encapsulation.h:33` | `WASEHOUSE_T` | `WAREHOUSE_T` |

---

### 19. `delay.h` 位置不当

**文件:** `MOTOR/delay.h`

`delay` 是通用模块（被多个子系统使用），不应放在 `MOTOR/` 目录。建议移到 `Core/Inc/`。

---

### 20. `StartDefaultTask` 浪费资源

**文件:** `Core/Src/freertos.c:123-132`

```c
void StartDefaultTask(void const *argument)
{
    for(;;) { osDelay(1); }
}
```
占用 128 words 堆栈和一个任务槽，但毫无作用。

**修复:** CubeMX 生成后直接删除任务创建代码，或注释掉。

---

### 21. `PID_Compute` 积分每次累积但 ki=0 时不使用

**文件:** `MOTOR/pid.c:60-86`

**问题:**
即使 `ki = 0`，积分项仍在累积（`pid->integral += error`），虽然被 `integralMax` 限制，
但浪费计算。

**修复:**
```c
if (pid->ki != 0.0f) {
    pid->integral += error;
    // ... 积分限幅 ...
}
```

---

## 📊 汇总

| 严重程度 | 数量 | 关键项 |
|----------|------|--------|
| 🔴 严重 | 4 | FPU 未启用、Fault Handler 为空、Error_Handler 无诊断、flag_finish 竞态 |
| 🟠 高 | 4 | 忙等待阻塞、周期不精确、ISR 内关全局中断、任务创建顺序 |
| 🟡 中 | 9 | PID 纯P、缺 volatile、IMU 空转、重复 include、死代码等 |
| 🔵 低 | 4 | 拼写错误、文件位置、空任务 |

**优先修复顺序建议:**
1. `configENABLE_FPU = 1`（影响所有浮点计算正确性）
2. `flag_finish` 原子快照（已实测验证导致 3 号电机丢反馈）
3. Fault Handler + Error_Handler 诊断（出问题时能定位）
4. 忙等待替换为 `vTaskDelay`（实时性）
5. PID 参数调优 + 周期精确化（控制品质）
