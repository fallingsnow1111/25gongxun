---
name: embedded
description: STM32F750V8 FreeRTOS 机器人固件：审查清单 + 调试流程。按总线冲突/竞态/时序三大类组织，覆盖 DMA TX 安全、多任务 UART 仲裁、IMU 时序、电机状态机、控制环调参经验。
---

# STM32F750V8 嵌入式审查 & 调试

> 看到症状先查下表定位理论要点，再看对应章节的具体处理。

| 症状 | 理论要点 | 详细章节 |
|------|---------|---------|
| 电机偶发不动/乱动 | T1 DMA buffer / T2 总线串行化 | §5 UART3 TX |
| 烧录后能跑，重新上电不跑 | T3 vTaskSuspend / T7 初始化顺序 | §7.1 Bus Arbitration |
| 复位后位置清零失效 | T2 总线串行化 / T3 vTaskSuspend | §7.1 + §10 |
| IMU归零后直行走弯 | T4 异步等待 | §10 IMU Reset Timing |
| 动作完成标志不触发/卡死 | T5 竞态窗口 | §7 Motor State Machine |
| 航向震荡到不了目标 | T9 控制链顺序 / T10 角度最短路径 | §3.13 §3.14 |
| 广播读命令数据全乱 | T6 多设备回包碰撞 | §6 Emm协议 |
| 任务栈扩大后稳定 | T8 FreeRTOS 栈 | §8 Task Priority |
| 斜着走/某轮不动 | T1 DMA buffer | §2 UART3 TX Debug |

---

# STM32F750V8 嵌入式审查 & 调试

---

## Part 0: 嵌入式工程理论要点 + 项目案例

每条"要点"是通用工程原则，"案例"是本项目里真实踩过的坑。

---

### T1. DMA 缓冲区生命周期必须覆盖整个传输过程

**要点**：`HAL_UART_Transmit_DMA` 立刻返回，DMA 在后台异步读 buffer。若 buffer 是局部变量，函数返回后栈帧销毁，DMA 读到的是已被覆盖的内存。

**案例**：`Send_motor_together` 的 `data[4]`、`motor_read_coordination` 的 `sendmotor_coordination_data[3]`、`send_speed_data_all` 的 `all_send[37]` 都是局部变量，造成偶发乱帧。全部加 `static` 后解决。

**规则**：所有传给 `HAL_*_DMA` 的 buffer 必须是 `static` 或全局。

---

### T2. 共享总线上多个写路径必须串行化

**要点**：单条 UART 物理总线同时只能有一个发送方。多个任务/ISR 并发写同一 UART → 帧损坏或 HAL_BUSY 静默丢帧。

**案例**：底盘任务（prio 6）持续调 `Motor_setspeed → UART3`，主任务同时调 `Motor_SetZero → UART3`。电机复位命令被底盘任务的速度命令打断，复位失败，下一段动作从错误位置出发，500mm 跑成 1600mm。

**规则**：复位前调 `Set_chassis_able(unable)` 挂起底盘任务，复位完成后再 `enable`。外设定时返回（定时推送）也是写路径，同样需要在复位前 Stop，复位后 Init。

---

### T3. vTaskSuspend 不挂起 DMA

**要点**：`vTaskSuspend` 只暂停任务的代码执行，不影响硬件外设。已启动的 DMA 传输会继续运行直到完成。如果任务被挂起时 DMA 还在读某个 buffer，该 buffer 仍被占用。

**案例**：底盘任务挂起后，定时返回的 DMA 仍持续接收电机推送的位置数据，覆盖刚 `Motor_SetZero` 清零的 `actual_angle`，导致下一动作起始位置错误。

**规则**：涉及外设操作的资源保护，必须在协议/命令层做隔离（先停止定时返回），而不能只依赖任务挂起。

---

### T4. 异步命令后必须等待硬件完成

**要点**：向外设发送"配置"或"复位"指令是异步的——UART 帧发出后硬件需要时间执行。立即读取状态会得到旧值。

**案例**：`Imu_setZero()` 发完 UART 命令立刻启动控制环，IMU 硬件还没完成归零（约需 200ms），`imu.yaw` 仍是旧值。底盘控制环用旧 yaw 做航向校正，车直行过程中偏弯，且每次转弯后必现。加 `Delay_ms(200)` 后解决。

**规则**：发完异步命令，等待时间 ≥ 硬件保证的最小稳定时间，然后再读状态。

---

### T5. 共享状态的写入顺序决定竞态窗口大小

**要点**：在多任务/ISR 环境中，如果一个操作依赖"A完成后B才能生效"，两者之间的代码窗口就是竞态窗口。任何任务切换发生在这个窗口内都可能导致错误。

**案例**：`Move_To_Target_area` 先调 `Motor_SetZero()`，再设 `MOTOR_ACTIONFALG = Incomplete`。底盘任务在 SetZero 和 Incomplete 之间醒来，看到 flag = finish（上一次遗留），直接返回不执行。正确顺序：先设 Incomplete，再 SetZero。

**规则**：竞态窗口越小越好。状态标志的设置应该尽可能晚（接近真正需要的时刻），读取应该尽可能早（减少依赖旧值的时间）。

---

### T6. 多设备共享单 UART 总线的回包碰撞

**要点**：多个设备挂在同一 UART 总线上，主控轮询时按地址逐个请求，每个设备响应自己的帧不冲突。但广播读命令或不受控的定时推送，会导致多个设备同时发包，物理电平叠加产生乱码。

**案例**：尝试用 0xAA 多电机命令广播读取位置 → 四个电机同时回包 → 总线碰撞，数据全错。改为定时返回（各自推送）+ 发配置命令时间隔 2ms，使四个电机的计时起点错开，回包自然不重叠。

**规则**：单 UART 总线上避免广播读命令。定时推送通过时间偏移（发配置时错开 N ms）实现自然串行化。

---

### T7. 外设初始化顺序影响上电行为

**要点**：多个外设共用同一 UART，初始化时若两个外设同时开启 RXNE 中断（一个用 RXNE，另一个用 DMA+IDLE），会产生中断和 DMA 同时接管同一 RDR 寄存器的冲突。上电时线路上的噪声可能触发 RXNE，破坏 HAL 状态机，导致 DMA 接收无法启动。

**案例**：`Telescopic_Init` 和 `POSTION_init` 都在 UART7 上开 RXNE，同时 `POSTION_init` 又用 DMA 接收。上电时 UART7 引脚有噪声，RXNE 抢先触发，HAL 进入错误状态，`Z_SetHeight` 的 `while(BIT != finish)` 永远等不到。删掉 `Telescopic_Init` 里的 RXNE 后解决。

**规则**：同一 UART 只能用一种接收机制（RXNE 中断 OR DMA+IDLE），二选一，不能同时开。

---

### T8. FreeRTOS 任务栈不够时的表现是随机的

**要点**：栈溢出时 FreeRTOS 不会立刻崩溃，溢出的栈帧会覆盖相邻内存（通常是其他任务的 TCB 或全局变量），导致随机的、难以复现的行为。上电不稳定、偶发跑飞、某些参数被随机覆盖，往往是栈不够的征兆。

**案例**：`Start_Task` 栈 128 字节，在加入 `POSTION_init`（含 `HAL_UARTEx_ReceiveToIdle_DMA` 大型 HAL 调用链）后偶发上电不启动。扩到 256 后稳定。

**规则**：初始化函数调链深、HAL 调用多时，任务栈至少 256。调用 `vTaskCreate` 时要考虑最深调用栈的所有局部变量之和。

---

### T9. 控制输出链的顺序错误引入累积误差

**要点**：在底盘控制中，PID 输出、斜率限制、减速上限、死区归零等操作有严格的依存关系。保存"上一帧状态"（如 last_y）必须在所有修正之后，否则下一帧的起点是修正前的值，导致斜率计算失效。

**案例**：`last_w` 在航向死区归零后保存，导致每次 w 进入死区后下一帧从 0 重新斜率计算，航向在 ±DEADZONE 附近产生持续振荡。将 `last_w = w_c_output` 移到死区判断之后解决。

**规则**：所有状态变量（last_y/x/w）必须在整个修正链的最末尾保存。控制输出链顺序：PID → 斜率限制 → 减速上限 → 位置死区 → 角度死区 → 到位判定 → 保存状态 → 发给电机。

---

### T10. 角度误差必须走最短路径

**要点**：角度是循环量（-180° 和 180° 是同一方向）。naive 差值 `target - actual` 在边界处会产生 360° 的虚假误差，导致控制器转一整圈而不是 0°。

**案例**：绝对模式目标角 270°，`normalize_angle(imu.yaw)` = -90°（同一物理方向）。`err_w = |270 - (-90)| = 360°`，控制器认为需要整圈旋转，电机一直转不停。加 while 归一化后解决。

**规则**：所有角度误差计算都必须归一化到 [-180°, +180°]：
```c
while (raw_err_w >  180.0f) raw_err_w -= 360.0f;
while (raw_err_w < -180.0f) raw_err_w += 360.0f;
```

---

## Part 1: 代码审查

### Pre-Review: Gather Context

- Active build target (check `.uvprojx` in MDK-ARM/)
- Recent changes (which files, why)
- Observed symptoms (motor not moving, wrong direction, segmented motion)

### 1. FPU & Cortex-M7 (CRITICAL)

- [ ] `configENABLE_FPU` in FreeRTOSConfig.h must be `1`
- [ ] ARMCC flag `--fpu=fpv5-d16` matches FPU enabled
- [ ] ITCM/DTCM not used for DMA buffers — M7 DTCM is core-coupled, DMA can't reach

### 2. Fault Handlers (CRITICAL)

All four in `stm32f7xx_it.c` are empty `while(1)` → any crash = silent hang.

- [ ] HardFault_Handler should save stacked PC. Recommended trampoline:
```c
void HardFault_Handler(void) {
    __asm volatile(
        "tst lr, #4        \n"
        "ite eq             \n"
        "mrseq r0, msp     \n"
        "mrsne r0, psp     \n"
        "b HardFault_Handler_C \n"
    );
}
void HardFault_Handler_C(uint32_t *stack) {
    volatile uint32_t pc = stack[6];
    volatile uint32_t cfsr = *(volatile uint32_t*)0xE000ED28;
    __BKPT(0);
    while(1);
}
```

### 3. volatile & Shared State

- [ ] `delay_time` (TIM6 ISR) — already `volatile`
- [ ] `motor_check.flag_finish` — ISR writes, task reads. Must be volatile AND accessed atomically
- [ ] `car` struct members shared between tasks — volatile but check atomic access
- [ ] Any `static` variable shared across function calls (e.g., `normalize_angle` had `static flag` bug)

### 4. ISR Safety

- [ ] UART3 DMA idle ISR (`MY_UART3_IRQHandler`) — stops DMA, double-buffer copy, parse. No long blocking
- [ ] TIM6 callback: `++delay_time` only — minimal, good
- [ ] TIM14 is HAL timebase — `HAL_GetTick()` depends on TIM14 ISR
- [ ] NVIC priority grouping 4: `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=5` → only priority>=5 can call FreeRTOS API

### 5. UART3 TX Reliability (CRITICAL)

- [ ] `uart3WriteBuf()` → `HAL_UART_Transmit_DMA()` — **return value never checked**. `HAL_BUSY` = silent frame loss
- [ ] `HAL_UART_Transmit_DMA` is non-blocking — function returns immediately, DMA transfers in background
- [ ] **ALL DMA TX buffers MUST be `static`**: stack buffers are freed when function returns, DMA reads garbage
  - `Send_motor_together` data[4] → static
  - `motor_read_coordination` sendmotor_coordination_data[3] → static
  - `send_speed_data_all` all_send[37] → static
  - If any TX func uses non-static buffer → intermittent garbage on UART
- [ ] `send_speed_data_switch()` — 4 DMA sends with `Delay_ms(4)`. Any failure = that motor gets no command
- [ ] **Single wheel occasionally "half a beat slow"** usually means one motor's `0xF6` speed frame arrived late or was dropped on the old per-motor TX path, so that motor reused the previous cycle's speed
- [ ] Three TX paths conflict under DMA: `Motor_SetZero`, `Send_motor_together`, `send_speed_data_switch`
- [ ] Recommended TX for critical frames (no HAL dependency):
```c
static void uart3_putc(uint8_t c) {
    CLEAR_BIT(huart3.Instance->CR3, USART_CR3_DMAT);
    volatile uint32_t to = 1000000;
    while (!(huart3.Instance->ISR & USART_ISR_TXE) && --to) {}
    huart3.Instance->TDR = c;
}
```

### 6. Emm Firmware Sync Protocol

#### 6.1 Multi-Motor Command (0xAA) — PREFERRED
```
Format: 00 AA [len_hi] [len_lo] | [motor1 8B] [motor2 8B] [motor3 8B] [motor4 8B] | 6B
```
- Header: 4 bytes (00 AA 00 25) — length=0x0025=37
- Body: 4 motor commands × 8 bytes each
- Tail: 1 byte checksum (6B)
- **sync flag still 0x01** — all 4 cached, then `Send_motor_together` triggers simultaneously
- **Advantage**: single DMA transfer, no inter-motor delay, eliminates 4×4ms=16ms spread
- **Buffer must be static**: 37-byte `all_send` buffer

#### 6.2 Speed Command Format (0xF6, 8 bytes)
```
byte[0]: motor address (0x01-0x04)
byte[1]: 0xF6 (command code)
byte[2]: direction (0x00=forward, 0x01=reverse)
byte[3-4]: speed (16-bit, big-endian)
byte[5]: acceleration 0xC8 (200)
byte[6]: sync flag (0x01=cached/同步, 0x00=immediate/立即)
byte[7]: checksum 0x6B
```

#### 6.3 Sync Trigger
```
00 FF 66 6B  — broadcast to all motors
```
- [ ] `sync=0x01` (cached) → requires `Send_motor_together(00 FF 66 6B)` trigger. Trigger fails = all motors stuck
- [ ] `sync=0x00` (immediate) → 4 motors start at staggered times (~12ms spread) → diagonal motion skewed
- [ ] `sync=0x00` more often explains a consistent phase offset between motors, not a single wheel only occasionally lagging one cycle
- [ ] Per manual: "每条命令发送完后，要延时几毫秒，防止命令之间粘包失效"

#### 6.4 Position Command Format (0xFD, 13 bytes)
```
byte[0]: motor address
byte[1]: 0xFD
byte[2]: direction
byte[3-4]: speed (0x2E, 0xE0)
byte[5]: acceleration 0xAE
byte[6-9]: pulse count (32-bit, pulse*14)
byte[10]: mode
byte[11]: sync 0x01
byte[12]: checksum 0x6B
```

### 7. Motor Control State Machine

- [ ] **Double-move**: `chassis_control` calls `Motor_SetZero()` after `finish` → zeroed actual creates fake error → re-drives
- [ ] **Re-trigger**: `Move_To_Target_area` sets `MOTOR_ACTIONFALG = Incomplete` after while loop → re-activates PID
- [ ] `MOTOR_ACTIONFALG = Incomplete` must be BEFORE `Motor_SetZero()`, not after
- [ ] `flag_finish` must be snapshot+cleared atomically in critical section

#### 7.1 UART3 Bus Arbitration (CRITICAL)

Chassis task runs as FreeRTOS task (priority 6) sharing UART3 with Main task (priority 5):
- [ ] During `Motor_SetZero()` in `Move_To_Target_area`, chassis task may still be running and calling `Motor_setspeed(0,0,0)` → writes to UART3 → corrupts reset command
- [ ] **Fix**: `Set_chassis_able(unable)` before `Motor_SetZero()` / `Imu_setZero()`, then `Set_chassis_able(enable)` after
- [ ] `Set_chassis_able` does NOT need `taskENTER_CRITICAL()` — `vTaskSuspend/vTaskResume` are thread-safe
- [ ] `Motor_SetZero()` alone is ~24ms (4 motors × 6ms delay) — long enough for chassis task to interfere multiple times

### 8. Task Priority

- [ ] Chassis prio 6 preempts Main prio 5 during `Motor_SetZero` → reads half-zeroed positions → garbage PID
- [ ] Critical frame sequences need `__disable_irq()` (~400µs, acceptable)
- [ ] Same priority (5) → Main can preempt Chassis during DMA TX → TX conflict

### 9. DMA & HAL Traps

- [ ] DMA buffers must NOT be in DTCM
- [ ] `HAL_UART_Transmit` (blocking) uses `HAL_GetTick()` for timeout → depends on TIM14
- [ ] Direct register TX (polling TXE/TC) independent of HAL tick — preferred for critical frames
- [ ] Before changing any file, verify it's in `.uvprojx` (authoritative for Keil build)
- [ ] UART3 is shared bus (RS485-like topology): ALL commands to motors AND their responses share the same TX/RX lines

### 10. IMU Reset Timing

- [ ] `Imu_setZero()` sends UART command to IMU chip — **IMU needs 100-200ms to complete reset**
- [ ] `Move_To_Target_area` calls `Imu_setZero()` then immediately starts control loop — reads stale yaw
- [ ] Missing `Delay_ms(100)` after `Imu_setZero()` causes:
  - First movement after turn runs with wrong heading reference
  - Path curves mid-route even though final heading is correct
- [ ] `main_task.c` test functions already have this delay: `Imu_setZero(); vTaskDelay(pdMS_TO_TICKS(100));`

### 11. Output Format

```
[SEVERITY: CRITICAL / HIGH / MEDIUM / INFO]
File: path:line
Problem: one-line description
Why: explanation
Fix: concrete change
```

---

## Part 2: 调试流程

### Phase 0: Quick Triage

1. `motor_check.flag_finish` — stuck != 0x0F?
2. `MOTOR_ACTIONFALG` — stuck at finish(1) or Incomplete(0) at wrong time?
3. `motorX.actual_angle` for all 4 motors — partial zero or garbage?
4. `imu.yaw` — stable when stationary? Drifting during motion?
5. `car.actual_x` / `car.actual_y` — check for X-axis drift during straight motion
6. Task timing: Chassis_Control_Task period, priority

### Phase 1: Crash Analysis

Keil debugger:
1. Set hardware breakpoint in `HardFault_Handler`
2. Reproduce crash → read R13 (SP), R14 (LR) from Registers
3. LR: `0xFFFFFFF9` = PSP, `0xFFFFFFF1` = MSP
4. Memory window at SP → 8 stacked words: R0-R3, R12, LR, **PC**, xPSR
5. PC (word 7, offset 0x18) = faulting instruction

`Peripherals > Core Peripherals > Fault Reports`:
- CFSR byte 0 (MemManage), byte 1 (BusFault), byte 2 (UsageFault)
- HFSR, BFAR

| CFSR Bit | Meaning |
|----------|---------|
| NOCP | FPU disabled but code uses floats |
| UNDEFINSTR | PC jumped to data (stack corruption) |
| DIVBYZERO | PID float division by zero |
| IMPRECISERR | Buffer overflow (UART DMA buffers) |

### Phase 2: UART3 TX Debug

**DMA silent frame loss**: `HAL_UART_Transmit_DMA` returns `HAL_BUSY` when gState != READY. Return value unchecked.

Symptoms:
- Alternating motors move (1,3 work; 2,4 don't)
- Wrong direction (only diagonal motors active)
- `sync=0x01` + trigger = nothing moves
- One wheel occasionally lags by one cycle while the other three respond normally → old per-motor TX path likely dropped or delayed exactly one motor's speed frame

Debug:
- Watch `motorX.actual_angle` — which ones change?
- Breakpoint on `Send_motor_together` — is it executing?
- Logic analyzer on UART3 TX to confirm bytes sent

**Isolation test**: `sync=0x00` moves but `sync=0x01` doesn't → TX issue (not logic issue).
If switching to `0xAA` multi-motor TX removes the "single wheel half-beat late" symptom, treat that as strong evidence that the old split TX path had a per-motor timing/loss problem rather than a mechanical mismatch.

### Phase 3: Heading Stability Debug

#### 3.1 Heading Drift During Straight Motion

**Symptom**: path curves mid-way, final heading is correct

**Root cause matrix**:
| Cause | Watch signal | Fix |
|-------|-------------|-----|
| IMU reset not finished | `imu.yaw` jumps after `Imu_setZero` | Add `Delay_ms(200)` after `Imu_setZero` |
| Gyro_Pid Kp too weak | yaw drifts monotonically | Increase Kp (try 5.0 with Kd=0.3) |
| Gyro_Pid Kd too low | yaw oscillates ±1° during motion | Increase Kd (try 3.0, watch for noise amplification) |
| w slew rate too tight | `w_c_output` clipped at 3-5, can't catch up | Increase `MAX_W_DELTA_TRANS` to 8-10 |
| IMU signal noisy at speed | yaw jitter proportional to velocity | Reduce Kd first, then check IMU mounting |

**Diagnosis procedure**:
1. Watch `imu.yaw` at rest → stable? (±0.2°) Yes=IMU OK
2. Watch `imu.yaw` during straight drive → drift or oscillate?
3. Drift = integral needed or Kp↑, Oscillate = Kd↓

#### 3.2 Heading Stuck After Turn

**Symptom**: robot reaches near target heading but oscillates indefinitely, never finishes

**Root cause chain**:
```
err_w = 0.5° (just over HEADING_DEADZONE 0.3°)
→ PID output = 2.6 × 0.5 = 1.3 (< motor minimum)
→ Motor doesn't move, err_w stays at 0.5°
→ settle_count never reaches 2 → timeout after 10s
```

**Fix**: Add MIN_SPEED boost for small PID outputs (see §7.3 below)

### Phase 4: Motor Logic Debug

**Double-move pattern**:
| Step | actual | target | why |
|------|--------|--------|-----|
| Move_To starts | →0 | T | Motor_SetZero |
| chassis drives | 0→T | T | PID |
| chassis finishes | →0 | T | Motor_SetZero in finish block! |
| while exits | 0 | T | MOTOR_ACTIONFALG→Incomplete! |
| chassis re-drives | 0→T | T | Fake error detected |

Fix: Delete `Motor_SetZero()` from finish block AND delete `MOTOR_ACTIONFALG=Incomplete` from end of `Move_To_Target_area`.

**Stuck after first move**: `MOTOR_ACTIONFALG=finish`, `Move_To_Target_area` sets it to `Incomplete` AFTER `Motor_SetZero()`, but chassis preempts DURING `Motor_SetZero()` and sees `finish` → stops. Fix: move `MOTOR_ACTIONFALG=Incomplete` to BEFORE `Motor_SetZero()`.

**Preemption during zeroing** (prio 6 vs 5): Chassis preempts during `Motor_SetZero`, reads half-cleared positions → garbage PID for one cycle. Fix: `__disable_irq()` around 4-byte sequences, OR suspend chassis task before zeroing.

### Phase 5: Report Format

```
## Diagnosis
Fault/Symptom: [description]
Root cause: [identified cause]
Confidence: HIGH / MEDIUM / LOW

## Fix
File: [path:line]
Change: [what to change]

## Next Check
[What to observe after fix]
```

### Vague Symptoms Quick Guide

| Symptom | Check | Common Cause |
|---------|-------|-------------|
| "不动了" | MOTOR_ACTIONFALG, actual_angle[4] | timeout or double-move |
| "斜着走" | all 4 motors moving? sync flag? | DMA frame loss or sync issue |
| "分段走" | double-move pattern | Motor_SetZero in wrong place |
| "直道弯" | imu.yaw during motion | IMU reset timing or PID params |
| "到头卡住" | error vs threshold vs motor output | deadzone gap or settle_count logic |
| "后退弯" | motor2 start timing | UART bus contention or mechanical drag |
| "跑完停不下来" | spd_cap, DECEL_K | deceleration too aggressive → overshoot |

---

## Part 3: Chassis Control Tuning (经验总结)

### 3.1 Fixed-Period Control Loop

Use `vTaskDelayUntil` for fixed 30ms period:
```c
TickType_t last_wake = xTaskGetTickCount();
const TickType_t period = pdMS_TO_TICKS(30);
while (1) {
    vTaskDelayUntil(&last_wake, period);
    chassis_control();
}
```
- [ ] Must be inside while(1), NOT outside
- [ ] Remove `Delay_ms(5)` from inside `chassis_control` once fixed period is set
- [ ] `motor_read_coordination_all()` takes ~4ms — factor into total loop time

### 3.2 Trapezoidal Velocity Profile

Three-stage velocity shaping using two mechanisms:

**Stage 1 — Acceleration limit** (slew rate limiter):
```c
static float last_y = 0, last_x = 0, last_w = 0;
float delta_y = y_c_output - last_y;
delta_y = clamp(delta_y, -MAX_DELTA, MAX_DELTA);
y_c_output = last_y + delta_y;
last_y = y_c_output;   // update AFTER all modifications
```
- MAX_DELTA ~10 per 30ms period → smooth ramp
- Must save `last_y/x/w` AFTER deadzone AND spd_cap modifications

**Stage 2 — Constant speed** (PID naturally saturates)

**Stage 3 — Distance-based deceleration**:
```c
float spd_cap = err * DECEL_K;   // DECEL_K ~0.55-0.8
y_c_output = clamp(y_c_output, -spd_cap, spd_cap);
```
- At far distance: spd_cap > PID output → no limit (constant speed)
- At near distance: spd_cap < PID output → forces deceleration
- DECEL_K too high → late braking, harsh stop
- DECEL_K too low → sluggish motion

### 3.3 w (Heading) Separated Control

Two modes with different slew limits:

| Mode | Limit | When active |
|------|-------|------------|
| TURN | MAX_W_DELTA_TURN (40) | `last_y < threshold && last_x < threshold` |
| TRANS | MAX_W_DELTA_TRANS (5-10) | `last_y > threshold \|\| last_x > threshold` |

```c
float w_delta_limit = (fabs(last_y) > 1.0f || fabs(last_x) > 1.0f) 
                      ? MAX_W_DELTA_TRANS : MAX_W_DELTA_TURN;
```
- Use `last_y/last_x` (actual output), NOT raw PID output — more stable switching
- TRANS limit prevents heading oscillation during translation
- TURN limit allows fast heading correction when stopped/turning

### 3.4 Heading Deadzone & Thresholds

Two independent thresholds:
```c
#define HEADING_DEADZONE     0.3f   // w output goes to zero (control precision)
#define ORIENTATION_THRESHOLD 0.6f   // settle_count accumulates (finish criteria)
```

| err_w range | w behavior | settle_count |
|-------------|-----------|-------------|
| ≤ DEADZONE | w = 0 | can count |
| DEADZONE < err_w ≤ THRESHOLD | PID corrects (no boost) | can count |
| > THRESHOLD | PID corrects | reset to 0 |

**Critical design rules**:
- `Direction_Calibration_turn` deadzone `|w_output|<2→0` should be REMOVED — use chassis_control deadzone exclusively
- If DEADZONE < THRESHOLD: the gap (e.g. 0.3°~0.6°) is where PID outputs are tiny and may cause motor creep. If visible oscillation occurs, set DEADZONE = THRESHOLD to eliminate the gap entirely
- If DEADZONE = THRESHOLD: more stable but less precision — choose based on whether the motor can reliably respond to sub-degree corrections

### 3.5 Motor Deadzone & MIN_SPEED

X42S stepper motors have minimum response velocity:
```c
if(err_w > HEADING_DEADZONE && 
   fabs(w_c_output) > 0 && 
   fabs(w_c_output) < MIN_SPEED)
{
    w_c_output = (w_c_output > 0) ? MIN_SPEED : -MIN_SPEED;
}
```
- MIN_SPEED = 2.0 is sweet spot (low enough to avoid overshoot, high enough to move)
- MIN_SPEED = 5.0 caused overshoot past threshold
- Only apply MIN_SPEED after slew rate limiter (slew output may be smaller than raw PID)

### 3.6 Settle Count (Anti-Bounce)

```c
static uint8_t settle_count = 0;
if (all three errors within thresholds && MOTOR_ACTIONFALG == Incomplete) {
    settle_count++;
    if (settle_count >= 2) MOTOR_ACTIONFALG = finish;
} else {
    settle_count = 0;  // reset on any deviation
}
```
- 2 cycles (60ms) is enough — balances responsiveness vs noise rejection
- 3 cycles may cause false timeout if one axis oscillates at threshold boundary

### 3.7 Position Deadzone

Hard cutoff with slew state reset:
```c
if(err_y <= POSITION_THRESHOLD) { y_c_output = 0; last_y = 0; }
if(err_x <= POSITION_THRESHOLD) { x_c_output = 0; last_x = 0; }
```
- `last_y = 0` is critical — prevents next cycle starting slew from residual value
- Without reset: `last_y = 5`, next cycle PID outputs 3 → delta = -2 → slow deceleration → motor creeps
- Hard cutoff can cause jerk if DECEL_K didn't bring velocity close to 0 first

### 3.8 Finish Branch Cleanup

```c
if(MOTOR_ACTIONFALG == finish) {
    last_y = 0; last_x = 0; last_w = 0;  // reset all slew states
    settle_count = 0;                     // reset settle counter
    Motor_setspeed(0, 0, 0);             // zero speed to motors
    return;                               // skip motor_check.flag_finish=0 below
}
```
- All static state must be reset for next movement
- `return` prevents `motor_check.flag_finish=0` that would trigger stale feedback

### 3.9 Axis Inversion

Minimal change to invert Y-axis in `Move_To_Target_area`:
```c
car.target_y = (-y * ratio_of_pulse_distance_y);   // negative sign inverts Y
```

### 3.10 PID Parameter Guidelines for X42S + IMU

| PID | Kp | Ki | Kd | Output Limit | Notes |
|-----|----|----|----|-------------|-------|
| chassis_pid_y/x | 0.28 | 0.005 | 0 | ±350 | Position control, integrator for steady-state |
| Gyro_Pid (heading) | 2.6 | 0.0 | 0.5 | ±150 | Heading control, Kd critical for oscillation damping |

- **Gyro_Pid Kd > 3.0**: noise amplification → yaw jitter during motion
- **Gyro_Pid Kp > 5.0 without Kd adjustment**: turn overshoot
- **Gyro_Pid Kp < 2.0**: too weak to counter wheel slip during translation
- Adding Ki to Gyro_Pid: dangerous without integral windup protection (integral separation)

### 3.11 control flow checklist (chassis_control)

The output chain in correct order:
```
1. Read actual positions (motor_read_coordination_all)
2. Calculate errors (err_y, err_x, err_w)
3. Raw PID outputs (PID_Compute × 3)
4. Slew rate limit (clamp delta vs MAX_DELTA / w_delta_limit)
5. Distance deceleration cap (spd_cap = err × DECEL_K)
6. Position deadzone (hard cutoff with last_y/x reset)
7. w MIN_SPEED boost (if err_w > HEADING_DEADZONE AND |w| < MIN_SPEED)
8. w deadzone (|w| = 0 if err_w ≤ HEADING_DEADZONE)
9. Settle count / finish detection
10. Save last_y/x/w (AFTER all modifications)
11. Motor_setspeed(actual_output)
```

Any deviation from this order introduces bugs.

### 3.12 Control Period Scaling

When changing control period, **all delta limits MUST scale proportionally**, not just the FreeRTOS tick:

```
new_limit = old_limit × (new_period / old_period)
```

Example — switching from 30ms to 20ms (×0.667):

| Parameter | 30ms value | 20ms value | Effect if NOT scaled |
|-----------|-----------|-----------|---------------------|
| MAX_DELTA | 10 | ~7 | Overshoot on position |
| MAX_W_DELTA_TURN | 40 | ~27 | Turn overshoot → oscillation |
| MAX_W_DELTA_TRANS | 7 | ~5 | Heading overshoot during translation |
| `Delay_ms(5)` in loop | 5ms | **REMOVE** | Waste 25% of period, unstable timing |

- [ ] `Delay_ms(N)` inside `chassis_control` MUST be removed when using `vTaskDelayUntil` — it inflates actual execution time beyond the fixed period
- [ ] `settle_count >= 2` remains 2 cycles regardless of period (time changes but anti-bounce logic doesn't)

### 3.13 MIN_SPEED Bang-Bang & HEADING Oscillation

**Root cause**: MIN_SPEED boost bypasses slew rate limiter, creating a bang-bang control loop at the HEADING_DEADZONE boundary:

```
err_w = 0.35° (just over HEADING_DEADZONE 0.3°)
→ PID output 0.91 → MIN_SPEED jacks to 2.0 (instant jump, no slew)
→ Motor overshoots → err_w = -0.2° (enters deadzone) → w=0
→ err_w drifts back to 0.35° → repeat
→ Result: visible left-right oscillation near target, settle_count never reaches 2
```

This is **especially severe at 20ms** (jump frequency increases by 50%).

**Fix priority**:
1. **Remove MIN_SPEED entirely** (preferred): the settle_count logic already tolerates small residual errors (err_w < ORIENTATION_THRESHOLD). If the motor can't respond to PID values < 2.0, the robot just settles at a sub-degree offset and finish is triggered anyway.
2. **Set HEADING_DEADZONE = ORIENTATION_THRESHOLD**: eliminates the intermediate zone where w=0 but not yet finish
3. **Only enable MIN_SPEED when err_w > 1.0°**: apply boost only for large errors, let small errors decay naturally

### 3.14 Shortest-Path Angle Error (Critical for Absolute Mode)

When using absolute angles (e.g., 180°, 270°), `normalize_angle(imu.yaw)` clamps `actual_w` to ±180°, breaking the naive `|target - actual|` error:

```
actual_w = normalize_angle(270°) = -90°
target_w = 270°
raw_err  = 270 - (-90) = 360°  ← thinks it needs a full circle
```

**Must normalize err_w to ±180°**:

```c
float raw_err_w = car.target_w - car.actual_w;
while (raw_err_w >  180.0f) raw_err_w -= 360.0f;
while (raw_err_w < -180.0f) raw_err_w += 360.0f;
float err_w = __fabs(raw_err_w);
```

- Applies to both relative AND absolute modes (improves relative mode edge cases too)
- `Direction_Calibration_turn` already handles shortest path internally (it's correct)
- But HEADING_DEADZONE check, ORIENTATION_THRESHOLD check, and settle_count ALL use `err_w` — they were broken for angles near ±180° without this fix

---
