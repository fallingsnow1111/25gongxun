# 底盘控制系统改进方案

## 现状

- 位置式 PID + 开环速度：外环位置 PID 输出直接作为速度命令发给电机，无速度反馈
- 电机位置数据靠轮询（motor_read_coordination_all），每次占用约 8ms，UART3 负担重
- IMU 仅解析偏航角（yaw），角速率未使用
- 航向控制为单环比例，抗扰能力弱

---

## 改进目标

1. 电机数据获取：轮询 → 定时推送，释放 UART3 总线
2. 底盘控制：单环位置 PID → 串级双环（位置外环 + 速度内环）
3. 航向控制：角度单环 → 角度/角速率串级双环
4. IMU：增加 angular_rate 解析，独立任务定时更新

---

## 一、电机数据获取改造（UART3 定时返回）

### 原理

X42S 支持"定时返回信息命令"（功能码 0x11 辅助码 0x18），配置后电机按设定周期自动推送位置数据，不需要主控轮询。

命令格式：
```
[Addr] [0x11] [0x18] [0x36] [time_hi] [time_lo] [0x6B]
```
- 0x36：返回实时位置
- time：定时间隔（毫秒），0x0000 表示停止

### 配置方案（4 个电机，10ms 推送，错开 2ms）

```c
void Motor_TimedReturn_Init(void)
{
    static uint8_t cmd[7] = {0, 0x11, 0x18, 0x36, 0x00, 0x0A, 0x6B}; // 10ms

    cmd[0] = 0x01;  uart3WriteBuf(cmd, 7);  Delay_ms(2);
    cmd[0] = 0x02;  uart3WriteBuf(cmd, 7);  Delay_ms(2);
    cmd[0] = 0x03;  uart3WriteBuf(cmd, 7);  Delay_ms(2);
    cmd[0] = 0x04;  uart3WriteBuf(cmd, 7);
}

void Motor_TimedReturn_Stop(void)
{
    static uint8_t cmd[7] = {0, 0x11, 0x18, 0x36, 0x00, 0x00, 0x6B}; // 停止
    for (uint8_t id = 1; id <= 4; id++) {
        cmd[0] = id;
        uart3WriteBuf(cmd, 7);
        Delay_ms(2);
    }
}
```

发送命令时间间隔 2ms → 四个电机从不同时刻开始计时 → 回包自然错开 2ms，115200 波特率下单帧 0.69ms，不碰撞。

### 接收处理改动（motor.c）

在现有帧解析后增加速度差分计算：

```c
// 在 U3_process_single_frame 解析 actual_angle 之后
static uint32_t last_tick[5] = {0};
static float prev_angle[5] = {0};

uint32_t now = HAL_GetTick();
uint32_t dt_ms = now - last_tick[id];
if (dt_ms > 0) {
    motorX.actual_speed = (motorX.actual_angle - prev_angle[id]) / (float)dt_ms;
}
prev_angle[id] = motorX.actual_angle;
last_tick[id] = now;
```

### chassis_control 改动

删除 `motor_read_coordination_all()`，改为检查数据新鲜度：

```c
// 原来
motor_read_coordination_all();
if (motor_check.flag_finish < 0x0F) { ... return; }

// 改后
if (HAL_GetTick() - last_motor_update_tick > 30) {
    Motor_setspeed(0, 0, 0);
    return;
}
```

### Relative_Position 模式与定时返回的冲突处理（motor_control.c）

底盘任务挂起期间电机硬件定时器仍在推送，会覆盖刚清零的 actual_angle。需在复位前后停止/恢复定时返回：

```c
if(mode == Relative_Position)
{
    Set_chassis_able(unable);
    Motor_TimedReturn_Stop();    // 停止推送，避免覆盖清零值
    Motor_SetZero();
    Imu_setZero();
    Delay_ms(200);
    Motor_TimedReturn_Init();    // 重新配置定时返回（含 2ms 错开）
    Set_chassis_able(enable);
}
```

---

## 二、IMU 任务独立化 + 角速率解析

### 新增 IMU 任务（5ms，优先级 7）

```c
void IMU_Task(void *pvParameters)
{
    TickType_t last_wake = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(5));
        // 从 DMA buffer 解析 yaw 和 angular_rate
        IMU_Parse();
    }
}
```

### 新增 angular_rate 字段

在 `IMU.h` 中扩展结构体：
```c
struct Imu {
    float yaw;
    float roll;
    float pitch;
    float angular_rate;   // 新增：Z 轴角速率（deg/s 或 rad/s，按 IMU 协议）
};
```

### IMU_Parse 函数

在现有 yaw 解析基础上增加角速率解析（具体偏移量参照 IMU 协议文档）。

---

## 三、串级控制架构

### 控制流（chassis_control，20ms）

```
// 平移轴
target_y - actual_y → chassis_pid_y（外环，位置）→ desired_vy
desired_vy - actual_vy → chassis_pid_vy_inner（内环，速度）→ y_cmd

target_x - actual_x → chassis_pid_x（外环，位置）→ desired_vx
desired_vx - actual_vx → chassis_pid_vx_inner（内环，速度）→ x_cmd

// 旋转轴
target_w - actual_w → Gyro_Pid（外环，角度）→ desired_w_rate
desired_w_rate - imu.angular_rate → chassis_pid_w_inner（内环，角速率）→ w_cmd

Motor_setspeed(y_cmd, x_cmd, w_cmd)
```

### 新增 PID 结构体（chassis_control_task.c）

```c
struct PIDstruct chassis_pid_vy_inner;   // y 速度内环
struct PIDstruct chassis_pid_vx_inner;   // x 速度内环
struct PIDstruct chassis_pid_w_inner;    // 角速率内环
```

### 初始化参数（待调）

```c
PID_Init(&chassis_pid_vy_inner, 0.5f, 0.01f, 0, 350.0f, -350.0f);
PID_Init(&chassis_pid_vx_inner, 0.5f, 0.01f, 0, 350.0f, -350.0f);
PID_Init(&chassis_pid_w_inner,  1.0f, 0.0f,  0, 150.0f, -150.0f);
```

---

## 四、涉及文件修改清单

| 文件 | 改动内容 |
|------|---------|
| `MOTOR/motor.h` | MOTO_DATA 增加 `actual_speed` 字段 |
| `MOTOR/motor.c` | 增加 Motor_TimedReturn_Init/Stop；接收回调增加 actual_speed 差分计算 |
| `MOTOR/motor_control.c` | Relative_Position 分支加 Motor_TimedReturn_Stop/Init 对 |
| `task/chassis_control_task.c` | 删 motor_read_coordination_all；增加内环 PID 结构体和计算；数据新鲜度检查；世界坐标累积和反变换 |
| `SENSOR/IMU.C` | 增加 angular_rate 解析（待查协议文档确认字节偏移） |
| `SENSOR/IMU.h` | 扩展 Imu 结构体增加 `angular_rate` 字段 |
| `task/start_task.c` | START_TASK_STACK 128 → 256；增加 Motor_TimedReturn_Init 调用；增加 IMU_Task 创建 |
| `mydefinition/Struct_encapsulation.h` | CARDATA_T 可选扩展 world_x/world_y（或放文件作用域） |

> **注意**：angular_rate 字节偏移需对照实际 IMU 型号协议文档，实现前确认。

---

## 五、改造收益

| 指标 | 当前 | 改后 |
|------|------|------|
| UART3 读占用时间 | ~8ms/20ms (40%) | 0ms（定时推送）|
| 底盘控制有效时间 | ~12ms/20ms | ~20ms |
| 底盘控制周期 | 20ms | 10ms |
| 航向控制 | 角度单环，易震荡 | 角度+角速率串级，阻尼好 |
| 加减速 | slew rate 开环限速 | 速度内环闭环跟踪 |
| 数据更新率 | 20ms（轮询） | 10ms（推送） |
| 路径规划 | 体坐标，先转再走 | 世界坐标，直接给路点 |

---

## 六、调参顺序建议

1. 先完成定时返回改造，验证 actual_speed 数据是否正确
2. 接入 angular_rate，验证 IMU 任务数据稳定
3. 先调角速率内环（w 轴），参数简单效果明显
4. 再调速度内环（x/y 轴），参考角速率内环经验
5. 外环参数基本不用大改（位置 PID 逻辑不变）
6. 接入世界坐标变换后，验证 Route_Test_ABS 坐标是否等效
7. 验证 L 型运动（边走边转）效果

---

## 七、世界坐标系变换

### 为什么需要

当前底盘编码器积分在**车体坐标系**，转弯后 `actual_y` 的物理方向随朝向变化。路径规划必须手动拆分"先转再走"，且同一坐标在不同朝向下物理含义不同。

加入世界坐标变换后，控制器统一在**世界坐标系**工作，路径规划只需给路点坐标，无需关心当前朝向。

### 实现（chassis_control_task.c）

**1. 扩展状态变量（CARDATA_T 结构体或文件作用域）**

```c
// 累积世界坐标（以出发原点为 (0,0)）
static float world_x = 0.0f;
static float world_y = 0.0f;
static float prev_body_y = 0.0f;
static float prev_body_x = 0.0f;
```

**2. 每帧更新世界坐标（在 Motor_Action_Calculate_actual 之后）**

```c
float delta_body_y = car.actual_y - prev_body_y;
float delta_body_x = car.actual_x - prev_body_x;
prev_body_y = car.actual_y;
prev_body_x = car.actual_x;

float yaw_rad = imu.yaw * (float)M_PI / 180.0f;
world_y += delta_body_y * cosf(yaw_rad) + delta_body_x * sinf(yaw_rad);
world_x += -delta_body_y * sinf(yaw_rad) + delta_body_x * cosf(yaw_rad);
```

**3. 误差计算改为世界系，再反变换到体系给 PID**

```c
float err_wy = car.target_y - world_y;   // target_y 改为世界坐标
float err_wx = car.target_x - world_x;

// 旋转回体坐标系
float err_body_y =  err_wy * cosf(yaw_rad) + err_wx * sinf(yaw_rad);
float err_body_x = -err_wy * sinf(yaw_rad) + err_wx * cosf(yaw_rad);

// 用体坐标误差做 PID（控制律不变）
y_c_output = PID_Compute(&chassis_pid_y, err_body_y, 0);
x_c_output = PID_Compute(&chassis_pid_x, err_body_x, 0);
```

**4. 到位判定改用世界系误差**

```c
float err_y = fabsf(err_wy);
float err_x = fabsf(err_wx);
```

**5. 清零时同步清世界坐标**

在 `Obimeter_SetZero` 里加：

```c
world_x = 0.0f;
world_y = 0.0f;
prev_body_y = 0.0f;
prev_body_x = 0.0f;
```

### 对路径规划的影响

改完后 `Move_To_Target_area` 的 x/y 参数变为世界坐标，不再依赖朝向：

```c
// 改前：必须先转再走，y=1600 含义随朝向变化
Move_To_Target_area(0, 0, 90°);       // 纯转
Move_To_Target_area(0, 1750, 0°);     // 纯平移

// 改后：直接给世界坐标，可以同时转和走
Move_To_Target_area(1750, 1100, 90°); // 控制器自动处理坐标变换
```

### 计算开销

每周期新增：2 次 `sinf/cosf`（STM32F7 FPU 硬件，~14 周期/次）+ 6 次乘加 ≈ 200ns，完全可忽略。

### 误差累积

纯世界坐标积分会随时间漂移，但 IMU 航向实时修正旋转矩阵，主要误差来源是轮子打滑。比赛全程约 2-3 分钟，估计漂移 < 2cm，可接受。

---

## 八、L 型运动（田字形地图专项优化）

### 场景描述

比赛地图为田字形，可行路径为田字的笔画，转弯点均为 90° 直角。当前方案在每个转角处必须先停车，再转向，再加速，耗时约 1-2 秒/个转角。

L 型运动目标：机器人在接近转角时，边减速边转向，完成角度对齐后直接加速驶入下一条直线，全程不停车。

### 实现思路

在路径规划层加混合逻辑，不需要修改底盘控制器本身（世界坐标变换上线后，控制器已支持同时走 x/y + 转向）。

**触发条件**：剩余距离 < 混合起始阈值 `BLEND_DIST`（建议 200-300mm）

```c
// Route_Test_ABS 中替换原来的 "先转再走" 为 L 型路点
// 例：从 (0,1600,0°) 走 L 型到 (1750,1100,90°)

// 原来（两步）：
Move_To_Target_area(0, 1100, 0°);      // 先退到拐角
Move_To_Target_area(0, 1100, 90°);     // 再转向
Move_To_Target_area(1750, 1100, 90°);  // 再平移

// 改后（一步，控制器自动处理）：
Move_To_Target_area(1750, 1100, 90°);  // 世界坐标直给，同时走完 y 方向剩余 + 转向 + 走 x 方向
```

世界坐标控制器在接到 `(1750, 1100, 90°)` 目标后，会同时输出 y 方向减速、旋转加速、x 方向加速的合成命令，自然形成 L 型轨迹。

### 减速区混合参数（chassis_control_task.c）

现有 `spd_cap = err * DECEL_K` 已经处理了减速，世界坐标下两个轴同时参与减速：

```c
float spd_cap_y = fabsf(err_wy) * DECEL_K;
float spd_cap_x = fabsf(err_wx) * DECEL_K;
```

两轴独立减速，各自收敛，自然实现 L 型轨迹而不需要额外逻辑。

### 预期效果

| 指标 | 当前（停车转弯） | L 型运动 |
|------|--------------|---------|
| 单个转角耗时 | ~1.5s | ~0.5s |
| 田字一圈（4 个转角） | 节省约 4s | - |
| 路径平滑度 | 有停顿 | 连续 |

### 注意事项

- L 型运动依赖世界坐标变换（第七节）上线后才有效
- 旋转和平移同时进行时，角速度内环（第三节）是保证走直的关键
- 建议先在直线和单个 L 型转角上验证，再应用到完整路线

---

## 九、2026-06-23 现场待处理记录

### 9.1 圆盘机抓完后底盘不继续走

**现象**：`yuan_pan_catch()` 抓完 3 个物料后，后续粗加工区路线不执行，车停住。

**已验证定位**：
- 原流程在进入圆盘机抓取前调用 `Set_chassis_able(unable)`。
- 抓取结束后即使补了 `Set_chassis_able(enable)`，现场仍出现不继续走。
- 临时去掉挂起底盘后，抓完 3 个物料可以继续执行后续路线。
- 因此问题集中在“挂起/恢复底盘任务后的状态链”，不是路线坐标本身。

**明日优先排查**：
1. `Set_chassis_able(enable)` 后 `Chassis_Control_Task` 是否真的恢复运行。
2. 第一条后续 `Move_To_Target_area(...)` 执行时，Watch：`MOTOR_ACTIONFALG`、`motor_check.flag_finish`、`car.target_y/x/w`、`car.actual_y/x/w`。
3. 如果 `target` 更新但 `actual` 不变，优先查底盘任务是否恢复、UART3 电机反馈是否回来。
4. 如果 `motor_check.flag_finish` 长期不是 `0x0F`，底盘控制会持续停车返回，需要先恢复反馈链。

**临时试验代码**：
```c
// 只用于定位，不作为最终方案
// Set_chassis_able(unable);
yuan_pan_catch();
// Set_chassis_able(enable);
```

**最终方向**：不要长期靠“不挂起底盘”规避问题。最终应恢复“抓取/复位时隔离底盘任务”的设计，但恢复后需要补齐底盘任务、反馈标志、状态机的重同步。

### 9.2 树莓派视觉偶发启动失败

**现象**：
```text
VIDEOIO(V4L2:/dev/video0): can't open camera by index
Camera error
System shutdown
```

**已验证定位**：
- `/dev/video0` 存在，`v4l2-ctl --list-devices` 能识别 USB 摄像头。
- `v4l2-ctl -d /dev/video0 --all` 能正常读取参数，摄像头节点可用。
- `fuser /dev/video0` 显示占用进程是自己的 Python 视觉程序。
- `strace -p <pid>` 能看到持续 `VIDIOC_DQBUF/QBUF`，说明进程实际在持续取帧。
- `xclock` 能弹窗，X11 显示链正常。
- 因此优先按软件问题处理：启动时序、重复打开摄像头、失败分支误判、GUI 显示路径，而不是优先怀疑线松。

**明日优先排查**：
1. 搜索树莓派实际运行脚本：
   ```bash
   grep -n "VideoCapture\|isOpened\|Camera error\|System shutdown\|imshow\|waitKey" /home/swae/Desktop/robot/color_line_det.py
   ```
2. 检查是否有多个 `cv.VideoCapture(0)` 或多个线程重复打开 `/dev/video0`。
3. 摄像头打开失败不要立即退出，先加 3~5 次重试，每次间隔 0.5s。
4. 主循环中加低频打印确认是否跑到 `imshow`。

**复现时必须抓取**：
```bash
fuser /dev/video0
v4l2-ctl -d /dev/video0 --all
strace -p <pid>
echo $DISPLAY
```

---

## 十、下一步视觉定位调试计划

### 10.1 原 `User_function_final()` 的绿色圆环定位思路

原流程在 `task/main_task.c` 的 `User_function_final()` 中，整体结构是：

1. 底盘先走到目标区域附近。
2. 到达视觉定位点后，临时 `Set_chassis_able(unable)`，避免底盘任务和视觉/机械臂动作互相干扰。
3. 调用视觉定位或抓放函数。
4. 定位/抓放完成后 `Set_chassis_able(enable)`，继续后续路线。

绿色圆环定位原来预留的调用是：
```c
Set_chassis_able(unable);
Circle_Position_Center_SPEED(GREEN_CIRCLE);
Set_chassis_able(enable);
```

注意：现在已经验证 `Set_chassis_able(unable/enable)` 这条链存在恢复风险，调绿色圆环时可以先不挂起底盘做定位试验；等定位逻辑稳定后，再回头修挂起恢复问题。

### 10.2 `Circle_Position_Center_SPEED(GREEN_CIRCLE)` 的工作方式

代码位置：`MOTOR/circle_control.c`

核心逻辑：
1. `Set_Circle_Center(113, 123)` 设置绿色圆环图像中心。
2. `send_NX(GREEN_CIRCLE)` 通过 USART6 通知视觉端切到绿色圆环识别模式。
3. 视觉端回传偏差，STM32 侧通过 `change_x / change_y` 获取。
4. `PID_Compute(&Pid_Circle_Positioning, change, 0)` 把视觉偏差转成底盘速度。
5. `Motor_setspeed(x_valspeed, y_valspeed, 0)` 直接让底盘微调。
6. 当 `change_x == 0 && change_y == 0` 时连续停车并退出。

最小测试调用建议：
```c
static void Green_Ring_Position_Test(void)
{
    Set_chassis_able(enable);
    vTaskDelay(pdMS_TO_TICKS(500));

    Circle_Position_Center_SPEED(GREEN_CIRCLE);

    Motor_setspeed(0, 0, 0);
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}
```

调试时 Watch：
- `change_x`
- `change_y`
- `Pid_Circle_Positioning`
- `car.actual_x`
- `car.actual_y`
- `motor_check.flag_finish`

### 10.3 圆盘机定位追踪调试顺序

不要一开始就调动态物料。按两步走：

1. **机械臂追踪静态物料**
   - 圆盘机不转或物料静止。
   - 视觉只需要稳定输出静态物料偏差。
   - 先验证机械臂能根据视觉偏差正确调整 `Y_SetLength`、`Z_SetHeight`、`M8010_SetAngle` 或夹爪位置。
   - 成功标准：同一静态物料多次识别，机械臂最终位置稳定，夹取成功率稳定。

2. **机械臂追踪动态物料**
   - 静态追踪稳定后，再让圆盘机运动。
   - 先低速动态，再逐步提高速度。
   - 重点看视觉延迟、串口回传延迟、机械臂动作延迟三者叠加后的跟随误差。
   - 成功标准：物料运动时偏差能收敛，不出现机械臂追过头或来回摆。

阶段切换原则：静态物料没有调稳之前，不进入动态追踪。否则动态误差会把视觉阈值、串口延迟、机械臂控制三个问题混在一起，难以定位。

---

## 十一、串口屏系统信息与主任务日志方案

目标：在 TJC 串口屏上显示机器人运行状态，例如当前阶段、主任务 log、关键 Watch 变量、任务状态、CPU/任务占用率等。

### 11.1 现有条件

- UART5 已用于串口屏 TX，接口在 `SENSOR/tjc_usart_hmi.c`：
  - `tjc_send_txt(obj, "txt", text)`
  - `tjc_send_val(obj, "val", val)`
- UART5 RX 同时接二维码模块，`SENSOR/QR_code.c` 中使用 UART5 RXNE 中断解析扫码数据。
- 当前 `printf` 重定向在 `Core/Src/usart.c`，目标是 `huart1`，但本板没有把 USART1 实际拉出接口，现场基本不可用。
- `FreeRTOSConfig.h` 当前只确认开启了 `INCLUDE_uxTaskPriorityGet`；任务列表、运行时间统计还没确认开启。

### 11.2 不建议直接把所有 printf 重定向到 UART5

原因：
- TJC 串口屏不是普通终端，发送格式必须是 `控件.属性=值 + 0xFF 0xFF 0xFF`。
- 直接把 printf 原样发到 UART5，串口屏不一定能显示，还可能把屏幕指令流打乱。
- UART5 RX 还承担二维码数据，日志刷太快会增加串口中断和主任务负担。
- 主任务 log 如果在控制环里高频发送，会影响实时性。

结论：对这块板子来说，串口屏几乎就是唯一外部文本出口；但仍不建议把所有 `printf` 直接生硬重定向到 UART5。更稳的做法是新增一个“串口屏日志/状态层”，只把筛选后的关键信息低频发送到 TJC。

### 11.3 推荐显示内容分层

**第一阶段：低风险状态显示**
- 当前阶段：扫码、去原料区、圆盘机抓取、去粗加工区、绿色圆环定位等。
- 关键变量：
  - `MOTOR_ACTIONFALG`
  - `motor_check.flag_finish`
  - `car.target_y / car.target_x / car.target_w`
  - `car.actual_y / car.actual_x / car.actual_w`
  - `change_x / change_y`
- 最近一条主任务 log，例如 `"yuan catch done"`、`"green ring start"`。

**第二阶段：任务状态**
- 显示已知任务是否运行：
  - `Main_Task`
  - `Chassis_Control_Task`
  - `Imu_Task`
  - `Action_sets_Task`
- 可先只显示任务阶段和心跳计数，不急着上完整 FreeRTOS 任务列表。

**第三阶段：CPU/任务占用率**
- 需要开启 FreeRTOS 运行时统计：
  - `configUSE_TRACE_FACILITY`
  - `configUSE_STATS_FORMATTING_FUNCTIONS`
  - `configGENERATE_RUN_TIME_STATS`
- 这一步改动较大，后做。
- 不建议直接用 `vTaskList()` / `vTaskGetRunTimeStats()` 高频刷屏，因为 `sprintf` 和统计遍历开销较大。

### 11.4 建议接口设计

新增轻量接口，优先走 HMI 状态层，而不是让业务代码到处直接 `printf`：

```c
void HMI_Log(const char *msg);
void HMI_SetStage(const char *stage);
void HMI_UpdateDebug(void);
```

示例：

```c
HMI_SetStage("GREEN_RING");
HMI_Log("start");

Circle_Position_Center_SPEED(GREEN_CIRCLE);

HMI_Log("done");
```

`HMI_UpdateDebug()` 由低频任务或主循环手动调用，建议 200ms~500ms 刷新一次，不要放在 20ms 底盘控制环里。

### 11.5 TJC 页面建议

屏幕上预留几个文本控件：

| 控件 | 内容 |
|------|------|
| `t0` | 当前阶段/大状态 |
| `t1` | 最近一条 log |
| `t2` | `MOTOR_ACTIONFALG` + `flag_finish` |
| `t3` | `target_y/x/w` |
| `t4` | `actual_y/x/w` |
| `t5` | `change_x/change_y` |
| `t6` | 任务心跳或 CPU 信息 |

发送示例：

```c
tjc_send_txt("t0", "txt", "GREEN_RING");
tjc_send_txt("t1", "txt", "vision ok");
tjc_send_txt("t2", "txt", "flag=0x0F act=0");
```

### 11.6 最小落地步骤

1. 保留 `__io_putchar` 现状不动，避免牵一发而动全身；板上调试信息先通过 HMI 状态层输出。
2. 新增 `HMI_Log()`，内部只调用 `tjc_send_txt("t1", "txt", msg)`。
3. 在 `Main_Task` 关键节点手动打 log：
   - 扫码开始/结束
   - 圆盘机抓取开始/结束
   - 绿色圆环定位开始/结束
   - 去粗加工区开始/结束
4. 新增 `HMI_UpdateDebug()`，低频刷新关键变量。
5. 如果后面确实想让 `printf` 也能上屏，单独提供一个受控接口，例如 `hmi_printf()`，内部做限频、截断和固定目标控件映射，不要直接替换全局 `printf`。
6. 等基础日志稳定后，再考虑 FreeRTOS 任务列表和 CPU 占用率。

### 11.7 风险与规则

- 不要在 ISR 中直接刷新串口屏。
- 不要在 `chassis_control()` 20ms 控制环里高频刷新屏幕。
- 不要把所有 `printf` 直接重定向到 UART5；UART5 是屏幕协议通道，不是普通调试终端。
- 日志字符串要短，避免 TJC 控件显示不下或 UART5 占用时间太长。
- 如果后续接入 FreeRTOS 统计，先只在静止调试时打开，确认不会影响控制周期。
- 如果必须做“类似 printf 的屏上输出”，也要走受控 `hmi_printf()`，限制频率、长度和目标控件，不能让任意模块直接往 UART5 灌原始文本。

## 十二、2026-06-23 现场现象补充

### 12.1 绿色圆环识别与光照

已观察到的现象：

- 打光时绿色圆环难以稳定识别。
- 关掉灯光后，绿色圆环识别率明显提高，识别成功率很高。

当前判断：

- 先按视觉侧光照/曝光/颜色阈值问题处理，不优先怀疑串口或 STM32 侧解析。
- 后续调绿环定位时，先固定光照条件；如果必须开灯，需要重新标定绿色阈值或调整摄像头曝光/白平衡。

后续检查项：

- 在树莓派/Nano 端记录开灯、关灯时的 HSV/二值化效果。
- 确认 `send_NX(GREEN_CIRCLE)` 后视觉端确实进入绿环模式。
- STM32 侧 Watch `COLOR_DATA`、`change_x`、`change_y`，区分“未识别”和“识别后坐标抖动”。

### 12.2 圆盘机夹取：debug 与直接上电差异

已观察到的现象：

- debug 模式下圆盘机夹取动作很快。
- 直接上电跑完整流程时，夹取前后停顿比较久，导致圆盘机物料可能已经转走。

当前判断：

- 先按软件时序差异排查，不优先怀疑机械硬件。
- 重点检查 `yuan_pan_catch()` 中的固定延时，以及 `Z_SetHeight()` / `Y_SetLength()` 等阻塞等待是否在直接上电时等到超时。

后续检查项：

- Watch `Z_POSTION.BIT`、`Z_POSTION.NOW`、`Z_POSTION.TARGE`，确认 Z 轴动作是否已经到位但 `BIT` 没变 `finish`。
- 对比 debug 与直接上电时，`Z_SetHeight(YUAN_PAN_HEIGHT)`、`Z_SetHeight(0)` 的实际返回时间。
- 如果 Z 轴完成反馈不稳定，先修完成标志或缩短抓取阶段超时，再调动态圆盘机追踪。
