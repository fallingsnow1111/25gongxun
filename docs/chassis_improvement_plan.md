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
