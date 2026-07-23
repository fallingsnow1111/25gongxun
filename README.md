# Robot Project 2024

STM32F750V8 + FreeRTOS 全向轮机器人固件。系统包括 X42S 四轮底盘、三轴机械臂、GO-M8010-6 关节电机、MaixCam Pro 视觉、二维码模块和 TJC 串口屏。

## 硬件概览

| 部分 | 当前实现 |
|------|----------|
| 主控 | STM32F750V8Tx + FreeRTOS |
| 底盘 | X42S 步进电机 x4，USART3 DMA，同步速度协议 |
| 航向 | IMU 绝对偏航角，USART2 DMA 接收 |
| 机械臂 | GO-M8010-6 关节电机 + Y/Z 步进轴 + 双夹爪舵机 |
| 视觉 | MaixCam Pro，物料/色环红绿蓝多目标识别 |
| 二维码 | XR1503MTEX，UART5 RX |
| 人机界面 | TJC 串口屏，UART5 TX，独立 FreeRTOS 任务 |

## 当前入口

FreeRTOS 启动后由 `task/start_task.c` 完成外设和任务初始化，再创建主任务。当前主任务位于 `task/main_task.c`，正式运行入口为：

```c
Flow_RunCurrent();
```

`Flow_RunCurrent()` 会先调用 `Flow_ArmPoseInit()` 初始化机械臂姿态，然后执行两轮完整比赛流程。

## 核心代码结构

| 路径 | 作用 |
|------|------|
| `task/start_task.c` | 系统启动、外设初始化，创建 HMI、IMU 和主任务 |
| `task/main_task.c` | 正式流程或单项测试的唯一入口 |
| `APP/task_flow.c` | 路径、扫码、圆盘机夹取、粗加工区、暂存区和返航主流程 |
| `APP/action_control.c` | 机械臂基础动作：识别姿态、仓库取料、色环放置 |
| `APP/catch.h` | 机械臂高度、伸出长度、仓位角度和特殊避障参数 |
| `APP/warehouse_app.c` | 二维码颜色顺序与三个物料仓的映射 |
| `APP/test.c` / `APP/test.h` | 可独立运行的调试和标定函数 |
| `MOTOR/motor.c` | X42S 通信、同步速度帧和软件里程计 |
| `MOTOR/motor_control.c` | 底盘直线移动、转向、航向保持和世界坐标累计 |
| `MOTOR/postion_control.c` | 机械臂 Y/Z 轴位置控制及到位等待 |
| `MOTOR/imu_control.c` | 航向角归一化、角度误差和漂移补偿 |
| `SENSOR/GO-M8010-6.c` | RS485 关节电机控制 |
| `SENSOR/IMU.C` | IMU DMA 接收和数据解析 |
| `SENSOR/QR_code.c` | 二维码接收、解析和结果显示 |
| `SENSOR/circe.c` | MaixCam 串口协议、物料/色环目标缓存 |
| `SENSOR/tjc_usart_hmi.c` | TJC 串口屏命令与日志接口 |
| `task/hmi_task.c` | 串口屏周期刷新任务 |
| `task/imu_task.c` | IMU 周期解析任务 |

主流程直接调用 `APP/action_control.c` 和 `APP/task_flow.c` 中的同步动作接口。

## 调用关系

```text
Start_Task
  -> Init_Task_Create
  -> HMI_Task_Create
  -> IMU_Task_Create
  -> Main_Task_create
       -> Flow_RunCurrent
            -> 路径移动
            -> 视觉定位
            -> 机械臂动作
            -> 下一阶段
```

底盘和机械臂动作大多是同步接口：函数返回时，该段动作已经完成或已经超时。视觉串口接收、IMU 解析和 HMI 刷新由独立任务或中断持续运行。

## 完整比赛流程

第一轮：

```text
机械臂姿态初始化
-> 启停区到二维码区
-> 扫码并装载第一轮颜色顺序
-> 到圆盘机夹取三件物料
-> 到粗加工区放置并回收
-> 到暂存区第一层放置
-> 返回圆盘机
```

第二轮：

```text
装载第二轮颜色顺序
-> 圆盘机夹取三件物料
-> 到粗加工区放置并回收
-> 到暂存区第二层放置
-> 返回启停区
```

主要阶段函数：

| 函数 | 作用 |
|------|------|
| `Route_Path1_StartToQR()` | 启停区到二维码区 |
| `Flow_QRRecognize()` | 等待二维码并装载第一轮顺序 |
| `Route_Path2_QRToTurntable()` | 二维码区到圆盘机 |
| `Flow_TurntableCatch()` | 按当前轮次顺序定位并夹取三色物料 |
| `Route_Path3_TurntableToProcessing()` | 圆盘机到粗加工区 |
| `Flow_ProcessingArea()` | 第一层放置并回收三件物料 |
| `Route_Path4_ProcessingToNext()` | 粗加工区到暂存区 |
| `Flow_StorageArea()` | 暂存区第一层或第二层放置 |
| `Route_Path5_StorageToTurntable()` | 第一轮暂存区返回圆盘机 |
| `Route_Path6_StorageToHome()` | 第二轮暂存区返回启停区 |

## 当前定位保护

- 色环定位总时限为 9 秒。
- 色环定位只使用当前颜色的新有效帧；丢帧时底盘停车。
- 连续 2 秒没有当前颜色帧时，以速度 20 后移 1 cm，再返回搜索开始时的锚点。
- 搜索移动过程中出现当前颜色帧会立即停止搜索并恢复校准。
- 色环定位超过 9 秒后跳过定位，直接执行放料并继续流程。
- 暂存区第二层物料定位时限为 8 秒；超时后跳过剩余定位，放完三件后正常返航。
- 航向姿态精调最多持续 5 秒，不会无限重试。

## 坐标与底盘接口

| 参数 | 正方向 | 负方向 |
|------|--------|--------|
| `vx` | 左移 | 右移 |
| `vy` | 前进 | 后退 |
| `target_angle` | 逆时针 | 顺时针 |

四个底盘电机的协议地址和安装位置：

| 电机 ID | 代码代号 | 安装位置 |
|---------|----------|----------|
| 1 | LB | 左后 |
| 2 | LF | 左前 |
| 3 | RF | 右前 |
| 4 | RB | 右后 |

常用接口：

```c
Chassis_MoveByDistance(vx, vy, target_angle, distance_cm);
Chassis_MoveByPulse(vx, vy, target_angle, target_pulse, ramp_ticks);
Chassis_TurnToAngle(target_angle, timeout_ms);
Chassis_FineTuneAngle(target_angle, timeout_ms);
Chassis_OpenLoop_SetTranslation(vx, vy, target_angle);
```

当前距离标定：

```c
#define CHASSIS_LONGITUDINAL_PULSE_PER_CM 2478.5f
#define CHASSIS_LATERAL_PULSE_PER_CM      2560.5f
```

`Chassis_MoveByDistance()` 的距离参数始终传正数，方向由 `vx` 或 `vy` 的符号决定。视觉微调使用低速开环平移接口，不调用阻塞式距离接口。

### 开环速度与里程计

- X42S 速度帧使用 `0xF6` 命令，四个电机先通过 `0xAA` 多机帧发送，再广播同步触发。
- 驱动器速度帧中的加速度字段为 0，正弦加减速由主控按 5 ms周期生成。
- `Motor_setspeed()` 用于普通整数 RPM 指令；`Motor_setspeed_fine()` 用于 0.1 RPM 分辨率的转向精调。
- 软件里程计按每个车轮的 `RPM x 10` 指令和实际保持时间积分，内部基础量为带符号的 `RPM x 10 x ms`。
- 对外的 `CHASSIS_ODOM_T` 将积分量换算为底盘 `x/y/w` 软件脉冲；`60000` 个 `RPM x ms` 对应电机理论旋转一圈。
- 每段运动结束后会读取四个驱动器的实际位置脉冲，用于与软件积分比较；该反馈目前用于标定和显示，不参与路径闭环。
- `Chassis_WorldBeginSegment()` 清零本段里程，`Chassis_WorldCommitSegment()` 按该段航向把车体位移累计到 `car.actual_x/y`，单位为 mm。

### 直线移动

`Chassis_MoveByDistance()` 只接受前后或左右单轴直线，不能同时传入非零 `vx` 和 `vy`。当前自动加减速配置：

| 距离 | 加速 | 减速 |
|------|------|------|
| 小于 40 cm | 80 tick = 400 ms | 80 tick = 400 ms |
| 大于等于 40 cm | 100 tick = 500 ms | 100 tick = 500 ms |

当加速和减速均为 100 tick 时，速度 20/40/80/160 完成完整加减速约需 4/8/16/32 cm。最大合适速度可近似估算：

```text
speed_max = distance_cm * pulse_per_cm / (ramp_ticks * 5ms)
```

示例：

```c
Chassis_MoveByDistance(40, 0, 0, 10);      /* 左移 10 cm */
Chassis_MoveByDistance(0, 80, 0, 20);      /* 前进 20 cm */
Chassis_MoveByDistance(0, 160, 0, 80);     /* 前进 80 cm */
Chassis_MoveByDistance(0, -80, -90, 20);   /* 保持 -90 度后退 20 cm */
```

### 原地转向

当前 `CHASSIS_TURN_USE_DOUBLE_LOOP` 为 `0`，原地转向使用单角度 PID 实验模式。代码中仍保留双环参数，切换宏为 `1` 时才启用角度外环和角速度内环。

| 参数 | 当前值 |
|------|--------|
| 到位角度误差 | `+/-0.1` 度 |
| 连续到位次数 | 7 个 5 ms周期 |
| 最小转向输出 | 2 RPM |
| 最大转向输出 | 150 RPM |
| 每 5 ms最大加速变化 | 2 RPM |
| 每 5 ms最大减速变化 | 4 RPM |

`Chassis_TurnToAngle()` 返回 `1` 表示在超时前稳定到位，返回 `0` 表示超时。目标角可以使用连续世界航向，例如 `-270`、`-360`；内部角度误差会归一化到最短旋转方向。

## 机械臂约束

- 圆盘机夹取、色环识别和色环放置的安全关节角为 `PUT_AND_CATCH_ANGLE`，当前值为 `-180` 度。
- 从物料仓取料后，必须先升到 `CIRCLE_SAFE_HEIGHT`，再旋转关节。
- `claw_move_1()` 用于圆盘机夹取和色环区物料回收。
- `claw_move_2()` 用于物料仓取料和色环区放置。
- 高度、长度和仓位角统一在 `APP/catch.h` 中调整。

当前主要机械参数：

| 参数 | 当前值 | 作用 |
|------|--------|------|
| `YUAN_PAN_HEIGHT` | 83 | 圆盘机夹取高度 |
| `YUAN_PAN_DETECT_HEIGHT` | 20 | 圆盘机识别高度 |
| `PUT_HOUSE_HEIGHT` | 55 | 放入物料仓高度 |
| `CIRCLE_DETECT_HEIGHT` | 60 | 色环识别高度 |
| `CIRCLE_PLACE_HEIGHT` | 155 | 粗加工区/暂存区第一层放置高度 |
| `CIRCLE_SECOND_LAYER_HEIGHT` | 93 | 暂存区第二层放置高度 |
| `CIRCLE_WAREHOUSE_HEIGHT` | 68 | 从物料仓夹取高度 |
| `CIRCLE_SECOND_LAYER_AVOID_LENGTH` | 95 | 第二层 231 蓝色避障伸出长度 |

### 第二层二维码动作

二维码三位数字依次表示 1、2、3 号仓内的颜色，其中 `1=红`、`2=绿`、`3=蓝`。程序按二维码顺序建立颜色与仓位映射，再把三件物料放到各自颜色位置。

| 第二轮码 | 当前第二层动作 |
|----------|----------------|
| `123` | 1 仓红、2 仓绿、3 仓蓝，均走普通流程 |
| `132` | 1 仓红、2 仓蓝、3 仓绿，均走普通流程 |
| `213` | 1 仓绿、2 仓红、3 仓蓝，均走普通流程 |
| `231` | 1 仓绿普通；2 仓蓝取料抬升后先伸 Y=95 再旋转；3 仓红使用连续角 `-394` 取料 |
| `312` | 1 仓蓝、2 仓红、3 仓绿，均走普通流程 |
| `321` | 1 仓蓝、2 仓绿普通；3 仓红使用连续角 `-394` 取料 |

## 视觉

视觉硬件为 MaixCam Pro。视觉程序位于：

```text
C:\Users\LuoXue\Desktop\Robot Project\Vision Project\gongxun_cv\raspberry_pi\color_line_det.py
```

HSV 阈值调试工具：

```text
C:\Users\LuoXue\Desktop\Robot Project\Vision Project\gongxun_cv\raspberry_pi\track.py
```

STM32 端通过 USART6 控制识别模式：

| 命令 | 作用 |
|------|------|
| `'0'` | 停止识别 |
| `'1'` | 识别红、绿、蓝物料 |
| `'2'` | 识别红、绿、蓝色环 |
| `'L'` | 打开辅助照明 |
| `'l'` | 关闭辅助照明 |

多目标数据分别缓存在 `vision_material[]` 和 `vision_ring[]`。高层通过 `Vision_GetMaterialTarget()` 和 `Vision_GetRingTarget()` 读取当前帧目标。

MaixCam Pro 返回的多目标二进制帧：

```text
55 5B mode count cls1 x1 y1 cls2 x2 y2 cls3 x3 y3 AA
```

- `mode=1` 表示物料，`mode=2` 表示色环。
- `count` 最大为 3。
- `cls=1/2/3` 分别表示红/绿/蓝。
- 坐标字节可能等于 `0xAA`，因此接收端按 `count` 计算完整帧长度，不以遇到第一个 `0xAA` 作为帧结束。
- 每收到一帧先清空对应模式缓存，再写入本帧目标，避免沿用已经离开画面的旧目标。

## 测试函数

所有公开测试入口声明在 `APP/test.h`，实现在 `APP/test.c`。

| 测试函数 | 作用 | 主要观察结果 |
|----------|------|--------------|
| `QR_Code_Test()` | 持续等待二维码并显示解析结果 | 第一轮、第二轮三位颜色码 |
| `Vision_Parse_Test()` | 启动物料识别并打印三色目标缓存 | 模式、目标数量、各颜色坐标 |
| `Chassis_Turn_Error_Test()` | 依次转到多个目标角，测量停车后的静态误差 | 目标角、最终角、误差、耗时 |
| `Chassis_AngularRate_Baseline_Test()` | 测量转向到位后的角度和角速度稳定性 | 平均/最大角度误差和残余角速度 |
| `Chassis_TurnRate_Map_Test()` | 标定转向 RPM 指令与实际角速度关系 | IMU 角速度、Yaw 反算角速度 |
| `Chassis_LowSpeed_Linearity_Test()` | 验证正反向 1 RPM 精细速度是否可执行 | 平均角速度、方向、最低有效转速 |
| `Chassis_Integral_Turn_Test()` | 不使用角度闭环，按速度和时间积分估算转角 | 积分角、IMU 实测角和两者误差 |
| `IMU_Static_Stability_Test()` | 静止 60 秒测量 IMU 漂移 | 10 秒检查点、峰峰值、漂移斜率 |
| `IMU_Drift_Rezero_Test()` | 每 10 秒重新归零，共测试 6 段 | 每段漂移和平均角速度 |
| `IMU_Stable_Straight_Test()` | IMU 稳定后锁定 0 度直行 100 cm | 路线弯曲、最终航向误差 |
| `Ring_Warehouse_Clearance_Test()` | 依次从 1/2/3 号仓取料并按第一层高度放置 | 仓位角、机械间隙、放置高度 |
| `Ring_Location_Test()` | 循环定位绿、蓝、红色环并按固定距离切换 | 像素收敛、航向保持、环间距 |
| `Ring_LocateOne_Place123_Test()` | 定位一次绿环，再按固定姿态放置 1/2/3 号仓物料 | 定一次放三件的角度和位置 |
| `Yuanpanji_Warehouse1_RingPlaceHeight_Test()` | 任意物料进入圆盘机 ROI 后夹取到 1 号仓，再从 1 号仓取出并按第一层高度放置 | 圆盘机夹取高度、仓库高度、第一层放置高度 |
| `Arm_Pose_Init_Test()` | 单独运行正式流程使用的机械臂姿态初始化 | Y/Z 零点、关节零点和初始化报错 |
| `Claw_Calibration_Test()` | 循环执行两个夹爪的打开/关闭位置 | 舵机安装位置和 PWM 行程 |

### 切换测试

在 `task/main_task.c` 的 `Main_Task()` 中只保留一个入口。例如：

```c
static void Main_Task(void *pvParameters)
{
    (void)pvParameters;

    Ring_Location_Test();

    while(1)
        vTaskDelay(pdMS_TO_TICKS(200));
}
```

正式跑图时恢复为：

```c
Flow_RunCurrent();
```

一次只运行一个测试函数。多数测试函数内部会持续循环或在结束后停住，不能和完整流程同时调用。

## 串口分配

| 串口 | 波特率 | 设备 | 作用 |
|------|--------|------|------|
| USART2 | 115200 | IMU | DMA 姿态接收 |
| USART3 | 115200 | X42S 四轮底盘 | 同步速度命令和位置脉冲读取 |
| UART5 TX | 115200 | TJC 串口屏 | 状态与日志显示 |
| UART5 RX | 115200 | XR1503MTEX | 二维码接收 |
| USART6 | 9600 | MaixCam Pro | 视觉模式命令和多目标坐标 |
| UART7 | 115200 | Y/Z 轴驱动器 | 位置命令和到位帧 |
| UART8 | 4000000 | GO-M8010-6 | RS485 关节电机控制 |

## 串口屏

HMI 固定状态每 100 ms刷新一次，日志通过队列滚动显示。

| 控件 | 当前显示 |
|------|----------|
| `t0` | 二维码结果，识别后锁定 |
| `t1` | 底盘监控标题 |
| `t2` | 世界航向及 `car.actual_x/y`，位置单位 mm |
| `t3` | 当前运动段的软件里程 `x/y/w` |
| `t4` | 有视觉目标时显示像素误差；否则显示周期超限、最大周期、发送失败和运动时间 |
| `t5` | 四轮 RPM x10 指令、机械臂故障或驱动器实际脉冲比较 |
| `t6`-`t8` | 三行滚动日志 |

关键状态优先使用 `HMI_LogInfo/Warn/Error()` 和 `HMI_SetPixelError()` 输出。高频 ISR/DMA 状态不应持续刷屏，应使用 Watch 变量观察。

## 编译

工程文件为 `MDK-ARM/MDK.uvprojx`，使用 Keil MDK-ARM 编译和烧录。

出现编译错误时，先查看：

```text
MDK-ARM/MDK/MDK.build_log.htm
MDK-ARM/MDK/MDK.htm
```

不要根据 `.o`、`.d`、`.crf` 或 `.lst` 等历史产物判断当前源码状态。
