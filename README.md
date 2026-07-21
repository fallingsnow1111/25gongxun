# Robot Project 2024

STM32F750V8 + FreeRTOS 全向轮机器人工程。当前系统由开环速度底盘、机械臂作业、MaixCam Pro 视觉和串口屏状态监视组成。

## 当前方案

| 部分 | 当前实现 |
|------|----------|
| 主控 | STM32F750V8Tx，FreeRTOS |
| 底盘 | X42S 步进电机 ×4，UART3 DMA，开环速度控制 |
| 航向 | IMU 绝对偏航角闭环保持 |
| 机械臂 | GO-M8010-6 关节电机 + Y/Z 轴步进电机 + 夹爪舵机 |
| 视觉 | MaixCam Pro，物料/色环三色同时识别 |
| 任务码 | XR1503MTEX 二维码模块 |
| 人机界面 | 淘晶驰 TJC 串口屏，独立 FreeRTOS 任务 |

## 底盘坐标与电机

跑图接口使用的方向定义：

| 参数 | 正值方向 | 负值方向 |
|------|------------|------------|
| `vx` | 左移 | 右移 |
| `vy` | 前进 | 后退 |
| `target_angle` | 逆时针 | 顺时针 |

| 电机ID | 代号 | 位置 |
|--------|------|------|
| 1 | LB | 左后 |
| 2 | LF | 左前 |
| 3 | RF | 右前 |
| 4 | RB | 右后 |

## 开环底盘控制

底盘驱动器内部加速度设为0，所有速度曲线由主控按5ms周期生成。四轮速度通过 `0xAA` 多电机帧打包，再用广播触发帧同步执行。

直线运动使用正弦加减速，并在运动中根据IMU偏航角保持车头方向。软件里程计按四轮实际发送的整数RPM与实际保持时间积分，单位为 `RPM*ms`，`60000` 软件脉冲等于电机理论旋转一圈。

当前实车标定系数：

```c
#define CHASSIS_LONGITUDINAL_PULSE_PER_CM 2478.5f /* 前进、后退 */
#define CHASSIS_LATERAL_PULSE_PER_CM      2560.5f /* 左移、右移 */
```

每段路径结束后可读取四个驱动器的实际位置脉冲，用于比较软件积分误差；该反馈只用于标定和调试，不参与跑图闭环。

### 直线跑图

```c
Chassis_MoveByDistance(vx, vy, target_angle, distance_cm);
```

- 仅用于前后或左右单轴直线，`distance_cm` 始终传正数。
- 速度由 `vx` 或 `vy` 的绝对值决定。
- 视觉定位微调使用低速速度控制，不调用距离接口。

示例：

```c
Chassis_MoveByDistance(40, 0, 0, 10);    /* 左移10cm */
Chassis_MoveByDistance(0, 80, 0, 20);    /* 前进20cm */
Chassis_MoveByDistance(0, 160, 0, 80);   /* 前进80cm */
Chassis_MoveByDistance(0, -80, -90, 20); /* 保持-90°航向后退20cm */
```

当前距离接口在40cm以下使用80 tick加减速，40cm以上使用100 tick加减速。

当加速和减速均为100 tick时，速度20/40/80/160完成加减速分别约需4/8/16/32cm。最大速度可按下式估算：

```text
speed_max = distance_cm * pulse_per_cm / (ramp_ticks * 5ms)
```

### 原地转向

```c
Chassis_TurnToAngle(target_angle, timeout_ms);
```

当前已验证的转向策略：

- 角度PID输出最大150RPM。
- 转向增速限制为每5ms增加2RPM。
- 转向减速限制为每5ms减少4RPM。
- PID输出小于2RPM时补偿为 `+2/-2RPM`，克服电机与整数RPM死区。
- 角度误差进入 `+/-0.1°` 后停车，连续10个5ms周期满足才判定到位。

实车连续90°转向已验证可收敛，停车500ms后的静态误差保持在0.2°以内。

## MaixCam Pro 视觉

视觉代码：

```text
C:\Users\LuoXue\Desktop\Robot Project\Vision Project\gongxun_cv\maxicam\maixcam.py
```

STM32通过USART6发送ASCII命令：

| 命令 | 模式 |
|------|------|
| `'0'` | 空闲 |
| `'1'` | 红、绿、蓝物料同时识别 |
| `'2'` | 红、绿、蓝色环同时识别 |
| `'L'` | 打开辅助照明LED |
| `'l'` | 关闭辅助照明LED |

MaixCam Pro返回多目标帧：

```text
55 5B mode count cls1 x1 y1 ... AA
```

STM32将红、绿、蓝目标分别缓存在 `vision_material[]` 和 `vision_ring[]`中。高层代码使用：

```c
Vision_StartMaterial();
Vision_StartRing();
Vision_Stop();
Vision_LED_On();
Vision_LED_Off();
Vision_GetMaterialTarget(color, &target);
Vision_GetRingTarget(color, &target);
```

## 机械臂安全约束

- Z轴电机ID为1，Y轴伸缩电机ID为2。
- 圆盘机夹取、色环识别和色环放置的关节作业角均为 `-180°`。
- 从仓库取料时，必须先上升到安全高度，再旋转关节。
- `claw_move_1()` 用于圆盘机夹取、色环识别和色环区物料回收。
- `claw_move_2()` 用于从仓库取料并放到色环区。

## 串口分配

| 串口 | 波特率 | 设备 | 用途 |
|------|--------|------|------|
| USART2 | 115200 | IMU | DMA姿态接收 |
| USART3 | 115200 | X42S ×4 | 底盘同步速度命令与调试脉冲读取 |
| UART5 | 115200 | TJC串口屏 / 二维码 | TX更新屏幕，RX接收任务码 |
| USART6 | 9600 | MaixCam Pro | 视觉模式命令与多目标坐标 |
| UART7 | 115200 | Y/Z轴电机 | DMA接收到位帧 |
| UART8 | 4000000 | GO-M8010-6 | RS485关节电机控制 |

## 串口屏

HMI任务每100ms刷新固定状态：

| 控件 | 内容 |
|------|------|
| `t0` | 二维码结果，识别后锁定 |
| `t1` | 固定标题 |
| `t2` | 车身航向和世界坐标 |
| `t3` | 软件脉冲积分 |
| `t4` | 视觉像素误差或5ms周期状态 |
| `t5` | 驱动器实际脉冲与软件积分误差 |
| `t6-t8` | 3行滚动日志 |

## 当前流程入口

测试函数统一放在 `APP/test.c`，`task/main_task.c` 只保留一行当前测试或流程入口。

当前流程分段：

```c
Route_Path1_StartToQR();
Flow_QRRecognize();
Route_Path2_QRToTurntable();
Flow_TurntableCatch();
Route_Path3_TurntableToProcessing();
Flow_ProcessingArea();
```

当前完整入口：

```c
Flow_RunCurrent();
```

`task/main_task.c` 目前用于转向误差测试，正式跑图前需将调用切换为需要的路径段或 `Flow_RunCurrent()`。

## 编译

工程使用 Keil MDK-ARM。代码修改后由用户在Keil中手动编译和烧录。


当前第二层 6 个二维码逻辑：
123：1仓红普通放红 → 2仓绿普通放绿 → 3仓蓝逆时针 -32° 取料放蓝
132：1仓红普通放红 → 2仓蓝普通放蓝 → 3仓绿逆时针 -32° 取料放绿
213：1仓绿普通放绿 → 2仓红普通放红 → 3仓蓝逆时针 -32° 取料放蓝
231：1仓绿普通放绿 → 2仓蓝特殊避障，取到蓝并 Z 上升后、旋转到蓝角前先伸 Y → 3仓红逆时针 -32° 取料放红
312：1仓蓝普通放蓝 → 2仓红普通放红 → 3仓绿逆时针 -32° 取料放绿
321：1仓蓝普通放蓝 → 2仓绿普通放绿 → 3仓红正常取料 -394°，再放红
