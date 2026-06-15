# Robot Project 2024

## 俯视图

```
                   Y+ (电池方向)
                    ↑
        ┌───────────┴───────────┐
        │                       │
        │   4号(RB)      1号(LB) │
        │                       │
        │       ┌─电池─┐       │
        │       └──────┘       │
        │                       │
  X+ ←  │         ┌─关节电机─┐  │  → X-
  (右)  │         │ M8010   │  │  (左)
        │         └─────────┘  │
        │                       │
        │   3号(RF)  [开关] 2号(LF)│
        │                       │
        └───────────┬───────────┘
                    ↓
                   Y- (车头/开关)
```

## 坐标系

**左手坐标系**，Y+ 朝向电池侧（车尾）。

| 轴 | 正方向 | 含义 |
|----|--------|------|
| Y+ | 后 | 电池所在方向 |
| Y- | 前 | 车头/开关方向 |
| X+ | 右 | |
| X- | 左 | 关节电机所在方向 |
| W+ | 逆时针（俯视） | 正角度旋转 |

## 电机编号

| 编号 | 代号 | 位置 |
|------|------|------|
| 1 | LB | 左后 |
| 2 | LF | 左前 |
| 3 | RF | 右前 |
| 4 | RB | 右后 |

`Motor_Send_Speed_together(1号, 2号, 3号, 4号)`

## 关键硬件位置

| 部件 | 位置 |
|------|------|
| 电池 | 1号与4号之间（车尾） |
| 开关 | 2号与3号之间（车头） |
| 关节电机 M8010 | 1号与2号之间（左侧） |
| IMU | 车体中部 |
| 二维码模块 | 车头 UART5 RX |
| 串口屏 | 车头 UART5 TX |

## 串口引脚表

| 串口 | TX 引脚 | RX 引脚 | 波特率 | 连接设备 | DMA |
|------|---------|---------|--------|---------|-----|
| USART1 | PA9 | PA10 | 115200 | 调试/printf 输出 | 无 |
| USART2 | PD5 | PD6 | 115200 | IMU（陀螺仪） | RX: DMA1_Stream5 |
| USART3 | PC10 | PC11 | 115200 | 底盘电机 ×4 (X42S) | RX: DMA1_Stream1 / TX: DMA1_Stream4 |
| UART5 | PC12 | PD2 | 115200 | 串口屏(TX) / 二维码(RX) | 无 |
| USART6 | PC6 | PC7 | 9600 | 树莓派视觉模块 | 无 |
| UART7 | PE8 | PE7 | 115200 | 升降/伸缩电机 (Y/Z轴) | RX: DMA1_Stream3 |
| UART8 | PE1 | PE0 | 4000000 | 关节电机 GO-M8010-6 (RS485) | 无 |


> UART8 波特率 4Mbps，需要 SYSCLK(216MHz) 作为时钟源。
> RS485 方向控制引脚（DE/RE）见 `gpio.c` `SET_485_DE_UP` / `SET_485_RE_UP` 宏。

## Move_To_Target_area 参数

```c
Move_To_Target_area(x, y, angle, enable, Relative_Position);
```

| 参数 | 含义 | 备注 |
|------|------|------|
| x | X 方向位移（mm） | 正值 = 右移 |
| y | Y 方向位移（mm） | 正值 = **向电池方向移动（Y+）**，负值 = 向车头方向移动 |
| angle | 目标旋转角（度） | 相对模式下为相对角度 |

> **y 轴说明**：`motor_control.c` 中 `car.target_y = (-y * ratio)`，调用时传正值即为向前（Y+），与坐标系定义一致。

## 视觉系统（树莓派）

代码路径：`C:\Users\LuoXue\Desktop\Robot Project\Vision Project\gongxun_cv\raspberry_pi\`

| 文件 | 功能 |
|------|------|
| `color_line_det.py` | 色块检测 + 串口发送，使用 `/dev/ttyAMA0`，波特率 9600 |
| `track.py` | HSV 调参工具，摄像头实时预览 |

树莓派串口配置（`color_line_det.py`）：
```python
ser = serial.Serial(
    port="/dev/ttyAMA0",
    baudrate=9600,
    ...
)
```

## 树莓派 ↔ 主控通信测试

最快测试方法：树莓派直接往串口发字节，主控用 USART1（调试口）或任意空闲串口接收打印确认。

**树莓派侧（一行测试脚本）：**
```python
import serial, time
ser = serial.Serial('/dev/ttyAMA0', 9600)
while True:
    ser.write(b'123')
    time.sleep(0.5)
```

**主控侧**：在对应 UART 的 RX 中断里 `printf` 打印收到的字节，用 USART1 输出到 PC 串口助手查看。

> 注意：树莓派 `/dev/ttyAMA0` 默认被蓝牙占用，需在 `/boot/config.txt` 加 `dtoverlay=disable-bt` 并重启才能用于通用串口。
