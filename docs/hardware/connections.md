# 当前硬件与接线

整理日期：2026-10-20；以下实物与接线确认来自 2026-10-19 记录。本页汇总当前方案，[状态表](../status.md) 单独说明哪些已实测通过，[历史记录](../history/2026-10-19-hardware-bringup.md) 保留原始测试和更正。

## 硬件

| 部件 | 已确认型号 / 配置 |
| --- | --- |
| K230 | CanMV-K230-LP4 V3.0；脚本读取固件 v1.8-0-gc2d1f5c，MicroPython 1.21；CanMV IDE K230 4.0.9 |
| 舵机 | 飞特 ST-3215-C018，ID 1～6，1,000,000 bps，转换板 URT-2 |
| 底盘主控 | 立创天猛星 MSPM0G3507 |
| 电机 / 驱动 | MG513XP28_12V ×2、现有 TB6612 双路稳压驱动板；详细引脚见 [电机例程接线](../../device/mspm0/motor_encoder/使用说明.md#1-接线) |
| 调试器 | 野火 fireDAP，CMSIS-DAP / SWD |

机械臂零位、方向和限位读取 [关节映射](../../configs/arm/joint_mapping.json)，不按 ID 数字推断。全部 17 个小车硬件原始资料保留在 [car/硬件信息](../../hardware/car)。

## 两种机械臂连接方式

### 现有 PC 辅助抓取

Linux PC 通过 USB OTG 与 K230 通信；K230 负责图像特征，PC 通过另一个 USB 串口转接板控制舵机。使用 [Linux 运行指南](../guides/linux-arm.md)。下方 UART1 直连接法是另一条已完成只读联调的路线，不能仅靠换线就运行当前 PC 程序。

### K230 UART1 直连 URT-2（只读已验证）

| K230 V3.0 | JP1 物理脚号 | URT-2 UART 排针 |
| --- | --- | --- |
| IO_3 / GPIO3 / UART1 TX | 18 | TX |
| IO_4 / GPIO4 / UART1 RX | 13 | RX |
| GND | 按丝印 | GND |
| 5V | 按丝印 | 5V 逻辑供电脚 |

- 此 URT-2 官方 MCU 接法为 **TX—TX、RX—RX**；不是普通两块 MCU 的 TX→RX 交叉接法。
- 电平开关置 **3V3**；K230 的 5V 线给转接板逻辑供电，信号仍匹配 3.3V，不将其改接成 5V UART。
- 此直连模式下 **URT-2 USB 拔除**，K230 OTG USB 可连 IDE。舵机独立供电并共地，舵机接 TTL-Bus。
- GPIO 编号不同于物理脚号，优先看 IO_3 / IO_4 丝印。测试舵机前拆除 IO_3↔IO_4 回环线。
- UART1 回环使用 115200/8N1；舵机使用 1 Mbps。两项测试的波特率不同。

断电后调整接线。该链路已读到六舵机，不代表 K230 原生运动或自主抓取已验收。

## K230 UART2 ↔ MSPM0 UART1

两板各自供电，仅互连以下三根通信线，115200 bps / 8N1 / 无流控：

| K230 V3.0 | JP1 物理脚号 | MSPM0 天猛星 |
| --- | --- | --- |
| IO_5 / GPIO5 / UART2 TX | 17 | A09 / PA09 / UART1 RX |
| IO_6 / GPIO6 / UART2 RX | 20 | A08 / PA08 / UART1 TX |
| GND | 按丝印 | GND |

用户已确认接线；尚未完成双板通信验证。IO_5 与 IO_3 同排，IO_6 与 IO_26 同排。不要互连两板的 5V/3V3 输出。MSPM0 UART0 / PA10、PA11 为另一条日志链路，不等于 UART1。

## MSPM0 供电与 SWD

| fireDAP 信号 | 天猛星引脚 | 用户确认的板端线色 |
| --- | --- | --- |
| VREF | 3V3 | 黑 |
| TCK / SWCLK | CLK / PA20 | 紫 |
| TMS / SWDIO | DIO / PA19 | 白 |
| GND | GND | 灰 |

VREF 感知目标电压，不承担目标板供电。天猛星 Type-C 输入为稳定 5V，MCU/逻辑为 3.3V；本轮没有收到万用表实测电压。电池不得直接接 Type-C、5V 或 3V3。

RST/NRST 为需要硬件复位 / Connect under Reset 时的可选线，未接本身不能解释 E310。当前 SWD 无应答原因未定。用户已确认实际信号对应；此前根据照片推断误接 5V/TX/RX 的判断已撤回，不再作为故障结论。

Keil 配置、下载步骤和失败记录分别见 [UART 工程说明](../../device/mspm0/uart_link/README.md)、[历史记录](../history/2026-10-19-hardware-bringup.md)。

## 整车方案边界

现有草案为 11.1V / 5000mAh / 10C 电池经分线、稳压分别供舵机、电机与控制板。该草案尚非整车带载供电验收结果。电机例程的 PA12 / STBY 接法及车轮悬空操作要求沿用 [原使用说明](../../device/mspm0/motor_encoder/使用说明.md)。

小车行驶时机械臂保持 [准备位和扭矩](../guides/arm/准备位脚本.md)。当前没有自动控制联动或持物行驶策略。
