# 硬件与接线

## 硬件清单

| 部件 | 型号 / 配置 |
| --- | --- |
| 视觉 | CanMV-K230-LP4 V3.0；固件 v1.8-0-gc2d1f5c、MicroPython 1.21；CanMV IDE K230 4.0.9 |
| 机械臂 | 飞特 ST-3215-C018 ×6，ID 1～6，1 Mbps；URT-2 转接板 |
| 底盘 | WHEELTEC R3X 两驱三轮底盘，MG513XP28_12V ×2，TB6612 双路稳压驱动板 |
| 主控 / 调试器 | 立创天猛星 MSPM0G3507 / 野火 fireDAP |
| 现有电池 | WHEELTEC E326S，标签 10.8 V、2550 mAh、最大持续放电 6 A；充电上限 12.6 V |
| 尚未接入 | 八路巡线模块 LineFollower_8CH V1.0、ICM-42688 V1.2 |

机械臂零位、方向和限位见 [关节映射](../../configs/arm/joint_mapping.json)。原始手册和照片在 [hardware/car](../../hardware/car)。轮径先采用采购资料标称65 mm；减速比及每圈计数待核对对应电机型号资料，轮距尚未确定。标称值不记作实测。

## 现有 PC 抓取

PC 分别通过 USB 连接 K230 OTG 和 URT-2；K230 提取视觉特征，PC 控制舵机。操作见 [Linux 指南](../guides/linux-arm.md)。下列 UART 直连是另一测试接法。

## K230 UART1 直连舵机

| K230 V3.0 | JP1 物理脚号 | URT-2 排针 |
| --- | --- | --- |
| IO_3 / UART1 TX | 18 | TX |
| IO_4 / UART1 RX | 13 | RX |
| GND | 按丝印 | GND |
| 5V | 按丝印 | 5V 逻辑供电 |

URT-2 的官方 MCU 接法为 **TX—TX、RX—RX**。电平开关置 **3V3**，拔除 URT-2 USB，舵机独立供电并共地、接 TTL-Bus。GPIO 编号不是物理脚号；测试前拆除 GPIO3/4 回环线。舵机 1 Mbps，回环测试 115200/8N1。

## K230 UART2 ↔ MSPM0 UART1

两板各自供电，只接以下三线，115200/8N1、无流控，不互连 5V/3V3 输出。

| K230 V3.0 | JP1 物理脚号 | MSPM0 |
| --- | --- | --- |
| IO_5 / UART2 TX | 17 | A09 / PA09 / UART1 RX |
| IO_6 / UART2 RX | 20 | A08 / PA08 / UART1 TX |
| GND | 按丝印 | GND |

IO_5 与 IO_3 同排，IO_6 与 IO_26 同排。MSPM0 UART0 的 PA10 TX / PA11 RX 另用于 115200/8N1 日志。

## MSPM0 SWD

| fireDAP | 天猛星 | 已确认线色 |
| --- | --- | --- |
| VREF | 3V3 | 黑 |
| TCK / SWCLK | CLK / PA20 | 紫 |
| TMS / SWDIO | DIO / PA19 | 白 |
| GND | GND | 灰 |

主控由自身 Type-C 供电；VREF 是电压参考，不是供电输出。RST 可接已确认的 NRST，用于硬件复位或 Connect under Reset。下载设置见 [板端联调](../guides/k230-and-mspm0.md)。

## 电机与供电

2026-10-20只读采样确认：用户只手转左轮时A路计数大幅变化，B路基本稳定，因此 **A=左轮、B=右轮**。车头按用户确认的此前电机测试前进方向；该方向下原始编码计数A增加、B减少。当前驱动方向设置两路均为 `MOTOR_*_REVERSE=0`；此处只记录对应关系，未改动驱动方向或计数符号。

电机控制引脚及验收见 [底盘例程](../../device/mspm0/motor_encoder/README.md#接线)。驱动板手册标称输入 4.5～15 V，单路持续 ≤1.2 A、峰值 ≤3.2 A（不可持续）。主控由 USB 供电时，不再接驱动板的 5V/3V3 输出，只共地；电池不得直接接主控 Type-C、5V 或 3V3。

整车分路稳压供电尚未带载验收。此前的 11.1 V / 5000 mAh / 10C 是方案草案，不是上表现有电池参数。行驶姿态见 [准备位](../guides/arm/准备位脚本.md)。
