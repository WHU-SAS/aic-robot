# 机械臂硬件与 UART1 联调记录

> 历史记录：保留当日测试、失败过程与判断更正。当前接线见 [硬件说明](../hardware/connections.md)，当前进度见 [状态表](../status.md)；当时的 Windows 路径和 COM 号不作为其他电脑的默认设置。

记录日期：2026-10-19。后续处理同一套硬件时，先查阅本文件；已确认的型号、ID 和波特率无需重复询问。若实物、固件或接线改变，再核对并更新。

本记录关注 **K230 UART1 直连 URT-2** 这条链路。仓库 `car-arm/` 中既有抓取工程的运行状态与参数，仍以其 README、AGENTS 和配置文件为准；本次 UART1 的验证范围不代表整个机械臂工程的验证范围。

## 用户确认的舵机参数

| 项目 | 值 | 依据 |
| --- | --- | --- |
| 舵机型号 | **飞特 ST-3215-C018** | 用户于记录日期明确提供 |
| 舵机数量 | 6 只 | 用户提供 ID 范围 |
| 总线 ID | **1、2、3、4、5、6** | 用户提供；2026-10-19 已由 K230→URT-2 链路逐一 Ping 和读取核验 |
| 舵机通信波特率 | **1,000,000 bps（1 Mbps）** | 用户明确提供 |
| 转换板 | **URT-2** | 用户实物照片确认 |

ID 与具体关节的对应关系、零位、方向、机械限位不由 ID 数字推断；控制已有抓取工程时读取其配置。舵机电源额定范围应查本完整型号的资料，本记录未据其他 ST-3215 变体代填。

## K230 与开发环境

| 项目 | 已确认信息 |
| --- | --- |
| 开发板实物型号 | **CanMV-K230-LP4 V3.0** |
| 运行板型 | `k230_canmv_v3p0 with K230` |
| 实际 CanMV 固件 | **v1.8-0-gc2d1f5c**，构建日期 `2026-07-03` |
| MicroPython | `sys.implementation.version = (1, 21, 0)` |
| 系统 | `rt-smart` |
| 可用模块 | `machine.UART`、`machine.FPIOA` 均存在；`board` 模块不存在 |
| 本机 IDE | CanMV IDE K230 **4.0.9** |
| 本机安装路径 | `D:\CanMV-IDE` |
| 本机配置目录 | `D:\CanMV-IDE\settings`，通过快捷方式参数 `-settingspath` 指定 |
| Windows 连接记录 | K230 在本次测试中为 **COM8**，USB VID/PID `1209:abd1`；端口号可能随电脑和连接变化 |
| PC 连接方式 | K230 的 **USB OTG** 接口连接电脑，用于 IDE 通信 |

IDE 状态栏曾显示“固件版本 0.4.0”，但开发板脚本实际返回 v1.8；以实际返回信息为准。原 AppData 配置路径持续报“文件不可读”，改用上述 D 盘配置目录后，退出保存和从桌面快捷方式重启已通过验证。

本次读取的版本原文：

```text
sys.version: 3.4.0; CanMV v1.8-0-gc2d1f5c(based on Micropython e00a144) on 2026-07-03
sys.implementation: (name='micropython', version=(1, 21, 0), _machine='k230_canmv_v3p0 with K230', _mpy=262)
os.uname: (sysname='rt-smart', nodename='rt-smart', release='v1.8-0-gc2d1f5c', version='v1.8-0-gc2d1f5c on 2026-07-03', machine='k230_canmv_v3p0')
```

## UART1 引脚

以下对应关系按 **CanMV-K230-LP4 V3.0** 官方硬件设计文件核对，并已用实物回环验证。

| 功能 | GPIO / 板上丝印 | JP1 排针物理脚号 | FPIOA 功能 |
| --- | --- | --- | --- |
| UART1 发送 | **GPIO3 / IO_3** | **18** | `FPIOA.UART1_TXD` |
| UART1 接收 | **GPIO4 / IO_4** | **13** | `FPIOA.UART1_RXD` |

GPIO 编号与物理脚号是两套编号。优先按实物 `IO_3`、`IO_4` 丝印定位。

按用户提供的排针背面照片方向：`IO_3` 位于左列，与右列 `IO_5` 同排；`IO_4` 位于右列，与左列 `IO_35` 同排。照片中对应焊点在丝印文字正上方；杜邦线插到另一面的对应排针。

## 已通过的回环测试

脚本：[../examples/uart1_loopback.py](../../device/k230/diagnostics/uart1_loopback.py)。在 CanMV IDE 中打开并运行；测试脚本不保存为开机自动运行的 `main.py`。

- 参数：**UART1，115200 bps，8 数据位，无校验，1 停止位（8N1）**。
- 测试接线：断电后，仅用一根杜邦线连接 `IO_3 ↔ IO_4`；这两个信号脚与 URT-2 等外设断开。
- 数据：每轮包含 `00 55 AA FF`、`UART1_`、不同序号和换行，逐字节比较。
- 用户反馈的两次运行：未接线时 **0/10，RX 为空**；接上回环线后 **10/10，全部 PASS**。
- 结论：这组引脚上的 UART1 **在 115200 bps 下外部回环通过**。

这次回环没有验证 1 Mbps、URT-2 转换电路或舵机应答。接 URT-2 前拆除回环线；对舵机使用配套协议的 Ping/只读查询。

## K230 直连 URT-2 的接线方案

状态：**2026-10-19 用户确认四根线已接好，随后在 K230 上实测：1 Mbps 下 ID 1～6 均 Ping 成功，并完成身份与状态寄存器读取，6/6 通过，错误状态均为 0x00。** 电平开关、URT-2 USB 与舵机独立电源按下列接线要求保持。

| K230 | URT-2 UART 排针 |
| --- | --- |
| `IO_3`（UART1 TX） | **TX** |
| `IO_4`（UART1 RX） | **RX** |
| `GND` | **GND** |
| `5V` | **5V 逻辑供电脚** |

- 飞特官方图中的 MCU 接法为 **TX—TX、RX—RX**；按该转换板丝印和文档连接。
- 断电操作，将 URT-2 的逻辑电平开关置于 **3V3** 侧，以匹配这里使用的 K230 信号。
- K230 保持 USB OTG 连接电脑；此 MCU 直连接法中，**URT-2 的 USB 线拔掉**，按官方“USB / MCU 二选一”的要求操作。
- 上表 5V 只给转换板逻辑部分供电。舵机用符合具体型号要求的独立电源，电源负极、URT-2 与 K230 共地；不得把电池电压接到 UART 排针。
- 该接法后续按舵机的 **1,000,000 bps** 通信；回环脚本的 115200 是单独测试参数。
- 本次验证已覆盖 **K230 UART1→URT-2→6 个舵机→K230 回读** 的整条通信链路。读取通过不等于位置标定、运动精度或运动安全性已验证。

## 六舵机只读检测

脚本：[../examples/uart1_servo_read.py](../../device/k230/diagnostics/uart1_servo_read.py)。适用当前 K230 V3.0/v1.8、URT-2、ST-3215-C018、ID 1～6、1,000,000 bps。用户已确认舵机独立电源接好并打开；本次舵机上报电压为 **12.1～12.2V**。这是舵机内部测量值，不替代万用表实测或电源规格确认。

运行前保持四线接法、URT-2 电平开关 3V3、URT-2 USB 拔除，拆除原 IO_3/IO_4 回环线。舵机应接 TTL-Bus，舵机电源与 K230 共地。

在 CanMV IDE 打开脚本并运行，查看串行终端。脚本只允许对 ID 1～6 单播 PING（0x01）/READ（0x02），不写位置、扭矩、ID、波特率或任何配置。它会：

1. 校验应答头、ID、长度与校验和，过滤原样发送回显。
2. 读取寄存器 3～6 的型号编码、配置 ID 和波特率代码（0 代表 1 Mbps）。型号编码不会被推断成完整 C018 型号标签。
3. 读取寄存器 56～63 的位置、电压（0.1V/单位）、温度；错误状态非零时单独告警。
4. 分别汇总 `PING_RESPONDED`、`READ_CONFIRMED`、`STATUS_CLEAN`。六个 ID 均完成有效身份与状态读取才输出 `ALL_SERVOS_READ_OK`，回显或无响应不算成功。

**当前状态：2026-10-19 已在真机执行，6/6 通过。** IDE 直接运行本脚本后，六个 ID 的 Ping、身份读取和状态读取均通过。结果如下，位置是读取当时的快照，不作为零位标定或运动目标：

| ID | 型号编码 | 波特率代码 | 位置原始值 | 舵机上报电压 | 温度 | 错误状态 |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 777 | 0（1 Mbps） | 2001 | 12.1V | 34°C | 0x00 |
| 2 | 777 | 0（1 Mbps） | 993 | 12.2V | 34°C | 0x00 |
| 3 | 777 | 0（1 Mbps） | 3186 | 12.1V | 33°C | 0x00 |
| 4 | 777 | 0（1 Mbps） | 3240 | 12.1V | 34°C | 0x00 |
| 5 | 777 | 0（1 Mbps） | 2039 | 12.1V | 36°C | 0x00 |
| 6 | 777 | 0（1 Mbps） | 2048 | 12.1V | 36°C | 0x00 |

```text
PING_RESPONDED: [1, 2, 3, 4, 5, 6]
READ_CONFIRMED: [1, 2, 3, 4, 5, 6]
MISSING_OR_FAILED: []
STATUS_CLEAN: [1, 2, 3, 4, 5, 6]
RESULT: 6 / 6 servos read
ALL_SERVOS_READ_OK
```

本次只发送 PING 和 READ，没有发送位置、速度、扭矩或配置写指令。`ALL_SERVOS_READ_OK` 表示通信读取通过；非零错误状态需另外排查，不自动授权运动。脚本已根据真机兼容性使用 `bytes(params)` 组包和不可变字节串解析，避免 MicroPython 不支持的字节数组删除操作。

## 5V 供电与 3V3 通信电平

两者用途不同，可以同时使用：

| 项目 | 用途 | 本方案要求 |
| --- | --- | --- |
| K230 → URT-2 的 `5V` 电源线 | 给转换板逻辑电路提供工作电源 | 接 URT-2 UART 排针的 5V 逻辑供电脚 |
| URT-2 的 `3V3 / 5V` 电平开关 | 选择串口转换电路的信号逻辑电平 | 与 K230 这里使用的 3.3V UART 匹配，置于 **3V3** |
| 舵机的独立电源 | 驱动六只舵机 | 按 ST-3215-C018 的实际规格选取，并与控制器共地 |

`3V3` 档不表示应将上表的 5V 供电线改接 K230 的 3.3V。反过来，转换板采用 5V 供电，也不表示 K230 的 TX/RX 能承受 5V 信号。K230 的 UART 信号电平必须匹配所用 I/O 域，不能接入 5V UART 信号。

这条 5V 线只承担转换板逻辑供电，六只舵机的负载电流通过独立舵机电源回路提供。

## UART2 与 MSPM0 的接线和供电

### 引脚选择与当前进度

2026-10-19，用户报告队友已接好 `GND—GND` 与 `K230 IO_6（与 IO_26 同排）—MSPM0 A09`。此时尚未做双板串口测试。

官方引脚功能表中，K230 `GPIO5` 支持 `UART2_TXD`、`GPIO6` 支持 `UART2_RXD`；本方案沿用这组映射。MSPM0G3507 的 `PA08` 为 UART1 TX，`PA09` 为 UART1 RX。因此已接的 **IO_6—A09 是 RX—RX**，需要把其 MSPM0 一端从 A09 移到 A08，再补 IO_5—A09。

最终接线如下；**2026-10-19 用户已确认按此方案接好三根线，尚未完成两板通信验证**：

| K230 V3.0 | JP1 物理脚号 | MSPM0 天猛星 | 方向 |
| --- | --- | --- | --- |
| **IO_5 / GPIO5 / UART2 TX** | **17** | **A09 / PA09 / UART1 RX** | K230 → MSPM0 |
| **IO_6 / GPIO6 / UART2 RX** | **20** | **A08 / PA08 / UART1 TX** | MSPM0 → K230 |
| **GND** | 按 GND 丝印 | **GND** | 共地 |

K230 实物丝印定位：`IO_5` 与 `IO_3` 同排；`IO_6` 与 `IO_26` 同排。优先看 GPIO 丝印，不能把 GPIO 编号当作物理脚号。MSPM0 的 `A08/A09` 是 `PA08/PA09`，不是芯片封装第 8/9 脚。

两端串口编号无需相同：这条整机链路称 UART2，是使用 **K230 的 UART2**；MSPM0 一端使用 **UART1**。MSPM0 的 PA10/PA11 UART0 可继续用于板载 USB 串口，不应混用。

### 软件配置与测试计划

- K230：`fpioa.set_function(5, FPIOA.UART2_TXD)`、`fpioa.set_function(6, FPIOA.UART2_RXD)`，使用 `UART.UART2`。
- MSPM0：启用 **UART1**，PA08 TX / PA09 RX。
- 首轮串口测试计划使用 **115200 bps、8 数据位、无校验、1 停止位（8N1）**，双方一致；这是底盘链路参数，舵机链路仍为 UART1 / 1 Mbps。
- 仓库当前 `car/demo/01_motor_encoder/` 配置的是 **UART0、PA10 TX / PA11 RX、115200 bps**，未包含 PA08/PA09 UART1 收发。仅接线不会让此现有固件自动回显；需要另行准备并烧录适配的通信测试固件。
- 已新建独立测试工程 [car/demo/02_uart1_link](../../device/mspm0/uart_link/README.md)，含 MSPM0 UART1 收发固件、Keil 工程、编译脚本和 K230 UART2 校验脚本。固件已编译，HEX 校验通过，仅覆盖 Main Flash 的 1672 字节；未改原电机工程。
- 用户选择 fireDAP / SWD 下载；首次烧录在读取 SWD 调试端口时失败：`E310: Debug access failed - cannot read DP register 0x00`。随后用户检查并表示接线没有问题；新工程请求将调试时钟从 5 MHz 降到 100 kHz 后，Keil 仍报相同 E310。**尚未烧录成功，也未开始两板通信测试。**
- 计划使用带序号和令牌的 `K230_TEST` / `MSPM0_ACK` 请求应答验证双向链路，20/20 才通过；原样回显不计成功。测试固件不配置电机 PWM，并按原工程定义将 PA12 / TB6612 STBY 置低。
- 对照原小车 demo 的本机 `.uvoptx`：探针为 fireDAP CMSIS-DAP，同一序列号，SWD / Protocol 2，同一 TI 设备包 1.3.1 与 Main Flash 128KB 算法。原 demo 目录保存的 `CMSIS-DAP_DebugLog_0001.log` 第 11 行显示：2026-09-29 20:09:58 成功读取 DP 地址 0x00，值 `0x6BA02477`。这确认此前 SWD 连接正常，不等于确认本次连接或完整烧录正常。
- 为排除新工程/Keil 设置影响，2026-10-19 使用标准 CMSIS-DAP 命令直接只读检测：探针固件报告 2.0.0，SWD 模式初始化成功；分别请求 100,000 Hz 和 10,000 Hz 时钟，读取 DP IDCODE 均返回 `05 00 07`（执行数 0，NO_ACK）。已断开诊断会话，未擦除或写入芯片。当前结论限于“目标 SWD 端口无应答”，不能判定 fireDAP 损坏或用户接错线。
- 等待核对当前目标板供电来源、RST/NRST 是否连接，以及两端完整接线照片，再决定复位连接方式；电脑只识别到 fireDAP 的 COM3 与 K230 的 COM8，未识别到天猛星自身 USB 串口。这不单独证明目标板无电，因为也可能独立供电或使用其他 USB 线。

- 2026-10-19，用户要求再次重试后，fireDAP 下载仍在 `DebugPortSetup` 阶段报相同 E310，未进入程序写入和校验；日志保存于本机忽略提交的 `car/demo/02_uart1_link/build/flash_retry_20261019.log`。错误提示已关闭，尚无烧录或双板 UART 通信成功结果。

- 2026-10-19，用户将所有线重新连接后要求再试，下载仍在 `DebugPortSetup` 读取 DP 0x00 时报 E310。日志：本机 `car/demo/02_uart1_link/build/flash_rewired_20261019.log`（忽略提交）。本次未进入程序写入和校验；尚未确认目标供电与复位状态，不据此判定具体故障原因。

- SWD 复位线说明：RST/NRST 并非普通 SWD 连接的必接线；询问是否连接是为判断能否使用硬件复位/Connect under Reset 排障，不表示当前错误已确定由缺少 RST 导致。需要使用该方式时，确认目标 NRST 引出点后再接 fireDAP RST，不猜针脚。

- 2026-10-19，用户提供三张当前接线照片。先前助手根据插头与相邻标注位置判断误接 5V/TX/RX，此判断被用户明确纠正并撤回：用户说明 **5V、TX、RX 均未接，连接的是下面的针脚**。照片不能清晰显示被插头遮挡的下排丝印，不能把“接错串口”作为已确认故障原因。
- 正确 SWD 信号对应仍为 VREF→目标 3V3、TMS→SWDIO/PA19、TCK→SWCLK/PA20、GND→GND。用户随后明确确认天猛星端颜色对应：**黑线→3V3、紫线→CLK、白线→DIO、灰线→GND**。这些板端信号名与所需 SWD 接线一致，不要求依据此前照片误判重新改线。
- 当前烧录失败原因尚未确定。板端实际信号对应已由用户确认；此前 E310/NO_ACK 测试结果有效，但不据此判断具体物理故障。用户已短按 RST 复位，随后重试仍在 DebugPortSetup 读取 DP 0x00 时返回 E310。普通复位后重试未解决连接问题。

- 2026-10-19 用户确认“已复位”后，已重新执行 fireDAP 下载，仍报 `E310: cannot read DP register 0x00`，未进入写入和校验；日志为本机忽略提交的 `car/demo/02_uart1_link/build/flash_after_reset_20261019.log`。随后已与用户配合执行 BSL + RST 按键和连续只读检测，结果见下；不代表已确认为应用程序关闭 SWD。

- 2026-10-19 用户在检测运行期间完成按住 BSL、短按并松开 RST 的操作并回复“按好了”。连续 120 秒、100 kHz 下 965 次 DP IDCODE 读取全部返回 `05 00 07`（NO_ACK），没有读到预期的 0x6BA02477，未继续烧录。诊断会话已正常 Disconnect；未执行擦除、烧录或解锁。本次仅确认按键操作已完成，未独立确认芯片是否成功进入 BSL。日志位于本机 `car/demo/02_uart1_link/build/swd_bsl_poll.log`。

### 供电
供电电压核对（标准值，尚非用户实测结果）：

| 测量或供电位置 | 本方案电压 |
| --- | --- |
| 天猛星 Type-C 供电输入 | 稳定 5V USB 电源 |
| 板上 5V—GND | 约 5V，受实际输入与电源路径影响 |
| 板上 3V3—GND | 约 3.3V，供 MCU/逻辑使用 |
| fireDAP VREF—GND | 接目标板 3V3，约 3.3V，用作参考而非目标板供电 |

当前 UART/SWD 排障应先测 3V3—GND，再核对 5V—GND。整机电池不能直接接入天猛星 Type-C、5V 或 3V3，引入 USB 前需稳压到 5V。不要将“Type-C 输入 5V”误解为 MCU 或 SWD 信号采用 5V。

依据：[立创天猛星官方入门手册](https://wiki.lckfb.com/zh-hans/tmx-mspm0g3507/keil-beginner/install.html)及[野火 VREF 说明](https://doc.embedfire.com/stm32_products/must_read/zh/latest/doc/quickstart/DAP/DAP.html)。


依据整机草案，K230 与 MSPM0 各自由 DC-DC 的 USB 输出供电，测试时也可分别由各自 USB 供电。**UART2 本身不要求额外占用 K230 的 5V 引脚。** 两板间只接上表的 TX、RX、GND，使用匹配的 3.3V UART 信号，不再互连 5V/3V3 电源输出，避免两供电源并接或反向供电。断电后调整信号线。

URT-2 的 TX—TX/RX—RX 是该转换板的丝印接法，不能照搬到这两块 MCU 的 UART 对接。5V 电源属于供电网络，各路 UART 不独占一个 5V 电源脚；若以后需要分支供电，应另核对总电流与接插件承载能力。

## 整机架构草案

依据用户手绘图保存，属于设计方案，未在本次 UART1 回环中验证。

- 电源：`11.1V / 5000mAh / 10C` 电池 → 直流分线器 → URT-2/机械臂舵机、电机驱动板/小车电机、双 USB DC-DC/K230 与 MSPM0。实际电压、负载电流及所需稳压仍按器件规格核对。
- 控制：K230 经 **UART1** 接 URT-2/机械臂；另一条串口链路计划用于 K230 与 MSPM0 通信。
- 底盘：MSPM0 控制电机驱动板，读取编码器和 IMU。另一条串口的引脚、协议和反馈格式未在本次记录中确定。

## 常用资料

- [飞特 FD 调试软件](https://gitee.com/ftservo/fddebug/tree/master)
- [FT 管理平台](http://doc.feetech.cn/#/prodinfodownload?srcType=FTServo-emanual-ab4947a479784a71a8cb7928)
- [飞特 URT-2 官方接线图](https://raw.githubusercontent.com/ftservo/ftservo-wiki/main/docs/assets/images/wiring/urt-2-ttl-wiring.png)
- [嘉楠官方 V3.0 硬件设计文件](https://download.kendryte.com/developer/k230/HDK/CanMV-K230%E5%B7%A5%E7%A8%8B%E8%AE%BE%E8%AE%A1%E6%96%87%E4%BB%B6/ProDocument_CanMV-K230-LP4-V3.0-20240509.epro)
- [CanMV v1.8 官方 UART1 示例](https://github.com/kendryte/canmv_k230/blob/v1.8/resources/examples/03-Machine/uart1.py)
- [嘉楠官方 UART 电平与接线说明](https://www.kendryte.com/k230_canmv/en/main/example/peripheral/uart.html)

- [飞特官方 SMS/STS 寄存器表](https://github.com/ftservo/FTServo_Arduino/blob/main/src/SMS_STS.h)

- [嘉楠 FPIOA 教程与 GPIO5/6 可选功能表](https://www.kendryte.com/k230_canmv/v0.4/zh/example/peripheral/fpioa.html)
- [立创天猛星 PA08/PA09 UART1 接线示例](https://wiki.lckfb.com/zh-hans/tmx-mspm0g3507/module/rf/open-mv4-camera.html)
