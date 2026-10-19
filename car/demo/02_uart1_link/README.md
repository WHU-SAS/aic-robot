# K230 UART2 ↔ MSPM0 UART1 收发测试

独立的 MSPM0G3507 测试工程，配合 CanMV K230 V3.0 使用。两端参数均为 **115200 bps、8N1、无流控**。

## 当前状态（2026-10-19）

- 用户已确认按下表完成三根通信线。
- MSPM0 固件已编译，生成 `build/uart1_link.axf` 和 `build/uart1_link.hex`；HEX 校验通过，写入范围仅 Main Flash，地址 `0x00000000～0x00000687`，共 1672 字节。
- 首次 fireDAP 烧录未完成，Keil 报 `E310: Debug access failed - cannot read DP register 0x00`。用户检查后表示接线无误；工程请求降至 100 kHz 重试仍失败。绕过 Keil 的只读检测在 100 kHz 与 10 kHz 都返回 NO_ACK，后续对照片中插接位置的判断被用户纠正：5V/TX/RX 均未接，接的是下面针脚。用户已进一步确认板端黑→3V3、紫→CLK、白→DIO、灰→GND，信号对应正确；当前失败原因尚未确定。
- 对照原 demo 的实际调试日志，2026-09-29 曾成功读取 SWD ID `0x6BA02477`；当前失败不能据此归因于 fireDAP 损坏或接线错误。
- 用户配合 BSL + RST 按键后，连续 120 秒检测期间共 965 次读取均 NO_ACK，未恢复 SWD；待检查板上 3V3/5V 实测电压和转接线通断。
- **尚未完成真机烧录和两板通信验证。** 下文的成功输出是验收标准，不是已有实测结果。

## 通信接线

断电后接线；两板各自供电，只互连 TX、RX、GND。

| K230 V3.0 | MSPM0 天猛星 | 数据方向 |
| --- | --- | --- |
| IO_5 / UART2 TX | A09 / PA09 / UART1 RX | K230 → MSPM0 |
| IO_6 / UART2 RX | A08 / PA08 / UART1 TX | MSPM0 → K230 |
| GND | GND | 共地 |

K230 IO_5 与 IO_3 同排，IO_6 与 IO_26 同排，优先按丝印定位。不要把 GPIO 编号当排针物理脚号。不要互连两板的 5V/3V3 电源输出。

本链路使用 K230 的 UART2 和 MSPM0 的 UART1。机械臂仍使用 K230 UART1 / GPIO3、GPIO4 / 1 Mbps，不更改。

## 程序行为

- [main.c](main.c)：MSPM0 接收 `K230_TEST,<序号>,<令牌>\n`，回复 `MSPM0_ACK,<序号>,<令牌>\n`。
- [k230_uart2_test.py](k230_uart2_test.py)：K230 连续发送 20 轮，每轮检查带相同序号和令牌的 MSPM0 应答。原样回显不会判为成功。
- 固件采用 32 MHz SYSOSC/BUSCLK；UART 分频对应约 115211.5 bps。
- UART0（PA10 TX / PA11 RX）另输出启动和收发日志，115200/8N1，可通过目标板自身 USB 串口辅助排查。
- 不配置电机 PWM，按现有电机工程的定义将 PA12 / TB6612 STBY 保持低电平。

## 编译

复用 `../01_motor_encoder/source/` 的 TI SDK、原工程的启动文件与链接脚本。需要 Keil Arm Compiler 6，默认安装路径为 `D:\Keil_v5\ARM\ARMCLANG\bin`。

在本目录运行：

```powershell
python build.py
```

其他安装路径：

```powershell
python build.py --toolchain '你的Keil目录\ARM\ARMCLANG\bin'
```

脚本只编译，不烧录。构建日志、映像和校验记录位于忽略提交的 `build/`。

## 使用 fireDAP 烧录

打开 [Keil 工程](keil/uart1_link.uvprojx)，选择 `Tianmengxing_UART1_Link_Test`。使用 CMSIS-DAP / SWD，安装与 MSPM0G3507 匹配的 TI 设备包（本机版本 1.3.1）。本机探针选择保存在忽略提交的 `.uvoptx` 中，其他电脑需自行选择探针。

检查接线时先断电，注意：VREF 是目标电压参考，目标板应独立供电。

| fireDAP | MSPM0 天猛星 |
| --- | --- |
| VREF | 3V3 |
| TMS / SWDIO | DIO / PA19 / SWDIO |
| TCK / SWCLK | CLK / PA20 / SWCLK |
| GND | GND |

接口核对：按实际丝印和信号名核对连接，不能因插头靠近 `5V / TX / RX` 标注就判断接了这些针。2026-10-19 用户明确说明这些针未接；下排引脚被插头遮挡，助手此前的“误接串口”判断已撤回。

### 用户确认的线色对应（2026-10-19）

| 线色 | 天猛星端实际引脚 | 应对应的 fireDAP 信号 |
| --- | --- | --- |
| 黑 | 3V3 | VREF |
| 紫 | CLK / PA20 | TCK |
| 白 | DIO / PA19 | TMS |
| 灰 | GND | GND |

上表板端连接由用户逐一确认，不根据照片中相邻 5V/TX/RX 标注推断。当前 SWD 无应答的原因仍未确定。

RST/NRST 不属于普通 SWD 通信必须接的信号。正常连接先使用上表四根线；若需由调试器执行硬件复位或使用 Connect under Reset 排障，再将 fireDAP RST 接到目标板已确认的 NRST/RST。未接 RST 本身不能解释本次 E310，也不能据此认定接线错误。

确认供电与接线后执行 Download。烧录会暂时替换 MSPM0 当前应用；原电机工程源文件未修改。不执行解锁、整片擦除或 NONMAIN 配置改写来绕过连接错误。

首次错误发生在读取 SWD 调试端口阶段，应先解决探针到目标芯片的连接，再测试 UART。电脑当前识别的 **COM3 属于 fireDAP 的虚拟串口**，不能据此判定天猛星自身 USB 串口已连接；K230 当前为 COM8，端口号换 USB 口后可能变化。

## 运行与验收

1. 确认 MSPM0 烧录成功且测试程序运行，保持上表三根通信线连接。
2. 用 CanMV IDE 连接 K230，打开 `k230_uart2_test.py`，点击运行，查看串行终端。
3. 正常情况下输出 20 轮 PASS，最后为：

```text
RESULT: 20 / 20
UART2_MSPM0_LINK_PASS
```

无应答时检查：MSPM0 是否已运行新固件、两板是否共地、IO_5→A09 / IO_6←A08 是否接对、双方是否 115200/8N1。若输出 `LOCAL_ECHO_ONLY`，应检查是否仍短接 K230 TX/RX；回显不代表收到 MSPM0 应答。

完整硬件参数与联调进展见 [硬件与联调记录](../../../arm/docs/硬件与联调记录.md)。
