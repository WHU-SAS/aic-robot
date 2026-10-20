# K230 ↔ MSPM0 UART 测试

MSPM0G3507 UART1 ↔ K230 UART2，115200/8N1、无流控。接线见 [硬件接线](../../../docs/hardware/connections.md#k230-uart2--mspm0-uart1)，编译、Keil 工程和下载步骤见 [板端联调](../../../docs/guides/k230-and-mspm0.md)。

Linux 在仓库根目录运行 `./scripts/mspm0-build uart_link`，生成本目录 `build/gcc/uart1_link.elf` / `.hex` / `.bin`。

## 程序与协议

- [uart_request_parser.h](uart_request_parser.h) 搜索完整帧头并校验两位序号、八位十六进制令牌；忽略帧前杂字节，在残缺帧后重新同步。
- [main.c](main.c) 接收 `K230_TEST,<序号>,<令牌>\n`，回复 `MSPM0_ACK,<序号>,<令牌>\n`。
- [k230_uart2_test.py](../../k230/diagnostics/k230_uart2_test.py) 发 20 轮请求，校验序号和令牌，拒绝原样回显。
- MSPM0 使用 32 MHz SYSOSC/BUSCLK，分频后约 115211.5 bps。
- UART0（PA10 TX / PA11 RX）另输出启动和收发日志，115200/8N1。
- 本固件不输出电机 PWM，PA12 / TB6612 STBY 保持低电平。

## 验收

先确认 MSPM0 固件烧录成功、复位运行，再在 CanMV IDE 执行 K230 测试脚本。通过输出应为：

```text
RESULT: 20 / 20
UART2_MSPM0_LINK_PASS
```

无应答时检查固件是否运行、共地、TX/RX 接线与波特率。`LOCAL_ECHO_ONLY` 表示本地回环，应拆除 TX/RX 短接。

2026-10-20 已在原固件复现冷启动首包失败，UART0 显示 `UART1_LINE_DISCARDED`。修复后的首次接收日志明确为 `00` + 完整请求；按帧头重新同步后，确认两板电源灯全灭再上电的一轮为 **20/20，首包通过**，没有预热或重试。`00` 的具体电气来源尚未测量。

本目录为纯通信测试固件；当前主控运行 [K230 UART 电机联调固件](../motor_uart/README.md)，协议不同，不能直接对它运行本页20轮测试。此前本UART固件的验证记录如下。Linux 映像 1872 字节，经主板 Type-C / BSL 写入、独立两次 CRC 核验通过；写入范围之外的 Main Flash 已核对。SWD 阶段的旧映像曾逐字节回读并检查 BCR/BSL 未变；本次 BSL 仅使用 Main Flash 范围擦写命令。

首次有效请求后，UART0 输出 `UART1_FIRST_RX_HEX`。要同步保存这份诊断，在仓库根目录运行：

```bash
./scripts/host tools/diagnostics/test_k230_mspm0_link.py \
  --mspm0-log-port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
```

先由人确认两板彻底断电再上电，测试前不连接 SWD、不发送预热包。日志在本机 `data/logs/`，不提交。编译及 BSL 步骤见 [板端联调](../../../docs/guides/k230-and-mspm0.md)。
