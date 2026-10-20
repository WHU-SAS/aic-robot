# K230 ↔ MSPM0 UART 测试

MSPM0G3507 UART1 ↔ K230 UART2，115200/8N1、无流控。接线见 [硬件接线](../../../docs/hardware/connections.md#k230-uart2--mspm0-uart1)，编译、Keil 工程和下载步骤见 [板端联调](../../../docs/guides/k230-and-mspm0.md)。

Linux 在仓库根目录运行 `./scripts/mspm0-build uart_link`，生成本目录 `build/gcc/uart1_link.elf` / `.hex` / `.bin`。

## 程序与协议

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

固件已在 Windows Keil 和 Linux GCC 编译（Main Flash 映像分别为 1672 / 1656 字节）；10 月 20 日 Linux 下烧录与逐字节回读已通过，USB 串口收到 `MSPM0_UART1_TEST_READY`，BCR/BSL 未变；接线、重新上电后首轮为 19/20（首包未应答），程序不变复测 20/20 通过，上电首包问题待定位。进度见 [项目首页](../../../README.md#当前情况)。
