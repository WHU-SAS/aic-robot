# 板端测试、编译与下载

接线统一见 [硬件接线](../hardware/connections.md)，当前进度见 [项目首页](../../README.md#当前情况)。CanMV 运行板端脚本，Linux 抓取入口在 PC 运行。

## CanMV 只读测试

在 CanMV IDE 连接开发板，打开下表脚本并点击运行；不保存为自动运行的 `main.py`。

| 测试 | 脚本 | 操作与通过标准 |
| --- | --- | --- |
| UART1 外部回环 | [uart1_loopback.py](../../device/k230/diagnostics/uart1_loopback.py) | 断开 URT-2，短接 GPIO3/4；115200/8N1，10/10；结束拆除短接 |
| UART1 舵机只读 | [uart1_servo_read.py](../../device/k230/diagnostics/uart1_servo_read.py) | 按接线页连接 URT-2，1 Mbps；六舵机 PING / READ 均成功，错误 0x00 |
| UART2 ↔ MSPM0 UART1 | [k230_uart2_test.py](../../device/k230/diagnostics/k230_uart2_test.py) | 先成功烧录 MSPM0 UART 固件；20/20 带序号和令牌的应答，原样回显不算通过 |

三项均已有通过记录。双板 UART 于 2026-10-20 在接线、重新上电后首轮测试 19/20（仅第 1 包未应答），保持程序不变复测 20/20；上电首包问题尚未定位。只读脚本会校验应答 ID、长度、校验和与错误状态。

## Linux 编译与 IDE

本机命令行工具位于 `~/.local/share/aic-tools/`，Python 工具环境独立；IDE 插件和设备包分别安装于 VS Code 与 CMSIS 缓存。

| 工具 | 版本 |
| --- | --- |
| Arm GNU（含 GCC / GDB / objcopy） | 14.3.Rel1 |
| pyOCD / CMSIS-Toolbox | 0.46.0 / 2.15.0 |
| TI MSPM0G1X0X_G3X0X_DFP | 1.3.1 |
| Keil Studio / Cortex-Debug | 1.20.3 / 1.12.1（VS Code 插件） |
| clangd / Ninja | 23.1.0 / 1.13.2 |

在仓库根目录执行，以下命令均不访问硬件：

```bash
./scripts/mspm0-tools                       # 查看工具版本
./scripts/mspm0-build all                   # 编译两个工程
./scripts/mspm0-build uart_link             # 只编译通信固件
./scripts/mspm0-build motor_encoder         # 只编译电机固件
./scripts/mspm0-tools python -m unittest discover -s device/mspm0/tests
```

复用现有 C 源码、引脚配置和 TI SDK 的 GCC 启动文件、链接脚本、驱动库；产物在各工程 `build/gcc/`，包括 ELF、HEX、BIN、构建日志与校验记录。两份固件已通过编译、Main Flash 地址和中断向量核对。2026-10-20 已将 UART 固件在 Linux 烧录并逐字节回读通过，收到启动日志；电机固件同日烧录、回读通过，并完成车轮悬空的按键启动、PWM 缓升、5 秒自动停车和编码器正反计数检查。

2026-10-20 已在 UART 固件上实测 GDB 硬件断点与指令单步：命中 `main`，单步 PC 从 `0x28c` 前进到 `0x28e`，删除断点后恢复运行；未改写 Flash。VS Code 图形界面的 F5 操作尚未现场验证。

VS Code 已配置 `Ctrl+Shift+B` 编译任务和 UART / 电机两项 Cortex-Debug 启动项。调试启动会连接并下载固件，等现场就位后再使用。配置模板在 `configs/mspm0/vscode-*.json`；本机 `.vscode/` 不提交。Keil Studio 提供 IDE 工具，现有工程通过上述 Python/GCC 入口构建，未转换为 CMSIS Solution。

本机 USB 权限规则已安装。其他 Linux 电脑需输入 sudo 密码安装一次，再重插 fireDAP：

```bash
sudo ./scripts/install-debug-usb-rules
```

本机用户已属于 `plugdev` 和 `dialout`。pyOCD 配置为 `configs/mspm0/pyocd.yaml`（100 kHz，关闭自动解锁，使用页擦除）。2026-10-20 已通过 fireDAP 连接并读取 DP ID `0x6BA02477`、CPUID `0x410CC601` 和 128 KiB Flash / 32 KiB SRAM 容量。随后完成 UART 固件烧录、全 Flash 回读和启动日志检查；只有前 2 KiB 被改写，BCR/BSL 配置未变化。原程序备份与记录保存在本机 `data/logs/mspm0_uart_bringup_*/`。

**调试连接会干扰运行**：当前 TI 设备包的连接序列会操作复位；实测 `attach` 后内核处于暂停状态，不能当作无侵入读取。通信测试前需明确恢复运行；电机运行中不要新连接调试器、打断点或单步。一次保持同一 SWD 会话并明确恢复运行的通信复测为 20/20，接收/应答计数各增加 20、丢弃计数为 0；退出调试器后复测也为 20/20。最初冷上电的首包丢失尚未复现，不能据此认定已修复。

工具下载来源和 SHA256 见 `configs/mspm0/tools-lock.json`，Python 工具版本见 `configs/mspm0/requirements-tools.txt`。现有生成的 C 配置可直接编译，SysConfig 和 CCS 暂不需要安装。

K230 接入 OTG USB、CanMV IDE 停止并断开后，可直接从 Linux 运行双板测试：

```bash
./scripts/host tools/diagnostics/test_k230_mspm0_link.py
```

当前 MSPM0 保留电机例程且已停车；运行此通信测试前需先烧录并运行 UART 测试固件，两份例程分别烧录运行。

复用板端既有 20 轮测试，不控制舵机。需要每轮带序号与令牌的 `MSPM0_ACK`，20/20 才通过；日志在 `data/logs/k230_mspm0_link_*/`。当前已通过一轮完整 20/20 验收；首次上电测试存在一次首包无应答，不能据此认定上电同步已稳定。

## Windows Keil 编译

需要 Keil Arm Compiler 6 和 MSPM0G3507 设备包。本机曾使用 Compiler 6.16、TI 设备包 1.3.1；其他安装位置用 `--toolchain` 指定。

在仓库根目录运行，选择一个工程：

```powershell
python device/mspm0/motor_encoder/build.py --toolchain D:/Keil_v5/ARM/ARMCLANG/bin
python device/mspm0/uart_link/build.py --toolchain D:/Keil_v5/ARM/ARMCLANG/bin
```

输出为各工程 `build/` 下的 HEX / AXF，不随 Git 提供。两个工程共享 `third_party/ti_mspm0_sdk/` 和 `device/mspm0/platform/`。Linux 可加 `--check-paths` 仅检查文件引用。

## Windows：fireDAP / SWD 下载

断开电池和电机驱动信号，主控 Type-C 独立供电。按 [SWD 接线表](../hardware/connections.md#mspm0-swd) 连接后：

1. 打开目标工程：电机 `device/mspm0/motor_encoder/keil/motor_demo.uvprojx` 或 UART `device/mspm0/uart_link/keil/uart1_link.uvprojx`。
2. `Options for Target → Debug` 选择 CMSIS-DAP Debugger、实际 fireDAP，Port 设 SW，先用 100 kHz。已接 RST 时可尝试 Connect under Reset。
3. 先确认能读到调试 ID；Flash Download 选择 **MSPM0G3507 Main Flash 128KB** 算法，Utilities 使用同一调试器。
4. 编译、Download，成功后复位。应用日志从主控自身 USB 串口读取，115200/8N1；fireDAP 的虚拟串口是另一设备。

2026-10-19 曾报 `E310: cannot read DP register 0x00` / `NO_ACK`；10 月 20 日 Linux 连接与读取已通过，Linux 下载与回读也已通过，原故障原因未定，Windows 本次未重测。此前未执行解锁、整片擦除或 NONMAIN 改写，故障证据见 [联调摘要](../history/2026-10-19-hardware-bringup.md)。

## 备选：主控 USB / BSL 下载

可使用立创 [MSPM0 在线烧录工具](https://wiki.lckfb.com/storage/html/mspm0-web-flasher/index.html)。Edge / Chrome 打开工具，按住 BSL → 按下并松开 RST → 松开 BSL，选择主控对应端口和本机生成的 HEX 下载；完成后断开网页串口、复位，再打开应用日志。BSL 波特率按工具设置，应用日志为 115200。下载会替换当前应用。

参考：[立创 Type-C / BSL](https://wiki.lckfb.com/zh-hans/web-tool/mspm0-web-flasher/introduce.html)、[fireDAP 操作说明](https://doc.embedfire.com/stm32_products/must_read/zh/latest/doc/quickstart/DAP/DAP.html)（目标器件和算法仍选 MSPM0）。
