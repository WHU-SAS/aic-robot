# 板端测试、编译与下载

接线统一见 [硬件接线](../hardware/connections.md)，当前进度见 [项目首页](../../README.md#当前情况)。CanMV 运行板端脚本，Linux 抓取入口在 PC 运行。

## CanMV 只读测试

在 CanMV IDE 连接开发板，打开下表脚本并点击运行；不保存为自动运行的 `main.py`。

| 测试 | 脚本 | 操作与通过标准 |
| --- | --- | --- |
| UART1 外部回环 | [uart1_loopback.py](../../device/k230/diagnostics/uart1_loopback.py) | 断开 URT-2，短接 GPIO3/4；115200/8N1，10/10；结束拆除短接 |
| UART1 舵机只读 | [uart1_servo_read.py](../../device/k230/diagnostics/uart1_servo_read.py) | 按接线页连接 URT-2，1 Mbps；六舵机 PING / READ 均成功，错误 0x00 |
| UART2 ↔ MSPM0 UART1 | [k230_uart2_test.py](../../device/k230/diagnostics/k230_uart2_test.py) | 先成功烧录 MSPM0 UART 固件；20/20 带序号和令牌的应答，原样回显不算通过 |

三项均已有通过记录。双板 UART 于 2026-10-20 复现首包前额外 `00` 导致旧解析器整行丢弃，改为按帧头重新同步后，确认彻底断电的一轮首包与整轮 20/20 通过。只读脚本会校验应答 ID、长度、校验和与错误状态。

## K230 麦克风与语音

IDE停止并断开后，从仓库根目录运行：

```bash
./scripts/host tools/diagnostics/test_k230_audio.py                       # 3秒麦克风采样统计
./scripts/host tools/diagnostics/test_k230_audio.py --record --seconds 5  # 保存并导出5秒WAV
```

2026-10-20已验证板载麦克风16kHz/16bit/单声道采样、1秒WAV保存与回传；采样未见削波。忽略启动时约1秒的直流瞬态，预热1.5秒后统计。日志及录音仅在本地`data/logs/k230_audio_*/`，SD录音保存在`/sdcard/voice_probe/`。该程序不初始化UART，不控制底盘或舵机，不修改启动脚本。

### 自定义唤醒：“思达思达”

唤醒词保存在`configs/voice/wake_word.json`，已完成三段录音注册；正常档用户初测约30%漏检，灵敏档12次呼叫记录11次触发；极近音词测试记录6次触发，用户接受此类误判，已将灵敏档设为检测默认值。正式独立入口已绑定单次抓取；新增“回准备位”的录音与部署见[独立语音入口](arm/K230_STANDALONE_PICK.md#注册回准备位)。原卡实测CanMV v1.8不含`open_kws`；2026-10-20已通过USB将官方V3.0 Daily镜像`v1.8-67-g277ddb6 / nncase 2.11.0`写入备用TF卡，烧录工具返回成功。新系统已启动，USBDBG、语音模型加载、16kHz麦克风、1280×960拍照及校验回传、UART1六轴只读6/6、UART2底盘状态20/20均通过；本轮未发送运动或扭矩指令。

- 固件来自[官方Daily Build](https://download.kendryte.com/developer/releases/canmv_k230_micropython/daily_build/)，包含`31-OPEN-KWS`模型、特征资源和注册示例；版本、来源与哈希见[固件锁定记录](../../configs/k230/voice-firmware-lock.json)。压缩包与解压镜像均通过官方MD5校验；烧录成功不等于逐字节回读验证。
- 用户需将旧卡另用：2026-10-21已将`/sdcard`、`/bin`、`/data`的全部可访问文件归档到本机`data/backups/k230_old_card_20261021/`，750个文件、161,865,368字节，逐文件SHA256与磁盘复读校验均通过，可用`SHA256SUMS`复核。旧卡可复用；这不是包含启动分区的整卡镜像，回退时需重新烧录固件并恢复文件。新卡不恢复旧`car_voice/start.py`为自动运行程序，其UART1运动指令与现有舵机接线冲突。
- 当前只是用户初测接受，未做长期背景误唤醒统计；动作绑定由用户指定。原标定、PID、速度和准备位保持不变；升级后实际定位与抓取精度尚未复核。

IDE停止并断开后，可重复检查接口及资源，或额外加载并释放模型；均不控制运动、不打开麦克风：

```bash
./scripts/host tools/diagnostics/check_k230_voice.py
./scripts/host tools/diagnostics/check_k230_voice.py --load-model
```

新固件使用芯片唯一USB序列号；主机入口已兼容在仅连接一块CanMV时自动找到新串口，多板连接需明确`--port`。

注册与检测入口（仓库根目录，IDE停止并断开）：

```bash
./scripts/host tools/diagnostics/enroll_k230_wake_word.py
./scripts/host tools/diagnostics/enroll_k230_wake_word.py --listen --seconds 20
./scripts/host tools/diagnostics/enroll_k230_wake_word.py --listen --sensitivity sensitive --seconds 60
```

距麦克风30～50cm；每段按回车后先静音预热约7秒，看到“现在请说”再自然说一遍“思达思达”。说完静音约0.64秒自动结束，共三段；不合格仅重录当前段。采用官方正常灵敏度与录音质量检查，注册成功后保存SD卡会话下的`keyword.json`，更新唤醒配置路径。原始PCM在SD卡`/sdcard/voice/sida_*/`，注册结果及日志在本机`data/logs/k230_wake_*/`，均不上传。录音工具只注册，不更新启动入口或驱动机械臂；运行时绑定需另行部署。`--listen`不控制运动；默认取配置中的灵敏档（官方两道判定一起调整），可用`--sensitivity normal`临时对照正常档。命令行选档仅对当前检测生效，不重写注册文件。日志记录档位、检测时长、周期统计和拒绝计数；计数不是识别准确率，需结合用户实际说话次数及内容判定漏检和误触发。

USB刷备用卡无需读卡器；进入BootROM后使用官方`k230-flash`，明确指定`SDCARD`、写入偏移`0x0`，不能沿用默认`EMMC`。重枚举权限安装一次：

```bash
sudo ./scripts/install-k230-usb-rules
```

烧录工具和镜像在本机`~/.local/share/aic-tools/k230-voice/`，原始日志、模型和录音不上传。检测准确率及播放尚未验证；旧示例的“小楠小楠”固定模型不能替代本次开放词汇模型。

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
./scripts/mspm0-build all                   # 编译三个工程
./scripts/mspm0-build uart_link             # 只编译通信固件
./scripts/mspm0-build motor_encoder         # 只编译独立按键电机固件
./scripts/mspm0-build motor_uart            # 编译K230控制底盘固件
./scripts/mspm0-tools python -m unittest discover -s device/mspm0/tests
```

复用现有 C 源码、引脚配置和 TI SDK 的 GCC 启动文件、链接脚本、驱动库；产物在各工程 `build/gcc/`，包括 ELF、HEX、BIN、构建日志与校验记录。三个GCC固件均已通过编译、Main Flash地址和中断向量核对。2026-10-20 已将 UART 固件在 Linux 烧录并逐字节回读通过，收到启动日志；电机固件同日烧录、回读通过，并完成车轮悬空的按键启动、PWM 缓升、5 秒自动停车和编码器正反计数检查。

2026-10-20 已在 UART 固件上实测 GDB 硬件断点与指令单步：命中 `main`，单步 PC 从 `0x28c` 前进到 `0x28e`，删除断点后恢复运行；未改写 Flash。VS Code 图形界面的 F5 操作尚未现场验证。

VS Code 已配置 `Ctrl+Shift+B` 编译任务和 UART / 电机两项 Cortex-Debug 启动项。调试启动会连接并下载固件，等现场就位后再使用。配置模板在 `configs/mspm0/vscode-*.json`；本机 `.vscode/` 不提交。Keil Studio 提供 IDE 工具，现有工程通过上述 Python/GCC 入口构建，未转换为 CMSIS Solution。

本机 USB 权限规则已安装。其他 Linux 电脑需输入 sudo 密码安装一次，再重插 fireDAP：

```bash
sudo ./scripts/install-debug-usb-rules
```

本机用户已属于 `plugdev` 和 `dialout`。pyOCD 配置为 `configs/mspm0/pyocd.yaml`（100 kHz，关闭自动解锁，使用页擦除）。2026-10-20 已通过 fireDAP 连接并读取 DP ID `0x6BA02477`、CPUID `0x410CC601` 和 128 KiB Flash / 32 KiB SRAM 容量。随后完成 UART 固件烧录、全 Flash 回读和启动日志检查；只有前 2 KiB 被改写，BCR/BSL 配置未变化。原程序备份与记录保存在本机 `data/logs/mspm0_uart_bringup_*/`。

**调试连接会干扰运行**：当前 TI 设备包的连接序列会操作复位；实测 `attach` 后内核处于暂停状态，不能当作无侵入读取。通信测试前需明确恢复运行；电机运行中不要新连接调试器、打断点或单步。一次保持同一 SWD 会话并明确恢复运行的通信复测为 20/20，接收/应答计数各增加 20、丢弃计数为 0；退出调试器后复测也为 20/20。随后在不连接 SWD 的冷上电测试中复现并修复了帧前 `00` 导致的首包丢弃，详见通信例程。

工具下载来源和 SHA256 见 `configs/mspm0/tools-lock.json`，Python 工具版本见 `configs/mspm0/requirements-tools.txt`。现有生成的 C 配置可直接编译，SysConfig 和 CCS 暂不需要安装。

K230 接入 OTG USB、CanMV IDE 停止并断开后，可直接从 Linux 运行双板测试：

```bash
./scripts/host tools/diagnostics/test_k230_mspm0_link.py
```

当前MSPM0运行 [motor_uart 联调固件](../../device/mspm0/motor_uart/README.md)，支持K230启停、状态与编码器回传，已完成架空轮实测。上述纯通信命令要求先另行烧录 `uart_link`，不能直接对当前电机固件运行。

当前固件的只读检查：`./scripts/host tools/diagnostics/test_k230_motor_uart.py`；车轮可靠架空后，增加 `--execute --wheels-lifted` 验证启停、5秒运行时限及1秒心跳丢失停车。

复用板端既有 20 轮测试，不控制舵机。需要每轮带序号与令牌的 `MSPM0_ACK`，20/20 才通过；日志在 `data/logs/k230_mspm0_link_*/`。冷上电首包已通过针对性复核：日志捕获帧前 `00`，新解析器正确接收首包并达到 20/20。同步读取 UART0 的命令见 [通信例程](../../device/mspm0/uart_link/README.md)。

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

## 当前下载方式：主控 USB / BSL

fireDAP 当前停用。天猛星自身 Type-C 带 CH340 串口，可在 Linux 下载；主控由 USB 供电时断开其他外部 5V，保留两板 GND 共地。

在仓库根目录先启动命令，看到 `READY` 后，再按住 **BSL → 按下并松开 RST → 松开 BSL**：

```bash
./scripts/mspm0-build uart_link
./scripts/mspm0-tools python tools/diagnostics/mspm0_bsl.py \
  --hex device/mspm0/uart_link/build/gcc/uart1_link.hex --execute
```

本机默认主控端口为 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`，其他端口用 `--port`。工具按 9600/8N1 等待 BSL，使用默认密码；失败不会反复猜密码。只擦写映像覆盖的 Main Flash 页，不发送全片擦除、工厂复位或 NONMAIN 写入命令。写入后重新握手、核对两次映像 CRC 与范围外 Main Flash CRC，全部匹配才启动应用。

本次 BSL 写入与另起会话的独立重复核验已真机通过。曾出现写后第一次 CRC 不一致，原失败日志已保留；遇到此情况可只核验、不重新擦写：

```bash
./scripts/mspm0-tools python tools/diagnostics/mspm0_bsl.py \
  --hex device/mspm0/uart_link/build/gcc/uart1_link.hex --verify-only --start-app
```

不带 `--start-app` 时只核验。BSL 默认禁止逐字节读出，工具不修改这一策略；CRC 核验不能表述为逐字节回读。记录保存在本机 `data/logs/mspm0_bsl_*/`。

也可用 Chrome/Edge 的 [立创在线烧录工具](https://wiki.lckfb.com/storage/html/mspm0-web-flasher/index.html)选择 HEX；其一键烧录包含 Main Flash 全片擦除。参考：[立创 Type-C / BSL](https://wiki.lckfb.com/zh-hans/web-tool/mspm0-web-flasher/introduce.html)、[TI BSL 协议](https://www.ti.com/lit/ug/slau887a/slau887a.pdf)、[fireDAP 操作说明](https://doc.embedfire.com/stm32_products/must_read/zh/latest/doc/quickstart/DAP/DAP.html)。
