# AIC 小车与机械臂

K230 提取蓝色 20 mm 方块特征，Linux PC 解算位姿、FK/IK 并通过 USB 控制机械臂；MSPM0 负责底盘。

## 当前情况

截至 2026-10-20，依据已有现场记录和本机离线检查：

| 项目 | 状态 |
| --- | --- |
| 识别并夹起方块 | 已跑通，2026-10-13 用户现场接受；结束保持抬升和扭矩 |
| 行驶准备位 | 入口已具备；由人确认到位，行驶期间保持姿态和扭矩 |
| K230 直连舵机 | UART1 回环 10/10、经 URT-2 只读六舵机 6/6 通过；运动仍由 PC 控制 |
| 底盘电机与编码器 | 2026-09-30 已记录按键、PWM、计数和前进测试；当前主控运行 UART 测试固件，不输出电机 PWM |
| K230 ↔ MSPM0 | 10 月 20 日 Linux 下 UART 固件烧录、回读和启动日志通过；双板通信待接入 K230 验证 |
| 软件检查 | Linux 离线检查和 52 项单元测试通过；两个 MSPM0 固件已用 Linux GCC 编译；UART 固件烧录及回读通过 |

**下一步：连接 K230，验证双板 UART 的 20 轮请求应答。** 指定位置放置、自动判断夹起和整车协同尚未实现。当前标定的历史 XY 拟合 RMS 约 8.4 mm，不等于实测抓取精度。

## 常用命令

在仓库根目录执行；首次使用先看 [Linux 安装与运行](docs/guides/linux-arm.md)。

```bash
./scripts/check                            # 离线检查
./scripts/pick-blue --detect-only           # 仅识别，不控制舵机
./scripts/pick-blue --execute --hands-free  # 识别、下探、夹取、抬升
./scripts/arm-ready --execute               # 小车行驶前回准备位
```

小车行驶时保持 [准备位与扭矩](docs/guides/arm/准备位脚本.md)，停车后抓取；准备位会张开夹爪，持物行驶策略待定。PID、速度和控制策略由用户决定。

## 文档与代码

| 需要了解什么 | 入口 |
| --- | --- |
| 抓取动作、偏移、日志 | [抓取流程](docs/guides/arm/K230_ONBOARD_PICK.md) |
| 当前 PID 和运动参数 | [舵机基线](docs/guides/arm/BASELINE.md) |
| 接线、供电、硬件型号 | [硬件接线](docs/hardware/connections.md) |
| CanMV 测试、Linux / Keil 编译和下载 | [板端联调](docs/guides/k230-and-mspm0.md) |
| 电机 / 编码器实验 | [底盘例程](device/mspm0/motor_encoder/README.md) |
| UART 协议与验收 | [双板通信例程](device/mspm0/uart_link/README.md) |
| SWD 故障证据 | [2026-10-19 联调摘要](docs/history/2026-10-19-hardware-bringup.md) |

`device/` 是板端代码，`host/` 是 PC 程序，`tools/` 是采集标定工具，`configs/` 是运行配置，`models/` 是机械臂模型，`hardware/` 是硬件资料。[第三方依赖](third_party/README.md) 单独存放，旧命令保留在 `car-arm/`。

开发、提交均使用本仓库，验证后直接上传 main。`data/`、依赖环境、构建输出、密钥和旧 PID 实验不提交；当前运行标定纳入 Git。
