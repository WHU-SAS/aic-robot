# AIC 小车与机械臂

K230 提取蓝色 20 mm 方块特征，Linux PC 解算位姿、FK/IK 并通过 USB 控制机械臂；MSPM0 负责底盘。

## 当前情况

截至 2026-10-20，依据已有现场记录和本机离线检查：

| 项目 | 状态 |
| --- | --- |
| 识别并夹起方块 | 已跑通，2026-10-13 用户现场接受；结束保持抬升和扭矩 |
| 行驶准备位 | 入口已具备；由人确认到位，行驶期间保持姿态和扭矩 |
| K230 直连舵机 | UART1回环10/10、六轴读取6/6；已通过K230开扭矩及一次同步发送准备位，抓取仍使用原PC入口 |
| 底盘电机与编码器 | 10 月 20 日 Linux 烧录/回读、悬空车轮按键启动、PWM 缓升、5 秒自动停车及编码器正反计数通过；当前主控已运行 K230 UART 电机联调固件，上电默认停车 |
| K230 ↔ MSPM0 | 10 月 20 日复现首包前多出 `00` 导致整行丢弃；接收端按帧头重同步后，确认彻底断电的首包与整轮 20/20 通过 |
| K230 控制底盘 | 架空轮启动、命令/B21停车、状态/编码器回传、5秒时限及1秒心跳丢失停车通过；按键停车后未自行重启 |
| 首次落地短前进 | 机械臂准备位下约1秒前进后停车；用户反馈无异常，停车后六轴仍保持扭矩 |
| K230 语音 | 备用卡已升级，开放词汇模型加载、麦克风及原有相机/UART只读复核通过；“思达思达”待录音注册，未绑定动作 |
| 软件检查 | Linux 离线检查、56项PC测试及13项MSPM0测试通过；三个MSPM0固件通过GCC编译和映像检查 |

**待推进：整车协同。** 指定位置放置、自动判断夹起和整车协同尚未实现。当前标定的历史 XY 拟合 RMS 约 8.4 mm，不等于实测抓取精度。

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
| K230 控制底盘启停 | [联调固件与命令](device/mspm0/motor_uart/README.md) |
| UART 协议与验收 | [双板通信例程](device/mspm0/uart_link/README.md) |
| SWD 故障证据 | [2026-10-19 联调摘要](docs/history/2026-10-19-hardware-bringup.md) |

`device/` 是板端代码，`host/` 是 PC 程序，`tools/` 是采集标定工具，`configs/` 是运行配置，`models/` 是机械臂模型，`hardware/` 是硬件资料。[第三方依赖](third_party/README.md) 单独存放，旧命令保留在 `car-arm/`。

开发、提交均使用本仓库，验证后直接上传 main。`data/`、依赖环境、构建输出、密钥和旧 PID 实验不提交；当前运行标定纳入 Git。
