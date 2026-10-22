# AIC 小车与机械臂

正式版目标：整车脱离PC，由K230运行视觉、语音与任务逻辑，MSPM0负责底盘。当前视觉目标为20mm蓝色方块。

第0版可行性演示已跑通：K230提取特征，PC解算位姿、FK/IK并控制机械臂；现作为参考演示保留。

## 当前情况

截至 2026-10-22，依据已有现场记录和本机离线检查：

| 项目 | 状态 |
| --- | --- |
| 第0版识别并夹起方块 | 已跑通，演示归入`demos/v0_blue_pick/`；结束保持抬升和扭矩 |
| 正式版视觉 | 已移植PC识别、位姿与IK到K230，保存图像上数值对照通过；整车协同尚未接入 |
| K230独立语音抓取 | “思达思达”触发单次抓取代码已实现，当前配置为累计45mm固定爪侧偏移；首次完整语音实物动作待现场验证 |
| 正式外参候选 | 固定爪触顶16组已采集，12组拟合、4组验证；水平RMS约3.51mm，待实物精度复核 |
| 行驶准备位 | 入口已具备；“回准备位”已注册并部署，双指令监听已确认；语音归位动作待现场验证，到位后保持扭矩 |
| K230 直连舵机 | UART1回环10/10、六轴读取6/6；已通过K230开扭矩及一次同步发送准备位；PC抓取45mm补偿版已由用户实物确认，暂定为基线；K230独立版本同图解算对照通过 |
| 底盘电机与编码器 | 10 月 20 日 Linux 烧录/回读、悬空车轮按键启动、PWM 缓升、5 秒自动停车及编码器正反计数通过；当前主控已运行 K230 UART 电机联调固件，上电默认停车 |
| K230 ↔ MSPM0 | 10 月 20 日复现首包前多出 `00` 导致整行丢弃；接收端按帧头重同步后，确认彻底断电的首包与整轮 20/20 通过 |
| K230 控制底盘 | 架空轮启动、命令/B21停车、状态/编码器回传、5秒时限及1秒心跳丢失停车通过；按键停车后未自行重启 |
| 首次落地短前进 | 机械臂准备位下约1秒前进后停车；用户反馈无异常，停车后六轴仍保持扭矩 |
| K230 语音 | 备用卡已升级，开放词汇模型加载、麦克风及原有相机/UART只读复核通过；“思达思达”已注册，用户接受灵敏档初测效果及极近音词误触发；独立抓取入口已绑定单次触发，首次完整实物动作待验证 |
| 软件检查 | Linux 离线检查、116项PC测试及13项MSPM0测试通过；三个MSPM0固件通过GCC编译和映像检查 |

**待推进：正式版板端视觉定位与整车协同。** 指定位置放置、自动判断夹起和整车协同尚未实现。新外参留出验证水平RMS约3.51mm，不等于实测抓取精度。

## 常用命令

在仓库根目录执行；首次使用先看 [Linux 安装与运行](docs/guides/linux-arm.md)。

```bash
./scripts/check                            # 离线检查
./scripts/k230-status                      # 纯接收实时状态，不中断K230
./scripts/enroll-ready                     # 录三遍“回准备位”；需退出状态监视
./scripts/deploy-pick --install --activate  # 更新K230独立语音抓取（下次启动生效）
./scripts/pick-pc --execute --hands-free    # 新外参：PC全流程验证，额外前伸0mm
./scripts/demo-v0-pick --detect-only        # 第0版：仅识别
./scripts/demo-v0-pick --execute --hands-free # 第0版：完整抓取演示
./scripts/arm-ready --execute               # 小车行驶前回准备位
```

小车行驶时保持 [准备位与扭矩](docs/guides/arm/准备位脚本.md)，停车后抓取；准备位会张开夹爪，持物行驶策略待定。PID、速度和控制策略由用户决定。

## 文档与代码

| 需要了解什么 | 入口 |
| --- | --- |
| 正式版视觉、复用代码与回放 | [视觉模块](docs/guides/vision.md) |
| 第0版演示与基线 | [第0版说明](demos/v0_blue_pick/README.md) |
| K230独立语音抓取、部署与日志 | [独立抓取](docs/guides/arm/K230_STANDALONE_PICK.md) |
| 新外参PC抓取验证、偏移和日志 | [PC验证入口](docs/guides/arm/PC_PICK_VALIDATION.md) |
| 第0版抓取动作、偏移、日志 | [抓取流程](docs/guides/arm/K230_ONBOARD_PICK.md) |
| 当前 PID 和运动参数 | [舵机基线](docs/guides/arm/BASELINE.md) |
| 接线、供电、硬件型号 | [硬件接线](docs/hardware/connections.md) |
| CanMV 测试、Linux / Keil 编译和下载 | [板端联调](docs/guides/k230-and-mspm0.md) |
| 电机 / 编码器实验 | [底盘例程](device/mspm0/motor_encoder/README.md) |
| K230 控制底盘启停 | [联调固件与命令](device/mspm0/motor_uart/README.md) |
| UART 协议与验收 | [双板通信例程](device/mspm0/uart_link/README.md) |
| SWD 故障证据 | [2026-10-19 联调摘要](docs/history/2026-10-19-hardware-bringup.md) |

`device/` 是板端代码，`host/` 是 PC 程序，`tools/` 是采集标定工具，`configs/` 是运行配置，`models/` 是机械臂模型，`hardware/` 是硬件资料，`demos/` 保留可行性演示。[第三方依赖](third_party/README.md) 单独存放，旧命令保留在 `car-arm/`。

开发、提交均使用本仓库，验证后直接上传 main。`data/`、依赖环境、构建输出、密钥和旧 PID 实验不提交；当前运行标定纳入 Git。
