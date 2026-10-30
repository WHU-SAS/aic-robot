# AIC 小车与机械臂

正式版目标：整车脱离PC，由K230运行视觉、语音与任务逻辑，MSPM0负责底盘。当前视觉目标为20mm蓝色方块。

第0版可行性演示已跑通：K230提取特征，PC解算位姿、FK/IK并控制机械臂；现作为参考演示保留。

## 当前情况

小车独立接近/停车版本已获用户认可并暂定为基线；2026-10-31新增整车独立任务入口，接近、停车验窗后复用同帧精定位抓取，完整实物联调待验证。见[整车完整任务](docs/guides/robot-task.md)。

| 项目 | 状态 |
| --- | --- |
| 第0版识别并夹起方块 | 已跑通，演示归入`demos/v0_blue_pick/`；结束保持抬升和扭矩 |
| 正式版视觉 | K230独立精定位已运行；车臂共享相机和原始精定位结果，圆角侧面初值已修复，部分保留停车图仍被桌面姿态约束拒绝 |
| K230独立语音抓取 | 已进行多轮语音实物试跑，固定爪侧偏移45mm；暂停5号自动旋转，定位偏差和下探动态仍需改进 |
| 正式外参候选 | 固定爪触顶20组；12组拟合、4组预留验证及4组新增验证，水平RMS约3.51/3.87mm（非实物抓取精度） |
| 行驶准备位 | PC及“回准备位”语音入口均具备；语音直接同步目标并保持当前扭矩，到位由现场确认 |
| K230 直连舵机 | UART1回环10/10、六轴读取6/6；已通过K230开扭矩及一次同步发送准备位；PC抓取45mm补偿版已由用户实物确认，暂定为基线；K230独立版本同图解算对照通过 |
| 底盘电机与编码器 | 当前运行`v2+MOVE+TURN+FOLLOW`固件，轮速PI、定距及单轮弧线定角已实现；USB/BSL下载与独立CRC核验通过，上电默认停车 |
| K230 ↔ MSPM0 | UART独立轮控制、状态与编码器回传通过；连续模式由FOLLOW启动、SPEED更新，20秒总时限及1秒失联停车保留，停车后SPEED不会自行重启 |
| K230 控制底盘 | 一次语音连续粗跟踪→一次移动细测→编码器补偿接近窗口中心→停后细测，最多微调一次；版本`16060786e9fe3e29`获用户现场认可，暂定为基线，尚不自动抓取 |
| 整车任务入口 | 新组合包由“思达思达”触发接近→停车验窗→同帧精定位交臂→抓取抬升；保留单独准备位命令，实物全链路及成功率待验证 |
| 独立定距与转向 | 100/150/300mm直行和左右5°/15°基础试跑已完成；盲走仍略左偏，编码器结果不作为外部物理距离或转角真值 |
| 演示素材 | K230原始H.264及时间戳保留，电脑逐帧合成跟框数据；后期分析不冒充K230实时检测性能 |
| K230 语音 | 备用卡已升级，开放词汇模型加载、麦克风及原有相机/UART只读复核通过；“思达思达”已注册，用户接受灵敏档初测效果及极近音词误触发；已用于独立语音抓取及准备位指令 |
| 软件检查 | 离线检查入口为`./scripts/check`；各模块另有针对性测试，结果以本次运行输出为准 |

**待推进：完整任务实物联调及定位/下探问题复核。** 指定位置放置和自动判断夹起尚未实现；同图回放一致、编码到位均不证明实物抓取精度。

## 常用命令

独立小车包由“思达思达”触发固定蓝块接近：远段70mm/s；细测交接后，剩余60～40mm连续降至40mm/s，最后40mm保持40mm/s。组合包沿用该行驶逻辑，增加验窗通过后的机械臂抓取。粗测定方向，移动细测定距离，测距候选不能代替停后完整定位。超过95%首次入窗尚未统计验收，全过程动态重获留后续。见[小车接近与停车](docs/guides/car-approach.md)及[整车完整任务](docs/guides/robot-task.md)。

在仓库根目录执行；首次使用先看 [Linux 安装与运行](docs/guides/linux-arm.md)。

```bash
./scripts/check                            # 离线检查
./scripts/mspm0-build motor_uart            # 仅编译当前底盘固件，不下载
./scripts/host tools/deploy/k230_car_align.py --near-parking # 仅生成当前小车部署包
./scripts/deploy-robot                     # 仅生成小车+机械臂完整任务包
./scripts/deploy-robot --install --activate # 更新整车程序（下次启动生效）
./scripts/k230-status                      # 纯接收实时状态，不中断K230
./scripts/enroll-ready                     # 录三遍“回准备位”；需退出状态监视
./scripts/deploy-pick --install --activate  # 更新K230独立语音抓取（下次启动生效）
./scripts/pick-pc --execute --hands-free    # 新外参：PC全流程验证，额外前伸0mm
./scripts/demo-v0-pick --detect-only        # 第0版：仅识别
./scripts/demo-v0-pick --execute --hands-free # 第0版：完整抓取演示
./scripts/arm-ready --execute               # 小车行驶前回准备位
./scripts/pid-three --execute --label baseline # 三组固定下降姿态PID对照（约29秒）
```

小车行驶时保持 [准备位与扭矩](docs/guides/arm/准备位脚本.md)，停车后抓取；准备位会张开夹爪，持物行驶策略待定。PID、速度和控制策略由用户决定。

## 文档与代码

| 需要了解什么 | 入口 |
| --- | --- |
| 正式版视觉、复用代码与回放 | [视觉模块](docs/guides/vision.md) |
| 整车独立接近、停车与抓取 | [完整任务](docs/guides/robot-task.md) |
| 第0版演示与基线 | [第0版说明](demos/v0_blue_pick/README.md) |
| K230独立语音抓取、部署与日志 | [独立抓取](docs/guides/arm/K230_STANDALONE_PICK.md) |
| 新外参PC抓取验证、偏移和日志 | [PC验证入口](docs/guides/arm/PC_PICK_VALIDATION.md) |
| 第0版抓取动作、偏移、日志 | [抓取流程](docs/guides/arm/K230_ONBOARD_PICK.md) |
| 三组固定动作 PID 对照 | [测试脚本](docs/guides/arm/PID_THREE_ACTIONS.md) |
| 当前 PID 和运动参数 | [舵机基线](docs/guides/arm/BASELINE.md) |
| 接线、供电、硬件型号 | [硬件接线](docs/hardware/connections.md) |
| CanMV 测试、Linux / Keil 编译和下载 | [板端联调](docs/guides/k230-and-mspm0.md) |
| 电机 / 编码器实验 | [底盘例程](device/mspm0/motor_encoder/README.md) |
| K230 控制底盘启停 | [联调固件与命令](device/mspm0/motor_uart/README.md) |
| UART 协议与验收 | [双板通信例程](device/mspm0/uart_link/README.md) |
| SWD 故障证据 | [2026-10-19 联调摘要](docs/history/2026-10-19-hardware-bringup.md) |

`device/` 是板端代码，`host/` 是 PC 程序，`tools/` 是采集标定工具，`configs/` 是运行配置，`models/` 是机械臂模型，`hardware/` 是硬件资料，`demos/` 保留可行性演示。[第三方依赖](third_party/README.md) 单独存放，旧命令保留在 `car-arm/`。

开发、提交均使用本仓库，验证后直接上传 main。`data/`、依赖环境、构建输出、密钥和旧 PID 实验不提交；当前运行标定纳入 Git。
