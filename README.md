# AIC 小车与机械臂

正式版目标：整车脱离PC，由K230运行视觉、语音与任务逻辑，MSPM0负责底盘。当前视觉目标为20mm蓝色方块。

第0版可行性演示已跑通：K230提取特征，PC解算位姿、FK/IK并控制机械臂；现作为参考演示保留。

## 当前情况

机械臂本阶段版本采用5号侧面旋转和0mm经验补偿；“回准备位”发送位1，“思达思达”先立即发送行驶用位2，再进入任务，均不等待准备位到位。小车接近、停车验窗后复用同帧精定位抓取；整车全链路及成功率仍待验证。见[整车完整任务](docs/guides/robot-task.md)。

| 项目 | 状态 |
| --- | --- |
| 第0版识别并夹起方块 | 已跑通，演示归入`demos/v0_blue_pick/`；结束保持抬升和扭矩 |
| 正式版视觉 | K230独立精定位已运行；车臂共享相机和原始精定位结果，圆角侧面初值已修复，部分保留停车图仍被桌面姿态约束拒绝 |
| 机械臂阶段基线 | 用户接受修正装配模型、25°开口、5号旋转及0mm经验补偿；完整任务默认采用，PC和独立抓取保留固定5号对照 |
| 正式外参候选 | 固定爪触顶20组；12组拟合、4组预留验证及4组新增验证，修正模型水平RMS约2.99/4.31mm（非实物抓取精度） |
| 夹住判断 | 正式PC、独立及整车入口按6号稳定停位差判断：Δ＜30未夹住，Δ≥30成功；未独立视觉验证 |
| 双准备位 | 位1为2/3/4号−85°/+85°/+85°；行驶位2为−80°/+80°/+80°，语音识别后先发目标再继续 |
| K230 直连舵机 | UART1回环10/10、六轴读取6/6；准备位同步发送及修正模型实物抓取已通过，板端与PC同图解算对照通过 |
| 底盘电机与编码器 | 当前运行`v2+MOVE+TURN+FOLLOW`固件，轮速PI、定距及单轮弧线定角已实现；USB/BSL下载与独立CRC核验通过，上电默认停车 |
| K230 ↔ MSPM0 | UART独立轮控制、状态与编码器回传通过；连续模式由FOLLOW启动、SPEED更新，20秒总时限及1秒失联停车保留，停车后SPEED不会自行重启 |
| K230 控制底盘 | 一次语音连续粗跟踪→一次移动细测→编码器补偿接近窗口中心→停后细测，最多微调一次；版本`16060786e9fe3e29`获用户现场认可，暂定为基线，尚不自动抓取 |
| 整车任务入口 | 组合包由“思达思达”先立即发准备位2，再接近→停车验窗→同帧精定位交臂→抓取抬升；“回准备位”调用位1，实物全链路及成功率待验证 |
| 独立定距与转向 | 100/150/300mm直行和左右5°/15°基础试跑已完成；盲走仍略左偏，编码器结果不作为外部物理距离或转角真值 |
| 演示素材 | K230原始H.264及时间戳保留，电脑逐帧合成跟框数据；后期分析不冒充K230实时检测性能 |
| K230 语音 | 备用卡已升级，开放词汇模型加载、麦克风及原有相机/UART只读复核通过；“思达思达”已注册，用户接受灵敏档初测效果及极近音词误触发；已用于独立语音抓取及准备位指令 |
| 软件检查 | 离线检查入口为`./scripts/check`；各模块另有针对性测试，结果以本次运行输出为准 |

**待推进：完整任务实物联调与成功率统计。** 指定位置放置与独立视觉持物验证尚未实现；当前夹住判断依据6号编码器阈值，同图回放一致、编码到位均不证明实物抓取精度。

## 常用命令

小车参数与当前验证边界见[小车接近与停车](docs/guides/car-approach.md)；新提速参数的实车计时、超过95%首次入窗均未统计验收。组合包在停后完整定位及验窗通过后调用机械臂，见[整车完整任务](docs/guides/robot-task.md)。

在仓库根目录执行；首次使用先看 [Linux 安装与运行](docs/guides/linux-arm.md)。

```bash
./scripts/check                            # 离线检查
./scripts/mspm0-build motor_uart            # 仅编译当前底盘固件，不下载
./scripts/host tools/deploy/k230_car_align.py --near-parking # 仅生成当前小车部署包
./scripts/deploy-robot                     # 仅生成完整任务包，默认5号旋转
./scripts/deploy-robot --install --activate # 更新整车程序（下次启动生效）
./scripts/k230-status                      # 纯接收实时状态，不中断K230
./scripts/enroll-ready                     # 录三遍“回准备位”；需退出状态监视
./scripts/deploy-pick --install --activate  # 独立语音抓取对照包，默认固定5号
./scripts/pick-pc --execute --hands-free    # K230识别、PC调度，默认固定5号
./scripts/demo-v0-pick --detect-only        # 第0版：仅识别
./scripts/demo-v0-pick --execute --hands-free # 第0版：完整抓取演示
./scripts/arm-ready --execute               # 准备位1
./scripts/arm-ready --pose 2 --execute      # 行驶准备位2
./scripts/pid-three --execute --label baseline # 三组固定下降姿态PID对照（约29秒）
```

小车行驶时保持 [准备位2与扭矩](docs/guides/arm/准备位脚本.md)，停车后抓取；准备位会张开夹爪，持物行驶策略待定。PID、速度和控制策略由用户决定。

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

## 源码可见与使用许可

本项目整体未采用开源许可证。团队原创且未另行授权的内容保留全部权利；源码可见不额外授予自由运行、复用、修改或分发的许可，另有法律规定或有效授权的除外。额外使用须取得相关权利人许可，详见 [版权与权限说明](LICENSE)。

公开仓库仍允许按 GitHub 服务条款查看和站内 Fork，也无法从技术上阻止下载。飞特 SDK、TI/CMSIS 等第三方内容继续适用其原许可证，不受本项目新增说明缩减；模型、图片和厂家资料的对外发布权限需单独核实。
