# AIC 小车与机械臂

WHU-SAS 团队的小车、机械臂与 CanMV K230 项目。工作、提交和上传均在本仓库根目录 `aic-robot/` 完成。

## 常用入口（Linux PC）

```bash
cd /home/xd/Desktop/aic-robot
./scripts/check                                 # 离线检查与测试，不访问硬件
./scripts/pick-blue --detect-only                # K230 识别，不打开舵机串口
./scripts/pick-blue --execute --hands-free       # 现场就位后执行抓取
./scripts/arm-ready --execute                    # 小车行驶前回准备位
```

抓取仍为 **K230 提取角点 → PC OpenCV 位姿 → PC FK/IK → USB 舵机控制**，未迁移成 K230 脱离 PC 的自主抓取。安装、串口权限与接线分别见 [Linux 指南](docs/guides/linux-arm.md)、[硬件说明](docs/hardware/connections.md)。

**小车行驶时，机械臂必须保持准备位与扭矩以维持平衡。** 准备位六轴为 `0°、−80°、80°、80°、0°、30°`，不同于中立位。脚本发目标后保持扭矩，由人确认到位；尚未自动联动小车启动。抓取在停车后进行，持物行驶策略另行确定。见 [准备位说明](docs/guides/arm/准备位脚本.md)。

## 功能与状态

| 功能 | 平台 / 说明 | 当前结果 |
| --- | --- | --- |
| 蓝色 20 mm 方块抓取 | Linux PC + K230；[详细流程](docs/guides/arm/K230_ONBOARD_PICK.md) | 2026-10-13 用户接受；尚未实现指定位置放置 |
| K230 直连舵机，只读 | CanMV；[板端指南](docs/guides/k230-and-mspm0.md) | 2026-10-19 读取 6/6 通过，未验证原生运动 |
| 双电机 / 编码器 | MSPM0 + Windows Keil；[示例](device/mspm0/motor_encoder/README.md) | 已有编译与主机逻辑测试记录；实物验收未记录完成 |
| K230 ↔ MSPM0 UART | CanMV + Windows Keil；[示例](device/mspm0/uart_link/README.md) | 固件曾编译；SWD 下载失败，双板通信尚未验证 |

完整证据与限制见 [状态表](docs/status.md)。目录迁移不代表新增真机验证。

## 目录

| 目录 | 职责 |
| --- | --- |
| `device/k230/` | 真正在 CanMV 上执行的视觉与诊断脚本 |
| `device/mspm0/` | 电机 / 编码器、UART 固件及共享启动 / scatter 文件 |
| `host/apps/` | PC 抓取、准备位、任务与只读探测入口 |
| `host/arm/`、`host/vision/`、`host/transport/` | FK/IK 与控制、视觉解算、USB / 舵机通信 |
| `host/tests/` | 不访问硬件的测试与迁移数值基准 |
| `tools/` | 采集、标定、分析工具；不作为运行库依赖 |
| `configs/arm/`、`configs/vision/runtime/` | 用户控制参数、关节映射、准备位与冻结运行标定 |
| `models/genkiarm/` | 当前 URDF 与全部 STL |
| `third_party/` | 当前补丁版 FTServo、完整 TI SDK、隔离的旧参考文件 |
| `hardware/car/` | 全部 17 个小车硬件资料 |
| `docs/`、`scripts/` | [文档导航](docs/README.md) 与统一操作入口 |
| `car-arm/` | 旧命令兼容转发，无第二套控制实现 |

迁移清单、来源与验证见 [整理记录](docs/reorganization.md)。`data/` 为本机采集、日志与进程锁，自动生成且不提交。旧数据未自动从其他项目导入。

## 当前控制行为

PID / 积分限幅以 [基线](configs/arm/user_selected_pid_demo_baseline.json) 为准，速度 / 加速度以 [运动默认值](configs/arm/motion_defaults.json) 为准。每阶段一次同步发送最终目标。抓取保持用户接受的 30° 张开、15 mm 模型下探、XY 径向额外伸展 10 mm、额外闭合 20 计数；夹爪静止 0.5 秒后抬升，结束保持扭矩。实际夹起仍由人确认。

机械臂控制设计由用户主导，结构整理不改参数、动作和控制策略。保留 host / vision 两套环境，不混用 NumPy。

## 团队协作与资料

先同步 main。按用户要求，后续完成验证后直接合入并上传 main，无需用户手动合并；遇到真实冲突或无法通过的必要验证再说明。原始照片 / 编码、运行日志、依赖环境、构建产物、密钥与探针本机配置不提交；旧 PID 实验不复制、恢复或上传。

完整 TI SDK、小车 17 个资料及运行模型继续纳入 Git。第三方许可和本地修改见 [third_party](third_party/README.md)，公开发布前核对模型与硬件资料授权。本私有仓库不统一声明开源许可证。
