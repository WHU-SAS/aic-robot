# AIC 小车与机械臂

WHU-SAS 团队项目。K230 提取蓝色20mm方块特征，Linux PC 完成位姿解算、FK/IK 和舵机控制；MSPM0 负责底盘。

## 运行

在仓库根目录执行，环境与串口权限见 [安装指南](docs/guides/linux-arm.md)。

```bash
./scripts/check                            # 离线检查，不访问硬件
./scripts/pick-blue --detect-only           # 仅识别
./scripts/pick-blue --execute --hands-free  # 现场就位后抓取
./scripts/arm-ready --execute               # 小车行驶前回准备位
```

**小车行驶时保持准备位与扭矩，行驶前由人确认到位。** 抓取在停车后进行。准备位会张开夹爪，持物行驶策略尚待确定。详见 [准备位说明](docs/guides/arm/准备位脚本.md)。

抓取结束保持抬升与扭矩。PID、速度和控制策略由用户决定，配置在 `configs/arm/`。详见 [抓取流程](docs/guides/arm/K230_ONBOARD_PICK.md) 与 [当前状态](docs/status.md)。

## 目录

| 目录 | 内容 |
| --- | --- |
| `device/` | K230 板端脚本、MSPM0 固件与共享启动文件 |
| `host/` | PC 应用、运动学、视觉、通信与测试 |
| `tools/` | 采集、标定和分析工具 |
| `configs/` | 控制参数与配套运行标定 |
| `models/` | 当前 URDF / STL |
| `third_party/` | FTServo、完整 TI SDK 与隔离参考 |
| `hardware/` | 硬件资料 |
| `scripts/`、`docs/` | 环境 / 命令入口与[文档](docs/README.md) |
| `car-arm/` | 旧命令兼容入口 |

`data/` 保存本机采集、日志和锁，不提交。依赖环境、构建输出、密钥和旧 PID 实验也不提交。第三方许可见 [依赖说明](third_party/README.md)，公开发布前核对模型和硬件资料授权。

开发从本仓库开始，先同步 main；验证后直接上传 main。
