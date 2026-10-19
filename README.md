# AIC 小车与机械臂

WHU-SAS 团队的小车、机械臂与 CanMV K230 开发仓库。主要工作目录为本仓库，机械臂抓取模块目前位于 `car-arm/`。

## 从这里开始

| 要做的事 | 平台与入口 | 已有结果 / 当前边界 |
| --- | --- | --- |
| 蓝色 20 mm 方块抓取 | Linux PC + K230 + USB 舵机转接板；[运行指南](docs/guides/linux-arm.md) | 2026-10-13 用户接受；K230 提取角点，PC 解算与驱动，尚无指定位置放置 |
| 小车行驶前回准备位 | Linux PC；[准备位脚本](car-arm/docs/准备位脚本.md) | 脚本已实现，发目标后保持扭矩；由人确认到位，未与底盘启动联动 |
| K230 直连六舵机，只读 | CanMV IDE；[板端操作指南](docs/guides/k230-and-mspm0.md) | 2026-10-19 读取 6/6 通过；尚未完成脱离 PC 的抓取 |
| 双电机 / 编码器示例 | MSPM0 + Windows Keil；[示例说明](car/demo/01_motor_encoder/README.md) | 有编译与主机逻辑测试记录；实物电机验收未记录完成 |
| K230 ↔ MSPM0 串口测试 | CanMV + Windows Keil；[UART 测试说明](car/demo/02_uart1_link/README.md) | 固件编译通过，SWD 下载失败，双板通信尚未验证 |

验证日期、证据和阻塞项集中维护在 [当前状态](docs/status.md)。接线查 [硬件说明](docs/hardware/connections.md)，旧排障查 [历史索引](docs/history/README.md)。

## 常用机械臂命令

以下命令在 **Linux**、`car-arm/` 目录执行。环境安装、串口权限和设备连接见 [Linux 指南](docs/guides/linux-arm.md)。

```bash
cd car-arm
# 只识别，不打开舵机串口
./scripts/host hardware/k230_blue_pick_onboard.py --detect-only
# 真机抓取：现场就位、手已离开运动区时执行
./scripts/host hardware/k230_blue_pick_onboard.py --execute --hands-free
# 小车行驶前回准备位，执行后由人确认到位
./scripts/host hardware/move_ready.py --execute
```

**小车行驶过程中，机械臂必须保持准备位和扭矩以维持平衡。** 准备位为六轴 `0°、−80°、80°、80°、0°、30°`，不同于六轴 2048 中立位。抓取在小车停稳后进行。准备位会张开夹爪，持物行驶策略尚待用户确定；当前没有自动联动。

## 当前目录

| 路径 | 内容 |
| --- | --- |
| [car-arm/](car-arm/README.md) | 已有 PC 辅助抓取、运行配置、标定快照、模型与工具 |
| [arm/](arm/README.md) | 原始 SDK / URDF 参考与 K230 UART 只读诊断 |
| [car/demo/](car/demo/) | MSPM0 电机、编码器和 UART 示例；TI SDK 暂随电机示例保留 |
| [car/硬件信息/](car/硬件信息/) | 全部 17 个硬件资料文件，含 [现有硬件与功能](car/硬件信息/现有硬件与功能.txt) |
| [docs/](docs/) | 当前状态、接线、平台指南、迁移计划和历史记录 |

目录整理分批进行，**目标布局尚未落地**。见 [整理计划与路径清单](docs/reorganization.md)。第一批只改文档，不改控制代码、参数或运行入口。

## 协作和资料边界

同步 `main` 后在独立分支开发，通过 Pull Request 审查合并。机械臂 PID、速度/加速度、运行阶段与控制策略由用户主导，目录整理不改变其行为。

运行代码、测试、配置、配套运行标定、完整模型、必要 SDK 和许可证纳入 Git。原始采集照片/编码、运行日志、本机依赖、构建产物和调试器本机设置不提交；旧 PID 实验不复制、恢复或上传。完整 TI SDK 和小车 17 个硬件资料继续保留。完整 WHEELTEC 厂家资料包未整包导入。

仓库按私有团队开发资料管理。各第三方目录许可证适用于对应文件；公开发布前核对硬件资料和模型授权，不对整个仓库统一声明开源许可证。
