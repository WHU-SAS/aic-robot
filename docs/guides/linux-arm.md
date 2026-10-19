# Linux：现有 PC 辅助抓取

当前源码位于仓库 `host/`、`device/`，配置位于 `configs/`，以下相对命令从仓库根目录开始。此入口使用 Linux 串口与进程接口，不作为 Windows / CanMV 原生运行命令。

## 角色与连接

K230 提取角点 / 棱线 → PC OpenCV 解算蓝色 20 mm 方块位姿 → PC FK/IK → USB 串口控制六轴。抓取时停止 CanMV IDE 脚本并断开 IDE，释放 K230 串口。

本模式 PC 分别通过 USB 连接 K230 与舵机转接板；K230 UART 直连诊断是 [另一条路线](k230-and-mspm0.md)，尚未替代该程序。

## 环境

```bash
cd /home/xd/Desktop/aic-robot
python3 -m venv .venv-host
.venv-host/bin/python -m pip install -r requirements-host.txt
python3 -m venv .venv-vision
.venv-vision/bin/python -m pip install -r requirements-vision.txt
```

`scripts/host` 用于串口、FK 与 SciPy；`scripts/vision` 用于 OpenCV。两套 NumPy 依赖分别维护，不合并环境。已有本机可沿用系统 Python 和 `.vision_deps` 回退方式；依赖目录不提交。2026-10-20 已有 Linux 环境离线检查通过，新电脑从零安装尚未验证。

## 检查与只读操作

```bash
# 以下两项不访问硬件
./scripts/host scripts/offline_check.py
./scripts/host -m unittest discover -s host/tests
# 相机识别，不打开舵机串口；IDE 须停止并断开
./scripts/pick-blue --detect-only
# 舵机仅 Ping / 读取，不发送运动目标
./scripts/host host/apps/probe_linux_servos.py
```

USB 连接或重连后，按本机实际用户和设备设置访问权限。本机示例：

```bash
sudo setfacl -m u:xd:rw   /dev/serial/by-id/usb-Kendryte_CanMV_001000000-if00   /dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79033467-if00
```

端口可用抓取入口的 `--camera-port` / `--port` 覆盖，用户名和 USB 序列号不是团队所有电脑的通用值。

## 真机抓取与准备位

现场就位、手已离开运动区时执行：

```bash
./scripts/pick-blue --execute --hands-free
```

每次重新定位，按四阶段执行，结束保持抬升和扭矩。当前没有指定位置放置、自动确认夹起、自动回中立或断扭矩。动作、参数和偏移含义见 [根 README](../../README.md) 与 [详细流程](arm/K230_ONBOARD_PICK.md)。

小车行驶前：

```bash
./scripts/arm-ready --execute
```

脚本一次同步发送准备位最终目标后退出，**不自动确认到位**。由人确认到位、停稳，并在行驶期间保持姿态和扭矩。准备位会张开夹爪；持物行驶另行确定。见 [准备位说明](arm/准备位脚本.md)。

可直接使用根入口 `./scripts/pick-blue`、`./scripts/arm-ready`、`./scripts/check`。旧 car-arm 命令由薄兼容层转发。

## 离线标定与回放

工具显式接收数据路径，输出写入本机 data/：

```bash
./scripts/vision tools/calibration/audit_intrinsics.py --images /路径/accepted
./scripts/vision tools/analysis/replay_onboard_detector.py /路径/运行记录 --output data/k230/runtime_checks/replay
```

`recheck_intrinsics.py` 默认与当前运行内参比较，可用 `--baseline` 指定旧基准。回放重复同一张保存图，只验证离线提取与定位流程，不验证真实三帧稳定性。
