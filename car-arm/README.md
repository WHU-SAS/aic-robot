# AIC car-arm

团队仓库模块路径：`car-arm/`，从独立项目提交 `445a2b9` 导入。

CanMV K230 + 六轴机械臂：识别桌面上的蓝色20mm方块，张开、下探夹取并抬升。当前版本已由用户现场运行并接受（2026-10-13）。指定位置放置功能留待下一阶段开发。

## 小车行驶前：回准备位（重要）

**小车行驶过程中，机械臂必须保持准备位并保持扭矩，用于维持整车平衡。** 在本模块目录执行：

```bash
./scripts/host hardware/move_ready.py --execute
```

行驶前先执行上述命令并确认机械臂已到位、停稳；行驶期间保持该姿态，抓取等改变姿态的操作在小车停稳后进行。脚本一次同步发送六轴最终目标后退出，扭矩继续保持。

准备位为1～6号 `0°、−80°、+80°、+80°、0°、+30°`，编码 `2048、1138、2958、2958、2048、2389`。这是专用行驶姿态，与六轴2048的中立位不同。

准备位包含6号张开30°目标；当前脚本不判断是否持物，持物行驶方案由用户另行确定。当前小车启动与准备位脚本尚未自动联动。详见[回准备位说明](docs/准备位脚本.md)。

## 抓取运行

从团队仓库根目录进入 `car-arm/`。IDE先停止脚本并断开连接，释放K230串口。

```bash
cd car-arm
sudo setfacl -m u:xd:rw \
  /dev/serial/by-id/usb-Kendryte_CanMV_001000000-if00 \
  /dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79033467-if00
./scripts/host hardware/k230_blue_pick_onboard.py --execute --hands-free
```

命令可重复运行，每次重新识别并读取当前编码。`--hands-free`表示现场已撤掉托扶力、手已离开运动区。完成后保持抬升姿态和扭矩。

只识别、不打开舵机串口：

```bash
./scripts/host hardware/k230_blue_pick_onboard.py --detect-only
```

USB重连后可能需要重新设置串口权限。设备名可通过 `--camera-port` / `--port` 指定。

## 当前执行逻辑

1. K230预热、最多采9帧，提取3帧一致的方块角点/棱线，正常仅传坐标。
2. PC用20mm尺寸、配套内外参解算底座坐标，优先整块三维立方体边线模型。
3. 目标沿底座指向方块的XY方向额外伸展10mm；原视觉位置单独保留。
4. 读取六轴编码、核对已保存参数，规划完整四阶段关节目标。
5. 每阶段一次同步发送最终目标：张开30°并悬停 → 下探 → 闭合停稳 → 抬升。

| 设置 | 当前值 |
| --- | --- |
| 张开目标 | 30°，编码2389，实际计数量化约29.97° |
| 悬停 / 抬升规划高度 | 模型最低指尖80mm |
| 下探高度 | 模型最低指尖距估计桌面15mm |
| 伸展补偿 | 每次目标径向外移10mm，不累加 |
| 闭合 | 预计20mm接触目标再闭合20计数 |
| 抬升条件 | 6号连续0.5秒位置范围≤2计数，即允许±1计数 |
| 2、3、4号速度 / 加速度 | 600 / 20 |
| 1、5、6号速度 / 加速度 | 150 / 5 |

PID等数值见 [当前基线](hardware/user_selected_pid_demo_baseline.json)，抓取设置见 [运行配置](hardware/k230_pick_config.json)。程序核对舵机参数，不写PID。实际夹起仍由现场确认。

## 目录

| 路径 | 内容 |
| --- | --- |
| `hardware/` | 抓取入口、关节映射、PID/速度配置、IK/FK与控制器 |
| `k230/onboard/` | K230板端识别程序 |
| `k230/*.py` | 预览、采集、标定、定位及分析工具 |
| `k230/calibration/runtime/` | 当前抓取使用的冻结内外参及来源哈希，纳入Git |
| `ros2_ws/src/arm_pkg/` | SDK、FK模块、URDF及STL；当前直接Python运行，无完整ROS2构建包 |
| `scripts/` | 主机/视觉环境入口和离线检查 |
| `docs/` | 当前状态、执行逻辑与Git整理说明 |

原始照片、配对编码、历史标定分析、运行日志、历史诊断和依赖环境保留本地，并由`.gitignore`排除。旧PID实验不纳入此项目或Git。

## 环境

本机 `scripts/host` 使用系统Python；`scripts/vision` 使用本地`.vision_deps`。在新电脑上可分别创建两个环境，两个入口会自动优先采用它们：

```bash
python3 -m venv .venv-host
.venv-host/bin/python -m pip install -r requirements-host.txt
python3 -m venv .venv-vision
.venv-vision/bin/python -m pip install -r requirements-vision.txt
```

两套NumPy版本分别供SciPy控制计算和OpenCV视觉使用。以上安装命令未在新电脑验证；仓库不携带已安装依赖。

## 离线检查和采集工具

```bash
./scripts/host scripts/offline_check.py
./scripts/host -m unittest discover -s hardware/tests
./scripts/host hardware/probe_linux_servos.py
./scripts/open_preview
./scripts/host k230/collect_raw_pair.py
```

前两项不访问硬件；探测只发送ping/读取请求。预览需要本机CanMV IDE，可通过`CANMV_IDE`指定可执行程序。采集只拍照、读编码，不驱动机械臂。

详细说明：[当前状态](docs/CURRENT_STATE.md)、[板端抓取流程](docs/K230_ONBOARD_PICK.md)、[PC兼容入口](docs/K230_PICK_TRIAL.md)、[Git整理](docs/GIT_PREP.md)。
