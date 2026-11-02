# Linux 安装与运行

所有命令从仓库根目录执行。PC 分别通过 USB 连接 K230 和舵机转接板；抓取前在 CanMV IDE 停止程序并断开连接。

## 安装环境

```bash
cd /home/xd/Desktop/aic-robot
python3 -m venv .venv-host
.venv-host/bin/python -m pip install -r requirements-host.txt
python3 -m venv .venv-vision
.venv-vision/bin/python -m pip install -r requirements-vision.txt
```

`./scripts/host` 用于串口、FK/IK，`./scripts/vision` 用于 OpenCV，两套 NumPy 环境分开。已有电脑也支持系统 Python 与 `.vision_deps`；新电脑从零安装尚未验证。

USB 重连后可能需要重新设置权限。将用户名及设备路径换成本机值：

```bash
sudo setfacl -m u:xd:rw \
  /dev/serial/by-id/usb-Kendryte_CanMV_001000000-if00 \
  /dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79033467-if00
```

抓取入口可用 `--camera-port` / `--port` 指定相机 / 舵机端口。

## 第0版演示与日常检查

```bash
./scripts/check                            # 离线检查和单元测试
./scripts/host host/apps/probe_linux_servos.py  # 舵机只读检查
./scripts/pick-blue --detect-only           # 仅定位
./scripts/pick-blue --detect-only --debug-roi  # 定位并保存诊断图
./scripts/pick-blue --execute --hands-free  # 现场就位、手离开后抓取
./scripts/arm-ready --execute               # 准备位1
./scripts/arm-ready --pose 2 --execute      # 行驶准备位2
./scripts/open_preview                     # 打开 CanMV IDE 预览脚本，连接后运行
```

动作及结束状态见 [抓取流程](arm/K230_ONBOARD_PICK.md)、[准备位](arm/准备位脚本.md)。预览和抓取共用 K230 串口，不能同时占用。

兼容的 PC 识图入口为 `./scripts/host host/apps/k230_blue_pick.py --execute --hands-free`：从 K230 导出照片，由 PC OpenCV 提取物理面；后续定位、IK 和动作与默认入口共用。`host/apps/run_task.py` 是另一个逐阶段人工确认的任务入口。

## 标定采集与离线分析

在 CanMV IDE 运行 [capture_calibration.py](../../device/k230/diagnostics/capture_calibration.py)：640×480 预览，每 3 秒保存一张无标记原图到 `/sdcard/calibration/session_XXXX/`，点击停止结束，不覆盖旧照片。改变标定板位置、距离、倾角，每个姿态停留 3～6 秒，先拍 20～40 张；固定镜头焦点。该采集分辨率与抓取的 1280×960 不同，采集本身不更新运行内参。

导出 session 后执行：

```bash
./scripts/vision tools/calibration/select_calibration.py /路径/session_0001
./scripts/vision tools/calibration/audit_intrinsics.py --images /路径/accepted
./scripts/vision tools/analysis/replay_onboard_detector.py /路径/运行记录 --output data/k230/runtime_checks/replay
```

筛选使用 5×7 ChArUco、DICT_5X5_100，要求 24 个内角点、17 个标记及 3 px 边距；结果写入新建 `selection_时间/accepted/` 和 `selection.json`，原图保留。完整入镜不代表内参质量合格。未挂载 SD 卡时，停止程序并断电后可用读卡器导出。

`recheck_intrinsics.py` 默认与运行内参比较，可用 `--baseline` 指定基准。回放默认优先使用保存的多帧`board_features.json`；只有单图时会使用近似提取并重复该图，不能验证真实多帧稳定性。日志和新采集均保存在不提交的 `data/`。
