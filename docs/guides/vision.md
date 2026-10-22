# 视觉模块

正式版目标：K230脱离PC独立运行，当前只识别20mm蓝色方块。

## 当前边界

- 已拆分可复用的单帧识别、多帧匹配和相机生命周期；识别核心保持第0版原样。
- 第0版完整演示在`demos/v0_blue_pick/`，仍依赖PC完成位姿解算和抓取规划。
- 板端OpenCV识别、位姿、FK/IK及语音单次抓取入口见[独立抓取](arm/K230_STANDALONE_PICK.md)，同图数值对照已通过；整车任务与媒体并行调度尚未实现。`CameraSession`当前由单次视觉任务独占管理，不与其他媒体任务同时启停。
- 第0版拆分保持原样；独立抓取另行移植当前PC识别链到`device/k230/pick/`，已在K230上回放保存图像，不把两套识别实现混称为同一模块。

## 代码职责

| 文件 | 职责 |
| --- | --- |
| `device/k230/vision/blue_face_core.py` | 蓝色分区、棱线、完整物理面和整块轮廓提取，核心保持原样 |
| `device/k230/vision/blue_detector.py` | `BlueCubeDetector.detect_frame(img)`单帧检测、多帧稳定匹配；不初始化硬件 |
| `device/k230/vision/camera.py` | `CameraSession`初始化一次、重复取帧、关闭资源 |
| `device/k230/vision/detect_blue_faces.py` | 第0版单次采样和特征/诊断图回传适配器 |
| `host/transport/board_vision_source.py` | PC调试时将板端模块打包到USBDBG；运行参数直接传入函数 |
| `host/vision/blue_side_edges.py` | PC图像后备识别：多列颜色分界线共识、独立边缘梯度和物理面校验 |
| `host/vision/localize_board_faces.py`、`cube_outline_pose.py` | PC位姿参考：多帧几何校验、20mm立方体解算、Camera到Base变换 |

识别链保留：LAB粗筛 → HSV分区 → 物理面/棱线 → 三帧匹配 → 几何与重投影校验。未验证的整块蓝色轮廓不当作一个方形物理面。失败原因和同帧诊断图继续保留。

`aic.blue_faces.v1`是特征数据协议版本，与“第0版可行性演示”的项目阶段分别编号。

## 配置与标定

`configs/vision/blue_cube.json`独立定义相机、20mm蓝色目标、检测区域、颜色粗筛和采样参数，PC视觉不再读取机械臂抓取配置。核心内其余已调好的阈值保持原值。

当前参考配置为1280×960 RGB565、不镜像、不翻转；ROI为`[0,432,1280,528]`；预热30帧，最多采9帧，仍要求三帧匹配。它沿用第0版工作区，不能据此认定已覆盖整车视野。

`configs/vision/runtime/`暂保留第0版内外参配对快照及哈希。改变对焦、相机与底座关系后应复核；其精度不等于正式版验收精度。演示的前伸10mm、下探15mm等参数在`demos/v0_blue_pick/pick.json`，未混入视觉原始坐标。

## 离线验证

```bash
./scripts/check
./scripts/vision tools/analysis/replay_onboard_detector.py /路径/运行记录 \
  --source features --output data/development/vision_replay
```

默认`--source auto`优先回放保存的`board_features.json`真实多帧记录；可用`--source image`回放单张完整JPEG。单图回放使用PC图像适配，不能证明真实多帧稳定性或固件LAB/矩形检测一致性。

2026-10-21整理前后：73组历史特征输出逐项一致（51组定位、22组拒绝）。原图帧提取也进行了相同离线适配条件下的前后对照。结果和原始样本仅留本机，不提交采集照片或旧PID实验。

## 后续开发顺序

1. 现场验证K230独立语音触发的完整实物抓取。
2. 继续评估不同位置的实测抓取偏差。
3. 接入整车任务，再验证语音、视觉与通信调度；机械臂控制策略由用户确定。

## 正式版外参采集

仅使用20mm方块平放桌面、固定爪末端轻触顶面中心。开口约30°，自然支撑，接触点在末端圆弧上的小范围变化按用户决定暂作误差接受。

```bash
./scripts/collect-tip-top
```

每次命令采一组：相机预热 → 三轮六轴编码/速度/扭矩读取 → 复制当前原图 → 三轮读取 → 原图保存SD并校验下载到电脑。由K230 UART1读取舵机；不发运动、扭矩或参数写入，不做识别、FK或外参拟合。IDE须停止并断开。

2026-10-22已实测采集16组，原图SHA256均通过、各组前后编码跨度均为0计数。当前批次位于本机`data/k230/extrinsics/formal_fixed_tip_top_20261022/`；可用`--session 路径`新建独立批次。每组保留原图、原始读数、板端时间戳、SHA256和日志；批次快照固定内参、映射、模型及采集代码。编码波动只报告，不自动删除样本。完成整批后统一分析；本批在16组中预先随机留出4组验证。

### 拟合与识别复核

```bash
./scripts/vision tools/analysis/fit_formal_tip_top.py \
  --session data/k230/extrinsics/formal_fixed_tip_top_20261022 \
  --output data/development/tip_top_fit_新输出目录
```

第3、5、10、12组在识别分析前随机指定为验证组，拟合及桌面法向估计仅使用其余12组。指尖仍为固定爪模型末端代理点；法向由训练组接触位置估计，不是独立实测。结果保存为候选，不自动替换运行外参。

第9、11组曾因颜色边界不稳定或顶面与侧面混合而失败。新增PC端后备识别依据多列蓝色/蓝色分界线共识及归一化颜色梯度测量四条边，保留重投影≤2px、边缘直线RMS≤3px要求。16组回放全部通过，原通过14组的角点及位姿不变；9、11组在55%～110%亮度回放中通过。合成几何测试覆盖曝光、指尖遮挡、纯蓝矩形、无目标和出框。第0版识别核心保持冻结；新增后备方法已随独立抓取另行移植并上板回放。

本批候选在4组留出样本上的水平RMS约3.51mm、三维RMS约4.38mm、最大三维偏差约5.55mm；这些是对人工接触/CAD标签的一致性误差，不是实物抓取精度。原图、完整候选及明细留在本机`data/`。用于实物验证的配对外参和桌面平面已另存到`configs/vision/trials/`，不覆盖第0版默认配置。

新外参的[PC全流程验证入口](arm/PC_PICK_VALIDATION.md)为`./scripts/pick-pc --execute --hands-free`；本次额外前伸和人工偏移默认均为0，真机全流程待验证。
