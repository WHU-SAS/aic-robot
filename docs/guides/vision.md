# 视觉模块

目标为桌面上的20mm蓝色方块。K230可独立完成精定位与语音抓取；小车和机械臂共用相机、标定及精定位接口，整车任务见[完整任务](robot-task.md)。

## 入口与职责

| 用途 | 入口 / 实现 |
| --- | --- |
| 独立语音抓取 | [操作与动作流程](arm/K230_STANDALONE_PICK.md)；`device/k230/pick/app.py` |
| PC实物对照 | [PC验证入口](arm/PC_PICK_VALIDATION.md)；`./scripts/pick-pc` |
| 远处粗跟踪、小车接近 | [小车模块](car-approach.md)；`device/k230/car/` |
| 共用精定位 | `device/k230/vision/runtime.py` → `device/k230/pick/vision.py` |
| 第0版特征识别演示 | [第0版说明](../../demos/v0_blue_pick/README.md)；`device/k230/vision/blue_detector.py`、`blue_face_core.py` |

第0版的三帧特征匹配和正式版的单张JPEG物理面识别是不同链路，不能用其中一个的参数描述另一个。抓取偏移属于机械臂规划，不进入视觉原始坐标。

## 共用结果与相机

`CameraSession`在同一运行进程内只允许一个活动所有者；任务复用实例，不能重复初始化或释放相机。`capture()`返回`(image, frame)`：图像为借用的原生RGB565帧，在下一次snapshot或关闭之前完成检测或保存。

| 接口 | 输入与结果 |
| --- | --- |
| `track_coarse(image, frame, detector)` | 原生帧及注入的`tracker.detect_frame`；返回`tracking/confirming/lost/no_target` |
| `localize_precise(bgr, frame, intrinsics, extrinsics, table_plane=None)` | 1280×960 RGB565→JPEG95→BGR；返回`localized/no_complete_face` |

结果协议为`aic.vision_observation.v1`：

- `mode`、`status`、`frame_id`、`image_size_px`标明能力、状态和帧来源。
- `capture_timestamp_ms`及兼容别名`timestamp_ms`均为拍摄调用时间。板端`clock_domain=k230_ticks_ms`、`timestamp_source=software_snapshot_call`；不是硬件曝光时间。未知旧图时间保持`None`，不以处理时间冒充采集时间。跨PC/板端或重启不能直接比较；板端时间差用`ticks_diff`。
- 精定位成功返回`cube_center_Base_mm`：Base毫米、方块几何中心、未经抓取补偿。`quality`记录重投影误差、选面和识别路径，`payload`保留原检测结果。
- 粗跟踪可返回`ground_contact_proxy_Base_mm`，不能冒充精定位中心。`lost/no_target`清除所选目标；精定位失败不携带旧XYZ。

PC单图入口、独立抓取及小车`observations.py`已接入该协议。部署时模块名为`vision_runtime.py`、`camera.py`。更新频率、有效期、模式切换和动作由任务层负责；共用层不初始化串口或发送动作。

## 精定位算法

1. 用蓝色连通区域及局部色调/亮度提出候选，优先搜索完整侧面；失败才进入旧恢复链。圆角造成底边初值缩短时，可用有原始轮廓支持的底线重新提出候选；窄侧面复用同一份顶侧分界采样，不另拍图或重复扫描。静态图若只提出方块整体外轮廓，复用同一份分界采样提出侧面；全局分界候选为空时也保留已取得的局部候选。反光两侧亮暗边混合时，从原64站剖面中拟合一条一致的内部分界。已有内部分界候选及行进测距路径保留原顺序。
2. 在原图测量四条边及顶侧分界。颜色阈值仅提供候选，缺边不按20mm尺寸补齐；边缘支持、直线拟合、交错采样一致性和位姿重投影RMS≤2px都须通过。静态侧面四边已测出但未通过正方形位姿检查时，复用右竖边原64站RGB剖面的局部峰，要求边内侧属于蓝色区域，优先选择直线拟合误差更小的连续边，再重新检查四边、位姿和桌面约束。原位姿通过时不触发这次局部恢复；行进测距不启用，默认总位姿求解次数仍不超过3次。
3. 优先侧面。两个可用侧面的面积比>1.25时取大面，否则分别求中心后等权平均；两面须共享棱边、法向近似正交、中心差≤8mm。没有可靠侧面才使用通过四边验证的顶面，包括旋转后的菱形投影。
4. 正式桌面配置按标定桌面方向筛选平面双解：方向容差20°、方块中心法向高度10±8mm。不使用当前舵机位置或历史目标选解，也不截断坐标。

主要文件：`pick/side_faces.py`负责侧面搜索与面积选择，`reflection_faces.py`负责反光/分界及边缘测量，`top_boundaries.py`验证顶面，`pose.py`解算位姿，`table_pose.py`处理桌面约束。PC与K230共用这些文件。

## 标定与采集

| 流程 | 配置 |
| --- | --- |
| 正式PC验证及独立抓取 | `configs/vision/trials/model_alignment_20261030.json`及配对外参 |
| 第0版演示 | `configs/vision/blue_cube.json` → `configs/vision/runtime/` |
| 共用内参 | `configs/vision/runtime/camera_intrinsics.json`；加载时核对外参中的内参SHA256 |

使用1280×960、不镜像、不翻转。相机对焦、成像模式或相机与底座相对位置变化后需复核标定。正式精定位不套用第0版前伸10mm。

```bash
./scripts/collect-tip-top
```

每次只采一组，不运动或写参数：固定爪末端轻触平放20mm方块顶面中心，自然支撑，拍照前后各读取三轮六轴反馈。方块中心近似在接触点沿桌面法向下10mm；接触点/CAD仍是代理参考。旧`grasp_pairs`的参考点含义不同，不能混用。

正式批次位于本机`data/k230/extrinsics/formal_fixed_tip_top_20261022/`。第1～16组中3/5/10/12组预留验证，其余12组拟合；17～20组仅验证冻结外参。原图、编码、采集快照和校验均保留，编码波动只报告，不自动删样本。

```bash
./scripts/vision tools/analysis/fit_formal_tip_top.py \
  --session data/k230/extrinsics/formal_fixed_tip_top_20261022 \
  --output data/development/tip_top_fit_新目录
```

上述工具复现采集时冻结模型的历史拟合，不会推广运行配置。当前修正模型重新拟合的标签一致性如下：

| 样本 | 水平RMS | 三维RMS | 最大三维误差 |
| --- | --- | --- | --- |
| 预留4组 | 2.99mm | 3.95mm | 4.75mm |
| 新增17～20组 | 4.31mm | 5.10mm | 6.37mm |

以上是对人工接触/CAD标签的一致性，不是独立实测抓取精度。

## 当前问题与回放

- 近左边界三张原图均可得到合格侧面；原始未补光图通过同一组采样中的连续轮廓边恢复，仍满足2px及桌面约束。历史同位置顶/侧结果曾差约6mm，回放恢复不等同于实物精度已确认。
- 暗光、反光与边界验证已有回归覆盖，仍可能明确拒绝不可靠图像；PC/板端同图数值一致不等于现场抓取准确。
- 运行记录见`data/logs/pick_pc_*`、`last_pick_evidence_*`和`vision_side_boundary_*`；定位日志保留角点、选面、重投影及分段耗时。

```bash
./scripts/vision host/vision/localize_pick_cube.py \
  --image /绝对路径/frame.jpg --output /tmp/vision_review \
  --config configs/vision/trials/model_alignment_20261030.json
```

第0版特征回放用`tools/analysis/replay_onboard_detector.py`；其`--source image`是PC图像适配，不能证明真实多帧稳定性。

历史原图、修复对照、旧部署和报告素材仅保存在本机`data/`。报告入口为`data/arm/report_assets/LATEST.txt`；不上传原图，也不在操作指南逐轮追加调试记录。

## 困难数据集（本机）

新版精定位逐轮保留原始JPEG95、采集元数据、识别结果、内外参快照及源码/部署版本，成功图也保留，供事后标记“识别成功但位置不准”。板端目录为`/sdcard/aic/vision/evidence/`：机械臂直接使用唯一文件完成识别；小车最多暂存3张精定位图，停车并关闭相机后落盘。完整任务的配置快照包含`car`和`arm`两部分，困难集从原`car`配置提取相机与桌面标定，同时完整保留原JSON。原图不覆盖、不自动清理；写入失败明确报告`vision_archive_error`，不额外改变动作条件。留档代码需随相应板端入口部署才生效。

汇总目录为`data/vision/hard_cases/`，只留本机，不Git。PC流程的精定位拒绝样本自动入集；板端目录导出后批量导入，定位成功但不准的轮次追加人工标签。已有历史失败与人工复查图；原标定缺失的旧图使用明确标注的对比配置，不计算跨标定坐标误差。旧预测不是实测真值，标记`selected`的待复查图不自动当作失败。

```bash
./scripts/vision-hard-cases add --source data/logs/某轮目录 --reason inaccurate --note "偏底座侧"
./scripts/vision-hard-cases import-evidence --source /已导出的板端目录/manifest.json
./scripts/vision-hard-cases replay
```

视觉算法更新后运行回放，检查识别恢复、选面、坐标变化和耗时；报告每次另存`reports/`。只有人工明确标注的预期结果才判通过/失败，未标注样本保持待复核；原图、旧结果和标签历史不随回放改写。
