# AIC car-arm 工作规则

- 本项目为 K230 项目；所有代码、模型、数据和配置从本根目录解析，不读取相邻 USB 摄像头项目。
- 小车行驶时机械臂必须保持用户指定准备位与扭矩以维持平衡；行驶前使用 `./scripts/host hardware/move_ready.py --execute` 并确认到位。抓取等改变姿态的操作在小车停稳后执行。准备位不是六轴2048中立位；不自行修改目标或新增小车联动、持物策略。
- 机械臂控制设计由用户主导。PID、速度/加速度、运行阶段与保护策略的改变由用户决定，不自行新增或调整。
- 当前参数在 `hardware/user_selected_pid_demo_baseline.json`；速度与加速度在 `hardware/motion_defaults.json`。正常动作和回中立使用相同配置、一次同步发送最终目标。
- 旧 PID 实验不属于本项目，不复制、恢复、提交或上传。`.local_archive/` 永远不导出。
- 采集原始图片与编码时不得驱动机械臂；只有用户明确要求真机动作时才执行。
- 当前默认抓取入口为 `hardware/k230_blue_pick_onboard.py`：K230提取角点，PC OpenCV计算位姿，PC执行FK/IK与串口控制。保留PC识图兼容入口。样本定义必须区分：`grasp_pairs/`旧模式为两指尖末端中点近似方块底面中心、沿桌面法向上移10mm；`fixed_tip_top_pairs/`新模式为同一个固定爪末端点轻触20mm方块顶面中心、沿桌面法向下移10mm。按对应采集定义分析，不混用标签。新模式模型接触点尚为代理，候选未自动替换默认外参。
- 当前抓取内外参为 `hardware/k230_pick_config.json` 指定的 `k230/calibration/runtime/` 快照；旧 `calibration_results/` 是本地历史分析，不用其候选覆盖当前运行参数。
- 用户已接受本轮抓取版本：开口30°/2389、下探模型15mm、径向伸展补偿10mm。后续控制参数改动继续由用户决定。
- 原始照片、编码、标定实验、运行日志、历史诊断只留本地，不纳入Git。当前运行标定快照必须纳入Git。
- `scripts/host` 用于串口、FK、SciPy；`scripts/vision` 用于 OpenCV，避免两套 NumPy 混用。
