# PC 机械臂代码工作规则

- 所有运行代码、配置、模型从仓库 `aic-robot/` 根目录解析，不读取相邻 USB 摄像头项目或旧独立工作区。
- 机械臂控制设计由用户主导；PID、速度/加速度、阶段、保护和持物策略不自行新增或改变。
- 运行参数为 `configs/arm/user_selected_pid_demo_baseline.json` 与 `motion_defaults.json`；每阶段一次同步最终目标，另行回中立同配置。
- 小车行驶时保持用户准备位和扭矩；根目录 `./scripts/arm-ready --execute`，由人确认到位。准备位不是中立位，不自行新增小车联动或持物行驶方案。
- 默认抓取 `./scripts/pick-blue`：K230 提取特征，PC 解算与控制。保留 PC 识图兼容入口；30°/2389 张开，模型下探15mm，径向额外伸展10mm，闭合20计数，夹爪静止0.5秒后抬升。
- 运行内外参为 `configs/arm/k230_pick_config.json` 指定的 `configs/vision/runtime/` 配套冻结快照，不用历史候选覆盖。
- 两种采集语义不得混用：grasp_pairs 是两指尖末端中点近似底面中心，再上移10mm；fixed_tip_top_pairs 是同一固定爪末端轻触方块顶面中心，再下移10mm。代理接触点不能当实测 TCP。
- 采集只拍照、读编码，不驱动机械臂；真机动作须有用户明确要求。
- 旧 PID 实验与 `.local_archive/` 不复制、恢复、提交或上传。原始照片/编码、历史分析与日志仅留本地；当前运行标定必须纳入 Git。
- `scripts/host` 和 `scripts/vision` 分开环境；板端 CanMV 脚本不导入 PC pyserial / SciPy。
