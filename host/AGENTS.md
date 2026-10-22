# PC 机械臂代码工作规则

- 所有运行代码、配置、模型从仓库 `aic-robot/` 根目录解析，不读取相邻 USB 摄像头项目或旧独立工作区。
- 机械臂控制设计由用户主导；PID、速度/加速度、阶段、保护和持物策略不自行新增或改变。
- 运行参数为 `configs/arm/user_selected_pid_demo_baseline.json` 与 `motion_defaults.json`；每阶段一次同步最终目标，另行回中立同配置。
- 小车行驶时保持用户准备位和扭矩；根目录 `./scripts/arm-ready --execute`，由人确认到位。准备位不是中立位，不自行新增小车联动或持物行驶方案。
- 第0版演示 `./scripts/demo-v0-pick`（兼容 `./scripts/pick-blue`）：K230 提取特征，PC 解算与控制。保留 PC 识图兼容入口；30°/2389 张开，模型下探15mm，径向额外伸展10mm，闭合20计数，夹爪静止0.5秒后抬升。
- 视觉配置由 `configs/vision/blue_cube.json` 独立加载；其中内外参指向 `configs/vision/runtime/` 第0版冻结参考，不用历史候选覆盖。第0版抓取专属参数位于 `demos/v0_blue_pick/pick.json`。
- 正式版目标是K230脱离PC独立运行，暂只识别20mm蓝色方块；PC位姿解算和IK保留为对照；`device/k230/pick/`已移植到板端并通过同图对照，不据此宣称整车协同或首次独立实物抓取已验收。
- 两种采集语义不得混用：grasp_pairs 是两指尖末端中点近似底面中心，再上移10mm；fixed_tip_top_pairs 是同一固定爪末端轻触方块顶面中心，再下移10mm。代理接触点不能当实测 TCP。
- 采集只拍照、读编码，不驱动机械臂；真机动作须有用户明确要求。
- 旧 PID 实验与 `.local_archive/` 不复制、恢复、提交或上传。原始照片/编码、历史分析与日志仅留本地；当前运行标定必须纳入 Git。
- `scripts/host` 和 `scripts/vision` 分开环境；板端 CanMV 脚本不导入 PC pyserial / SciPy。

- 新外参PC验证用 `./scripts/pick-pc` 和 `configs/arm/pick_validation_pc.json`：候选位于 `configs/vision/trials/`；用户指定额外前伸0mm，以张开30°时两指末端中点为参考，并按用户指定让方块向固定爪侧偏移45mm（配置moving-side offset=-45），保留15mm/80mm及20计数夹紧、静止0.5秒抬升，不覆盖第0版默认配置。

- 用户已选“思达思达”触发K230单次独立抓取。`scripts/deploy-pick`只部署和校验，下次启动监听；不在部署过程中发送运动或设置上电自动抓取。语音沿用已有注册及灵敏档，动作沿用45mm版本。

- 新增“回准备位”直接语音命令：独立注册ID ready，复用`ready_pose.json`和`device/k230/arm/ready_pose.py`，含夹爪张开30°、到位保持扭矩；不走视觉或启动底盘。`scripts/enroll-ready`只录音/识别，不运动；注册后需重新部署才进入正式监听。

- 用户2026-10-22明确取消语音“回准备位”的全部舵机检查：使用`run_ready_direct`，只一次同步最终目标，保持当前扭矩状态，禁止追加扭矩开启写入（已发生目标写入但动作中断）；无参数/位置/扭矩前检、无读回、无到位等待。仅保留2ms串口发送收尾及报文编码/写入失败处理。此修改仅作用于语音准备位，不改抓取阶段检查和原诊断工具。
