# 机械臂与共用视觉工作规则

## 仓库与职责

- 唯一开发/提交仓库是`/home/xd/Desktop/aic-robot`。运行资源从仓库根目录解析，不读取旧独立工程或USB项目；新K230代码不写回旧目录。
- 机械臂控制设计由用户主导。PID、速度/加速度、阶段条件、补偿、保护、归位和持物策略不自行改动。真机动作须有用户明确要求；采集只拍照、读编码。
- 机械臂Agent维护`device/k230/pick/`、机械臂与共用视觉；小车Agent维护`device/k230/car/`及底盘。共用文件改动按实际需要通知协调，不覆盖对方未提交文件。
- 本地`AGENT_COMMUNICATION.local.md`不Git，仅收到另一Agent通知时读取，不逐任务打开或轮询。

## 当前运行规则

- PID与运动参数以`configs/arm/user_selected_pid_demo_baseline.json`、`motion_defaults.json`为准。正常动作及另行回中立默认一次同步最终目标；独立下探采用下述4号提前时序。
- 独立抓取保持`9b7e306`动作基础，5号保持本轮读到的角度；暂停自动旋转/后续碰撞搜索。失败版本保留在Git及`archive/wrist-rotation-trial-20261023`标签，固定支点采集方案不恢复。
- 独立抓取：规划后直接同步悬停；六轴同一轮均在±30后下探；先发4号、200ms后发其他五轴；下探六轴均在±10后闭合；6号连续5次在同一±1带内（跨度≤2）后抬升。轮询50ms，读取失败不推进，不按超时跳段。
- 下探到位样本复用为`descend_position_sample`，不额外读取。抬升目标发送后只等1秒恢复语音，不当作到位确认；运动期间不监听或排队。
- “思达思达”触发单次抓取；“回准备位”直接同步准备位目标，无舵机前检/读回/到位等待，只保留报文和发送错误处理。两者均不写PID或开/关扭矩；故障不自动归位或卸力。
- 首次语音预热32段；动作结束重建音频与识别引擎并reset后直接监听，不再次丢弃32段。
- 小车行驶保持`ready_pose.json`准备位及扭矩；PC入口为`./scripts/arm-ready --execute`，由人确认到位。准备位不是中立位，会张开夹爪30°，不自行增加持物行驶或底盘联动。
- 用户已授权确认需求的K230修改：验证后自行写入、回读校验、激活并确认监听恢复；部署不触发动作。仅文档/本机清理不部署或访问硬件。

## 视觉与对照入口

- 正式PC及独立抓取使用`pick_validation_pc.json`：张开中点、固定爪侧45mm（moving-side offset=-45）、额外前伸0mm、最低指尖80/15/80mm、夹紧20计数。阶段规则仍以对应入口为准。
- 2026-10-30装配修复对照通过`--pick-config configs/arm/pick_model_check.json`显式选择；含新URDF、重拟外参、零经验偏移，尚未场测/切换默认。保持原默认与小车坐标不变，说明见`docs/guides/arm/PC_PICK_VALIDATION.md`。
- 第0版`./scripts/demo-v0-pick`保留PC解算与原时序，专属配置在`demos/v0_blue_pick/`；前伸10mm，不能混入正式视觉坐标。
- 正式精定位使用`configs/vision/trials/fixed_tip_top_20261022.json`，第0版使用`blue_cube.json`。内外参配对、相机1280×960 RGB565→JPEG95→BGR路径不随意改变。
- 共用`CameraSession`仅一个活动所有者；粗跟踪代理不是精定位中心。采集时间不以处理时间替代，跨时钟/重启不能直接比较；未知旧图时间保持None。接口见`docs/guides/vision.md`。
- 保留侧面优先、面积比1.25、双侧面中心差≤8mm及四边/2px检查。正式桌面双解用标定桌面方向20°与高度10±8mm筛选，不用编码、历史目标或抓取成败选视觉解；原始标定拟合和第0版不启用此桌面约束。
- `grasp_pairs`与`fixed_tip_top_pairs`参考点不同，不混用；正式固定指尖触顶后沿桌面法向下10mm近似方块中心，不能当实测TCP。所有回放精度说明与实物验收区分。
- `scripts/pid-three`按`pid_three_actions.json`三个固定目标做对照，每段同步最终目标；动作5秒、准备位3.5秒，指标只记录不作门槛。沿用当前PID/扭矩，不每轮重生成目标，真机由用户亲自执行。

## 数据与文档

- `scripts/host`用于串口/FK/IK，`scripts/vision`用于PC OpenCV；板端不导入PC pyserial/SciPy。
- 原图、编码、失败素材、历史分析和部署记录仅留本机`data/`，当前运行标定纳入Git。旧PID实验与`.local_archive/`不复制、恢复或上传。
- 人读指南只写当前用法、关键边界和未解决问题，不逐轮追加调试日志；报告素材入口为`data/arm/report_assets/LATEST.txt`。不删除已归档原图和失败证据。
