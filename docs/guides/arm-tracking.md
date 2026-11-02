# 下一版机械臂执行接口

已完成接口，供下一版整车持续追踪调用；候选包已安装并完成只读规划计时，执行入口尚未启用。当前硬件仍运行原四阶段版本；已有PC、独立语音和整车单次入口保持兼容。此接口不管理相机、目标选择、搜索或底盘。计时结果及尚存观察空档见[全流程追踪升级](full-process-tracking.md)。

## 调度交接

```python
from app import begin_from_observation_steps

planning = begin_from_observation_steps(arm_config, accepted_observation,
    target_version, source_config=car_config, emit=record)
progress = next(planning)  # 调度每次推进；间隔由整车安排观察。
if target_changed:
    planning.close()  # 取消纯规划，不发归位或舵机目标。
elif progress['stage'] == 'complete':
    arm = progress['execution']  # 此时仍未发动作。
    planning.close()             # 已移交的arm由上层负责关闭。
    state = arm.step(target_version=current_target_version)
```

新规划入口依次让出`planning_reference`、`planning_initialized`及逆解进度，只有末尾`complete`携带惰性执行器。事件绑定原`target_version/frame_id`；目标变化时关闭旧规划，不重新贴版本。旧`begin_from_observation`保留同步兼容。

按用户2026-11-02选择，所有K230在线规划入口（含旧单次和独立入口）无条件用`drive_ready_control.goals`作准备位2参考，初轮、恢复轮均不读取当前位置、不额外归位。`planning_reference`记录`source=ready_pose2_command`、参考计数和`measured=False`，不表示实物已到位。PC显式离线`start_counts`、纯Geometry接口，以及执行阶段真实反馈和夹持判据不变。

追踪静态精定位采用`tracking_task.static_localization_policy=shared_side_only`：共享完整侧面未通过就带`search/rejected`交整车换视角，不进入同图旧恢复穷举、不交付部分XYZ。旧独立与离线入口保留全量恢复，详见[视觉模块](vision.md)。

新入口只将几何搜索穷尽转换为`PickPlanUnavailable`，供整车换视角；标定、观测契约和配置错误保持原异常。旧同步`Geometry.plan`与新`plan_steps`消费同一套候选数学，选解顺序和报告值不变；底层FK/IK仍同步，时间片是软预算。相机始终归整车所有。

第一次执行器`step`才发送悬停目标，每次调用前传最新目标版本。`invalidate_target(reason)`撤销未发阶段，下一次`step`用准备位2覆盖旧目标，包括4号已发、其他五轴尚未发的200ms期间。

一步通常只发送一个阶段包或读取一轮反馈；发现空夹/失夹时，同一步立即发送恢复目标。步进无等待循环，但既有串口请求仍为同步、有原有超时，整车观察应在步进之间调度。观察调度方式和频率由整车决定。执行器不开关相机，不自行识别移动或把遮挡当移动。

## 动作与确认

| 阶段 | 推进条件 |
| --- | --- |
| 夹取前悬停 | 六轴同轮目标误差均≤30计数 |
| 下探 | 先发4号，至少200ms后再发其他五轴；六轴同轮误差均≤10计数才闭合 |
| 闭合 | 6号连续5次在同一±1带内，取末样本计算Δ＝实际位置−闭合目标；Δ<30立即恢复，≥30才抬升 |
| 抬升 | 1～5号同轮目标误差均≤10；6号单独满足Δ≥30，到位后在同一固定±1参考带连续稳定200ms才成功 |

抬升期间6号出带但Δ仍≥30，保持夹持、重新建立参考带并重计稳定时间；不以相邻读数差判断。上次有效轮读结束至本次轮读开始的调度空档超过既有轮询周期的两倍（当前100ms）时，重计稳定时间；本轮UART读取耗时另计，不将正常读操作误当漏调度。这仅影响成功证据，不触发恢复或任务终止。1～5号离开到位范围或读取失败会中断本次稳定确认，失败读取记未知，不能当空夹。6号有效读数Δ<30立即恢复。成功是用户指定的编码器判据，未增加视觉验收条件。

恢复同步发送准备位2（包含张爪），从实际发包时刻起500ms后`can_reobserve=True`。上层即可重新拍照定位，定位成功继续行驶，不等机械臂实际归位。无次数上限，不在臂内重定位或重复规划。正常唤醒发送准备位2后不等待的规则不变。

配置中`tracking_stage_policy`独立保存`lift_tolerance_counts=10`、`lift_gripper_stable_tolerance_counts=1`、`lift_gripper_stable_ms=200`、`recovery_observation_delay_ms=500`。抬升的±1暂定，后续可单独放宽，不影响闭合的5次稳定条件。PID、速度、几何、补偿不变。

## 状态与资源

上层已有疑似目标外移待确认时，可调用`step(target_version=..., allow_success=False)`。执行器仍读6号、复核原稳定条件并立即处理Δ<30；满足原判据则返回`success_ready=True`但保持运行，不提前结束。上层排除疑点后恢复默认`allow_success=True`，确认移位则更新目标版本触发原恢复流程；不新增固定等待或夹持阈值。

- `status`：`running`、`recovering`、`reobserve_ready`、`succeeded`、`fault`、`closed`。只有`succeeded`表示本轮成功；`fault`交由整车处理，不新增最终失败条件。
- `phase`和`pending_stage`区分当前阶段与未发阶段；`last_sample`含实际读取轴、轮读起止软件时刻及`read_ms`；样本实际间隔与调度空档分别记录。180ms轮读加50ms间隔意味着约230ms反馈周期，不能称为50ms采样，也不能把整轮完成时刻当成六轴同时采样。
- 当前读取接口未返回每次内部请求的重试历史；读取失败仍清空稳定确认，但成功返回前是否曾重试，不能仅据轮读耗时证明。最老轴样本的新鲜度与串口提速另行验证，不在此放宽阈值。
- `grasp_confirmed`是当前编码器判断，读取失败或发送会张爪的恢复目标后为`None`；`grip_confirmation`保留上一次证据及采样阶段/时刻，不代表恢复后的当前持物状态。
- 准备位2发出后不会再读取归位位置。用`recovery_goal_sent_ms`、`can_reobserve`和`next_poll_in_ms`衔接观察；时刻只在同一K230时钟域比较。
- `close()`仅释放UART并撤销未发动作，不发送停止、归位或断扭矩。需要恢复时先失效并执行`step`，不能把关闭执行器当实体停机。

离线覆盖假UART真实包编码、模拟时钟回绕、200/500ms边界、Δ29/30、固定参考带漂移、读取失败与旧计划撤销。板端性能、观察遮挡及准备位撤回路径仍待后续现场联调；本次未访问硬件。
