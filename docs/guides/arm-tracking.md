# 下一版机械臂执行接口

已完成离线接口，供下一版整车持续追踪调用，尚未部署。当前硬件仍运行原四阶段版本；已有PC、独立语音和整车单次入口保持兼容。此接口不管理相机、目标选择、搜索或底盘。

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

新规划入口依次让出`arm_snapshot`、`planning_initialized`及原逆解候选/阶段进度，只有末尾`complete`携带惰性执行器。所有事件固定原`target_version/frame_id`；目标变化时关闭规划生成器，不能把旧计划换上新版本继续。旧`begin_from_observation`仍是同步兼容入口。新入口仅把几何搜索穷尽的`NoPickSolution`转成`PickPlanUnavailable`，供整车换视角；标定、观测契约和配置错误保持原异常，不能靠挪车重试解决。规划期间相机保持归整车所有；旧同步`Geometry.plan`与新`plan_steps`消费同一套候选数学，根排序、tilt顺序、阶段求解与报告值不变。单次六轴读取或底层FK/IK仍同步，不能把软时间片当成硬抢占。第一次执行器`step`才开始悬停。每次调用前传最新目标版本，规划期间目标有变化也不能执行旧计划。`invalidate_target(reason)`立即撤销未发阶段，下一次`step`优先同步发送准备位2；已发送目标由该新目标覆盖，包括4号已发、其他五轴尚未发的200ms期间。

一步通常只发送一个阶段包或读取一轮反馈；发现空夹/失夹时，同一步立即发送恢复目标。步进无等待循环，但既有串口请求仍为同步、有原有超时，整车观察应在步进之间调度。观察调度方式和频率由整车决定。执行器不开关相机，不自行识别移动或把遮挡当移动。

## 动作与确认

| 阶段 | 推进条件 |
| --- | --- |
| 夹取前悬停 | 六轴同轮目标误差均≤30计数 |
| 下探 | 先发4号，至少200ms后再发其他五轴；六轴同轮误差均≤10计数才闭合 |
| 闭合 | 6号连续5次在同一±1带内，取末样本计算Δ＝实际位置−闭合目标；Δ<30立即恢复，≥30才抬升 |
| 抬升 | 1～5号同轮目标误差均≤10；6号单独满足Δ≥30，到位后在同一固定±1参考带连续稳定200ms才成功 |

抬升期间6号出带但Δ仍≥30，保持夹持、重新建立参考带并重计稳定时间；不以相邻读数差判断。有效采样间隔超过既有轮询周期的两倍（当前100ms）也重计稳定时间，不能把视觉或规划造成的无采样空档算进连续200ms；此项仅影响成功证据，不触发恢复或任务终止。1～5号离开到位范围或读取失败会中断本次稳定确认，失败读取记未知，不能当空夹。6号有效读数Δ<30立即恢复。成功是用户指定的编码器判据，未增加视觉验收条件。

恢复同步发送准备位2（包含张爪），从实际发包时刻起500ms后`can_reobserve=True`。上层即可重新拍照定位，定位成功继续行驶，不等机械臂实际归位。无次数上限，不在臂内重定位或重复规划。正常唤醒发送准备位2后不等待的规则不变。

配置中`tracking_stage_policy`独立保存`lift_tolerance_counts=10`、`lift_gripper_stable_tolerance_counts=1`、`lift_gripper_stable_ms=200`、`recovery_observation_delay_ms=500`。抬升的±1暂定，后续可单独放宽，不影响闭合的5次稳定条件。PID、速度、几何、补偿不变。

## 状态与资源

上层已有疑似目标外移待确认时，可调用`step(target_version=..., allow_success=False)`。执行器仍读6号、复核原稳定条件并立即处理Δ<30；满足原判据则返回`success_ready=True`但保持运行，不提前结束。上层排除疑点后恢复默认`allow_success=True`，确认移位则更新目标版本触发原恢复流程；不新增固定等待或夹持阈值。

- `status`：`running`、`recovering`、`reobserve_ready`、`succeeded`、`fault`、`closed`。只有`succeeded`表示本轮成功；`fault`交由整车处理，不新增最终失败条件。
- `phase`和`pending_stage`区分当前阶段与未发阶段；`last_sample`含实际采样轴及软件时刻，不能把未采样轴的旧位置当本轮读数。
- `grasp_confirmed`是当前编码器判断，读取失败或发送会张爪的恢复目标后为`None`；`grip_confirmation`保留上一次证据及采样阶段/时刻，不代表恢复后的当前持物状态。
- 准备位2发出后不会再读取归位位置。用`recovery_goal_sent_ms`、`can_reobserve`和`next_poll_in_ms`衔接观察；时刻只在同一K230时钟域比较。
- `close()`仅释放UART并撤销未发动作，不发送停止、归位或断扭矩。需要恢复时先失效并执行`step`，不能把关闭执行器当实体停机。

离线覆盖假UART真实包编码、模拟时钟回绕、200/500ms边界、Δ29/30、固定参考带漂移、读取失败与旧计划撤销。板端性能、观察遮挡及准备位撤回路径仍待后续现场联调；本次未访问硬件。
