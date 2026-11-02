# 下一版机械臂执行接口

已完成离线代码，尚未接入整车持续追踪或部署。当前硬件仍运行原四阶段版本；已有PC、独立语音和整车单次入口保持兼容。此接口不管理相机、目标选择、搜索或底盘。

## 调度交接

```python
from app import begin_from_observation

arm = begin_from_observation(arm_config, accepted_observation,
                            target_version, source_config=car_config, emit=record)
try:
    while True:
        state = arm.step(target_version=current_target_version)
        # 在步进之间由整车调度观察；目标版本由整车维护。
        if state['status'] == 'succeeded':
            break
        if state['can_reobserve']:
            break  # 上层重新定位、创建新计划；不复用旧执行器。
        if state['status'] == 'fault':
            break  # 交还上层处理通信/执行故障，不代表整车任务放弃。
        # 按 next_poll_in_ms 安排下一步；期间可让出观察。
finally:
    arm.close()
```

构造先校验同帧精定位与标定配对，再读一次六轴并沿用原逆解；不发目标。第一次`step`才开始悬停。每次调用前传最新目标版本，规划期间目标有变化也不能执行旧计划。`invalidate_target(reason)`立即撤销未发阶段，下一次`step`优先同步发送准备位2；已发送目标由该新目标覆盖，包括4号已发、其他五轴尚未发的200ms期间。

一步通常只发送一个阶段包或读取一轮反馈；发现空夹/失夹时，同一步立即发送恢复目标。步进无等待循环，但既有串口请求仍为同步、有原有超时，整车观察应在步进之间调度。观察调度方式和频率由整车决定。执行器不开关相机，不自行识别移动或把遮挡当移动。

## 动作与确认

| 阶段 | 推进条件 |
| --- | --- |
| 夹取前悬停 | 六轴同轮目标误差均≤30计数 |
| 下探 | 先发4号，至少200ms后再发其他五轴；六轴同轮误差均≤10计数才闭合 |
| 闭合 | 6号连续5次在同一±1带内，取末样本计算Δ＝实际位置−闭合目标；Δ<30立即恢复，≥30才抬升 |
| 抬升 | 1～5号同轮目标误差均≤10；6号单独满足Δ≥30，到位后在同一固定±1参考带连续稳定200ms才成功 |

抬升期间6号出带但Δ仍≥30，保持夹持、重新建立参考带并重计稳定时间；不以相邻读数差判断。1～5号离开到位范围或读取失败会中断本次稳定确认，失败读取记未知，不能当空夹。6号有效读数Δ<30立即恢复。成功是用户指定的编码器判据，未增加视觉验收条件。

恢复同步发送准备位2（包含张爪），从实际发包时刻起500ms后`can_reobserve=True`。上层即可重新拍照定位，定位成功继续行驶，不等机械臂实际归位。无次数上限，不在臂内重定位或重复规划。正常唤醒发送准备位2后不等待的规则不变。

配置中`tracking_stage_policy`独立保存`lift_tolerance_counts=10`、`lift_gripper_stable_tolerance_counts=1`、`lift_gripper_stable_ms=200`、`recovery_observation_delay_ms=500`。抬升的±1暂定，后续可单独放宽，不影响闭合的5次稳定条件。PID、速度、几何、补偿不变。

## 状态与资源

- `status`：`running`、`recovering`、`reobserve_ready`、`succeeded`、`fault`、`closed`。只有`succeeded`表示本轮成功；`fault`交由整车处理，不新增最终失败条件。
- `phase`和`pending_stage`区分当前阶段与未发阶段；`last_sample`含实际采样轴及软件时刻，不能把未采样轴的旧位置当本轮读数。
- `grasp_confirmed`是当前编码器判断，读取失败或发送会张爪的恢复目标后为`None`；`grip_confirmation`保留上一次证据及采样阶段/时刻，不代表恢复后的当前持物状态。
- 准备位2发出后不会再读取归位位置。用`recovery_goal_sent_ms`、`can_reobserve`和`next_poll_in_ms`衔接观察；时刻只在同一K230时钟域比较。
- `close()`仅释放UART并撤销未发动作，不发送停止、归位或断扭矩。需要恢复时先失效并执行`step`，不能把关闭执行器当实体停机。

离线覆盖假UART真实包编码、模拟时钟回绕、200/500ms边界、Δ29/30、固定参考带漂移、读取失败与旧计划撤销。板端性能、观察遮挡及准备位撤回路径仍待后续现场联调；本次未访问硬件。
