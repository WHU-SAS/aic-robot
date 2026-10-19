#!/usr/bin/env python3
"""Plan or execute a six-axis task using the saved PID and motion defaults."""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT as REPO_ROOT, bootstrap as _bootstrap
_bootstrap(include_tools=False)
import argparse
import fcntl
import json
import signal
import select
import time
import sys
import termios
from datetime import datetime
from pathlib import Path

ROOT = REPO_ROOT
from task_controller import Configuration, FloorCheck, TaskController, TaskError
from host.transport.servo_bus import ServoBus

DEFAULT_PORT = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79033467-if00'


def select_route_check(config, data, steps, hold_at_last_target,
                       ready_pose_without_model_floor=False, finger_floor_m=.03):
    """The user-approved exception is limited to one held ready-pose stage."""
    if not ready_pose_without_model_floor:
        return FloorCheck(config, finger_floor_m=finger_floor_m)
    import math
    expected = {i:config.mapping.radians_to_count(i, math.radians(deg))
                for i,deg in enumerate([0,-80,80,80,0,30], 1)}
    if (not hold_at_last_target or data.get('name') != 'ready_pose'
            or len(steps) != 1 or steps[0]['name'] != 'ready_pose'
            or steps[0]['goals'] != expected):
        raise TaskError('model floor exception is authorized only for the single held user-defined ready pose')

    def operator_reviewed_route(start, target):
        # Target and device limits are still checked by TaskController. This
        # annotation is explicitly not a geometric clearance certificate.
        if target != expected:
            raise TaskError('ready-pose floor exception cannot apply to another target')
        return {'status':'not_checked_by_user_authorization',
                'scope':'ready_pose_only', 'model_floor_check_enabled':False,
                'reason':'User approved removing inherited model floor gate for ready pose; no geometric clearance claim.'}
    return operator_reviewed_route


def dispatch_ready_immediately(controller, steps):
    """Command invocation authorizes this one ready goal; no stdin or arrival claim."""
    import math
    expected = {i:controller.config.mapping.radians_to_count(i, math.radians(deg))
                for i,deg in enumerate([0,-80,80,80,0,30], 1)}
    if len(steps) != 1 or steps[0]['name'] != 'ready_pose' or steps[0]['goals'] != expected:
        raise TaskError('immediate dispatch is authorized only for the single user-defined ready pose')
    fresh = controller.sample()
    if any(row['torque'] != 1 for row in fresh.values()):
        raise TaskError('six-axis torque enable not confirmed')
    controller.config.check_counts(expected, controller.limits)
    controller.send(expected, 'ready_pose')
    controller.emit({'event':'ready_pose_goal_sent_torque_retained', 'goals':expected,
                     'notice':'Final target sent; no input prompt, neutral return or torque release; not an arrival claim.'})
    return 'ready_pose_goal_sent_torque_retained'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--task', type=Path, help='JSON file of named six-axis final goals')
    group.add_argument('--neutral', action='store_true', help='one synchronized neutral return')
    parser.add_argument('--execute', action='store_true', help='open hardware; otherwise offline only')
    parser.add_argument('--start-counts', nargs=6, type=int, default=[2048]*6,
                        help='offline geometry starting pose; execution always reads actual pose')
    parser.add_argument('--hold-at-last-target', action='store_true',
                        help='task-only: omit neutral return; retain torque at last explicitly authorized target')
    parser.add_argument('--hands-free', action='store_true',
                        help='operator confirms hands already out of motion area; skip release prompt')
    parser.add_argument('--recover', action='store_true', help='operator-authorized independent neutral recovery; requires --neutral')
    parser.add_argument('--model-finger-floor-mm', type=float, default=30.,
                        help='explicit per-run model floor for two finger meshes, 10..30 mm; default 30, other links unchanged')
    parser.add_argument('--ready-pose-without-model-floor', action='store_true',
                        help='user-approved model-floor exception, restricted to one held ready-pose stage')
    parser.add_argument('--ready-pose-immediate', action='store_true',
                        help='user-authorized ready-only immediate dispatch without stdin prompts')
    parser.add_argument('--port', default=DEFAULT_PORT)
    args = parser.parse_args()
    if args.recover and not args.neutral: parser.error('--recover requires --neutral')
    if args.hold_at_last_target and not args.task: parser.error('--hold-at-last-target requires --task')
    config = Configuration(ROOT)
    data = json.loads(args.task.read_text()) if args.task else {'name':'neutral_only','steps':[]}
    steps = config.task(data)
    if args.hold_at_last_target:
        steps = steps[:-1]
        if not steps: parser.error('--hold-at-last-target requires a nonempty task')
    floor = select_route_check(config, data, steps, args.hold_at_last_target,
                               args.ready_pose_without_model_floor,
                               args.model_finger_floor_mm / 1000.)
    if args.ready_pose_immediate and not args.ready_pose_without_model_floor:
        parser.error('--ready-pose-immediate requires the ready-only task exception')
    if not args.execute:
        from task_controller import six_counts
        start = six_counts(args.start_counts); config.check_counts(start)
        plan = []
        for step in steps:
            plan.append({**step, 'profiles':config.profiles, 'packet_count':1,
                         'geometry':floor(start, step['goals'])})
            start = step['goals']
        print(json.dumps({'status':'offline_plan_only', 'assumed_start':args.start_counts,
                          'pid':config.baseline['parameters'], 'steps':plan,
                          'control_policy':config.policy,
                          'model_floor_check_enabled':not args.ready_pose_without_model_floor,
                          'stdin_confirmation_required':not args.ready_pose_immediate,
                          'dispatch_authorization':'command_invocation_ready_only' if args.ready_pose_immediate else 'per_stage_operator_confirmation',
                          'model_floor_override_scope':'ready_pose_only' if args.ready_pose_without_model_floor else None,
                          'notice':'No serial access. Model floor checks do not certify physical objects, self collision or cables.'},ensure_ascii=False,indent=2))
        return 0
    if not config.policy.get('approved_by_user'):
        parser.error('当前控制方式尚未获得用户确认')
    folder = ROOT / 'data/logs' / datetime.now().strftime('task_%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True)
    record = {'status':'preflight', 'task':data, 'baseline':config.baseline,
              'finger_model_floor_mm':args.model_finger_floor_mm,
              'model_floor_check_enabled':not args.ready_pose_without_model_floor,
              'stdin_confirmation_required':not args.ready_pose_immediate,
              'dispatch_authorization':'command_invocation_ready_only' if args.ready_pose_immediate else 'per_stage_operator_confirmation',
              'model_floor_override_scope':'ready_pose_only' if args.ready_pose_without_model_floor else None,
              'hold_at_last_target':args.hold_at_last_target,
              'defaults':config.defaults, 'control_policy':config.policy, 'events_file':'events.jsonl'}
    def save(): (folder/'record.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n')
    save()
    def interrupted(signum, frame): raise TaskError(f'operator/process stop signal {signum}')
    signal.signal(signal.SIGTERM, interrupted)
    (ROOT/'data/run').mkdir(parents=True, exist_ok=True)
    with (ROOT/'data/run/controller.lock').open('a') as lock, (folder/'events.jsonl').open('a',buffering=1) as events:
        def emit(event):
            events.write(json.dumps(event,ensure_ascii=False)+'\n')
            if event['event'] not in ('feedback',): print(json.dumps(event,ensure_ascii=False),flush=True)
        try:
            fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            with ServoBus(args.port) as bus:
                fcntl.ioctl(bus.port.ser.fileno(), termios.TIOCEXCL)
                try:
                    controller = TaskController(bus,config,floor,emit)
                    controller.prepare(steps,recover=args.recover)
                    if args.ready_pose_immediate:
                        record['status']='command_line_authorized_ready_dispatch';save()
                        record['status']=dispatch_ready_immediately(controller,steps)
                    else:
                        record['status']='holding_waiting_hands_free'; save()
                        if not args.hands_free:
                            reply=input('六轴扭矩已确认开启。请现场观察并撤掉托扶力，输入“已放手”进入阶段确认；其他输入中止：').strip()
                            if reply != '已放手': raise TaskError('release confirmation not received')
                        def confirm(preview, read_feedback):
                            record['status']='awaiting_operator_confirmation'
                            record['pending_stage']=preview['stage'];save()
                            if preview['stage']=='final_observation':
                                prompt=('最终任务目标已发送，扭矩仍开启。观察后输入“确认”记录结果并结束；不会断扭矩：' if args.hold_at_last_target else '中立最终目标已发送，扭矩仍开启。观察后输入“确认”记录结果并结束；不会断扭矩：')
                            else:
                                prompt=f"下一阶段 {preview['stage']}，目标 {preview['goals']}。由你判断现场情况，输入“确认”才发送；输入“结束”退出："
                            print(prompt,flush=True)
                            last_print=0.
                            while True:
                                # Keep reading/logging while operator decides. No numerical
                                # threshold declares completion or advances the task.
                                ready,_,_=select.select([sys.stdin],[],[],.02)
                                if ready:
                                    reply=sys.stdin.readline()
                                    if not reply: return False
                                    if reply.strip()=='确认':
                                        record['status']='operator_authorized_stage';save();return True
                                    if reply.strip()=='结束':return False
                                    print('请明确输入“确认”或“结束”；尚未发送下一阶段目标。',flush=True)
                                rows=read_feedback()
                                if time.monotonic()-last_print>=1:
                                    last_print=time.monotonic()
                                    action = ('等待输入【确认】记录观察并结束，扭矩保留' if preview['stage']=='final_observation' else f"等待输入【确认】才发送阶段 {preview['stage']}；输入【结束】退出")
                                    print(action + ' | 反馈 ' + ' | '.join(f"{i}:p={v['position']} Δ={v['position']-preview['goals'][i]:+d} v={v['speed']} torque={v['torque']}" for i,v in rows.items()),flush=True)
                        record['status']='manual_stage_control';save()
                        record['status']=controller.run(steps,confirm)
                        record.pop('pending_stage',None)
                except BaseException as exc:
                    record['status']='fault_no_extra_commands';record['error']=repr(exc)
                    emit({'event':'fault','message':str(exc),'policy':'No additional recovery target or forced torque-OFF after failure; movement may continue and actual torque state needs readback.'})
                    raise
                finally:
                    fcntl.ioctl(bus.port.ser.fileno(),termios.TIOCNXCL)
        except BaseException as exc:
            if record['status']=='preflight': record['status']='preflight_failed_no_motion'
            record['error']=repr(exc);save();print('Run record: '+str(folder/'record.json'),file=sys.stderr)
            return 1
    save();print('Session ended, torque unchanged: '+str(folder/'record.json'));return 0


if __name__ == '__main__':
    raise SystemExit(main())
