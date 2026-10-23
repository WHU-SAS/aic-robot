#!/usr/bin/env python3
"""Three frozen ready->descend->ready trials over K230; all metrics are post-run."""
import argparse
import ast
from datetime import datetime
import hashlib
import json
from pathlib import Path
from statistics import median
import sys
from zoneinfo import ZoneInfo

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT


def make_config(action_seconds=None, ready_seconds=None):
    config = json.loads((ROOT/'configs/arm/pid_three_actions.json').read_text())
    motion = json.loads((ROOT/config['motion_defaults_reference']).read_text())['motion']
    config['profiles'] = [motion[str(i)] for i in range(1, 7)]
    action_seconds = config['action_sample_s'] if action_seconds is None else action_seconds
    ready_seconds = config['ready_sample_s'] if ready_seconds is None else ready_seconds
    stages = [{'name':'ready_initial', 'goals':config['ready_counts'], 'seconds':ready_seconds}]
    for index, action in enumerate(config['actions'], 1):
        stages.append({'name':action['name'], 'goals':action['counts'], 'seconds':action_seconds})
        stages.append({'name':'ready_after_' + str(index), 'goals':config['ready_counts'], 'seconds':ready_seconds})
    config['stages'] = stages
    return config


def board_script(config):
    read = (ROOT/'device/k230/diagnostics/uart1_servo_read.py').read_text().split('\ndef main():')[0]
    source = (ROOT/'device/k230/arm/ready_pose.py').read_text()
    tree = ast.parse(source)
    packet = '\n'.join(ast.get_source_segment(source, n) for n in tree.body
                       if isinstance(n, ast.FunctionDef) and n.name in ('frame', 'sync_frame'))
    runner = (ROOT/'device/k230/arm/pid_three.py').read_text()
    body = '\n'.join((read, packet, runner, 'run_trial(json.loads(%r))' % json.dumps(config)))
    return ('try:\n exec(' + repr(body) + ')\nexcept BaseException as exc:\n import sys\n'
            ' sys.print_exception(exc)\nfinally:\n print("EXPORT_DONE")\n')


def settled_at(points, center, band):
    # The whole remaining observed suffix must stay in-band for >=0.5 s.
    last_bad = max((i for i, (_, p) in enumerate(points) if abs(p-center) > band), default=-1)
    rest = points[last_bad+1:]
    return rest[0][0] if rest and rest[-1][0]-rest[0][0] >= .5 else None


def reversal_count(values, excursion=3):
    if not values:
        return 0
    anchor = extreme = values[0]
    direction = reversals = 0
    for value in values[1:]:
        if not direction:
            if abs(value-anchor) >= excursion:
                direction = 1 if value > anchor else -1
                extreme = value
        elif (value-extreme)*direction >= 0:
            extreme = value
        elif (extreme-value)*direction >= excursion:
            reversals += 1
            direction = -direction
            extreme = value
    return reversals


def summarize(config, events):
    stages = []
    for index, stage in enumerate(config['stages']):
        samples = [e for e in events if e['event']=='sample' and e['index']==index]
        rows = []
        for axis, goal in enumerate(stage['goals']):
            points = [(e['rows'][axis][0], e['rows'][axis][1]) for e in samples
                      if e['rows'][axis][1] is not None]
            missing = len(samples)-len(points)
            row = {'id':axis+1, 'goal':goal, 'valid_samples':len(points), 'missing_samples':missing}
            if points:
                end = points[-1][0]
                center = median([p for t, p in points if t >= end-.5])
                tail = [p for t, p in points if t >= end-1]
                stable = settled_at(points, center, 2) if not missing else None
                near = next((i for i, (_, p) in enumerate(points) if abs(p-center)<=12), None)
                correction = max(0, stable-points[near][0]) if stable is not None and near is not None else None
                row.update(final_position=points[-1][1], final_error=points[-1][1]-goal,
                           tail_median_error=center-goal, tail_peak_to_peak=max(tail)-min(tail),
                           stable_at_s=stable, correction_s=correction,
                           goal_settled_at_s=settled_at(points,goal,6) if not missing else None,
                           reversals_after_near=reversal_count([p for _,p in points[near:]]) if near is not None else 0,
                           observed_s=end, median_sample_interval_s=median([b[0]-a[0] for a,b in zip(points,points[1:])]) if len(points)>1 else None)
            rows.append(row)
        stages.append({'name':stage['name'], 'completed':any(e['event']=='stage_complete' and e['index']==index for e in events), 'axes':rows})
    return {'complete':any(e['event']=='complete' for e in events), 'stages':stages,
            'parameters':next((e['rows'] for e in events if e['event']=='parameters'),None)}


def report_text(report):
    lines = ['偏差=实际-目标（计数）；修正/停稳仅统计本次短窗口，--表示未测得。',
             '阶段/轴  最终偏差  修正秒  停稳秒  目标带秒  尾段波动  折返次数']
    def show(value):
        return '--' if value is None else '%.2f' % value
    for stage in report['stages']:
        if not any(r['valid_samples'] for r in stage['axes']):
            continue
        lines.append(stage['name'])
        for r in stage['axes']:
            if not r['valid_samples']:
                lines.append('  %d: 无有效反馈' % r['id']);continue
            lines.append('  %d: %+d  %s  %s  %s  %d  %d' %
                         (r['id'], r['final_error'], show(r['correction_s']), show(r['stable_at_s']),
                          show(r['goal_settled_at_s']), r['tail_peak_to_peak'], r['reversals_after_near']))
    return '\n'.join(lines)+'\n'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='运行三组固定动作')
    parser.add_argument('--label', default='current_pid', help='本轮参数名称，仅用于记录')
    parser.add_argument('--port', help='K230串口；默认自动查找')
    parser.add_argument('--action-seconds', type=float, help='每个动作采样秒数，默认5')
    parser.add_argument('--ready-seconds', type=float, help='每次准备位采样秒数，默认3.5')
    args = parser.parse_args(argv)
    for value in (args.action_seconds,args.ready_seconds):
        if value is not None and (not 0 < value < float('inf')):
            parser.error('采样秒数须为有限正数')
    config = make_config(args.action_seconds,args.ready_seconds)
    for stage in config['stages']:
        print('%s %s %.1fs' % (stage['name'],stage['goals'],stage['seconds']),flush=True)
    if not args.execute:
        print('以上为固定目标；加 --execute 运行。');return 0
    from host.transport.download_calibration import CameraLink,PORT
    out = ROOT/'data/logs'/datetime.now(ZoneInfo('Asia/Shanghai')).strftime('pid_three_%Y%m%d_%H%M%S_%f')
    out.mkdir(parents=True)
    script = board_script(config)
    (out/'run.json').write_text(json.dumps({'label':args.label,'config':config,'script_sha256':hashlib.sha256(script.encode()).hexdigest()},ensure_ascii=False,indent=2)+'\n')
    events = [];link = None;failure = None
    try:
        link = CameraLink(args.port or PORT)
        # USBDBG stops voice and releases media; after an IDE interrupt CanMV
        # can reboot the interpreter once. Only this harmless handshake retries.
        for attempt in range(2):
            try:
                lines = list(link.execute("print('PID_IDLE')\nprint('EXPORT_DONE')\n",timeout=5))
                if b'PID_IDLE' not in lines:
                    raise RuntimeError('K230未进入脚本模式')
                break
            except TimeoutError:
                if attempt:raise
        with (out/'samples.jsonl').open('w') as raw, (out/'board.log').open('w') as board:
            for line in link.execute(script,timeout=sum(s['seconds'] for s in config['stages'])+45):
                text = line.decode(errors='replace');board.write(text+'\n');board.flush()
                if text.startswith('PID_SAMPLE '):
                    event = json.loads(text[11:]);events.append(event)
                    raw.write(json.dumps(event)+'\n');raw.flush()
                    if event['event']=='stage_started':print('执行 '+event['name'],flush=True)
                elif text.strip():print(text,flush=True)
        if not any(e['event']=='complete' for e in events):raise RuntimeError('测试未完整结束，已保留采样')
    except (Exception,KeyboardInterrupt) as exc:
        failure = repr(exc)
        if link is not None:
            try:link.command(0x06)
            except Exception:pass
        print('结束：'+failure,flush=True)
    finally:
        if link is not None:link.port.close()
        report = summarize(config,events);report['error'] = failure
        (out/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        text = report_text(report);(out/'summary.txt').write_text(text);print(text)
        print('RECORD '+str(out),flush=True)
    return 1 if failure else 0


if __name__ == '__main__':
    raise SystemExit(main())
