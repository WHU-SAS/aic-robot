#!/usr/bin/env python3
"""Record three user-triggered takes and enroll 思达思达; no motion commands."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import sys
from zoneinfo import ZoneInfo
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT, bootstrap
bootstrap()
from host.transport.download_calibration import CameraLink, PORT


def execute(link, source, call, timeout):
    script = ('try:\n exec(' + repr(source + '\n' + call) + ')\n'
              'except BaseException as exc:\n import sys\n sys.print_exception(exc)\n'
              'finally:\n print("EXPORT_DONE")\n')
    lines = []
    for raw in link.execute(script, timeout=timeout):
        line = raw.decode(errors='replace')
        lines.append(line)
        if line == 'VOICE_WARMUP':
            print('麦克风预热并测底噪，请先保持安静约7秒……', flush=True)
        elif line.startswith('VOICE_READY '):
            print('\n>>> 现在请说：思达思达（只说一遍，说完保持安静）', flush=True)
        elif line == 'VOICE_SPEECH':
            print('已听到声音，录音中……', flush=True)
        elif line == 'VOICE_LISTEN_READY':
            print('开始检测。请每隔约3秒说一次“思达思达”；这里只打印事件。', flush=True)
        elif line.startswith('VOICE_WAKE '):
            print('已唤醒：思达思达  ' + line[11:], flush=True)
        elif not line.startswith(('VOICE_PROGRESS ', 'VOICE_ENROLLED ', 'VOICE_STATS ')):
            print(line, flush=True)
    return lines


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default=PORT)
    parser.add_argument('--listen', action='store_true', help='Test the registered word instead of enrolling')
    parser.add_argument('--sensitivity', choices=('normal', 'sensitive', 'conservative'), default='normal',
                        help='Official listening profile; enrollment always uses normal')
    parser.add_argument('--seconds', type=int, default=20, choices=range(5, 121), metavar='5..120')
    args = parser.parse_args()
    if not args.listen and args.sensitivity != 'normal':
        parser.error('--sensitivity only applies to --listen')
    config_path = ROOT / 'configs/voice/wake_word.json'
    config = json.loads(config_path.read_text())
    if config['wake_word'] != '思达思达' or config['backend'] != 'open_kws':
        parser.error('This entry is configured for 思达思达 / open_kws only')
    stamp = datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f')
    folder = ROOT / 'data/logs' / ('k230_wake_' + stamp)
    folder.mkdir(parents=True)
    remote = '/sdcard/voice/sida_' + stamp
    source = (ROOT / 'device/k230/audio/wake_word.py').read_text()
    record = {'status': 'failed', 'motion_commands': False, 'remote_session': remote,
              'mode': 'listen' if args.listen else 'enroll', 'takes': []}
    if args.listen:
        record.pop('remote_session')
        record.update(enrollment_file=config.get('enrollment_file'),
                      sensitivity=args.sensitivity, seconds=args.seconds)
    link, lines = None, []
    try:
        if args.listen:
            if not config.get('enrollment_file'):
                raise RuntimeError('尚未注册唤醒词，请先不带 --listen 运行')
            input('IDE停止并断开。按回车开始检测（预热结束后会提示）……')
            link = CameraLink(args.port)
            lines += execute(link, source, 'listen(%r,%d,%r)' % (config['enrollment_file'], args.seconds, args.sensitivity), args.seconds + 20)
            if not any(x.startswith('VOICE_LISTEN_DONE ') for x in lines):
                raise RuntimeError('检测未完成，详见日志')
            record['wake_count'] = int(next(x.split()[-1] for x in lines if x.startswith('VOICE_LISTEN_DONE ')))
            summary = next((json.loads(x[len('VOICE_LISTEN_SUMMARY '):]) for x in lines
                            if x.startswith('VOICE_LISTEN_SUMMARY ')), None)
            record['summary'] = summary
            record['status'] = 'detection_completed'
            return 0
        print('IDE请停止并断开；距麦克风30～50cm，正常音量。每段看到“现在请说”后再开口。')
        for number in (1, 2, 3):
            while True:
                input('\n按回车准备第%d/3段；输入 Ctrl+C 可退出……' % number)
                if link is None:
                    link = CameraLink(args.port)
                batch = execute(link, source, 'capture_take(%r,%d)' % (remote, number), 35)
                lines += batch
                take = next((json.loads(x[11:]) for x in batch if x.startswith('VOICE_TAKE ')), None)
                if take is None:
                    raise RuntimeError('录音未完成，详见日志')
                record['takes'].append(take)
                if take['ok']:
                    print('第%d段合格：%s' % (number, json.dumps(take.get('metrics'), ensure_ascii=False)), flush=True)
                    break
                print('本段不合格：%s。调整距离或环境后按回车重录。' % take['reason'], flush=True)
        print('三段已采集，正在计算注册参数……', flush=True)
        batch = execute(link, source, 'finalize(%r)' % remote, 150)
        lines += batch
        enrolled = next((json.loads(x[15:]) for x in batch if x.startswith('VOICE_ENROLLED ')), None)
        if enrolled is None:
            raise RuntimeError('注册计算未成功；三段原始音频保留在板端会话目录')
        record['enrollment'] = enrolled
        (folder / 'keyword.json').write_text(json.dumps(enrolled, ensure_ascii=False, indent=2) + '\n')
        config.update(status='enrolled_pending_detection_validation', enrollment_file=enrolled['path'])
        config_path.write_text(json.dumps(config, ensure_ascii=False, indent=2) + '\n')
        record['status'] = 'enrolled'
        print('“思达思达”注册完成，已保存到SD卡。运行同一命令加 --listen 可测试。', flush=True)
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        record['error'] = str(exc) or 'User interrupted'
        print('录音/注册停止：' + record['error'], flush=True)
        if link is not None:
            try:
                link.command(0x06)  # Stop microphone/KWS script; never a motion command.
            except Exception:
                pass
        return 1
    finally:
        if link is not None:
            link.port.close()
        (folder / 'record.json').write_text(json.dumps(record, ensure_ascii=False, indent=2) + '\n')
        (folder / 'board_log.txt').write_text('\n'.join(lines) + '\n')
        print('RECORD', folder, flush=True)


if __name__ == '__main__':
    raise SystemExit(main())
