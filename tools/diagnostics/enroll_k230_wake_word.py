#!/usr/bin/env python3
"""Record three prompted takes for pick or ready; enrollment/listening never moves the arm."""
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


def execute(link, source, call, timeout, phrase='思达思达'):
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
            print('\n>>> 现在请说：'+phrase+'（只说一遍，说完保持安静）', flush=True)
        elif line == 'VOICE_SPEECH':
            print('已听到声音，录音中……', flush=True)
        elif line == 'VOICE_LISTEN_READY':
            print('开始检测。请每隔约3秒说一次“'+phrase+'”；这里只打印事件，不驱动机械臂。', flush=True)
        elif line.startswith('VOICE_WAKE '):
            print('已识别：'+phrase+'  ' + line[11:], flush=True)
        elif not line.startswith(('VOICE_PROGRESS ', 'VOICE_ENROLLED ', 'VOICE_STATS ')):
            print(line, flush=True)
    return lines


def selection(config, command):
    if command == 'pick':
        return config, config.get('keyword_id', 'sida'), config['wake_word']
    item = config['commands'][command]
    return item, item['keyword_id'], item['word']


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default=PORT)
    parser.add_argument('--command', choices=('pick','ready'), default='pick', help='Which phrase to enroll or test')
    parser.add_argument('--listen', action='store_true', help='Test the registered word instead of enrolling')
    parser.add_argument('--sensitivity', choices=('normal', 'sensitive', 'conservative'),
                        help='Override configured listening profile; enrollment always uses normal')
    parser.add_argument('--seconds', type=int, default=20, choices=range(5, 121), metavar='5..120')
    args = parser.parse_args(argv)
    if not args.listen and args.sensitivity not in (None, 'normal'):
        parser.error('--sensitivity only applies to --listen')
    config_path = ROOT / 'configs/voice/wake_word.json'
    config = json.loads(config_path.read_text())
    if config['backend'] != 'open_kws':
        parser.error('This entry requires open_kws')
    target, keyword_id, phrase = selection(config, args.command)
    if args.listen:
        args.sensitivity = args.sensitivity or config.get('detection_sensitivity', 'normal')
        if args.sensitivity not in ('normal', 'sensitive', 'conservative'):
            parser.error('Invalid detection_sensitivity in wake_word.json')
    stamp = datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f')
    folder = ROOT / 'data/logs' / ('k230_wake_' + stamp)
    folder.mkdir(parents=True)
    remote = '/sdcard/voice/' + keyword_id + '_' + stamp
    source = (ROOT / 'device/k230/audio/wake_word.py').read_text()
    record = {'status': 'failed', 'motion_commands': False, 'remote_session': remote,
              'mode': 'listen' if args.listen else 'enroll', 'command':args.command, 'word':phrase, 'takes': []}
    if args.listen:
        record.pop('remote_session')
        record.update(enrollment_file=target.get('enrollment_file'),
                      sensitivity=args.sensitivity, seconds=args.seconds)
    link, lines = None, []
    try:
        if args.listen:
            if not target.get('enrollment_file'):
                raise RuntimeError('尚未注册唤醒词，请先不带 --listen 运行')
            input('IDE停止并断开，并退出k230-status监视。按回车开始检测（仅识别，不运动）……')
            link = CameraLink(args.port)
            lines += execute(link, source, 'listen(%r,%d,%r)' % (target['enrollment_file'], args.seconds, args.sensitivity), args.seconds + 20, phrase)
            if not any(x.startswith('VOICE_LISTEN_DONE ') for x in lines):
                raise RuntimeError('检测未完成，详见日志')
            record['wake_count'] = int(next(x.split()[-1] for x in lines if x.startswith('VOICE_LISTEN_DONE ')))
            summary = next((json.loads(x[len('VOICE_LISTEN_SUMMARY '):]) for x in lines
                            if x.startswith('VOICE_LISTEN_SUMMARY ')), None)
            record['summary'] = summary
            record['status'] = 'detection_completed'
            return 0
        print('录制指令：'+phrase+'。IDE请停止并断开，退出k230-status监视。\n距麦克风30～50cm，正常音量；每段看到“现在请说”后再开口。\n本脚本会暂停板端监听，但不会发送舵机指令；机械臂已有目标和扭矩不变。',flush=True)
        for number in (1, 2, 3):
            while True:
                input('\n按回车准备第%d/3段；输入 Ctrl+C 可退出……' % number)
                if link is None:
                    link = CameraLink(args.port)
                batch = execute(link, source, 'capture_take(%r,%d)' % (remote, number), 35, phrase)
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
        batch = execute(link, source, 'finalize(%r,%r,%r)' % (remote, keyword_id, phrase), 150, phrase)
        lines += batch
        enrolled = next((json.loads(x[15:]) for x in batch if x.startswith('VOICE_ENROLLED ')), None)
        if enrolled is None:
            raise RuntimeError('注册计算未成功；三段原始音频保留在板端会话目录')
        if enrolled['path'] != remote+'/keyword.json' or enrolled['keyword']['kw_id'] != keyword_id:
            raise RuntimeError('注册结果与所选指令不符，未更新配置')
        record['enrollment'] = enrolled
        (folder / 'keyword.json').write_text(json.dumps(enrolled, ensure_ascii=False, indent=2) + '\n')
        target.update(status='enrolled_pending_detection_validation', enrollment_file=enrolled['path'])
        temp_config=config_path.with_suffix('.json.tmp')
        temp_config.write_text(json.dumps(config, ensure_ascii=False, indent=2) + '\n')
        temp_config.replace(config_path)
        record['status'] = 'enrolled'
        print('“'+phrase+'”注册完成，已保存到SD卡；其他指令注册未改动。\n运行同一命令加 --listen 可仅测试识别。测试完成后执行 ./scripts/deploy-pick --install --activate，再仅重启K230生效。', flush=True)
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
        if link is not None:print('板端监听已暂停。完成测试/部署后，仅重启K230恢复监听；请勿切断正在保持姿态的舵机电源。',flush=True)


if __name__ == '__main__':
    raise SystemExit(main())
