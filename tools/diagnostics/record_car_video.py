#!/usr/bin/env python3
"""Record K230 raw H.264 video for PC postproduction. Default only builds the script."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import argparse
from datetime import datetime
from zoneinfo import ZoneInfo
import glob
import json
import struct
import subprocess
import time
from host.project_paths import ROOT
from host.transport.download_calibration import CameraLink, PORT
from tools.diagnostics.replay_k230_car_tracker import restore_listener


def build_script(directory, duration_ms):
    source = (ROOT / 'device/k230/car/video_storage.py').read_text() + '\n' + (
        ROOT / 'device/k230/car/raw_video.py').read_text()
    return '''import sys
try:
 exec(%r,globals())
 record_raw_video(%r,%d)
except BaseException as exc:
 sys.print_exception(exc)
finally:print('EXPORT_DONE')
''' % (source, directory, duration_ms)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--duration', type=int, default=60)
    parser.add_argument('--port', default=PORT)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    if not 1 <= args.duration <= 600: parser.error('--duration must be 1..600 seconds')
    stamp = datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f')
    remote = '/sdcard/car_video/' + stamp
    output = args.output or ROOT / 'data/car/raw_video' / stamp
    output.mkdir(parents=True, exist_ok=True)
    if (output / 'result.json').exists(): parser.error('Choose a new output directory')
    source = build_script(remote, args.duration * 1000)
    (output / 'board_script.py').write_text(source)
    result = {'status': 'built_no_hardware_access', 'motion_performed': False,
              'remote_directory': remote, 'duration_requested_s': args.duration,
              'listener_restored': False, 'raw_video_only': True}
    link = None; logs = []
    try:
        if args.execute:
            port = args.port
            if port == PORT and not Path(port).exists():
                ports = glob.glob('/dev/serial/by-id/usb-Kendryte_CanMV_*-if00')
                if len(ports) != 1: raise RuntimeError('Expected one K230')
                port = ports[0]
            if subprocess.run(['fuser', port], capture_output=True).returncode != 1:
                raise RuntimeError('K230 port busy or ownership check failed')
            link = CameraLink(port)
            link.command(0x06)
            deadline = time.monotonic() + 12
            idle_since = None
            while True:
                time.sleep(.1)
                link.output()
                link.command(0x87, 4)
                if struct.unpack('<I', link.read(4))[0] == 0:
                    if idle_since is None: idle_since = time.monotonic()
                    if time.monotonic() - idle_since >= .6: break
                else: idle_since = None
                if time.monotonic() > deadline: raise TimeoutError('Previous script did not stop')
            link.command(0x0D, 4)
            link.port.write(struct.pack('<I', 0)); link.port.flush()
            print('仅录视频，不运行识别、不驱动小车或机械臂。', flush=True)
            print('SD素材目录：' + remote, flush=True)
            for raw in link.execute(source, timeout=args.duration + 30):
                line = raw.decode(errors='replace'); logs.append(line)
                if line == 'CAR_VIDEO_READY':
                    print('录像已开始，可以移动方块；%d秒后自动结束。' % args.duration, flush=True)
                elif line.startswith('CAR_VIDEO_STORAGE '):
                    result['storage'] = json.loads(line[len('CAR_VIDEO_STORAGE '):])
                    print('SD卡可用 %.1f MiB；本次预计需要至少 %.1f MiB。' % (
                        result['storage']['available_bytes'] / 1048576,
                        result['storage']['required_bytes'] / 1048576), flush=True)
                elif line.startswith('CAR_VIDEO_DONE '):
                    result['video'] = json.loads(line[len('CAR_VIDEO_DONE '):])
                else: print(line, flush=True)
            if 'video' not in result:
                if result.get('storage', {}).get('sufficient') is False:
                    raise RuntimeError('SD卡空间不足，未启动录像；先备份并清理空间后重试。')
                raise RuntimeError('Video not finalized; see board_log.txt')
            result['status'] = 'raw_video_recorded'
    except KeyboardInterrupt:
        result['status'] = 'interrupted_partial_video_retained'
    except Exception as exc:
        result.update(status='failed', error=repr(exc))
        print(str(exc), file=sys.stderr)
    finally:
        if link is not None:
            print('正在恢复语音监听，请等待……', flush=True)
            try:
                lines = restore_listener(link)
                (output / 'listener_log.txt').write_text('\n'.join(lines) + '\n')
                result['listener_restored'] = True
            except Exception as exc:
                result['listener_error'] = repr(exc)
            link.port.close()
        (output / 'board_log.txt').write_text('\n'.join(logs) + '\n')
        (output / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
        print('RECORD', output, flush=True)
        if result['status'] == 'raw_video_recorded':
            print('录像完成。语音恢复后关机，用读卡器复制上述目录内的 raw.h264 和全部 JSON/JSONL。', flush=True)
    return 0 if result['status'] in ('built_no_hardware_access', 'raw_video_recorded') and (
        not args.execute or result['listener_restored']) else 1


if __name__ == '__main__': raise SystemExit(main())
