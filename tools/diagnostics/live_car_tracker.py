#!/usr/bin/env python3
"""K230 live color tracking with browser preview; never sends robot commands."""
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
import webbrowser
from host.project_paths import ROOT
from host.car.runtime import load
from host.car.live_view import LiveView
from host.transport.download_calibration import CameraLink, PORT
from tools.diagnostics.replay_k230_car_tracker import restore_listener


def build_script(config, geometry, duration_ms, interval_ms, record_directory=None):
    paths = ['device/k230/car/ground_projection.py', 'device/k230/car/blue_tracker.py',
             'device/k230/car/live_recording.py', 'device/k230/car/live_preview.py']
    sources = [(ROOT / p).read_text() for p in paths]
    sources[0] = sources[0].replace('from ..pick.pose import undistort, project',
                                  'from pose import undistort, project', 1)
    model = {'K': geometry.K, 'D': geometry.D, 'T': geometry.T,
             'plane': {'normal': geometry.normal, 'offset_mm': geometry.offset}}
    return '''import sys,json
try:
 with open('/sdcard/aic/pick/current.json') as f:release=json.load(f)['release']
 sys.path.insert(0,release)
 for source in %r:exec(source,globals())
 run_live(%r,%r,%d,%d,%r)
except BaseException as exc:
 sys.print_exception(exc)
finally:print('EXPORT_DONE')
''' % (sources, config['tracking'], model, duration_ms, interval_ms, record_directory)


def read_preview(link):
    # Existing verified USBDBG framebuffer protocol; JPEG bytes are transferred
    # directly, rather than slow base64 via stdout or files on SD.
    link.command(0x81, 12)
    width, height, size = struct.unpack('<III', link.read(12))
    if not width or not height or size <= 3: return None
    if width > 4096 or height > 4096 or size > 4 * 1024 * 1024:
        raise RuntimeError('Invalid preview frame dimensions/size')
    link.command(0x82, size)
    jpeg = link.read(size)
    if not jpeg.startswith(b'\xff\xd8'): raise RuntimeError('Preview is not JPEG')
    return jpeg


def execute_live(link, source, timeout):
    """USBDBG execution with idle yields so Stop works even without telemetry."""
    link.command(0x06); time.sleep(.15); link.output()
    payload = source.encode()
    if len(payload) % 64 == 0: payload += b'\n'
    link.command(0x05, len(payload)); time.sleep(.06)
    link.port.write(payload); link.port.flush()
    started = time.monotonic(); pending = b''
    while time.monotonic() - started < timeout:
        pending += link.output()
        if len(pending) > 65536: raise RuntimeError('Unterminated live telemetry line')
        while b'\n' in pending:
            line, pending = pending.split(b'\n', 1); line = line.rstrip(b'\r')
            if line == b'EXPORT_DONE': return
            yield line
        yield b''
        time.sleep(.02)
    raise TimeoutError('K230 live test did not finish')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--duration', type=int, default=120, help='Live duration seconds, default 120')
    parser.add_argument('--interval-ms', type=int, default=100, help='Diagnostic minimum observation period')
    parser.add_argument('--port', default=PORT)
    parser.add_argument('--http-port', type=int, default=0)
    parser.add_argument('--no-browser', action='store_true')
    parser.add_argument('--no-record', action='store_true', help='Explicitly disable SD demonstration recording')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    if not 1 <= args.duration <= 1800: parser.error('--duration must be 1..1800 seconds')
    if not 30 <= args.interval_ms <= 1000: parser.error('--interval-ms must be 30..1000')
    if not 0 <= args.http_port <= 65535: parser.error('Invalid --http-port')
    _, config, geometry = load()
    stamp = datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f')
    output = args.output or ROOT / 'data/car/live_tracking' / stamp
    output.mkdir(parents=True, exist_ok=True)
    if (output / 'result.json').exists(): parser.error('Choose a new output directory')
    record_directory = None if args.no_record else '/sdcard/car_live/' + stamp
    source = build_script(config, geometry, args.duration * 1000, args.interval_ms, record_directory)
    (output / 'board_script.py').write_text(source)
    (output / 'tracking_config.json').write_text(json.dumps(config['tracking'], indent=2) + '\n')
    result = {'status': 'built_no_hardware_access', 'motion_performed': False,
        'duration_requested_s': args.duration, 'listener_restored': False,
        'observations': 0, 'preview_frames': 0, 'state_counts': {}, 'state_transitions': [],
        'SD_recording_directory': record_directory, 'recording_format': 'JPEG sequence; export to video on PC'}
    link = None; viewer = None; last_jpeg = None
    native_done = False; old_status = None; native_ready = False
    started = time.monotonic(); last_preview = started; last_observation = started
    try:
        if args.execute:
            port = args.port
            if port == PORT and not Path(port).exists():
                ports = glob.glob('/dev/serial/by-id/usb-Kendryte_CanMV_*-if00')
                if len(ports) != 1: raise RuntimeError('Expected one connected K230')
                port = ports[0]
            owner = subprocess.run(['fuser', port], capture_output=True, text=True)
            if owner.returncode != 1: raise RuntimeError('K230 port busy or ownership check failed')
            viewer = LiveView(args.http_port); viewer.start()
            print('实时画面：' + viewer.url, flush=True)
            print('小车保持静止；等待画面后移动方块。测试%d秒，Ctrl+C或网页按钮结束。' % args.duration, flush=True)
            if record_directory:
                print('演示录像素材直接写入SD卡：' + record_directory + '（结束后可用读卡器复制）', flush=True)
            if not args.no_browser:
                try: webbrowser.open(viewer.url)
                except Exception as exc: print('请手动打开上方网址：' + str(exc), flush=True)
            link = CameraLink(port)
            link.command(0x06); time.sleep(.15); link.output()
            link.command(0x0D, 4)
            link.port.write(struct.pack('<I', 1)); link.port.flush()
            with (output / 'board_log.txt').open('w') as board_log, (output / 'observations.jsonl').open('w') as events:
                for raw in execute_live(link, source, timeout=args.duration + 45):
                    line = raw.decode(errors='replace')
                    if line:
                        board_log.write(line + '\n'); board_log.flush()
                    if line == 'CAR_LIVE_READY':
                        native_ready = True; started = time.monotonic()
                        print('相机已就绪，现在可以移动方块。', flush=True)
                    elif line.startswith('CAR_LIVE '):
                        row = json.loads(line[len('CAR_LIVE '):])
                        now = time.monotonic(); last_observation = now
                        row['host_received_monotonic_s'] = now
                        events.write(json.dumps(row, ensure_ascii=False) + '\n'); events.flush()
                        result['observations'] += 1
                        state = row['status']
                        result['state_counts'][state] = result['state_counts'].get(state, 0) + 1
                        if state != old_status:
                            result['state_transitions'].append({'frame_id': row['frame_id'], 'status': state})
                            print('状态：%s，帧 %s' % (state, row['frame_id']), flush=True)
                            old_status = state
                        viewer.update(observation=row, message='', observation_age_s=0)
                    elif line.startswith('CAR_LIVE_DONE '):
                        result['native_summary'] = json.loads(line[len('CAR_LIVE_DONE '):]); native_done = True
                    if viewer.stop.is_set(): break
                    now = time.monotonic()
                    if native_ready and not native_done and now - last_preview >= .2:
                        jpeg = read_preview(link)
                        if jpeg is not None:
                            last_jpeg = jpeg; result['preview_frames'] += 1
                            viewer.frame(jpeg, 1 / max(time.monotonic() - last_preview, .001))
                            last_preview = time.monotonic()
                    viewer.update(observation_age_s=time.monotonic() - last_observation)
            result['status'] = 'live_completed' if native_done else 'stopped_by_user' if viewer.stop.is_set() else 'failed'
            if result['status'] == 'failed': raise RuntimeError('K230 did not complete live test; see board_log.txt')
            if not result['observations'] or not result['preview_frames']:
                raise RuntimeError('Missing live observations or browser preview frames')
    except KeyboardInterrupt:
        result['status'] = 'stopped_by_user'
        print('正在结束测试并恢复语音，请稍等……', flush=True)
    except Exception as exc:
        result.update(status='failed', error=repr(exc)); print(str(exc), file=sys.stderr)
    finally:
        result['test_status_before_listener_restore'] = result['status']
        if link is not None:
            try:
                print('正在恢复原语音监听……', flush=True)
                restored = restore_listener(link)
                (output / 'listener_log.txt').write_text('\n'.join(restored) + '\n')
                result['listener_restored'] = True
            except BaseException as exc:
                result.update(status='listener_restore_failed', listener_restore_error=repr(exc))
            link.port.close()
        if last_jpeg is not None: (output / 'last_preview.jpg').write_bytes(last_jpeg)
        if viewer is not None:
            viewer.update(finished=True, message='测试结束，请查看终端记录')
            viewer.close()
        result['host_total_s'] = time.monotonic() - started
        (output / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
        print('RECORD', output, flush=True)
    return 0 if result['status'] in ('built_no_hardware_access', 'live_completed', 'stopped_by_user') else 1


if __name__ == '__main__': raise SystemExit(main())
