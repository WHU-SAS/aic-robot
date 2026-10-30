#!/usr/bin/env python3
"""K230 UART2 motor integration bench. Default: STATUS only, no motor start."""
import argparse
import ast
from datetime import datetime
from zoneinfo import ZoneInfo
import json
import glob
from pathlib import Path
import struct
import subprocess
import sys
import time
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT, bootstrap
bootstrap()
from host.transport.download_calibration import CameraLink, PORT
from tools.diagnostics.replay_k230_car_tracker import restore_listener


def stop_previous_script(link):
    link.command(0x06)
    deadline = time.monotonic() + 12
    idle_since = None
    while True:
        time.sleep(.1)
        link.output()
        link.command(0x87, 4)
        if struct.unpack('<I', link.read(4))[0] == 0:
            if idle_since is None: idle_since = time.monotonic()
            if time.monotonic() - idle_since >= .6: return
        else:
            idle_since = None
        if time.monotonic() > deadline:
            raise TimeoutError('Previous K230 script did not stop')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default=PORT)
    parser.add_argument('--execute', action='store_true', help='Run the selected motor test with the declared wheel/ground setup')
    parser.add_argument('--wheels-lifted', action='store_true', help='Confirm both drive wheels are securely suspended')
    parser.add_argument('--forward-once', action='store_true', help='Single approximately 1-second forward pulse, then STOP')
    parser.add_argument('--on-ground', action='store_true', help='Confirm cleared ground path, arm ready pose, and observer present')
    parser.add_argument('--button', action='store_true', help='One start then manually press B21; requires --execute --wheels-lifted')
    parser.add_argument('--stop', action='store_true', help='Send STOP and confirm zero output')
    parser.add_argument('--drive', nargs=3, type=int, metavar=('LEFT_PWM', 'RIGHT_PWM', 'DURATION_MS'),
                        help='One signed independent-wheel pulse; requires --execute and declared setup')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    if sum((bool(args.forward_once), bool(args.button), bool(args.stop), args.drive is not None)) > 1:
        parser.error('choose one of --forward-once, --button, --stop, or --drive')
    ground_motion = args.forward_once or args.drive is not None
    if args.execute and (args.stop or not (args.wheels_lifted or ground_motion and args.on_ground)):
        parser.error('--execute requires --wheels-lifted or an explicit pulse with --on-ground; cannot combine with --stop')
    if args.on_ground and (not ground_motion or not args.execute or args.wheels_lifted):
        parser.error('--on-ground requires --execute with --forward-once or --drive, without --wheels-lifted')
    if args.forward_once and (not args.execute or args.button):
        parser.error('--forward-once requires --execute and cannot combine with --button')
    if args.button and not args.execute:
        parser.error('--button requires --execute --wheels-lifted')
    if args.drive is not None:
        if not args.execute:
            parser.error('--drive requires --execute')
        left, right, duration = args.drive
        if max(abs(left), abs(right)) > 1600 or not 10 <= duration <= 5000 or duration % 10:
            parser.error('--drive PWM must be within +/-1600 and duration 10..5000 ms in 10 ms steps')
    mode = 'drive' if args.drive is not None else 'forward' if args.forward_once else 'button' if args.button else 'bench' if args.execute else 'stop' if args.stop else 'status'
    out = args.output or ROOT/'data/logs'/datetime.now(ZoneInfo('Asia/Shanghai')).strftime('k230_motor_uart_%Y%m%d_%H%M%S_%f')
    out.mkdir(parents=True, exist_ok=True)
    source = (ROOT/'device/k230/diagnostics/k230_motor_uart.py').read_text()
    source = source.replace('MODE = "status"', 'MODE = ' + repr(mode), 1)
    if args.drive is not None:
        source = source.replace('DRIVE = None', 'DRIVE = ' + repr(tuple(args.drive)), 1)
    script = ('try:\n exec(' + repr(source) + ')\n'
              'except BaseException as exc:\n import sys\n sys.print_exception(exc)\n'
              'finally:\n print("EXPORT_DONE")\n')
    result = {'status': 'failed', 'mode': mode, 'servo_commands_sent': False,
              'listener_restored': False,
              'states': [], 'checks': [],
              'policy': {'max_pwm': 1600, 'period': 8000, 'ramp_ms': 500,
                         'max_run_ms': 5000, 'heartbeat_wait_ms': 200, 'loss_timeout_ms': 1000}}
    result['setup'] = 'on_ground' if args.on_ground else 'wheels_lifted' if args.wheels_lifted else 'read_only_or_stop'
    if mode == 'forward':
        result['policy']['requested_forward_duration_ms'] = 1000
    if mode == 'drive':
        result['policy']['requested_drive'] = args.drive
    link, lines = None, []
    exit_code = 1
    start = time.monotonic()
    try:
        port = args.port
        if port == PORT and not Path(port).exists():
            ports = glob.glob('/dev/serial/by-id/usb-Kendryte_CanMV_*-if00')
            if len(ports) != 1: raise RuntimeError('Expected one connected K230')
            port = ports[0]
        if subprocess.run(['fuser', port], capture_output=True).returncode != 1:
            raise RuntimeError('K230 port busy or ownership check failed')
        link = CameraLink(port)
        link.command(0x80, 12)
        result['k230_firmware'] = list(struct.unpack('<III', link.read(12)))
        stop_previous_script(link)
        for raw in link.execute(script, timeout=45):
            line = raw.decode(errors='replace')
            lines.append(line)
            print(line, flush=True)
            if line.startswith('MOTOR_STATE '):
                result['states'].append(ast.literal_eval(line[len('MOTOR_STATE '):]))
            elif line.startswith('MOTOR_CHECK PASS '):
                result['checks'].append(line[len('MOTOR_CHECK PASS '):])
        expected = {'bench': 'MOTOR_BENCH_PASS', 'status': 'MOTOR_STATUS_PASS 20/20',
                    'stop': 'MOTOR_STOP_PASS', 'button': 'MOTOR_BUTTON_PASS',
                    'forward': 'MOTOR_FORWARD_PASS', 'drive': 'MOTOR_DRIVE_PASS'}[mode]
        if expected not in lines:
            raise RuntimeError('Board did not complete ' + mode + '; see board_log.txt')
        if mode == 'forward':
            from tools.diagnostics.chassis_metrics import displacement
            config = json.loads((ROOT/'configs/car/drive_nominal.json').read_text())
            result['nominal_config'] = config
            initial = next(state for state in result['states'] if state['command'] == 'START')
            result['nominal_displacement_including_coast'] = displacement(config, initial, result['states'][-1])
            print('NOMINAL_DISPLACEMENT', json.dumps(result['nominal_displacement_including_coast'], ensure_ascii=False), flush=True)
        result['status'] = 'passed'
        exit_code = 0
    except KeyboardInterrupt:
        result.update(status='interrupted', error='Interrupted by user')
        print('正在停止测试并恢复语音监听，请等待……', flush=True)
    except Exception as exc:
        result['error'] = str(exc)
        print('MOTOR LINK FAILED:', exc, file=sys.stderr, flush=True)
    finally:
        if link is not None:
            result['motor_test_status'] = result['status']
            try:
                restored = restore_listener(link)
                (out/'listener_log.txt').write_text('\n'.join(restored) + '\n')
                result['listener_restored'] = True
            except BaseException as exc:
                result.update(status='listener_restore_failed', listener_error=repr(exc))
                exit_code = 1
            finally:
                link.port.close()
        result['elapsed_s'] = time.monotonic() - start
        (out/'board_log.txt').write_text('\n'.join(lines) + '\n')
        (out/'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
        print('RECORD', out, flush=True)
    return exit_code

if __name__ == '__main__':
    raise SystemExit(main())
