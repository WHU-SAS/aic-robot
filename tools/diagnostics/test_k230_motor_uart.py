#!/usr/bin/env python3
"""K230 UART2 motor integration bench. Default: STATUS only, no motor start."""
import argparse
import ast
from datetime import datetime
import json
from pathlib import Path
import struct
import sys
import time
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT, bootstrap
bootstrap()
from host.transport.download_calibration import CameraLink, PORT

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default=PORT)
    parser.add_argument('--execute', action='store_true', help='Run the selected motor test with the declared wheel/ground setup')
    parser.add_argument('--wheels-lifted', action='store_true', help='Confirm both drive wheels are securely suspended')
    parser.add_argument('--forward-once', action='store_true', help='Single approximately 1-second forward pulse, then STOP')
    parser.add_argument('--on-ground', action='store_true', help='Confirm cleared ground path, arm ready pose, and observer present')
    parser.add_argument('--button', action='store_true', help='One start then manually press B21; requires --execute --wheels-lifted')
    parser.add_argument('--stop', action='store_true', help='Send STOP and confirm zero output')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.execute and (args.stop or not (args.wheels_lifted or args.forward_once and args.on_ground)):
        parser.error('--execute requires --wheels-lifted, or --forward-once --on-ground; cannot combine with --stop')
    if args.on_ground and (not args.forward_once or not args.execute or args.wheels_lifted):
        parser.error('--on-ground only applies to --execute --forward-once without --wheels-lifted')
    if args.forward_once and (not args.execute or args.button):
        parser.error('--forward-once requires --execute and cannot combine with --button')
    if args.button and not args.execute:
        parser.error('--button requires --execute --wheels-lifted')
    mode = 'forward' if args.forward_once else 'button' if args.button else 'bench' if args.execute else 'stop' if args.stop else 'status'
    out = args.output or ROOT/'data/logs'/datetime.now().strftime('k230_motor_uart_%Y%m%d_%H%M%S_%f')
    out.mkdir(parents=True, exist_ok=True)
    source = (ROOT/'device/k230/diagnostics/k230_motor_uart.py').read_text()
    source = source.replace('MODE = "status"', 'MODE = ' + repr(mode), 1)
    script = ('try:\n exec(' + repr(source) + ')\n'
              'except BaseException as exc:\n import sys\n sys.print_exception(exc)\n'
              'finally:\n print("EXPORT_DONE")\n')
    result = {'status': 'failed', 'mode': mode, 'servo_commands_sent': False,
              'states': [], 'checks': [],
              'policy': {'max_pwm': 1600, 'period': 8000, 'ramp_ms': 500,
                         'max_run_ms': 5000, 'heartbeat_wait_ms': 200, 'loss_timeout_ms': 1000}}
    result['setup'] = 'on_ground' if args.on_ground else 'wheels_lifted' if args.wheels_lifted else 'read_only_or_stop'
    if mode == 'forward':
        result['policy']['requested_forward_duration_ms'] = 1000
    link, lines = None, []
    start = time.monotonic()
    try:
        link = CameraLink(args.port)
        link.command(0x80, 12)
        result['k230_firmware'] = list(struct.unpack('<III', link.read(12)))
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
                    'forward': 'MOTOR_FORWARD_PASS'}[mode]
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
        return 0
    except Exception as exc:
        result['error'] = str(exc)
        print('MOTOR LINK FAILED:', exc, file=sys.stderr, flush=True)
        return 1
    finally:
        if link is not None:
            link.port.close()
        result['elapsed_s'] = time.monotonic() - start
        (out/'board_log.txt').write_text('\n'.join(lines) + '\n')
        (out/'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
        print('RECORD', out, flush=True)

if __name__ == '__main__':
    raise SystemExit(main())
