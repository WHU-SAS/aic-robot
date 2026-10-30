#!/usr/bin/env python3
"""Build/install the independent car voice test; no boot or deployment motion."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT, bootstrap
bootstrap()
import argparse
import base64
from datetime import datetime
from zoneinfo import ZoneInfo
import json
import math
from host.transport.download_calibration import CameraLink, PORT
from tools.deploy.k230_pick import digest, encoded, execute, mkdir, write_file
from tools.diagnostics.test_k230_motor_uart import stop_previous_script

REMOTE = '/sdcard/aic/car_straight'


def build_bundle(angle_deg=None):
    config = json.loads((ROOT / 'configs/car/straight.json').read_text())
    config['action'] = 'move'
    if angle_deg is not None:
        if type(angle_deg) is not int or not 1 <= abs(angle_deg) <= 30:
            raise ValueError('Turn angle must be an integer in -30..-1 or 1..30 degrees')
        config.update(action='turn', angle_deg=angle_deg,
                      notes='Nominal encoder angle only; physical turn accuracy not accepted yet.')
        del config['distance_mm'], config['allowed_error_fraction']
    config['voice'] = json.loads((ROOT / 'configs/voice/wake_word.json').read_text())
    nominal = json.loads((ROOT / 'configs/car/drive_nominal.json').read_text())
    config['track_width_mm'] = nominal['track_width_mm']
    config['mm_per_count'] = math.pi * nominal['wheel_diameter_mm'] / (
        nominal['gear_ratio'] * nominal['encoder_ppr_motor_shaft'] * nominal['decoder_edges_per_cycle'])
    client = (ROOT / 'device/k230/diagnostics/k230_motor_uart.py').read_text().split('\ndef run_bench(')[0]
    client = client.replace("                        print('MOTOR_STATE', state)\n", '')
    files = {'straight_voice.py': (ROOT / 'device/k230/car/straight_voice.py').read_bytes(),
             'motor_link.py': client.encode(),
             'wake_word.py': (ROOT / 'device/k230/audio/wake_word.py').read_bytes(),
             'config.json': encoded(config)}
    release_id = digest(b''.join(files[name] for name in sorted(files)))[:16]
    remote = REMOTE + '/releases/' + release_id
    startup = ('import sys\nsys.path.insert(0,%r)\n'
               'from straight_voice import run_forever\nrun_forever(%r)\n') % (remote, remote)
    return config, files, remote, startup


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default=PORT)
    parser.add_argument('--install', action='store_true')
    parser.add_argument('--activate', action='store_true')
    parser.add_argument('--turn-deg', type=int, choices=[*range(-30, 0), *range(1, 31)],
                        metavar='DEGREES', help='Signed angle: positive left, negative right (1..30); default is straight')
    args = parser.parse_args(argv)
    if args.activate and not args.install:
        parser.error('--activate requires --install')
    config, files, remote, startup = build_bundle(args.turn_deg)
    stamp = datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S')
    out = ROOT / 'data/deployments' / ('k230_car_straight_' + stamp)
    out.mkdir(parents=True)
    for name, data in files.items():
        (out / name).write_bytes(data)
    (out / 'main.py').write_text(startup)
    record = {'remote': remote, 'status': 'built_not_installed', 'activated': False,
              'action': config['action'], 'distance_mm': config.get('distance_mm'),
              'angle_deg': config.get('angle_deg'), 'files': {n: digest(b) for n, b in files.items()}}
    try:
        if args.install:
            link = CameraLink(args.port)
            try:
                stop_previous_script(link)
                mkdir(link, remote)
                for name, data in files.items():
                    write_file(link, remote + '/' + name, data)
                record['status'] = 'installed_not_active'
                if args.activate:
                    lines = execute(link, "import binascii\nwith open('/sdcard/main.py','rb') as f:b=f.read()\nprint('PREVIOUS '+binascii.b2a_base64(b).decode().strip())")
                    previous = next(line[9:] for line in lines if line.startswith('PREVIOUS '))
                    (out / 'previous_main.py').write_bytes(base64.b64decode(previous))
                    write_file(link, '/sdcard/main.py', startup.encode())
                    record.update(status='active_on_next_boot', activated=True)
            finally:
                link.port.close()
        print('CAR_STRAIGHT_BUNDLE', record['status'], out)
        return 0
    finally:
        (out / 'deployment.json').write_bytes(encoded(record))


if __name__ == '__main__':
    raise SystemExit(main())
