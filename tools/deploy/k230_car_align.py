#!/usr/bin/env python3
"""Install car visual alignment, coarse approach or near visual parking."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import argparse
import base64
from datetime import datetime
from zoneinfo import ZoneInfo
import json
from host.project_paths import ROOT
from host.car.runtime import load
from host.transport.download_calibration import CameraLink, PORT
from tools.deploy.k230_car_straight import build_bundle as motion_bundle
from tools.deploy.k230_pick import digest, encoded, execute, mkdir, write_file
from tools.diagnostics.test_k230_motor_uart import stop_previous_script


def build_bundle(approach_mode=False, automatic=False, continuous=False, near_parking=False):
    continuous = continuous or near_parking
    if automatic and continuous:
        raise ValueError('Select staged or continuous approach')
    workspace, approach, projection = load()
    vision = json.loads((ROOT / approach['vision_config']).read_text())
    config, files, _, _ = motion_bundle(5)
    del config['action'], config['angle_deg'], config['notes']
    config.update(alignment=json.loads((ROOT / 'configs/car/visual_align.json').read_text()),
                  tracking=approach['tracking'],
                  coarse_center_tolerance_px=approach['coarse_center_tolerance_px'],
                  vision={key: vision[key] for key in ('camera', 'sampling')},
                  projection={'K': projection.K, 'D': projection.D, 'T': projection.T,
                              'plane': {'normal': projection.normal, 'offset_mm': projection.offset},
                              'forward_axis_Base': projection.forward_axis_Base,
                              'motion_origin_Base_mm': projection.motion_origin_Base_mm})
    if approach_mode or automatic or continuous:
        config['forward'] = json.loads((ROOT / 'configs/car/visual_approach.json').read_text())
        config['forward']['near_Base_X_mm'] = approach['near_localization_start_Base_X_mm']
        config['allowed_error_fraction'] = json.loads((ROOT / 'configs/car/straight.json').read_text())['allowed_error_fraction']
        config['alignment']['notes'] = 'Turn gain0.75 compensates the whole visual-to-motion chain; one TURN or one MOVE per wake. Forward settings are separate.'
    if automatic:
        config['automatic_approach'] = True
        config['forward']['notes'] = 'One wake repeats stopped visual TURN/MOVE steps until near handoff; no final parking or arm action.'
        config['alignment']['notes'] = 'Whole-chain turn gain0.75. Reobserve after each stopped action; finish at near, lost target or non-normal motor stop.'
    if continuous:
        config['continuous'] = json.loads((ROOT / 'configs/car/continuous_approach.json').read_text())
        config['forward']['notes'] = 'Continuous coarse visual steering to the existing near handoff; final parking and arm action not enabled.'
        config['alignment']['notes'] = 'Continuous mode uses its own heading gain, not the old one-shot TURN gain0.75.'
    if near_parking:
        localization = vision['localization']
        config.update(intrinsics=json.loads((ROOT / localization['intrinsics']).read_text()),
                      extrinsics=json.loads((ROOT / localization['extrinsics']).read_text()),
                      table_plane=localization['table_plane_Base_mm'], workspace=workspace,
                      near_parking=json.loads((ROOT / 'configs/car/near_parking.json').read_text()))
        config['stopped_observation_ms'] = config['near_parking']['stopped_observation_ms']
        config['forward']['notes'] = 'Coarse vision supplies steering; one shared physical-side result and encoders supply distance to the inset window near-quarter point, shifted one-quarter depth from its centre toward the arm. Capture distance is separately configured; no old coarse endpoint and no arm action.'
    for name in ('visual_align', 'align_voice', 'blue_tracker', 'ground_projection', 'observations',
                 'continuous_math', 'continuous_approach'):
        files[name + '.py'] = (ROOT / ('device/k230/car/' + name + '.py')).read_bytes()
    for name in ('pose', 'numeric'):
        files[name + '.py'] = (ROOT / ('device/k230/pick/' + name + '.py')).read_bytes()
    if near_parking:
        for name in ('near_vision', 'near_control', 'near_runtime', 'parking_window'):
            files[name + '.py'] = (ROOT / ('device/k230/car/' + name + '.py')).read_bytes()
        for name in ('vision', 'reflection_faces', 'side_faces', 'face_selection',
                     'table_pose', 'top_boundaries'):
            files[name + '.py'] = (ROOT / ('device/k230/pick/' + name + '.py')).read_bytes()
        for name in ('physical_side', 'physical_side_session', 'side_range', 'evidence',
                     'edge_profiles', 'separator_profiles', 'profile_recovery',
                     'cube_pose', 'cube_corners', 'cube_observation'):
            files[name + '.py'] = (ROOT / ('device/k230/vision/' + name + '.py')).read_bytes()
    files['camera.py'] = (ROOT / 'device/k230/vision/camera.py').read_bytes()
    files['vision_runtime.py'] = (ROOT / 'device/k230/vision/runtime.py').read_bytes()
    if near_parking:
        config['runtime_version'] = {
            'source_sha256':{name:digest(data) for name,data in files.items() if name != 'config.json'},
            'vision_config_path':approach['vision_config'],
            'vision_config_sha256':digest((ROOT/approach['vision_config']).read_bytes()),
            'intrinsics_sha256':digest((ROOT/localization['intrinsics']).read_bytes()),
            'extrinsics_sha256':digest((ROOT/localization['extrinsics']).read_bytes())}
    files['config.json'] = encoded(config)
    release_id = digest(b''.join(files[name] for name in sorted(files)))[:16]
    remote = '/sdcard/aic/car_align/releases/' + release_id
    startup = ("import sys\nsys.path.insert(0,%r)\n"
               "from align_voice import run_forever\nrun_forever(%r)\n") % (remote, remote)
    return config, files, remote, startup


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default=PORT)
    parser.add_argument('--install', action='store_true')
    parser.add_argument('--activate', action='store_true')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--approach', action='store_true', help='Enable one forward step when centered; otherwise turn-only')
    mode.add_argument('--auto-approach', action='store_true', help='One wake repeats visual TURN/MOVE steps to the near handoff')
    mode.add_argument('--continuous-approach', action='store_true', help='Continuous visual wheel-speed updates; requires FOLLOW/SPEED firmware')
    mode.add_argument('--near-parking', action='store_true', help='Continuous coarse approach then physical-face localization and near parking; no arm action')
    args = parser.parse_args(argv)
    if args.activate and not args.install:
        parser.error('--activate requires --install')
    config, files, remote, startup = build_bundle(args.approach, args.auto_approach,
                                                 args.continuous_approach, args.near_parking)
    out = ROOT / 'data/deployments' / datetime.now(ZoneInfo('Asia/Shanghai')).strftime('k230_car_align_%Y%m%d_%H%M%S')
    out.mkdir(parents=True)
    for name, data in files.items():
        (out / name).write_bytes(data)
    (out / 'main.py').write_text(startup)
    record = {'status': 'built_not_installed', 'remote': remote, 'activated': False,
              'mode': 'near_visual_parking' if args.near_parking else 'continuous_visual_approach' if args.continuous_approach else 'automatic_visual_approach' if args.auto_approach else 'visual_approach' if args.approach else 'visual_align',
              'log_path': '/sdcard/aic/car_near/runs.jsonl' if args.near_parking else '/sdcard/aic/car_follow/runs.jsonl' if args.continuous_approach else '/sdcard/aic/car_align/runs.jsonl',
              'motion_commands_sent': False, 'arm_commands_sent': False,
              'files': {name: digest(data) for name, data in files.items()}}
    try:
        if args.install:
            link = CameraLink(args.port)
            try:
                stop_previous_script(link)
                mkdir(link, remote)
                if args.near_parking:
                    mkdir(link, '/sdcard/aic/car_near')
                elif args.continuous_approach:
                    mkdir(link, '/sdcard/aic/car_follow')
                for name, data in files.items():
                    write_file(link, remote + '/' + name, data)
                record['status'] = 'installed_not_active'
                if args.activate:
                    lines = execute(link, "import binascii\nwith open('/sdcard/main.py','rb') as f:b=f.read()\nprint('PREVIOUS '+binascii.b2a_base64(b).decode().strip())")
                    (out / 'previous_main.py').write_bytes(base64.b64decode(next(x[9:] for x in lines if x.startswith('PREVIOUS '))))
                    write_file(link, '/sdcard/main.py', startup.encode())
                    record.update(status='active_on_next_boot', activated=True)
            finally:
                link.port.close()
        print('CAR_ALIGN_BUNDLE', record['status'], out)
        return 0
    finally:
        (out / 'deployment.json').write_bytes(encoded(record))


if __name__ == '__main__':
    raise SystemExit(main())
