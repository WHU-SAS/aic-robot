#!/usr/bin/env python3
"""Build and verify the combined car/arm release; activation only changes boot."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT, bootstrap
bootstrap()
import argparse
import base64
import json
from datetime import datetime
from zoneinfo import ZoneInfo
from device.k230.car.ground_projection import GroundProjection
from host.transport.download_calibration import CameraLink, PORT
from tools.deploy.k230_car_align import build_bundle as car_bundle
from tools.deploy.k230_pick import (build_bundle as arm_bundle, digest, encoded,
                                   execute, mkdir, write_file)
from tools.diagnostics.test_k230_motor_uart import stop_previous_script

REMOTE = '/sdcard/aic/robot'
DEFAULT_PICK_CONFIG = ROOT / 'configs/arm/pick_wrist_rotation.json'


def _paired_config(car, arm):
    for name in ('intrinsics', 'extrinsics'):
        if car[name] != arm[name]:
            raise ValueError('Car/arm calibration differs: ' + name)
    plane = arm['vision']['localization']['table_plane_Base_mm']
    if car['table_plane'] != plane:
        raise ValueError('Car/arm calibration differs: table_plane')
    intrinsics = car['intrinsics']
    width, height = car['tracking']['image_size_px']
    K = [list(row) for row in intrinsics['camera_matrix']]
    K[0] = [v * width / intrinsics['image_width'] for v in K[0]]
    K[1] = [v * height / intrinsics['image_height'] for v in K[1]]
    transform = car['workspace'].get('coordinate_reexpression', {}).get(
        'T_new_Base_from_old_Base_mm')
    axis = [row[0] for row in transform[:3]] if transform else None
    origin = [row[3] for row in transform[:3]] if transform else None
    geometry = GroundProjection(K, intrinsics['distortion_coefficients'],
        car['extrinsics']['T_Base_from_Camera_mm'], plane, axis, origin)
    expected = {'K':geometry.K, 'D':geometry.D, 'T':geometry.T,
                'plane':{'normal':geometry.normal, 'offset_mm':geometry.offset},
                'forward_axis_Base':geometry.forward_axis_Base,
                'motion_origin_Base_mm':geometry.motion_origin_Base_mm}
    if car['projection'] != expected:
        raise ValueError('Car projection differs from the paired calibration')


def build_bundle(pick_config=None, tracking=False):
    car, car_files, _, _ = car_bundle(near_parking=True)
    arm, arm_files, _ = arm_bundle(DEFAULT_PICK_CONFIG if pick_config is None else pick_config)
    _paired_config(car, arm)
    config = {'car':car, 'arm':arm, 'voice':arm['voice']}
    files = {}
    for source in (car_files, arm_files):
        for name, data in source.items():
            if name in ('config.json', 'manifest.json'):
                continue
            if name in files and digest(files[name]) != digest(data):
                raise ValueError('Common file differs: ' + name)
            files[name] = data
    for name, path in (
            ('robot_task.py', 'device/k230/robot/task.py'),
            ('robot_app.py', 'device/k230/robot/app.py'),
            ('static_pick_session.py', 'device/k230/vision/static_pick_session.py')):
        data = (ROOT / path).read_bytes()
        if name in files and digest(files[name]) != digest(data):
            raise ValueError('Common file differs: ' + name)
        files[name] = data
    if tracking:
        car['tracking_task'] = json.loads((ROOT / 'configs/car/tracking_task.json').read_text())
        for name, path in (
                ('target_memory.py', 'device/k230/car/target_memory.py'),
                ('tracking_drive.py', 'device/k230/car/tracking_drive.py'),
                ('robot_tracking.py', 'device/k230/robot/tracking.py'),
                ('candidate_association.py', 'device/k230/vision/candidate_association.py'),
                ('cooperative_static_session.py', 'device/k230/vision/cooperative_static_session.py')):
            files[name] = (ROOT / path).read_bytes()
    files['config.json'] = encoded(config)
    manifest = {'schema':'aic.robot.v1', 'trigger':'思达思达',
        'startup_motion':False, 'pc_runtime_required':False,
        'completion':'arm_stage_completed_with_encoder_grip_confirmation',
        'files':{name:{'bytes':len(data), 'sha256':digest(data)}
                 for name, data in sorted(files.items())}}
    if tracking:
        manifest.update(mode='continuous_tracking',
            completion='encoder_grip_confirmed_after_lift',
            continuous_observation=True, requires_mspm0_arc=True,
            physical_tracking_validated=False)
    manifest['release_id'] = digest(encoded(manifest))[:16]
    files['manifest.json'] = encoded(manifest)
    return config, files, manifest


def startup(remote):
    return ("# Boot only listens; no deployment or startup motion.\n"
            "import sys\nsys.path.insert(0,%r)\n"
            "from robot_app import run_forever\nrun_forever(%r)\n") % (remote, remote)


def verify(link, remote, manifest, config):
    """Read/hash/import the release without opening UART, camera or audio."""
    modules = [name[:-3] for name in sorted(manifest['files'])
               if name.endswith('.py') and name != '__init__.py']
    source = """import sys,json,binascii,uhashlib
sys.path.insert(0,%r)
with open(%r+'/manifest.json','rb') as f:raw=f.read()
if len(raw)!=%d or binascii.hexlify(uhashlib.sha256(raw).digest()).decode()!=%r:
 raise RuntimeError('Release manifest corrupt')
m=json.loads(raw)
for name,item in m['files'].items():
 h=uhashlib.sha256();size=0
 with open(%r+'/'+name,'rb') as f:
  while True:
   block=f.read(4096)
   if not block:break
   h.update(block);size+=len(block)
 if size!=item['bytes'] or binascii.hexlify(h.digest()).decode()!=item['sha256']:
  raise RuntimeError('Release corrupt: '+name)
modules=%r
for name in modules:
 if name in sys.modules:del sys.modules[name]
for name in modules:__import__(name)
from robot_app import load,run_forever
from robot_task import execute_task
from static_pick_session import StaticPickSession
from app import run_from_observation,run_ready,wait_for_wake
cfg=load(%r)
if m['schema']!='aic.robot.v1' or not all(key in cfg for key in ('car','arm','voice')):
 raise ValueError('Invalid robot release')
print('ROBOT_RELEASE_VERIFIED '+m['release_id'])
""" % (remote, remote, len(encoded(manifest)), digest(encoded(manifest)),
       remote, modules, remote)
    if 'ROBOT_RELEASE_VERIFIED ' + manifest['release_id'] not in execute(link, source):
        raise RuntimeError('Board robot import/hash check failed')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default=PORT)
    parser.add_argument('--install', action='store_true')
    parser.add_argument('--activate', action='store_true')
    parser.add_argument('--pick-config', type=Path, help='override the accepted wrist-aligned arm stage profile used by default')
    parser.add_argument('--tracking', action='store_true', help='build the new persistent tracking candidate; default keeps the single-task comparison')
    args = parser.parse_args(argv)
    if args.activate and not args.install:
        parser.error('--activate requires --install')
    config, files, manifest = build_bundle(args.pick_config, tracking=True) if args.tracking else build_bundle(args.pick_config)
    remote = REMOTE + '/releases/' + manifest['release_id']
    stamp = datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f')
    output = ROOT / 'data/deployments' / ('k230_robot_' + stamp)
    output.mkdir(parents=True)
    for name, data in files.items():
        (output / name).write_bytes(data)
    boot = startup(remote).encode()
    (output / 'main.py').write_bytes(boot)
    record = {'status':'built_no_hardware_access', 'remote':remote,
        'release_id':manifest['release_id'], 'activated':False,
        'motion_commands_sent':False, 'arm_commands_sent':False,
        'files':{name:digest(data) for name, data in files.items()}}
    link = None
    try:
        if args.install:
            link = CameraLink(args.port)
            stop_previous_script(link)
            mkdir(link, remote)
            mkdir(link, '/sdcard/aic/pick')
            for name, data in files.items():
                write_file(link, remote + '/' + name, data)
            verify(link, remote, manifest, config)
            record['status'] = 'installed_verified_not_active'
            if args.activate:
                source = """import binascii
try:
 with open('/sdcard/main.py','rb') as f:previous=f.read()
 print('BACKUP '+binascii.b2a_base64(previous).decode().strip())
except OSError as exc:
 if exc.args[0]!=2:raise
 print('BACKUP_MISSING')
"""
                lines = execute(link, source)
                backup = next((line[7:] for line in lines if line.startswith('BACKUP ')), None)
                if backup is not None:
                    (output / 'previous_main.py').write_bytes(base64.b64decode(backup))
                elif 'BACKUP_MISSING' not in lines:
                    raise RuntimeError('Cannot back up the previous startup')
                write_file(link, '/sdcard/main.py', boot)
                record.update(status='activated_next_boot_only', activated=True)
        print('ROBOT_DEPLOYMENT', record['status'], remote, flush=True)
        return 0
    except Exception as exc:
        record.update(status='failed', error=repr(exc))
        print(str(exc), file=sys.stderr)
        return 1
    finally:
        if link is not None:
            link.port.close()
        (output / 'deployment.json').write_bytes(encoded(record))
        print('RECORD', output)


if __name__ == '__main__':
    raise SystemExit(main())
