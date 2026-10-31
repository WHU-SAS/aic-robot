"""One K230-owned precise image; return JSON geometry, keep original JPEG on SD.

The selected release must already be installed without activation. This adapter
never installs a release, changes startup/pointers, reads servos or sends goals.
"""
import hashlib
import json
import math
from pathlib import Path
import time
import uuid

from host.transport.download_calibration import CameraLink, PORT

RESULT_PREFIX = 'PICK_PERCEPTION_RESULT '
RESULT_SCHEMA = 'aic.k230.pick_perception.v1'


def _sha(data):
    return hashlib.sha256(data).hexdigest()


def _stop_previous_script(link):
    # Lazy imports also keep the PC entry -> deploy -> PC parameters cycle out
    # of module initialization. Stopping includes the existing soft-reboot wait.
    from tools.diagnostics.test_k230_motor_uart import stop_previous_script
    stop_previous_script(link)


def build_script(remote, manifest, manifest_sha256, request_id):
    """Call only app.perceive after checking the exact installed bundle bytes."""
    modules = sorted(name[:-3] for name in manifest['files'] if name.endswith('.py'))
    return '''import os,sys,gc,json,time,binascii,uhashlib
request_id=%r
remote=%r
release_id=%r
expected_manifest_sha256=%r
expected_files=json.loads(%r)
result={'schema':%r,'request_id':request_id,'release_id':release_id,
        'release_path':remote,'status':'error','motion_performed':False,
        'verified_files':False,'evidence':None,'archive_errors':[]}
started=time.ticks_ms()
def file_hash(path):
 h=uhashlib.sha256();size=0
 with open(path,'rb') as f:
  while True:
   data=f.read(4096)
   if not data:break
   h.update(data);size+=len(data)
 return binascii.hexlify(h.digest()).decode(),size
try:
 try:
  actual,unused=file_hash(remote+'/manifest.json')
 except OSError:
  raise RuntimeError('Selected K230 perception release is not installed; install the selected pick profile without --activate')
 if actual!=expected_manifest_sha256:raise RuntimeError('Selected K230 perception manifest mismatch')
 with open(remote+'/manifest.json') as f:installed=json.load(f)
 if installed.get('release_id')!=release_id or installed.get('files')!=expected_files:
  raise RuntimeError('Selected K230 perception release identity mismatch')
 for name,item in expected_files.items():
  actual,size=file_hash(remote+'/'+name)
  if actual!=item['sha256'] or size!=item['bytes']:
   raise RuntimeError('Selected K230 perception file mismatch: '+name)
 result['verified_files']=True
 result['config_sha256']=expected_files['config.json']['sha256']
 result['manifest_sha256']=expected_manifest_sha256
 result['verification_ms']=time.ticks_diff(time.ticks_ms(),started)
 for name in %r:
  if name in sys.modules:del sys.modules[name]
 sys.path.insert(0,remote)
 gc.collect()
 import app
 config=app.load(remote)
 original_log=app.log
 def perception_log(event,details=None,persist=True):
  if event=='vision_evidence_saved':result['evidence']=details
  elif event=='vision_archive_error':result['archive_errors'].append(str(details))
  original_log(event,details,persist)
 app.log=perception_log
 phase=time.ticks_ms()
 found=app.perceive(config)
 result['perception_ms']=time.ticks_diff(time.ticks_ms(),phase)
 result['found']=found
 result['status']='localized'
except BaseException as exc:
 result['error']=str(exc)
finally:
 result['request_total_ms']=time.ticks_diff(time.ticks_ms(),started)
 print(%r+json.dumps(result))
 print('EXPORT_DONE')
''' % (request_id, remote, manifest['release_id'], manifest_sha256,
       json.dumps(manifest['files']), RESULT_SCHEMA, modules, RESULT_PREFIX)


def _localized_found(result):
    """Validate the existing precise-result capability, without changing gates."""
    found = result.get('found')
    if not isinstance(found, dict) or found.get('status') != 'complete_face_candidate':
        raise RuntimeError('K230 did not return a complete physical-face result')
    point = found.get('cube_center_Base_mm')
    if (not isinstance(point, (list, tuple)) or len(point) != 3 or
            any(type(v) not in (int, float) or not math.isfinite(v) for v in point)):
        raise RuntimeError('K230 did not return finite Base XYZ')
    observation = found.get('observation')
    expected = {'schema':'aic.vision_observation.v1', 'mode':'precise', 'status':'localized',
                'coordinate_frame':'Base', 'units':'mm', 'reference_point':'cube_geometric_center'}
    if (not isinstance(observation, dict) or
            any(observation.get(k) != v for k,v in expected.items()) or
            observation.get('xy_compensation_applied') is not False or
            observation.get('cube_center_Base_mm') != list(point)):
        raise RuntimeError('K230 precise observation does not match the raw Base result')
    stamp = observation.get('capture_timestamp_ms')
    if (observation.get('frame_id') is None or observation.get('image_size_px') != [1280,960] or
            type(stamp) is not int or stamp < 0 or observation.get('timestamp_ms', stamp) != stamp or
            observation.get('clock_domain') != 'k230_ticks_ms' or
            observation.get('odometry_predicted') is True or observation.get('range_only') is True):
        raise RuntimeError('K230 precise observation has no valid original capture identity')
    rms = (observation.get('quality') or {}).get('reprojection_rms_px')
    if type(rms) not in (int,float) or not math.isfinite(rms) or not 0 <= rms <= 2:
        raise RuntimeError('K230 precise result does not meet the existing2px criterion')
    return found


def _write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')


def perceive(output, pick_config=None, port=PORT):
    """Get fresh board localization from an already installed explicit profile.

    Success writes vision_record/context, onboard_perception and camera_log.
    Failure keeps context/log/protocol diagnostics and raises before arm access.
    Original inference JPEG/config/manifest remain in the board EvidenceRun;
    ordinary calls never export image bytes or fall back to an old result.
    """
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    if any((output/name).exists() for name in ('vision_record.json','onboard_perception.json')):
        raise FileExistsError('Perception output already contains a result; use a new run directory')
    request_id = uuid.uuid4().hex
    record = {'schema':RESULT_SCHEMA, 'request_id':request_id,
              'status':'requested', 'motion_performed':False}
    lines = []
    link = None
    began = time.monotonic()
    try:
        # k230_pick imports the PC entry for parameters: do not move this import
        # to module scope, where the live PC entry is itself importing us.
        from tools.deploy.k230_pick import build_bundle
        config, files, manifest = build_bundle(pick_config)
        remote = '/sdcard/aic/pick/releases/'+manifest['release_id']
        manifest_sha = _sha(files['manifest.json'])
        record.update(release_id=manifest['release_id'], release_path=remote)
        localization = config['vision']['localization']
        context = {'schema':'aic.vision_context.v1', 'perception_backend':'k230',
            'vision':config['vision'], 'intrinsics':config['intrinsics'],
            'extrinsics':config['extrinsics'],
            'table_plane':localization.get('table_plane_Base_mm')
                if localization.get('table_pose_constraints_enabled') else None,
            'version':{'release_id':manifest['release_id'], 'release_path':remote,
                'manifest_sha256':manifest_sha,
                'source_sha256':{name:item['sha256'] for name,item in manifest['files'].items()
                                 if name.endswith('.py')}},
            'request_id':request_id, 'config_sha256':manifest['files']['config.json']['sha256'],
            'calibration_sources':{'intrinsics':localization['intrinsics'],
                                   'extrinsics':localization['extrinsics']}}
        _write_json(output/'vision_context.json', context)
        link = CameraLink(port)
        _stop_previous_script(link)
        terminal = []
        for raw in link.execute(build_script(remote, manifest, manifest_sha, request_id), timeout=180):
            text = raw.decode(errors='replace')
            lines.append(text)
            if text.startswith(RESULT_PREFIX):
                terminal.append(json.loads(text[len(RESULT_PREFIX):]))
        if len(terminal) != 1:
            raise RuntimeError('Expected one current K230 perception result; received '+str(len(terminal)))
        result = terminal[0]
        record['board_result'] = result
        if (not isinstance(result,dict) or result.get('schema') != RESULT_SCHEMA or
                result.get('request_id') != request_id or result.get('release_id') != manifest['release_id'] or
                result.get('release_path') != remote):
            raise RuntimeError('K230 perception request or release mismatch')
        if result.get('status') != 'localized':
            raise RuntimeError(result.get('error') or 'K230 precise localization failed')
        if (result.get('verified_files') is not True or
                result.get('config_sha256') != manifest['files']['config.json']['sha256'] or
                result.get('manifest_sha256') != manifest_sha or result.get('motion_performed') is not False):
            raise RuntimeError('K230 perception bundle verification is incomplete')
        found = _localized_found(result)
        vision = {'status':'localized', 'method':'K230 onboard JPEG95 physical-face localization',
            'perception_backend':'k230', 'cube_center_Base_mm':list(found['cube_center_Base_mm']),
            'observation':found['observation'], 'vision':found,
            'perception_timing_ms':found.get('perception_timing_ms',{}),
            'evidence':result.get('evidence'), 'archive_errors':result.get('archive_errors',[]),
            'release_id':manifest['release_id'], 'release_path':remote, 'request_id':request_id,
            'board_timing_ms':{key:result.get(key) for key in
                ('verification_ms','perception_ms','request_total_ms')},
            'motion_performed':False, 'xy_compensation_applied':False}
        if 'cube_center_Camera_mm' in found:
            vision['cube_center_Camera_mm'] = found['cube_center_Camera_mm']
        if localization.get('table_plane_Base_mm') is not None:
            vision['table_plane_Base_mm'] = localization['table_plane_Base_mm']
        _write_json(output/'vision_record.json', vision)
        record.update(status='localized', evidence=vision['evidence'])
        return vision
    except BaseException as exc:
        record.update(status='error', error=repr(exc))
        raise
    finally:
        try:
            if link is not None:
                link.port.close()
        finally:
            record['transport_timing_s'] = {'total':time.monotonic()-began}
            (output/'camera_log.txt').write_text('\n'.join(lines)+'\n')
            _write_json(output/'onboard_perception.json', record)
