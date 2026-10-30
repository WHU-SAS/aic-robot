"""Local, append-only hard-image evidence and offline precise-vision replay.

Predictions are not ground truth. Missing historical calibration is never filled
silently from today's configuration. No camera, serial, or motion APIs are used.
"""
import hashlib
import json
import math
import re
import time
from datetime import datetime, timezone
from pathlib import Path

from host.project_paths import ROOT

DEFAULT_DATASET = ROOT / 'data/vision/hard_cases'
REASONS = ('recognition_failure', 'inaccurate', 'selected')


def _stamp():
    return datetime.now(timezone.utc).isoformat()


def _bytes_hash(data):
    return hashlib.sha256(data).hexdigest()


def _canonical(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


def _json(path):
    return json.loads(Path(path).read_text())


def _new_json(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')


def _copy_once(path, data):
    path = Path(path)
    if path.exists():
        if path.read_bytes() != data:
            raise ValueError('Refusing to overwrite evidence: ' + str(path))
        return
    with path.open('xb') as stream:
        stream.write(data)


def _resolve(path, parent):
    path = Path(path)
    if path.is_absolute():
        return path
    local = parent / path
    return local if local.exists() else ROOT / path


def calibration_snapshot(config_path, role='comparison'):
    """Capture exact config plus resolved K/D/T/table; role must be explicit."""
    path = Path(config_path).resolve()
    config = _json(path)
    vision = config.get('vision', config)
    loc = vision.get('localization', vision)
    files = {str(path): path.read_bytes()}
    loaded = {}
    for key in ('intrinsics', 'extrinsics'):
        value = config.get(key, loc.get(key))
        if isinstance(value, dict):
            loaded[key] = value
        elif isinstance(value, str):
            target = _resolve(value, path.parent)
            if not target.is_file():
                raise ValueError('Missing calibration file: ' + str(target))
            # Historical snapshots must not resolve to mutable repository files.
            if role == 'original' and not target.is_relative_to(path.parent):
                raise ValueError('Historical calibration points outside its saved snapshot')
            files[str(target)] = target.read_bytes()
            loaded[key] = _json(target)
        else:
            raise ValueError('Configuration has no saved ' + key)
    intr, ext = loaded['intrinsics'], loaded['extrinsics']
    for key in ('camera_matrix', 'distortion_coefficients', 'image_width', 'image_height'):
        if key not in intr:
            raise ValueError('Missing intrinsics field: ' + key)
    if 'T_Base_from_Camera_mm' not in ext:
        raise ValueError('Missing extrinsics transform')
    intr_ref = config.get('intrinsics', loc.get('intrinsics'))
    if isinstance(intr_ref, str) and ext.get('intrinsics_sha256'):
        actual = _bytes_hash(_resolve(intr_ref, path.parent).read_bytes())
        if actual != ext['intrinsics_sha256']:
            raise ValueError('Intrinsics/extrinsics SHA256 mismatch')
    table = config.get('table_plane', config.get('table_plane_Base_mm'))
    if table is None and loc.get('table_pose_constraints_enabled'):
        table = loc.get('table_plane_Base_mm')
    geometry = dict(intrinsics=intr, extrinsics=ext, table_plane=table)
    return dict(schema='aic.vision_calibration_snapshot.v1', role=role,
                sha256=_bytes_hash(_canonical(geometry)), **geometry), files


def summarize(observation):
    """Normalize old PC, native and shared observations without inventing labels."""
    observation = observation or {}
    raw = observation.get('payload', observation.get('vision', observation))
    if not isinstance(raw, dict):
        raw = {}
    poses = raw.get('poses') or []
    pose = poses[0] if poses else {}
    quality = observation.get('quality') or {}
    face_kind = pose.get('table_geometry', {}).get('face_kind')
    if face_kind is None:
        if raw.get('side_faces') or quality.get('side_selection', raw.get('side_selection')):
            face_kind = 'side'
        elif raw.get('top_boundary_verified'):
            face_kind = 'top'
    status = observation.get('status', raw.get('status', 'unknown'))
    if status == 'complete_face_candidate':
        status = 'localized'
    return dict(status=status, center_Base_mm=observation.get('cube_center_Base_mm', raw.get('cube_center_Base_mm')),
                reason=observation.get('reason', raw.get('reason')),
                reprojection_rms_px=quality.get('reprojection_rms_px', pose.get('face_reprojection_rms_px')),
                side_selection=quality.get('side_selection', raw.get('side_selection')),
                face_kind=face_kind,
                vision_path=quality.get('vision_path', raw.get('vision_path')))


def _source_material(source):
    source = Path(source).resolve()
    folder = source if source.is_dir() else source.parent
    image = folder / 'frame.jpg' if source.is_dir() else source
    if not image.is_file():
        raise ValueError('No frame.jpg: ' + str(image))
    files = {}
    records = {}
    for name in ('vision_record.json', 'vision_candidates.json', 'execution.json', 'capture.json',
                 'manifest.json', 'observed.json', 'observation.json', 'config.json', 'vision_context.json'):
        path = folder / name
        if path.is_file():
            files[str(path)] = path.read_bytes()
            records[name] = _json(path)
    old = records.get('vision_record.json', records.get('vision_candidates.json'))
    if old is None:
        old = records.get('observation.json', records.get('execution.json', {}).get('vision', {}))
    observed = (old or {}).get('observation', old or {})
    frame = observed.get('frame')
    if frame is None and observed.get('frame_id') is not None:
        frame = {key: observed[key] for key in ('frame_id', 'capture_timestamp_ms', 'clock_domain',
                 'timestamp_source', 'image_size_px') if key in observed}
    return image, files, old or {}, frame, records


def _append_label(folder, reason, note='', expected=None, source=None):
    if reason not in REASONS:
        raise ValueError('Reason must be one of: ' + ', '.join(REASONS))
    event = dict(timestamp=_stamp(), reason=reason, note=note, source=source)
    if expected:
        center = expected.get('center_Base_mm')
        if center is not None and (len(center) != 3 or not all(math.isfinite(v) for v in center)):
            raise ValueError('Expected center requires three finite human-provided values')
        tolerance = expected.get('tolerance_mm', 6)
        if not math.isfinite(tolerance) or tolerance <= 0:
            raise ValueError('Expected center tolerance must be positive')
        event['expected'] = expected
    with (folder / 'labels.jsonl').open('a') as stream:
        stream.write(json.dumps(event, ensure_ascii=False, allow_nan=False) + '\n')
    return event


def label_case(dataset, case_id, reason, note='', expected=None):
    if not re.fullmatch(r'[0-9a-f]{16}-[0-9a-f]{12}', case_id):
        raise ValueError('Invalid case ID')
    folder = Path(dataset) / 'cases' / case_id
    _json(folder / 'case.json')  # Require an existing case, not an arbitrary path.
    return _append_label(folder, reason, note, expected)


def add_case(source, reason, note='', dataset=DEFAULT_DATASET, vision_config=None,
             observation=None, frame=None, original_config=None, source_files=None,
             source_version=None, expected=None):
    """Copy one original; duplicate image+calibration appends evidence/labels only."""
    image, files, old, inferred_frame, records = _source_material(source)
    if source_files:
        files.update({str(Path(p).resolve()): Path(p).read_bytes() for p in source_files})
    old = old if observation is None else observation
    frame = inferred_frame if frame is None else frame
    calibration, metadata_gaps = None, []
    if original_config is None:
        if records.get('vision_context.json'):
            original_config = image.parent / 'vision_context.json'
            source_version = source_version or records['vision_context.json'].get('version')
        elif records.get('config.json'):
            original_config = image.parent / 'config.json'
    if original_config:
        try:
            calibration, more = calibration_snapshot(original_config, 'original')
            files.update(more)
        except (ValueError, OSError, KeyError) as exc:
            metadata_gaps.append('Original calibration unavailable: ' + str(exc))
    if calibration is None and vision_config:
        calibration, more = calibration_snapshot(vision_config, 'comparison')
        files.update(more)
        metadata_gaps.append('Explicit comparison calibration; original calibration is unknown')
    if calibration is None:
        metadata_gaps.append('Original calibration snapshot missing; replay needs explicit comparison config')
    if frame is None:
        metadata_gaps.append('Original capture frame/time metadata unavailable')
    data = image.read_bytes()
    digest = _bytes_hash(data)
    captured_digest = records.get('capture.json', {}).get('sha256')
    if captured_digest and captured_digest != digest:
        raise ValueError('PC capture image SHA256 mismatch')
    binding = 'unknown' if calibration is None else calibration['role'] + ':' + calibration['sha256']
    case_id = digest[:16] + '-' + _bytes_hash(binding.encode())[:12]
    folder = Path(dataset) / 'cases' / case_id
    folder.mkdir(parents=True, exist_ok=True)
    (folder / 'evidence').mkdir(exist_ok=True)
    image_file = 'image' + (image.suffix.lower() or '.jpg')
    _copy_once(folder / image_file, data)
    for original_path, content in files.items():
        name = Path(original_path).name
        _copy_once(folder / 'evidence' / (_bytes_hash(content)[:16] + '_' + name), content)
    path = folder / 'case.json'
    if not path.exists():
        if calibration:
            _new_json(folder / 'calibration.json', calibration)
        _new_json(path, dict(schema='aic.vision_hard_case.v1', case_id=case_id, imported_at=_stamp(),
                            source=str(image), image_file=image_file, image_sha256=digest,
                            old_observation=old, frame=frame, source_version=source_version,
                            calibration_file='calibration.json' if calibration else None,
                            calibration_role=calibration['role'] if calibration else None,
                            calibration_sha256=calibration['sha256'] if calibration else None,
                            metadata_gaps=metadata_gaps,
                            source_files=[dict(path=p, sha256=_bytes_hash(b)) for p, b in files.items()]))
    else:
        existing = _json(path)
        if existing['image_sha256'] != digest or existing['calibration_sha256'] != (calibration or {}).get('sha256'):
            raise ValueError('Case identity collision; refusing overwrite')
    _append_label(folder, reason, note, expected, str(image))
    return case_id


def _jpeg_problem(data):
    if not data.startswith(b'\xff\xd8') or not data.endswith(b'\xff\xd9'):
        return 'Missing complete JPEG markers; original evidence is incomplete'
    import cv2
    import numpy as np
    decoded = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
    if decoded is None or tuple(decoded.shape[:2]) != (960, 1280):
        return 'JPEG cannot be decoded as a complete 1280x960 precise frame'
    return None


def import_evidence(source, dataset=DEFAULT_DATASET, reason=None, note=''):
    """Import board evidence. Successful images require explicit human selection."""
    source = Path(source).resolve()
    if source.is_dir():
        source = source / 'manifest.json'
    manifest = _json(source)
    if manifest.get('schema') != 'aic.vision_evidence.v1':
        raise ValueError('Unsupported evidence schema')
    imported, skipped = [], []
    for entry in manifest.get('frames', []):
        image_name = entry.get('image_file')
        image = source.parent / image_name if image_name else None
        if image is None or not image.is_file():
            skipped.append(dict(label=entry.get('label'), image_status='missing', reason='Image missing from evidence'))
            continue
        data = image.read_bytes()
        if entry.get('image_sha256') and entry['image_sha256'] != _bytes_hash(data):
            raise ValueError('Evidence image SHA256 mismatch: ' + str(image))
        if entry.get('image_bytes') is not None and entry['image_bytes'] != len(data):
            raise ValueError('Evidence image size mismatch: ' + str(image))
        image_problem = _jpeg_problem(data)
        if image_problem:
            skipped.append(dict(label=entry.get('label'), image_status='invalid', reason=image_problem))
            continue
        observation = entry.get('observation') or {}
        failed = summarize(observation)['status'] in ('failed', 'no_complete_face', 'error', 'no_target')
        failed = failed or (not observation and manifest.get('status') in ('failed', 'error', 'perception_error'))
        selected = reason or ('recognition_failure' if failed else None)
        if selected is None:
            skipped.append(dict(label=entry.get('label'), reason='No difficulty label; use --reason selected or inaccurate'))
            continue
        config_name = manifest.get('config_file')
        config = source.parent / config_name if config_name else None
        imported.append(add_case(image, selected, note, dataset, observation=observation,
                                 frame=entry.get('frame'), original_config=config,
                                 source_files=[source], source_version=manifest.get('version')))
    return dict(status='partial' if any(row.get('image_status') in ('missing', 'invalid') for row in skipped) else 'complete',
                imported=imported, skipped=skipped)


def list_cases(dataset=DEFAULT_DATASET):
    result = []
    for path in sorted((Path(dataset) / 'cases').glob('*/case.json')):
        case = _json(path)
        labels = [json.loads(line) for line in (path.parent / 'labels.jsonl').read_text().splitlines() if line]
        result.append(dict(case_id=case['case_id'], image_sha256=case['image_sha256'],
                           calibration_role=case['calibration_role'], old=summarize(case['old_observation']),
                           labels=labels, metadata_gaps=case['metadata_gaps']))
    return result


def _expectations(labels, actual, allow_metric=True):
    expected = {}
    for event in labels:
        expected.update(event.get('expected', {}))
    if not expected:
        return dict(status='uncertain', reason='No explicit human expected result; localization is not proof of accuracy')
    checks, extra = {}, {}
    if 'status' in expected:
        checks['status'] = actual['status'] == expected['status']
    if 'face_kind' in expected:
        checks['face_kind'] = actual['status'] == 'localized' and actual['face_kind'] == expected['face_kind']
    if 'center_Base_mm' in expected:
        if allow_metric:
            center = actual.get('center_Base_mm')
            error = math.dist(center, expected['center_Base_mm']) if center is not None else None
            checks['center'] = error is not None and error <= expected.get('tolerance_mm', 6)
            extra['human_center_error_mm'] = error
        else:
            extra['center_skipped'] = 'Comparison calibration; historical Base labels are not transferable'
    status = ('pass' if all(checks.values()) else 'fail') if checks else 'uncertain'
    return dict(status=status, checks=checks, expected=expected, **extra)


def replay_case(folder, vision_config=None, localizer=None, image_loader=None):
    """Run saved image once; injected functions are only for mechanics tests."""
    folder = Path(folder)
    case = _json(folder / 'case.json')
    image = folder / case['image_file']
    report = dict(case_id=case['case_id'], old=summarize(case['old_observation']),
                  metadata_gaps=list(case['metadata_gaps']), delta_mm=None, elapsed_ms=None)
    if _bytes_hash(image.read_bytes()) != case['image_sha256']:
        raise ValueError('Archived original image SHA256 mismatch')
    calibration = _json(folder / case['calibration_file']) if case['calibration_file'] else None
    if calibration is None and vision_config:
        calibration, _ = calibration_snapshot(vision_config, 'comparison')
    if calibration is None:
        report.update(replay_status='skipped', reason='Missing original calibration and no explicit comparison config')
        return report
    actual_hash = _bytes_hash(_canonical({k: calibration[k] for k in ('intrinsics', 'extrinsics', 'table_plane')}))
    if actual_hash != calibration['sha256'] or (case['calibration_sha256'] and actual_hash != case['calibration_sha256']):
        raise ValueError('Archived calibration SHA256 mismatch')
    if vision_config and case['calibration_file']:
        report['comparison_config_ignored'] = 'Saved calibration retained; comparison only fills missing snapshots'
    report['calibration_role'] = calibration['role']
    report['calibration_sha256'] = actual_hash
    report['original_calibration_known'] = calibration['role'] == 'original'
    if image_loader is None:
        import cv2
        image_loader = lambda p: cv2.imread(str(p))
    im = image_loader(image)
    if im is None or tuple(im.shape[:2]) != (960, 1280):
        report.update(replay_status='failed', reason='Expected unmodified 1280x960 K230 source image')
        return report
    frame = case.get('frame') or {}
    frame = dict(frame)
    frame['frame_id'] = frame.get('frame_id') or 'saved:' + case['image_sha256']
    frame.setdefault('image_size_px', [1280, 960])
    frame.setdefault('capture_timestamp_ms', None)
    frame.setdefault('clock_domain', None)
    frame.setdefault('timestamp_source', 'unavailable')
    if localizer is None:
        from device.k230.vision.runtime import localize_precise
        localizer = localize_precise
    started = time.monotonic()
    try:
        observation = localizer(im, frame, calibration['intrinsics'], calibration['extrinsics'], calibration['table_plane'])
        report.update(replay_status='completed', new=summarize(observation), observation=observation)
    except Exception as exc:
        report.update(replay_status='failed', reason=type(exc).__name__ + ': ' + str(exc))
        return report
    finally:
        report['elapsed_ms'] = (time.monotonic() - started) * 1000
    old_record = case.get('old_observation') or {}
    old_frame_id = old_record.get('observation', old_record).get('frame_id')
    frame_matches = old_frame_id is None or old_frame_id == frame['frame_id']
    if calibration['role'] == 'original' and frame_matches and report['old']['center_Base_mm'] is not None and report['new']['center_Base_mm'] is not None:
        report['delta_mm'] = [b - a for a, b in zip(report['old']['center_Base_mm'], report['new']['center_Base_mm'])]
        report['delta_meaning'] = 'Same saved image/calibration; prediction change only, not accuracy error'
    else:
        report['delta_reason'] = 'No old center, original calibration unknown, or frame mismatch; coordinate delta not comparable'
    labels = [json.loads(line) for line in (folder / 'labels.jsonl').read_text().splitlines() if line]
    report['evaluation'] = _expectations(labels, report['new'], allow_metric=calibration['role'] == 'original')
    return report


def replay(dataset=DEFAULT_DATASET, case_ids=None, vision_config=None, output=None):
    dataset = Path(dataset)
    cases = list_cases(dataset)
    known = {row['case_id'] for row in cases}
    if case_ids and set(case_ids) - known:
        raise ValueError('Unknown case IDs: ' + ', '.join(set(case_ids) - known))
    chosen = [row for row in cases if not case_ids or row['case_id'] in case_ids]
    output = Path(output) if output else dataset / 'reports' / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
    output.mkdir(parents=True, exist_ok=False)
    sources = sorted((ROOT / 'device/k230/pick').glob('*.py')) + sorted((ROOT / 'device/k230/vision').glob('*.py'))
    report = dict(schema='aic.vision_hard_replay.v1', timestamp=_stamp(),
                  algorithm_sha256={str(p.relative_to(ROOT)): _bytes_hash(p.read_bytes()) for p in sources}, cases=[])
    if vision_config:
        comparison, _ = calibration_snapshot(vision_config, 'comparison')
        report['explicit_comparison_calibration'] = comparison
    for row in chosen:
        result = replay_case(dataset / 'cases' / row['case_id'], vision_config)
        _new_json(output / (row['case_id'] + '.json'), result)
        report['cases'].append(result)
    summary = dict(localized=0, rejected=0, errors=0, skipped=0,
                   expectation_pass=0, expectation_fail=0, expectation_uncertain=0)
    for case in report['cases']:
        status = case['replay_status']
        bucket = ('localized' if case.get('new', {}).get('status') == 'localized' else 'rejected') if status == 'completed' else ('skipped' if status == 'skipped' else 'errors')
        summary[bucket] += 1
        summary['expectation_' + case.get('evaluation', {}).get('status', 'uncertain')] += 1
    check_status = ('fail' if not report['cases'] or summary['errors'] or summary['expectation_fail']
                    else 'uncertain' if summary['expectation_uncertain'] else 'pass')
    report.update(summary=summary, check_status=check_status,
                  check_scope='Explicit human expectations and runtime errors; unlabeled recognition is not accuracy proof')
    _new_json(output / 'report.json', report)
    return dict(report=str(output / 'report.json'), cases=len(report['cases']),
                completed=sum(c['replay_status'] == 'completed' for c in report['cases']),
                summary=summary, check_status=check_status)
