"""Read-only PC assembly of the shared board-side approach math."""
import hashlib
import json
from pathlib import Path

from host.project_paths import ROOT
from device.k230.car.ground_projection import GroundProjection


def load(root=ROOT):
    root = Path(root)
    config = json.loads((root / 'configs/car/approach.json').read_text())
    workspace = json.loads((root / config['workspace_config']).read_text())
    vision = json.loads((root / config['vision_config']).read_text())
    loc = vision['localization']
    intrinsic_path = root / loc['intrinsics']
    intrinsics = json.loads(intrinsic_path.read_text())
    extrinsics = json.loads((root / loc['extrinsics']).read_text())
    if hashlib.sha256(intrinsic_path.read_bytes()).hexdigest() != extrinsics['intrinsics_sha256']:
        raise ValueError('Camera calibration pair mismatch')
    width, height = config['tracking']['image_size_px']
    if (width, height) != (vision['camera']['width'], vision['camera']['height']):
        raise ValueError('Tracker and precise camera resolutions differ')
    if vision['camera']['hmirror'] or vision['camera']['vflip']:
        raise ValueError('Tracker expects the calibrated unmirrored camera')
    K = [list(row) for row in intrinsics['camera_matrix']]
    K[0] = [v * width / intrinsics['image_width'] for v in K[0]]
    K[1] = [v * height / intrinsics['image_height'] for v in K[1]]
    geometry = GroundProjection(K, intrinsics['distortion_coefficients'],
        extrinsics['T_Base_from_Camera_mm'], loc['table_plane_Base_mm'],
        config.get('forward_axis_Base'), config.get('motion_origin_Base_mm'))
    return workspace, config, geometry
