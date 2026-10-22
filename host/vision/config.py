"""Load the blue-cube profile independently of any arm/demo configuration."""
import json
from pathlib import Path
from host.project_paths import ROOT

VISION_CONFIG = ROOT / 'configs/vision/blue_cube.json'


def load_vision_config(path=None):
    config = json.loads(Path(path or VISION_CONFIG).read_text())
    if config.get('schema') != 'aic.blue_cube_config.v1':
        raise ValueError('Unsupported blue cube configuration')
    return config


def table_height_at(localization,xy_mm):
    """Vertical Base-Z intersection of the configured table plane at XY."""
    import math
    plane=localization.get('table_plane_Base_mm')
    if plane is None:return float(localization['table_z_Base_mm'])
    nx,ny,nz=map(float,plane['normal'])
    if not all(math.isfinite(v) for v in (nx,ny,nz,*xy_mm,plane['offset_mm'])) or abs(nz)<1e-8:
        raise ValueError('Invalid table plane')
    return (float(plane['offset_mm'])-nx*xy_mm[0]-ny*xy_mm[1])/nz
