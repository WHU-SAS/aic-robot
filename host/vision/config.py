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
