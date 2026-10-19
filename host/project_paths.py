"""Canonical repository paths; importing this module never accesses hardware."""
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / 'data/k230'

def bootstrap(include_tools=False):
    paths = [ROOT, ROOT/'host/apps', ROOT/'host/arm', ROOT/'host/transport',
             ROOT/'host/vision', ROOT/'third_party/ftservo_python']
    if include_tools:
        paths += [ROOT/'tools/capture', ROOT/'tools/calibration', ROOT/'tools/analysis', ROOT/'scripts']
    for path in reversed(paths):
        if str(path) not in sys.path:
            sys.path.insert(0, str(path))
