"""Compatibility entry; implementation lives at repository root host/apps."""
from pathlib import Path
import runpy
import sys
root=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(root))
runpy.run_path(str(root/'host/apps'/Path(__file__).name),run_name='__main__')
