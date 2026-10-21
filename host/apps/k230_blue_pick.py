#!/usr/bin/env python3
"""Compatibility entry for the V0 PC-assisted blue-cube demo."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from demos.v0_blue_pick.run import main, drive, forward_pick_target

if __name__ == '__main__':
    raise SystemExit(main())
