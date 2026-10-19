#!/usr/bin/env python3
"""K230 extracts faces; PC solves pose/IK and reuses the existing four-stage pick."""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT as REPO_ROOT, bootstrap as _bootstrap
_bootstrap(include_tools=False)
from k230_blue_pick import main
if __name__=='__main__':raise SystemExit(main(default_backend='k230'))
