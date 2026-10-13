#!/usr/bin/env python3
"""K230 extracts faces; PC solves pose/IK and reuses the existing four-stage pick."""
from k230_blue_pick import main
if __name__=='__main__':raise SystemExit(main(default_backend='k230'))
