"""Translate former car-arm script paths; all execution uses the canonical root."""
import json
from pathlib import Path
import subprocess
import sys
ROOT = Path(__file__).resolve().parents[1]
mode, *arguments = sys.argv[1:]
mapping = json.loads((ROOT/'scripts/legacy_paths.json').read_text())
translated = []
for argument in arguments:
    key = argument.removeprefix('./')
    translated.append(str(ROOT/mapping[key]) if key in mapping else argument)
raise SystemExit(subprocess.run([str(ROOT/'scripts'/mode), *translated], cwd=ROOT).returncode)
