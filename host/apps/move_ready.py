#!/usr/bin/env python3
"""Move to the user-defined ready pose immediately when --execute is provided.

Default invocation is offline only. --execute authorizes a ready-only dispatch
without additional stdin confirmations. No PID writes, extra motion stages, neutral return or torque release.
"""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT as REPO_ROOT, bootstrap as _bootstrap
_bootstrap(include_tools=False)
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = REPO_ROOT
from task_controller import Configuration, TaskError


def ready_task(config, profile):
    angles = profile['angles_deg']
    if not isinstance(angles, list) or len(angles) != 6:
        raise TaskError('ready pose must define six joint angles in ID order')
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in angles):
        raise TaskError('ready pose angles must be finite numbers')
    positions = [config.mapping.radians_to_count(i, math.radians(v))
                 for i, v in enumerate(angles, 1)]
    task = {'name': 'ready_pose', 'steps': [{'name': 'ready_pose', 'positions': positions}]}
    config.task(task)
    return task


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='immediately dispatch the ready goal after required read-only checks; no prompts')
    parser.add_argument('--hands-free', action='store_true',
                        help='operator confirms hands already away; existing option, no new control policy')
    parser.add_argument('--port', help='explicit arm serial port; otherwise use existing stable arm device ID')
    parser.add_argument('--start-counts', nargs=6, type=int,
                        help='offline preview start only; execution reads actual positions')
    args = parser.parse_args(argv)
    config = Configuration(ROOT)
    profile = json.loads((ROOT / 'configs/arm/ready_pose.json').read_text())
    task = ready_task(config, profile)
    command = [sys.executable, str(ROOT / 'host/apps/run_task.py'),
               '--hold-at-last-target']
    if profile.get('model_floor_check_enabled') is False:
        command.append('--ready-pose-without-model-floor')
    if profile.get('dispatch_authorization') == 'command_invocation_ready_only':
        command.append('--ready-pose-immediate')
    if args.execute:
        command.append('--execute')
    if args.hands_free:
        command.append('--hands-free')
    if args.port:
        command += ['--port', args.port]
    if args.start_counts is not None:
        command += ['--start-counts', *map(str, args.start_counts)]
    with tempfile.TemporaryDirectory(prefix='aic_ready_') as folder:
        path = Path(folder) / 'task.json'
        path.write_text(json.dumps(task, ensure_ascii=False, indent=2) + '\n')
        command += ['--task', str(path)]
        return subprocess.run(command, cwd=ROOT, check=False).returncode


if __name__ == '__main__':
    raise SystemExit(main())
