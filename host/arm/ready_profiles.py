"""Resolve the two user-defined ready poses without changing motion profiles."""
import json
from pathlib import Path


def load_ready_profile(root, pose=1):
    if type(pose) is not int or pose not in (1, 2):
        raise ValueError('Ready pose must be1 or2')
    filename = 'ready_pose.json' if pose == 1 else 'ready_pose2.json'
    return json.loads((Path(root) / 'configs/arm' / filename).read_text())
