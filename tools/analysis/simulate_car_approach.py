#!/usr/bin/env python3
"""Exercise real approach decisions with an explicitly idealized chassis model.

Distances per pulse are assumptions, not motor calibration. No UART, camera,
arm, firmware deployment or runtime motion-profile writes are performed.
"""
import argparse
import json
import math
from pathlib import Path

from device.k230.car.approach import ApproachPlanner
from device.k230.car.parking_window import evaluate
from host.car.runtime import load


def simulate(workspace, config, geometry, start_x, offset_y):
    planner = ApproachPlanner(workspace, config)
    line = workspace['centerline']['local_Base_Y_line']
    x, y = start_x, line['slope_per_X'] * start_x + line['intercept_mm'] + offset_y
    trace = []
    for step in range(240):
        now = step * 300
        z = (geometry.offset - geometry.normal[0] * x - geometry.normal[1] * y) / geometry.normal[2]
        u, _ = geometry.base_to_pixel([x, y, z + 10])
        if x > config['near_localization_start_Base_X_mm']:
            observation = {'status': 'tracking', 'timestamp_ms': now,
                'image_midline_error_px': u - 640,
                'ground_contact_proxy_Base_mm': [x, y, z]}
            plan = planner.coarse(observation, now)
        else:
            observation = {'status': 'localized', 'timestamp_ms': now,
                           'cube_center_Base_mm': [x, y, z + 10]}
            plan = planner.fine(observation, now)
        intent = plan['intent']
        trace.append({'step': step, 'Base_XY_mm': [x, y], 'intent': intent})
        if intent == 'arrived':
            return {'start': [start_x, offset_y], 'arrived': True,
                    'final_window': evaluate(x, y, workspace), 'trace': trace}
        if intent == 'forward_coarse': x -= 50
        elif intent == 'forward_fine': x -= 4
        elif intent == 'reverse_fine': x += 4
        elif intent.startswith('turn_'):
            angle = math.radians(3 if intent.endswith('coarse') else 0.5)
            if 'left' in intent: angle = -angle
            x, y = math.cos(angle) * x - math.sin(angle) * y, math.sin(angle) * x + math.cos(angle) * y
        else:
            return {'start': [start_x, offset_y], 'arrived': False, 'trace': trace}
    return {'start': [start_x, offset_y], 'arrived': False, 'trace': trace}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    workspace, config, geometry = load()
    cases = [simulate(workspace, config, geometry, x, y)
             for x in (230, 300, 500, 750, 1000) for y in (-100, 0, 100)]
    result = {'scope': 'idealized decision-loop simulation; not real stopping accuracy',
        'assumptions': {'forward_coarse_mm': 50, 'forward_fine_mm': 4,
            'turn_coarse_degrees': 3, 'turn_fine_degrees': 0.5,
            'rotation_origin': 'arm Base origin, provisional',
            'vision': 'exact generated observations; no slip, noise or latency'},
        'automatic_arm_action': False, 'cases': cases,
        'passed': sum(case['arrived'] for case in cases), 'total': len(cases)}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({k: result[k] for k in ('passed', 'total', 'scope')}))
    return 0 if result['passed'] == result['total'] else 1


if __name__ == '__main__': raise SystemExit(main())
