"""Unified six-axis task controller. Configuration/validation is hardware-free."""
from __future__ import annotations

import json
import math
import time
from pathlib import Path

IDS = tuple(range(1, 7))
REGISTERS = {21: 'P', 22: 'D', 23: 'I', 24: 'startup', 25: 'integral_limit',
             26: 'deadband26', 27: 'deadband27'}


class TaskError(RuntimeError):
    pass


def six_counts(values):
    if not isinstance(values, list) or len(values) != 6:
        raise TaskError('positions must contain six integer encoder counts, ID order 1–6')
    if any(type(v) is not int or not 0 <= v <= 4095 for v in values):
        raise TaskError('each position must be an integer from 0 to 4095')
    return dict(zip(IDS, values))


class Configuration:
    def __init__(self, root):
        self.root = Path(root)
        from arm_pkg.joint_mapping import JointMapping
        self.mapping = JointMapping.load(self.root / 'hardware/joint_mapping.json')
        self.baseline = json.loads((self.root / 'hardware/user_selected_pid_demo_baseline.json').read_text())
        self.defaults = json.loads((self.root / 'hardware/motion_defaults.json').read_text())
        self.policy = json.loads((self.root / 'hardware/task_control_policy.json').read_text())
        if self.policy.get('stage_progression') != 'explicit_operator_confirmation_for_every_stage_including_neutral':
            raise TaskError('manual controller requires explicit per-stage confirmation policy')
        if self.policy.get('automatic_torque_release') or self.policy.get('automatic_stage_completion'):
            raise TaskError('automatic progression/release is not authorized in this controller')
        self.profiles = {}
        if set(self.baseline['parameters']) != {str(i) for i in IDS}:
            raise TaskError('baseline must contain all six axes')
        if self.defaults['motion'] != self.defaults['return_to_neutral']:
            raise TaskError('normal and neutral-return profiles must match')
        if self.defaults['phase18_expected'] != 12:
            raise TaskError('this controller requires phase18=12 speed units')
        for i in IDS:
            v = self.defaults['motion'][str(i)]
            speed, acc = v['speed_raw'], v['acceleration_raw']
            if type(speed) is not int or not 1 <= speed <= 1000 or type(acc) is not int or not 1 <= acc <= 100:
                raise TaskError(f'ID{i}: invalid speed/acceleration')
            b = self.baseline['parameters'][str(i)]
            if (b['speed'], b['acceleration']) != (speed, acc):
                raise TaskError(f'ID{i}: baseline and motion defaults disagree')
            for name in REGISTERS.values():
                if type(b[name]) is not int or not 0 <= b[name] <= 254:
                    raise TaskError(f'ID{i}: invalid {name}')
            self.profiles[i] = {'speed_raw': speed, 'acceleration_raw': acc}
        self.neutral = {i: self.mapping.joint(i)['zero_count'] for i in IDS}
        if any(v != 2048 for v in self.neutral.values()):
            raise TaskError('project neutral requires six goals of 2048')

    def check_counts(self, goals, limits=None):
        if set(goals) != set(IDS):
            raise TaskError('all six goals are required')
        for i, value in goals.items():
            low, high = self.mapping.joint(i)['software_count_limits']
            if type(value) is not int or not low <= value <= high:
                raise TaskError(f'ID{i}: count {value} outside software limits [{low},{high}]')
            if limits and not limits[i][0] <= value <= limits[i][1]:
                raise TaskError(f'ID{i}: count {value} outside actual device limits')

    def check_start_counts(self, values, limits):
        # Feedback may sit one or two counts beyond a software target boundary.
        # Actual device limits remain strict; this does not relax commanded limits.
        if set(values) != set(IDS):
            raise TaskError('all six feedback positions are required')
        for i,value in values.items():
            low,high=self.mapping.joint(i)['software_count_limits']
            if type(value) is not int or not low-2 <= value <= high+2 or not limits[i][0] <= value <= limits[i][1]:
                raise TaskError(f'ID{i}: initial feedback outside device limits or software boundary tolerance')

    def task(self, data):
        if not isinstance(data, dict) or set(data) - {'name', 'steps'}:
            raise TaskError('task fields: name, steps; profiles come only from saved configuration')
        raw = data.get('steps')
        if not isinstance(raw, list) or len(raw) > 100:
            raise TaskError('steps must be a list of at most 100 stages')
        steps = []
        for k, v in enumerate(raw):
            if not isinstance(v, dict) or set(v) - {'name', 'positions', 'hold_s'}:
                raise TaskError(f'stage{k}: expected name, positions, optional hold_s')
            goals = six_counts(v.get('positions'))
            self.check_counts(goals)
            hold = v.get('hold_s', 0)
            if type(hold) not in (int, float) or not math.isfinite(hold) or not 0 <= hold <= 60:
                raise TaskError('hold_s must be finite and in [0,60]')
            name = v.get('name', f'stage_{k+1}')
            if not isinstance(name, str) or not name.strip() or name == 'return_to_neutral':
                raise TaskError('stage name must be nonempty and not reserved return_to_neutral')
            steps.append({'name': name, 'goals': goals, 'hold_s': hold})
        # The controller owns the final return; never split it into individual axes.
        steps.append({'name': 'return_to_neutral', 'goals': dict(self.neutral), 'hold_s': 0})
        return steps


class FloorCheck:
    """Check independent joint progress, rather than assuming equal arrival times."""
    def __init__(self, config, finger_floor_m=.03):
        if not math.isfinite(finger_floor_m) or not .01 <= finger_floor_m <= .03:
            raise TaskError("finger model floor must be between 10 and 30 mm")
        self.finger_floor_m = finger_floor_m
        from arm_pkg.urdf_fk import URDFForwardKinematics
        from motion_floor_envelope import certify_floor
        self.config = config
        self.fk = URDFForwardKinematics(str(config.root / 'ros2_ws/src/arm_pkg/urdf/genkiarm.urdf'))
        config.mapping.validate_urdf(config.root / 'ros2_ws/src/arm_pkg/urdf/genkiarm.urdf')
        self.certify = certify_floor
        self.cache = {}

    def __call__(self, start, target):
        import numpy as np
        minimum, cells = self.certify(self.fk, self.config.mapping,
                                     np.array([start[i] for i in IDS]),
                                     np.array([target[i] for i in IDS]), self.cache,
                                     lambda: None, max_cells=4096, finger_floor_m=self.finger_floor_m)
        return {'minimum_floor_m': minimum, 'cells': cells,
                'finger_model_floor_m': self.finger_floor_m,
                'limits': 'model table only; does not certify cables, self collision or object contact'}


class TaskController:
    def __init__(self, bus, config, route_check, emit=lambda event: None,
                 clock=time.monotonic, sleep=time.sleep):
        self.bus, self.config, self.route_check = bus, config, route_check
        self.emit, self.clock, self.sleep = emit, clock, sleep
        self.limits = {}

    def sample(self):
        rows = {}
        for i in IDS:
            row = self.bus.read_control_feedback(i)
            row['t'] = self.clock()
            if (type(row['position']) is not int or not 0 <= row['position'] <= 4095
                    or row['torque'] not in (0, 1)):
                raise TaskError(f'ID{i}: invalid position/torque feedback')
            rows[i] = row
        self.emit({'event': 'feedback', 'rows': rows})
        return rows

    def verify_parameters(self):
        for i in IDS:
            if self.bus.ping(i) != 777:
                raise TaskError(f'ID{i}: model mismatch')
            for a, expected in {18: 12, 33: 0, 55: 1}.items():
                if self.bus.read_register(i, a) != expected:
                    raise TaskError(f'ID{i}: register{a} mismatch')
            for a, name in REGISTERS.items():
                expected = self.config.baseline['parameters'][str(i)][name]
                actual = self.bus.read_register(i, a, 1)
                if actual != expected:
                    raise TaskError(f'ID{i}: {name} actual={actual}, baseline={expected}; no writes performed')
            self.limits[i] = self.bus.read_limits(i)
        self.emit({'event': 'baseline_verified', 'limits': self.limits,
                   'profiles': self.config.profiles, 'pid_write': False})

    def prepare(self, steps, recover=False):
        self.verify_parameters()
        rows = self.sample()
        start = {i: rows[i]['position'] for i in IDS}
        self.config.check_start_counts(start, self.limits)
        prev = start
        for step in steps:
            self.config.check_counts(step['goals'], self.limits)
            certificate = self.route_check(prev, step['goals'])
            self.emit({'event': 'route_checked', 'stage': step['name'], 'certificate': certificate})
            prev = step['goals']
        fresh = self.sample()
        if recover and any(v['torque'] != 1 for v in fresh.values()):
            raise TaskError('independent recovery requires six axes already holding torque')
        if not recover and any(v['torque'] == 0 for v in fresh.values()):
            hold = {i:max(self.config.mapping.joint(i)['software_count_limits'][0],
                          min(self.config.mapping.joint(i)['software_count_limits'][1],fresh[i]['position'])) for i in IDS}
            # Bus initialization accepts only fresh stopped current-position goals.
            # This check prevents writing stale sampled positions during movement.
            self.send(hold, 'initialize_current_hold', require_torque=False)
            for i in IDS:
                row = self.bus.read_control_feedback(i)
                if row['goal'] != hold[i]:
                    raise TaskError(f'ID{i}: holding goal not acknowledged')
                if row['torque'] == 0:
                    self.bus._written(self.bus.sdk.write1ByteTxRx(i, 40, 1), 'enable holding torque', i)
            fresh = self.sample()
        if any(v['torque'] != 1 for v in fresh.values()):
            raise TaskError('six-axis torque enable not confirmed')
        self.emit({'event': 'torque_confirmed', 'rows':fresh,
                   'message':'六轴扭矩已开启；是否保持平稳请现场观察，若正在托扶现在撤掉托扶力。'})
        return fresh

    def send(self, goals, stage, require_torque=True):
        self.emit({'event': 'command_intent', 'stage': stage, 'goals': goals,
                   'profiles': self.config.profiles, 'packet_count': 1})
        stamp = self.bus.sync_command_positions_profiled(goals, self.config.profiles,
                                                        require_torque=require_torque)
        self.emit({'event': 'command_sent', 'stage': stage, 'sent_at_ns': stamp})

    def run(self, steps, confirm):
        """Only a positive operator decision authorizes each final-goal packet.

        confirm receives a preview and a feedback reader. It may monitor as long
        as desired but must explicitly return True. There is no timing, position,
        speed or settling rule that advances stages or releases torque.
        """
        if not steps:
            raise TaskError('at least one explicitly confirmed stage is required')
        if not callable(confirm):
            raise TaskError('every stage requires an operator confirmation callback')
        for index, step in enumerate(steps):
            rows = self.sample()
            start = {i: rows[i]['position'] for i in IDS}
            try:
                certificate = self.route_check(start, step['goals'])
            except ValueError as exc:
                # A preview may be taken while the preceding goal is still moving.
                # The mandatory fresh route check below still blocks unsafe dispatch.
                certificate = {'status':'pending_fresh_recheck', 'preview_error':str(exc)}
            preview = {'event':'awaiting_stage_confirmation', 'stage_index':index+1,
                       'stage':step['name'], 'goals':step['goals'], 'rows':rows,
                       'profiles':self.config.profiles, 'geometry':certificate,
                       'hold_s_annotation':step['hold_s'],
                       'notice':'hold_s is informational only in manual mode; no automatic timer'}
            self.emit(preview)
            if confirm(preview, self.sample) is not True:
                self.emit({'event':'operator_stopped_no_extra_commands', 'stage':step['name']})
                return 'operator_stopped_torque_unchanged'
            # A confirmation may arrive much later. Use actual feedback for the
            # route, never the preview pose, and never rewrite it as a hold goal.
            self.verify_parameters()
            fresh = self.sample()
            start = {i:fresh[i]['position'] for i in IDS}
            certificate = self.route_check(start, step['goals'])
            self.emit({'event':'live_route_checked','stage':step['name'],'certificate':certificate})
            self.config.check_counts(step['goals'], self.limits)
            self.send(step['goals'], step['name'])
            rows = self.sample()
            self.emit({'event':'stage_command_dispatched', 'stage':step['name'], 'rows':rows,
                       'notice':'Packet sent, not a claim of arrival or stability'})
        is_neutral = steps[-1]['name'] == 'return_to_neutral'
        self.emit({'event':('final_neutral_command_sent_torque_retained' if is_neutral else 'final_task_target_sent_torque_retained'),'rows':rows,
                   'notice':'Await operator observation; no arrival claim and no torque release'})
        final_preview = {'event':'awaiting_final_observation', 'stage':'final_observation',
                         'goals':dict(steps[-1]['goals']),'rows':rows,
                         'notice':'Confirm observed result or stop; neither sends another goal nor releases torque'}
        self.emit(final_preview)
        if confirm(final_preview, self.sample) is not True:
            return ('operator_stopped_after_neutral_command_torque_unchanged' if is_neutral else 'operator_stopped_after_final_task_target_torque_unchanged')
        self.emit({'event':'operator_confirmed_final_observation_torque_retained','rows':self.sample()})
        return 'operator_confirmed_torque_retained'
