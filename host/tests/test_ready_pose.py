"""Hardware-free regressions for the user-approved ready-pose exception."""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))
from host.project_paths import ROOT as REPO_ROOT, bootstrap as _bootstrap
_bootstrap(include_tools=True)
import copy
import json
import tempfile
import contextlib
import io
from types import SimpleNamespace
from pathlib import Path
import sys
import unittest
from unittest.mock import patch, MagicMock

ROOT = REPO_ROOT
from task_controller import Configuration, TaskController, TaskError
from move_ready import ready_task
import move_ready
from host.arm.ready_profiles import load_ready_profile
import run_task
from run_task import select_route_check, dispatch_ready_immediately
from test_task_controller import Clock, FakeBus


class ReadyPoseTests(unittest.TestCase):
    def setUp(self):
        self.config = Configuration(ROOT)
        profile = json.loads((ROOT/'configs/arm/ready_pose.json').read_text())
        self.task = ready_task(self.config, profile)
        self.steps = self.config.task(self.task)[:-1]
        self.clock = Clock()
        self.bus = FakeBus(self.config, self.clock)
        # Actual logged low pose that the inherited geometry gate rejected.
        for i,count in enumerate([1936,2377,2958,2575,2093,2169],1):
            self.bus.rows[i].update(position=count, goal=count)
        self.route = select_route_check(self.config,self.task,self.steps,True,True)
        self.controller = TaskController(self.bus,self.config,self.route,
                                         clock=self.clock.now,sleep=self.clock.sleep)

    def test_logged_low_start_one_final_packet_and_torque_retained(self):
        seen=[]
        def confirm(p,reader):
            seen.append(p['stage']);reader();return True
        self.controller.prepare(self.steps)
        self.assertEqual(self.bus.commands,[])
        self.controller.run(self.steps,confirm)
        self.assertEqual(seen,['ready_pose','final_observation'])
        self.assertEqual(len(self.bus.commands),1)
        self.assertEqual(self.bus.commands[0][0],self.steps[0]['goals'])
        self.assertEqual(self.bus.commands[0][1],self.config.profiles)
        self.assertEqual(self.bus.releases,[])
        self.assertEqual(self.bus.mutations,['sync'])

    def test_operator_decline_still_sends_nothing(self):
        self.controller.prepare(self.steps)
        self.controller.run(self.steps,lambda p,r:False)
        self.assertEqual(self.bus.commands,[])

    def test_pid_mismatch_still_blocks_dispatch(self):
        self.bus.bad_parameter=True
        with self.assertRaises(TaskError):self.controller.prepare(self.steps)
        self.assertEqual(self.bus.mutations,[])

    def test_actual_device_target_limits_still_enforced(self):
        self.bus.read_limits=lambda i:(2040,3072) if i==6 else (1024,2800)
        with self.assertRaises(TaskError):self.controller.prepare(self.steps)
        self.assertEqual(self.bus.commands,[])

    def test_exception_rejects_neutral_return_or_other_task(self):
        for hold,task,steps in [(False,self.task,self.config.task(self.task)),
                                (True,{'name':'other'},self.steps)]:
            with self.assertRaises(TaskError):
                select_route_check(self.config,task,steps,hold,True)
        changed=copy.deepcopy(self.steps);changed[0]['goals'][2]+=1
        with self.assertRaises(TaskError):
            select_route_check(self.config,self.task,changed,True,True)
        with self.assertRaises(TaskError):self.route({},self.config.neutral)

    def test_other_tasks_keep_original_floor_check(self):
        with patch('run_task.FloorCheck') as floor:
            select_route_check(self.config,{'name':'other'},[],False,False)
            floor.assert_called_once_with(self.config,finger_floor_m=.03)


    def test_immediate_dispatch_keeps_one_goal_and_torque(self):
        self.controller.prepare(self.steps)
        with patch('builtins.input',side_effect=AssertionError('no stdin allowed')):
            result=dispatch_ready_immediately(self.controller,self.steps)
        self.assertEqual(result,'ready_pose_goal_sent_torque_retained')
        self.assertEqual(len(self.bus.commands),1)
        self.assertEqual(self.bus.commands[0][0],self.steps[0]['goals'])
        self.assertEqual(self.bus.releases,[])

    def test_immediate_ready_initializes_only_disabled_torque_before_final_goal(self):
        # Model torque independently of goal writes: SyncWritePosEx cannot arm
        # a disabled servo. The regular FakeBus intentionally omits that detail.
        case = self
        class TorqueAwareBus(FakeBus):
            def __init__(self, config, clock, initial_rows, disabled):
                super().__init__(config, clock)
                self.rows = copy.deepcopy(initial_rows)
                for i in disabled:
                    self.rows[i].update(torque=0, goal=2048)
                self.trace = []
                self.final_sent = False
                self.sdk = SimpleNamespace(write1ByteTxRx=self.enable)
            def enable(self, servo_id, address, value):
                case.assertEqual((address, value), (40, 1))
                case.assertEqual(self.rows[servo_id]['torque'], 0)
                case.assertEqual(self.rows[servo_id]['goal'], self.rows[servo_id]['position'])
                self.trace.append(('enable', servo_id))
                self.rows[servo_id]['torque'] = 1
                return (0, 0)
            def _written(self, result, action, servo_id):
                case.assertEqual(result, (0, 0))
            def read_control_feedback(self, servo_id):
                case.assertFalse(self.final_sent, 'Ready dispatch must not monitor after its final goal')
                return super().read_control_feedback(servo_id)
            def sync_command_positions_profiled(self, goals, profiles, require_torque=True):
                torque = {i: row['torque'] for i, row in self.rows.items()}
                if require_torque:
                    case.assertTrue(all(value == 1 for value in torque.values()))
                    self.trace.append(('ready', dict(goals)))
                    self.final_sent = True
                else:
                    case.assertEqual(goals, {i: row['position'] for i, row in self.rows.items()})
                    self.trace.append(('hold', dict(goals)))
                stamp = super().sync_command_positions_profiled(goals, profiles, require_torque)
                for i, value in torque.items():
                    self.rows[i]['torque'] = value
                return stamp

        for disabled in ((1, 2, 3, 4, 5, 6), (2, 6)):
            with self.subTest(disabled=disabled):
                bus = TorqueAwareBus(self.config, self.clock, self.bus.rows, disabled)
                controller = TaskController(bus, self.config, self.route,
                    clock=self.clock.now, sleep=self.clock.sleep)
                with patch('builtins.input', side_effect=AssertionError('No ready confirmation prompt')):
                    controller.prepare(self.steps)
                    result = dispatch_ready_immediately(controller, self.steps)
                self.assertEqual(result, 'ready_pose_goal_sent_torque_retained')
                self.assertEqual(bus.trace,
                    [('hold', {i: row['position'] for i, row in self.bus.rows.items()})]
                    + [('enable', i) for i in disabled]
                    + [('ready', self.steps[0]['goals'])])
                self.assertEqual([command[2] for command in bus.commands], [False, True])
                self.assertEqual(bus.releases, [])
                self.assertTrue(all(row['torque'] == 1 for row in bus.rows.values()))

    def test_immediate_dispatch_cannot_send_other_targets(self):
        self.controller.prepare(self.steps)
        changed=copy.deepcopy(self.steps);changed[0]['goals'][2]+=1
        with self.assertRaises(TaskError):dispatch_ready_immediately(self.controller,changed)
        self.assertEqual(self.bus.commands,[])

    def test_execute_entry_never_reads_stdin(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);(root/'configs/arm').mkdir(parents=True)
            (root/'data/run').mkdir(parents=True)
            for name in ['joint_mapping.json','user_selected_pid_demo_baseline.json',
                         'motion_defaults.json','task_control_policy.json','ready_pose.json']:
                (root/'configs/arm'/name).write_bytes((ROOT/'configs/arm'/name).read_bytes())
            task=root/'task.json';task.write_text(json.dumps(self.task))
            self.bus.port=SimpleNamespace(ser=SimpleNamespace(fileno=lambda:0))
            context=MagicMock();context.__enter__.return_value=self.bus
            argv=['run_task.py','--task',str(task),'--execute','--hold-at-last-target',
                  '--ready-pose-without-model-floor','--ready-pose-immediate']
            with patch.object(run_task,'ROOT',root),patch.object(run_task,'ServoBus',return_value=context), \
                 patch.object(run_task.fcntl,'ioctl'),patch.object(run_task.signal,'signal'), \
                 patch.object(sys,'argv',argv),patch('builtins.input',side_effect=AssertionError('unexpected prompt')), \
                 patch.object(run_task.select,'select',side_effect=AssertionError('unexpected stdin wait')), \
                 contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(run_task.main(),0)
            self.assertEqual(len(self.bus.commands),1)
            self.assertEqual(self.bus.mutations,['sync'])
            records=list((root/'data/logs').glob('task_*/record.json'))
            self.assertEqual(len(records),1)
            record=json.loads(records[0].read_text())
            self.assertEqual(record['status'],'ready_pose_goal_sent_torque_retained')
            self.assertFalse(record['stdin_confirmation_required'])
            self.assertEqual(self.bus.releases,[])

    def test_numbered_ready_poses_dispatch_only_the_selected_target(self):
        expected = {1:[2048,1081,3015,3015,2048,2332],
                    2:[2048,1138,2958,2958,2048,2332]}
        for pose in (1,2):
            with self.subTest(pose=pose):
                task = ready_task(self.config,load_ready_profile(ROOT,pose))
                steps = self.config.task(task)[:-1]
                goals = steps[0]['goals']
                self.assertEqual([goals[i] for i in range(1,7)],expected[pose])
                route = select_route_check(self.config,task,steps,True,True,
                                           ready_pose_number=pose)
                controller = TaskController(self.bus,self.config,route,
                    clock=self.clock.now,sleep=self.clock.sleep)
                controller.prepare(steps)
                before=len(self.bus.commands)
                result=dispatch_ready_immediately(controller,steps,pose)
                self.assertEqual(result,'ready_pose_goal_sent_torque_retained')
                self.assertEqual(len(self.bus.commands),before+1)
                self.assertEqual(self.bus.commands[-1][0],goals)
                other=2 if pose==1 else 1
                with self.assertRaises(TaskError):
                    select_route_check(self.config,task,steps,True,True,
                                       ready_pose_number=other)
                with self.assertRaises(TaskError):
                    dispatch_ready_immediately(controller,steps,other)
                changed=copy.deepcopy(steps);changed[0]['goals'][2]+=1
                with self.assertRaises(TaskError):
                    select_route_check(self.config,task,changed,True,True,
                                       ready_pose_number=pose)
                with self.assertRaises(TaskError):
                    dispatch_ready_immediately(controller,changed,pose)
        self.assertEqual(self.bus.releases,[])

    def test_ready_cli_passes_explicit_pose_and_matching_goals(self):
        seen=[]
        def run(command,**kwargs):
            task_path=Path(command[command.index('--task')+1])
            seen.append((list(command),json.loads(task_path.read_text())))
            return SimpleNamespace(returncode=0)
        with patch.object(move_ready.subprocess,'run',side_effect=run):
            self.assertEqual(move_ready.main([]),0)
            self.assertEqual(move_ready.main(['--pose','2']),0)
        for pose,(command,task) in enumerate(seen,1):
            self.assertEqual(command[command.index('--ready-pose-number')+1],str(pose))
            expected=ready_task(self.config,load_ready_profile(ROOT,pose))
            self.assertEqual(task,expected)
            self.assertIn('--ready-pose-immediate',command)
            self.assertNotIn('--execute',command)

    def test_unknown_ready_pose_cannot_select_an_arbitrary_profile(self):
        for pose in (0,3,True,'2'):
            with self.subTest(pose=pose),self.assertRaises(ValueError):
                load_ready_profile(ROOT,pose)

    def test_route_annotation_never_claims_geometry_passed(self):
        result=self.route({},self.steps[0]['goals'])
        self.assertEqual(result['status'],'not_checked_by_user_authorization')
        self.assertFalse(result['model_floor_check_enabled'])


if __name__=='__main__':unittest.main()
