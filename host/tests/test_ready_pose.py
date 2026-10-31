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

    def test_route_annotation_never_claims_geometry_passed(self):
        result=self.route({},self.steps[0]['goals'])
        self.assertEqual(result['status'],'not_checked_by_user_authorization')
        self.assertFalse(result['model_floor_check_enabled'])


if __name__=='__main__':unittest.main()
