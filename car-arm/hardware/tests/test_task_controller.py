"""Hardware-free protocol and control-state regression tests."""
import copy
import contextlib
import io
from unittest.mock import patch
import importlib.util
import json
import sys
import threading
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT/'hardware'), str(ROOT/'ros2_ws/src/arm_pkg')]
from task_controller import Configuration, TaskController, TaskError, REGISTERS, IDS
from arm_pkg.servo_bus import ServoBus, ServoError
from arm_pkg.scservo_sdk import COMM_SUCCESS


class Clock:
    def __init__(self): self.t = 0.
    def now(self): return self.t
    def sleep(self, duration): self.t += duration


class FakeBus:
    def __init__(self, config, clock, torque=1):
        self.config, self.clock = config, clock
        self.commands, self.releases, self.mutations = [], [], []
        self.rows = {i:{'id':i,'position':2048,'speed':0,'goal':2048,'torque':torque,
                        'command_speed_raw':config.profiles[i]['speed_raw'],
                        'acceleration':config.profiles[i]['acceleration_raw']} for i in IDS}
        self.bad_parameter = False
        self.bad_feedback = False
        self.bad_return = False
        self.final_drift = False
        self.overshoot = False
        self.interrupt = False
    def ping(self, i): return 777
    def read_register(self, i, a, size=1):
        if a in REGISTERS:
            assert size == 1  # 24 and 25 must remain independent byte reads.
            value=self.config.baseline['parameters'][str(i)][REGISTERS[a]]
            return value+1 if self.bad_parameter and i==2 and a==21 else value
        return {18:12,33:0,55:1}[a]
    def read_limits(self, i): return (2040,3072) if i==6 else (1024,3072)
    def read_control_feedback(self, i):
        self.clock.sleep(.0002)
        if self.commands and self.bad_feedback: raise ServoError('lost feedback')
        if self.commands and self.interrupt: raise KeyboardInterrupt()
        row=dict(self.rows[i])
        if self.commands and self.overshoot and i==2: row['position']+=13
        if len(self.commands)>=2 and self.bad_return and i==2: row['position']=2055
        return row
    def sync_command_positions_profiled(self, goals, profiles, require_torque=True):
        self.commands.append((dict(goals),copy.deepcopy(profiles),require_torque))
        self.mutations.append('sync')
        for i in IDS:
            self.rows[i].update(position=goals[i],goal=goals[i],torque=1,
                                command_speed_raw=profiles[i]['speed_raw'],acceleration=profiles[i]['acceleration_raw'])
        return int(self.clock.now()*1e9)
    def release(self, i):
        self.releases.append((i,self.clock.now()))
        self.mutations.append('release');self.rows[i]['torque']=0


class ControlTests(unittest.TestCase):
    def setUp(self):
        self.config=Configuration(ROOT);self.clock=Clock();self.bus=FakeBus(self.config,self.clock)
        self.events=[]
        self.c=TaskController(self.bus,self.config,lambda start,goal:{},self.events.append,
                              self.clock.now,self.clock.sleep)
        self.steps=self.config.task({'steps':[{'name':'test','positions':[2048,2105,2105,2105,2048,2105]}]})
    def run_task(self, confirm=lambda preview, reader: True):
        self.c.prepare(self.steps)
        return self.c.run(self.steps, confirm)
    def test_success_requires_each_stage_and_retains_torque(self):
        decisions=[]
        def confirm(preview, reader):
            decisions.append(preview['stage'])
            self.assertEqual(len(self.bus.commands),len(decisions)-1)
            reader()
            return True
        self.assertEqual(self.run_task(confirm),'operator_confirmed_torque_retained')
        self.assertEqual(decisions,['test','return_to_neutral','final_observation'])
        self.assertEqual(len(self.bus.commands),2)
        self.assertEqual(self.bus.commands[-1][0],{i:2048 for i in IDS})
        self.assertTrue(all(p==self.config.profiles for _,p,_ in self.bus.commands))
        self.assertEqual(self.bus.releases,[])
        self.assertTrue(all(r['torque']==1 for r in self.bus.rows.values()))
    def test_declined_first_stage_sends_nothing(self):
        self.run_task(lambda preview, reader: False)
        self.assertEqual(self.bus.commands,[])
        self.assertEqual(self.bus.releases,[])
    def test_declined_return_does_not_automatically_return(self):
        self.run_task(lambda preview, reader: preview['stage']=='test')
        self.assertEqual(len(self.bus.commands),1)
        self.assertEqual(self.bus.releases,[])
    def test_declined_final_observation_does_not_release(self):
        result=self.run_task(lambda preview, reader: preview['stage']!='final_observation')
        self.assertEqual(result,'operator_stopped_after_neutral_command_torque_unchanged')
        self.assertEqual(len(self.bus.commands),2)
        self.assertEqual(self.bus.releases,[])
    def test_preview_failure_still_requires_fresh_route_check(self):
        self.c.prepare(self.steps)
        calls=[]
        def route(start,goal):
            calls.append(1)
            if len(calls)==1: raise ValueError('preview pose still moving')
            return {}
        self.c.route_check=route
        self.assertEqual(self.c.run(self.steps,lambda p,r:True),'operator_confirmed_torque_retained')
        self.assertEqual(len(self.bus.commands),2)
        self.assertEqual(self.events[-1]['event'],'operator_confirmed_final_observation_torque_retained')
    def test_failed_fresh_check_after_preview_blocks_command(self):
        self.c.prepare(self.steps)
        def route(start,goal):raise ValueError('uncertified route')
        self.c.route_check=route
        with self.assertRaises(ValueError):self.c.run(self.steps,lambda p,r:True)
        self.assertEqual(self.bus.commands,[])
    def test_hold_final_observation_uses_actual_task_goal(self):
        steps=self.steps[:-1]
        seen=[]
        def confirm(p,r):
            if p['stage']=='final_observation':seen.append(p['goals'])
            return True
        self.c.prepare(steps);self.c.run(steps,confirm)
        self.assertEqual(seen,[steps[-1]['goals']])
        self.assertEqual(len(self.bus.commands),1)
        self.assertEqual(self.bus.releases,[])
    def test_no_confirmation_callback_rejected(self):
        with self.assertRaises(TaskError):self.c.run(self.steps,None)
        self.assertEqual(self.bus.commands,[])
    def test_empty_direct_run_rejected(self):
        with self.assertRaises(TaskError):self.c.run([],lambda p,r:True)
        self.assertEqual(self.bus.commands,[])
    def test_initial_hold_separate_from_final_stage_goals(self):
        for row in self.bus.rows.values():row['torque']=0
        self.run_task()
        self.assertEqual(len(self.bus.commands),3)
        self.assertFalse(self.bus.commands[0][2])
        self.assertEqual(self.bus.commands[0][0],{i:2048 for i in IDS})
        self.assertEqual(self.bus.releases,[])
    def test_one_count_gripper_feedback_below_target_limit_initializes_at_limit(self):
        self.bus.rows[6]['position']=2047
        self.c.prepare(self.steps)
        self.assertEqual(self.bus.commands,[])
        with self.assertRaises(TaskError):self.config.check_counts({i:(2047 if i==6 else 2048) for i in IDS})
    def test_gripper_feedback_beyond_initialization_tolerance_rejected(self):
        self.bus.rows[6]['position']=2045
        with self.assertRaises(TaskError):self.c.prepare(self.steps)
        self.assertEqual(self.bus.commands,[])
    def test_pid_mismatch_no_writes(self):
        self.bus.bad_parameter=True
        with self.assertRaises(TaskError):self.c.prepare(self.steps)
        self.assertEqual(self.bus.mutations,[])
    def test_invalid_late_stage_no_writes(self):
        with self.assertRaises(TaskError):self.config.task({'steps':[{'positions':[2048]*6},{'positions':[2048,999,2048,2048,2048,2048]}]})
        self.assertEqual(self.bus.mutations,[])
    def test_model_route_failure_no_packet(self):
        def fail(start,goal):raise TaskError('floor failure')
        self.c.route_check=fail
        with self.assertRaises(TaskError):self.c.prepare(self.steps)
        self.assertEqual(self.bus.mutations,[])
    def test_feedback_loss_no_extra_goal_or_release(self):
        self.bus.bad_feedback=True
        with self.assertRaises(ServoError):self.run_task()
        self.assertEqual(len(self.bus.commands),1)
        self.assertEqual(self.bus.releases,[])
    def test_interrupt_no_extra_goal_or_release(self):
        self.bus.interrupt=True
        with self.assertRaises(KeyboardInterrupt):self.run_task()
        self.assertEqual(len(self.bus.commands),1)
        self.assertEqual(self.bus.releases,[])
    def test_cancelled_thresholds_do_not_abort_or_advance(self):
        original=self.bus.read_control_feedback
        def read(i):
            self.clock.sleep(.2)
            row=original(i)
            if self.bus.commands:row.update(position=rows_position(i),speed=1600)
            return row
        def rows_position(i):return 2055 if i==2 else self.bus.rows[i]['position']
        self.bus.read_control_feedback=read
        def confirm(preview,reader):
            count=len(self.bus.commands)
            self.clock.sleep(120)
            for _ in range(3):reader()
            self.assertEqual(len(self.bus.commands),count)
            self.assertEqual(self.bus.releases,[])
            return True
        self.run_task(confirm)
        self.assertEqual(len(self.bus.commands),2)
        self.assertEqual(self.bus.releases,[])
    def test_target_error_not_automatic_settle_gate(self):
        self.bus.overshoot=True;self.bus.bad_return=True
        self.assertEqual(self.run_task(),'operator_confirmed_torque_retained')
        self.assertEqual(self.bus.releases,[])
    def test_hold_s_annotation_does_not_advance(self):
        self.steps[0]['hold_s']=60
        seen=[]
        def confirm(preview,reader):
            if preview['stage']=='test':seen.append(preview['hold_s_annotation'])
            return False
        self.run_task(confirm)
        self.assertEqual(seen,[60]);self.assertEqual(self.bus.commands,[])
    def test_invalid_feedback_remains_error(self):
        original=self.bus.read_control_feedback
        def read(i):
            row=original(i)
            if self.bus.commands and i==2:row['position']=4096
            return row
        self.bus.read_control_feedback=read
        with self.assertRaises(TaskError):self.run_task()
        self.assertEqual(len(self.bus.commands),1)
    def test_unapproved_policy_blocks_serial_open(self):
        spec=importlib.util.spec_from_file_location('run_task_cli',ROOT/'hardware/run_task.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        pending=Configuration(ROOT);pending.policy=copy.deepcopy(pending.policy);pending.policy['approved_by_user']=False
        with patch.object(module,'Configuration',return_value=pending), patch.object(module,'ServoBus',side_effect=AssertionError('must not open hardware')), patch.object(sys,'argv',['run_task.py','--neutral','--execute']), contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as exc:module.main()
        self.assertEqual(exc.exception.code,2)


class Group:
    def __init__(self):self.params={};self.sent=[];self.fail=False
    def clearParam(self):self.params.clear()
    def txPacket(self):self.sent.append(dict(self.params));return -1 if self.fail else COMM_SUCCESS


class SDK:
    def __init__(self):self.groupSyncWrite=Group();self.data=[0]*31;self.error=0;self.result=COMM_SUCCESS
    def SyncWritePosEx(self,i,pos,speed,acc):
        self.groupSyncWrite.params[i]=[acc,pos&255,pos>>8,0,0,speed&255,speed>>8];return True
    def getTxRxResult(self,result):return str(result)
    def readTxRx(self,i,a,n):assert (a,n)==(40,31);return self.data,self.result,self.error
    def scs_tohost(self,value,bit):return -(value & 0x7fff) if value & 0x8000 else value


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.b=ServoBus.__new__(ServoBus);self.b.lock=threading.RLock();self.b.sdk=SDK()
        self.b.read_limits=lambda i:(1024,3072)
        self.b.read_register=lambda i,a,size=1:0
        self.b.read_torque=lambda i:1
        self.b.read_position_speed=lambda i:(2048,0)
        self.config=Configuration(ROOT)
    def test_per_axis_wire_profiles_single_packet(self):
        goals={i:2048+i for i in IDS}
        self.b.sync_command_positions_profiled(goals,self.config.profiles)
        packets=self.b.sdk.groupSyncWrite.sent
        self.assertEqual(len(packets),1);self.assertEqual(set(packets[0]),set(IDS))
        for i,bytes_ in packets[0].items():
            self.assertEqual(bytes_[0],20 if i in (2,3,4) else 5)
            self.assertEqual(bytes_[5] | (bytes_[6]<<8),600 if i in (2,3,4) else 150)
            self.assertEqual(bytes_[1] | (bytes_[2]<<8),goals[i])
        self.assertEqual(self.b.sdk.groupSyncWrite.params,{})
    def test_invalid_last_axis_no_transmission(self):
        goals={i:2048 for i in IDS};goals[6]=4096
        with self.assertRaises(ValueError):self.b.sync_command_positions_profiled(goals,self.config.profiles)
        self.assertEqual(self.b.sdk.groupSyncWrite.sent,[])
    def test_initialization_cannot_move_unpowered_arm(self):
        goals={i:2048 for i in IDS};goals[2]=2105
        with self.assertRaises(ServoError):self.b.sync_command_positions_profiled(goals,self.config.profiles,require_torque=False)
        self.assertEqual(self.b.sdk.groupSyncWrite.sent,[])
    def test_failed_transmission_clears_buffer(self):
        self.b.sdk.groupSyncWrite.fail=True
        with self.assertRaises(ServoError):self.b.sync_command_positions_profiled({i:2048 for i in IDS},self.config.profiles)
        self.assertEqual(self.b.sdk.groupSyncWrite.params,{})
    def test_contiguous_feedback_signed_speed_and_widths(self):
        d=self.b.sdk.data;d[0]=1;d[2]=0x34;d[3]=0x08;d[16]=0;d[17]=8;d[18]=9;d[19]=0x80;d[29]=0x34;d[30]=0x12
        row=self.b.read_control_feedback(2)
        self.assertEqual((row['goal'],row['position'],row['speed'],row['current_raw']),(2100,2048,-9,0x1234))
    def test_short_or_error_packet_never_becomes_feedback(self):
        self.b.sdk.data=[0]*30
        with self.assertRaises(ServoError):self.b.read_control_feedback(2)
        self.b.sdk.data=[0]*31;self.b.sdk.error=1
        with self.assertRaises(ServoError):self.b.read_control_feedback(2)


if __name__=='__main__':unittest.main()
