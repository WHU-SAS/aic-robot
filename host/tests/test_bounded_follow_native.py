"""K230 protocol/Task with the production C parser and10ms FOLLOW controller.

The camera/solver and ideal encoder plant are explicit doubles. Native blocking
advances the MCU clock independently; these tests prove endpoint authority and
handoff order, not motor dynamics, physical millimetres or field success rate.
"""
import ctypes
import json
import math
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

from device.k230.car.near_control import target_x
from device.k230.car.parking_window import bounds_mm
from device.k230.robot.tracking import TrackingTask
from host.project_paths import ROOT
from host.tests import test_fine_observation_pause as paused
from host.tests.test_fine_observation_budget import CostScene
from tools.deploy.k230_car_straight import build_bundle


FIRMWARE = ROOT / 'device/mspm0/motor_uart'


class NativeClock:
    def __init__(self, library, scale):
        self.lib, self.scale = library, scale
        self.ms, self.next_tick = 0, 10
        self.left_counts = self.right_counts = 0.
        self.a = self.b = 0
        self.on_motion = None
        self.endpoint_stops = []

    def now(self):
        return self.ms

    def advance(self, amount):
        end = self.ms + amount
        while self.next_tick <= end:
            self.ms = self.next_tick
            running = bool(self.lib.bridge_running())
            left = self.lib.bridge_reference_left() * .01 if running else 0.
            right = self.lib.bridge_reference_right() * .01 if running else 0.
            self.left_counts += left / self.scale
            self.right_counts -= right / self.scale
            a, b = round(self.left_counts), round(self.right_counts)
            da, db = a-self.a, b-self.b
            self.a, self.b = a, b
            self.lib.bridge_tick_counts(da, db)
            if self.on_motion is not None and (da or db):
                self.on_motion(da*self.scale, -db*self.scale)
            if (running and not self.lib.bridge_running() and
                    self.lib.bridge_follow_cap_completed()):
                self.endpoint_stops.append(self.ms)
            self.next_tick += 10
        self.ms = end


class NativeLink:
    """Production MotorLink framing/expect semantics over an in-memory UART."""
    def __init__(self, library, clock):
        self.lib, self.clock = library, clock
        self.commands, self.packets = [], []
        self.drop_next_ack = False
        owner = self

        class FPIOA:
            UART2_TXD, UART2_RXD = 5, 6
            def set_function(self, *args):
                pass

        class UART:
            UART2, EIGHTBITS, PARITY_NONE, STOPBITS_ONE = 2, 8, None, 1
            def __init__(self, *args, **kwargs):
                self.rx = b''
            def write(self, payload):
                command = ','.join(payload.decode().strip().rsplit(',',1)[0].split(',')[2:])
                response = b''
                for byte in payload:
                    result = owner.lib.bridge_feed(byte)
                    if result:
                        response += result
                if owner.drop_next_ack:
                    owner.drop_next_ack = False
                else:
                    self.rx += response
                owner.commands.append((clock.now(),command))
                if response:
                    owner.packets.append(dict(at=clock.now(),command=command,
                                              state=owner.module['decode'](response)))
                return len(payload)
            def read(self):
                result, self.rx = self.rx, b''
                return result
            def deinit(self):
                pass

        source = build_bundle()[1]['motor_link.py']
        self.module = {'__name__':'aic_native_motor_link'}
        with patch.dict(sys.modules, {'machine':types.SimpleNamespace(UART=UART,FPIOA=FPIOA)}):
            exec(compile(source,'native_motor_link.py','exec'),self.module)
        self.module['time'] = types.SimpleNamespace(ticks_ms=clock.now,
            ticks_us=lambda:clock.now()*1000,ticks_diff=lambda a,b:a-b,sleep_ms=clock.advance)
        self.real = self.module['MotorLink']()

    @property
    def sequence(self):
        return self.real.sequence

    @property
    def run(self):
        return int(self.lib.bridge_running())

    def advance(self):
        pass  # NativeClock already clocks every MCU tick independently.

    def sync(self):
        return self.real.sync()

    def request(self, command):
        return self.real.request(command)

    def expect(self, command):
        return self.real.expect(command)


class BoundedFollowNativeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which('cc')
        if compiler is None:
            raise unittest.SkipTest('native C compiler unavailable')
        cls.output = tempfile.TemporaryDirectory(prefix='aic-bounded-native-')
        path = Path(cls.output.name)/'bridge.so'
        result = subprocess.run([compiler,'-std=c99','-O1','-Wall','-Wextra','-Werror',
            '-shared','-fPIC','-I',str(FIRMWARE),str(FIRMWARE/'tests/bridge.c'),
            '-lm','-o',str(path)],capture_output=True,text=True,timeout=60)
        if result.returncode:
            raise RuntimeError(result.stdout+result.stderr)
        cls.lib = ctypes.CDLL(str(path))
        cls.lib.bridge_feed.argtypes = [ctypes.c_uint8]
        cls.lib.bridge_feed.restype = ctypes.c_char_p
        cls.lib.bridge_tick_counts.argtypes = [ctypes.c_int32,ctypes.c_int32]
        cls.lib.bridge_reference_left.restype = ctypes.c_double
        cls.lib.bridge_reference_right.restype = ctypes.c_double

    @classmethod
    def tearDownClass(cls):
        cls.output.cleanup()

    def link(self):
        self.lib.bridge_reset()
        scale = math.pi*65/28000
        clock = NativeClock(self.lib,scale)
        return clock,NativeLink(self.lib,clock)

    def scene(self, ratio=0):
        helper = paused.FineObservationPauseTests()
        f,unused = helper.fixture('moving',move_during_fine=False,overhead_fraction=ratio)
        old = f.task
        clock,link = self.link()
        f.clock, f.motor = clock,link
        clock.on_motion = f.apply_arc
        f.task = TrackingTask(f.capture,old.track,link,f.projection,f.config,clock.now,
            f.begin_arm,f.Fine,f.Fine,emit=lambda e,d:f.events.append((e,d)),
            reset_tracker=f.native_tracker.reset)
        scene = CostScene(self,base=f,ratio=ratio,slices=5)
        return scene

    def until(self, scene, condition, limit=5000):
        f = scene.f
        for unused in range(limit):
            f.task.step()
            self.assertIsNone(f.task.fault,f.task.snapshot())
            if condition():
                return
            f.clock.advance(f.task.idle_delay_ms())
        self.fail(dict(phase=f.task.snapshot()['phase'],x=f.x,commands=f.motor.commands[-8:]))

    def test_zero_cap_and_completed_speed_ack_do_not_start_or_restart(self):
        clock,link = self.link()
        link.sync()
        for command in ('FOLLOW,0,0,0','SPEED,0,0,0'):
            state = link.expect(command)
            self.assertEqual((state['run'],state['pwm']),(0,0))
            self.assertFalse(self.lib.bridge_follow_cap_completed())
        link.expect('FOLLOW,50,50')
        clock.advance(300)
        link.expect('SPEED,50,50,10')
        count = len(link.commands)
        clock.advance(780)
        self.assertEqual(len(link.commands),count)
        self.assertTrue(clock.endpoint_stops)
        self.assertFalse(link.run)
        state = link.expect('SPEED,50,50,20')
        self.assertEqual((state['result'],state['run'],state['reason']),(0,0,6))
        plain = link.request('SPEED,50,50')
        self.assertEqual((plain['result'],plain['run']),(3,0))

    def test_mcu_stops_during_780ms_native_and_task_requires_a_new_static_image(self):
        for ratio in (0,.1):
            with self.subTest(ratio=ratio):
                scene = self.scene(ratio)
                f = scene.f
                f.x = 480
                moved, blocked = [], []
                capture,advance = f.task.capture,f.Fine.advance

                def move_near_during_acceleration():
                    if not moved and f.clock.now() >= 300:
                        f.x = target_x(f.config['workspace'])+55
                        f.y = sum(bounds_mm(f.config['workspace'],f.x)['Base_Y_mm'])/2
                        moved.append(f.clock.now())
                    return capture()

                def native_call(session):
                    first = session.index==0 and session.ticks==0
                    f.fine_cost_ms = 780 if first else 20
                    started,packets = f.clock.now(),len(f.motor.commands)
                    result = advance(session)
                    if first:
                        blocked.append(dict(start=started,end=f.clock.now(),packets=packets,
                                            packets_after=len(f.motor.commands)))
                    f.fine_cost_ms = 20
                    return result

                f.task.capture,f.Fine.advance = move_near_during_acceleration,native_call
                self.until(scene,lambda:any(row['purpose']=='static' for row in f.fine_start_states))
                self.assertEqual(len(blocked),1)
                block = blocked[0]
                self.assertEqual(block['end']-block['start'],780)
                self.assertEqual(block['packets'],block['packets_after'])
                stop = next(at for at in f.clock.endpoint_stops
                            if block['start'] < at < block['end'])
                acknowledgement = next(row for row in f.motor.packets
                    if row['at'] >= block['end'] and row['command'].startswith('SPEED,'))
                self.assertEqual(len(acknowledgement['command'].split(',')),4)
                self.assertEqual(tuple(acknowledgement['state'][k] for k in ('result','run','reason')),
                                 (0,0,6))
                self.assertTrue(any(e=='tracking_mcu_distance_stop' for e,d in f.events))
                static_frame = next(row['frame'] for row in f.fine_start_states if row['purpose']=='static')
                static = next(row for row in scene.starts if row['frame']['frame_id']==static_frame)
                self.assertGreater(static['frame']['capture_timestamp_ms'],stop)
                moving_frame = scene.starts[0]['frame']['frame_id']
                self.assertNotIn(moving_frame,f.bind_attempts)
                self.assertFalse(f.handoffs)
                self.assertFalse(f.arms)
                self.assertFalse(any(row['state']['run'] for row in f.motor.packets
                    if row['at'] > acknowledgement['at'] and
                       row['command'].startswith(('FOLLOW,','SPEED,'))))

    def test_firmware_profile_matches_the_paired_near_configuration(self):
        config = json.loads((ROOT/'configs/car/near_parking.json').read_text())
        source = (FIRMWARE/'follow_control.h').read_text()
        for macro,key in (('FOLLOW_CAP_CRUISE_MM_S','cruise_mm_s'),
                          ('FOLLOW_CAP_FINISH_MM_S','finish_mm_s'),
                          ('FOLLOW_CAP_SLOWDOWN_GAIN_PER_S','slowdown_gain_per_s')):
            value = float(re.search(r'#define '+macro+r' ([0-9.]+)f',source).group(1))
            self.assertEqual(value,config[key])
        straight = (FIRMWARE/'straight_control.h').read_text()
        coast = float(re.search(r'#define STRAIGHT_COAST_S ([0-9.]+)f',straight).group(1))
        self.assertEqual(coast,config['coast_s'])

    def test_a_new_request_with_the_same_wheels_gets_a_real_bound_ack_before_factory(self):
        scene = self.scene()
        f = scene.f
        self.until(scene,lambda:f.task.job is not None)
        task,drive = f.task,f.task.drive
        task._cancel_job('fixture_previous_request_completed')
        image,frame = task._observe(force_status=True)
        drive._remember_coarse(task.coarse)
        plan = drive._coarse_prediction(None,f.clock.now(),False)
        command = drive._follow_command('SPEED',plan)
        task._command(command)
        old_context = task._bounded_context
        decision = {}
        drive._request('moving',task.coarse,task.state,f.clock.now(),decision,False)
        task.decision = decision
        packets = len(f.motor.commands)
        created = len(f.created_fines)
        task._start_job('moving',image,frame)
        self.assertEqual(len(f.created_fines),created+1)
        self.assertEqual(len(f.motor.commands),packets+1)
        self.assertEqual(f.motor.commands[-1][1],command)
        self.assertNotEqual(task._bounded_context,old_context)
        self.assertEqual(task._bounded_context,
                         (task.memory.version,drive._fine_request['request_id']))

    def completed_cap_scene(self):
        scene = self.scene()
        f = scene.f
        self.until(scene,lambda:f.task.job is not None)
        f.task._command('SPEED,50,50,10')
        f.clock.advance(600)
        self.assertTrue(self.lib.bridge_follow_cap_completed())
        self.assertFalse(f.motor.run)
        self.assertTrue(f.task.state['run'])  # K230 was computing, not querying.
        return scene

    def test_plain_bearing_update_race_keeps_rejected_ack_then_reads_real_status(self):
        scene = self.completed_cap_scene()
        f = scene.f
        before = len(f.motor.packets)
        state = f.task._command('SPEED,-20,20')
        packets = f.motor.packets[before:]
        self.assertEqual([row['command'] for row in packets],['SPEED,-20,20','STATUS'])
        self.assertEqual(tuple(packets[0]['state'][k] for k in ('result','run','reason')),(3,0,6))
        self.assertEqual((state['command'],state['result'],state['run'],state['reason']),
                         ('STATUS',0,0,6))
        self.assertNotEqual(state['seq'],packets[0]['state']['seq'])
        self.assertTrue(f.task.drive._fine_request['stopped_for_verification'])
        self.assertIsNone(f.task.fault)
        audit = next(d for e,d in f.events if e=='tracking_bound_update_rejected_after_completion')
        self.assertEqual(audit['rejected_reply']['result'],3)

    def test_no_ack_cannot_reuse_an_old_busy_reply_as_normal_completion(self):
        scene = self.completed_cap_scene()
        f = scene.f
        old = f.motor.request('SPEED,-20,20')
        self.assertEqual((old['result'],old['run'],old['reason']),(3,0,6))
        f.motor.drop_next_ack = True
        f.task.drive.anchor = f.task.drive._coarse_reference = None
        at = len(f.motor.commands)
        f.task.step()
        self.assertIsNotNone(f.task.fault)
        self.assertIn('No valid ACK',f.task.fault)
        self.assertFalse(any(e=='tracking_bound_update_rejected_after_completion' for e,d in f.events))
        commands = [c for t,c in f.motor.commands[at:]]
        self.assertNotIn('STATUS',commands)
        self.assertIn('STOP',commands)
        self.assertFalse(f.handoffs)


if __name__=='__main__':
    unittest.main()
