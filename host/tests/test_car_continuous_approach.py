"""Continuous visual driving with real planning and simulated time/UART."""
import io
import json
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from device.k230.car import align_voice, continuous_approach, visual_align
from device.k230.vision import runtime as vision_runtime
from host.car.runtime import load
from tools.deploy.k230_car_align import build_bundle


class Clock:
    def __init__(self):
        self.ms = 1000
        self.sleeps = []

    def now(self):
        return self.ms

    def sleep(self, ms):
        self.sleeps.append(ms)
        self.ms += ms


class Link:
    def __init__(self, clock, events):
        self.clock, self.events = clock, events
        self.run = False
        self.commands = []
        self.closed = False
        self.reason = 0
        self.cost_ms = 23
        self.fail_follow = False
        self.stopped_reply = False
        self.coast_once = False
        self.count = 0

    def state(self):
        return {'ms': self.clock.ms + 2000000, 'encoder_a': self.count,
                'encoder_b': -self.count, 'run': int(self.run),
                'result': 0, 'reason': self.reason}

    def sync(self):
        self.events.append(('sync', self.clock.ms))
        return self.state()

    def request(self, command):
        self.commands.append(command)
        name = command.split(',')[0]
        self.events.append((name, self.clock.ms))
        self.clock.ms += self.cost_ms
        if name in ('FOLLOW', 'SPEED'):
            self.run = True
            if name == 'FOLLOW' and self.fail_follow:
                raise RuntimeError('ACK missing after FOLLOW reached MCU')
            if self.stopped_reply:
                self.run, self.reason = False, 3
        elif name == 'STOP':
            self.run, self.reason = False, 1
        elif name == 'STATUS' and self.coast_once:
            self.count += 5
            self.coast_once = False
        return self.state()

    def expect(self, command):
        return self.request(command)

    def close(self):
        self.closed = True
        self.events.append(('uart_close', self.clock.ms))


class ContinuousApproachTests(unittest.TestCase):
    def setUp(self):
        _, _, self.geometry = load()
        self.config, _, _, _ = build_bundle(continuous=True)
        self.clock = Clock()
        self.events = []
        self.link = Link(self.clock, self.events)

    def observation(self, x=800, u=640, status='tracking', stamp=None):
        stamp = self.clock.ms if stamp is None else stamp
        normal, offset = self.geometry.normal, self.geometry.offset
        y = 30
        z = (offset - normal[0]*x - normal[1]*y) / normal[2]
        return {'schema': 'aic.vision_observation.v1', 'mode': 'coarse',
                'frame_id': stamp, 'capture_timestamp_ms': stamp,
                'timestamp_ms': stamp, 'clock_domain': 'k230_ticks_ms',
                'image_size_px': [1280, 960], 'status': status,
                'center_pixel': [u, 500],
                'ground_contact_proxy_Base_mm': [x, y, z]}

    def observer(self, settings):
        sequence = iter(settings)
        def observe():
            options = next(sequence)
            self.events.append(('observe', self.clock.ms, self.link.run))
            result = self.observation(**options)
            self.clock.ms += 37
            return result
        return observe

    def run_drive(self, observe):
        return continuous_approach.run_continuous(observe, self.link,
                    self.geometry, self.config, self.clock.now, self.clock.sleep)

    def test_follow_once_speed_updates_while_observing_without_intermediate_stop(self):
        near = self.config['forward']['near_Base_X_mm']
        result = self.run_drive(self.observer([{'u': 400}, {}, {'x': near}]))
        self.assertEqual(result['status'], 'near')
        self.assertTrue(result['motion_performed'])
        names = [command.split(',')[0] for command in self.link.commands]
        self.assertEqual(names[:3], ['FOLLOW', 'SPEED', 'STOP'])
        self.assertEqual(names.count('FOLLOW'), 1)
        self.assertEqual(names.count('STOP'), 1)
        self.assertEqual([event[2] for event in self.events if event[0] == 'observe'],
                         [False, True, True])
        left, right = map(int, self.link.commands[0].split(',')[1:])
        self.assertGreater(right, left)  # A forward left arc, not a TURN command.
        self.assertGreater(left + right, 0)
        self.assertFalse(self.link.run)
        stopped = result['stop_states']
        self.assertGreaterEqual(stopped[-1]['ms'] - stopped[0]['ms'], 200)
        self.assertTrue(all(not state['run'] for state in stopped))

    def test_cycle_period_includes_observation_and_uart_time(self):
        near = self.config['forward']['near_Base_X_mm']
        result = self.run_drive(self.observer([{}, {}, {'x': near}]))
        self.assertEqual([sample['cycle_ms'] for sample in result['samples']], [60, 60])
        self.assertEqual(self.clock.sleeps[:2], [40, 40])
        self.assertEqual([e[1] for e in self.events if e[0] == 'observe'],
                         [1000, 1100, 1200])

    def test_lost_target_stops_and_waits_for_post_coast_encoder_stability(self):
        self.link.coast_once = True
        result = self.run_drive(self.observer([{}, {'status': 'lost'}]))
        self.assertEqual(result['status'], 'no_target')
        self.assertEqual([c.split(',')[0] for c in self.link.commands[:2]],
                         ['FOLLOW', 'STOP'])
        states = result['stop_states']
        self.assertNotEqual(states[0]['encoder_a'], states[1]['encoder_a'])
        self.assertGreaterEqual(states[-1]['ms'] - states[1]['ms'], 200)
        self.assertEqual(states[-1]['encoder_a'], states[1]['encoder_a'])
        self.assertFalse(self.link.run)

    def test_non_normal_mcu_stop_never_restarts(self):
        self.link.stopped_reply = True
        result = self.run_drive(self.observer([{}]))
        self.assertEqual(result['status'], 'motion_stopped')
        self.assertEqual(result['motor_stop_reason'], 3)
        self.assertEqual(self.link.commands[0].split(',')[0], 'FOLLOW')
        self.assertTrue(all(c == 'STATUS' for c in self.link.commands[1:]))
        self.assertFalse(self.link.run)

    def test_follow_ack_exception_still_stops_pending_motion(self):
        self.link.fail_follow = True
        with self.assertRaisesRegex(RuntimeError, 'ACK missing'):
            self.run_drive(self.observer([{}]))
        self.assertEqual([c.split(',')[0] for c in self.link.commands[:2]],
                         ['FOLLOW', 'STOP'])
        self.assertTrue(all(c == 'STATUS' for c in self.link.commands[2:]))
        self.assertFalse(self.link.run)

    def test_near_initial_target_does_not_drive(self):
        result = self.run_drive(self.observer([
            {'x': self.config['forward']['near_Base_X_mm']}]))
        self.assertEqual(result['status'], 'near')
        self.assertFalse(result['motion_performed'])
        self.assertEqual(self.link.commands, [])

    def test_past_near_unaligned_initial_target_does_not_start_follow(self):
        near = self.config['forward']['near_Base_X_mm']
        for u in (400, 880):
            with self.subTest(u=u):
                self.clock = Clock()
                self.events = []
                self.link = Link(self.clock, self.events)
                result = self.run_drive(self.observer([{'x': near-20, 'u': u}]))
                self.assertEqual(result['status'], 'near_unaligned')
                self.assertFalse(result['last_plan']['centered'])
                self.assertFalse(result['motion_performed'])
                self.assertEqual(self.link.commands, [])

    def test_past_near_unaligned_while_driving_stops_instead_of_approaching_more(self):
        near = self.config['forward']['near_Base_X_mm']
        result = self.run_drive(self.observer([{}, {'x': near-20, 'u': 400}]))
        self.assertEqual(result['status'], 'near_unaligned')
        self.assertFalse(result['last_plan']['centered'])
        self.assertTrue(result['motion_performed'])
        self.assertEqual([c.split(',')[0] for c in self.link.commands[:2]],
                         ['FOLLOW', 'STOP'])
        self.assertTrue(all(c == 'STATUS' for c in self.link.commands[2:]))
        self.assertFalse(self.link.run)
        states = result['stop_states']
        self.assertGreaterEqual(states[-1]['ms'] - states[0]['ms'], 200)

    def test_unconfirmed_missing_or_expired_initial_target_does_not_drive(self):
        for options in ({'status': 'confirming'}, {'status': 'no_target'},
                        {'stamp': 1}):
            with self.subTest(options=options):
                self.clock = Clock()
                self.events = []
                self.link = Link(self.clock, self.events)
                result = self.run_drive(self.observer(
                    [options] * self.config['alignment']['observation_frames']))
                self.assertEqual(result['status'], 'no_target')
                self.assertFalse(result['motion_performed'])
                self.assertEqual(self.link.commands, [])

    def test_continuous_steering_does_not_apply_old_one_shot_turn_gain(self):
        observation = self.observation(u=400)
        speed = {'left_mm_s': 0, 'right_mm_s': 0}
        first = continuous_approach.continuous_plan(observation, self.geometry,
                                     self.config, self.clock.ms, speed)
        self.config['alignment']['turn_gain'] = 1
        second = continuous_approach.continuous_plan(observation, self.geometry,
                                      self.config, self.clock.ms, speed)
        self.assertEqual(first, second)

    def test_voice_uses_one_camera_and_persistent_tracker_then_logs_after_close(self):
        owner = self
        cameras, trackers, warmups, writes = [], [], [], []
        class EndCycle(Exception):
            pass
        def wake(voice, warmup):
            warmups.append(warmup)
            if len(warmups) > 1:
                raise EndCycle()
        class Camera:
            def __init__(self, config):
                cameras.append(self)
                self.closed = False
                self.frames = 0
            def open(self):
                owner.events.append(('camera_open', owner.clock.ms))
            def capture(self):
                self.frames += 1
                owner.events.append(('capture', owner.clock.ms, owner.link.run))
                return self.frames, {'frame_id': self.frames,
                    'capture_timestamp_ms': owner.clock.ms,
                    'clock_domain': 'k230_ticks_ms',
                    'timestamp_source': 'software_snapshot_call',
                    'image_size_px': [1280, 960]}
            def close(self):
                self.closed = True
                owner.events.append(('camera_close', owner.clock.ms))
        class Tracker:
            def __init__(self, config, projection):
                trackers.append(self)
                self.count = 0
            def detect_frame(self, image, stamp):
                self.count += 1
                result = owner.observation(stamp=stamp,
                    x=owner.config['forward']['near_Base_X_mm'] if self.count == 5 else 800,
                    status='confirming' if self.count < 3 else 'tracking')
                result['confirmations'] = self.count
                owner.clock.ms += 37
                return result
        class Log(io.StringIO):
            def write(self, value):
                owner.assertTrue(cameras[0].closed)
                owner.assertTrue(owner.link.closed)
                owner.assertFalse(owner.link.run)
                writes.append(json.loads(value))
                return super().write(value)
        def open_file(path, mode='r'):
            if mode == 'r':
                owner.assertEqual(path, '/test/config.json')
                return io.StringIO(json.dumps(owner.config))
            owner.assertEqual((path, mode), ('/sdcard/aic/car_follow/runs.jsonl', 'a'))
            return Log()
        clock_api = SimpleNamespace(ticks_ms=self.clock.now,
                    ticks_diff=lambda a, b: a-b, sleep_ms=self.clock.sleep)
        with patch.dict(sys.modules, {
                'straight_voice': SimpleNamespace(wait_for_wake=wake),
                'visual_align': visual_align,
                'ground_projection': SimpleNamespace(GroundProjection=lambda *a: self.geometry),
                'motor_link': SimpleNamespace(MotorLink=lambda: self.link),
                'camera': SimpleNamespace(CameraSession=Camera),
                'continuous_approach': continuous_approach,
                'vision_runtime': vision_runtime,
                'blue_tracker': SimpleNamespace(BlueTracker=Tracker)}), \
                patch.object(align_voice, 'time', clock_api), \
                patch.object(vision_runtime, 'time', clock_api), \
                patch.object(align_voice.os, 'exitpoint', create=True), \
                patch('builtins.open', side_effect=open_file):
            with self.assertRaises(EndCycle):
                align_voice.run_forever('/test')
        self.assertEqual(warmups, [32, 0])
        self.assertEqual((len(cameras), cameras[0].frames), (1, 5))
        self.assertEqual((len(trackers), trackers[0].count), (1, 5))
        self.assertEqual([event[2] for event in self.events if event[0] == 'capture'],
                         [False, False, False, True, True])
        self.assertEqual(len(writes), 1)
        self.assertEqual(writes[0]['status'], 'near')
        self.assertEqual(len(writes[0]['samples']), 2)
        names = [event[0] for event in self.events]
        self.assertLess(names.index('STOP'), names.index('camera_close'))
        self.assertLess(names.index('camera_close'), names.index('uart_close'))

    def test_continuous_bundle_is_explicit_and_old_modes_remain_available(self):
        turn, _, turn_remote, _ = build_bundle()
        step, _, _, _ = build_bundle(approach_mode=True)
        staged, _, _, _ = build_bundle(automatic=True)
        continuous, files, remote, startup = build_bundle(continuous=True)
        self.assertNotIn('forward', turn)
        for old in (turn, step, staged):
            self.assertNotIn('continuous', old)
        self.assertTrue(staged['automatic_approach'])
        self.assertNotIn('automatic_approach', continuous)
        self.assertEqual(continuous['continuous']['period_ms'], 100)
        self.assertNotEqual(remote, turn_remote)
        self.assertIn(remote, startup)
        self.assertIn('continuous_approach.py', files)
        self.assertIn('continuous_math.py', files)
        for name, data in files.items():
            if name.endswith('.py'):
                compile(data, name, 'exec')
        with self.assertRaises(ValueError):
            build_bundle(automatic=True, continuous=True)


if __name__ == '__main__':
    unittest.main()
