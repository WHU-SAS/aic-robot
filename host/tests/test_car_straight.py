"""Offline tests of voice isolation, coast observation and the shipped bundle."""
import json
import math
import io
import sys
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from device.k230.car import straight_voice as app
from tools.deploy.k230_car_straight import build_bundle


class StraightTests(unittest.TestCase):
    def test_wake_ignores_ready_and_releases_audio_before_motion(self):
        calls = []
        class Engine:
            events = iter([None, [('ready', 1)], [('sida', 1)]])
            def add_keyword(self, keyword): calls.append(keyword['kw_id'])
            def reset(self): calls.append('reset')
            def push(self, pcm): return next(self.events)
            def deinit(self): calls.append('engine_closed')
        class Stream:
            def volume(self, **kw): pass
            def read(self): return b'pcm'
            def stop_stream(self): calls.append('stream_stopped')
            def close(self): calls.append('stream_closed')
        class Audio:
            def open(self, **kw): return Stream()
            def terminate(self): calls.append('audio_closed')
        with patch.dict(sys.modules, {
                'wake_word': SimpleNamespace(engine=lambda sensitivity: Engine(), CHUNK=2560),
                'media.pyaudio': SimpleNamespace(PyAudio=Audio, paInt16=16)}), \
                patch('builtins.open', return_value=io.StringIO('{"kw_id":"sida"}')), \
                patch.object(app.os, 'exitpoint', lambda: None, create=True):
            app.wait_for_wake(dict(enrollment_file='keyword.json', detection_sensitivity='sensitive'), 32)
        self.assertEqual(calls, ['sida', 'reset', 'stream_stopped', 'stream_closed', 'audio_closed', 'engine_closed'])

    def test_one_move_and_final_distance_includes_coast(self):
        def state(ms, a, b, run=0, reason=6):
            return dict(ms=ms, encoder_a=a, encoder_b=b, run=run, reason=reason)
        class Link:
            calls = []
            def sync(self): return state(0, 100, -50)
            def move(self, distance):
                self.calls.append(('MOVE', distance))
                return state(0, 100, -50, 1, 0)
            responses = iter([state(100, 1100, -1050, 1, 0),
                              state(200, 2100, -2050), state(250, 2150, -2100),
                              state(300, 2160, -2110), state(350, 2160, -2110),
                              state(400, 2160, -2110), state(450, 2160, -2110),
                              state(500, 2160, -2110)])
            def expect(self, command):
                self.calls.append(command)
                return next(self.responses)
        link = Link()
        cfg = dict(distance_mm=150, allowed_error_fraction=.05, mm_per_count=.01,
                   heartbeat_interval_ms=100, stopped_observation_ms=200)
        with patch.object(app, 'time', SimpleNamespace(sleep_ms=lambda ms: None)):
            result = app.run_motion(link, cfg)
        self.assertEqual(link.calls.count(('MOVE', 150)), 1)
        self.assertEqual(link.calls[1:3], ['HB', 'HB'])
        self.assertTrue(all(x == 'STATUS' for x in link.calls[3:]))
        self.assertAlmostEqual(result['mean_nominal_mm'], 20.6)
        self.assertAlmostEqual(result['cutoff_nominal_mm'], 20)
        self.assertFalse(result['physical_distance_measured'])
        self.assertEqual(result['stop_reason'], 6)

    def test_turn_uses_both_wheels_and_does_not_issue_a_second_motion(self):
        for angle in (15, -15):
            with self.subTest(angle=angle):
                class Link:
                    calls = []
                    def sync(self):
                        return dict(ms=0, encoder_a=0, encoder_b=0)
                    def turn(self, degrees):
                        self.calls.append(('TURN', degrees))
                        return dict(ms=0, encoder_a=0, encoder_b=0, run=1, reason=0)
                    def expect(self, command):
                        self.calls.append(command)
                        mm = (1, 31) if angle > 0 else (31, 1)
                        return dict(ms=len(self.calls)*100, encoder_a=mm[0]*100,
                                    encoder_b=-mm[1]*100, run=0, reason=7)
                link = Link()
                cfg = dict(action='turn', angle_deg=angle, track_width_mm=119.2,
                           mm_per_count=.01, heartbeat_interval_ms=100, stopped_observation_ms=200)
                with patch.object(app, 'time', SimpleNamespace(sleep_ms=lambda ms: None)):
                    result = app.run_motion(link, cfg)
                self.assertEqual(link.calls, [('TURN', angle), 'HB', 'STATUS', 'STATUS'])
                self.assertAlmostEqual(result['nominal_angle_deg'], math.copysign(30 / 119.2 * 180 / math.pi, angle))
                self.assertEqual(result['stop_reason'], 7)
                self.assertNotIn('requested_mm', result)
                self.assertFalse(result['physical_angle_measured'])

    def test_encoder_wrap_and_right_sign(self):
        result = app.wheel_distances(
            dict(encoder_a=-2147483644, encoder_b=2147483643),
            dict(encoder_a=2147483644, encoder_b=-2147483645), 1)
        self.assertEqual(result, [8, 8])

    def test_car_bundle_has_no_arm_or_camera_entry(self):
        cfg, files, remote, startup = build_bundle()
        self.assertEqual(cfg['distance_mm'], 300)
        self.assertEqual(cfg['allowed_error_fraction'], .05)
        self.assertAlmostEqual(cfg['mm_per_count'], math.pi * 65 / 28000)
        self.assertEqual(set(files), {'straight_voice.py', 'motor_link.py', 'wake_word.py', 'config.json'})
        self.assertIn('straight_voice import run_forever', startup)
        self.assertNotIn('run_once', startup)
        for name, data in files.items():
            if name.endswith('.py'):
                compile(data, name, 'exec')
        self.assertNotIn('run_bench', files['motor_link.py'].decode())
        self.assertIn("keyword['kw_id']", files['straight_voice.py'].decode())

    def test_turn_bundle_has_signed_angle_and_no_distance_acceptance(self):
        for angle in (15, -15):
            cfg, files, remote, startup = build_bundle(angle)
            self.assertEqual((cfg['action'], cfg['angle_deg']), ('turn', angle))
            self.assertEqual(cfg['track_width_mm'], 119.2)
            self.assertNotIn('distance_mm', cfg)
            self.assertNotIn('allowed_error_fraction', cfg)
            self.assertIn('def turn(', files['motor_link.py'].decode())
        for angle in (0, 31, -31, 1.5):
            with self.assertRaises(ValueError): build_bundle(angle)


if __name__ == '__main__':
    unittest.main()
