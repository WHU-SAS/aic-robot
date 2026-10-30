"""Calibrated direction, stale-frame handling and one-turn integration."""
import math
import io
import json
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from device.k230.car import visual_align
from host.car.runtime import load
from tools.deploy.k230_car_align import build_bundle


class VisualAlignTests(unittest.TestCase):
    def setUp(self):
        _, _, self.geometry = load()
        self.config, self.files, self.remote, self.startup = build_bundle()

    def observation(self, u=400, v=500, stamp=100):
        return {'schema': 'aic.vision_observation.v1', 'mode': 'coarse',
                'frame_id': stamp, 'capture_timestamp_ms': stamp, 'timestamp_ms': stamp,
                'clock_domain': 'k230_ticks_ms', 'image_size_px': [1280, 960],
                'status': 'tracking', 'center_pixel': [u, v],
                'image_midline_error_px': 999}  # Must calculate from the center.

    def test_direction_midline_and_integer_amplitude(self):
        for row in (300, 500, 750):
            for u, sign in ((400, 1), (880, -1)):
                plan = visual_align.alignment_plan(self.observation(u, row), self.geometry, self.config, 100)
                self.assertEqual(plan['status'], 'turn')
                self.assertGreater(plan['angle_deg'] * sign, 0)
                self.assertAlmostEqual(plan['bearing_error_deg'] * self.config['alignment']['turn_gain'], plan['angle_deg'], delta=.5)
            for u in (592, 640, 688):
                plan = visual_align.alignment_plan(self.observation(u, row), self.geometry, self.config, 100)
                self.assertEqual((plan['status'], plan['angle_deg']), ('aligned', 0))
            self.assertGreater(self.geometry.K[0][2], 640)  # Principal point differs.
            self.assertAlmostEqual(visual_align.alignment_plan(self.observation(640, row), self.geometry, self.config, 100)['bearing_error_deg'], 0)
        self.assertEqual(visual_align.alignment_plan(self.observation(10), self.geometry, self.config, 100)['angle_deg'], 15)

    def test_yaw_geometry_reduces_image_error_with_real_calibration(self):
        from device.k230.pick.pose import undistort, project
        for u in (400, 880):
            v = 500
            angle = visual_align.alignment_plan(self.observation(u, v), self.geometry, self.config, 100)['bearing_error_deg']
            ray = undistort([u, v], self.geometry.K, self.geometry.D) + [1.]
            T = self.geometry.T
            direction = [sum(row[j]*ray[j] for j in range(3)) for row in T[:3]]
            n = self.geometry.normal
            cross = [n[1]*direction[2]-n[2]*direction[1], n[2]*direction[0]-n[0]*direction[2], n[0]*direction[1]-n[1]*direction[0]]
            c, s = math.cos(math.radians(angle)), math.sin(math.radians(angle))
            rotated = [c*direction[k]-s*cross[k]+(1-c)*sum(a*b for a,b in zip(n,direction))*n[k] for k in range(3)]
            camera = [sum(T[k][j]*rotated[k] for k in range(3)) for j in range(3)]
            self.assertLess(abs(project(camera, self.geometry.K, self.geometry.D)[0]-640), 3)

    def test_no_unconfirmed_or_expired_geometry_drives(self):
        for status in ('confirming', 'lost', 'no_target'):
            obs = self.observation(); obs['status'] = status
            self.assertEqual(visual_align.alignment_plan(obs, self.geometry, self.config, 100)['angle_deg'], 0)
        for now in (99, 601):
            self.assertEqual(visual_align.alignment_plan(self.observation(), self.geometry, self.config, now)['angle_deg'], 0)

    def test_uart_delay_expires_observation_before_any_turn(self):
        clock = [100]
        class Link:
            def sync(self): clock[0] = 601; return {}
        with patch.object(visual_align, 'run_motion') as motion:
            result = visual_align.align_once(self.observation, Link(), self.geometry, self.config, lambda: clock[0])
        self.assertFalse(result['motion_performed']); motion.assert_not_called()

    def test_centered_or_missing_target_never_queries_or_drives_motors(self):
        for status in ('tracking', 'no_target'):
            obs = self.observation(640); obs['status'] = status
            with patch.object(visual_align, 'run_motion') as motion:
                result = visual_align.align_once(lambda: obs, None, self.geometry, self.config, lambda: 100)
            self.assertFalse(result['motion_performed']); motion.assert_not_called()

    def test_one_turn_then_reobserve_even_when_target_still_off_center(self):
        calls = []
        baseline = {'ms': 10}
        class Link:
            def sync(self): calls.append('sync'); return baseline
        def observe(): calls.append('observe'); return self.observation()
        def motion(link, config, baseline=None):
            calls.append(('TURN', config['angle_deg']))
            self.assertIs(baseline, baseline_expected)
            self.assertEqual(config['action'], 'turn')
            return {'nominal_angle_deg': 6.9, 'stop_reason': 7}
        baseline_expected = baseline
        with patch.object(visual_align, 'run_motion', side_effect=motion):
            result = visual_align.align_once(observe, Link(), self.geometry, self.config, lambda: 100)
        self.assertEqual(calls, ['observe', 'sync', ('TURN', 7), 'observe'])
        self.assertEqual(result['status'], 'turn_complete')
        self.assertFalse(result['automatic_arm_action'])

    def test_bundle_owns_no_arm_and_does_not_start_at_boot(self):
        for name, data in self.files.items():
            if name.endswith('.py'): compile(data, name, 'exec')
        self.assertNotIn('app.py', self.files)
        self.assertNotIn('control.py', self.files)
        self.assertIn('from align_voice import run_forever', self.startup)
        self.assertEqual(self.config['vision']['camera']['width'], 1280)
        self.assertEqual(self.config['tracking']['confirmation_frames'], 3)
        self.assertNotIn('distance_mm', self.config)

    def test_slow_camera_cleanup_happens_after_motion_and_post_observation(self):
        from device.k230.car import align_voice
        calls = []; clock = [100]; owner = self
        class EndCycle(Exception): pass
        class Camera:
            def __init__(self, config): pass
            def open(self): calls.append('camera_open')
            def close(self): calls.append('camera_close'); clock[0] += 700
        class Link:
            def sync(self): calls.append('sync'); return {}
            def close(self): calls.append('uart_close')
        def observe(*args):
            calls.append('observe')
            return owner.observation(stamp=clock[0])
        def motion(*args, **kw):
            calls.append('turn'); return {'nominal_angle_deg': 9, 'stop_reason': 7}
        def open_file(path, mode='r'):
            return io.StringIO(json.dumps(owner.config) if mode == 'r' else '')
        with patch.dict(sys.modules, {
                'straight_voice': SimpleNamespace(wait_for_wake=unittest.mock.Mock(side_effect=[None, EndCycle()])),
                'visual_align': visual_align,
                'ground_projection': SimpleNamespace(GroundProjection=lambda *a: owner.geometry),
                'camera': SimpleNamespace(CameraSession=Camera), 'motor_link': SimpleNamespace(MotorLink=Link)}), \
                patch.object(align_voice, 'observe', side_effect=observe), \
                patch.object(align_voice, 'time', SimpleNamespace(ticks_ms=lambda: clock[0])), \
                patch.object(visual_align, 'run_motion', side_effect=motion), \
                patch('builtins.open', side_effect=open_file):
            with self.assertRaises(EndCycle): align_voice.run_forever('/test')
        self.assertEqual(calls, ['camera_open', 'observe', 'sync', 'turn',
                                 'observe', 'camera_close', 'uart_close'])


if __name__ == '__main__': unittest.main()
