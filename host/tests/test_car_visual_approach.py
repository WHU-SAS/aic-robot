"""One fresh visual observation dispatches one TURN or one bounded MOVE."""
import unittest
from types import SimpleNamespace
from unittest.mock import patch
from device.k230.car import visual_align, straight_voice
from device.k230.car.continuous_math import forward_x_per_mm
from host.car.runtime import load
from tools.deploy.k230_car_align import build_bundle


class VisualApproachTests(unittest.TestCase):
    def setUp(self):
        _, _, self.geometry = load()
        self.config, _, _, _ = build_bundle(approach_mode=True)

    def observation(self, x=800, u=640, stamp=100):
        return {'schema': 'aic.vision_observation.v1', 'mode': 'coarse',
                'frame_id': stamp, 'capture_timestamp_ms': stamp, 'timestamp_ms': stamp,
                'clock_domain': 'k230_ticks_ms', 'image_size_px': [1280, 960],
                'status': 'tracking', 'center_pixel': [u, 500],
                'ground_contact_proxy_Base_mm': [x, 30, -60]}

    def test_off_center_turns_instead_of_moving(self):
        for u, sign in ((400, 1), (880, -1)):
            plan = visual_align.motion_plan(self.observation(u=u), self.geometry, self.config, 100)
            self.assertEqual(plan['status'], 'turn')
            self.assertGreater(plan['angle_deg'] * sign, 0)
            self.assertEqual(plan['turn_gain'], .75)
            self.assertNotIn('distance_mm', plan)

    def test_forward_steps_shrink_at_existing_near_handoff(self):
        near = self.config['forward']['near_Base_X_mm']
        scale = forward_x_per_mm(self.geometry)
        for gap, expected in ((1000, 100), (100.1, 100), (50.9, 50), (10.1, 10)):
            plan = visual_align.motion_plan(self.observation(x=near+gap*scale), self.geometry, self.config, 100)
            self.assertEqual((plan['status'], plan['distance_mm']), ('move', expected))
        for gap in (9.9, 0, -30):
            plan = visual_align.motion_plan(self.observation(x=near+gap*scale), self.geometry, self.config, 100)
            self.assertEqual((plan['status'], plan['distance_mm']), ('near', 0))
            self.assertTrue(plan['precise_localization_required'])

    def test_missing_and_stale_target_never_forward(self):
        obs = self.observation(); obs['status'] = 'no_target'
        self.assertEqual(visual_align.motion_plan(obs, self.geometry, self.config, 100)['status'], 'no_target')
        self.assertEqual(visual_align.motion_plan(self.observation(), self.geometry, self.config, 601)['status'], 'no_target')
        with self.assertRaises(ValueError):
            visual_align.motion_plan(self.observation(x=float('nan')), self.geometry, self.config, 100)

    def test_move_has_one_command_heartbeat_settle_and_new_observation(self):
        calls = []
        class Link:
            def sync(self):
                calls.append('sync'); return dict(ms=0, encoder_a=0, encoder_b=0)
            def move(self, mm):
                calls.append(('MOVE', mm))
                return dict(ms=0, encoder_a=0, encoder_b=0, run=1, reason=0)
            def turn(self, angle): raise AssertionError('No turn after centered observation')
            def expect(self, command):
                calls.append(command)
                return dict(ms=(len(calls)-2)*100, encoder_a=13000, encoder_b=-13000, run=0, reason=6)
        observations = iter([self.observation(), self.observation(x=700)])
        def observe(): calls.append('observe'); return next(observations)
        with patch.object(straight_voice, 'time', SimpleNamespace(sleep_ms=lambda ms: None)):
            result = visual_align.align_once(observe, Link(), self.geometry, self.config, lambda: 100)
        self.assertEqual(calls, ['observe', 'sync', ('MOVE', 100), 'HB', 'STATUS', 'STATUS', 'observe'])
        self.assertEqual(result['status'], 'move_complete')
        self.assertEqual(result['motion']['action'], 'move')
        self.assertEqual(result['motion']['stop_reason'], 6)
        self.assertEqual(result['after_plan']['status'], 'move')
        self.assertFalse(result['automatic_arm_action'])

    def test_turn_does_not_automatically_follow_with_forward(self):
        observations = iter([self.observation(u=400), self.observation()])
        link = SimpleNamespace(sync=lambda: {})
        with patch.object(visual_align, 'run_motion', return_value={'stop_reason': 7}) as motion:
            result = visual_align.align_once(lambda: next(observations), link, self.geometry, self.config, lambda: 100)
        motion.assert_called_once()
        self.assertEqual(motion.call_args.args[1]['action'], 'turn')
        self.assertEqual(result['after_plan']['status'], 'move')
        self.assertEqual(result['status'], 'aligned')

    def test_no_move_after_uart_expires_frame_or_near_handoff(self):
        clock = [100]
        def sync(): clock[0] = 601; return {}
        with patch.object(visual_align, 'run_motion') as motion:
            result = visual_align.align_once(self.observation, SimpleNamespace(sync=sync), self.geometry, self.config, lambda: clock[0])
        self.assertFalse(result['motion_performed']); motion.assert_not_called()
        result = visual_align.align_once(lambda: self.observation(x=300), None, self.geometry, self.config, lambda: 100)
        self.assertEqual(result['status'], 'near'); self.assertFalse(result['motion_performed'])

    def test_old_turn_only_bundle_is_still_available(self):
        old, _, _, _ = build_bundle()
        self.assertNotIn('forward', old)
        self.assertEqual(self.config['forward']['max_step_mm'], 100)
        self.assertEqual(self.config['allowed_error_fraction'], .05)


if __name__ == '__main__': unittest.main()
