"""One wake sequences existing stopped visual actions to the coarse handoff."""
import io
import json
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import patch
from device.k230.car import visual_align, align_voice
from host.car.runtime import load
from tools.deploy.k230_car_align import build_bundle


class AutomaticApproachTests(unittest.TestCase):
    def setUp(self):
        _, _, self.geometry = load()
        self.config, _, _, _ = build_bundle(automatic=True)

    def observation(self, x=800, u=640, status='tracking'):
        return {'schema': 'aic.vision_observation.v1', 'mode': 'coarse',
                'frame_id': 100, 'capture_timestamp_ms': 100, 'timestamp_ms': 100,
                'clock_domain': 'k230_ticks_ms', 'image_size_px': [1280, 960],
                'status': status, 'center_pixel': [u, 500],
                'ground_contact_proxy_Base_mm': [x, 30, -60]}

    def test_turn_move_and_short_move_each_use_new_observation(self):
        near = self.config['forward']['near_Base_X_mm']
        observations = iter([self.observation(u=400), self.observation(),
                             self.observation(), self.observation(x=near+35),
                             self.observation(x=near+35), self.observation(x=near)])
        clock = [100]
        frames = []
        def observe():
            obs = next(observations)
            clock[0] += 10
            obs.update(timestamp_ms=clock[0], capture_timestamp_ms=clock[0], frame_id=clock[0])
            frames.append(clock[0])
            return obs
        commands = []
        def motion(link, config, baseline=None):
            commands.append((config['action'], config.get('angle_deg', config.get('distance_mm'))))
            clock[0] += 1800
            return {'action': config['action'], 'stop_reason': 7 if config['action'] == 'turn' else 6}
        saved = []
        with patch.object(visual_align, 'run_motion', side_effect=motion):
            result = visual_align.approach_until_near(observe, SimpleNamespace(sync=lambda: {}),
                self.geometry, self.config, lambda: clock[0], lambda *args: saved.append(args))
        self.assertEqual(commands, [('turn', 7), ('move', 100), ('move', 35)])
        self.assertEqual((result['status'], result['action_count']), ('near', 3))
        self.assertEqual([row[0]['plan']['frame_id'] for row in saved], frames[::2])
        self.assertEqual([row[2] for row in saved], [None, None, 'near'])
        self.assertTrue(all(not row[0]['automatic_arm_action'] for row in saved))
        self.assertNotIn('steps', result)  # Per-step logs are streamed, not accumulated.

    def test_initial_near_and_unconfirmed_target_do_not_query_motor(self):
        for obs, expected in ((self.observation(x=300), 'near'),
                              (self.observation(status='confirming'), 'no_target')):
            saved = []
            result = visual_align.approach_until_near(lambda: obs, None, self.geometry,
                self.config, lambda: 100, lambda *args: saved.append(args))
            self.assertEqual((result['status'], result['action_count']), (expected, 0))
            self.assertEqual(len(saved), 1)

    def test_lost_post_observation_ends_even_when_step_status_is_move_complete(self):
        observations = iter([self.observation(), self.observation(status='lost')])
        with patch.object(visual_align, 'run_motion', return_value={'action': 'move', 'stop_reason': 6}) as motion:
            result = visual_align.approach_until_near(lambda: next(observations),
                SimpleNamespace(sync=lambda: {}), self.geometry, self.config, lambda: 100, lambda *args: None)
        motion.assert_called_once()
        self.assertEqual(result['last']['status'], 'move_complete')
        self.assertEqual(result['status'], 'no_target')

    def test_existing_button_timeout_stop_does_not_restart_next_action(self):
        for action, u, reason in (('move', 640, 2), ('move', 640, 3), ('turn', 400, 4)):
            with patch.object(visual_align, 'run_motion', return_value={'action': action, 'stop_reason': reason}) as motion:
                result = visual_align.approach_until_near(lambda: self.observation(u=u),
                    SimpleNamespace(sync=lambda: {}), self.geometry, self.config, lambda: 100, lambda *args: None)
            motion.assert_called_once()
            self.assertEqual(result['status'], 'motion_stopped')

    def test_uart_expiration_between_steps_prevents_next_move(self):
        clock = [100]
        syncs = []
        def sync():
            syncs.append(True)
            if len(syncs) == 2:
                clock[0] = 601
            return {}
        observations = iter([self.observation(u=400), self.observation(), self.observation()])
        with patch.object(visual_align, 'run_motion', return_value={'action': 'turn', 'stop_reason': 7}) as motion:
            result = visual_align.approach_until_near(lambda: next(observations),
                SimpleNamespace(sync=sync), self.geometry, self.config, lambda: clock[0], lambda *args: None)
        motion.assert_called_once()
        self.assertEqual((result['status'], result['action_count']), ('no_target', 1))

    def test_camera_audio_and_logs_span_whole_voice_task(self):
        calls = []
        observations = iter([self.observation(u=400), self.observation(),
                             self.observation(), self.observation(x=300)])
        owner = self
        class EndCycle(Exception): pass
        class Camera:
            def __init__(self, cfg): pass
            def open(self): calls.append('camera_open')
            def close(self): calls.append('camera_close')
        class Link:
            def sync(self): calls.append('sync'); return {}
            def close(self): calls.append('uart_close')
        def observe(*args): calls.append('observe'); return next(observations)
        def motion(link, config, baseline=None):
            calls.append(config['action'])
            return {'action': config['action'], 'stop_reason': 7 if config['action'] == 'turn' else 6}
        def open_file(path, mode='r'):
            if mode == 'r': return io.StringIO(json.dumps(owner.config))
            calls.append('log')
            return io.StringIO()
        wake = unittest.mock.Mock(side_effect=[None, EndCycle()])
        with patch.dict(sys.modules, {
                'straight_voice': SimpleNamespace(wait_for_wake=wake), 'visual_align': visual_align,
                'ground_projection': SimpleNamespace(GroundProjection=lambda *args: owner.geometry),
                'camera': SimpleNamespace(CameraSession=Camera), 'motor_link': SimpleNamespace(MotorLink=Link)}), \
                patch.object(align_voice, 'observe', side_effect=observe), \
                patch.object(align_voice, 'time', SimpleNamespace(ticks_ms=lambda: 100)), \
                patch.object(visual_align, 'run_motion', side_effect=motion), \
                patch('builtins.open', side_effect=open_file):
            with self.assertRaises(EndCycle): align_voice.run_forever('/test')
        self.assertEqual(calls, ['camera_open', 'observe', 'sync', 'turn', 'observe', 'log',
                                 'observe', 'sync', 'move', 'observe', 'log', 'camera_close', 'uart_close'])
        self.assertEqual(wake.call_args_list[0].args[1], 32)
        self.assertEqual(wake.call_args_list[1].args[1], 0)

    def test_single_step_and_turn_only_remain_explicit_rollbacks(self):
        single, _, _, _ = build_bundle(approach_mode=True)
        turn, _, _, _ = build_bundle()
        self.assertFalse(single.get('automatic_approach', False))
        self.assertFalse(turn.get('automatic_approach', False))
        self.assertNotIn('forward', turn)
        self.assertTrue(self.config['automatic_approach'])
        self.assertEqual(self.config['forward']['max_step_mm'], 100)


if __name__ == '__main__': unittest.main()
