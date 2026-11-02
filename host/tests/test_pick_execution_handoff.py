"""Next-version arm handoff constructs an inert session and shares no camera."""
import copy
from contextlib import ExitStack, contextmanager
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from host.project_paths import ROOT
from host.tests.test_pick_handoff import load_app, observation, car_source
from tools.deploy.k230_pick import build_bundle


class CooperativePickHandoffTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bundle, _, _ = build_bundle()

    def setUp(self):
        self.config = copy.deepcopy(self.bundle)
        self.source = car_source(self.config)
        self.observation = observation()
        self.app = load_app()
        self.start = [2043, 2100, 2250, 1950, 2048, 2389]
        self.plan = {name: [2100 + 10*i + j for j in range(6)]
                     for i, name in enumerate(('hover', 'descend', 'close', 'lift'))}
        self.events = []

    @contextmanager
    def harness(self):
        def snapshot(config):
            self.events.append('snapshot')
            return [{'id': i+1, 'position': p} for i, p in enumerate(self.start)]

        geometry = Mock()
        def plan(xy, start):
            self.events.append('plan')
            return copy.deepcopy(self.plan)
        geometry.plan.side_effect = plan
        session = Mock(name='inert_execution')
        def factory(*args, **kwargs):
            self.events.append('factory')
            return session
        with ExitStack() as stack:
            run = Mock(side_effect=factory)
            stack.enter_context(patch.dict(sys.modules, {
                'control': SimpleNamespace(open_pick_execution=run)}))
            read = stack.enter_context(patch.object(self.app, 'arm_snapshot', side_effect=snapshot))
            construct = stack.enter_context(patch.object(self.app, 'Geometry', return_value=geometry))
            log = stack.enter_context(patch.object(self.app, 'log'))
            forbidden = {}
            for name in ('CameraSession', 'localize', 'localize_precise', 'perceive',
                         'run_pick_stage_direct', 'run_ready_direct', 'run_ready'):
                forbidden[name] = stack.enter_context(patch.object(self.app, name,
                    side_effect=AssertionError('Construction may not own camera or send motion')))
            yield SimpleNamespace(factory=run, session=session, read=read,
                geometry=geometry, construct=construct, log=log, forbidden=forbidden)

    def begin(self, **kwargs):
        return self.app.begin_from_observation(self.config, self.observation, 17,
                                               source_config=self.source, **kwargs)

    def test_begin_plans_once_and_returns_inert_execution_without_motion_or_camera(self):
        original = copy.deepcopy(self.observation)
        with self.harness() as h:
            result = self.begin()
        self.assertIs(result, h.session)
        self.assertEqual(self.events, ['snapshot', 'plan', 'factory'])
        h.read.assert_called_once_with(self.config['control'])
        h.geometry.plan.assert_called_once_with([.245, -.028], self.start)
        h.session.step.assert_not_called()
        h.session.close.assert_not_called()
        for callback in h.forbidden.values():callback.assert_not_called()
        args = h.factory.call_args.args
        self.assertIs(args[0], self.config['control'])
        self.assertIs(args[1], self.config['drive_ready_control'])
        self.assertEqual(args[2], self.plan)
        self.assertEqual(args[3], dict(self.config['pick']['standalone_stage_policy'],
            **self.config['pick'].get('tracking_stage_policy',{})))
        self.assertEqual(args[4], 17)
        self.assertEqual(self.observation, original)
        h.log.assert_any_call('tracking_handoff_localized', {
            'frame_id': 71, 'capture_timestamp_ms': 5,
            'cube_center_Base_mm': [245., -28., -65.], 'target_version': 17,
            'source': 'accepted_static_observation'})

    def test_tracking_policy_overrides_are_copied_and_do_not_mutate_legacy_policy(self):
        original = copy.deepcopy(self.config['pick']['standalone_stage_policy'])
        overrides = {'lift_tolerance_counts': 10, 'reobserve_delay_ms': 500,
                     'poll_interval_ms': 60}
        self.config['pick']['tracking_stage_policy'] = dict(overrides)
        with self.harness() as h:
            self.begin()
        passed = h.factory.call_args.args[3]
        self.assertEqual(passed, dict(original, **overrides))
        self.assertIsNot(passed, self.config['pick']['standalone_stage_policy'])
        self.assertIsNot(passed, self.config['pick']['tracking_stage_policy'])
        self.assertEqual(self.config['pick']['standalone_stage_policy'], original)
        self.assertEqual(self.config['pick']['tracking_stage_policy'], overrides)

    def test_default_events_use_existing_pick_log_and_external_emitter_is_preserved(self):
        with self.harness() as h:
            self.begin()
            h.factory.call_args.kwargs['emit']('grip_confirmation', {'delta_counts': 30})
            h.log.assert_any_call('grip_confirmation', {'delta_counts': 30})
        emit = Mock()
        with self.harness() as h:
            self.begin(emit=emit)
            self.assertIs(h.factory.call_args.kwargs['emit'], emit)

    def test_wrong_handoff_or_calibration_fails_before_read_and_factory(self):
        for corrupt in ('source', 'range', 'image', 'quality'):
            with self.subTest(corrupt=corrupt):
                self.source = car_source(self.config)
                self.observation = observation()
                if corrupt == 'source':
                    self.source['intrinsics']['camera_matrix'][0][0] += 1
                elif corrupt == 'range':self.observation['range_only'] = True
                elif corrupt == 'image':self.observation['image_size_px'] = [640, 480]
                else:self.observation['quality']['reprojection_rms_px'] = 2.01
                with self.harness() as h:
                    with self.assertRaises(ValueError):self.begin()
                    h.read.assert_not_called()
                    h.construct.assert_not_called()
                    h.factory.assert_not_called()

    def test_planning_failure_never_constructs_execution_or_dispatches(self):
        with self.harness() as h:
            h.geometry.plan.side_effect = ValueError('No complete plan')
            with self.assertRaisesRegex(ValueError, 'No complete plan'):self.begin()
            h.read.assert_called_once()
            h.factory.assert_not_called()
            for callback in h.forbidden.values():callback.assert_not_called()

    def test_missing_next_version_transport_fails_before_position_read(self):
        with self.harness() as h, patch.dict(sys.modules, {'control': SimpleNamespace()}):
            with self.assertRaises(AttributeError):self.begin()
            h.read.assert_not_called()
            h.factory.assert_not_called()

    def test_missing_ready_pose2_fails_before_position_read(self):
        del self.config['drive_ready_control']
        with self.harness() as h:
            with self.assertRaises(KeyError):self.begin()
            h.read.assert_not_called()
            h.factory.assert_not_called()

    def test_original_capture_identity_is_not_renewed_by_new_execution(self):
        with self.harness() as h, patch.object(self.app.time, 'ticks_ms',
                                               return_value=100000, create=True):
            self.begin()
        self.assertEqual(self.observation['capture_timestamp_ms'], 5)
        self.assertEqual(self.observation['timestamp_ms'], 5)
        h.factory.assert_called_once()

    def test_bundle_carries_engine_without_changing_legacy_startup(self):
        config, files, manifest = build_bundle()
        control = files['control.py'].decode()
        self.assertIn('def open_pick_execution(', control)
        self.assertIn('class PickExecution', control)
        self.assertIn('def begin_from_observation(', files['app.py'].decode())
        self.assertIn("run_once(config,execute=True)", files['app.py'].decode())
        self.assertFalse(manifest['startup_motion'])
        self.assertEqual(config['drive_ready_control']['ready_pose_number'], 2)


if __name__ == '__main__':unittest.main()
