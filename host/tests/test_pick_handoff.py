"""Accepted static-image handoff to arm stages; no camera, UART or hardware."""
import copy
import importlib.util
from contextlib import ExitStack, contextmanager
import math
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from host.project_paths import ROOT
from tools.deploy.k230_pick import build_bundle

STAGES = ('hover', 'descend', 'close', 'lift')
BOARD = ROOT / 'device/k230/pick'


def load_app():
    spec = importlib.util.spec_from_file_location('test_pick_handoff_app', BOARD/'app.py')
    app = importlib.util.module_from_spec(spec)
    imports = {
        'camera': SimpleNamespace(CameraSession=None),
        'vision': SimpleNamespace(localize=None),
        'vision_runtime': SimpleNamespace(localize_precise=None),
        'kinematics': SimpleNamespace(Geometry=None),
        'control': SimpleNamespace(run_pick_stage_direct=None, run_ready_direct=None),
        'voice_commands': SimpleNamespace(registered_commands=None, resolve_events=None),
    }
    with patch.dict(sys.modules, imports):
        spec.loader.exec_module(app)
    return app


def observation():
    return {
        'schema': 'aic.vision_observation.v1', 'mode': 'precise', 'status': 'localized',
        'coordinate_frame': 'Base', 'units': 'mm',
        'reference_point': 'cube_geometric_center', 'xy_compensation_applied': False,
        'cube_center_Base_mm': [245., -28., -65.],
        'frame_id': 71, 'capture_timestamp_ms': 5, 'timestamp_ms': 5,
        'clock_domain': 'k230_ticks_ms', 'image_size_px': [1280, 960],
        'quality': {'reprojection_rms_px': .43, 'vision_path': 'legacy_recovery'},
        'payload': {'status': 'complete_face_candidate', 'side_selection': 'top'},
    }


def car_source(config):
    """The car bundle uses a top-level plane and only camera/sampling in vision."""
    return copy.deepcopy({
        'intrinsics': config['intrinsics'], 'extrinsics': config['extrinsics'],
        'table_plane': config['vision']['localization']['table_plane_Base_mm'],
        'vision': {key: config['vision'][key] for key in ('camera', 'sampling')},
    })


class PickHandoffTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bundle, _, _ = build_bundle()

    def setUp(self):
        self.config = copy.deepcopy(self.bundle)
        self.source = car_source(self.config)
        self.app = load_app()
        self.observation = observation()
        self.start = [2043, 2100, 2250, 1950, 2048, 2389]
        self.plan = {name: [2100+10*i+j for j in range(6)]
                     for i, name in enumerate(STAGES)}
        self.events = []

    @contextmanager
    def harness(self, real_geometry=None, grip_delta=45):
        def snapshot(config):
            self.events.append('arm_snapshot')
            return [{'id': i+1, 'position': p} for i, p in enumerate(self.start)]

        def plan(xy, start):
            self.events.append('plan')
            return copy.deepcopy(self.plan)

        geometry = Mock()
        geometry.plan.side_effect = plan

        def construct(*args):
            self.events.append('Geometry')
            return geometry

        def stage(action, policy):
            self.events.append(policy['name'])
            if policy['name']=='close':
                from device.k230.arm.pick_stage import classify_grip
                goal=action['goals'][5]
                return classify_grip(goal,[goal+grip_delta]*5)
            return {'stage': policy['name'], 'positions': action['goals']}

        with ExitStack() as stack:
            stack.enter_context(patch.dict(sys.modules, {'control': SimpleNamespace()}))
            read = stack.enter_context(patch.object(self.app, 'arm_snapshot', side_effect=snapshot))
            constructor = stack.enter_context(patch.object(
                self.app, 'Geometry', real_geometry or Mock(side_effect=construct)))
            run_stage = stack.enter_context(patch.object(self.app, 'run_pick_stage_direct', side_effect=stage))
            log = stack.enter_context(patch.object(self.app, 'log'))
            for name in ('CameraSession', 'localize', 'localize_precise', 'perceive'):
                stack.enter_context(patch.object(self.app, name,
                    side_effect=AssertionError('Handoff must not recapture or relocalize')))
            yield SimpleNamespace(read=read, constructor=constructor,
                                  geometry=geometry, stage=run_stage, log=log)

    def call(self, **kwargs):
        return self.app.run_from_observation(self.config, self.observation,
                                             source_config=self.source, **kwargs)

    def test_live_handoff_reads_current_pose_then_dispatches_exact_four_stages(self):
        before = copy.deepcopy(self.observation)
        with self.harness() as h:
            result = self.call(execute=True)
        self.assertEqual(self.events, ['arm_snapshot', 'Geometry', 'plan']+list(STAGES))
        h.read.assert_called_once_with(self.config['control'])
        h.geometry.plan.assert_called_once_with([.245, -.028], self.start)
        plane = self.config['vision']['localization']['table_plane_Base_mm']
        n = plane['normal']
        expected_z = (plane['offset_mm']-n[0]*245-n[1]*-28)/n[2]
        h.constructor.assert_called_once_with(self.config['model'], self.config['pick'], expected_z)
        for name in STAGES:self.assertEqual(result[name], self.plan[name])
        self.assertIs(result['grasp_confirmed'],True)
        self.assertEqual(result['grip_confirmation']['delta_counts'],45)
        for name, call in zip(STAGES, h.stage.call_args_list):
            action, policy = call.args
            expected_action = dict(self.config['control'], goals=self.plan[name])
            self.assertEqual(action, expected_action)
            self.assertEqual(policy['name'], name)
            self.assertEqual(policy['policy'], self.config['pick']['standalone_stage_policy'])
        h.log.assert_any_call('handoff_localized', {
            'frame_id': 71, 'capture_timestamp_ms': 5,
            'cube_center_Base_mm': [245., -28., -65.],
            'source': 'accepted_static_observation'})
        h.log.assert_any_call('descend_position_sample',
                             {'stage': 'descend', 'positions': self.plan['descend']})
        self.assertEqual(self.observation, before)

    def test_failed_grip_confirmation_still_lifts_once_and_returns_without_retry(self):
        for delta,confirmed in ((29,False),(30,True)):
            with self.subTest(delta=delta):
                self.events=[]
                with self.harness(grip_delta=delta) as h:
                    result=self.call(execute=True)
                self.assertEqual(self.events,['arm_snapshot','Geometry','plan']+list(STAGES))
                self.assertEqual(h.stage.call_count,4)
                self.assertEqual([call.args[1]['name'] for call in h.stage.call_args_list],list(STAGES))
                self.assertIs(result['grasp_confirmed'],confirmed)
                self.assertEqual(result['grip_confirmation']['delta_counts'],delta)
                self.assertEqual(result['grip_confirmation']['goal'],self.plan['close'][5])
                h.log.assert_any_call('complete_torque_on',unittest.mock.ANY)
                final=h.log.call_args_list[-1].args
                self.assertEqual(final[0],'complete_torque_on')
                self.assertIs(final[1]['grasp_confirmed'],confirmed)

    def test_dry_plan_requires_explicit_pose_and_never_reads_or_dispatches(self):
        start = tuple(self.start)
        with self.harness() as h:
            result = self.call(execute=False, start_counts=start)
        h.read.assert_not_called()
        h.stage.assert_not_called()
        h.geometry.plan.assert_called_once_with([.245, -.028], list(start))
        self.assertEqual(result, self.plan)
        self.assertEqual(start, tuple(self.start))

    def test_dry_plan_without_valid_six_integer_counts_fails_before_arm_read(self):
        for start in (None, [], self.start[:5], self.start+[2048],
                      [2048.0]*6, [True]*6, '2048,2048,2048,2048,2048,2048'):
            with self.subTest(start=start), self.harness() as h:
                with self.assertRaisesRegex(ValueError, 'six start_counts'):
                    self.call(start_counts=start)
                h.read.assert_not_called()
                h.constructor.assert_not_called()
                h.stage.assert_not_called()

    def test_live_execution_rejects_offline_start_counts_before_arm_read(self):
        with self.harness() as h:
            with self.assertRaisesRegex(ValueError, 'current arm feedback'):
                self.call(execute=True, start_counts=self.start)
            h.read.assert_not_called()
            h.constructor.assert_not_called()
            h.stage.assert_not_called()

    def assert_rejected_before_arm_read(self):
        with self.harness() as h:
            with self.assertRaises(ValueError):
                self.call(execute=True)
            h.read.assert_not_called()
            h.constructor.assert_not_called()
            h.stage.assert_not_called()

    def test_wrong_capabilities_and_motion_derived_targets_are_rejected(self):
        changes = [
            {'mode': 'coarse'}, {'status': 'no_complete_face'},
            {'schema': 'aic.car.side_range.v1', 'status': 'range_candidate'},
            {'odometry_predicted': True}, {'range_only': True},
            {'xy_compensation_applied': True}, {'xy_compensation_applied': None},
            {'coordinate_frame': 'Camera'}, {'units': 'm'},
            {'reference_point': 'fixed_fingertip'}, {'payload': ['not a result object']},
        ]
        for change in changes:
            with self.subTest(change=change):
                self.observation = observation()
                self.observation.update(change)
                self.assert_rejected_before_arm_read()

    def test_nonfinite_or_malformed_xyz_is_rejected(self):
        for center in ([math.nan, 0, 0], [0, math.inf, 0], [0, 0, -math.inf],
                       [True, 0, 0], [245, -28], [245, '-28', -65], None):
            with self.subTest(center=center):
                self.observation = observation()
                self.observation['cube_center_Base_mm'] = center
                self.assert_rejected_before_arm_read()

    def test_missing_or_invalid_face_quality_is_rejected(self):
        for rms in (None, -1, 2.001, math.nan, math.inf, True, '.5'):
            with self.subTest(rms=rms):
                self.observation = observation()
                self.observation['quality'] = {'reprojection_rms_px': rms}
                self.assert_rejected_before_arm_read()

    def test_capture_identity_alias_and_clock_must_be_original_and_valid(self):
        for change in ({'frame_id': None}, {'capture_timestamp_ms': None},
                       {'capture_timestamp_ms': True}, {'capture_timestamp_ms': -1},
                       {'timestamp_ms': 6}, {'clock_domain': 'pc_monotonic'},
                       {'image_size_px': [640, 480]}):
            with self.subTest(change=change):
                self.observation = observation()
                self.observation.update(change)
                self.assert_rejected_before_arm_read()

    def test_missing_source_calibration_is_rejected_before_arm_read(self):
        self.source = None
        self.assert_rejected_before_arm_read()

    def test_mismatched_source_calibration_is_rejected_before_arm_read(self):
        def alter_k(s): s['intrinsics']['camera_matrix'][0][0] += 1
        def alter_d(s): s['intrinsics']['distortion_coefficients'][0] += .001
        def alter_width(s): s['intrinsics']['image_width'] //= 2
        def alter_height(s): s['intrinsics']['image_height'] //= 2
        def alter_t(s): s['extrinsics']['T_Base_from_Camera_mm'][0][3] += 1
        def alter_plane(s): s['table_plane']['offset_mm'] += 1
        def alter_normal(s): s['table_plane']['normal'][0] += .01
        def alter_camera_width(s): s['vision']['camera']['width'] //= 2
        def alter_camera_mirror(s): s['vision']['camera']['hmirror'] = True
        def alter_camera_flip(s): s['vision']['camera']['vflip'] = True
        def remove_calibration(s): del s['extrinsics']
        for alter in (alter_k, alter_d, alter_width, alter_height, alter_t,
                      alter_plane, alter_normal, alter_camera_width,
                      alter_camera_mirror, alter_camera_flip, remove_calibration):
            with self.subTest(alter=alter.__name__):
                self.source = car_source(self.config)
                alter(self.source)
                self.assert_rejected_before_arm_read()

    def test_accepted_static_solve_keeps_old_capture_timestamp_without_second_age_gate(self):
        # The task already checked post-stop capture, stationarity and actual
        # completion time. A consumer must not age from capture again or renew it.
        with self.harness() as h, patch.object(self.app.time, 'ticks_ms',
                                              return_value=100000, create=True):
            self.call(execute=True)
        h.read.assert_called_once()
        self.assertEqual(h.stage.call_count, 4)
        self.assertEqual(self.observation['capture_timestamp_ms'], 5)
        self.assertEqual(self.observation['timestamp_ms'], 5)

    def test_precise_envelope_needs_no_legacy_payload_and_accepts_existing_top_recovery(self):
        del self.observation['payload']
        self.observation['quality']['reprojection_rms_px'] = 2
        with self.harness() as h:
            self.call(start_counts=self.start)
        h.geometry.plan.assert_called_once_with([.245, -.028], self.start)

    def test_real_dry_handoff_matches_existing_pc_four_stage_inverse_kinematics(self):
        from host.apps.pick_validation_pc import make_plan, parameters
        from device.k230.pick import numeric
        spec = importlib.util.spec_from_file_location('test_handoff_real_geometry', BOARD/'kinematics.py')
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {'numeric': numeric}):
            spec.loader.exec_module(module)
        pick, pc_config = parameters()
        start = [2048]*6
        _, expected = make_plan({'cube_center_Base_mm': self.observation['cube_center_Base_mm']},
                                start, pick, pc_config)
        with self.harness(real_geometry=module.Geometry) as h:
            actual = self.call(start_counts=start)
        for name in STAGES:
            self.assertEqual(actual[name], expected[name], name)
        h.read.assert_not_called()
        h.stage.assert_not_called()


if __name__ == '__main__':
    unittest.main()
