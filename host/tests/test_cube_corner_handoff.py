"""Five-corner handoff is a separate capability; mock IO only."""
import copy
import math
import unittest

from host.tests import test_pick_handoff as legacy
from tools.deploy.k230_pick import build_bundle

EVIDENCE = 'nonplanar_cube_five_corners'


def cube_observation(rms=2.7):
    observation = legacy.observation()
    objects = [[-10., 0., 20.], [-10., 20., 20.], [10., 20., 20.],
               [10., 0., 0.], [-10., 0., 0.]]
    corners = [[-10., 0., 20.], [10., 0., 20.], [10., 0., 0.], [-10., 0., 0.],
               [-10., 20., 20.], [10., 20., 20.], [10., 20., 0.], [-10., 20., 0.]]
    def pixel(point):
        return [640.+1000.*point[0]/(point[2]+200.),
                480.+1000.*point[1]/(point[2]+200.)]
    pose = {'R_Object_to_Camera': [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]],
            't_Object_to_Camera_mm': [0., 0., 200.], 'object_center_mm': [0., 10., 10.],
            'edge_length_mm': 20., 'object_points_mm': objects,
            'image_points_px': [pixel(p) for p in objects],
            'corner_reprojection_rms_px': rms, 'cube_center_Camera_mm': [0., 10., 210.],
            'cube_projected_corners_px': [pixel(p) for p in corners],
            'cube_projection_source': 'pose_transform'}
    observation['quality'] = {'evidence_type': EVIDENCE, 'reprojection_rms_px': rms,
        'reprojection_limit_px': 3, 'boundary_method': 'six_measured_edges_five_corners',
        'vision_path': 'shared_nonplanar_cube_corners'}
    observation['payload'] = {'status': 'complete_cube_candidate',
        'evidence_type': EVIDENCE, 'cube_pose': pose,
        'boundary_method': 'six_measured_edges_five_corners',
        'vision_path': 'shared_nonplanar_cube_corners',
        'cube_center_Base_mm': list(observation['cube_center_Base_mm']),
        'observed_edges': [{'edge_id': i, 'support': 64} for i in range(6)]}
    return observation


class CubeCornerHandoffTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bundle, _, _ = build_bundle()

    def setUp(self):
        self.app = legacy.load_app()

    def reject_without_arm_io(self, observation):
        case = legacy.PickHandoffTests()
        case.bundle = self.bundle
        case.setUp()
        case.observation = observation
        with case.harness() as h:
            with self.assertRaises(ValueError):
                case.call(execute=True)
            h.read.assert_not_called()
            h.constructor.assert_not_called()
            h.stage.assert_not_called()

    def test_new_capability_accepts_its_own_3px_boundary_without_inventing_a_face(self):
        for rms in (0., 2.01, 2.7, 3.):
            with self.subTest(rms=rms):
                observation = cube_observation(rms)
                before = copy.deepcopy(observation)
                found = self.app._handoff_target(observation)
                self.assertEqual(observation, before)
                self.assertEqual(found['cube_pose'], observation['payload']['cube_pose'])
                self.assertIs(found['observation'], observation)
                self.assertEqual(found['cube_center_Base_mm'], observation['cube_center_Base_mm'])
                self.assertIsNot(found['cube_center_Base_mm'], observation['cube_center_Base_mm'])
                self.assertNotIn('poses', found)
                self.assertNotIn('side_faces', found)
                self.assertNotIn('face_reprojection_rms_px', found['cube_pose'])
                self.assertEqual(found['observation']['capture_timestamp_ms'], 5)

    def test_shared_runtime_normalized_cube_passes_the_consumer_unchanged(self):
        import json
        from device.k230.vision.runtime import localize_precise
        from device.k230.pick.pose import project
        original = cube_observation()
        raw = original['payload']
        pose = raw['cube_pose']
        pose['R_Object_to_Camera'] = [[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]]
        pose['t_Object_to_Camera_mm'] = [0., 30., 220.]
        K = [[1000., 0., 640.], [0., 1000., 480.], [0., 0., 1.]]
        def pixel(point):
            return project([point[0], 30.-point[2], 220.+point[1]], K, [])
        pose['image_points_px'] = [pixel(p) for p in pose['object_points_mm']]
        raw['observed_edges'] = [{'quality': {'sampled_stations': 64, 'support': 64,
                                                'rms_px': .5}} for _ in range(6)]
        raw['half_corner_errors_px'] = [0.]*10
        intr = {'camera_matrix': K, 'distortion_coefficients': [],
                'image_width': 1280, 'image_height': 960}
        ext = {'T_Base_from_Camera_mm': [[1., 0., 0., 0.], [0., 0., 1., 0.],
                                       [0., -1., 0., 30.], [0., 0., 0., 1.]]}
        plane = {'normal': [0., 0., 1.], 'offset_mm': 0.}
        frame = {key: original[key] for key in ('frame_id', 'capture_timestamp_ms',
                                                'clock_domain', 'image_size_px')}
        result = localize_precise(None, frame, intr, ext, plane,
                                  localizer=lambda *args: raw)
        self.assertEqual(result['status'], 'localized', result)
        self.assertEqual(result['cube_center_Base_mm'], [0., 230., 10.])
        self.assertLess(result['quality']['reprojection_rms_px'], 1e-9)
        for observation in (result, json.loads(json.dumps(result))):
            found = self.app._handoff_target(observation)
            self.assertEqual(found['cube_pose'], observation['payload']['cube_pose'])
            self.assertEqual(found['cube_center_Base_mm'], [0., 230., 10.])

    def test_old_face_limit_stays_2px_and_does_not_gain_cube_authority(self):
        for rms in (0., 2.):
            observation = legacy.observation()
            observation['quality']['reprojection_rms_px'] = rms
            self.assertEqual(self.app._handoff_target(observation)['status'], 'complete_face_candidate')
        observation = legacy.observation()
        observation['quality']['reprojection_rms_px'] = 2.001
        self.reject_without_arm_io(observation)

    def test_valid_cube_handoff_preserves_the_four_motion_stages_and_no_recapture(self):
        case = legacy.PickHandoffTests()
        case.bundle = self.bundle
        case.setUp()
        case.observation = cube_observation()
        with case.harness() as h:
            result = case.call(execute=True)
        self.assertEqual([v for v in case.events if v in legacy.STAGES], list(legacy.STAGES))
        h.read.assert_not_called()
        h.geometry.plan.assert_called_once_with([.245, -.028], case.ready)
        self.assertEqual(result['hover'], case.plan['hover'])

    def test_inconsistent_markers_cannot_fall_through_to_the_face_policy(self):
        removals = [('payload', 'status'), ('payload', 'evidence_type'),
                    ('payload', 'cube_pose'), ('quality', 'evidence_type'),
                    ('quality', 'reprojection_limit_px'), ('quality', 'boundary_method'),
                    ('quality', 'vision_path')]
        for section, key in removals:
            with self.subTest(section=section, key=key):
                observation = cube_observation(.5)
                del observation[section][key]
                self.reject_without_arm_io(observation)
        for section, key, value in [('payload', 'status', 'complete_face_candidate'),
                ('payload', 'evidence_type', 'unknown'), ('quality', 'evidence_type', 'unknown'),
                ('quality', 'reprojection_limit_px', 2), ('quality', 'reprojection_limit_px', True),
                ('quality', 'boundary_method', 'four_measured_edges'),
                ('quality', 'vision_path', 'legacy_recovery')]:
            with self.subTest(section=section, key=key, value=value):
                observation = cube_observation(.5)
                observation[section][key] = value
                self.reject_without_arm_io(observation)
        # One new field is enough to require the complete new contract.
        observation = legacy.observation()
        observation['payload']['cube_pose'] = {}
        self.reject_without_arm_io(observation)
        for key, value in [('boundary_method', 'six_measured_edges_five_corners'),
                           ('vision_path', 'shared_nonplanar_cube_corners')]:
            with self.subTest(partial_quality_marker=key):
                observation = legacy.observation()
                observation['quality'][key] = value
                self.reject_without_arm_io(observation)

    def test_corner_rms_must_be_finite_within_3_and_match_reported_quality(self):
        for rms in (None, True, '2.7', -1., 3.0001, math.nan, math.inf):
            for field in ('quality', 'cube_pose'):
                with self.subTest(rms=rms, field=field):
                    observation = cube_observation()
                    if field == 'quality':observation['quality']['reprojection_rms_px'] = rms
                    else:observation['payload']['cube_pose']['corner_reprojection_rms_px'] = rms
                    self.reject_without_arm_io(observation)
        observation = cube_observation(2.7)
        observation['payload']['cube_pose']['corner_reprojection_rms_px'] = 2.69
        self.reject_without_arm_io(observation)

    def test_incomplete_pose_and_incompatible_units_are_rejected_before_dispatch(self):
        mutations = [('R_Object_to_Camera', [[1., 0., 0.]]),
            ('R_Object_to_Camera', [[True, 0., 0.], [0., 1., 0.], [0., 0., 1.]]),
            ('R_Object_to_Camera', [[1., 0., 0.], [0., math.nan, 0.], [0., 0., 1.]]),
            ('R_Object_to_Camera', [[-1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]),
            ('R_Object_to_Camera', [[2., 0., 0.], [0., 1., 0.], [0., 0., 1.]]),
            ('t_Object_to_Camera_mm', [1., 2.]), ('object_center_mm', [0., 0., 0.]),
            ('edge_length_mm', .02), ('object_points_mm', [[0., 0., 0.]]*4),
            ('image_points_px', [[0., 0.]]*6), ('cube_center_Camera_mm', [0., math.inf, 200.]),
            ('cube_projected_corners_px', [[0., 0.]]*7), ('cube_projection_source', 'inferred_bbox')]
        for key, value in mutations:
            with self.subTest(key=key, value=value):
                observation = cube_observation(.5)
                observation['payload']['cube_pose'][key] = value
                self.reject_without_arm_io(observation)
        observation = cube_observation()
        observation['payload']['cube_center_Base_mm'][0] += .1
        self.reject_without_arm_io(observation)

    def test_cube_evidence_cannot_masquerade_as_measured_planar_faces(self):
        for key in ('face_reprojection_rms_px', 'face_quad_px', 'physical_edges', 'side_faces', 'poses'):
            for section in ('payload', 'cube_pose'):
                with self.subTest(key=key, section=section):
                    observation = cube_observation(.5)
                    target = observation['payload'] if section == 'payload' else observation['payload']['cube_pose']
                    target[key] = []
                    self.reject_without_arm_io(observation)

    def test_original_capture_and_raw_target_requirements_also_apply_to_cubes(self):
        for change in ({'units': 'm'}, {'range_only': True}, {'xy_compensation_applied': True},
                {'clock_domain': 'host'}, {'timestamp_ms': 6}, {'capture_timestamp_ms': None},
                {'mode': 'coarse'}, {'status': 'no_complete_face'}):
            with self.subTest(change=change):
                observation = cube_observation()
                observation.update(change)
                self.reject_without_arm_io(observation)


if __name__ == '__main__':
    unittest.main()
