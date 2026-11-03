"""Five/six-corner handoffs are distinct capabilities; mock IO only."""
import copy
import math
import unittest

from host.tests import test_pick_handoff as legacy
from tools.deploy.k230_pick import build_bundle

EVIDENCE = 'nonplanar_cube_five_corners'
SIX_EVIDENCE = 'nonplanar_cube_six_corners'
SIX_BOUNDARY = 'six_measured_outline_edges_six_corners'
SIX_PATH = 'shared_nonplanar_cube_outline'
BACK_TOP_EVIDENCE = 'nonplanar_cube_face_back_top'
BACK_TOP_BOUNDARY = 'seven_measured_edges_front_and_back_top_corners'
BACK_TOP_PATH = 'shared_nonplanar_cube_face_back_top'


def cube_observation(rms=2.7, corner_count=5):
    observation = legacy.observation()
    objects = [[-10., 0., 20.], [-10., 20., 20.], [10., 20., 20.],
               [10., 0., 0.], [-10., 0., 0.]]
    if corner_count == 6:
        objects.insert(3, [10., 20., 0.])
    evidence = EVIDENCE if corner_count == 5 else SIX_EVIDENCE
    boundary = 'six_measured_edges_five_corners' if corner_count == 5 else SIX_BOUNDARY
    path = 'shared_nonplanar_cube_corners' if corner_count == 5 else SIX_PATH
    corners = [[-10., 0., 20.], [10., 0., 20.], [10., 0., 0.], [-10., 0., 0.],
               [-10., 20., 20.], [10., 20., 20.], [10., 20., 0.], [-10., 20., 0.]]
    def pixel(point):
        return [640.+1000.*point[0]/(point[2]+200.),
                480.+1000.*point[1]/(point[2]+200.)]
    pose = {'R_Object_to_Camera': [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]],
            't_Object_to_Camera_mm': [0., 0., 200.], 'object_center_mm': [0., 10., 10.],
            'edge_length_mm': 20., 'object_points_mm': objects,
            'image_points_px': [[pixel(p)[0] + (rms if corner_count == 6 else 0.), pixel(p)[1]]
                                for p in objects],
            'corner_reprojection_rms_px': rms, 'cube_center_Camera_mm': [0., 10., 210.],
            'cube_projected_corners_px': [pixel(p) for p in corners],
            'cube_projection_source': 'pose_transform'}
    observation['quality'] = {'evidence_type': evidence, 'reprojection_rms_px': rms,
        'reprojection_limit_px': 3, 'boundary_method': boundary,
        'vision_path': path}
    observation['payload'] = {'status': 'complete_cube_candidate',
        'evidence_type': evidence, 'cube_pose': pose,
        'boundary_method': boundary, 'vision_path': path,
        'cube_center_Base_mm': list(observation['cube_center_Base_mm']),
        'observed_edges': [{'edge_id': i, 'support': 64} for i in range(6)]}
    return observation


def back_top_observation(rms=2.7):
    observation = cube_observation(rms, 6)
    raw = observation['payload']
    for record in (raw, observation['quality']):
        record.update(evidence_type=BACK_TOP_EVIDENCE,
                      boundary_method=BACK_TOP_BOUNDARY, vision_path=BACK_TOP_PATH)
    pose = raw['cube_pose']
    pose['object_points_mm'] = [[-10., 0., 20.], [10., 0., 20.], [10., 0., 0.],
                               [-10., 0., 0.], [-10., 20., 20.], [10., 20., 20.]]
    pose['image_points_px'] = [[640.+1000.*p[0]/(p[2]+200.)+rms,
                                480.+1000.*p[1]/(p[2]+200.)]
                               for p in pose['object_points_mm']]
    raw['observed_edges'] = [{'quality': {'sampled_stations': 64, 'support': 64,
                                         'rms_px': .5}} for _ in range(7)]
    raw['half_corner_errors_px'] = [0.]*12
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
        for count in (5, 6):
            with self.subTest(corner_count=count):
                import json
                from device.k230.vision.runtime import localize_precise
                from device.k230.pick.pose import project
                original = cube_observation(corner_count=count)
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
                raw['half_corner_errors_px'] = [0.]*(2*count)
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
                self.assertEqual(result['payload']['cube_pose']['corner_count'], count)
                self.assertEqual(result['quality']['evidence_type'], EVIDENCE if count == 5 else SIX_EVIDENCE)
                for observation in (result, json.loads(json.dumps(result))):
                    found = self.app._handoff_target(observation)
                    self.assertEqual(found['cube_pose'], observation['payload']['cube_pose'])
                    self.assertEqual(found['cube_center_Base_mm'], [0., 230., 10.])

    def test_six_corner_real_projection_and_json_preserve_the_distinct_capability(self):
        import json
        for rms in (0., 2.01, 2.7, 3.):
            with self.subTest(rms=rms):
                observation = cube_observation(rms, 6)
                pose = observation['payload']['cube_pose']
                self.assertEqual(pose['object_points_mm'], [[-10., 0., 20.], [-10., 20., 20.],
                    [10., 20., 20.], [10., 20., 0.], [10., 0., 0.], [-10., 0., 0.]])
                squared = []
                for point, measured in zip(pose['object_points_mm'], pose['image_points_px']):
                    camera = [a+b for a,b in zip(point, pose['t_Object_to_Camera_mm'])]
                    predicted = [640.+1000.*camera[0]/camera[2],
                                 480.+1000.*camera[1]/camera[2]]
                    squared.append(sum((a-b)**2 for a,b in zip(predicted, measured)))
                self.assertAlmostEqual(math.sqrt(sum(squared)/6), rms)
                before = copy.deepcopy(observation)
                for value in (observation, json.loads(json.dumps(observation))):
                    found = self.app._handoff_target(value)
                    self.assertEqual(found['evidence_type'], SIX_EVIDENCE)
                    self.assertEqual(found['cube_pose'], pose)
                    self.assertEqual(found['observation']['quality']['reprojection_limit_px'], 3)
                    self.assertEqual(found['observation']['capture_timestamp_ms'], 5)
                    self.assertNotIn('poses', found)
                self.assertEqual(observation, before)

    def test_five_and_six_markers_cannot_exchange_their_point_counts(self):
        for count, other in ((5, 6), (6, 5)):
            for field in ('object_points_mm', 'image_points_px'):
                with self.subTest(count=count, field=field):
                    observation = cube_observation(.5, count)
                    observation['payload']['cube_pose'][field] = cube_observation(.5, other)['payload']['cube_pose'][field]
                    self.reject_without_arm_io(observation)

    def test_six_raw_and_quality_markers_must_match_without_legacy_fallback(self):
        required = [('payload', 'status'), ('payload', 'evidence_type'),
                    ('payload', 'cube_pose'), ('payload', 'boundary_method'),
                    ('payload', 'vision_path'), ('quality', 'evidence_type'),
                    ('quality', 'boundary_method'), ('quality', 'vision_path')]
        for section, key in required:
            with self.subTest(missing=(section, key)):
                observation = cube_observation(.5, 6)
                del observation[section][key]
                self.reject_without_arm_io(observation)
        for section, key, value in [
                ('payload', 'evidence_type', EVIDENCE), ('quality', 'evidence_type', EVIDENCE),
                ('payload', 'evidence_type', ['bad']),
                ('payload', 'boundary_method', 'six_measured_edges_five_corners'),
                ('quality', 'boundary_method', 'six_measured_edges_five_corners'),
                ('payload', 'vision_path', 'shared_nonplanar_cube_corners'),
                ('quality', 'vision_path', 'shared_nonplanar_cube_corners')]:
            with self.subTest(mismatch=(section, key, value)):
                observation = cube_observation(.5, 6)
                observation[section][key] = value
                self.reject_without_arm_io(observation)
        for section in ('payload', 'quality'):
            for key, value in [('evidence_type', SIX_EVIDENCE),
                               ('boundary_method', SIX_BOUNDARY), ('vision_path', SIX_PATH)]:
                with self.subTest(partial=(section, key)):
                    observation = legacy.observation()
                    observation[section][key] = value
                    self.reject_without_arm_io(observation)

    def test_optional_corner_count_must_match_evidence_in_every_record(self):
        for count in (5, 6):
            for section in ('payload', 'cube_pose', 'quality'):
                observation = cube_observation(.5, count)
                target = observation['payload']['cube_pose'] if section == 'cube_pose' else observation[section]
                target['corner_count'] = count
                self.app._handoff_target(observation)
                for value in (True, str(count), float(count), 11-count):
                    with self.subTest(count=count, section=section, value=value):
                        target['corner_count'] = value
                        self.reject_without_arm_io(observation)

    def test_old_five_raw_labels_remain_optional_but_explicit_conflicts_are_rejected(self):
        observation = cube_observation(.5)
        observation['payload'].pop('boundary_method')
        observation['payload'].pop('vision_path')
        self.assertEqual(self.app._handoff_target(observation)['evidence_type'], EVIDENCE)
        for key, value in [('boundary_method', SIX_BOUNDARY), ('vision_path', SIX_PATH)]:
            with self.subTest(key=key):
                changed = copy.deepcopy(observation)
                changed['payload'][key] = value
                self.reject_without_arm_io(changed)

    def test_six_corner_keeps_quality_geometry_and_no_fake_planar_face_requirements(self):
        mutations = [('corner_reprojection_rms_px', 3.001),
                     ('corner_reprojection_rms_px', math.nan),
                     ('R_Object_to_Camera', [[-1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]),
                     ('edge_length_mm', .02), ('object_center_mm', [0., 0., 0.])]
        for key, value in mutations:
            with self.subTest(key=key, value=value):
                observation = cube_observation(.5, 6)
                observation['payload']['cube_pose'][key] = value
                self.reject_without_arm_io(observation)
        for field in ('face_reprojection_rms_px', 'face_quad_px', 'physical_edges', 'side_faces', 'poses'):
            for section in ('payload', 'cube_pose'):
                with self.subTest(field=field, section=section):
                    observation = cube_observation(.5, 6)
                    target = observation['payload']['cube_pose'] if section == 'cube_pose' else observation[section]
                    target[field] = []
                    self.reject_without_arm_io(observation)
        observation = cube_observation(.5, 6)
        observation['quality']['reprojection_rms_px'] = 3.001
        self.reject_without_arm_io(observation)

    def test_valid_six_handoff_keeps_motion_stages_and_uses_the_same_raw_center(self):
        case = legacy.PickHandoffTests()
        case.bundle = self.bundle
        case.setUp()
        case.observation = cube_observation(.5, 6)
        with case.harness() as h:
            result = case.call(execute=True)
        self.assertEqual([v for v in case.events if v in legacy.STAGES], list(legacy.STAGES))
        h.read.assert_not_called()
        h.geometry.plan.assert_called_once_with([.245, -.028], case.ready)
        self.assertEqual(result['hover'], case.plan['hover'])

    def test_back_top_real_projection_and_json_use_their_separate_3px_capability(self):
        import json
        for rms in (0., 2.01, 2.7, 3.):
            with self.subTest(rms=rms):
                observation = back_top_observation(rms)
                pose = observation['payload']['cube_pose']
                squared = []
                for point, measured in zip(pose['object_points_mm'], pose['image_points_px']):
                    predicted = [640.+1000.*point[0]/(point[2]+200.),
                                 480.+1000.*point[1]/(point[2]+200.)]
                    squared.append(sum((a-b)**2 for a,b in zip(predicted, measured)))
                self.assertAlmostEqual(math.sqrt(sum(squared)/6), rms)
                self.assertEqual(len(observation['payload']['observed_edges']), 7)
                before = copy.deepcopy(observation)
                for value in (observation, json.loads(json.dumps(observation))):
                    found = self.app._handoff_target(value)
                    self.assertEqual(found['evidence_type'], BACK_TOP_EVIDENCE)
                    self.assertEqual(found['cube_pose'], pose)
                    self.assertEqual(found['observation']['quality']['reprojection_limit_px'], 3)
                    self.assertNotIn('poses', found)
                    self.assertNotIn('side_faces', found)
                self.assertEqual(observation, before)

    def test_back_top_requires_complete_matching_markers_and_cannot_borrow_outline(self):
        for section in ('payload', 'quality'):
            for key in ('evidence_type', 'boundary_method', 'vision_path'):
                with self.subTest(missing=(section, key)):
                    observation = back_top_observation(.5)
                    del observation[section][key]
                    self.reject_without_arm_io(observation)
            for key, value in [('evidence_type', SIX_EVIDENCE),
                               ('boundary_method', SIX_BOUNDARY), ('vision_path', SIX_PATH),
                               ('evidence_type', EVIDENCE), ('evidence_type', 'physical_face')]:
                with self.subTest(mismatch=(section, key, value)):
                    observation = back_top_observation(.5)
                    observation[section][key] = value
                    self.reject_without_arm_io(observation)
            for key, value in [('evidence_type', BACK_TOP_EVIDENCE),
                               ('boundary_method', BACK_TOP_BOUNDARY), ('vision_path', BACK_TOP_PATH)]:
                with self.subTest(partial=(section, key)):
                    observation = legacy.observation()
                    observation[section][key] = value
                    self.reject_without_arm_io(observation)

    def test_back_top_preserves_shared_count_rms_and_no_planar_face_validation(self):
        for field, value in [('object_points_mm', [[0., 0., 0.]]*5),
                             ('image_points_px', [[0., 0.]]*7),
                             ('corner_reprojection_rms_px', 3.001),
                             ('corner_reprojection_rms_px', math.nan),
                             ('R_Object_to_Camera', [[-1., 0., 0.], [0., 1., 0.], [0., 0., 1.]])]:
            with self.subTest(field=field):
                observation = back_top_observation(.5)
                observation['payload']['cube_pose'][field] = value
                self.reject_without_arm_io(observation)
        for section in ('payload', 'cube_pose', 'quality'):
            for value in (5, 7, True, 6., '6'):
                with self.subTest(section=section, count=value):
                    observation = back_top_observation(.5)
                    target = observation['payload']['cube_pose'] if section == 'cube_pose' else observation[section]
                    target['corner_count'] = value
                    self.reject_without_arm_io(observation)
        for section in ('payload', 'cube_pose'):
            for field in ('poses', 'side_faces', 'face_reprojection_rms_px'):
                with self.subTest(section=section, field=field):
                    observation = back_top_observation(.5)
                    target = observation['payload']['cube_pose'] if section == 'cube_pose' else observation[section]
                    target[field] = []
                    self.reject_without_arm_io(observation)

    def test_back_top_handoff_keeps_existing_motion_stages_without_camera_or_reads(self):
        case = legacy.PickHandoffTests()
        case.bundle = self.bundle
        case.setUp()
        case.observation = back_top_observation(.5)
        with case.harness() as h:
            result = case.call(execute=True)
        self.assertEqual([v for v in case.events if v in legacy.STAGES], list(legacy.STAGES))
        h.read.assert_not_called()
        h.geometry.plan.assert_called_once_with([.245, -.028], case.ready)
        self.assertEqual(result['hover'], case.plan['hover'])

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
