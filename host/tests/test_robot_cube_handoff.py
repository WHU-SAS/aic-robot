"""Actual shared-runtime cube evidence through the car-to-arm coordinator."""
import copy
import math
import unittest
from unittest.mock import Mock

from device.k230.robot.task import handoff_observation, execute_task
from device.k230.vision.runtime import localize_precise
from host.tests.test_cube_observation import (fixture, FRAME, INTRINSICS, PLANE,
    FACE_BACK_TYPE, FACE_BACK_METHOD, FACE_BACK_PATH)


def parked_cube(rms=2.7, six=False, face_back_top=False):
    raw, extrinsics, unused, unused = fixture(six=six, face_back_top=face_back_top)
    raw['cube_pose']['image_points_px'] = [[x+rms, y] for x, y in
        raw['cube_pose']['image_points_px']]
    observation = localize_precise(None, dict(FRAME, clock_domain='k230_ticks_ms'),
        INTRINSICS, extrinsics, PLANE, localizer=lambda *args: raw)
    if observation['status'] != 'localized':
        raise AssertionError(observation)
    return parking(observation)


def parking(observation):
    return {'status': 'parked', 'verification_observation': observation,
        'verification': {'status': 'parked', 'accepted': True,
            'frame_id': observation['frame_id'],
            'capture_timestamp_ms': observation['capture_timestamp_ms'],
            'cube_center_Base_mm': list(observation['cube_center_Base_mm'])}}


class RobotCubeHandoffTests(unittest.TestCase):
    def rejected_without_grasp(self, parked):
        grasp = Mock()
        result = execute_task(lambda: parked, grasp)
        self.assertEqual(result['status'], 'invalid_handoff', result)
        self.assertFalse(result['arm_started'])
        grasp.assert_not_called()

    def test_runtime_cube_reuses_the_same_final_frame_without_fabricating_faces(self):
        parked = parked_cube()
        before = copy.deepcopy(parked)
        observation = parked['verification_observation']
        self.assertGreater(observation['quality']['reprojection_rms_px'], 2.)
        self.assertIs(handoff_observation(parked), observation)
        grasp = Mock(return_value={'grasp_confirmed': True})
        result = execute_task(lambda: parked, grasp)
        self.assertEqual(result['status'], 'complete')
        self.assertTrue(result['arm_started'])
        grasp.assert_called_once_with(observation)
        self.assertNotIn('poses', observation['payload'])
        self.assertNotIn('side_faces', observation['payload'])
        self.assertNotIn('face_reprojection_rms_px', observation['payload']['cube_pose'])
        self.assertFalse(result['physical_grasp_confirmed'])
        self.assertEqual(parked, before)

    def test_cube_markers_must_match_the_new_quality_policy_without_face_fallback(self):
        changes = [('payload', 'status', 'complete_face_candidate'),
            ('payload', 'evidence_type', 'unknown'),
            ('quality', 'evidence_type', 'unknown'),
            ('quality', 'reprojection_limit_px', 2.),
            ('quality', 'reprojection_limit_px', True),
            ('quality', 'boundary_method', 'four_measured_edges'),
            ('quality', 'vision_path', 'legacy_recovery')]
        for section, key, value in changes:
            parked = parked_cube(.5)
            obs = parked['verification_observation']
            obs[section][key] = value
            # An inconsistent cube must not borrow the old truthy poses test.
            obs['payload']['poses'] = [{'face_reprojection_rms_px': .1}]
            with self.subTest(section=section, key=key, value=value):
                self.rejected_without_grasp(parked)
        for section, key in [('payload', 'cube_pose'), ('quality', 'evidence_type')]:
            parked = parked_cube(.5)
            del parked['verification_observation'][section][key]
            with self.subTest(missing=(section, key)):
                self.rejected_without_grasp(parked)

    def test_cube_quality_and_corner_rms_are_finite_within3_and_consistent(self):
        parked = parked_cube()
        obs = parked['verification_observation']
        obs['quality']['reprojection_rms_px'] = 3.
        obs['payload']['cube_pose']['corner_reprojection_rms_px'] = 3.
        self.assertIs(handoff_observation(parked), obs)
        for value in (-.1, 3.001, math.nan, math.inf, True, '2.7'):
            for section in ('quality', 'cube_pose'):
                parked = parked_cube()
                obs = parked['verification_observation']
                if section == 'quality':obs['quality']['reprojection_rms_px'] = value
                else:obs['payload']['cube_pose']['corner_reprojection_rms_px'] = value
                with self.subTest(value=value, section=section):
                    self.rejected_without_grasp(parked)
        parked = parked_cube()
        parked['verification_observation']['payload']['cube_pose']['corner_reprojection_rms_px'] += .01
        self.rejected_without_grasp(parked)

    def test_cube_keeps_the_same_accepted_parking_frame_time_and_center_requirements(self):
        for changed in ('frame', 'time', 'center', 'payload_center'):
            parked = parked_cube()
            obs = parked['verification_observation']
            if changed == 'frame':obs['frame_id'] += 1
            elif changed == 'time':
                obs['capture_timestamp_ms'] += 1
                obs['timestamp_ms'] += 1
            elif changed == 'center':obs['cube_center_Base_mm'][0] += 1
            else:obs['payload']['cube_center_Base_mm'][0] += 1
            with self.subTest(changed=changed):
                self.rejected_without_grasp(parked)

    def test_incomplete_or_nonrigid_cube_cannot_start_the_arm(self):
        changes = [('cube_pose', None), ('R_Object_to_Camera', [[1., 0., 0.]]),
            ('R_Object_to_Camera', [[-1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]),
            ('R_Object_to_Camera', [[2., 0., 0.], [0., 1., 0.], [0., 0., 1.]]),
            ('t_Object_to_Camera_mm', [1., 2.]), ('edge_length_mm', .02),
            ('object_center_mm', [0., 0., 0.]), ('image_points_px', [[0., 0.]]*4),
            ('cube_projected_corners_px', [[0., 0.]]*7),
            ('cube_projection_source', 'inferred_bbox')]
        for key, value in changes:
            parked = parked_cube(.5)
            obs = parked['verification_observation']
            if key == 'cube_pose':obs['payload'][key] = value
            else:obs['payload']['cube_pose'][key] = value
            with self.subTest(key=key, value=value):
                self.rejected_without_grasp(parked)

    def test_old_complete_face_handoff_and_its_required_poses_are_unchanged(self):
        parked = parked_cube(.5)
        obs = parked['verification_observation']
        obs['payload'] = {'status': 'complete_face_candidate',
            'cube_center_Base_mm': list(obs['cube_center_Base_mm']),
            'poses': [{'face_reprojection_rms_px': .5}]}
        obs['quality'] = {'reprojection_rms_px': .5}
        self.assertIs(handoff_observation(parked), obs)
        obs['payload']['poses'] = []
        self.rejected_without_grasp(parked)

    def test_explicit_six_corner_handoff_preserves_the_same_outline_frame_without_faces(self):
        parked = parked_cube(six=True)
        observation = parked['verification_observation']
        before = copy.deepcopy(parked)
        self.assertEqual(observation['quality']['evidence_type'], 'nonplanar_cube_six_corners')
        self.assertIs(handoff_observation(parked), observation)
        grasp = Mock(return_value={'grasp_confirmed': True})
        result = execute_task(lambda: parked, grasp)
        self.assertTrue(result['arm_started'])
        grasp.assert_called_once_with(observation)
        self.assertEqual(parked, before)
        self.assertNotIn('poses', observation['payload'])
        self.assertEqual(len(observation['payload']['cube_pose']['image_points_px']), 6)

    def test_five_and_six_corner_handoffs_cannot_borrow_each_others_policy_or_counts(self):
        for six in (False, True):
            other = ('nonplanar_cube_five_corners' if six else 'nonplanar_cube_six_corners')
            for changed in ('quality_type', 'payload_type', 'both_types', 'payload_method',
                            'payload_path', 'quality_method', 'quality_path', 'point_count'):
                parked = parked_cube(.5, six=six)
                obs = parked['verification_observation']
                if changed in ('quality_type', 'both_types'):obs['quality']['evidence_type'] = other
                if changed in ('payload_type', 'both_types'):obs['payload']['evidence_type'] = other
                if changed.endswith('method'):
                    obs[changed.split('_')[0]]['boundary_method'] = ('six_measured_edges_five_corners'
                        if six else 'six_measured_outline_edges_six_corners')
                if changed.endswith('path'):
                    obs[changed.split('_')[0]]['vision_path'] = ('shared_nonplanar_cube_corners'
                        if six else 'shared_nonplanar_cube_outline')
                if changed == 'point_count':
                    pose = obs['payload']['cube_pose']
                    if six:
                        pose['object_points_mm'].pop(3);pose['image_points_px'].pop(3)
                    else:
                        pose['object_points_mm'].insert(3, [10., 20., 0.])
                        pose['image_points_px'].insert(3, [700., 500.])
                with self.subTest(six=six, changed=changed):self.rejected_without_grasp(parked)

    def test_seven_edge_face_back_top_reuses_exact_runtime_frame_without_face_fallback(self):
        parked = parked_cube(face_back_top=True)
        observation = parked['verification_observation']
        before = copy.deepcopy(parked)
        self.assertEqual(observation['quality']['evidence_type'], FACE_BACK_TYPE)
        self.assertEqual(observation['quality']['boundary_method'], FACE_BACK_METHOD)
        self.assertEqual(observation['quality']['vision_path'], FACE_BACK_PATH)
        self.assertGreater(observation['quality']['reprojection_rms_px'], 2.)
        self.assertIs(handoff_observation(parked), observation)
        grasp = Mock(return_value={'grasp_confirmed': True})
        result = execute_task(lambda: parked, grasp)
        grasp.assert_called_once_with(observation)
        self.assertTrue(result['arm_started'])
        self.assertFalse(result['physical_grasp_confirmed'])
        self.assertEqual(len(observation['payload']['observed_edges']), 7)
        self.assertNotIn('poses', observation['payload'])
        self.assertEqual(parked, before)

    def test_face_back_top_handoff_rejects_wrong_source_contract_and_cross_type_markers(self):
        for changed in ('quality_type', 'raw_type', 'method', 'path', 'six_sources',
                        'bad_seventh', 'six_halves', 'outer_layout'):
            parked = parked_cube(.5, face_back_top=True)
            observation = parked['verification_observation']; raw = observation['payload']
            if changed == 'quality_type':observation['quality']['evidence_type'] = 'nonplanar_cube_six_corners'
            elif changed == 'raw_type':raw['evidence_type'] = 'nonplanar_cube_six_corners'
            elif changed == 'method':raw['boundary_method'] = 'six_measured_outline_edges_six_corners'
            elif changed == 'path':observation['quality']['vision_path'] = 'shared_nonplanar_cube_outline'
            elif changed == 'six_sources':raw['observed_edges'].pop()
            elif changed == 'bad_seventh':raw['observed_edges'][-1]['quality']['support'] = 44
            elif changed == 'six_halves':raw['half_corner_errors_px'] = [.2]*6
            else:raw['cube_pose']['object_points_mm'] = fixture(six=True)[0]['cube_pose']['object_points_mm']
            with self.subTest(changed=changed):self.rejected_without_grasp(parked)


if __name__ == '__main__':
    unittest.main()
