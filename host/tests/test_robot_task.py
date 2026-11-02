"""Car-to-arm authority, frame identity and one-shot task sequencing.

The positive handoff uses the real stopped parking verifier. Resources below
belong to the approach callback, as on the board; its return is their lifetime
boundary. This tests coordination, not physical grabbing or camera quality.
"""
import copy
import importlib.util
import json
import unittest
from unittest.mock import Mock

from device.k230.car.near_control import verify_parked
from device.k230.car.parking_window import bounds_mm
from host.car.runtime import load
from host.project_paths import ROOT


class RobotTaskTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Load the pure coordinator exactly as a flat board module; importing
        # it must not require camera, UART, audio or an arm action runtime.
        spec = importlib.util.spec_from_file_location(
            '_tested_robot_task', ROOT/'device/k230/robot/task.py')
        cls.task = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.task)

    def setUp(self):
        workspace, _, projection = load()
        config = json.loads((ROOT/'configs/car/near_parking.json').read_text())
        x = sum(bounds_mm(workspace)['Base_X_mm'])/2
        y = sum(bounds_mm(workspace, x)['Base_Y_mm'])/2
        n = projection.normal
        z = (projection.offset+10-n[0]*x-n[1]*y)/n[2]
        point = [x, y, z]
        delta = [point[k]-projection.origin[k] for k in range(3)]
        camera = [sum(projection.T[k][j]*delta[k] for k in range(3)) for j in range(3)]
        pose = {'face_reprojection_rms_px': .5, 'cube_center_Camera_mm': camera,
                'inward_face_normal_Camera': [0., 0., 1.]}
        observation = {
            'schema': 'aic.vision_observation.v1', 'mode': 'precise',
            'frame_id': 91, 'capture_timestamp_ms': 4000, 'timestamp_ms': 4000,
            'clock_domain': 'k230_ticks_ms', 'timestamp_source': 'software_snapshot_call',
            'image_size_px': [1280, 960], 'status': 'localized',
            'coordinate_frame': 'Base', 'units': 'mm',
            'reference_point': 'cube_geometric_center', 'xy_compensation_applied': False,
            'cube_center_Base_mm': point, 'motion_performed': False,
            'payload': {'status': 'complete_face_candidate',
                        'cube_center_Base_mm': list(point),
                        'cube_center_Camera_mm': camera, 'poses': [pose],
                        'vision_path': 'shared_bounded_physical_sides'}}
        verdict = verify_parked(observation, projection, workspace, config,
                               4700, 3900, 4700, stationary_since_capture=True)
        self.assertTrue(verdict['accepted'])
        self.parked = {
            'schema': 'aic.car.near_parking.v2', 'status': 'parked',
            'motion_performed': True, 'automatic_arm_action': False,
            'verification': verdict, 'verification_observation': observation,
            'verifications': [{'verdict': verdict, 'observation': observation}],
            'stop_states': [{'run': 0, 'pwm': 0, 'reason': 1}]}
        self.arm_result = {'stages_completed': True, 'torque_state': 'unchanged',
            'grasp_confirmed':True,'grip_confirmation':{
                'axis':6,'goal':2155,'position':2185,'delta_counts':30,
                'threshold_counts':30,'grasp_confirmed':True,'method':'axis6_stable_goal_delta'}}

    def execute(self, parking=None, approach_error=None, arm_error=None, emit=True):
        parking = self.parked if parking is None else parking
        self.timeline, self.events = [], []
        self.resources = {'camera': False, 'motor_uart': False}

        def approach():
            self.resources.update(camera=True, motor_uart=True)
            self.timeline.append('approach_resources_open')
            try:
                if approach_error is not None:
                    raise approach_error
                return parking
            finally:
                self.resources['camera'] = False
                self.timeline.append('camera_closed')
                self.resources['motor_uart'] = False
                self.timeline.append('motor_uart_closed')

        def grasp(observation):
            self.assertFalse(any(self.resources.values()),
                             'Arm must start after the approach resource owner returns')
            self.timeline.append('grasp')
            if arm_error is not None:
                raise arm_error
            return self.arm_result

        def record(event, details=None):
            self.events.append(event)
            self.timeline.append('event:'+event)

        self.approach = Mock(side_effect=approach)
        self.grasp = Mock(side_effect=grasp)
        self.emit = Mock(side_effect=record)
        return self.task.execute_task(self.approach, self.grasp, self.emit if emit else None)

    def test_accepted_stopped_frame_is_reused_once_after_resource_release(self):
        original = copy.deepcopy(self.parked)
        result = self.execute()
        self.assertEqual(result['status'], 'complete')
        self.approach.assert_called_once_with()
        self.grasp.assert_called_once()
        self.assertIs(self.grasp.call_args.args[0], self.parked['verification_observation'])
        for event in ('camera_closed', 'motor_uart_closed'):
            self.assertLess(self.timeline.index(event), self.timeline.index('grasp'))
        self.assertEqual(self.events,
                         ['approach_started', 'approach_finished', 'grasp_started', 'complete'])
        self.assertEqual(self.parked, original)
        self.assertIsNot(result.get('actual_grasp_success'), True)

    def test_motion_completion_remains_distinct_from_encoder_grasp_confirmation(self):
        for delta,confirmed in ((29,False),(30,True)):
            with self.subTest(delta=delta):
                confirmation=dict(self.arm_result['grip_confirmation'],
                    position=2155+delta,delta_counts=delta,grasp_confirmed=confirmed)
                self.arm_result.update(grip_confirmation=confirmation,grasp_confirmed=confirmed)
                result=self.execute()
                self.assertEqual(result['status'],'complete')
                self.assertTrue(result['arm_started'])
                self.assertIs(result['grasp_confirmed'],confirmed)
                self.assertEqual(result['grip_confirmation'],confirmation)
                self.assertIs(result['physical_grasp_confirmed'],False)
                self.approach.assert_called_once();self.grasp.assert_called_once()
                self.assertIs(self.emit.call_args.args[1]['grasp_confirmed'],confirmed)

    def test_missing_grip_result_does_not_become_a_positive_confirmation(self):
        self.arm_result={'stages_completed':True}
        result=self.execute()
        self.assertEqual(result['status'],'complete')
        self.assertIsNot(result.get('grasp_confirmed'),True)
        self.assertIs(result['physical_grasp_confirmed'],False)

    def test_moving_range_history_is_not_reused_as_the_arm_observation(self):
        moving = {'status': 'no_complete_face', 'frame_id': 40,
                  'capture_timestamp_ms': 2000,
                  'range_candidate': {'status': 'range_candidate',
                      'raw_center_Base_mm': [430, 30, -50],
                      'not_arm_or_parking_authority': True}}
        self.parked['fine_observations'] = [moving]
        result = self.execute()
        self.assertEqual(result['status'], 'complete')
        supplied = self.grasp.call_args.args[0]
        self.assertIs(supplied, self.parked['verification_observation'])
        self.assertIsNot(supplied, moving)
        self.assertEqual(supplied['frame_id'], 91)
        self.assertEqual(supplied['capture_timestamp_ms'], 4000)

    def test_each_failed_approach_finishes_without_arm_or_retry(self):
        for status in ('no_target', 'precise_unavailable', 'no_precise_target',
                       'motion_stopped', 'parking_failed'):
            with self.subTest(status=status):
                failed = copy.deepcopy(self.parked)
                failed['status'] = status
                result = self.execute(failed)
                self.assertEqual(result['status'], 'parking_failed')
                self.approach.assert_called_once_with()
                self.grasp.assert_not_called()
                self.assertNotIn('grasp_started', self.events)
                self.assertNotIn('complete', self.events)

    def test_window_refusal_prevents_grasp_even_with_a_valid_complete_pose(self):
        self.parked['verification']['accepted'] = False
        self.parked['verification']['status'] = 'not_parked'
        result = self.execute()
        self.assertEqual(result['status'], 'parking_failed')
        self.grasp.assert_not_called()

    def test_range_only_candidate_inside_a_saved_window_has_no_arm_authority(self):
        observation = self.parked['verification_observation']
        point = observation.pop('cube_center_Base_mm')
        observation.pop('payload')
        observation['status'] = 'no_complete_face'
        observation['range_candidate'] = {
            'schema': 'aic.car.side_range.v1', 'status': 'range_candidate',
            'coordinate_frame': 'Base', 'units': 'mm', 'Base_X_mm': point[0],
            'raw_center_Base_mm': point, 'X_interval_mm': [point[0], point[0]],
            'not_arm_or_parking_authority': True}
        result = self.execute()
        self.assertEqual(result['status'], 'invalid_handoff')
        self.grasp.assert_not_called()

    def test_coarse_or_compensated_coordinates_do_not_borrow_the_saved_verdict(self):
        for change in ({'mode': 'coarse'}, {'coordinate_frame': 'Camera'},
                       {'xy_compensation_applied': True}):
            with self.subTest(change=change):
                parked = copy.deepcopy(self.parked)
                parked['verification_observation'].update(change)
                result = self.execute(parked)
                self.assertEqual(result['status'], 'invalid_handoff')
                self.grasp.assert_not_called()

    def test_a_different_frame_cannot_borrow_the_accepted_parking_verdict(self):
        self.parked['verification_observation']['frame_id'] = 90
        result = self.execute()
        self.assertEqual(result['status'], 'invalid_handoff')
        self.grasp.assert_not_called()

    def test_original_capture_time_must_match_the_accepted_verification(self):
        observation = self.parked['verification_observation']
        observation['capture_timestamp_ms'] = observation['timestamp_ms'] = 3990
        result = self.execute()
        self.assertEqual(result['status'], 'invalid_handoff')
        self.grasp.assert_not_called()

    def test_incomplete_or_different_payload_cannot_borrow_the_saved_verdict(self):
        for defect in ('missing_payload', 'wrong_status', 'empty_poses', 'different_center'):
            with self.subTest(defect=defect):
                parked = copy.deepcopy(self.parked)
                observation = parked['verification_observation']
                if defect == 'missing_payload':
                    observation.pop('payload')
                elif defect == 'wrong_status':
                    observation['payload']['status'] = 'no_complete_face'
                elif defect == 'empty_poses':
                    observation['payload']['poses'] = []
                else:
                    observation['payload']['cube_center_Base_mm'][0] += 1
                result = self.execute(parked)
                self.assertEqual(result['status'], 'invalid_handoff')
                self.grasp.assert_not_called()
                self.assertNotIn('grasp_started', self.events)

    def test_approach_exception_is_preserved_and_never_starts_the_arm(self):
        error = RuntimeError('car communication failed')
        with self.assertRaises(RuntimeError) as caught:
            self.execute(approach_error=error)
        self.assertIs(caught.exception, error)
        self.approach.assert_called_once_with()
        self.grasp.assert_not_called()
        self.assertFalse(any(self.resources.values()))
        self.assertNotIn('complete', self.events)

    def test_arm_exception_is_preserved_without_repeating_the_car_or_arm(self):
        error = RuntimeError('arm position read failed')
        with self.assertRaises(RuntimeError) as caught:
            self.execute(arm_error=error)
        self.assertIs(caught.exception, error)
        self.approach.assert_called_once_with()
        self.grasp.assert_called_once()
        self.assertNotIn('complete', self.events)

    def test_emitting_is_optional_and_two_calls_represent_two_distinct_wakes(self):
        for unused in range(2):
            result = self.execute(emit=False)
            self.assertEqual(result['status'], 'complete')
            self.approach.assert_called_once_with()
            self.grasp.assert_called_once()
            self.emit.assert_not_called()


if __name__ == '__main__':
    unittest.main()
