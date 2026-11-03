"""A measured travel anchor outlives uncertain coarse colour observations."""
import copy
import unittest

from device.k230.car.target_memory import TargetMemory


class Projection:
    normal = [0., 0., 1.]
    offset = 0.
    forward_axis_Base = [1., 0., 0.]
    motion_origin_Base_mm = [0., 0., 0.]

    def base_to_pixel(self, point):
        return [640 + 500 * point[1] / point[0], 480 + 20000 / point[0]]

    def pixel_to_plane(self, pixel, height_mm=0):
        x = 20000 / (pixel[1] - 480)
        return [x, (pixel[0] - 640) * x / 500, height_mm]

    def camera_table_foot(self):
        return [0., 0., 0.]


class StaticAnchorAuthorityTests(unittest.TestCase):
    def setUp(self):
        self.projection = Projection()
        self.config = {'track_width_mm': 100, 'tracking': {
            'image_size_px': [1280, 960], 'max_age_ms': 500,
            'confirmation_frames': 1, 'max_size_ratio': 3,
            'max_center_jump_fraction': .15}, 'tracking_task': {
                'candidate_confirmation_frames': 1,
                'movement_confirmation_frames': 2,
                'movement_pixel_tolerance_px': 12,
                'movement_distance_fraction': .10}}
        self.memory = TargetMemory(self.projection, self.config)
        self.stamp = 0

    def observation(self, point=(500, 30, 0), **changes):
        self.stamp += 100
        pixel = self.projection.base_to_pixel(point)
        evidence = dict(visible=True, associated=True, ambiguous=False,
            candidate_count=1, association_confirmations=1,
            navigation_confirmed=True, timestamp_ms=self.stamp,
            distance_valid=True, ground_contact_proxy_Base_mm=list(point),
            center_pixel=pixel, bbox_px=[pixel[0]-10, pixel[1]-10, 20, 20],
            edge_directions=[])
        evidence.update(changes)
        return dict(status='tracking', frame_id=self.stamp,
            timestamp_ms=self.stamp, capture_timestamp_ms=self.stamp,
            visual_evidence=evidence)

    def absent(self):
        return self.observation(visible=False, associated=False,
            distance_valid=False, candidate_count=0, center_pixel=None, bbox_px=None)

    def seed(self, capability='full'):
        source = self.observation()
        self.memory.update(source)
        fine = dict(schema='aic.vision_observation.v1', mode='precise',
            status='localized', frame_id=source['frame_id'],
            capture_timestamp_ms=source['capture_timestamp_ms'],
            timestamp_ms=source['capture_timestamp_ms'], clock_domain='k230_ticks_ms',
            image_size_px=[1280, 960], coordinate_frame='Base', units='mm',
            reference_point='cube_geometric_center', xy_compensation_applied=False,
            cube_center_Base_mm=[490, 35, 10], target_association={'status': 'matched'})
        if capability == 'range':
            point = fine.pop('cube_center_Base_mm')
            fine.update(status='no_complete_face', range_candidate={
                'schema': 'aic.car.side_range.v1', 'status': 'range_candidate',
                'coordinate_frame': 'Base', 'units': 'mm', 'Base_X_mm': point[0],
                'raw_center_Base_mm': point, 'not_arm_or_parking_authority': True})
        self.assertTrue(self.memory.bind_precise(fine, source, 0))
        return source, fine

    def assert_anchor(self, fine):
        state = self.memory.snapshot()
        self.assertEqual(state['version'], 0)
        self.assertTrue(state['anchor_valid'])
        self.assertEqual(state['last_precise'], fine)
        return state

    def test_running_unassociated_jump_cannot_replace_full_or_range_anchor(self):
        for capability in ('full', 'range'):
            with self.subTest(capability=capability):
                self.setUp()
                unused, fine = self.seed(capability)
                identity = copy.deepcopy(self.memory._identity_visual)
                for unused in range(3):
                    state = self.memory.update(self.observation((1100, 600, 0),
                        associated=False, reason='unassociated_blue'), motion_state={'run': 1})
                    self.assert_anchor(fine)
                    self.assertFalse(state['tracking_update'])
                    self.assertFalse(state['associated'])
                self.assertEqual(self.memory._identity_visual, identity)

    def test_retarget_api_cannot_revoke_an_existing_measured_distance(self):
        unused, fine = self.seed()
        current = self.observation()
        self.memory.update(current, motion_state={'run': 1})
        self.assertIsNone(self.memory.retarget_current(current, 'fine_current_mismatch'))
        self.assert_anchor(fine)

    def test_gap_then_new_native_candidate_does_not_inherit_or_revoke_anchor(self):
        for capability in ('full', 'range'):
            with self.subTest(capability=capability):
                self.setUp()
                source, fine = self.seed(capability)
                identity = copy.deepcopy(self.memory._identity_visual)
                self.memory.update(self.absent(), motion_state={'run': 1})
                self.stamp += 600  # Native acquisition can have expired too.
                for count in (1, 2, 3):
                    state = self.memory.update(self.observation((1100, 600, 0),
                        association_confirmations=count,
                        bbox_px=[910, 495, 4, 4]), motion_state={'run': 1})
                    self.assert_anchor(fine)
                    self.assertFalse(state['associated'])
                    self.assertEqual(state['last_seen'], source)
                    self.assertEqual(self.memory._identity_visual, identity)

    def test_ambiguous_or_missing_frames_never_revoke_distance(self):
        unused, fine = self.seed()
        for unknown in (self.observation(associated=False, ambiguous=True, candidate_count=8),
                        self.absent(), self.absent()):
            self.memory.update(unknown, motion_state={'run': 1})
            self.assert_anchor(fine)

    def test_old_or_duplicate_input_cannot_add_movement_votes(self):
        unused, fine = self.seed()
        current = self.observation((500, 100, 0))
        self.memory.update(current, motion_state={'run': 0})
        old = copy.deepcopy(current)
        old['capture_timestamp_ms'] -= 100
        old['frame_id'] -= 100
        for duplicate in (current, old, current):
            self.memory.update(duplicate, motion_state={'run': 0})
            self.assert_anchor(fine)

    def test_unassociated_unique_patch_cannot_confirm_stationary_movement(self):
        unused, fine = self.seed()
        for unused in range(3):
            state = self.memory.update(self.observation((500, 100, 0),
                associated=False, reason='unassociated_blue'), motion_state={'run': 0})
            self.assert_anchor(fine)
            self.assertFalse(state['confirmed_moved'])

    def test_stationary_same_target_translation_keeps_existing_two_frame_rule(self):
        self.seed()
        first = self.memory.update(self.observation((500, 100, 0)), motion_state={'run': 0})
        self.assertEqual(first['movement_state'], 'suspected')
        second = self.memory.update(self.observation((500, 100, 0)), motion_state={'run': 0})
        self.assertTrue(second['confirmed_moved'])
        self.assertEqual(second['version'], 1)
        self.assertFalse(second['anchor_valid'])

    def test_running_same_target_translation_is_confirmed_after_two_frames(self):
        self.seed()
        first = self.memory.update(self.observation((500, 100, 0)), motion_state={'run': 1})
        self.assertEqual(first['version'], 0)
        self.assertFalse(first['confirmed_moved'])
        second = self.memory.update(self.observation((500, 100, 0)), motion_state={'run': 1})
        self.assertTrue(second['confirmed_moved'])
        self.assertEqual(second['version'], 1)
        self.assertFalse(second['anchor_valid'])

    def test_running_translation_after_real_encoder_forward_motion_is_confirmed(self):
        self.seed()
        for index in (1, 2):
            # The body advances7mm per frame. Human lateral movement contributes
            # a separate70px shift which remains after ego compensation.
            state = self.memory.update(self.observation((500-7*index, 100, 0)),
                [(7, 7)], motion_state={'run': 1})
            self.assertEqual(state['confirmed_moved'], index == 2)
            self.assertEqual(state['version'], 1 if index == 2 else 0)

    def test_large_same_size_jump_then_held_position_is_confirmed_after_two_frames(self):
        for run in (0, 1):
            with self.subTest(run=run):
                self.setUp()
                self.seed()
                for index in (1, 2):
                    # An associated whole block is moved300px once, then held
                    # still. The second observation confirms its new position;
                    # it need not move another300px to count as confirmation.
                    state = self.memory.update(self.observation((500, 330, 0)),
                        motion_state={'run': run})
                    self.assertEqual(state['confirmed_moved'], index == 2)
                    self.assertEqual(state['version'], 1 if index == 2 else 0)

    def test_large_jump_with_forward_encoder_motion_confirms_after_it_stops_moving(self):
        self.seed()
        for index in (1, 2):
            state = self.memory.update(self.observation((500-7*index, 330, 0)),
                [(7, 7)], motion_state={'run': 1})
            self.assertEqual(state['confirmed_moved'], index == 2)
            self.assertEqual(state['version'], 1 if index == 2 else 0)

    def test_edge_hint_and_one_missing_frame_do_not_revoke_measured_distance(self):
        near = self.observation(center_pixel=[20, 520], bbox_px=[10, 510, 20, 20])
        self.memory.update(near)
        fine = dict(schema='aic.vision_observation.v1', mode='precise', status='localized',
            frame_id=near['frame_id'], capture_timestamp_ms=near['capture_timestamp_ms'],
            clock_domain='k230_ticks_ms', image_size_px=[1280, 960],
            coordinate_frame='Base', units='mm', reference_point='cube_geometric_center',
            xy_compensation_applied=False, cube_center_Base_mm=[490, 35, 10])
        self.assertTrue(self.memory.bind_precise(fine, near, 0))
        self.memory.update(self.observation(center_pixel=[6, 520],
            bbox_px=[0, 510, 12, 20], distance_valid=False,
            edge_directions=['left']), motion_state={'run': 1})
        state = self.memory.update(self.absent(), motion_state={'run': 1})
        self.assert_anchor(fine)
        self.assertFalse(state['confirmed_moved'])
        self.assertEqual(state['last_disappearance_direction'], 'left')

    def test_missing_frame_resets_suspicion_without_lending_old_votes(self):
        unused, fine = self.seed()
        self.memory.update(self.observation((500, 100, 0)), motion_state={'run': 0})
        self.memory.update(self.absent(), motion_state={'run': 0})
        state = self.memory.update(self.observation((500, 170, 0)), motion_state={'run': 0})
        self.assert_anchor(fine)
        self.assertFalse(state['confirmed_moved'])
        state = self.memory.update(self.observation((500, 170, 0)), motion_state={'run': 0})
        self.assertTrue(state['confirmed_moved'])

    def test_running_wheel_model_error_alone_is_not_confirmed_motion(self):
        unused, fine = self.seed()
        for unused in range(8):
            state = self.memory.update(self.observation(), [(0, 13)], motion_state={'run': 1})
            self.assert_anchor(fine)
            self.assertFalse(state['confirmed_moved'])
        for unused in range(3):
            self.memory.update(self.observation(), motion_state={'run': 0})
            self.assert_anchor(fine)

    def test_running_box_expansion_and_proxy_noise_do_not_move_endpoint(self):
        unused, fine = self.seed()
        for point, box in (((1100, 30, 0), [650, 490, 40, 60]),
                           ((250, 30, 0), [640, 480, 80, 120])):
            for unused in range(3):
                state = self.memory.update(self.observation(point,
                    center_pixel=[670, 520], bbox_px=box), motion_state={'run': 1})
                self.assert_anchor(fine)
                self.assertFalse(state['confirmed_moved'])

    def test_single_coarse_disagreement_is_not_a_changed_fine_capability(self):
        source, fine = self.seed()
        self.memory.update(self.observation((500, 100, 0)), motion_state={'run': 1})
        self.assertNotEqual(self.memory.precise_consistency(fine, source, []), 'changed')
        self.assert_anchor(fine)

    def test_unanchored_following_keeps_existing_immediate_tracking_update(self):
        self.memory.update(self.observation())
        state = self.memory.update(self.observation((500, 330, 0),
            associated=False, reason='unassociated_blue'), motion_state={'run': 1})
        self.assertEqual(state['event'], 'tracking_update')
        self.assertEqual(state['version'], 1)


if __name__ == '__main__':
    unittest.main()
