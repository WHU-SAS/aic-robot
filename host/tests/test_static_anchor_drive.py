"""Precise distance remains authoritative across synthetic coarse noise.

These use the real projection/Drive with explicit image and encoder protocol
fixtures; they do not model wheel dynamics or claim a physical success rate.
"""
import copy
import unittest

from device.k230.car.continuous_math import forward_x_per_mm
from host.tests import test_car_tracking_drive as fixtures


class StaticAnchorDriveTests(unittest.TestCase):
    def setUp(self):
        self.f = fixtures.TrackingDriveTests()
        self.f.setUp()

    def moving_complete(self, *, status='localized'):
        f = self.f
        f.update(1000)
        requested = f.update(1100, 450, run=True)
        self.assertEqual(requested['request_fine'], 'moving')
        observation = f.precise(1100,
            f.target+85*forward_x_per_mm(f.projection), 'moving', status=status)
        # Keep the original source coarse pair. Its inaccurate range is not
        # allowed to replace the physical fine centre after it is accepted.
        observation['paired_coarse'] = f.coarse(1100, 450)
        return observation

    def accept_anchor(self):
        complete = self.moving_complete()
        result = self.f.drive.consume_moving_precise(complete,
            self.f.state(1200, run=True, travel=7), 1200, 1)
        self.assertTrue(result['precise_consumed'])
        self.assertFalse(result['allow_arm'])
        self.assertIsNotNone(self.f.drive.anchor)
        self.assertEqual(result['command'].split(',')[0], 'SPEED')
        self.assertEqual(int(result['command'].split(',')[3]), 78)
        return complete, result

    def test_same_frame_fine_uses_latest_status_before_any_new_coarse(self):
        complete, result = self.accept_anchor()
        for key in ('frame_id', 'capture_timestamp_ms', 'center_pixel', 'bbox_px'):
            self.assertEqual(self.f.drive.paired_coarse[key], complete['paired_coarse'][key])
        self.assertAlmostEqual(result['plan']['remaining_mm'], 78, delta=.01)
        self.assertEqual(result['plan']['heading_source'], 'fine_odometry')

    def test_far_fragment_and_noise_version_cannot_change_78mm_to_776mm(self):
        self.accept_anchor()
        result = self.f.update(1300, self.f.target+776, run=True, travel=7,
            target_version=2, target_event={'target_valid': True})
        self.assertIsNotNone(self.f.drive.anchor)
        self.assertEqual(int(result['command'].split(',')[3]), 78)
        self.assertAlmostEqual(result['plan']['remaining_mm'], 78, delta=.01)
        self.assertEqual(self.f.drive.version, 2)
        self.assertFalse(self.f.drive._searching)
        self.assertFalse(result['allow_arm'])

    def test_drop_ambiguity_exit_and_invalid_identity_keep_the_encoder_endpoint(self):
        for kind in ('missing', 'ambiguous', 'exit', 'unknown'):
            with self.subTest(kind=kind):
                self.setUp()
                self.accept_anchor()
                coarse = self.f.missing(1300)
                event = {'target_valid': False, 'search': True}
                if kind == 'ambiguous':
                    coarse = self.f.coarse(1300, 1000)
                    coarse['visual_evidence']['ambiguous'] = True
                    event['visibility'] = 'ambiguous'
                elif kind == 'exit':
                    event.update(event='exited', last_disappearance_direction='bottom')
                elif kind == 'unknown':
                    event['visibility'] = 'unknown'
                result = self.f.update(1300, run=True, travel=17, coarse=coarse,
                    target_version=2, target_event=event)
                self.assertIsNotNone(self.f.drive.anchor)
                self.assertEqual(result['command'].split(',')[0], 'SPEED')
                self.assertEqual(int(result['command'].split(',')[3]), 68)
                self.assertFalse(self.f.drive._searching)
                self.assertFalse(self.f.drive._search_reverse_required)
                self.assertFalse(result['allow_arm'])

    def test_only_confirmed_physical_movement_revokes_the_old_anchor(self):
        self.accept_anchor()
        result = self.f.update(1300, 650, run=True, travel=17,
            target_version=2, target_event={'event': 'moved',
                'movement_state': 'confirmed', 'confirmed_moved': True,
                'target_valid': True})
        self.assertIsNone(self.f.drive.anchor)
        self.assertGreater(int(result['command'].split(',')[3]), 300)
        self.assertFalse(result['allow_arm'])

    def test_finite_completion_cannot_restart_using_the_old_moving_image(self):
        complete = self.moving_complete()
        result = self.f.drive.consume_moving_precise(complete,
            self.f.state(1200, travel=85, reason=6), 1200, 1)
        self.assertTrue(result['precise_consumed'])
        self.assertTrue(result['moving_fine_stopped'])
        self.assertIsNone(self.f.drive.anchor)
        self.assertIsNone(result['command'])
        self.assertFalse(result['allow_arm'])

    def test_coast_stop_keeps_settling_when_zero_speed_leaves_positive_distance(self):
        self.accept_anchor()
        # Encoder travel leaves about5mm; measured forward speed makes the
        # original0.12s coast estimate reach the destination before power-off.
        stop = self.f.update(2500, run=True, travel=80)
        self.assertEqual(stop['command'], 'STOP')
        self.assertGreater(stop['plan']['remaining_mm'], 0)
        self.assertLess(stop['plan']['remaining_mm'], 6)
        waiting = self.f.update(2510, travel=80, reason=1)
        self.assertIsNone(waiting['command'])
        self.assertEqual(waiting['phase'], 'settling')
        verify = self.f.update(2560, travel=80, reason=1)
        self.assertIsNone(verify['command'])
        self.assertEqual(verify['request_fine'], 'static')
        self.assertFalse(verify['allow_arm'])

    def test_explicit_loss_search_waits_for_the_endpoint_and_stable_stop(self):
        complete, unused = self.accept_anchor()
        event = {'target_valid': False, 'search': True,
                 'last_disappearance_direction': 'right'}
        approaching = self.f.update(1300, run=True, travel=17,
            coarse=self.f.missing(1300), target_event=event)
        self.assertEqual(approaching['command'].split(',')[0], 'SPEED')
        self.assertEqual(int(approaching['command'].split(',')[3]), 68)
        self.assertFalse(self.f.drive._searching)
        stop = self.f.update(2500, run=True, travel=80,
            coarse=self.f.missing(2500), target_event=event)
        self.assertEqual(stop['command'], 'STOP')
        not_stable = self.f.update(2510, travel=80, reason=1,
            coarse=self.f.missing(2510), target_event=event)
        self.assertIsNone(not_stable['command'])
        self.assertFalse(self.f.drive._searching)
        self.assertIsNotNone(self.f.drive.anchor)
        searching = self.f.update(2560, travel=80, reason=1,
            coarse=self.f.missing(2560), target_event=event)
        self.assertTrue(self.f.drive._searching)
        self.assertIsNone(self.f.drive.anchor)
        self.assertEqual(searching['command'], 'FOLLOW,20,-20')
        self.assertFalse(searching['allow_arm'])
        self.assertIsNone(self.f.drive.consume_moving_precise(complete,
            self.f.state(2560, travel=80, reason=1), 2560, 1))
        later = self.f.update(2660, run=True, travel=80,
            coarse=self.f.missing(2660), target_event=event)
        self.assertEqual(later['command'], 'HB')
        self.assertFalse(later['allow_arm'])

    def test_failed_fine_is_consumed_but_cannot_become_an_anchor(self):
        complete = self.moving_complete(status='no_complete_face')
        complete['observation'].pop('cube_center_Base_mm')
        result = self.f.drive.consume_moving_precise(complete,
            self.f.state(1200, run=True, travel=7), 1200, 1)
        self.assertTrue(result['precise_consumed'])
        self.assertIsNone(self.f.drive.anchor)
        self.assertFalse(result['allow_arm'])

    def test_invalid_pair_is_rejected_without_consuming_the_request(self):
        complete = self.moving_complete()
        request = copy.deepcopy(self.f.drive._fine_request)
        complete['paired_coarse']['frame_id'] += 1
        result = self.f.drive.consume_moving_precise(complete,
            self.f.state(1200, run=True, travel=7), 1200, 1)
        self.assertIsNone(result)
        self.assertEqual(self.f.drive._fine_request, request)
        self.assertIsNone(self.f.drive.anchor)


if __name__ == '__main__':
    unittest.main()
