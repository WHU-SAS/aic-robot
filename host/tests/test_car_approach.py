"""Coarse vision and stop-observe decisions, independent of camera/motor I/O."""
import copy
import math
import unittest

from device.k230.car.blue_tracker import BlueTracker
from device.k230.car.approach import ApproachPlanner, pulse_for
from device.k230.car.parking_window import bounds_mm
from host.car.runtime import load


class Blob:
    def __init__(self, rect, pixels=None):
        self.box = rect; self.count = pixels if pixels is not None else rect[2] * rect[3]
    def rect(self): return self.box
    def pixels(self): return self.count
    def cx(self): return self.box[0] + (self.box[2] - 1) / 2
    def cy(self): return self.box[1] + (self.box[3] - 1) / 2


class Image:
    def __init__(self, blobs): self.blobs = blobs; self.options = None
    def find_blobs(self, thresholds, **options):
        self.options = options
        return self.blobs


class ApproachTests(unittest.TestCase):
    def setUp(self):
        self.workspace, self.config, self.geometry = load()
        self.planner = ApproachPlanner(self.workspace, self.config)

    def center_y(self, x):
        return sum(bounds_mm(self.workspace, x)['Base_Y_mm']) / 2

    def table_point(self, x, y):
        n = self.geometry.normal
        return [x, y, (self.geometry.offset - n[0] * x - n[1] * y) / n[2]]

    def projected_blob(self, x=750, y=None):
        if y is None: y = self.center_y(x)
        contact = self.table_point(x, y)
        top = [contact[k] + 20 * self.geometry.normal[k] for k in range(3)]
        bottom_px, top_px = self.geometry.base_to_pixel(contact), self.geometry.base_to_pixel(top)
        h = max(3, int(round(bottom_px[1] - top_px[1])) + 1)
        w = max(3, int(round(h * .75)))
        return Blob([int(round(bottom_px[0] - w / 2)), int(round(bottom_px[1] - h + 1)), w, h])

    def coarse(self, error=0, x=750, timestamp=100):
        return {'status': 'tracking', 'timestamp_ms': timestamp,
                'image_midline_error_px': error, 'ground_contact_proxy_Base_mm': [x, 0, 0]}

    def fine(self, x, y, timestamp=100):
        return {'status': 'localized', 'timestamp_ms': timestamp,
                'cube_center_Base_mm': [x, y, -60]}

    def test_table_ray_roundtrip_at_start_and_final_distances(self):
        for x in (250, 500, 750, 1000):
            point = self.table_point(x, self.center_y(x))
            result = self.geometry.pixel_to_plane(self.geometry.base_to_pixel(point))
            self.assertLess(max(abs(a - b) for a, b in zip(point, result)), .01)
        with self.assertRaises(ValueError):
            self.geometry.pixel_to_plane([float('nan'), 500])

    def test_far_cube_uses_full_frame_and_requires_three_observations(self):
        for x in (500, 750, 1000):
            tracker = BlueTracker(self.config['tracking'], self.geometry)
            image = Image([self.projected_blob(x)])
            for k in range(3): result = tracker.detect_frame(image, 100 + k * 50)
            self.assertEqual(result['status'], 'tracking')
            self.assertEqual(image.options['roi'], (0, 0, 1280, 960))
            self.assertLess(abs(result['ground_contact_proxy_Base_mm'][0] - x), 40)
            if x == 1000:
                self.assertLess(result['bbox_px'][1], 432)

    def test_disappearance_does_not_switch_immediately_to_another_blob(self):
        tracker = BlueTracker(self.config['tracking'], self.geometry)
        first = self.projected_blob(750)
        for k in range(3): tracker.detect_frame(Image([first]), k * 50)
        other = self.projected_blob(750, self.center_y(750) + 150)
        result = tracker.detect_frame(Image([other]), 150)
        self.assertEqual(result['status'], 'lost')
        self.assertEqual(tracker.detect_frame(Image([other]), 200)['status'], 'confirming')

    def test_long_gap_and_no_target_reset_confirmation(self):
        tracker = BlueTracker(self.config['tracking'], self.geometry)
        image = Image([self.projected_blob()])
        tracker.detect_frame(image, 0); tracker.detect_frame(image, 50)
        self.assertEqual(tracker.detect_frame(image, 1000)['status'], 'confirming')
        self.assertEqual(tracker.detect_frame(Image([]), 1050)['status'], 'lost')
        self.assertEqual(tracker.detect_frame(Image([]), 1100)['status'], 'no_target')

    def test_clipped_sparse_and_flat_blobs_are_rejected(self):
        tracker = BlueTracker(self.config['tracking'], self.geometry)
        bad = [Blob([0, 500, 40, 40]), Blob([600, 500, 40, 40], 25), Blob([600, 500, 100, 3])]
        result = tracker.detect_frame(Image(bad), 100)
        self.assertEqual(result['status'], 'no_target')
        self.assertEqual(len(result['rejected']), 3)

    def test_coarse_decisions_align_approach_then_request_stationary_localization(self):
        self.assertEqual(self.planner.coarse(self.coarse(100), 100)['intent'], 'turn_right_coarse')
        self.assertEqual(self.planner.coarse(self.coarse(-100), 100)['intent'], 'turn_left_coarse')
        self.assertEqual(self.planner.coarse(self.coarse(), 100)['intent'], 'forward_coarse')
        self.assertEqual(self.planner.coarse(self.coarse(x=300), 100)['intent'], 'localize')
        self.assertEqual(self.planner.coarse(self.coarse(), 100, False)['intent'], 'wait_for_stop')

    def test_stale_or_missing_vision_never_requests_movement(self):
        for observation in (None, {}, self.coarse(timestamp=0), self.coarse(timestamp=1000)):
            self.assertEqual(self.planner.coarse(observation, 600)['intent'], 'stop')
        self.assertEqual(self.planner.fine(self.fine(250, 25, 0), 600)['intent'], 'stop')
        self.assertEqual(self.planner.fine(self.fine(250, 25), 100, False)['intent'], 'stop')

    def test_fine_decisions_use_camera_centered_inset_rectangle(self):
        x0, x1 = bounds_mm(self.workspace)['Base_X_mm']
        x = (x0 + x1) / 2; y = self.center_y(x)
        result = self.planner.fine(self.fine(x, y), 100)
        self.assertEqual(result['intent'], 'forward_fine')
        self.assertFalse(result['automatic_arm_action'])
        near_x = x0 + 2
        self.assertEqual(self.planner.fine(self.fine(near_x, self.center_y(near_x)), 100)['intent'], 'arrived')
        self.assertEqual(self.planner.fine(self.fine(x1 + 5, self.center_y(x1 + 5)), 100)['intent'], 'forward_fine')
        self.assertEqual(self.planner.fine(self.fine(x0 - 5, self.center_y(x0 - 5)), 100)['intent'], 'reverse_fine')
        self.assertEqual(self.planner.fine(self.fine(x, y + 30), 100)['intent'], 'turn_left_fine')
        self.assertEqual(self.planner.fine(self.fine(x, y - 30), 100)['intent'], 'turn_right_fine')

    def test_unselected_or_wrong_direction_pulses_are_not_issued(self):
        plan = self.planner.coarse(self.coarse(), 100)
        with self.assertRaisesRegex(ValueError, 'not selected'):
            pulse_for(plan, self.config['motion_profiles'])
        profiles = {'forward_coarse': {'left_pwm': -800, 'right_pwm': 800, 'duration_ms': 300}}
        with self.assertRaisesRegex(ValueError, 'signs disagree'): pulse_for(plan, profiles)
        profiles['forward_coarse']['left_pwm'] = 800
        self.assertEqual(pulse_for(plan, profiles), (800, 800, 300))
        self.assertIsNone(pulse_for({'intent': 'arrived'}, {}))


if __name__ == '__main__': unittest.main()
