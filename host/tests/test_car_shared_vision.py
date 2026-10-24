import copy
import unittest

from device.k230.car.approach import ApproachPlanner
from device.k230.car.observations import for_approach
from device.k230.vision.runtime import localize_precise, track_coarse
from device.k230.car.parking_window import bounds_mm
from host.car.runtime import load


class CarSharedVisionTests(unittest.TestCase):
    def setUp(self):
        self.workspace, self.config, _ = load()
        self.planner = ApproachPlanner(self.workspace, self.config)
        self.frame = {'frame_id': 1, 'capture_timestamp_ms': 100,
                      'clock_domain': 'k230_ticks_ms', 'image_size_px': [1280, 960],
                      'timestamp_source': 'software_snapshot_call'}

    def coarse(self):
        return track_coarse(None, self.frame, lambda image, stamp:
            {'timestamp_ms': stamp, 'status': 'tracking', 'image_midline_error_px': 0,
             'ground_contact_proxy_Base_mm': [750, 50, -70]})

    def precise(self, success=True):
        x = bounds_mm(self.workspace)['Base_X_mm'][0] + 2
        y = sum(bounds_mm(self.workspace, x)['Base_Y_mm']) / 2
        intr = {'camera_matrix': [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                'image_width': 1280, 'image_height': 960, 'distortion_coefficients': []}
        ext = {'T_Base_from_Camera_mm': [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]]}
        raw = {'status': 'complete_face_candidate' if success else 'no_complete_face',
               'cube_center_Base_mm': [x, y, -60], 'poses': [{'face_reprojection_rms_px': 1}]}
        return localize_precise(None, self.frame, intr, ext, localizer=lambda *args: raw)

    def test_shared_coarse_and_precise_are_consumed_without_changing_policy(self):
        self.assertEqual(self.planner.coarse(self.coarse(), 100)['intent'], 'forward_coarse')
        self.assertEqual(self.planner.fine(self.precise(), 100)['intent'], 'arrived')
        self.assertEqual(self.planner.fine(self.precise(False), 100)['intent'], 'stop')

    def test_modes_and_coordinate_meanings_cannot_be_interchanged(self):
        self.assertEqual(self.planner.fine(self.coarse(), 100)['intent'], 'stop')
        self.assertEqual(self.planner.coarse(self.precise(), 100)['intent'], 'stop')
        for key, value in [('units', 'm'), ('coordinate_frame', 'Camera'),
                           ('reference_point', 'bottom_center'), ('xy_compensation_applied', True)]:
            result = self.precise(); result[key] = value
            self.assertEqual(self.planner.fine(result, 100)['intent'], 'stop', key)

    def test_unknown_stale_future_or_wrong_clock_does_not_become_fresh(self):
        original = self.precise()
        for stamp, now in [(None, 100), (0, 1000), (1000, 100)]:
            result = copy.deepcopy(original)
            result['capture_timestamp_ms'] = result['timestamp_ms'] = stamp
            self.assertEqual(self.planner.fine(result, now)['intent'], 'stop')
        for patch in [{'clock_domain': 'pc_monotonic_ms'}, {'timestamp_ms': 101},
                      {'image_size_px': [640, 480]}, {'frame_id': None}]:
            result = dict(original, **patch)
            self.assertEqual(self.planner.fine(result, 100)['intent'], 'stop')
        self.assertEqual(original['capture_timestamp_ms'], 100)

    def test_lost_target_drops_selected_geometry_and_never_uses_payload(self):
        result = self.coarse(); result['status'] = 'lost'
        normalized = for_approach(result, 'coarse')
        self.assertNotIn('ground_contact_proxy_Base_mm', normalized)
        self.assertEqual(self.planner.coarse(result, 100)['intent'], 'stop')
        precise = self.precise(); precise['status'] = 'no_complete_face'
        self.assertIn('cube_center_Base_mm', precise['payload'])
        self.assertNotIn('cube_center_Base_mm', for_approach(precise, 'precise'))
        self.assertEqual(self.planner.fine(precise, 100)['intent'], 'stop')


if __name__ == '__main__': unittest.main()
