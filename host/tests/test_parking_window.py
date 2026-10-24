"""Parking decisions preserve the user's bounds and reversed lateral sign."""
import copy
import json
from pathlib import Path
import unittest

from device.k230.car.parking_window import bounds_mm, evaluate, polygon_mm


ROOT = Path(__file__).resolve().parents[2]


class ParkingWindowTests(unittest.TestCase):
    def setUp(self):
        self.config = {'normalized_window': {
            'longitudinal': {'near_Base_X_mm': 200., 'far_Base_X_mm': 300.,
                             'allowed': [0., .5]},
            'lateral': {'left_Base_Y_mm': 80., 'right_Base_Y_mm': -40.,
                        'allowed': [-.5, .5]},
        }}

    def test_reference_endpoints_and_no_clipping(self):
        self.assertEqual(evaluate(200, 80, self.config)['lateral_normalized'], -1)
        self.assertEqual(evaluate(300, -40, self.config)['longitudinal_normalized'], 1)
        self.assertEqual(evaluate(300, -40, self.config)['lateral_normalized'], 1)
        for x, y in ((190, 20), (310, 20), (220, 90), (220, -50)):
            self.assertFalse(evaluate(x, y, self.config)['inside'])
        self.assertGreater(evaluate(310, 20, self.config)['longitudinal_normalized'], 1)
        self.assertLess(evaluate(220, 90, self.config)['lateral_normalized'], -1)

    def test_all_four_allowed_corners_are_inclusive(self):
        self.assertEqual(bounds_mm(self.config),
                         {'Base_X_mm': [200., 250.], 'Base_Y_mm': [-10., 50.]})
        for x in (200, 250):
            for y in (-10, 50):
                self.assertTrue(evaluate(x, y, self.config)['inside'])
        for x, y in ((199.999, 20), (250.001, 20), (225, -10.001), (225, 50.001)):
            self.assertFalse(evaluate(x, y, self.config)['inside'])

    def test_base_centerline_is_not_the_lateral_interval_midpoint(self):
        point = evaluate(225, 0, self.config)
        self.assertTrue(point['inside'])
        self.assertAlmostEqual(point['lateral_normalized'], 1 / 3)
        self.assertEqual(point['centerline_error_mm'], 0)
        self.assertEqual(evaluate(225, 20, self.config)['lateral_normalized'], 0)

    def test_ten_mm_inset_contains_the_cube_footprint_at_every_corner(self):
        self.config['normalized_window']['inset_mm'] = 10
        limits = bounds_mm(self.config)
        self.assertEqual(limits, {'Base_X_mm': [210., 240.], 'Base_Y_mm': [0., 40.]})
        for x in limits['Base_X_mm']:
            for y in limits['Base_Y_mm']:
                self.assertTrue(evaluate(x, y, self.config)['inside'])
                self.assertTrue(200 <= x - 10 <= x + 10 <= 250)
                self.assertTrue(-10 <= y - 10 <= y + 10 <= 50)
        for x, y in ((209.999, 20), (240.001, 20), (225, -.001), (225, 40.001)):
            self.assertFalse(evaluate(x, y, self.config)['inside'])
        before = copy.deepcopy(self.config)
        before['normalized_window']['inset_mm'] = 0
        self.assertEqual(evaluate(205, 20, self.config)['longitudinal_normalized'],
                         evaluate(205, 20, before)['longitudinal_normalized'])
        self.assertTrue(evaluate(205, 20, before)['inside'])
        self.assertFalse(evaluate(205, 20, self.config)['inside'])

    def test_invalid_or_oversized_inset_is_rejected(self):
        for inset in (-1, float('nan'), float('inf'), 26):
            self.config['normalized_window']['inset_mm'] = inset
            with self.assertRaises(ValueError):
                evaluate(225, 20, self.config)

    def test_invalid_measurements_and_degenerate_reference_are_rejected(self):
        for value in (float('nan'), float('inf'), None, True):
            with self.assertRaises(ValueError):
                evaluate(value, 0, self.config)
            with self.assertRaises(ValueError):
                evaluate(225, value, self.config)
        broken = copy.deepcopy(self.config)
        broken['normalized_window']['lateral']['right_Base_Y_mm'] = 80
        with self.assertRaises(ValueError):
            evaluate(225, 0, broken)

    def test_camera_centered_window_follows_the_image_line_with_depth(self):
        self.config['centerline'] = {
            'image_midline_is_final_centering_reference': True,
            'local_Base_Y_line': {'slope_per_X': .1, 'intercept_mm': 1.},
        }
        self.config['normalized_window']['inset_mm'] = 10
        for x in (210., 225., 240.):
            y = .1 * x + 1
            result = evaluate(x, y, self.config)
            self.assertAlmostEqual(result['lateral_normalized'], 0)
            self.assertAlmostEqual(result['centerline_error_mm'], 0)
            self.assertTrue(result['inside'])
            self.assertEqual(bounds_mm(self.config, x)['Base_Y_mm'], [y - 20, y + 20])
        for x, y in polygon_mm(self.config):
            self.assertTrue(evaluate(x, y, self.config)['inside'])
        self.assertFalse(evaluate(225, .1 * 225 + 1 + 20.001, self.config)['inside'])

    def test_actual_window_preserves_inset_even_when_it_excludes_base_centerline(self):
        config = json.loads((ROOT / 'configs/car/workspace_definition.json').read_text())
        limits = bounds_mm(config)
        x = sum(limits['Base_X_mm']) / 2
        self.assertEqual(config['normalized_window']['inset_mm'], 10)
        self.assertFalse(evaluate(x, 0, config)['inside'])
        self.assertTrue(evaluate(x, sum(limits['Base_Y_mm']) / 2, config)['inside'])
        for x, y in polygon_mm(config):
            self.assertTrue(evaluate(x, y, config)['inside'])

    def test_actual_camera_target_projects_to_image_midline(self):
        from device.k230.pick.pose import project
        config = json.loads((ROOT / 'configs/car/workspace_definition.json').read_text())
        model = config['centerline']['local_Base_Y_line']
        vision = json.loads((ROOT / model['vision_config']).read_text())['localization']
        intr = json.loads((ROOT / vision['intrinsics']).read_text())
        ext = json.loads((ROOT / vision['extrinsics']).read_text())
        plane = vision['table_plane_Base_mm']; n = plane['normal']
        height = model['cube_center_height_above_table_mm']
        offset = plane['offset_mm'] + height * sum(v * v for v in n) ** .5
        T = ext['T_Camera_from_Base_mm']
        x0, x1 = bounds_mm(config)['Base_X_mm']
        for x in (x0, (x0 + x1) / 2, x1):
            y = model['slope_per_X'] * x + model['intercept_mm']
            z = (offset - n[0] * x - n[1] * y) / n[2]
            cam = [sum(row[k] * v for k, v in enumerate((x, y, z))) + row[3]
                   for row in T[:3]]
            pixel = project(cam, intr['camera_matrix'], intr['distortion_coefficients'])
            self.assertLess(abs(pixel[0] - config['centerline']['image_midline_x_px']), .01)


if __name__ == '__main__':
    unittest.main()
