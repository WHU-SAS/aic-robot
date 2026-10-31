"""A coordinate reexpression keeps the selected physical parking region.

Independent rigid transforms compare the retained old calibration/window with
the configuration-only approximation, rather than accepting a second survey.
"""
import hashlib
import json
import math
import unittest

from device.k230.car.ground_projection import GroundProjection
from device.k230.car.near_control import target_x
from device.k230.car.parking_window import bounds_mm, evaluate, polygon_mm
from host.project_paths import ROOT


def read(path):
    return json.loads((ROOT / path).read_text())


class CalibrationPairingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.old = read('configs/car/workspace_definition.json')
        cls.approach = read('configs/car/approach.json')
        cls.new = read(cls.approach['workspace_config'])
        cls.policy = read('configs/car/near_parking.json')
        cls.old_vision = read('configs/vision/trials/fixed_tip_top_20261022.json')
        cls.new_vision = read(cls.approach['vision_config'])
        cls.old_loc = cls.old_vision['localization']
        cls.new_loc = cls.new_vision['localization']
        cls.old_ext = read(cls.old_loc['extrinsics'])
        cls.new_ext = read(cls.new_loc['extrinsics'])
        cls.old_T = cls.old_ext['T_Base_from_Camera_mm']
        cls.new_T = cls.new_ext['T_Base_from_Camera_mm']
        intr = read(cls.new_loc['intrinsics'])
        cls.old_projection = GroundProjection(intr['camera_matrix'],
            intr['distortion_coefficients'], cls.old_T, cls.old_loc['table_plane_Base_mm'])
        cls.new_projection = GroundProjection(intr['camera_matrix'],
            intr['distortion_coefficients'], cls.new_T, cls.new_loc['table_plane_Base_mm'])

    def point(self, x, y, loc, height=10):
        plane = loc['table_plane_Base_mm']
        n = plane['normal']
        z = (plane['offset_mm'] + height * math.sqrt(sum(v*v for v in n))
             - n[0]*x - n[1]*y) / n[2]
        return [x, y, z]

    def mapped(self, old_point):
        # Go via Camera using the rigid inverse; do not use the saved M.
        camera = [sum(self.old_T[k][j] * (old_point[k]-self.old_T[k][3])
                      for k in range(3)) for j in range(3)]
        return [sum(row[j]*camera[j] for j in range(3)) + row[3]
                for row in self.new_T[:3]]

    def center(self, config, x, loc):
        line = config['centerline']['local_Base_Y_line']
        return self.point(x, line['slope_per_X']*x+line['intercept_mm'], loc)

    def test_new_pair_is_explicit_and_historical_calibration_is_retained(self):
        self.assertEqual(self.approach['vision_config'],
                         'configs/vision/trials/model_alignment_20261030.json')
        self.assertEqual(self.old['centerline']['local_Base_Y_line']['vision_config'],
                         'configs/vision/trials/fixed_tip_top_20261022.json')
        provenance = self.new['coordinate_reexpression']
        self.assertEqual(provenance['source_workspace_sha256'], hashlib.sha256(
            (ROOT/provenance['source_workspace']).read_bytes()).hexdigest())
        line = self.new['centerline']['local_Base_Y_line']
        for key in ('intrinsics', 'extrinsics'):
            self.assertEqual(line[key+'_sha256'], hashlib.sha256(
                (ROOT/self.new_loc[key]).read_bytes()).hexdigest())
        self.assertFalse(provenance['physical_field_revalidated'])

    def test_inset_and_allowed_intervals_are_not_applied_twice_or_narrowed(self):
        for key in ('longitudinal', 'lateral'):
            self.assertEqual(self.new['normalized_window'][key]['allowed'],
                             self.old['normalized_window'][key]['allowed'])
        self.assertEqual(self.new['normalized_window']['inset_mm'], 10)
        self.assertEqual(self.new['coordinate_reexpression']['inset_applied_times'], 1)
        old_x = bounds_mm(self.old)['Base_X_mm']
        expected_x = [self.mapped(self.center(self.old, x, self.old_loc))[0]
                      for x in old_x]
        actual_x = bounds_mm(self.new)['Base_X_mm']
        for actual, expected in zip(actual_x, expected_x):
            self.assertAlmostEqual(actual, expected, places=9)
        for x, y in polygon_mm(self.new):
            self.assertTrue(evaluate(x, y, self.new)['inside'])

    def test_axis_approximation_records_its_actual_boundary_displacement(self):
        mapped = [self.mapped(self.point(x, y, self.old_loc))
                  for x, y in polygon_mm(self.old)]
        current = polygon_mm(self.new)
        x_error = max(abs(a[0]-b[0]) for a, b in zip(mapped, current))
        corner_error = max(math.hypot(a[0]-b[0], a[1]-b[1])
                           for a, b in zip(mapped, current))
        self.assertLess(x_error, 1.01)
        self.assertGreater(x_error, 1.0)
        self.assertAlmostEqual(x_error, self.new['coordinate_reexpression'][
            'max_near_far_edge_X_deviation_mm'], places=9)
        self.assertAlmostEqual(corner_error, self.new['coordinate_reexpression'][
            'max_corresponding_corner_XY_deviation_mm'], places=9)
        # Measure the transformed left/right side edges at the same new X.
        # Opposite corner Y differences are not the width of this slanted cap.
        mid_x = sum(bounds_mm(self.new)['Base_X_mm'])/2
        def at_x(a, b):
            return a[1]+(mid_x-a[0])*(b[1]-a[1])/(b[0]-a[0])
        width = at_x(mapped[0], mapped[3])-at_x(mapped[1], mapped[2])
        limits = bounds_mm(self.new, mid_x)['Base_Y_mm']
        self.assertAlmostEqual(limits[1]-limits[0], width, places=9)

    def test_target_preserves_near_quarter_choice_and_camera_midline(self):
        self.assertEqual(self.approach['preferred_stop_fraction_of_inset_depth'], .25)
        expected = self.mapped(self.center(self.old, target_x(self.old), self.old_loc))
        actual = self.center(self.new, target_x(self.new), self.new_loc)
        self.assertAlmostEqual(actual[0], expected[0], places=9)
        self.assertLess(abs(actual[1]-expected[1]), .001)
        self.assertAlmostEqual(actual[0], 248.22224411468, places=8)
        lo, hi = bounds_mm(self.new)['Base_X_mm']
        for i in range(21):
            p = self.center(self.new, lo+(hi-lo)*i/20, self.new_loc)
            self.assertLess(abs(self.new_projection.base_to_pixel(p)[0]-640), .001)

    def test_coarse_triggers_keep_the_image_ray_on_the_new_runtime_table(self):
        pairing = self.approach['coordinate_pairing']
        before = [pairing['source_near_localization_start_Base_X_mm'],
                  *pairing['source_tracking_forward_range_mm'], 500]
        after = [self.approach['near_localization_start_Base_X_mm'],
                 *self.approach['tracking']['forward_range_mm'],
                 self.policy['capture_Base_X_mm']]
        for x, actual in zip(before, after):
            low, high = -1000., 1000.
            for _ in range(70):
                y = (low+high)/2
                p = self.point(x, y, self.old_loc, height=0)
                if self.old_projection.base_to_pixel(p)[0] > 640:
                    low = y
                else:
                    high = y
            original = self.point(x, (low+high)/2, self.old_loc, height=0)
            moved = self.mapped(original)
            # Changing Base coordinates leaves the same physical ray unchanged.
            pixel = self.new_projection.base_to_pixel(moved)
            old_pixel = self.old_projection.base_to_pixel(original)
            for a, b in zip(pixel, old_pixel):
                self.assertAlmostEqual(a, b, places=8)
            # The newly fitted table differs slightly from M(old table).
            # The running proxy is the same ray's intersection with NEW table,
            # not merely the transformed old contact point.
            proxy = self.new_projection.pixel_to_plane(old_pixel, height_mm=0)
            self.assertAlmostEqual(actual, proxy[0], places=8)
            plane = self.new_loc['table_plane_Base_mm']
            self.assertAlmostEqual(sum(a*b for a,b in zip(plane['normal'], proxy)),
                                   plane['offset_mm'], places=8)

    def test_old_motion_axis_and_origin_are_explicitly_rotated_into_new_base(self):
        expected_origin = self.mapped([0, 0, 0])
        old_x_tip = self.mapped([1, 0, 0])
        expected_axis = [old_x_tip[k]-expected_origin[k] for k in range(3)]
        for actual, expected in zip(self.approach['forward_axis_Base'], expected_axis):
            self.assertAlmostEqual(actual, expected, places=12)
        for actual, expected in zip(self.approach['motion_origin_Base_mm'], expected_origin):
            self.assertAlmostEqual(actual, expected, places=12)


if __name__ == '__main__':
    unittest.main()
