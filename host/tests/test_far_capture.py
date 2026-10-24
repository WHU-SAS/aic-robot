import ast
import unittest
from device.k230.car.ground_projection import GroundProjection
from tools.diagnostics.capture_car_far import build_script


class FarCaptureTests(unittest.TestCase):
    def test_planar_distance_is_from_camera_foot_not_arm_origin(self):
        g = GroundProjection([[500, 0, 640], [0, 500, 480], [0, 0, 1]],
            [0, 0, 0, 0, 0],
            [[1, 0, 0, 10], [0, -1, 0, 20], [0, 0, -1, 100], [0, 0, 0, 1]],
            {'normal': [0, 0, 1], 'offset_mm': 0})
        self.assertEqual(g.camera_table_foot(), [10, 20, 0])
        self.assertEqual(g.table_distance([310, 420, 0]), 500)
        with self.assertRaises(ValueError): g.table_distance([310, 420, 10])

    def test_camera_foot_uses_table_normal_for_tilted_plane(self):
        g = GroundProjection([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [],
            [[1, 0, 0, 10], [0, 1, 0, 20], [0, 0, 1, 100]],
            {'normal': [0, 1, 1], 'offset_mm': 0})
        for actual, expected in zip(g.camera_table_foot(), [10, -40, 40]):
            self.assertAlmostEqual(actual, expected)

    def test_batch_script_has_one_save_per_prompt_and_no_robot_interface(self):
        script = build_script('/sdcard/car_far/test', 30, 2000)
        tree = ast.parse(script)
        calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)]
        saves = [n for n in calls if isinstance(n.func, ast.Attribute) and n.func.attr == 'save']
        self.assertEqual(len(saves), 1)
        self.assertLess(script.index('CAR_PHOTO_READY '), script.index('image.save('))
        self.assertLess(script.index('image.save('), script.index('CAR_PHOTO_SAVED '))
        self.assertIn('range(1,30+1)', script)
        self.assertIn(')<2000', script)
        for forbidden in ('UART(', 'run_pick', 'run_ready', 'DRIVE', 'STARTUP='):
            self.assertNotIn(forbidden, script)


if __name__ == '__main__': unittest.main()
