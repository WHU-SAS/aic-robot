import ast
import base64
import hashlib
import unittest
from device.k230.car.ground_projection import GroundProjection
from tools.diagnostics.capture_car_far import build_script, download
from host.car.distance_demo import visible_table_path
from host.car.runtime import load


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

    def test_download_reports_progress_and_verifies_size_and_hash(self):
        content = b'opaque JPEG transfer payload'
        class Link:
            def __init__(self, size_delta=0): self.size_delta = size_delta
            def execute(self, script, timeout):
                ast.parse(script)
                self.script = script
                yield ('SIZE %d' % (len(content) + self.size_delta)).encode()
                yield b'DATA 0 ' + base64.b64encode(content)
                yield ('HASH %d %s' % (len(content), hashlib.sha256(content).hexdigest())).encode()
        progress = []
        actual, digest = download(Link(), '/sdcard/car_far/test/frame_01.jpg',
                                  lambda received, total: progress.append((received, total)))
        self.assertEqual(actual, content)
        self.assertEqual(digest, hashlib.sha256(content).hexdigest())
        self.assertEqual(progress, [(0, len(content)), (len(content), len(content))])
        held = Link()
        with_setup, same_digest = download(held, '/sdcard/car_far/test/frame_01.jpg',
                                          script_prefix='light_is_off = True\n')
        self.assertEqual(with_setup, content)
        self.assertEqual(same_digest, digest)
        self.assertTrue(held.script.startswith('light_is_off = True\n'))
        with self.assertRaises(RuntimeError):
            download(Link(1), '/sdcard/car_far/test/frame_01.jpg')

    def test_offscreen_camera_foot_is_clipped_without_huge_distortion_coordinates(self):
        _, _, geometry = load()
        bottom = geometry.estimated_bottom_from_image_center([708, 285])
        points, outside = visible_table_path(geometry, bottom, geometry.camera_table_foot(), 1280, 960)
        self.assertTrue(outside)
        self.assertGreater(len(points), 2)
        for x, y in points:
            self.assertTrue(-.001 <= x <= 1279.001 and -.001 <= y <= 959.001)
        self.assertAlmostEqual(points[0][1], 959)


if __name__ == '__main__': unittest.main()
