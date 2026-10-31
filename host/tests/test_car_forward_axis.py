"""The nominal encoder model is unchanged by a paired rigid Base-frame change."""
import ast
import json
from pathlib import Path
import unittest

import numpy as np

from device.k230.car.continuous_approach import midline_heading
from device.k230.car.continuous_math import (forward_x_per_mm, predict_bearing,
                                            table_forward)
from device.k230.car.ground_projection import GroundProjection
from device.k230.car.near_control import correction_plan, predict_center
from device.k230.car.visual_align import motion_plan
from host.car.runtime import load

ROOT = Path(__file__).resolve().parents[2]


class ForwardAxisTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        folder = ROOT / 'configs/vision/trials'
        cls.old_vision = json.loads((folder / 'fixed_tip_top_20261022.json').read_text())
        cls.new_vision = json.loads((folder / 'model_alignment_20261030.json').read_text())
        cls.old_T = np.array(json.loads((folder / 'fixed_tip_top_20261022_extrinsics.json').read_text())['T_Base_from_Camera_mm'])
        cls.new_T = np.array(json.loads((folder / 'model_alignment_20261030_extrinsics.json').read_text())['T_Base_from_Camera_mm'])
        cls.M = cls.new_T @ np.linalg.inv(cls.old_T)
        cls.axis, cls.origin = cls.M[:3, 0].tolist(), cls.M[:3, 3].tolist()
        old_plane = cls.old_vision['localization']['table_plane_Base_mm']
        mapped_normal = cls.M[:3, :3] @ old_plane['normal']
        cls.mapped_plane = {'normal': mapped_normal.tolist(),
            'offset_mm': old_plane['offset_mm'] + float(mapped_normal @ cls.M[:3, 3])}
        intrinsics = json.loads((ROOT / cls.old_vision['localization']['intrinsics']).read_text())
        K, D = intrinsics['camera_matrix'], intrinsics['distortion_coefficients']
        cls.old = GroundProjection(K, D, cls.old_T.tolist(), old_plane)
        cls.new = GroundProjection(K, D, cls.new_T.tolist(), cls.mapped_plane, cls.axis, cls.origin)
        cls.refit = GroundProjection(K, D, cls.new_T.tolist(),
            cls.new_vision['localization']['table_plane_Base_mm'], cls.axis, cls.origin)

    def mapped(self, point):
        return (self.M @ np.array(list(point)+[1.]))[:3].tolist()

    def test_planar_forward_direction_and_host_configuration_are_paired(self):
        expected = self.M[:3, :3] @ table_forward(self.old.normal)
        np.testing.assert_allclose(table_forward(self.new.normal, self.axis), expected, atol=1e-12)
        _, config, projection = load()
        np.testing.assert_allclose(projection.forward_axis_Base, config['forward_axis_Base'], atol=1e-14)
        np.testing.assert_allclose(projection.motion_origin_Base_mm, config['motion_origin_Base_mm'], atol=1e-14)

    def test_bundle_and_voice_constructor_keep_the_paired_motion_frame(self):
        from tools.deploy.k230_car_align import build_bundle
        config, files, _, _ = build_bundle()
        approach = json.loads((ROOT / 'configs/car/approach.json').read_text())
        for key in ('forward_axis_Base', 'motion_origin_Base_mm'):
            self.assertEqual(config['projection'][key], approach[key])
        constructor = next(node for node in ast.walk(ast.parse(files['align_voice.py']))
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
            and node.func.id == 'GroundProjection')
        self.assertEqual(len(constructor.args), 6)
        for argument, key in zip(constructor.args[-2:], ('forward_axis_Base', 'motion_origin_Base_mm')):
            self.assertIsInstance(argument, ast.Call)
            self.assertEqual((argument.func.value.id, argument.func.attr), ('model', 'get'))
            self.assertEqual(argument.args[0].value, key)

    def test_straight_reverse_and_curved_prediction_and_pixels_are_covariant(self):
        for point in ([300, 25, -27], [500, -40, -16]):
            for deltas in ([(20, 20)], [(-12, -12)], [(3, 9), (8, 5)], [(-3, 3)]):
                with self.subTest(point=point, deltas=deltas):
                    old = predict_center(point, deltas, 119.2, self.old.normal)
                    new = predict_center(self.mapped(point), deltas, 119.2, self.new.normal,
                                         self.axis, self.origin)
                    np.testing.assert_allclose(new, self.mapped(old), atol=1e-10)
                    np.testing.assert_allclose(self.new.base_to_pixel(new),
                                               self.old.base_to_pixel(old), atol=1e-9)

    def test_translation_origin_is_required_for_arc_covariance(self):
        point, deltas = [300, 25, -27], [(3, 20)]
        expected = self.mapped(predict_center(point, deltas, 119.2, self.old.normal))
        wrong = predict_center(self.mapped(point), deltas, 119.2, self.new.normal, self.axis)
        self.assertGreater(np.linalg.norm(np.array(wrong)-expected), .5)

    def test_coarse_heading_prediction_uses_declared_forward_not_new_Base_X(self):
        wrong = GroundProjection(self.new.K, self.new.D, self.new.T, self.mapped_plane)
        for row in (500, 700):
            old_heading = midline_heading(self.old, row, 1280)
            new_heading = midline_heading(self.new, row, 1280)
            self.assertAlmostEqual(new_heading, old_heading, places=10)
            self.assertGreater(abs(midline_heading(wrong, row, 1280)-old_heading), 3.)
            self.assertAlmostEqual(predict_bearing(12, 500, old_heading, 2, 5, 119.2),
                                   predict_bearing(12, 500, new_heading, 2, 5, 119.2), places=10)

    def test_MOVE_distance_divides_by_projected_forward_x(self):
        scale = forward_x_per_mm(self.new)
        self.assertAlmostEqual(scale, table_forward(self.new.normal, self.axis)[0])
        for distance in (-30, 30):
            verdict = {'status': 'not_parked', 'accepted': False,
                'window': {'lateral_allowed': True}, 'target_Base_X_mm': 250,
                'cube_center_Base_mm': [250+distance*scale, 0, 0],
                'forward_X_per_mm': scale}
            self.assertEqual(correction_plan(verdict)['distance_mm'], distance)
        # Historical calls without the new axis retain their old X/mm shortcut.
        self.assertEqual(forward_x_per_mm(self.old), 1.)

    def test_single_step_MOVE_also_uses_nominal_wheel_distance(self):
        from tools.deploy.k230_car_align import build_bundle
        config, _, _, _ = build_bundle(approach_mode=True)
        _, _, projection = load()
        model = config['projection']
        np.testing.assert_allclose(model['forward_axis_Base'], projection.forward_axis_Base, atol=1e-14)
        np.testing.assert_allclose(model['motion_origin_Base_mm'], projection.motion_origin_Base_mm, atol=1e-14)
        restored = GroundProjection(model['K'], model['D'], model['T'], model['plane'],
                                    model.get('forward_axis_Base'), model.get('motion_origin_Base_mm'))
        self.assertAlmostEqual(midline_heading(restored, 650, 1280),
                               midline_heading(projection, 650, 1280), places=12)
        scale = forward_x_per_mm(projection)
        observation = {'schema': 'aic.vision_observation.v1', 'mode': 'coarse',
            'status': 'tracking', 'frame_id': 1, 'capture_timestamp_ms': 100,
            'clock_domain': 'k230_ticks_ms', 'image_size_px': [1280, 960],
            'center_pixel': [640, 650],
            'ground_contact_proxy_Base_mm': [config['forward']['near_Base_X_mm']+50.9*scale, 0, 0]}
        self.assertEqual(motion_plan(observation, projection, config, 100)['distance_mm'], 50)

    def test_refitted_plane_is_an_approximation_separate_from_rigid_covariance(self):
        self.assertGreater(np.linalg.norm(np.array(self.refit.normal)-self.new.normal), 1e-5)
        point, deltas = self.mapped([300, 25, -27]), [(10, 30), (8, 8)]
        exact = predict_center(point, deltas, 119.2, self.new.normal, self.axis, self.origin)
        refit = predict_center(point, deltas, 119.2, self.refit.normal, self.axis, self.origin)
        # These numeric bounds describe this calibration pair, not physical accuracy.
        self.assertLess(np.linalg.norm(np.array(refit)-exact), .1)
        self.assertLess(np.linalg.norm(np.array(self.refit.base_to_pixel(refit))-
                                       self.new.base_to_pixel(exact)), 1.)


if __name__ == '__main__':
    unittest.main()
