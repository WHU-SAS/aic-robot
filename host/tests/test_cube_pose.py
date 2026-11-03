"""Portable rigid-pose math with known independent3D observations.

These tests use scalar projection and Python's standard library. They neither
import a PC scientific stack nor grant image quality/control acceptance.
"""
import copy
import json
import math
from pathlib import Path
import random
import shutil
import subprocess
import sys
import tempfile
import unittest

from device.k230.vision import cube_pose as pose
from device.k230.pick.numeric import matmul, matvec, rotation, transpose, unit


CANONICAL = [[-10., 0., 20.], [-10., 20., 20.], [10., 20., 20.],
             [10., 0., 0.], [-10., 0., 0.]]
OPPOSITE = [[10., 0., 20.], [10., 20., 20.], [-10., 20., 20.],
            [-10., 0., 0.], [10., 0., 0.]]
K = [[1540., 0., 640.], [0., 1510., 480.], [0., 0., 1.]]
D = [-.18, .045, .0012, -.0017, -.008]


def render(R, t, objects):
    """Independent Brown camera projection of actual rigid3D points."""
    pixels = []
    for point in objects:
        camera = [sum(R[i][j]*point[j] for j in range(3))+t[i]
                  for i in range(3)]
        x, y = camera[0]/camera[2], camera[1]/camera[2]
        k1, k2, p1, p2, k3 = D
        r2 = x*x+y*y
        gain = 1+k1*r2+k2*r2*r2+k3*r2*r2*r2
        u = x*gain+2*p1*x*y+p2*(r2+2*x*x)
        v = y*gain+p1*(r2+2*y*y)+2*p2*x*y
        pixels.append([K[0][0]*u+K[0][2], K[1][1]*v+K[1][2]])
    return pixels


def camera_rotation(yaw, tilt=0.):
    up = unit([0., -.98, -.2])
    ex = [1., 0., 0.]
    nominal = transpose([ex, pose.cross(up, ex), up])
    return matmul(nominal, matmul(rotation([0., 0., 1.], yaw),
                                 rotation([1., 0., 0.], tilt)))


def transformed_center(R, t, center=(0., 10., 10.)):
    return [v+t[i] for i, v in enumerate(matvec(R, center))]


class CubePoseTests(unittest.TestCase):
    def assert_vector_close(self, actual, expected, tolerance):
        self.assertLessEqual(math.sqrt(sum((a-b)**2 for a, b in
            zip(actual, expected))), tolerance)

    def test_known_rigid_poses_both_object_mappings_and_reordered_points(self):
        permutation = [4, 2, 0, 3, 1]
        for yaw in (-1., -.3, .3, 1.):
            for objects in (CANONICAL, OPPOSITE):
                with self.subTest(yaw=yaw, first=objects[0]):
                    R = camera_rotation(yaw, .08)
                    t = [13.*math.sin(yaw), 35., 260.]
                    pixels = render(R, t, objects)
                    before = copy.deepcopy((objects, pixels, K, D))
                    # Table-up is deliberately only a rough seed: the actual
                    # object is tilted and the seed is another3degrees off.
                    up = matvec(rotation([1., 0., 0.], .052),
                                unit([0., -.98, -.2]))
                    solution = pose.solve_cube(objects, pixels, K, D, up)
                    self.assertLess(solution['reprojection_RMS_px'], 1e-6)
                    self.assert_vector_close(solution['t'], t, 1e-5)
                    self.assert_vector_close(solution['cube_center_Camera_mm'],
                        transformed_center(R, t), 1e-5)
                    for actual, expected in zip(solution['R'], R):
                        self.assert_vector_close(actual, expected, 1e-6)
                    reordered = pose.solve_cube([objects[i] for i in permutation],
                        [pixels[i] for i in permutation], K, D, up)
                    self.assert_vector_close(reordered['cube_center_Camera_mm'],
                        solution['cube_center_Camera_mm'], 1e-5)
                    self.assertEqual((objects, pixels, K, D), before)

    def test_opposite_mapping_initialization_has_correct_object_x_sign(self):
        for objects in (CANONICAL, OPPOSITE):
            R = camera_rotation(.45)
            t = [-12., 30., 230.]
            pixels = render(R, t, objects)
            initialized, translation = pose.initialize(objects, pixels, K, D,
                [row[2] for row in R])
            # Check the seed itself: LM must not hide an accidental180degree
            # object-X reversal introduced by image point ordering.
            self.assertGreater(sum(initialized[i][0]*R[i][0]
                                   for i in range(3)), .999999)
            self.assert_vector_close(translation, t, 1e-5)

    def test_light_pixel_noise_recovers_truth_without_a_table_penalty(self):
        rng = random.Random(20261103)
        for yaw in (-.8, .6):
            R = camera_rotation(yaw, .11)
            t = [8., 33., 240.]
            pixels = [[v+rng.gauss(0., .15) for v in point]
                      for point in render(R, t, OPPOSITE)]
            solution = pose.solve_cube(OPPOSITE, pixels, K, D,
                unit([0., -.98, -.2]))
            self.assertLess(solution['reprojection_RMS_px'], .4)
            self.assert_vector_close(solution['cube_center_Camera_mm'],
                transformed_center(R, t), .8)
            # The recovered up follows the tilted object, rather than being
            # forced to equal the supplied nominal table-up direction.
            self.assert_vector_close(solution['up_Camera'],
                [row[2] for row in R], .025)

    def test_generic_objects_non_axis_horizontal_pair_and_explicit_center(self):
        objects = [[-8., -3., 1.], [13., 7., 1.], [-2., 19., 16.],
                   [4., 6., 23.], [-11., 15., 9.], [8., -4., 12.],
                   [0., 8., 4.]]
        R = camera_rotation(-.6, .04)
        t = [14., 35., 320.]
        center = [1., 5., 8.]
        solution = pose.solve_cube(objects, render(R, t, objects), K, D,
            unit([0., -.98, -.2]), center_object_mm=center)
        self.assertLess(solution['reprojection_RMS_px'], 1e-6)
        self.assert_vector_close(solution['cube_center_Camera_mm'],
            transformed_center(R, t, center), 1e-5)

    def test_generic_analytic_jacobian_matches_independent_projection_difference(self):
        objects = CANONICAL+[[1., 7., 9.], [-4., 15., 13.]]
        R = camera_rotation(.4, .1)
        t = [15., 36., 210.]
        columns = pose.jacobian(R, t, objects, K, D)
        for axis, column in enumerate(columns):
            h = 1e-6 if axis < 3 else 1e-4
            delta = [0.]*6
            delta[axis] = h
            plus_R, plus_t = pose.update(R, t, delta)
            plus = render(plus_R, plus_t, objects)
            delta[axis] = -h
            minus_R, minus_t = pose.update(R, t, delta)
            minus = render(minus_R, minus_t, objects)
            numerical = [(a-b)/(2*h) for pa, pb in zip(plus, minus)
                         for a, b in zip(pa, pb)]
            with self.subTest(axis=axis):
                self.assertLess(max(abs(a-b) for a, b in
                    zip(column, numerical)), 1e-5)

    def test_flat_packaging_runs_with_only_scalar_helpers_and_standard_library(self):
        root = Path(__file__).resolve().parents[2]
        with tempfile.TemporaryDirectory(prefix='cube_pose_flat_') as tmp:
            directory = Path(tmp)
            for source in (root/'device/k230/vision/cube_pose.py',
                           root/'device/k230/pick/numeric.py',
                           root/'device/k230/pick/pose.py'):
                shutil.copyfile(source, directory/source.name)
            code = '''import sys,builtins,json
original=builtins.__import__
def checked(name,*args,**kwargs):
    if name.split('.')[0]in('numpy','cv2','scipy','ulab'):raise AssertionError(name)
    return original(name,*args,**kwargs)
builtins.__import__=checked
sys.path[:0]=[sys.argv[1]]
import cube_pose
objects=[[-10.,0.,20.],[-10.,20.,20.],[10.,20.,20.],[10.,0.,0.],[-10.,0.,0.]]
R=[[1.,0.,0.],[0.,0.,-1.],[0.,1.,0.]];t=[2.,30.,220.]
K=[[1000.,0.,640.],[0.,1000.,480.],[0.,0.,1.]]
pixels=[cube_pose.project([a+b for a,b in zip(cube_pose.matvec(R,v),t)],K,[]) for v in objects]
result=cube_pose.solve_cube(objects,pixels,K,[],[0.,-1.,0.])
print(json.dumps({'RMS':result['reprojection_RMS_px'],'center':result['cube_center_Camera_mm']}))
'''
            result = subprocess.run([sys.executable, '-S', '-c', code, tmp],
                cwd=tmp, text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            output = json.loads(result.stdout)
            self.assertLess(output['RMS'], 1e-6)
            self.assert_vector_close(output['center'], [2., 20., 230.], 1e-6)


if __name__ == '__main__':
    unittest.main()
