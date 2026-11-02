"""Distance ambiguity and use boundaries, independent of image proposal search."""
import copy
import importlib.util
import subprocess
import unittest
from host.project_paths import ROOT

HAS_VISION = importlib.util.find_spec('cv2') is not None
if HAS_VISION:
    from device.k230.pick.table_pose import context, constrain_face
    from device.k230.vision.side_range import range_candidate


@unittest.skipUnless(HAS_VISION, 'Dedicated vision environment')
class SideRangeTests(unittest.TestCase):
    def setUp(self):
        self.K = [[100., 0., 640.], [0., 100., 480.], [0., 0., 1.]]
        self.T = [[1., 0., 0., 0.], [0., 1., 0., 0.],
                  [0., 0., 1., 0.], [0., 0., 0., 1.]]
        self.table = context(self.T, {'normal': [0., 0., 1.], 'offset_mm': 0.})

    def pose(self, x=400., z=26., normal=None, rms=.3):
        return {'cube_center_Camera_mm': [x, 0., z],
                'inward_face_normal_Camera': normal or [1., 0., 0.],
                'face_reprojection_rms_px': rms}

    def face(self, poses, adjacent=False):
        x = 200. if adjacent else 100.
        return {'face_quad_px': [[x, 100.], [x+100., 100.],
                                 [x+100., 200.], [x, 200.]],
                'area_px2': 10000., 'poses': poses,
                'physical_edges': [{'support': 64, 'sampled_stations': 64,
                                    'rms_px': .3} for unused in range(4)]}

    def solve(self, faces):
        return range_candidate(faces, self.K, [], self.T, self.table)

    def test_failed_table_height_can_supply_distance_without_changing_free_pose(self):
        face = self.face([self.pose()])
        before = copy.deepcopy(face)
        self.assertIsNone(constrain_face(face, self.table))
        result = self.solve([face])
        self.assertEqual(result['status'], 'range_candidate')
        self.assertTrue(result['not_arm_or_parking_authority'])
        self.assertNotIn('cube_center_Base_mm', result)
        self.assertEqual(result['Base_X_mm'], 400.)
        self.assertEqual(result['raw_center_Base_mm'], [400., 0., 26.])
        self.assertFalse(result['audit'][0]['poses'][0]['table_geometry']['accepted'])
        self.assertEqual(face, before)

    def test_height_does_not_choose_distance_and_interval_keeps_other_valid_solution(self):
        face = self.face([self.pose(), self.pose(402., 10., normal=[.995, 0., .1])])
        self.assertEqual(constrain_face(face, self.table)['poses'][0]['cube_center_Camera_mm'][0], 402.)
        result = self.solve([face])
        self.assertEqual(result['Base_X_mm'], 400.)
        self.assertEqual(result['X_interval_mm'], [400., 402.])

    def test_adjacent_faces_must_agree_in_full_center_even_if_X_matches(self):
        first = self.face([self.pose()])
        second = self.face([self.pose(z=45., normal=[0., 1., 0.])], adjacent=True)
        with self.assertRaisesRegex(ValueError, 'centers or normals disagree'):
            self.solve([first, second])

    def test_pair_range_interval_includes_all_compatible_averages(self):
        first = self.face([self.pose(400.), self.pose(404.)])
        second = self.face([self.pose(402., normal=[0., 1., 0.]),
                            self.pose(406., normal=[0., 1., 0.])], adjacent=True)
        result = self.solve([first, second])
        self.assertEqual(result['side_selection'], 'mean_of_similar_area_sides')
        self.assertEqual(result['Base_X_mm'], 401.)
        self.assertEqual(result['X_interval_mm'], [401., 405.])

    def projected_pose(self, x, tag, **options):
        pose = self.pose(x, **options)
        pose.update(cube_projected_corners_px=[[i, tag] for i in range(8)],
                    cube_projection_source='pose_projected_cube_corners')
        return pose

    def test_single_selected_face_keeps_legacy_pose_without_projection(self):
        face = self.face([self.pose()])
        result = self.solve([face])
        self.assertEqual(len(result['selected_faces']), 1)
        selected = result['selected_faces'][0]
        self.assertTrue(selected['used_for_center'])
        self.assertEqual(selected['face_quad_px'], face['face_quad_px'])
        self.assertEqual(selected['selected_pose']['cube_center_Camera_mm'],
                         face['poses'][0]['cube_center_Camera_mm'])
        self.assertNotIn('cube_projected_corners_px', selected['selected_pose'])
        self.assertEqual(result['Base_X_mm'], 400.)
        self.assertEqual(result['X_interval_mm'], [400., 400.])
        self.assertEqual(len(result['audit']), 1)

    def test_larger_side_retains_selected_pair_pose_and_excludes_unused_side(self):
        first = self.face([self.projected_pose(400., 1),
                           self.projected_pose(410., 2, rms=.4)])
        first['area_px2'] = 15000.
        second = self.face([self.projected_pose(412., 3, normal=[0., 1., 0.])],
                           adjacent=True)
        before = copy.deepcopy([first, second])
        result = self.solve([first, second])
        self.assertEqual(result['side_selection'], 'larger_verified_side')
        self.assertEqual(result['Base_X_mm'], 410.)
        self.assertEqual(result['X_interval_mm'], [410., 410.])
        self.assertEqual(len(result['selected_faces']), 1)
        used = result['selected_faces'][0]
        self.assertEqual(used['face_quad_px'], first['face_quad_px'])
        self.assertTrue(used['used_for_center'])
        self.assertEqual(used['selected_pose']['original_pose_rank'], 1)
        self.assertEqual(used['selected_pose']['cube_projected_corners_px'],
                         first['poses'][1]['cube_projected_corners_px'])
        self.assertEqual(used['selected_pose']['cube_projection_source'],
                         first['poses'][1]['cube_projection_source'])
        self.assertEqual(len(result['audit']), 2)
        self.assertEqual(len(result['audit'][0]['poses']), 2)
        self.assertEqual(result['audit'][1]['face_quad_px'], second['face_quad_px'])
        self.assertEqual([first, second], before)

    def test_mean_side_evidence_keeps_two_original_selected_poses_not_mean_pose(self):
        first = self.face([self.projected_pose(400., 1),
                           self.projected_pose(410., 2, rms=.4)])
        second = self.face([self.projected_pose(412., 3, normal=[0., 1., 0.])],
                           adjacent=True)
        result = self.solve([first, second])
        self.assertEqual(result['side_selection'], 'mean_of_similar_area_sides')
        self.assertEqual(result['Base_X_mm'], 411.)
        self.assertEqual(result['X_interval_mm'], [411., 411.])
        used = result['selected_faces']
        self.assertEqual(len(used), 2)
        self.assertTrue(all(face['used_for_center'] for face in used))
        self.assertEqual([face['selected_pose']['cube_center_Camera_mm'][0]
                          for face in used], [410., 412.])
        self.assertEqual([face['selected_pose']['cube_projected_corners_px']
                          for face in used],
                         [first['poses'][1]['cube_projected_corners_px'],
                          second['poses'][0]['cube_projected_corners_px']])
        self.assertEqual([face['face_quad_px'] for face in used],
                         [first['face_quad_px'], second['face_quad_px']])
        self.assertEqual(len(result['audit']), 2)

    def test_horizontal_face_and_bad_reprojection_do_not_supply_range(self):
        for pose in (self.pose(normal=[0., 0., 1.]), self.pose(rms=2.01)):
            self.assertIsNone(self.solve([self.face([pose])]))


@unittest.skipIf(HAS_VISION, 'Already in vision environment')
class DedicatedEnvironmentTest(unittest.TestCase):
    def test_side_range_in_dedicated_environment(self):
        result = subprocess.run([str(ROOT/'scripts/vision'), '-m', 'unittest',
            'host.tests.test_side_range'], cwd=ROOT, text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)


if __name__ == '__main__':
    unittest.main()
