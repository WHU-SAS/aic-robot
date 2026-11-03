"""Car-only bounded side localization, exercised in the vision environment."""
import importlib.util
import copy
import json
import subprocess
import unittest
from unittest.mock import patch
from host.project_paths import ROOT

HAS_VISION = importlib.util.find_spec('cv2') is not None
if HAS_VISION:
    import cv2
    import numpy as np
    from device.k230.car import near_vision
    from device.k230.vision import physical_side
    from device.k230.pick.reflection_faces import measure_edge
    from device.k230.pick.side_faces import select_sides


@unittest.skipUnless(HAS_VISION, 'Dedicated vision environment')
class NearVisionTests(unittest.TestCase):
    def scene(self, yaw=.7, roll=.02):
        K = np.array([[1200., 0., 640.], [0., 1600., 580.], [0., 0., 1.]])
        rv, tv = np.array([.4, yaw, roll]), np.array([-10., -10., 250.])
        front = np.array([[0., 0., 0.], [20., 0., 0.], [20., 20., 0.], [0., 20., 0.]])
        right = np.array([[20., 0., 0.], [20., 0., 20.], [20., 20., 20.], [20., 20., 0.]])
        top = np.array([[0., 0., 20.], [20., 0., 20.], [20., 0., 0.], [0., 0., 0.]])
        image = np.full((960, 1280, 3), (80, 140, 170), np.uint8)
        for face, color in ((top, (245, 99, 0)), (right, (145, 36, 1)), (front, (180, 120, 10))):
            q = cv2.projectPoints(face, rv, tv, K, np.zeros(5))[0].reshape(-1, 2)
            cv2.fillConvexPoly(image, np.rint(q).astype(np.int32), color)
        image = cv2.GaussianBlur(image, (3, 3), .7)
        # Synthetic camera-to-Base transform puts the cube on z=-20, center=-10.
        rb = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]]) @ cv2.Rodrigues(rv)[0].T
        T = np.eye(4); T[:3, :3] = rb; T[:3, 3] = -rb @ tv
        intr = {'camera_matrix': K.tolist(), 'image_width': 1280, 'image_height': 960,
                'distortion_coefficients': [0.]*5}
        return image, intr, {'T_Base_from_Camera_mm': T.tolist()}, {'normal': [0., 0., 1.], 'offset_mm': -20.}

    def box(self, image):
        mask = cv2.inRange(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), (90, 120, 35), (140, 255, 255))
        contours, unused = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return list(cv2.boundingRect(max(contours, key=cv2.contourArea)))

    def frame(self, stamp=100):
        return {'frame_id': 7, 'capture_timestamp_ms': stamp, 'clock_domain': 'k230_ticks_ms',
                'timestamp_source': 'software_snapshot_call', 'image_size_px': [1280, 960]}

    def solve(self, image, intr, ext, plane, bbox=None, original=False, **options):
        solver = physical_side._original_localize_steps if original else near_vision.localize_steps
        steps = list(solver(image, self.frame(), bbox or self.box(image),
                     intr, ext, plane, **options))
        self.assertEqual(steps[-1]['stage'], 'complete')
        return steps, steps[-1]['observation']

    def assert_precise_evidence(self, result):
        """Public geometry keeps its actual face2px or cube3px contract."""
        self.assertEqual(result['status'], 'localized', result)
        payload, quality = result['payload'], result['quality']
        evidence = quality.get('evidence_type')
        if evidence in ('nonplanar_cube_five_corners', 'nonplanar_cube_six_corners'):
            count = 6 if evidence == 'nonplanar_cube_six_corners' else 5
            self.assertEqual(payload['evidence_type'], evidence)
            self.assertEqual(payload['status'], 'complete_cube_candidate')
            self.assertEqual(quality['reprojection_limit_px'], 3.)
            self.assertLessEqual(quality['reprojection_rms_px'], 3.)
            self.assertNotIn('poses', payload)
            pose = payload['cube_pose']
            points = pose['image_points_px']
            self.assertEqual(len(points), count)
            self.assertEqual(len(pose['object_points_mm']), count)
            self.assertEqual(len(payload['observed_edges']), 6)
            self.assertEqual(len(payload['half_corner_errors_px']), 2*count)
            self.assertTrue(all(0 <= v <= 3 for v in payload['half_corner_errors_px']))
            methods = {
                5: ('six_measured_edges_five_corners', 'shared_nonplanar_cube_corners'),
                6: ('six_measured_outline_edges_six_corners', 'shared_nonplanar_cube_outline')}
            self.assertEqual((payload['boundary_method'], payload['vision_path']), methods[count])
            edges = [edge['quality'] for edge in payload['observed_edges']]
            if count == 6:
                self.assertLessEqual(pose['table_geometry']['object_up_deviation_deg'], 20.)
        else:
            self.assertIn(evidence, (None, 'physical_face_with_G_ridge'))
            self.assertEqual(payload['status'], 'complete_face_candidate')
            self.assertLessEqual(quality['reprojection_rms_px'], 2.)
            pose = payload['poses'][0]
            self.assertLessEqual(pose['face_reprojection_rms_px'], 2.)
            points, edges = payload['face_quad_px'], payload['physical_edges']
            self.assertEqual(len(points), 4)
            self.assertEqual(len(edges), 4)
        for edge in edges:
            self.assertEqual(edge['sampled_stations'], 64)
            self.assertGreaterEqual(edge['support'], 45)
            self.assertLessEqual(edge['rms_px'], 3.)
        geometry = pose['table_geometry']
        self.assertTrue(geometry['accepted'])
        self.assertLessEqual(geometry['normal_deviation_deg'], 20.)
        self.assertLessEqual(abs(geometry['height_error_mm']), 8.)
        return points

    def test_mean_center_removes_rigid_projection_but_keeps_selected_side_evidence(self):
        faces=[]
        for i,normal in enumerate(([1.,0.,0.],[0.,0.,1.])):
            x=100.+100*i
            pose={'cube_center_Camera_mm':[2.*i,0.,300.],
                  'inward_face_normal_Camera':normal,'face_reprojection_rms_px':.2+.1*i,
                  'cube_projected_corners_px':[[j,i] for j in range(8)],
                  'cube_projection_source':'pose_transform'}
            faces.append({'face_quad_px':[[x,100.],[x+100.,100.],[x+100.,200.],[x,200.]],
                          'area_px2':10000.,'poses':[pose],'physical_edges':[]})
        before=copy.deepcopy(faces)
        K=[[1200.,0.,640.],[0.,1210.,480.],[0.,0.,1.]]
        result=select_sides(faces,K,[])
        self.assertEqual(result['side_selection'],'mean_of_similar_area_sides')
        self.assertEqual(result['poses'][0]['cube_center_Camera_mm'],[1.,0.,300.])
        self.assertNotIn('cube_projected_corners_px',result['poses'][0])
        self.assertNotIn('cube_projection_source',result['poses'][0])
        for face,original in zip(result['side_faces'],faces):
            self.assertTrue(face['used_for_center'])
            self.assertIs(face['selected_pose'],original['poses'][0])
            self.assertEqual(face['selected_pose']['cube_projection_source'],'pose_transform')
            self.assertEqual(len(face['selected_pose']['cube_projected_corners_px']),8)
        self.assertEqual(faces,before)
        # A dominant face still represents its own solved rigid transform.
        faces[0]['area_px2']=15000.
        dominant=select_sides(faces,K,[])
        self.assertEqual(dominant['side_selection'],'larger_verified_side')
        self.assertIs(dominant['poses'][0],faces[0]['poses'][0])
        self.assertEqual(dominant['poses'][0]['cube_projection_source'],'pose_transform')

    def test_public_real_geometry_and_full_pixel_coordinates(self):
        for yaw in (.7, .95):
            image, intr, ext, plane = self.scene(yaw)
            steps, result = self.solve(image, intr, ext, plane)
            self.assertEqual(result['status'], 'localized', result)
            self.assertEqual(result['capture_timestamp_ms'], 100)
            self.assertEqual(result['frame_id'], 7)
            self.assertEqual(result['coordinate_frame'], 'Base')
            self.assertFalse(result['xy_compensation_applied'])
            np.testing.assert_allclose(result['cube_center_Base_mm'], [10., -10., -10.], atol=3.)
            points = self.assert_precise_evidence(result)
            self.assertGreater(min(p[0] for p in points), 400)
            self.assertGreater(min(p[1] for p in points), 400)
            self.assertEqual(sum(s['stage'] == 'complete' for s in steps), 1)
            self.assertNotIn('original_face_fallback', [s['stage'] for s in steps])

    def test_cooperative_edges_match_original_64_station_measurement(self):
        image, unused, unused, unused = self.scene()
        smooth = cv2.GaussianBlur(image, (3, 3), .7)
        # Use a physical edge found by the bounded pipeline, rather than a box.
        image, intr, ext, plane = self.scene()
        unused, result = self.solve(image, intr, ext, plane, original=True)
        a, b = result['payload']['face_quad_px'][:2]
        expected = measure_edge(smooth, a, b)
        steps = list(near_vision._edge_steps(smooth, a, b))
        actual = steps[-1]['edge']
        np.testing.assert_allclose(actual[0], expected[0], atol=1e-12)
        self.assertEqual(actual[1]['support'], expected[1]['support'])
        self.assertEqual(actual[1]['sampled_stations'], expected[1]['sampled_stations'])
        self.assertAlmostEqual(actual[1]['rms_px'], expected[1]['rms_px'], places=5)
        np.testing.assert_allclose(actual[2], expected[2], atol=1e-12)
        self.assertEqual([s['stations'] for s in steps[:-1]], list(range(8, 65, 8)))

    def test_bright_top_only_box_keeps_the_physical_side_below_it(self):
        image, intr, ext, plane = self.scene()
        mask = cv2.inRange(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), (90, 120, 220), (140, 255, 255))
        cs, unused = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        top_box = list(cv2.boundingRect(max(cs, key=cv2.contourArea)))
        unused, result = self.solve(image, intr, ext, plane, bbox=top_box)
        self.assertEqual(result['status'], 'localized', result)
        points = self.assert_precise_evidence(result)
        x, y, w, h = result['search']['roi_px']
        self.assertGreater(y+h, top_box[1]+top_box[3]+24)
        self.assertGreater(max(p[1] for p in points), top_box[1]+top_box[3]+24)
        # Expanded ROI is not permission to infer a hidden/missing bottom.
        cut = top_box[1]+top_box[3]+24
        broken = image.copy(); broken[cut:, :] = (150, 55, 5)
        unused, rejected = self.solve(broken, intr, ext, plane, bbox=top_box)
        self.assertEqual(rejected['status'], 'no_complete_face')
        self.assertNotIn('cube_center_Base_mm', rejected)

    def test_missing_edges_and_uniform_blue_are_not_completed(self):
        image, intr, ext, plane = self.scene()
        box = self.box(image)
        missing = image.copy(); missing[545:, :] = (150, 55, 5)
        rectangle = np.full_like(image, (80, 140, 170))
        cv2.rectangle(rectangle, (580, 500), (720, 700), (180, 120, 10), -1)
        for broken in (missing, rectangle):
            unused, result = self.solve(broken, intr, ext, plane, bbox=box)
            self.assertEqual(result['status'], 'no_complete_face')
            self.assertNotIn('cube_center_Base_mm', result)
            self.assertNotIn('payload', result)

    def test_no_global_or_top_fallback_when_roi_fails(self):
        image, intr, ext, plane = self.scene(); box = self.box(image)
        image[:] = 90
        with patch('device.k230.pick.vision.localize', side_effect=AssertionError('Slow fallback forbidden')):
            unused, result = self.solve(image, intr, ext, plane, bbox=box)
        self.assertEqual(result['status'], 'no_complete_face')
        self.assertNotIn('cube_center_Base_mm', result)

    def test_table_constraint_is_preserved(self):
        image, intr, ext, plane = self.scene()
        wrong = dict(plane, offset_mm=plane['offset_mm']+30)
        unused, result = self.solve(image, intr, ext, wrong)
        self.assertEqual(result['status'], 'no_complete_face')
        self.assertNotIn('cube_center_Base_mm', result)

    def test_bad_frame_shape_and_uncalibrated_table_are_rejected(self):
        image, intr, ext, plane = self.scene()
        for frame in (dict(self.frame(), image_size_px=[640, 480]), dict(self.frame(), frame_id=None)):
            with self.assertRaises(ValueError):
                list(near_vision.localize_steps(image, frame, self.box(image), intr, ext, plane))
        with self.assertRaises(ValueError):
            self.solve(image[:480], intr, ext, plane)
        with self.assertRaises(ValueError):
            self.solve(image, intr, ext, None)

    def test_old_original_center_photo_replays_with_same_physical_side(self):
        path = ROOT/'data/car/workspace/overlay_20261024_001214_026101/frame.jpg'
        if not path.exists():
            self.skipTest('Original camera image is local-only evidence')
        cfg = json.loads((ROOT/'configs/vision/trials/fixed_tip_top_20261022.json').read_text())['localization']
        intr = json.loads((ROOT/cfg['intrinsics']).read_text())
        ext = json.loads((ROOT/cfg['extrinsics']).read_text())
        image = cv2.imread(str(path))
        unused, result = self.solve(image, intr, ext, cfg['table_plane_Base_mm'], original=True)
        self.assertEqual(result['status'], 'localized')
        np.testing.assert_allclose(result['cube_center_Base_mm'], [251.54695, 31.10842, -57.86134], atol=.8)
        self.assertTrue(all(e['support'] >= 45 for e in result['payload']['physical_edges']))

    def test_native_top_blob_photo_recovers_bottom_outside_old_roi(self):
        path = ROOT/'data/car/near_parking/live_frame.jpg'
        if not path.exists():
            self.skipTest('Original camera image is local-only evidence')
        cfg = json.loads((ROOT/'configs/vision/trials/fixed_tip_top_20261022.json').read_text())['localization']
        intr = json.loads((ROOT/cfg['intrinsics']).read_text())
        ext = json.loads((ROOT/cfg['extrinsics']).read_text())
        unused, result = self.solve(cv2.imread(str(path)), intr, ext, cfg['table_plane_Base_mm'],
                                     bbox=[335, 459, 147, 104], original=True)
        self.assertEqual(result['status'], 'localized')
        self.assertEqual(result['search']['roi_px'], [311, 435, 195, 284])
        q = result['payload']['face_quad_px']
        self.assertGreater(min(p[1] for p in q[2:]), 660)
        self.assertLess(max(p[1] for p in q[:2]), 515)
        self.assertLessEqual(result['quality']['reprojection_rms_px'], 2.)
        # Different observed separator seeds can move subpixel edge fits;
        # preserve the physical-side solution, not an incidental seed's floats.
        np.testing.assert_allclose(result['cube_center_Base_mm'], [281.26565, 65.79357, -54.71127], atol=1.)

    def test_original_same_hue_shaded_front_uses_an_observed_separator(self):
        folder = ROOT/'data/car/near_parking/stopped_scene_20261030_193117'
        if not (folder/'frame.jpg').exists():
            self.skipTest('Original camera frame is local-only evidence')
        cfg = json.loads((folder/'active_config.json').read_text())
        capture = json.loads((folder/'capture.json').read_text())
        steps, result = self.solve(cv2.imread(str(folder/'frame.jpg')), cfg['intrinsics'],
            cfg['extrinsics'], cfg['table_plane'], bbox=capture['coarse']['bbox_px'], original=True)
        self.assertEqual(result['status'], 'localized', result)
        self.assertTrue(any(s['stage'] == 'seam_seed' for s in steps))
        self.assertLessEqual(result['search']['measured_seed_count'], 3)
        q = result['payload']['face_quad_px']
        self.assertTrue(all(430 < p[1] < 455 for p in q[:2]))
        self.assertTrue(all(p[1] > 580 for p in q[2:]))
        self.assertTrue(all(e['support'] >= 45 and e['sampled_stations'] == 64
                            for e in result['payload']['physical_edges']))
        self.assertLessEqual(result['quality']['reprojection_rms_px'], 2.)
        self.assertTrue(result['payload']['poses'][0]['table_geometry']['accepted'])
        np.testing.assert_allclose(result['cube_center_Base_mm'], [307.6834, 22.1247, -52.3997], atol=2.)

    def test_separator_search_handles_slanted_faces(self):
        for roll in (-.08, .08):
            image, intr, ext, plane = self.scene(.7, roll)
            unused, result = self.solve(image, intr, ext, plane)
            self.assert_precise_evidence(result)
            np.testing.assert_allclose(result['cube_center_Base_mm'], [10., -10., -10.], atol=3.)


@unittest.skipIf(HAS_VISION, 'Already in vision environment')
class DedicatedEnvironmentTest(unittest.TestCase):
    def test_car_near_vision_in_dedicated_environment(self):
        result = subprocess.run([str(ROOT/'scripts/vision'), '-m', 'unittest',
            'host.tests.test_car_near_vision'], cwd=ROOT, text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)


if __name__ == '__main__':
    unittest.main()
