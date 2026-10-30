"""Shared fast physical-side path; no camera, UART or arm actions."""
import importlib.util
import json
import subprocess
import unittest
from unittest.mock import patch
from host.project_paths import ROOT

HAS_VISION = importlib.util.find_spec('cv2') is not None
if HAS_VISION:
    import cv2
    import numpy as np
    from device.k230.vision import physical_side
    from device.k230.car import near_vision
    from host.tests import test_car_near_vision as fixtures


@unittest.skipUnless(HAS_VISION, 'Dedicated vision environment')
class SharedPhysicalSideTests(unittest.TestCase):
    def setUp(self):
        self.fixture = fixtures.NearVisionTests()

    def observation(self, image, intr, ext, plane, **options):
        steps = list(physical_side.localize_steps(image, self.fixture.frame(), self.fixture.box(image),
            intr, ext, plane, **options))
        return steps, steps[-1]['observation']

    def formal_thin_side_scene(self):
        localization = json.loads((ROOT/'configs/vision/trials/fixed_tip_top_20261022.json').read_text())['localization']
        intr = json.loads((ROOT/localization['intrinsics']).read_text())
        ext = json.loads((ROOT/localization['extrinsics']).read_text())
        plane = localization['table_plane_Base_mm']
        line = json.loads((ROOT/'configs/car/workspace_definition.json').read_text())['centerline']['local_Base_Y_line']
        n = np.array(plane['normal']); n /= np.linalg.norm(n)
        ax = np.array([1.,0.,0.]); ax -= np.dot(ax,n)*n; ax /= np.linalg.norm(ax)
        bx = np.cross(n,ax)
        x = 400.; y = line['slope_per_X']*x+line['intercept_mm']
        center = np.array([x,y,(plane['offset_mm']+10-n[0]*x-n[1]*y)/n[2]])
        T = np.array(ext['T_Base_from_Camera_mm']); camera = T[:3,3]
        vertices = np.array([center+i*10*ax+j*10*bx+k*10*n
            for k in (-1,1) for j in (-1,1) for i in (-1,1)])
        cp = (T[:3,:3].T@(vertices-camera).T).T
        pixels = cv2.projectPoints(cp,np.zeros(3),np.zeros(3),np.array(intr['camera_matrix']),
            np.array(intr['distortion_coefficients']))[0].reshape(-1,2)
        faces = [([0,1,3,2],-n),([4,6,7,5],n),([0,4,5,1],-bx),
                 ([2,3,7,6],bx),([0,2,6,4],-ax),([1,5,7,3],ax)]
        visible = []
        for ids,normal in faces:
            if np.dot(camera-vertices[ids].mean(axis=0),normal)>0:
                visible.append((float(cp[ids,2].mean()),pixels[ids],abs(np.dot(normal,n))<.5))
        image = np.full((960,1280,3),(80,140,170),np.uint8)
        colours,side_count = [(180,120,10),(145,36,1)],0
        for unused,points,side in sorted(visible,key=lambda face:-face[0]):
            colour = colours[side_count%2] if side else (245,99,0)
            side_count += side
            cv2.fillConvexPoly(image,np.rint(points).astype(np.int32),colour)
        image = cv2.GaussianBlur(image,(3,3),.7)
        image[:,:,0]=(image[:,:,0]>>3)<<3; image[:,:,1]=(image[:,:,1]>>2)<<2; image[:,:,2]=(image[:,:,2]>>3)<<3
        ok,jpeg=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,95]); self.assertTrue(ok)
        image=cv2.imdecode(jpeg,cv2.IMREAD_COLOR)
        largest = max((points for unused,points,side in visible if side),
            key=lambda points:abs(cv2.contourArea(points.astype(np.float32))))
        return image,intr,ext,plane,center,largest

    def test_thin_side_keeps_the_front_edge_instead_of_the_outer_silhouette(self):
        image,intr,ext,plane,truth,expected = self.formal_thin_side_scene()
        results=[]
        for native in (True,False):
            with patch.object(physical_side,'_BATCH_EDGES',native), patch.object(
                    physical_side,'pose_candidates',wraps=physical_side.pose_candidates) as solve:
                unused,result=self.observation(image,intr,ext,plane,include_range=True)
            self.assertEqual(result['status'],'localized',result)
            self.assertNotIn('range_candidate',result)
            self.assertEqual(solve.call_count,1)
            self.assertLess(abs(result['cube_center_Base_mm'][0]-truth[0]),2.)
            self.assertLess(physical_side.corner_distance(result['payload']['face_quad_px'],expected.tolist()),2.)
            self.assertTrue(any(e.get('edge_ownership')=='internal_blue_blue'
                for e in result['payload']['physical_edges']))
            self.assertTrue(all(e['sampled_stations']==64 and e['support']>=45
                for e in result['payload']['physical_edges']))
            self.assertLessEqual(result['quality']['reprojection_rms_px'],2.)
            self.assertTrue(result['payload']['poses'][0]['table_geometry']['accepted'])
            results.append(result)
        np.testing.assert_allclose(results[0]['cube_center_Base_mm'],results[1]['cube_center_Base_mm'],atol=.01)

    def test_blue_background_does_not_replace_front_separator_with_top_outline(self):
        folder=ROOT/'data/car/one_shot_parking/failed70/evidence'
        if not (folder/'moving_fine.jpg').exists():
            self.skipTest('Original moving frame is local-only evidence')
        cfg=json.loads((folder/'config.json').read_text())
        frame=json.loads((folder/'manifest.json').read_text())['frames'][0]['frame']
        image=cv2.imread(str(folder/'moving_fine.jpg'))
        for native in (True,False):
            with self.subTest(native=native), patch.object(physical_side,'_BATCH_EDGES',native):
                result=list(physical_side.localize_steps(image,frame,[598,247,100,137],
                    cfg['intrinsics'],cfg['extrinsics'],cfg['table_plane'],include_range=True))[-1]['observation']
            self.assertEqual(result['status'],'no_complete_face')
            self.assertNotIn('cube_center_Base_mm',result)
            candidate=result['range_candidate']
            quad=candidate['audit'][0]['face_quad_px']
            line=physical_side.fit(quad[:2])
            # Pixel landmarks on this retained photo, not surveyed millimetres.
            # The real top/front crease crosses (640,272); the top silhouette
            # crosses (640,249) against the blue background storage box.
            self.assertLess(physical_side.residual([640,272],line),3.)
            self.assertGreater(physical_side.residual([640,249],line),15.)
            self.assertTrue(candidate['not_arm_or_parking_authority'])
            self.assertFalse(candidate['height_used_for_selection'])

    def test_weak_separator_can_supply_range_without_replacing_failed_strict_pose(self):
        folder=ROOT/'data/car/one_shot_parking/diagnostics/trial_20261030_213040'
        image_path=folder/'run_evidence/moving_fine.jpg'
        if not image_path.exists():
            self.skipTest('Original difficult frame is local-only evidence')
        cfg=json.loads((folder/'run_evidence/config.json').read_text())
        frame=json.loads((folder/'run_evidence/manifest.json').read_text())['frames'][0]['frame']
        image=cv2.imread(str(image_path)); counts=[]
        for include_range in (False,True):
            with patch.object(physical_side,'pose_candidates',wraps=physical_side.pose_candidates) as solve:
                result=list(physical_side.localize_steps(image,frame,[583,251,97,135],
                    cfg['intrinsics'],cfg['extrinsics'],cfg['table_plane'],include_range=include_range))[-1]['observation']
            counts.append(solve.call_count)
            self.assertEqual(result['status'],'no_complete_face')
            self.assertNotIn('cube_center_Base_mm',result)
            self.assertEqual(result['capture_timestamp_ms'],frame['capture_timestamp_ms'])
            if include_range:
                ranged=result['range_candidate']
                self.assertEqual(ranged['status'],'range_candidate')
                self.assertTrue(ranged['not_arm_or_parking_authority'])
                # Historical prediction, not surveyed truth. Subpixel edge
                # fits may change slightly without changing the measured face.
                self.assertAlmostEqual(ranged['Base_X_mm'],414.271678,delta=1.)
                self.assertLess(ranged['X_interval_mm'][1]-ranged['X_interval_mm'][0],1.)
            else:
                self.assertNotIn('range_candidate',result)
        self.assertEqual(counts,[1,1])

    def test_background_gradient_flank_does_not_hide_the_real_front_crease(self):
        folder=ROOT/'data/car/one_shot_parking/failed70_retry/evidence'
        if not (folder/'moving_fine.jpg').exists():
            self.skipTest('Original moving frame is local-only evidence')
        cfg=json.loads((folder/'config.json').read_text())
        frame=json.loads((folder/'manifest.json').read_text())['frames'][0]['frame']
        image=cv2.imread(str(folder/'moving_fine.jpg'))
        for native in (True,False):
            with self.subTest(native=native), patch.object(physical_side,'_BATCH_EDGES',native):
                result=list(physical_side.localize_steps(image,frame,[622,253,103,141],
                    cfg['intrinsics'],cfg['extrinsics'],cfg['table_plane'],include_range=True))[-1]['observation']
                self.assertIn('range_candidate',result)
                candidate=result['range_candidate']
                line=physical_side.fit(candidate['audit'][0]['face_quad_px'][:2])
                # Retained image landmarks, not measured Base coordinates.
                self.assertLess(physical_side.residual([660,275],line),3.)
                self.assertGreater(physical_side.residual([660,259],line),10.)
                self.assertEqual(result['capture_timestamp_ms'],frame['capture_timestamp_ms'])
                self.assertEqual(result['status'],'no_complete_face')
                self.assertNotIn('cube_center_Base_mm',result)
                self.assertTrue(candidate['not_arm_or_parking_authority'])

    def test_car_is_a_shared_function_alias(self):
        self.assertIs(near_vision.localize_steps, physical_side.localize_steps)
        self.assertIs(near_vision.localize, physical_side.localize)

    def test_analytic_pose_jacobian_matches_central_differences(self):
        # Independent Rodrigues perturbations include rotation about the left
        # camera axes, while translation stays independent of that rotation.
        K = [[1557.15, 0., 652.62], [0., 2071.93, 351.65], [0., 0., 1.]]
        D = [.007061, -.46861, -.024136, -.0007221, 4.66149]
        for vector, translation in (([.4, .7, .02], [-10., -10., 250.]),
                                    ([-.8, -.35, .2], [100., -40., 480.])):
            R = cv2.Rodrigues(np.array(vector))[0]
            actual = physical_side._pose_jacobian(R.tolist(), translation, K, D)
            expected = []
            for axis in range(6):
                h = 1e-6 if axis < 3 else 1e-4
                residuals = []
                for sign in (1., -1.):
                    delta = np.zeros(6); delta[axis] = sign*h
                    rr = cv2.Rodrigues(delta[:3])[0] @ R
                    tt = np.array(translation)+delta[3:]
                    pixels = [physical_side.project((rr @ np.array(obj)+tt).tolist(), K, D)
                              for obj in physical_side.OBJECT]
                    residuals.append(np.array(pixels).reshape(-1))
                expected.append((residuals[0]-residuals[1])/(2*h))
            np.testing.assert_allclose(actual, expected, atol=2e-7, rtol=2e-7)

    def test_analytic_pose_refinement_preserves_both_solutions_and_objective(self):
        from device.k230.pick.pose import pose_candidates
        K = [[1557.15, 0., 652.62], [0., 2071.93, 351.65], [0., 0., 1.]]
        D = [.007061, -.46861, -.024136, -.0007221, 4.66149]
        quads = []
        for vector, translation in (([.4, .7, .02], [-10., -10., 250.]),
                                    ([-.8, -.35, .2], [100., -40., 480.])):
            R = cv2.Rodrigues(np.array(vector))[0]
            quad = [physical_side.project((R @ np.array(obj)+translation).tolist(), K, D)
                    for obj in physical_side.OBJECT]
            quads.append(np.array(quad)+[[.3, -.2], [-.4, .1], [.2, .4], [-.1, -.3]])
        # The real failed image and the latest native two-side image exercise
        # measured corners, not only perfectly projected synthetic squares.
        for path in (ROOT/'data/car/near_parking/shared_vision_optimization/paired_replay.json',
                     ROOT/'data/car/near_parking/shared_fast_native_20261030_205406/live_profile.json'):
            if path.exists():
                data = json.loads(path.read_text())
                def collect(value):
                    if isinstance(value, dict):
                        if 'face_quad_px' in value:
                            quads.append(value['face_quad_px'])
                        for item in value.values():
                            collect(item)
                    elif isinstance(value, list):
                        for item in value:
                            collect(item)
                collect(data)
        for quad in quads:
            original = pose_candidates(np.array(quad).tolist(), K, D, refine=True)
            analytic = pose_candidates(np.array(quad).tolist(), K, D, refine=True,
                                       refiner=physical_side.refine_pose)
            self.assertEqual(len(analytic), len(original))
            self.assertEqual(len(analytic), 2)
            # Match the two ambiguous seeds by center so a numerical RMS tie
            # cannot change which solutions the comparison associates.
            unmatched = list(analytic)
            for pose in original:
                best = min(unmatched, key=lambda item: np.linalg.norm(np.array(
                    item['cube_center_Camera_mm'])-pose['cube_center_Camera_mm']))
                unmatched.remove(best)
                np.testing.assert_allclose(best['cube_center_Camera_mm'],
                    pose['cube_center_Camera_mm'], atol=.002, rtol=0.)
                np.testing.assert_allclose(best['inward_face_normal_Camera'],
                    pose['inward_face_normal_Camera'], atol=.0002, rtol=0.)
                self.assertAlmostEqual(best['face_reprojection_rms_px'],
                    pose['face_reprojection_rms_px'], places=5)

    def test_portable_and_native_sampling_preserve_geometry_and_64_stations(self):
        if not physical_side._BATCH_EDGES:
            self.skipTest('Native remap not available')
        for yaw in (.7, .95):
            args = self.fixture.scene(yaw)
            with patch.object(physical_side, '_BATCH_EDGES', False):
                old_steps, old = self.observation(*args)
            new_steps, new = self.observation(*args)
            self.assertEqual(new['status'], 'localized')
            self.assertEqual(new['payload']['side_selection'], old['payload']['side_selection'])
            np.testing.assert_allclose(new['cube_center_Base_mm'], old['cube_center_Base_mm'], atol=.01)
            self.assertEqual(new['search']['sampling_backend'], 'native_remap')
            for face in new['payload']['side_faces']:
                self.assertTrue(all(e['sampled_stations'] == 64 and e['support'] >= 45
                                    for e in face['physical_edges']))
            self.assertEqual(new['capture_timestamp_ms'], 100)

    def test_synchronous_raw_result_and_steps_share_complete_face_payload(self):
        image, intr, ext, plane = self.fixture.scene()
        steps, observation = self.observation(image, intr, ext, plane)
        raw = physical_side.localize(image, intr['camera_matrix'], intr['distortion_coefficients'],
            ext['T_Base_from_Camera_mm'], plane)
        np.testing.assert_allclose(raw['cube_center_Base_mm'], observation['cube_center_Base_mm'], atol=.01)
        self.assertTrue(raw['physical_edges'])
        self.assertTrue(raw['poses'])
        self.assertEqual(raw['side_selection'], observation['payload']['side_selection'])
        from device.k230.pick.pose import project
        camera = raw['cube_center_Camera_mm']
        np.testing.assert_allclose(camera, raw['poses'][0]['cube_center_Camera_mm'], atol=1e-9)
        np.testing.assert_allclose(project(camera, intr['camera_matrix'], intr['distortion_coefficients']),
            raw['poses'][0]['cube_center_pixel'], atol=1e-9)
        T = ext['T_Base_from_Camera_mm']
        np.testing.assert_allclose([sum(T[i][j]*camera[j] for j in range(3))+T[i][3] for i in range(3)],
            raw['cube_center_Base_mm'], atol=1e-9)
        self.assertFalse(observation['xy_compensation_applied'])
        self.assertTrue(any(step['stage'] == 'edge_reused' for step in steps))

    def test_blank_image_is_not_a_synchronous_face_or_top_fallback(self):
        image, intr, ext, plane = self.fixture.scene(); image[:] = 90
        with self.assertRaises(ValueError):
            physical_side.localize(image, intr['camera_matrix'], intr['distortion_coefficients'],
                ext['T_Base_from_Camera_mm'], plane)

    def test_auto_box_steps_match_raw_and_preserve_failure_metadata(self):
        image, intr, ext, plane = self.fixture.scene()
        frame = self.fixture.frame()
        steps = list(physical_side.localize_steps(image, frame, None, intr, ext, plane))
        result = steps[-1]['observation']
        raw = physical_side.localize(image, intr['camera_matrix'], intr['distortion_coefficients'],
            ext['T_Base_from_Camera_mm'], plane)
        self.assertEqual(result['status'], 'localized')
        np.testing.assert_allclose(result['cube_center_Base_mm'], raw['cube_center_Base_mm'], atol=.01)
        self.assertEqual(result['frame_id'], frame['frame_id'])
        self.assertEqual(result['capture_timestamp_ms'], frame['capture_timestamp_ms'])
        blank = image.copy(); blank[:] = 90
        rejected = list(physical_side.localize_steps(blank, frame, None, intr, ext, plane))[-1]['observation']
        self.assertEqual(rejected['status'], 'no_complete_face')
        self.assertEqual(rejected['capture_timestamp_ms'], frame['capture_timestamp_ms'])
        self.assertNotIn('cube_center_Base_mm', rejected)
        self.assertNotIn('payload', rejected)
        folder = ROOT/'data/car/near_parking/stopped_scene_20261030_193117'
        if (folder/'frame.jpg').exists():
            cfg = json.loads((folder/'active_config.json').read_text())
            capture = json.loads((folder/'capture.json').read_text())
            original = cv2.imread(str(folder/'frame.jpg'))
            actual = list(physical_side.localize_steps(original, capture['frame'], None,
                cfg['intrinsics'], cfg['extrinsics'], cfg['table_plane']))[-1]['observation']
            same = physical_side.localize(original, cfg['intrinsics']['camera_matrix'],
                cfg['intrinsics']['distortion_coefficients'], cfg['extrinsics']['T_Base_from_Camera_mm'],
                cfg['table_plane'])
            self.assertEqual(actual['status'], 'localized')
            self.assertLess(actual['search']['roi_px'][1], 400)
            np.testing.assert_allclose(actual['cube_center_Base_mm'], same['cube_center_Base_mm'], atol=.01)
            self.assertEqual(actual['capture_timestamp_ms'], capture['frame']['capture_timestamp_ms'])

    def test_native_and_portable_same_original_stationary_failure_frame(self):
        folder = ROOT/'data/car/near_parking/stopped_scene_20261030_193117'
        if not (folder/'frame.jpg').exists():
            self.skipTest('Original frame is local-only evidence')
        cfg = json.loads((folder/'active_config.json').read_text())
        capture = json.loads((folder/'capture.json').read_text())
        image = cv2.imread(str(folder/'frame.jpg'))
        args = (image, capture['frame'], capture['coarse']['bbox_px'], cfg['intrinsics'],
                cfg['extrinsics'], cfg['table_plane'])
        with patch.object(physical_side, '_BATCH_EDGES', False):
            reference = list(physical_side.localize_steps(*args))[-1]['observation']
        steps = list(physical_side.localize_steps(*args)); actual = steps[-1]['observation']
        self.assertEqual(actual['status'], 'localized')
        np.testing.assert_allclose(actual['cube_center_Base_mm'], reference['cube_center_Base_mm'], atol=.01)
        self.assertLessEqual(actual['quality']['reprojection_rms_px'], 2.)
        self.assertLessEqual(actual['search']['measured_seed_count'], 1)
        self.assertEqual(actual['capture_timestamp_ms'], capture['frame']['capture_timestamp_ms'])
        self.assertTrue(any(s['stage'] == 'edge_reused' for s in steps))


@unittest.skipIf(HAS_VISION, 'Already in vision environment')
class DedicatedEnvironmentTest(unittest.TestCase):
    def test_shared_physical_side_in_dedicated_environment(self):
        result = subprocess.run([str(ROOT/'scripts/vision'), '-m', 'unittest',
            'host.tests.test_shared_physical_side'], cwd=ROOT, text=True, capture_output=True)
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)


if __name__ == '__main__':
    unittest.main()
