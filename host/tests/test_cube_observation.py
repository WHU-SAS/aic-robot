"""Five-corner evidence authority, scalar fixtures and real runtime envelopes.

No OpenCV, NumPy or SciPy imports: known rigid3D points are projected with an
independent scalar camera model. A successful image packet is not a physical
grasp claim, nor a substitute for the older measured-face2px policy.
"""
import copy
import math
import unittest

from device.k230.pick.numeric import matmul, matvec, rotation, transpose, unit
from device.k230.vision import cube_observation as cube
from device.k230.vision.runtime import localize_precise
from device.k230.vision.candidate_association import associate_fine_candidate


K = [[1500., 0., 640.], [0., 1480., 480.], [0., 0., 1.]]
D = [-.12, .035, .001, -.0015, -.006]
INTRINSICS = {'camera_matrix': K, 'distortion_coefficients': D,
              'image_width': 1280, 'image_height': 960}
FRAME = {'frame_id': 91, 'capture_timestamp_ms': 1234,
         'clock_domain': 'k230_test_boot', 'image_size_px': [1280, 960],
         'timestamp_source': 'sensor_snapshot'}
PLANE = {'normal': [0., 0., 1.], 'offset_mm': 0.}
OBJECT = [[-10., 0., 20.], [-10., 20., 20.], [10., 20., 20.],
          [10., 0., 0.], [-10., 0., 0.]]
FACE_BACK_OBJECT = [[-10., 0., 20.], [10., 0., 20.], [10., 0., 0.],
                    [-10., 0., 0.], [-10., 20., 20.], [10., 20., 20.]]
FACE_BACK_TYPE = 'nonplanar_cube_face_back_top'
FACE_BACK_METHOD = 'seven_measured_edges_front_and_back_top_corners'
FACE_BACK_PATH = 'shared_nonplanar_cube_face_back_top'


def projected(R, t, points):
    result = []
    for point in points:
        camera = [sum(R[i][j]*point[j] for j in range(3))+t[i]
                  for i in range(3)]
        x, y = camera[0]/camera[2], camera[1]/camera[2]
        r2 = x*x+y*y
        k1, k2, p1, p2, k3 = D
        gain = 1+k1*r2+k2*r2*r2+k3*r2*r2*r2
        u = x*gain+2*p1*x*y+p2*(r2+2*x*x)
        v = y*gain+p1*(r2+2*y*y)+2*p2*x*y
        result.append([K[0][0]*u+K[0][2], K[1][1]*v+K[1][2]])
    return result


def fixture(sign=1, six=False, face_back_top=False):
    up = unit([0., -.98, -.2])
    table_R = transpose([[1., 0., 0.], [0., up[2], -up[1]], up])
    yaw = rotation([0., 0., 1.], .28)
    R = matmul(table_R, yaw)
    t = [4., 35., 220.]
    base_R = transpose(table_R)
    origin_Base = [100., -20., 0.]
    base_t = [origin_Base[i]-value for i, value in
              enumerate(matvec(base_R, t))]
    T = [row+[base_t[i]] for i, row in enumerate(base_R)]
    T.append([0., 0., 0., 1.])
    objects = list(OBJECT)
    if six:objects.insert(3, [10., 20., 0.])
    if face_back_top:objects = list(FACE_BACK_OBJECT)
    objects = [[sign*p[0], p[1], p[2]] for p in objects]
    pixels = projected(R, t, objects)
    edges = [{'name': name, 'quality': {'sampled_stations': 64,
              'support': 60, 'rms_px': .35}}
             for name in ('front_left', 'top_depth', 'back_top',
                          'back_right', 'front_right', 'front_bottom')]
    raw = {'status': 'complete_cube_candidate',
           'evidence_type': ('nonplanar_cube_six_corners' if six else
                             'nonplanar_cube_five_corners'),
           'boundary_method': ('six_measured_outline_edges_six_corners' if six else
                               'six_measured_edges_five_corners'),
           'vision_path': ('shared_nonplanar_cube_outline' if six else
                           'shared_nonplanar_cube_corners'),
           'observed_edges': edges, 'half_corner_errors_px': [.2]*(12 if six else 10),
           'cube_pose': {'R_Object_to_Camera': R,
               't_Object_to_Camera_mm': t,
               'object_points_mm': objects, 'image_points_px': pixels,
               'object_center_mm': [0., 10., 10.], 'edge_length_mm': 20,
               'corner_reprojection_rms_px': 0.}}
    if face_back_top:
        raw.update(evidence_type=FACE_BACK_TYPE, boundary_method=FACE_BACK_METHOD,
                   vision_path=FACE_BACK_PATH, corner_labels=['A', 'B', 'C', 'D', 'E', 'F'])
        raw['observed_edges'].append({'name': 'back_top_right',
            'quality': {'sampled_stations': 64, 'support': 60, 'rms_px': .35}})
        raw['half_corner_errors_px'] = [.2]*12
    return raw, {'T_Base_from_Camera_mm': T}, origin_Base, yaw


class CubeObservationTests(unittest.TestCase):
    def observe(self, raw, extrinsics):
        return localize_precise(None, FRAME, INTRINSICS, extrinsics, PLANE,
            localizer=lambda *args: copy.deepcopy(raw))

    def assert_rejected_without_coordinates(self, observation):
        self.assertEqual(observation['status'], 'no_complete_face', observation)
        for field in ('cube_center_Base_mm', 'cube_center_Camera_mm',
                      'payload', 'quality', 'coordinate_frame'):
            self.assertNotIn(field, observation)
        self.assertEqual(observation['capture_timestamp_ms'], 1234)
        self.assertFalse(observation['motion_performed'])

    def test_five_true_corners_use_their_own_approved3px_evidence_type(self):
        raw, extrinsics, unused, unused = fixture()
        # A deliberately imperfect, unchanged observation is a2.5px residual
        # against the supplied rigid pose. It must not impersonate a2px face.
        raw['cube_pose']['image_points_px'] = [[x+2.5, y] for x, y in
            raw['cube_pose']['image_points_px']]
        result = self.observe(raw, extrinsics)
        self.assertEqual(result['status'], 'localized', result)
        quality = result['quality']
        self.assertEqual(quality['evidence_type'], 'nonplanar_cube_five_corners')
        self.assertEqual(quality['boundary_method'], 'six_measured_edges_five_corners')
        self.assertEqual(quality['reprojection_limit_px'], 3.)
        self.assertAlmostEqual(quality['reprojection_rms_px'], 2.5, places=8)
        self.assertEqual(result['reference_point'], 'cube_geometric_center')
        self.assertEqual(result['frame_id'], FRAME['frame_id'])
        self.assertEqual(result['capture_timestamp_ms'], 1234)
        self.assertFalse(result['xy_compensation_applied'])

    def test_legacy_measured_face_still_has_its_own_hard2px_limit(self):
        unused, extrinsics, unused, unused = fixture()
        face = {'status': 'complete_face_candidate',
                'cube_center_Base_mm': [100., -10., 10.],
                'poses': [{'face_reprojection_rms_px': 2.}]}
        self.assertEqual(self.observe(face, extrinsics)['status'], 'localized')
        face['poses'][0]['face_reprojection_rms_px'] = 2.01
        self.assert_rejected_without_coordinates(self.observe(face, extrinsics))

    def test_forged_rms_cannot_override_the_actual_five_point_residual(self):
        raw, extrinsics, unused, unused = fixture()
        raw['cube_pose']['corner_reprojection_rms_px'] = 0.
        raw['cube_pose']['image_points_px'] = [[x+3.25, y] for x, y in
            raw['cube_pose']['image_points_px']]
        raw['cube_center_Base_mm'] = [9999., 9999., 9999.]
        self.assert_rejected_without_coordinates(self.observe(raw, extrinsics))
        # Conversely, an untrusted stale large hint does not reject valid
        # geometry; the raw observed points, not the hint, define the RMS.
        raw, extrinsics, unused, unused = fixture()
        raw['cube_pose']['corner_reprojection_rms_px'] = 100.
        result = self.observe(raw, extrinsics)
        self.assertEqual(result['status'], 'localized')
        self.assertLess(result['quality']['reprojection_rms_px'], 1e-8)

    def test_reflection_scale_and_nonfinite_poses_never_leak_precise_xyz(self):
        for kind in ('reflection', 'scale', 'nan', 'infinite_translation'):
            raw, extrinsics, unused, unused = fixture()
            pose = raw['cube_pose']
            if kind == 'reflection':
                for row in pose['R_Object_to_Camera']:row[0] *= -1
            elif kind == 'scale':
                pose['R_Object_to_Camera'] = [[1.03*v for v in row]
                    for row in pose['R_Object_to_Camera']]
            elif kind == 'nan':pose['R_Object_to_Camera'][0][0] = float('nan')
            else:pose['t_Object_to_Camera_mm'][1] = float('inf')
            raw['cube_center_Base_mm'] = [1., 2., 3.]
            with self.subTest(kind=kind):
                self.assert_rejected_without_coordinates(self.observe(raw, extrinsics))

    def test_missing_correspondences_wrong_model_or_missing_source_edges_are_rejected(self):
        for kind in ('four_corners', 'wrong_edge_length', 'scaled_object_points',
                     'five_edges', 'unsupported_edge', 'inconsistent_halves'):
            raw, extrinsics, unused, unused = fixture()
            if kind == 'four_corners':
                raw['cube_pose']['image_points_px'].pop()
                raw['cube_pose']['object_points_mm'].pop()
            elif kind == 'wrong_edge_length':raw['cube_pose']['edge_length_mm'] = 21
            elif kind == 'scaled_object_points':
                raw['cube_pose']['object_points_mm'] = [[1.05*v for v in point]
                    for point in raw['cube_pose']['object_points_mm']]
            elif kind == 'five_edges':raw['observed_edges'].pop()
            elif kind == 'unsupported_edge':
                raw['observed_edges'][0]['quality']['support'] = 44
            else:raw['half_corner_errors_px'][0] = 3.1
            with self.subTest(kind=kind):
                self.assert_rejected_without_coordinates(self.observe(raw, extrinsics))

    def test_base_center_and_cube_projection_are_recomputed_from_the_rigid_pose(self):
        raw, extrinsics, origin, yaw = fixture()
        raw['cube_center_Base_mm'] = [999., 998., 997.]
        raw['cube_pose']['cube_center_Camera_mm'] = [-1., -2., -3.]
        raw['cube_pose']['cube_projected_corners_px'] = [[0., 0.]]*8
        before = copy.deepcopy(raw)
        result = self.observe(raw, extrinsics)
        expected = [a+b for a, b in zip(origin, matvec(yaw, [0., 10., 10.]))]
        self.assertEqual(result['status'], 'localized', result)
        for actual, true in zip(result['cube_center_Base_mm'], expected):
            self.assertAlmostEqual(actual, true, places=8)
        pose = result['payload']['cube_pose']
        self.assertEqual(pose['cube_projection_source'], 'pose_transform')
        self.assertEqual(len(pose['cube_projected_corners_px']), 8)
        self.assertNotEqual(pose['cube_projected_corners_px'], [[0., 0.]]*8)
        self.assertTrue(pose['table_geometry']['accepted'])
        self.assertAlmostEqual(pose['table_geometry']['center_height_mm'], 10.)
        self.assertEqual(raw, before)

    def test_build_result_solves_both_five_corner_layouts_before_runtime_validation(self):
        for sign in (1, -1):
            raw, extrinsics, unused, unused = fixture(sign)
            measurement = {'object_points_mm': raw['cube_pose']['object_points_mm'],
                'image_points_px': raw['cube_pose']['image_points_px'],
                'observed_edges': raw['observed_edges'],
                'half_corner_errors_px': raw['half_corner_errors_px']}
            before = copy.deepcopy(measurement)
            built = cube.build_result(measurement, K, D,
                extrinsics['T_Base_from_Camera_mm'], PLANE)
            result = self.observe(built, extrinsics)
            with self.subTest(sign=sign):
                self.assertEqual(result['status'], 'localized', result)
                self.assertLess(result['quality']['reprojection_rms_px'], 1e-6)
                self.assertEqual(result['payload']['evidence_type'],
                    'nonplanar_cube_five_corners')
                self.assertEqual(measurement, before)

    def test_same_frame_association_uses_the_cube_projection_and_ignores_legacy_sides(self):
        raw, extrinsics, unused, unused = fixture()
        fine = self.observe(raw, extrinsics)
        # Poison unused legacy structures: the new evidence packet must use
        # its own normalized cube pose, rather than an old side-face audit.
        far = {'cube_projection_source': 'pose_transform',
               'cube_projected_corners_px': [[10., 10.], [20., 10.],
                   [20., 20.], [10., 20.]]*2}
        fine['payload']['side_faces'] = [{'used_for_center': True,
                                         'selected_pose': far}]
        fine['payload']['poses'] = [far]
        pose = fine['payload']['cube_pose']
        corners = pose['cube_projected_corners_px']
        xs, ys = [p[0] for p in corners], [p[1] for p in corners]
        bbox = [min(xs), min(ys), max(xs)-min(xs), max(ys)-min(ys)]
        cue = {'bbox_px': bbox, 'center_pixel': pose['cube_center_pixel'],
            'timestamp_ms': 1234, 'frame_id': FRAME['frame_id'],
            'visible': True, 'associated': True, 'ambiguous': False,
            'candidate_count': 1, 'distance_valid': False,
            'navigation_confirmed': False}
        coarse = dict(FRAME, timestamp_ms=1234, visual_evidence=cue,
            visual_candidates=[copy.deepcopy(cue)], bbox_px=bbox,
            center_pixel=cue['center_pixel'])
        result = associate_fine_candidate(fine, coarse)
        self.assertEqual(result['status'], 'matched', result)
        self.assertEqual(result['geometry_capability'], 'full_cube_corner_pose')
        self.assertEqual(len(result['used_pose_hulls']), 1)
        coarse['frame_id'] += 1
        self.assertEqual(associate_fine_candidate(fine, coarse)['reason'],
                         'source_frame_mismatch')

    def test_six_real_correspondences_keep_their_explicit_outline_policy_and_all_eight_projections(self):
        for sign in (1, -1):
            raw, extrinsics, unused, unused = fixture(sign, six=True)
            raw['cube_pose']['image_points_px'] = [[x+2.5, y] for x, y in
                raw['cube_pose']['image_points_px']]
            result = self.observe(raw, extrinsics)
            with self.subTest(sign=sign):
                self.assertEqual(result['status'], 'localized', result)
                self.assertEqual(result['quality']['evidence_type'], 'nonplanar_cube_six_corners')
                self.assertEqual(result['quality']['boundary_method'],
                                 'six_measured_outline_edges_six_corners')
                self.assertEqual(result['quality']['vision_path'], 'shared_nonplanar_cube_outline')
                self.assertAlmostEqual(result['quality']['reprojection_rms_px'], 2.5, places=8)
                self.assertEqual(result['quality']['reprojection_limit_px'], 3.)
                pose = result['payload']['cube_pose']
                self.assertEqual(len(pose['cube_projected_corners_px']), 8)
                self.assertLess(pose['table_geometry']['object_up_deviation_deg'], 1e-5)
                self.assertEqual(result['capture_timestamp_ms'], FRAME['capture_timestamp_ms'])

    def test_six_measurements_build_only_with_an_explicit_type_and_do_not_impersonate_five(self):
        for sign in (1, -1):
            raw, extrinsics, unused, unused = fixture(sign, six=True)
            measurement = {'evidence_type': raw['evidence_type'],
                'object_points_mm': raw['cube_pose']['object_points_mm'],
                'image_points_px': raw['cube_pose']['image_points_px'],
                'observed_edges': raw['observed_edges'],
                'half_corner_errors_px': raw['half_corner_errors_px']}
            before = copy.deepcopy(measurement)
            built = cube.build_result(measurement, K, D,
                extrinsics['T_Base_from_Camera_mm'], PLANE)
            self.assertEqual(built['evidence_type'], 'nonplanar_cube_six_corners')
            self.assertEqual(built['boundary_method'], 'six_measured_outline_edges_six_corners')
            self.assertLess(built['cube_pose']['corner_reprojection_rms_px'], 1e-6)
            self.assertEqual(measurement, before)
            del measurement['evidence_type']
            with self.assertRaises(ValueError):
                cube.build_result(measurement, K, D, extrinsics['T_Base_from_Camera_mm'], PLANE)

    def test_cube_policies_reject_cross_typed_count_method_path_and_interleaved_measurements(self):
        for six in (False, True):
            for changed in ('count', 'method', 'path', 'half_count', 'rms', 'order'):
                raw, extrinsics, unused, unused = fixture(six=six)
                pose = raw['cube_pose']
                if changed == 'count':
                    if six:
                        pose['object_points_mm'].pop(3);pose['image_points_px'].pop(3)
                    else:
                        pose['object_points_mm'].insert(3, [10., 20., 0.])
                        pose['image_points_px'].insert(3, [700., 500.])
                elif changed == 'method':raw['boundary_method'] = 'four_measured_edges'
                elif changed == 'path':raw['vision_path'] = 'shared_nonplanar_cube_corners' if six else 'shared_nonplanar_cube_outline'
                elif changed == 'half_count':raw['half_corner_errors_px'] = [.2]*(10 if six else 12)
                elif changed == 'rms':pose['image_points_px'] = [[x+3.1, y] for x, y in pose['image_points_px']]
                else:
                    pose['object_points_mm'][0], pose['object_points_mm'][1] = pose['object_points_mm'][1], pose['object_points_mm'][0]
                    pose['image_points_px'][0], pose['image_points_px'][1] = pose['image_points_px'][1], pose['image_points_px'][0]
                with self.subTest(six=six, changed=changed):
                    self.assert_rejected_without_coordinates(self.observe(raw, extrinsics))
        # The independent odd/even pair may be summarized once per corner.
        raw, extrinsics, unused, unused = fixture(six=True)
        raw['half_corner_errors_px'] = [.2]*6
        self.assertEqual(self.observe(raw, extrinsics)['status'], 'localized')

    def test_six_corner_table_check_requires_object_up_not_only_a_horizontal_front_normal(self):
        raw, extrinsics, unused, unused = fixture(six=True)
        pose = raw['cube_pose']
        original = pose['R_Object_to_Camera']
        center = [value+pose['t_Object_to_Camera_mm'][i] for i, value in
                  enumerate(matvec(original, [0., 10., 10.]))]
        for angle in (19., 21., 180.):
            tilted = matmul(original, rotation([0., 1., 0.], math.radians(angle)))
            pose['R_Object_to_Camera'] = tilted
            pose['t_Object_to_Camera_mm'] = [center[i]-value for i, value in
                enumerate(matvec(tilted, [0., 10., 10.]))]
            pose['image_points_px'] = projected(tilted, pose['t_Object_to_Camera_mm'],
                                                pose['object_points_mm'])
            result = self.observe(raw, extrinsics)
            with self.subTest(angle=angle):
                if angle == 19.:
                    self.assertEqual(result['status'], 'localized', result)
                    self.assertAlmostEqual(result['payload']['table_geometry']['object_up_deviation_deg'], 19.)
                else:self.assert_rejected_without_coordinates(result)

    def test_six_corner_association_uses_the_complete_cube_hull_in_the_same_frame(self):
        raw, extrinsics, unused, unused = fixture(six=True)
        fine = self.observe(raw, extrinsics)
        self.assertEqual(fine['status'], 'localized', fine)
        pose = fine['payload']['cube_pose']
        corners = pose['cube_projected_corners_px']
        xs, ys = [p[0] for p in corners], [p[1] for p in corners]
        bbox = [min(xs), min(ys), max(xs)-min(xs), max(ys)-min(ys)]
        cue = {'bbox_px': bbox, 'center_pixel': pose['cube_center_pixel'],
            'timestamp_ms': 1234, 'frame_id': FRAME['frame_id'],
            'visible': True, 'associated': True, 'ambiguous': False,
            'candidate_count': 1, 'distance_valid': False}
        coarse = dict(FRAME, timestamp_ms=1234, visual_evidence=cue,
            visual_candidates=[copy.deepcopy(cue)], bbox_px=bbox,
            center_pixel=cue['center_pixel'])
        fine['payload']['poses'] = [{'cube_projected_corners_px': [[0., 0.]]*8}]
        result = associate_fine_candidate(fine, coarse)
        self.assertEqual(result['status'], 'matched', result)
        self.assertEqual(result['geometry_capability'], 'full_cube_corner_pose')
        coarse['capture_timestamp_ms'] += 1
        self.assertEqual(associate_fine_candidate(fine, coarse)['reason'], 'source_frame_mismatch')

    def test_face_and_back_top_are_explicit_seven_edge_geometry_with_all_eight_projections(self):
        for sign in (1, -1):
            raw, extrinsics, origin, yaw = fixture(sign, face_back_top=True)
            raw['cube_pose']['image_points_px'] = [[x+2.5, y] for x, y in
                raw['cube_pose']['image_points_px']]
            before = copy.deepcopy(raw)
            result = self.observe(raw, extrinsics)
            with self.subTest(sign=sign):
                self.assertEqual(result['status'], 'localized', result)
                self.assertEqual(result['quality']['evidence_type'], FACE_BACK_TYPE)
                self.assertEqual(result['quality']['boundary_method'], FACE_BACK_METHOD)
                self.assertEqual(result['quality']['vision_path'], FACE_BACK_PATH)
                self.assertEqual(result['quality']['reprojection_limit_px'], 3.)
                self.assertAlmostEqual(result['quality']['reprojection_rms_px'], 2.5, places=8)
                self.assertEqual(result['capture_timestamp_ms'], FRAME['capture_timestamp_ms'])
                self.assertNotIn('poses', result['payload'])
                self.assertNotIn('face_quad_px', result['payload'])
                self.assertEqual(len(result['payload']['observed_edges']), 7)
                pose = result['payload']['cube_pose']
                self.assertEqual(pose['corner_count'], 6)
                self.assertEqual(len(pose['cube_projected_corners_px']), 8)
                expected = [a+b for a, b in zip(origin, matvec(yaw, [0., 10., 10.]))]
                for actual, true in zip(result['cube_center_Base_mm'], expected):
                    self.assertAlmostEqual(actual, true, places=8)
                self.assertLess(pose['table_geometry']['object_up_deviation_deg'], 1e-5)
                self.assertEqual(raw, before)

    def test_face_back_top_build_requires_explicit_type_without_guessing_from_six_points(self):
        for sign in (1, -1):
            raw, extrinsics, unused, unused = fixture(sign, face_back_top=True)
            measurement = {'evidence_type': FACE_BACK_TYPE,
                'object_points_mm': raw['cube_pose']['object_points_mm'],
                'image_points_px': raw['cube_pose']['image_points_px'],
                'observed_edges': raw['observed_edges'],
                'half_corner_errors_px': raw['half_corner_errors_px'],
                'corner_labels': raw['corner_labels']}
            before = copy.deepcopy(measurement)
            built = cube.build_result(measurement, K, D,
                extrinsics['T_Base_from_Camera_mm'], PLANE)
            with self.subTest(sign=sign):
                self.assertEqual(built['evidence_type'], FACE_BACK_TYPE)
                self.assertEqual(built['boundary_method'], FACE_BACK_METHOD)
                self.assertEqual(built['vision_path'], FACE_BACK_PATH)
                self.assertLess(built['cube_pose']['corner_reprojection_rms_px'], 1e-6)
                self.assertEqual(measurement, before)
                del measurement['evidence_type']
                with self.assertRaises(ValueError):
                    cube.build_result(measurement, K, D, extrinsics['T_Base_from_Camera_mm'], PLANE)

    def test_face_back_top_rejects_cross_type_six_sources_and_partial_half_evidence(self):
        for changed in ('type', 'method', 'path', 'model', 'six_sources', 'eight_sources',
                        'unsupported_seventh', 'six_halves', 'bad_half', 'actual_rms'):
            raw, extrinsics, unused, unused = fixture(face_back_top=True)
            if changed == 'type':raw['evidence_type'] = 'nonplanar_cube_six_corners'
            elif changed == 'method':raw['boundary_method'] = 'six_measured_outline_edges_six_corners'
            elif changed == 'path':raw['vision_path'] = 'shared_nonplanar_cube_outline'
            elif changed == 'model':
                other, unused, unused, unused = fixture(six=True)
                raw['cube_pose']['object_points_mm'] = other['cube_pose']['object_points_mm']
                raw['cube_pose']['image_points_px'] = other['cube_pose']['image_points_px']
            elif changed == 'six_sources':raw['observed_edges'].pop()
            elif changed == 'eight_sources':raw['observed_edges'].append(copy.deepcopy(raw['observed_edges'][0]))
            elif changed == 'unsupported_seventh':raw['observed_edges'][-1]['quality']['support'] = 44
            elif changed == 'six_halves':raw['half_corner_errors_px'] = [.2]*6
            elif changed == 'bad_half':raw['half_corner_errors_px'][-1] = 3.01
            else:raw['cube_pose']['image_points_px'] = [[x+3.01, y] for x, y in raw['cube_pose']['image_points_px']]
            with self.subTest(changed=changed):
                self.assert_rejected_without_coordinates(self.observe(raw, extrinsics))
        # Seven sources cannot be relabelled as the old six-source capability.
        raw, extrinsics, unused, unused = fixture(six=True)
        raw['observed_edges'].append(copy.deepcopy(raw['observed_edges'][0]))
        self.assert_rejected_without_coordinates(self.observe(raw, extrinsics))

    def test_face_back_top_requires_upright_table_geometry_and_exact_same_frame_association(self):
        raw, extrinsics, unused, unused = fixture(face_back_top=True)
        fine = self.observe(raw, extrinsics)
        self.assertEqual(fine['status'], 'localized', fine)
        pose = fine['payload']['cube_pose']
        corners = pose['cube_projected_corners_px']
        xs, ys = [p[0] for p in corners], [p[1] for p in corners]
        bbox = [min(xs), min(ys), max(xs)-min(xs), max(ys)-min(ys)]
        cue = {'bbox_px': bbox, 'center_pixel': pose['cube_center_pixel'],
            'timestamp_ms': 1234, 'frame_id': FRAME['frame_id'],
            'visible': True, 'associated': True, 'ambiguous': False,
            'candidate_count': 1, 'distance_valid': False}
        coarse = dict(FRAME, timestamp_ms=1234, visual_evidence=cue,
            visual_candidates=[copy.deepcopy(cue)], bbox_px=bbox,
            center_pixel=cue['center_pixel'])
        matched = associate_fine_candidate(fine, coarse)
        self.assertEqual(matched['status'], 'matched', matched)
        self.assertEqual(matched['geometry_capability'], 'full_cube_corner_pose')
        coarse['frame_id'] += 1
        self.assertEqual(associate_fine_candidate(fine, coarse)['reason'], 'source_frame_mismatch')
        original = raw['cube_pose']['R_Object_to_Camera']
        center = [value+raw['cube_pose']['t_Object_to_Camera_mm'][i] for i, value in
                  enumerate(matvec(original, [0., 10., 10.]))]
        tilted = matmul(original, rotation([0., 1., 0.], math.radians(21.)))
        raw['cube_pose']['R_Object_to_Camera'] = tilted
        raw['cube_pose']['t_Object_to_Camera_mm'] = [center[i]-value for i, value in
            enumerate(matvec(tilted, [0., 10., 10.]))]
        raw['cube_pose']['image_points_px'] = projected(tilted,
            raw['cube_pose']['t_Object_to_Camera_mm'], raw['cube_pose']['object_points_mm'])
        self.assert_rejected_without_coordinates(self.observe(raw, extrinsics))


if __name__ == '__main__':
    unittest.main()
