"""Offline footprint association; these pixels are not physical XYZ truth."""
import copy
import unittest

from device.k230.vision.candidate_association import associate_fine_candidate


def cube_pose(x=0., y=0.):
    # First four are the measured side; the other four complete its 20mm cube.
    return {'cube_projection_source': 'pose_transform',
        'cube_projected_corners_px': [[x+a, y+b] for a, b in
            ((0, 20), (40, 20), (40, 60), (0, 60),
             (5, 0), (35, 0), (35, 40), (5, 40))]}


def cue(center=(20., 10.), bbox=(0, 0, 40, 60), stamp=100):
    return {'bbox_px': list(bbox), 'center_pixel': list(center),
            'timestamp_ms': stamp, 'distance_valid': True}


def pair():
    source = cue()
    coarse = {'frame_id': 7, 'capture_timestamp_ms': 100,
        'clock_domain': 'k230_ticks_ms', 'image_size_px': [1280, 960], 'status': 'confirming',
        'visual_evidence': dict(source, visible=True, associated=True,
            ambiguous=False, candidate_count=1, navigation_confirmed=False),
        'visual_candidates': [source]}
    fine = {'frame_id': 7, 'capture_timestamp_ms': 100,
        'clock_domain': 'k230_ticks_ms', 'image_size_px': [1280, 960], 'status': 'localized',
        'payload': {'side_faces': [{'used_for_center': True,
            'selected_pose': cube_pose()}], 'poses': [cube_pose(500)]}}
    return fine, coarse


class CandidateAssociationTests(unittest.TestCase):
    def test_pixel_space_and_selected_source_cannot_be_mixed(self):
        for fault in ('image_size', 'clock', 'negative_time', 'top_center'):
            with self.subTest(fault=fault):
                fine,coarse=pair()
                if fault=='image_size':fine['image_size_px']=[640,480]
                elif fault=='clock':fine['clock_domain']=coarse['clock_domain']=None
                elif fault=='negative_time':
                    fine['capture_timestamp_ms']=coarse['capture_timestamp_ms']=-1
                else:coarse['center_pixel']=[900.,500.]
                self.assertEqual(associate_fine_candidate(fine,coarse)['status'],'unknown')

    def test_top_only_source_outside_measured_face_inside_full_cube(self):
        fine, coarse = pair()
        self.assertLess(coarse['visual_evidence']['center_pixel'][1],
                        fine['payload']['side_faces'][0]['selected_pose']['cube_projected_corners_px'][0][1])
        result = associate_fine_candidate(fine, coarse)
        self.assertEqual(result['status'], 'matched')
        self.assertEqual(result['geometry_capability'], 'full_pose')
        self.assertTrue(result['used_pose_hulls'][0]['supports_source'])

    def test_separated_fine_b_cannot_bind_to_source_a(self):
        fine, coarse = pair()
        fine['payload']['side_faces'][0]['selected_pose'] = cube_pose(200)
        result = associate_fine_candidate(fine, coarse)
        self.assertEqual((result['status'], result['reason']),
                         ('unknown', 'no_used_pose_support'))

    def test_every_used_pose_must_support_not_just_one(self):
        fine, coarse = pair()
        fine['payload']['side_faces'].append({'used_for_center': True,
                                             'selected_pose': cube_pose(200)})
        result = associate_fine_candidate(fine, coarse)
        self.assertEqual(result['reason'], 'partial_used_pose_support')
        self.assertEqual([x['supports_source'] for x in result['used_pose_hulls']], [True, False])

    def test_unused_side_and_aggregate_cannot_supply_support(self):
        fine, coarse = pair()
        fine['payload']['side_faces'] = [
            {'used_for_center': True, 'selected_pose': cube_pose(200),
             'poses': [cube_pose()]},
            {'used_for_center': False, 'selected_pose': cube_pose()}]
        fine['payload']['poses'] = [cube_pose()]
        self.assertEqual(associate_fine_candidate(fine, coarse)['status'], 'unknown')
        fine['payload']['side_faces'] = []
        self.assertEqual(associate_fine_candidate(fine, coarse)['reason'], 'missing_used_pose_geometry')

    def test_independent_single_face_uses_its_selected_first_pose(self):
        fine, coarse = pair()
        fine['payload'] = {'poses': [cube_pose(), cube_pose(500)]}
        self.assertEqual(associate_fine_candidate(fine, coarse)['status'], 'matched')
        fine['payload']['poses'].reverse()
        self.assertEqual(associate_fine_candidate(fine, coarse)['status'], 'unknown')

    def test_moving_range_uses_selected_faces_never_raw_audit(self):
        fine, coarse = pair()
        fine.update(status='no_complete_face', range_candidate={
            'status': 'range_candidate', 'selected_faces': [{'used_for_center': True,
                'selected_pose': cube_pose(200)}],
            'audit': [{'poses': [cube_pose()]}]})
        fine['payload'] = {'side_faces': [{'used_for_center': True, 'selected_pose': cube_pose()}]}
        result = associate_fine_candidate(fine, coarse)
        self.assertEqual(result['geometry_capability'], 'moving_range')
        self.assertEqual(result['status'], 'unknown')
        fine['range_candidate']['selected_faces'][0]['selected_pose'] = cube_pose()
        self.assertEqual(associate_fine_candidate(fine, coarse)['status'], 'matched')
        del fine['range_candidate']['selected_faces']
        self.assertEqual(associate_fine_candidate(fine, coarse)['status'], 'unknown')

    def test_invalid_distance_and_tentative_navigation_can_match_fine(self):
        fine, coarse = pair()
        coarse['status'] = 'no_target'
        coarse['visual_evidence'].update(distance_valid=False, navigation_confirmed=False)
        coarse['visual_candidates'][0]['distance_valid'] = False
        result = associate_fine_candidate(fine, coarse)
        self.assertEqual(result['status'], 'matched')
        self.assertIs(result['source_candidate']['distance_valid'], False)
        self.assertIs(coarse['visual_evidence']['navigation_confirmed'], False)

    def test_complete_pool_far_cue_does_not_reject(self):
        fine, coarse = pair()
        coarse['visual_candidates'].append(cue((500, 500), (480, 470, 40, 60)))
        coarse['visual_evidence']['candidate_count'] = 2
        self.assertEqual(associate_fine_candidate(fine, coarse)['status'], 'matched')

    def test_independent_cue_in_fine_projection_is_ambiguous(self):
        fine, coarse = pair()
        coarse['visual_candidates'].append(cue((30, 35), (20, 10, 30, 50)))
        coarse['visual_evidence']['candidate_count'] = 2
        result = associate_fine_candidate(fine, coarse)
        self.assertEqual(result['reason'], 'competing_candidate_in_fine_projection')

    def test_strictly_contained_fragment_does_not_compete(self):
        fine, coarse = pair()
        coarse['visual_candidates'].append(cue((20, 25), (10, 15, 20, 20)))
        coarse['visual_evidence']['candidate_count'] = 2
        self.assertEqual(associate_fine_candidate(fine, coarse)['status'], 'matched')

    def test_missing_incomplete_or_stale_pool_is_unknown(self):
        for mutation in ('missing', 'count', 'stale', 'source_absent', 'invalid_cue'):
            with self.subTest(mutation=mutation):
                fine, coarse = pair()
                if mutation == 'missing': del coarse['visual_candidates']
                if mutation == 'count': coarse['visual_evidence']['candidate_count'] = 2
                if mutation == 'stale': coarse['visual_candidates'][0]['timestamp_ms'] = 99
                if mutation == 'source_absent': coarse['visual_candidates'][0]['center_pixel'] = [21, 10]
                if mutation == 'invalid_cue': del coarse['visual_candidates'][0]['bbox_px']
                self.assertEqual(associate_fine_candidate(fine, coarse)['status'], 'unknown')

    def test_ambiguous_unassociated_or_wrong_frame_never_matches(self):
        for mutation in ('ambiguous', 'invisible', 'unassociated', 'frame', 'stamp', 'evidence_stamp', 'clock'):
            with self.subTest(mutation=mutation):
                fine, coarse = pair()
                if mutation == 'ambiguous': coarse['visual_evidence']['ambiguous'] = True
                if mutation == 'invisible': coarse['visual_evidence']['visible'] = False
                if mutation == 'unassociated': coarse['visual_evidence']['associated'] = False
                if mutation == 'frame': coarse['frame_id'] = 8
                if mutation == 'stamp': coarse['capture_timestamp_ms'] = 99
                if mutation == 'evidence_stamp': coarse['visual_evidence']['timestamp_ms'] = 99
                if mutation == 'clock': coarse['clock_domain'] = 'other'
                self.assertEqual(associate_fine_candidate(fine, coarse)['status'], 'unknown')

    def test_missing_or_bad_projection_never_falls_back_to_face_quad(self):
        for mutation in ('missing', 'source', 'corners', 'nonfinite', 'degenerate'):
            with self.subTest(mutation=mutation):
                fine, coarse = pair()
                selected = fine['payload']['side_faces'][0]
                selected['face_quad_px'] = [[0, 0], [40, 0], [40, 60], [0, 60]]
                pose = selected['selected_pose']
                if mutation == 'missing': del pose['cube_projected_corners_px']
                if mutation == 'source': pose['cube_projection_source'] = 'invented'
                if mutation == 'corners': pose['cube_projected_corners_px'].pop()
                if mutation == 'nonfinite': pose['cube_projected_corners_px'][0][0] = float('nan')
                if mutation == 'degenerate': pose['cube_projected_corners_px'] = [[1, 1]]*8
                self.assertEqual(associate_fine_candidate(fine, coarse)['status'], 'unknown')

    def test_tolerance_is_distance_to_hull_not_expanded_corner_box(self):
        fine, coarse = pair()
        pose = cube_pose()
        pose['cube_projected_corners_px'] = [[x, y] for x, y in
            ((0, 0), (40, 0), (40, 60), (0, 60))*2]
        fine['payload']['side_faces'][0]['selected_pose'] = pose
        for point, expected in (((-1.9, 20), 'matched'), ((-2.1, 20), 'unknown'),
                                ((-1.9, -1.9), 'unknown')):
            coarse['visual_evidence']['center_pixel'] = list(point)
            coarse['visual_candidates'][0]['center_pixel'] = list(point)
            self.assertEqual(associate_fine_candidate(fine, coarse)['status'], expected)

    def test_inputs_unchanged_and_result_owns_audit_pixels(self):
        fine, coarse = pair()
        before = copy.deepcopy((fine, coarse))
        result = associate_fine_candidate(fine, coarse)
        self.assertEqual((fine, coarse), before)
        result['source_candidate']['center_pixel'][0] = -999
        result['used_pose_hulls'][0]['hull_px'][0][0] = -999
        self.assertEqual((fine, coarse), before)


if __name__ == '__main__':
    unittest.main()
