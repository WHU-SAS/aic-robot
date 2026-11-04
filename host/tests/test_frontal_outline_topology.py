"""Image-only contour repair, without surveyed pose or per-image offsets."""
import importlib.util
import json
import unittest

from host.project_paths import ROOT

HAS_VISION = importlib.util.find_spec('cv2') is not None
if HAS_VISION:
    import cv2
    from device.k230.vision import cube_corners, cube_outline, physical_side as p


@unittest.skipUnless(HAS_VISION, 'Dedicated local CV environment')
class FrontalOutlineTopologyTests(unittest.TestCase):
    def test_adjacent_split_back_edge_is_only_a_collinear_proposal(self):
        polygon = [[61, 24], [153, 27], [209, 32], [219, 310], [24, 310]]
        box = [24, 24, 195, 300]
        with self.assertRaises(ValueError):
            cube_corners._upper_back_proposal(polygon, box)
        a, b, evidence = cube_corners._merged_upper_back_proposal(p, polygon, box)
        self.assertEqual((a, b), ([61, 24], [209, 32]))
        self.assertEqual(evidence['contour_run_px'], polygon[:3])
        self.assertLessEqual(evidence['max_contour_line_error_px'], 3.)

    def test_bent_upper_contour_is_not_joined_into_an_edge(self):
        polygon = [[61, 24], [153, 60], [209, 32], [219, 310], [24, 310]]
        with self.assertRaises(ValueError):
            cube_corners._merged_upper_back_proposal(p, polygon, [24, 24, 195, 300])

    def saved_outline(self, number):
        directory = ROOT/'data/vision/lit_validation/20261104_100443_212133'
        paths = [directory/'config.json', directory/('frame_%02d.json'%number)]
        if any(not v.is_file() for v in paths):
            self.skipTest('Retained full-camera development image unavailable')
        cfg = json.loads(paths[0].read_text())['car']
        meta = json.loads(paths[1].read_text())
        image = cv2.imread(str(directory/meta['image_file']))
        self.assertIsNotNone(image)
        prepared = cube_corners.prepare(image, meta['bbox_px'])
        return list(cube_outline.measure_steps(image, meta['bbox_px'],
            cfg['intrinsics'], cfg['extrinsics'], cfg['table_plane'], prepared=prepared))

    def test_measured_same_wall_split_cannot_become_a_sixth_cube_corner(self):
        events = self.saved_outline(4)
        self.assertEqual(sum(v['stage']=='cube_outline_edge_complete' for v in events), 6)
        self.assertFalse(any(v['stage']=='cube_candidate' for v in events))
        self.assertEqual([v['reason'] for v in events if v['stage']=='cube_outline_rejected'],
            ['Cube outline has no distinct measured six-edge topology'])

    def test_real_six_distinct_measured_edges_keep_the_outline_candidate(self):
        events = self.saved_outline(5)
        candidates = [v['measurement'] for v in events if v['stage']=='cube_candidate']
        self.assertEqual(len(candidates), 1)
        self.assertEqual(candidates[0]['evidence_type'], 'nonplanar_cube_six_corners')
        self.assertEqual(len(candidates[0]['observed_edges']), 6)
        self.assertTrue(all(v<=3 for v in candidates[0]['half_corner_errors_px']))


if __name__ == '__main__':
    unittest.main()
