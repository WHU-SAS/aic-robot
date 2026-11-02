"""Current blue evidence survives unavailable tabletop geometry and association."""
import copy
import unittest

from device.k230.car.blue_tracker import BlueTracker
from device.k230.car.observations import for_approach
from device.k230.vision.runtime import track_coarse


class Blob:
    def __init__(self, box, pixels=None):
        self.box = list(box)
        self.count = pixels if pixels is not None else box[2]*box[3]

    def rect(self): return self.box
    def pixels(self): return self.count
    def cx(self): return self.box[0]+(self.box[2]-1)/2
    def cy(self): return self.box[1]+(self.box[3]-1)/2


class Image:
    def __init__(self, *blobs):
        self.blobs, self.calls = blobs, 0

    def find_blobs(self, *args, **kwargs):
        self.calls += 1
        return self.blobs


class Projection:
    def __init__(self, ratio=1):
        self.ratio = ratio

    def blob_geometry(self, box):
        return {'ground_contact_proxy_Base_mm': [box[0]+200., 0., 0.],
                'expected_vertical_height_px': box[3]/self.ratio,
                'observed_to_expected_height_ratio': self.ratio}


class VisualEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.config = {'image_size_px': [1280, 960], 'lab_thresholds': [[0, 100, -10, 127, -128, -35]],
            'min_pixels': 25, 'min_area_px2': 25, 'max_aspect_ratio': 4,
            'min_fill_ratio': .2, 'forward_range_mm': [150, 1500],
            'height_ratio_range': [.4, 2.5], 'confirmation_frames': 3,
            'max_age_ms': 500, 'max_center_jump_fraction': .15, 'max_size_ratio': 3}
        self.tracker = BlueTracker(self.config, Projection())
        self.frame = {'frame_id': 8, 'capture_timestamp_ms': 100,
            'timestamp_ms': 100, 'clock_domain': 'k230_ticks_ms', 'image_size_px': [1280, 960]}

    def test_valid_measurement_retains_old_confirmation_and_fresh_evidence(self):
        image = Image(Blob([300, 400, 40, 40]))
        for stamp in (0, 50, 100):
            result = self.tracker.detect_frame(image, stamp)
        self.assertEqual((result['status'], image.calls), ('tracking', 3))
        evidence = result['visual_evidence']
        self.assertTrue(evidence['associated'])
        self.assertTrue(evidence['distance_valid'])
        self.assertEqual(evidence['ground_contact_proxy_Base_mm'], result['ground_contact_proxy_Base_mm'])
        self.assertEqual(evidence['timestamp_ms'], 100)

    def test_clipped_and_unmeasurable_blue_still_supply_current_image_evidence(self):
        cases = [(Blob([0, 400, 40, 40]), 'clipped_blob', ['left']),
                 (Blob([1240, 400, 40, 40]), 'clipped_blob', ['right']),
                 (Blob([300, 0, 40, 40]), 'clipped_blob', ['top']),
                 (Blob([300, 920, 40, 40]), 'clipped_blob', ['bottom']),
                 (Blob([300, 400, 40, 40], 25), 'sparse_blob', []),
                 (Blob([300, 400, 100, 3]), 'implausible_aspect_ratio', [])]
        for blob, reason, edges in cases:
            with self.subTest(reason=reason, edges=edges):
                self.tracker.reset()
                image = Image(blob)
                result = self.tracker.detect_frame(image, 100)
                evidence = result['visual_evidence']
                self.assertEqual((result['status'], image.calls), ('no_target', 1))
                self.assertTrue(evidence['visible'])
                self.assertTrue(evidence['associated'])
                self.assertEqual((evidence['reason'], evidence['edge_directions']), (reason, edges))
                self.assertFalse(evidence['distance_valid'])
                self.assertNotIn('ground_contact_proxy_Base_mm', evidence)
                self.assertNotIn('ground_contact_proxy_Base_mm', result)

    def test_suspected_airborne_height_does_not_publish_its_table_intersection(self):
        tracker = BlueTracker(self.config, Projection(ratio=4))
        result = tracker.detect_frame(Image(Blob([300, 400, 40, 40])), 100)
        evidence = result['visual_evidence']
        self.assertEqual(result['status'], 'no_target')
        self.assertTrue(evidence['visible'])
        self.assertEqual(evidence['reason'], 'inconsistent_with_20mm_height')
        self.assertFalse(evidence['distance_valid'])
        self.assertNotIn('ground_contact_proxy_Base_mm', evidence)

    def test_missing_frame_keeps_no_old_selected_image_or_geometry(self):
        self.tracker.detect_frame(Image(Blob([300, 400, 40, 40])), 0)
        for stamp, status in ((50, 'lost'), (100, 'no_target')):
            result = self.tracker.detect_frame(Image(), stamp)
            evidence = result['visual_evidence']
            self.assertEqual(result['status'], status)
            self.assertEqual(evidence['timestamp_ms'], stamp)
            self.assertFalse(evidence['visible'])
            self.assertFalse(evidence['distance_valid'])
            self.assertNotIn('center_pixel', evidence)
            self.assertNotIn('ground_contact_proxy_Base_mm', evidence)

    def test_associated_target_beats_larger_unrelated_blue_candidate(self):
        target = Blob([300, 400, 40, 40])
        self.tracker.detect_frame(Image(target), 0)
        result = self.tracker.detect_frame(Image(Blob([900, 400, 80, 80]),
                                                 Blob([303, 401, 40, 40])), 100)
        evidence = result['visual_evidence']
        self.assertTrue(evidence['associated'])
        self.assertFalse(evidence['ambiguous'])
        self.assertEqual(evidence['bbox_px'], [303, 401, 40, 40])
        self.assertEqual(evidence['candidate_count'], 2)

    def test_geometry_rejection_does_not_switch_association_to_another_target(self):
        self.tracker.detect_frame(Image(Blob([5, 400, 40, 40])), 0)
        result = self.tracker.detect_frame(Image(Blob([0, 400, 35, 40]),
                                                 Blob([900, 400, 40, 40])), 100)
        evidence = result['visual_evidence']
        self.assertEqual(result['status'], 'lost')
        self.assertTrue(evidence['associated'])
        self.assertEqual(evidence['bbox_px'][0], 0)
        self.assertEqual(evidence['reason'], 'clipped_blob')
        self.assertFalse(evidence['distance_valid'])

    def test_ambiguous_acquisition_does_not_claim_largest_blob_is_current_target(self):
        result = self.tracker.detect_frame(Image(Blob([300, 400, 40, 40]),
                                                 Blob([900, 400, 80, 80])), 100)
        # The legacy API remains available, while the persistent task sees
        # explicit ambiguity and must not treat the old maximum-area choice as identity.
        self.assertEqual(result['status'], 'confirming')
        evidence = result['visual_evidence']
        self.assertTrue(evidence['visible'])
        self.assertTrue(evidence['ambiguous'])
        self.assertFalse(evidence['associated'])
        self.assertNotIn('center_pixel', evidence)
        self.assertNotIn('ground_contact_proxy_Base_mm', evidence)

    def test_unique_jump_remains_visible_without_immediate_target_reassignment(self):
        self.tracker.detect_frame(Image(Blob([300, 400, 40, 40])), 0)
        image = Image(Blob([900, 400, 40, 40]))
        result = self.tracker.detect_frame(image, 100)
        evidence = result['visual_evidence']
        self.assertEqual(result['status'], 'lost')
        self.assertFalse(evidence['associated'])
        self.assertFalse(evidence['ambiguous'])
        self.assertEqual(evidence['bbox_px'][0], 900)
        self.assertTrue(evidence['distance_valid'])
        self.assertEqual(evidence['ground_contact_proxy_Base_mm'][0], 1100)
        self.assertEqual(evidence['reason'], 'unassociated_blue')
        # Reacquisition does not carry old identity forever; reset is available
        # once the task has confirmed a new target version.
        self.tracker.reset()
        self.assertTrue(self.tracker.detect_frame(image, 150)['visual_evidence']['associated'])

    def test_multiple_plausible_matches_are_exposed_as_ambiguous(self):
        self.tracker.detect_frame(Image(Blob([300, 400, 40, 40])), 0)
        result = self.tracker.detect_frame(Image(Blob([290, 400, 40, 40]),
                                                 Blob([310, 400, 40, 40])), 100)
        self.assertTrue(result['visual_evidence']['ambiguous'])
        self.assertNotIn('center_pixel', result['visual_evidence'])

    def test_current_evidence_survives_shared_and_car_adapters_without_old_coordinates(self):
        result = track_coarse(Image(Blob([0, 400, 40, 40])), self.frame,
                              self.tracker.detect_frame)
        normalized = for_approach(result, 'coarse')
        self.assertEqual(result['status'], 'no_target')
        self.assertTrue(normalized['visual_evidence']['visible'])
        self.assertEqual(normalized['visual_evidence']['edge_directions'], ['left'])
        self.assertNotIn('ground_contact_proxy_Base_mm', normalized)
        self.assertNotIn('cube_center_Base_mm', normalized)
        self.assertNotIn('ground_contact_proxy_Base_mm', normalized['visual_evidence'])

    def test_evidence_cannot_refresh_old_frame_or_claim_precise_coordinates(self):
        for evidence in ({'timestamp_ms': 99},
                         {'timestamp_ms': 100, 'cube_center_Base_mm': [1, 2, 3]}):
            raw = {'status': 'lost', 'timestamp_ms': 100, 'visual_evidence': evidence}
            with self.assertRaises(ValueError):
                track_coarse(None, self.frame, lambda *args: raw)
            envelope = dict(self.frame, schema='aic.vision_observation.v1', mode='coarse',
                            status='lost', visual_evidence=evidence)
            with self.assertRaises(ValueError):
                for_approach(envelope, 'coarse')

    def test_unavailable_distance_is_removed_without_mutating_raw_evidence(self):
        evidence = {'timestamp_ms': 100, 'visible': True, 'associated': True,
            'distance_valid': False, 'ground_contact_proxy_Base_mm': [1, 2, 3]}
        original = copy.deepcopy(evidence)
        result = track_coarse(None, self.frame, lambda *args:
            {'status': 'lost', 'visual_evidence': evidence})
        self.assertNotIn('ground_contact_proxy_Base_mm', result['visual_evidence'])
        self.assertNotIn('ground_contact_proxy_Base_mm', for_approach(result, 'coarse')['visual_evidence'])
        self.assertEqual(evidence, original)


if __name__ == '__main__':
    unittest.main()
