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
        self.assertEqual(evidence['confirmations'],3)
        self.assertEqual(evidence['association_confirmations'],3)
        self.assertTrue(evidence['navigation_confirmed'])

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
                self.assertEqual(evidence['confirmations'],0)
                self.assertFalse(evidence['navigation_confirmed'])
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
        self.assertEqual(result['status'], 'no_target')
        self.assertNotIn('ground_contact_proxy_Base_mm',result)
        evidence = result['visual_evidence']
        self.assertTrue(evidence['visible'])
        self.assertTrue(evidence['ambiguous'])
        self.assertFalse(evidence['associated'])
        self.assertNotIn('center_pixel', evidence)
        self.assertNotIn('ground_contact_proxy_Base_mm', evidence)

    def test_selected_A_evidence_never_authorizes_larger_B_coordinates(self):
        a,b=Blob([300,400,40,40]),Blob([900,400,80,80])
        # Formerly the ambiguous frame secretly seeded legacy B, then its
        # independently selected coordinates could accompany A's evidence.
        self.tracker.detect_frame(Image(a,b),0)
        self.tracker.detect_frame(Image(a),50)
        for stamp in (100,150):
            result=self.tracker.detect_frame(Image(a,b),stamp)
            evidence=result['visual_evidence']
            for key in ('bbox_px','center_pixel','pixels','ground_contact_proxy_Base_mm','confirmations'):
                self.assertEqual(result[key],evidence[key],key)
            self.assertEqual(result['bbox_px'],a.rect())
            self.assertEqual(result['ground_contact_proxy_Base_mm'][0],500)
        self.assertEqual(result['status'],'tracking')

    def test_ambiguity_breaks_confirmation_and_one_returning_frame_cannot_track(self):
        a=Blob([300,400,40,40])
        for stamp in (0,50,100):self.tracker.detect_frame(Image(a),stamp)
        result=self.tracker.detect_frame(Image(a,Blob([310,400,40,40])),150)
        self.assertEqual(result['status'],'lost')
        self.assertTrue(result['visual_evidence']['ambiguous'])
        self.assertFalse(result['visual_evidence']['navigation_confirmed'])
        self.assertNotIn('ground_contact_proxy_Base_mm',result)
        for stamp,count,status in ((200,1,'confirming'),(250,2,'confirming'),(300,3,'tracking')):
            result=self.tracker.detect_frame(Image(a),stamp)
            self.assertEqual((result['status'],result['confirmations']),(status,count))
            self.assertEqual(result['visual_evidence']['confirmations'],count)

    def test_loss_jump_and_invalid_geometry_break_the_same_confirmation_chain(self):
        for interruption in ('missing','unassociated','geometry'):
            with self.subTest(interruption=interruption):
                self.setUp();a=Blob([300,400,40,40]);b=Blob([900,400,40,40])
                for stamp in (0,50,100):self.tracker.detect_frame(Image(a),stamp)
                if interruption=='geometry':self.tracker.projection.ratio=4
                current=Image(b) if interruption=='unassociated' else Image(a)
                self.tracker.detect_frame(Image() if interruption=='missing' else current,150)
                self.tracker.projection.ratio=1
                result=self.tracker.detect_frame(current,200)
                self.assertEqual((result['status'],result['confirmations']),('confirming',1))
                self.assertFalse(result['visual_evidence']['navigation_confirmed'])

    def test_expired_repeated_and_backward_timestamps_do_not_reuse_confirmation(self):
        for stamp in (1000,100,50):
            with self.subTest(stamp=stamp):
                self.setUp();image=Image(Blob([300,400,40,40]))
                for before in (0,50,100):self.tracker.detect_frame(image,before)
                result=self.tracker.detect_frame(image,stamp)
                self.assertEqual((result['status'],result['confirmations']),('confirming',1))
                self.assertFalse(result['visual_evidence']['navigation_confirmed'])

    def test_strictly_contained_measurable_patch_keeps_actual_outer_measurement(self):
        outer,inner=Blob([442,159,75,84],3016),Blob([471,210,35,25],331)
        for stamp in (0,50,100):result=self.tracker.detect_frame(Image(outer,inner),stamp)
        self.assertEqual(result['status'],'tracking')
        self.assertEqual(result['bbox_px'],outer.rect())
        self.assertEqual(result['pixels'],3016)
        self.assertEqual(len(result['candidates']),2)
        self.assertEqual(result['visual_evidence']['candidate_count'],2)
        self.assertFalse(result['visual_evidence']['ambiguous'])

    def test_invalid_outer_cue_cannot_suppress_valid_contained_cube(self):
        class MixedProjection(Projection):
            def blob_geometry(self,box):
                return Projection(ratio=4 if box[2]==75 else 1).blob_geometry(box)
        self.tracker=BlueTracker(self.config,MixedProjection())
        result=self.tracker.detect_frame(Image(Blob([442,159,75,84],3016),
            Blob([471,210,35,25],331)),100)
        self.assertEqual(result['bbox_px'],[471,210,35,25])
        self.assertEqual(result['visual_evidence']['bbox_px'],result['bbox_px'])
        self.assertFalse(result['visual_evidence']['ambiguous'])

    def test_invalid_inner_fragment_cannot_interrupt_its_unique_measurable_outer(self):
        outer=Blob([442,159,75,84],3016)
        fragment=Blob([460,180,40,40],25)
        for stamp in range(0,500,50):
            result=self.tracker.detect_frame(Image(outer,fragment),stamp)
            self.assertFalse(result['visual_evidence']['ambiguous'])
            self.assertEqual(result['bbox_px'],outer.rect())
            if stamp>=100:self.assertEqual(result['status'],'tracking')

    def test_native_stopped_record_keeps_outer_tracking_for_ten_frames(self):
        # Frozen native bbox/centroid/pixel/geometry from stopped_static.json,
        # not PC re-segmentation. Rejected records contain bbox/reason only;
        # placeholder pixel counts there exercise selection, not segmentation.
        measured=[([442,159,75,84],[468.,192.],3016,[848.0721,117.8427,-102.5591],52.1185,1.611712),
                  ([471,210,35,25],[488.,223.],331,[881.6261,118.1497,-104.4201],50.09207,.499081)]
        outside=[[78,81,12,4],[68,103,7,8],[32,114,9,10],[17,116,7,6],[60,117,8,7]]
        height=[[147,165,20,5],[495,232,12,8],[1235,618,31,8],[1222,621,9,7],
                [1217,624,10,9],[1193,630,10,7],[1180,631,13,13],[1203,634,7,6],
                [1155,642,11,8],[1123,649,26,11],[1109,654,11,7],[1090,660,12,5],
                [1084,664,20,9],[1068,667,19,6],[1035,673,11,5],[1019,674,13,5],
                [1050,673,10,8],[1031,680,11,4],[921,685,10,9]]
        class RecordedBlob(Blob):
            def __init__(self,row):super().__init__(row[0],row[2]);self.center=row[1]
            def cx(self):return self.center[0]
            def cy(self):return self.center[1]
        class RecordedProjection(Projection):
            def blob_geometry(self,box):
                for row in measured:
                    if box==row[0]:
                        return dict(ground_contact_proxy_Base_mm=row[3],
                            expected_vertical_height_px=row[4],observed_to_expected_height_ratio=row[5])
                return dict(ground_contact_proxy_Base_mm=[2000. if box in outside else 500.,0.,0.],
                    expected_vertical_height_px=box[3]/4,observed_to_expected_height_ratio=4.)
        tracker=BlueTracker(self.config,RecordedProjection())
        blobs=[RecordedBlob(row) for row in measured]+[Blob([0,96,17,27])]+[
            Blob(box) for box in outside+height]
        self.assertEqual(len(blobs),27)
        for count in range(1,11):
            result=tracker.detect_frame(Image(*blobs),count*50)
            evidence=result['visual_evidence']
            self.assertEqual(result['bbox_px'],measured[0][0])
            self.assertEqual(result['center_pixel'],measured[0][1])
            self.assertEqual(result['ground_contact_proxy_Base_mm'],measured[0][3])
            self.assertEqual(result['pixels'],3016)
            self.assertEqual((result['confirmations'],evidence['confirmations']),(count,count))
            self.assertEqual(evidence['association_confirmations'],count)
            self.assertEqual(evidence['candidate_count'],27)
            self.assertEqual(len(result['candidates']),2)
            self.assertFalse(evidence['ambiguous'])
            self.assertEqual(result['status'],'tracking' if count>=3 else 'confirming')

    def test_independent_or_partly_overlapping_objects_remain_ambiguous(self):
        for other in ([900,400,80,80],[310,410,40,40]):
            with self.subTest(other=other):
                self.tracker.reset()
                for stamp in (0,50,100):
                    result=self.tracker.detect_frame(Image(Blob([300,400,40,40]),Blob(other)),stamp)
                    self.assertEqual(result['status'],'no_target')
                    self.assertTrue(result['visual_evidence']['ambiguous'])
                    self.assertFalse(result['visual_evidence']['navigation_confirmed'])

    def test_tentative_geometry_can_confirm_association_without_navigation_authority(self):
        image=Image(Blob([0,400,40,40]))
        for stamp,count in ((0,1),(50,2),(100,3)):
            result=self.tracker.detect_frame(image,stamp)
            evidence=result['visual_evidence']
            self.assertEqual(evidence['association_confirmations'],count)
            self.assertEqual(evidence['confirmations'],0)
            self.assertFalse(evidence['navigation_confirmed'])
            self.assertNotIn('ground_contact_proxy_Base_mm',evidence)
        self.tracker.detect_frame(Image(),150)
        result=self.tracker.detect_frame(image,200)
        self.assertEqual(result['visual_evidence']['association_confirmations'],1)

    def test_canonical_confirmation_survives_shared_and_car_adapters(self):
        for number,stamp in enumerate((0,50,100),1):
            frame=dict(self.frame,frame_id=number,capture_timestamp_ms=stamp,timestamp_ms=stamp)
            result=for_approach(track_coarse(Image(Blob([300,400,40,40])),frame,
                self.tracker.detect_frame),'coarse')
            self.assertEqual(result['confirmations'],result['visual_evidence']['confirmations'])
            self.assertEqual(result['ground_contact_proxy_Base_mm'],
                result['visual_evidence']['ground_contact_proxy_Base_mm'])
            self.assertEqual(result['visual_evidence']['navigation_confirmed'],number==3)

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

    def test_shared_result_keeps_invalid_cues_for_same_frame_fine_competition(self):
        image=Image(Blob([300,400,40,40]),Blob([900,400,100,3]))
        result=track_coarse(image,self.frame,self.tracker.detect_frame)
        cues=result['visual_candidates']
        self.assertEqual(image.calls,1)
        self.assertEqual(len(cues),2)
        self.assertEqual(len(result['payload']['candidates']),1)
        self.assertIs(cues,result['payload']['visual_candidates'])
        self.assertTrue(all(cue['timestamp_ms']==self.frame['capture_timestamp_ms'] for cue in cues))
        self.assertEqual(cues[1]['measurement_reason'],'implausible_aspect_ratio')
        self.assertFalse(cues[1]['distance_valid'])

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
