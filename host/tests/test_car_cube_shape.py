"""Single-frame cube geometry with recorded native boxes and real calibration.

The thirty positives are the archived K230 native replay measurements from
far_review_20261024_141214/native_retry/result.json. Near positives come from
approach_replay_20261024/summary.json. The three negatives are native evidence
from overshoot_repair_repeat_01: blue tape print/specks, not a cube. Frozen
measurements exercise geometry and selection, not pixel segmentation parity.
"""
import copy
import unittest
from unittest.mock import patch

from device.k230.car.blue_tracker import BlueTracker
from device.k230.car.ground_projection import GroundProjection
from device.k230.car.target_memory import TargetMemory
from device.k230.car.tracking_drive import DriveController
from device.k230.vision.runtime import track_coarse
from host.tests.test_car_visual_evidence import Blob,Image
from tools.deploy.k230_robot import build_bundle


FAR_NATIVE=[
 ([662,232,94,108],9217,[708,285]),([453,164,75,81],5482,[489,203]),
 ([315,266,108,126],12164,[368,328]),([642,156,72,78],5055,[678,194]),
 ([24,238,87,111],8452,[67,292]),([554,201,65,93],5768,[586,247]),
 ([325,157,75,81],5347,[362,197]),([649,232,71,104],7140,[684,284]),
 ([619,149,69,77],4773,[653,187]),([375,146,65,75],4390,[407,183]),
 ([419,227,97,106],9216,[467,279]),([724,191,84,91],6902,[765,236]),
 ([420,151,66,76],4553,[452,188]),([223,238,106,114],10558,[276,293]),
 ([711,233,93,109],9115,[757,286]),([479,144,67,73],4455,[511,180]),
 ([243,188,86,91],6352,[282,231]),([701,201,86,96],7523,[743,248]),
 ([864,269,108,123],12053,[918,330]),([775,120,61,65],3550,[805,152]),
 ([448,119,57,66],3042,[473,150]),([373,123,54,70],3073,[398,158]),
 ([365,174,81,88],6201,[405,217]),([223,141,59,76],3889,[251,178]),
 ([813,161,71,81],5288,[847,201]),([493,279,112,131],13030,[549,342]),
 ([256,168,75,85],5060,[290,208]),([531,114,59,64],3407,[560,145]),
 ([551,176,69,85],5195,[584,218]),([551,176,69,85],5156,[584,218]),
]
FALSE_NATIVE=[([285,669,328,92],8593,[493,712]),
              ([1087,320,172,43],3438,[1185,341]),
              ([248,283,21,37],271,[259,302])]


class NativeBlob(Blob):
    def __init__(self,row):
        super().__init__(row[0],row[1]);self.center=row[2]
    def cx(self):return self.center[0]
    def cy(self):return self.center[1]


class CubeShapeTests(unittest.TestCase):
    def setUp(self):
        self.config=build_bundle(tracking=True)[0]['car']
        model=self.config['projection']
        self.projection=GroundProjection(model['K'],model['D'],model['T'],model['plane'],
            model['forward_axis_Base'],model['motion_origin_Base_mm'])
        self.tracker=BlueTracker(self.config['tracking'],self.projection)

    def coarse(self,rows,stamp=100):
        frame=dict(frame_id=stamp,capture_timestamp_ms=stamp,timestamp_ms=stamp,
                   clock_domain='k230_ticks_ms',image_size_px=[1280,960])
        return track_coarse(Image(*(NativeBlob(row) for row in rows)),frame,
                            self.tracker.detect_frame)

    def test_thirty_native_far_targets_remain_measurable_on_their_first_frame(self):
        self.assertEqual(len(FAR_NATIVE),30)
        self.assertEqual(self.config['tracking']['confirmation_frames'],1)
        for index,row in enumerate(FAR_NATIVE,1):
            with self.subTest(original=index):
                self.tracker.reset()
                observation=self.coarse([row])
                self.assertEqual(observation['status'],'tracking')
                evidence=observation['visual_evidence']
                self.assertTrue(evidence['cube_shape_compatible'])
                self.assertTrue(evidence['distance_valid'])
                self.assertTrue(evidence['navigation_confirmed'])
                self.assertEqual(evidence['association_confirmations'],1)

    def test_historical_near_left_center_right_boxes_keep_physical_compatibility(self):
        for box in ([525,562,167,262],[991,624,249,305],[74,613,239,303],
                    [541,527,169,273]):
            with self.subTest(box=box):
                self.tracker.reset()
                row=(box,box[2]*box[3],[box[0]+box[2]/2,box[1]+box[3]/2])
                evidence=self.coarse([row])['visual_evidence']
                self.assertTrue(evidence['cube_shape_compatible'])
                self.assertTrue(evidence['distance_valid'])

    def test_recorded_tape_and_speck_pass_old_gates_but_have_no_new_distance_authority(self):
        c=self.config['tracking']
        self.assertEqual(c['max_aspect_ratio'],4)
        self.assertEqual(c['height_ratio_range'],[.4,2.5])
        for row in FALSE_NATIVE:
            with self.subTest(box=row[0]):
                box,pixels,unused=row
                old=self.projection.blob_geometry(box)
                self.assertTrue(c['height_ratio_range'][0]<=
                    old['observed_to_expected_height_ratio']<=c['height_ratio_range'][1])
                self.assertLessEqual(max(box[2]/box[3],box[3]/box[2]),c['max_aspect_ratio'])
                self.assertGreaterEqual(pixels/(box[2]*box[3]),c['min_fill_ratio'])
                self.tracker.reset()
                observation=self.coarse([row])
                evidence=observation['visual_evidence']
                self.assertTrue(evidence['visible'])
                self.assertFalse(evidence['distance_valid'])
                self.assertFalse(evidence['navigation_confirmed'])
                self.assertEqual(evidence['reason'],'inconsistent_with_20mm_cube_shape')
                self.assertFalse(evidence['cube_shape_compatible'])
                self.assertNotIn('ground_contact_proxy_Base_mm',evidence)
                self.assertNotIn('ground_contact_proxy_Base_mm',observation)
                self.assertEqual(observation['visual_candidates'][0]['bbox_px'],box)
                self.assertEqual(observation['payload']['candidates'],[])

    def test_one_measurable_matched_cube_beats_nearby_weak_print_without_ambiguity(self):
        cube=([541,527,169,273],42501,[624,663])
        self.coarse([cube])
        for stamp in (200,300):
            observation=self.coarse([FALSE_NATIVE[0],cube],stamp)
            self.assertEqual(observation['bbox_px'],cube[0])
            self.assertFalse(observation['visual_evidence']['ambiguous'])
            self.assertEqual(observation['visual_evidence']['candidate_count'],2)
            self.assertTrue(observation['visual_evidence']['distance_valid'])
            self.assertEqual(len(observation['visual_candidates']),2)

    def test_multiple_physical_matched_cubes_do_not_choose_the_largest(self):
        cube=([541,527,169,273],42501,[624,663])
        other=([700,500,210,280],45000,[805,640])
        self.coarse([cube])
        observation=self.coarse([cube,other],200)
        self.assertEqual(len(observation['payload']['candidates']),2)
        self.assertTrue(observation['visual_evidence']['ambiguous'])
        self.assertFalse(observation['visual_evidence']['navigation_confirmed'])
        self.assertNotIn('ground_contact_proxy_Base_mm',observation)

    def test_rejected_old_print_cannot_hide_a_new_unique_physical_cube(self):
        print_patch=FALSE_NATIVE[1]
        cube=([541,527,169,273],42501,[624,663])
        for stamp in (100,200):
            rejected=self.coarse([print_patch],stamp)
            self.assertFalse(rejected['visual_evidence']['distance_valid'])
        observation=self.coarse([print_patch,cube],300)
        self.assertEqual(observation['bbox_px'],cube[0])
        self.assertEqual(observation['status'],'tracking')
        evidence=observation['visual_evidence']
        self.assertTrue(evidence['associated'])
        self.assertTrue(evidence['navigation_confirmed'])
        self.assertEqual(evidence['association_confirmations'],1)

    def test_rejected_old_print_does_not_pick_largest_of_two_new_physical_cubes(self):
        print_patch=FALSE_NATIVE[1]
        self.coarse([print_patch])
        cube=([541,527,169,273],42501,[624,663])
        other=([700,500,210,280],45000,[805,640])
        observation=self.coarse([print_patch,cube,other],200)
        self.assertTrue(observation['visual_evidence']['ambiguous'])
        self.assertFalse(observation['visual_evidence']['navigation_confirmed'])
        self.assertNotIn('ground_contact_proxy_Base_mm',observation)

    def test_clipped_original_cube_keeps_its_associated_direction_without_new_distance(self):
        original=self.coarse([FAR_NATIVE[4]])
        drive=DriveController(self.projection,self.config)
        drive._remember_search_region(original)
        region=copy.deepcopy(drive._search_region)
        with patch.object(self.projection,'cube_shape_geometry',
                          wraps=self.projection.cube_shape_geometry) as shape:
            clipped=self.coarse([([0,238,80,111],8000,[39,292])],200)
            self.assertEqual(shape.call_count,0)
        evidence=clipped['visual_evidence']
        self.assertTrue(evidence['visible'])
        self.assertTrue(evidence['associated'])
        self.assertEqual(evidence['reason'],'clipped_blob')
        self.assertFalse(evidence['distance_valid'])
        direction=drive._search_heading(clipped,{'visible':False,'associated':False},200,{})
        self.assertEqual(direction,'left')
        self.assertEqual(drive._search_region,region)

    def test_rejected_tape_cannot_retarget_or_replace_the_reliable_search_region(self):
        cube=([541,527,169,273],42501,[624,663])
        original=self.coarse([cube])
        memory=TargetMemory(self.projection,self.config)
        memory.update(original,motion_state={'run':1})
        identity=copy.deepcopy(memory._identity_visual)
        last_center=list(memory._last_center)
        motion_visual=copy.deepcopy(memory._motion_visual)
        drive=DriveController(self.projection,self.config)
        drive._remember_search_region(original)
        region=copy.deepcopy(drive._search_region)
        rejected=self.coarse([FALSE_NATIVE[0]],200)
        state=memory.update(rejected,motion_state={'run':1})
        self.assertFalse(state['associated'])
        self.assertFalse(state['table_distance_valid'])
        self.assertFalse(state['confirmed_moved'])
        self.assertTrue(state['precise_candidate'])
        self.assertIsNone(memory._query_visual)
        self.assertEqual(memory._identity_visual,identity)
        self.assertEqual(memory._last_center,last_center)
        self.assertEqual(memory._motion_visual,motion_visual)
        version=memory.version
        self.assertIsNone(memory.retarget_current(rejected))
        self.assertEqual(memory.version,version)
        drive._remember_search_region(rejected)
        self.assertEqual(drive._search_region,region)
        restored=self.coarse([cube],300)
        state=memory.update(restored,motion_state={'run':1})
        self.assertEqual(memory.version,version)
        self.assertFalse(state['confirmed_moved'])
        self.assertTrue(state['associated'])
        self.assertTrue(state['table_distance_valid'])
        self.assertEqual(memory._identity_visual,identity)

    def test_four_extra_projections_only_follow_original_candidate_checks(self):
        box=FAR_NATIVE[0][0]
        original=self.projection.blob_geometry(box)
        with patch.object(self.projection,'base_to_pixel',wraps=self.projection.base_to_pixel) as project:
            result=self.projection.cube_shape_geometry(box,original)
            self.assertEqual(project.call_count,4)
            self.assertTrue(result['cube_shape_compatible'])
        with patch.object(self.projection,'cube_shape_geometry',
                          wraps=self.projection.cube_shape_geometry) as shape:
            observation=self.coarse([([300,400,8,8],64,[304,404])])
            self.assertFalse(observation['visual_evidence']['distance_valid'])
            self.assertEqual(shape.call_count,0)


if __name__=='__main__':unittest.main()
