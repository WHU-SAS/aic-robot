"""Single native frame acquisition; movement and loss retain separate gates."""
import copy
import unittest

from device.k230.car.blue_tracker import BlueTracker
from device.k230.car.target_memory import TargetMemory
from device.k230.vision.runtime import track_coarse
from tools.deploy.k230_robot import build_bundle
from host.tests import test_car_target_memory as memory_fixtures
from host.tests import test_car_tracking_drive as drive_fixtures
from host.tests import test_robot_tracking as robot_fixtures
from host.tests.test_car_visual_evidence import Blob, Image, Projection


class SingleFrameCandidateTests(unittest.TestCase):
    def setUp(self):
        self.config=build_bundle(tracking=True)[0]['car']

    def test_first_native_frame_can_navigate_and_unmeasurable_one_can_query(self):
        frame=dict(frame_id=1,capture_timestamp_ms=100,timestamp_ms=100,
            clock_domain='k230_ticks_ms',image_size_px=[1280,960])
        for ratio in (1,4):
            with self.subTest(height_ratio=ratio):
                projection=Projection(ratio)
                tracker=BlueTracker(self.config['tracking'],projection)
                coarse=track_coarse(Image(Blob([300,400,40,40])),frame,tracker.detect_frame)
                state=TargetMemory(projection,self.config).update(coarse)
                self.assertEqual(coarse['visual_evidence']['association_confirmations'],1)
                self.assertTrue(state['precise_candidate'])
                if ratio==1:
                    self.assertEqual(coarse['status'],'tracking')
                    self.assertTrue(coarse['visual_evidence']['navigation_confirmed'])
                    self.assertTrue(state['table_distance_valid'])
                else:
                    self.assertFalse(coarse['visual_evidence']['distance_valid'])
                    self.assertFalse(state['table_distance_valid'])

    def test_new_raw_search_direction_is_available_on_its_first_frame(self):
        fixture=drive_fixtures.TrackingDriveTests()
        fixture.setUp()
        fixture.config['tracking']['confirmation_frames']=self.config['tracking']['confirmation_frames']
        # Archived native20mm target; the old arbitrary69x35 patch is not a
        # whole-cube silhouette under the calibrated projection.
        coarse=fixture.native_cue(100,Blob([493,279,112,131],13030))
        self.assertEqual(coarse['visual_evidence']['association_confirmations'],1)
        self.assertTrue(coarse['visual_candidates'][0]['distance_valid'])
        # The native selected target is unavailable; a separate unique raw cue
        # must not wait for the previous three-frame search-hint cache.
        coarse['visual_evidence']=dict(visible=False,associated=False,ambiguous=False)
        direction=fixture.drive._search_heading(coarse,
            {'visible':False,'associated':False},100,{})
        self.assertEqual(direction,'left')

    def test_one_frame_keeps_full_and_range_until_same_target_motion_is_confirmed(self):
        for capability in ('full','range'):
            with self.subTest(capability=capability):
                fixture=memory_fixtures.TargetMemoryTests()
                fixture.setUp()
                fixture.config['tracking']=copy.deepcopy(self.config['tracking'])
                fixture.config['tracking_task']=copy.deepcopy(self.config['tracking_task'])
                memory=TargetMemory(fixture.projection,fixture.config)
                old=fixture.visual(association_confirmations=1,navigation_confirmed=True)
                memory.update(old)
                fine=fixture.precise(old)
                if capability=='range':
                    point=fine.pop('cube_center_Base_mm')
                    fine.update(status='no_complete_face',range_candidate={
                        'schema':'aic.car.side_range.v1','status':'range_candidate',
                        'coordinate_frame':'Base','units':'mm','Base_X_mm':point[0],
                        'raw_center_Base_mm':point,'not_arm_or_parking_authority':True})
                self.assertTrue(memory.bind_precise(fine,old,0))
                memory.update(fixture.absent())
                fixture.stamp+=600
                current=fixture.visual((500,230,0),association_confirmations=1,
                    navigation_confirmed=True)
                state=memory.update(current)
                self.assertEqual(state['version'],0)
                self.assertTrue(state['anchor_valid'])
                self.assertFalse(state['confirmed_moved'])
                self.assertEqual(state['last_precise'],fine)
                self.assertEqual(state['paired_coarse'],old)
                # Native acquisition stays one-frame, but a measured travel
                # anchor is revoked only after the original movement gate.
                current=fixture.visual((500,230,0),association_confirmations=2,
                    navigation_confirmed=True)
                state=memory.update(current)
                self.assertEqual(state['version'],1)
                self.assertTrue(state['associated'])
                self.assertFalse(state['target_valid'])
                self.assertTrue(state['confirmed_moved'])
                self.assertTrue(state['table_distance_valid'])
                self.assertEqual(state['last_seen'],current)
                self.assertFalse(state['anchor_valid'])
                self.assertFalse(memory.bind_precise(fine,old,0))
                self.assertFalse(memory.bind_precise(fine,old,1))
                # The existing epoch timestamp rule still rejects a fine pose
                # from the frame which caused the target version transition.
                same=fixture.precise(current)
                same['target_association']={'status':'matched'}
                self.assertFalse(memory.bind_precise(same,current,1))

    def test_one_missing_frame_does_not_start_search_but_three_do(self):
        fixture=robot_fixtures.RobotTrackingTests()
        fixture.setUp()
        fixture.config['tracking']['confirmation_frames']=1
        fixture.hide_frames=3
        self.addCleanup(fixture.task.close)
        fixture.tick()
        self.assertEqual(fixture.task._missing,1)
        self.assertFalse(fixture.task.drive._searching)
        for unused in range(100):
            fixture.tick()
            if fixture.task._missing>=2:break
        self.assertEqual(fixture.task._missing,2)
        self.assertFalse(fixture.task.drive._searching)
        for unused in range(100):
            fixture.tick()
            if fixture.task._missing>=3:break
        self.assertEqual(fixture.task._missing,3)
        self.assertTrue(fixture.task.drive._searching)

    def test_epoch_transition_takes_one_fresh_frame_before_computing_fine(self):
        fixture=robot_fixtures.RobotTrackingTests()
        fixture.setUp()
        fixture.config['tracking']['confirmation_frames']=1
        self.addCleanup(fixture.task.close)
        ordinary=fixture.task.track
        def single(point,frame):
            result=ordinary(point,frame)
            result['confirmations']=1
            result['visual_evidence'].update(confirmations=1,association_confirmations=1)
            return result
        fixture.task.track=single
        fixture.task.drive._observe_motor(fixture.motor.sync(),fixture.clock.now())
        fixture.clock.advance(100)
        fixture.task.drive._observe_motor(fixture.motor.sync(),fixture.clock.now())
        fixture.y-=80
        image,frame=fixture.capture()
        old=single(image,frame)
        fixture.task.memory.update(old)
        old_fine=fixture.precise(image,frame)
        self.assertTrue(fixture.task.memory.bind_precise(old_fine,old,0))
        fixture.y+=80
        fixture.clock.advance(100)
        image,frame=fixture.capture()
        first=fixture.task.memory.update(single(image,frame))
        self.assertEqual(first['version'],0)
        self.assertTrue(first['anchor_valid'])
        fixture.clock.advance(100)
        fixture.task.step()
        transition=fixture.task.coarse['capture_timestamp_ms']
        self.assertEqual(fixture.task.memory.version,1)
        self.assertEqual(fixture.task.memory._epoch_start_stamp,transition)
        self.assertEqual(fixture.task.metrics['fine_started'],0)
        self.assertFalse(fixture.created_fines)
        self.assertFalse(fixture.task.memory.bind_precise(old_fine,old,0))
        fixture.clock.advance(fixture.task.period)
        fixture.task.step()
        self.assertEqual(fixture.task.metrics['fine_started'],1)
        self.assertGreater(fixture.created_fines[0].frame['capture_timestamp_ms'],transition)
        fixture.finish()
        self.assertEqual(fixture.task.metrics['fine_started'],1)
        self.assertEqual(len(fixture.handoffs),1)
        self.assertGreater(fixture.handoffs[0][1]['capture_timestamp_ms'],transition)

    def test_confirmed_target_motion_still_needs_two_observations(self):
        fixture=memory_fixtures.TargetMemoryTests()
        fixture.setUp()
        fixture.config['tracking']=copy.deepcopy(self.config['tracking'])
        fixture.config['tracking_task']=copy.deepcopy(self.config['tracking_task'])
        memory=TargetMemory(fixture.projection,fixture.config)
        memory.update(fixture.visual(association_confirmations=1,navigation_confirmed=True))
        first=memory.update(fixture.visual((500,100,0),association_confirmations=1,
            navigation_confirmed=True))
        self.assertEqual(first['movement_state'],'suspected')
        self.assertEqual(first['version'],0)
        second=memory.update(fixture.visual((500,100,0),association_confirmations=2,
            navigation_confirmed=True))
        self.assertEqual(second['movement_state'],'confirmed')
        self.assertEqual(second['version'],1)


if __name__=='__main__':
    unittest.main()
