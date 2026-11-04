"""A commanded parking correction is not evidence of external cube motion.

The world-turn control is synthetic and independent of the encoder predictor.
Its differing true/nominal yaw reproduces an observability problem, not a
measurement of the field car's yaw error or correction accuracy.
"""
import copy
import math
import unittest

from device.k230.car.target_memory import TargetMemory
from host.tests import test_car_tracking_drive as drive_fixtures
from host.tests import test_static_anchor_authority as memory_fixtures


class CorrectionMotionMemoryTests(unittest.TestCase):
    def memory_fixture(self):
        fixture=memory_fixtures.StaticAnchorAuthorityTests()
        fixture.setUp()
        fixture.seed()
        return fixture

    def fixed_world_view(self, yaw, world=(500.,30.,0.)):
        return [math.cos(yaw)*world[0]+math.sin(yaw)*world[1],
                -math.sin(yaw)*world[0]+math.cos(yaw)*world[1],world[2]]

    def test_missing_correction_context_reproduces_the_two_frame_false_motion(self):
        fixture=self.memory_fixture()
        for index in (1,2):
            # The block stays at one world point. Actual body yaw advances
            # .09rad; real wheel feedback advances the nominal model .04rad.
            state=fixture.memory.update(fixture.observation(self.fixed_world_view(.09*index)),
                [(-2.,2.)],motion_state={'run':1})
            self.assertEqual(state['confirmed_moved'],index==2)
        self.assertEqual(state['version'],1)
        self.assertGreater(state['residual_px'],40.)

    def test_owned_correction_and_stop_boundary_rebase_without_rewriting_feedback(self):
        fixture=self.memory_fixture()
        for index,run in ((1,1),(2,1),(3,0)):
            observation=fixture.observation(self.fixed_world_view(.09*index))
            status={'run':run,'encoder_a':-20*index,'encoder_b':-20*index,
                    'reason':0 if run else 8,'ms':100000+index*100,'result':0}
            feedback_before=copy.deepcopy(status)
            observation_before=copy.deepcopy(observation)
            state=fixture.memory.update(observation,[(-2.,2.)],motion_state=status,
                controlled_correction=True)
            self.assertEqual(status,feedback_before)
            self.assertEqual(observation,observation_before)
            self.assertEqual(state['version'],0)
            self.assertFalse(state['confirmed_moved'])
            self.assertTrue(state['anchor_valid'])
            self.assertNotEqual(state['movement_state'],'suspected')
            self.assertTrue(state['motion_model_uncertain'])
        # The next genuinely stopped frame cannot borrow the preceding turn's
        # residual or its suspicion count. It becomes the next fine's pair.
        current=fixture.observation(self.fixed_world_view(.27))
        stopped=fixture.memory.update(current,motion_state={'run':0})
        self.assertEqual(stopped['version'],0)
        self.assertFalse(stopped['confirmed_moved'])
        self.assertFalse(stopped['motion_model_uncertain'])
        fine=copy.deepcopy(fixture.memory.last_precise)
        fine.update(frame_id=current['frame_id'],timestamp_ms=current['capture_timestamp_ms'],
            capture_timestamp_ms=current['capture_timestamp_ms'],
            cube_center_Base_mm=self.fixed_world_view(.27)[:2]+[10.])
        self.assertTrue(fixture.memory.bind_precise(fine,current,0))

    def test_ordinary_forward_true_target_translation_still_uses_two_frames(self):
        fixture=self.memory_fixture()
        for index in (1,2):
            # Independent7mm forward travel plus a human70mm lateral move.
            status={'run':1,'encoder_a':7*index,'encoder_b':-7*index}
            before=copy.deepcopy(status)
            state=fixture.memory.update(fixture.observation((500-7*index,100,0)),
                [(7.,7.)],motion_state=status)
            self.assertEqual(status,before)
            self.assertEqual(state['confirmed_moved'],index==2)
        self.assertEqual(state['version'],1)

    def test_return_to_coarse_releases_fine_authority_without_erasing_its_evidence(self):
        fixture=self.memory_fixture()
        previous=copy.deepcopy(fixture.memory.last_precise)
        fixture.memory.release_precise()
        state=fixture.memory.snapshot()
        self.assertEqual(state['version'],0)
        self.assertFalse(state['anchor_valid'])
        self.assertTrue(state['needs_precise'])
        self.assertTrue(state['target_valid'])
        self.assertFalse(state['confirmed_moved'])
        self.assertEqual(state['last_precise'],previous)

    def test_correction_context_does_not_hide_a_real_edge_exit_into_an_empty_frame(self):
        fixture=memory_fixtures.StaticAnchorAuthorityTests()
        fixture.setUp()
        first=fixture.projection.pixel_to_plane([20.,520.])
        fixture.memory.update(fixture.observation(first),motion_state={'run':0})
        at_edge=fixture.projection.pixel_to_plane([2.,520.])
        edge=fixture.memory.update(fixture.observation(at_edge,edge_directions=['left']),
            motion_state={'run':1},controlled_correction=True)
        self.assertEqual(edge['last_disappearance_direction'],'left')
        gone=fixture.memory.update(fixture.absent(),motion_state={'run':1},
            controlled_correction=True)
        self.assertFalse(gone['visible'])
        self.assertEqual(gone['event'],'exited')
        self.assertEqual(gone['last_disappearance_direction'],'left')
        self.assertFalse(gone['target_valid'])


class CorrectionDriveLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.fixture=drive_fixtures.TrackingDriveTests()
        self.fixture.setUp()
        self.memory=TargetMemory(self.fixture.projection,self.fixture.config)
        self.world=self.fixture.point(self.fixture.target,40.)

    def observation(self, now, point):
        fixture=self.fixture
        coarse=fixture.coarse(now,point[0])
        pixel=fixture.projection.base_to_pixel(point)
        box=[pixel[0]-20,pixel[1]-20,40.,40.]
        floor=[point[k]-10*fixture.projection.normal[k] for k in range(3)]
        values=dict(center_pixel=pixel,bbox_px=box,ground_contact_proxy_Base_mm=floor)
        coarse.update(values)
        coarse['visual_evidence'].update(values)
        return coarse

    def view_after_turn(self, yaw):
        projection=self.fixture.projection
        normal=projection.normal
        axis=projection.forward_axis_Base
        dot=sum(a*b for a,b in zip(normal,axis))
        forward=[axis[k]-dot*normal[k] for k in range(3)]
        length=math.sqrt(sum(v*v for v in forward))
        forward=[v/length for v in forward]
        left=[normal[1]*forward[2]-normal[2]*forward[1],
              normal[2]*forward[0]-normal[0]*forward[2],
              normal[0]*forward[1]-normal[1]*forward[0]]
        origin=projection.motion_origin_Base_mm
        relative=[self.world[k]-origin[k] for k in range(3)]
        x,y,z=[sum(a*b for a,b in zip(relative,axis))for axis in(forward,left,normal)]
        a,b=math.cos(yaw)*x+math.sin(yaw)*y,-math.sin(yaw)*x+math.cos(yaw)*y
        return [origin[k]+a*forward[k]+b*left[k]+z*normal[k] for k in range(3)]

    def start_real_correction(self):
        fixture=self.fixture
        for now in(1000,1050):
            coarse=self.observation(now,self.world)
            target=self.memory.update(coarse,motion_state=fixture.state(now))
            decision=fixture.update(now,coarse=coarse,target_version=0,target_event=target)
        self.assertEqual(decision['request_fine'],'static')
        complete=fixture.precise(1050,fixture.target,version=0)
        complete['observation']['cube_center_Base_mm']=list(self.world)
        complete['paired_coarse']=coarse
        self.assertTrue(self.memory.bind_precise(complete['observation'],coarse,0))
        decision=fixture.update(1150,coarse=self.observation(1150,self.world),
            target_version=0,target_event=self.memory.snapshot(),precise=complete)
        self.assertTrue(decision['command'].startswith('ARC,'),decision)
        self.assertEqual(decision['corrections'],1)
        self.assertEqual(fixture.drive.phase,'correcting')
        return complete

    def test_correction_finishes_then_requires_a_new_static_before_any_second_correction(self):
        old=self.start_real_correction()
        fixture=self.fixture
        for index,now in enumerate((1250,1350),1):
            coarse=self.observation(now,self.view_after_turn(.09*index))
            feedback=fixture.state(now,run=True,travel=-2*index,right=2*index)
            before=copy.deepcopy(feedback)
            target=self.memory.update(coarse,[(-2.,2.)],motion_state=feedback,
                controlled_correction=True)
            decision=fixture.update(now,run=True,travel=-2*index,right=2*index,
                coarse=coarse,target_version=self.memory.version,target_event=target)
            self.assertEqual(feedback,before)
            self.assertEqual(self.memory.version,0)
            self.assertEqual(decision['command'],'HB')
            self.assertEqual(decision['corrections'],1)
        coarse=self.observation(1450,self.view_after_turn(.18))
        target=self.memory.update(coarse,motion_state=fixture.state(1450,travel=-4,right=4,reason=8),
            controlled_correction=True)
        ended=fixture.update(1450,travel=-4,right=4,reason=8,coarse=coarse,
            target_version=0,target_event=target,precise=old)
        self.assertIsNone(ended['command'])
        self.assertFalse(ended['precise_consumed'])
        self.assertFalse(ended['allow_arm'])
        self.assertEqual(ended['corrections'],1)
        coarse=self.observation(1500,self.view_after_turn(.18))
        target=self.memory.update(coarse,motion_state=fixture.state(1500,travel=-4,right=4,reason=8))
        ready=fixture.update(1500,travel=-4,right=4,reason=8,coarse=coarse,
            target_version=0,target_event=target,precise=old)
        self.assertEqual(ready['request_fine'],'static')
        self.assertFalse(ready['precise_consumed'])
        self.assertNotEqual(ready['fine_request_id'],old['request_id'])
        repeated=fixture.update(1550,travel=-4,right=4,reason=8,coarse=coarse,
            target_version=0,target_event=target,precise=old,fine_pending=True)
        self.assertIsNone(repeated['command'])
        self.assertFalse(repeated['allow_arm'])
        self.assertFalse(repeated['precise_consumed'])
        # Only a newly measured stopped pose may authorize another correction.
        fresh=fixture.precise(1500,fixture.target-10,version=0)
        fresh['paired_coarse']=coarse
        current=self.observation(1600,self.view_after_turn(.18))
        corrected=fixture.update(1600,travel=-4,right=4,reason=8,coarse=current,
            target_version=0,target_event=target,precise=fresh)
        self.assertTrue(corrected['precise_consumed'])
        self.assertTrue(corrected['command'].startswith('ARC,'),corrected)
        self.assertEqual(corrected['corrections'],2)
        self.assertFalse(corrected['allow_arm'])

    def test_new_static_far_result_returns_to_coarse_then_rearms_moving_fine_nearby(self):
        self.start_real_correction()
        fixture=self.fixture
        ended=fixture.update(1250,reason=8,coarse=self.observation(1250,self.world),
            target_version=0,target_event=self.memory.snapshot())
        self.assertIsNone(ended['command'])
        far=fixture.point(650.)
        pair=self.observation(1300,far)
        pair['visual_evidence'].update(distance_valid=False,navigation_confirmed=False)
        ready=fixture.update(1300,reason=8,coarse=pair,target_version=0,
            target_event=self.memory.snapshot())
        self.assertEqual(ready['request_fine'],'static')
        self.assertIsNone(ready['command'])
        fresh=fixture.precise(1300,650.,version=0)
        fresh['paired_coarse']=pair
        fresh['observation']['cube_center_Base_mm']=far
        current=self.observation(1450,far)
        consumed=fixture.update(1450,reason=8,coarse=current,
            target_version=0,target_event=self.memory.snapshot(),precise=fresh)
        self.assertTrue(consumed['precise_consumed'])
        self.assertTrue(consumed['return_to_coarse'])
        self.assertIsNone(fixture.drive.anchor)
        self.assertIsNone(fixture.drive._last_precise)
        self.assertFalse(consumed['allow_arm'])
        fixture.assert_bounded_follow(consumed,'FOLLOW')
        self.assertEqual(consumed['plan']['mode'],'coarse')
        self.assertIsNone(consumed['request_fine'])
        near=fixture.point(450.)
        #200mm at about67mm/s, not an impossible one-frame encoder jump.
        approached=fixture.update(4450,run=True,travel=200,coarse=self.observation(4450,near),
            target_version=0,target_event=self.memory.snapshot())
        self.assertEqual(approached['request_fine'],'moving')
        self.assertFalse(approached['allow_arm'])

    def test_far_coarse_noise_cannot_replace_the_owned_near_correction(self):
        self.start_real_correction()
        fixture=self.fixture
        anchor=copy.deepcopy(fixture.drive.anchor)
        decision=fixture.update(1250,run=True,travel=-2,right=2,
            coarse=self.observation(1250,fixture.point(650.)),target_version=0,
            target_event=self.memory.snapshot())
        self.assertEqual(decision['command'],'HB')
        self.assertEqual(fixture.drive.anchor,anchor)
        self.assertIsNotNone(fixture.drive._last_precise)
        self.assertIsNone(decision['request_fine'])
        self.assertFalse(decision['allow_arm'])

    def test_unanchored_static_failure_can_leave_hold_for_trustworthy_far_coarse_only(self):
        for invalid in (None, 'distance_valid', 'navigation_confirmed', 'associated', 'stale'):
            with self.subTest(invalid=invalid):
                fixture=drive_fixtures.TrackingDriveTests()
                fixture.setUp()
                fixture.request_static()
                failed=fixture.precise(1050,fixture.target,status='no_complete_face')
                stopped=fixture.update(1150,fixture.target,precise=failed)
                self.assertIsNone(stopped['command'])
                self.assertEqual(stopped['request_fine'],'static')
                self.assertIsNone(fixture.drive.anchor)
                pending=fixture.update(1200,coarse=fixture.coarse(1200,650.),fine_pending=True)
                self.assertIsNone(pending['command'])
                # Finish the already-owned exposure before reconsidering its
                # static hold. Coarse range alone cannot interrupt that job.
                completed=fixture.precise(1150,fixture.target,status='no_complete_face')
                coarse=fixture.coarse(1200 if invalid=='stale' else 1250,650.)
                if invalid not in(None,'stale'):
                    coarse['visual_evidence'][invalid]=False
                result=fixture.update(2000 if invalid=='stale' else 1250,coarse=coarse,precise=completed)
                if invalid is None:
                    self.assertIsNotNone(result['command'],result)
                    fixture.assert_bounded_follow(result,'FOLLOW')
                    self.assertTrue(result['precise_consumed'])
                    self.assertTrue(result['return_to_coarse'])
                    self.assertIsNone(fixture.drive._last_precise)
                    self.assertIsNone(fixture.drive._fine_request)
                else:
                    self.assertIsNone(result['command'])
                self.assertIsNone(fixture.drive.anchor)
                self.assertFalse(result['allow_arm'])


if __name__=='__main__':
    unittest.main()
