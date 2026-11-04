"""Completed static-image freshness is separate from movement classification.

Real Task/Memory/Drive/native-tracker logic uses synthetic visual poses and
UARTs. This checks capability ownership, not physical localization accuracy.
"""
import copy
import unittest

from host.tests import test_car_target_memory as memory_fixtures
from host.tests import test_fine_observation_pause as task_fixtures
from host.tests import test_robot_tracking as robot_fixtures


class StaticFineCurrentImageTests(unittest.TestCase):
    def seeded_task(self,move=True):
        helper=task_fixtures.FineObservationPauseTests()
        f,changes=helper.fixture('static',move_during_fine=move)
        image,frame=f.capture();coarse=f.task.track(image,frame)
        fine=f.precise(image,frame)
        # Match the existing fixture's explicit projected-pose protocol.
        # This is a synthetic past measurement, not a CV/image claim.
        u,v=f.projection.base_to_pixel(image)
        front=[[u-40,v-40],[u+40,v-40],[u+40,v+40],[u-40,v+40]]
        pose=dict(fine['payload']['poses'][0],
            cube_projected_corners_px=front+[[x+5,y+5]for x,y in front],
            cube_projection_source='pose_transform')
        fine['payload']['side_faces']=[dict(face_quad_px=front,
            selected_pose=pose,used_for_center=True)]
        fine['target_association']=f.task._match_fine(coarse,fine)
        self.assertEqual(fine['target_association']['status'],'matched')
        f.task.memory.update(coarse)
        self.assertTrue(f.task.memory.bind_precise(fine,coarse,0))
        f.task.drive.version=0
        f.task.drive._bind_anchor(fine,coarse)
        f.task.drive.phase='settling'
        f.clock.advance(5)
        return helper,f,changes,fine

    def test_static_requires_current_view_even_while_old_distance_anchor_is_retained(self):
        f=memory_fixtures.TargetMemoryTests();f.setUp();f.seed()
        source=f.coarse();f.memory.update(source)
        new_static=f.precise(source)
        previous=copy.deepcopy(f.memory.last_precise)
        f.memory.update(f.coarse((500,100,0)),motion_state={'run':0},
            controlled_correction=True)
        self.assertFalse(f.memory.snapshot()['confirmed_moved'])
        self.assertEqual(f.memory.version,0)
        self.assertEqual(f.memory.precise_consistency(new_static,source,()),'unknown')
        self.assertEqual(f.memory.precise_consistency(new_static,source,(),require_current=True),'changed')
        self.assertEqual(f.memory.last_precise,previous)

    def test_previous_good_anchor_cannot_authorize_a_different_completed_static_image(self):
        helper,f,changes,previous=self.seeded_task()
        done=helper.complete_paused_image(f,changes,'static')
        old=done['observation']['frame_id'];version=f.task.memory.version
        self.assertNotEqual(old,previous['frame_id'])
        self.assertEqual(f.task.memory.last_precise['frame_id'],previous['frame_id'])
        helper.consume_fresh_frame(f,done)
        self.assertNotIn(old,f.bind_attempts)
        self.assertFalse(f.handoffs);self.assertFalse(f.arms)
        self.assertEqual(f.task.memory.version,version)
        self.assertEqual(f.task.metrics['target_changes'],0)
        self.assertEqual(f.task.decision['status'],'fine_outdated')
        self.assertTrue(any(event=='tracking_fine_current_mismatch'for event,detail in f.events))
        f.finish()
        self.assertGreaterEqual(len(f.created_fines),2)
        self.assertTrue(all(observation['frame_id']!=old and
            observation['capture_timestamp_ms']>=changes[0]['at']
            for stamp,observation,version in f.handoffs))

    def test_unchanged_static_image_can_replace_previous_anchor_and_reach_arm(self):
        unused,f,changes,previous=self.seeded_task(move=False)
        for unused in range(2000):
            f.tick()
            if f.task.complete is not None:break
        else:self.fail('No completed static image')
        current=f.task.complete['observation']['frame_id']
        self.assertNotEqual(current,previous['frame_id'])
        f.tick()
        self.assertIn(current,f.bind_attempts)
        self.assertFalse(any(event=='tracking_fine_current_mismatch'for event,detail in f.events))
        result=f.finish()
        self.assertEqual(f.handoffs[0][1]['frame_id'],current)
        self.assertEqual(result['status'],'succeeded')

    def test_existing_events_retain_measured_center_and_correction_encoder_reference(self):
        f=robot_fixtures.RobotTrackingTests();f.setUp();f.y+=40
        f.finish()
        fine=next(detail for event,detail in f.events if event=='tracking_fine_completed')
        self.assertEqual(fine['cube_center_Base_mm'],f.created_fines[0].latest['cube_center_Base_mm'])
        self.assertEqual(fine['reprojection_rms_px'],.3)
        corrected=next(detail for event,detail in f.events if event=='tracking_fine_consumed'
            and (detail.get('command') or '').startswith('ARC,'))
        self.assertEqual(corrected['static_verification']['status'],'not_parked')
        self.assertIn('window',corrected['static_verification'])
        self.assertEqual(corrected['correction_plan']['command'],corrected['command'])
        self.assertIn('nominal_predicted_center_Base_mm',corrected['correction_plan'])
        self.assertEqual(corrected['correction_reference']['encoder_a'],0)
        self.assertEqual(corrected['correction_reference']['encoder_b'],0)
        self.assertLessEqual(len(f.task.timeline),64)


if __name__=='__main__':unittest.main()
