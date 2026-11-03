"""Bound original-image diagnostics without granting old images authority."""
import copy
import unittest

from host.tests import test_robot_tracking as fixtures
from host.tests import test_pending_fine_motion as pending


class DetachedEvidence:
    def __init__(self, item):
        self.item=item

    def take_evidence(self, label):
        item,self.item=self.item,None
        return item


class TrackingEvidenceAuditTests(unittest.TestCase):
    def test_first_and_last_moving_plus_last_static_are_bounded_and_keep_original_bytes(self):
        f=fixtures.RobotTrackingTests()
        f.setUp()
        task=f.task
        originals={}

        def save(number,purpose):
            frame=dict(frame_id=number,capture_timestamp_ms=number*100)
            coarse=dict(frame, bbox_px=[number,2,30,40],center_pixel=[number+15,22])
            jpeg=bytes([number])*10
            item=dict(frame=frame,jpeg=jpeg,observation=dict(
                status='localized',frame_id=number,capture_timestamp_ms=number*100,
                cube_center_Base_mm=[250.,0.,0.]))
            originals[number]=(item,jpeg)
            task.job=dict(session=DetachedEvidence(item),number=number,purpose=purpose,
                target_version=task.memory.version,paired_coarse=coarse,
                localization_bbox_px=list(coarse['bbox_px']))
            task._save_job()
            task.job=None
            self.assertLessEqual(len(task.evidence),3)

        save(1,'moving')
        original_frame=copy.deepcopy(task.evidence[0]['frame'])
        save(2,'static')
        task.memory.invalidate('new_target_epoch')
        save(3,'moving')
        save(4,'moving')
        save(5,'static')
        task.memory.invalidate('another_target_epoch')
        save(6,'moving')
        task._release_evidence(task.memory.version)
        self.assertEqual({row['frame']['frame_id'] for row in task.evidence},{1,5,6})
        first=next(row for row in task.evidence if row['frame']['frame_id']==1)
        self.assertIs(first,originals[1][0])
        self.assertIs(first['jpeg'],originals[1][1])
        self.assertEqual(first['frame'],original_frame)
        for row in task.evidence:
            self.assertIs(row['jpeg'],originals[row['frame']['frame_id']][1])
        self.assertNotEqual(first['frame']['target_version'],task.memory.version)
        self.assertFalse(task.memory.bind_precise(first['observation'],
            first['frame']['paired_coarse'],first['frame']['target_version']))

    def test_mismatch_records_original_and_current_geometry_without_a_new_camera_or_status(self):
        helper=pending.PendingFineMotionTests()
        scene=helper.scene(0,slices=20)
        f=scene.f
        helper.until(scene,lambda:f.task.complete is not None)
        original=copy.deepcopy(f.task.complete['paired_coarse'])
        f.motor.advance()
        f.y+=40
        helper.until(scene,lambda:any(row['event']=='tracking_fine_current_mismatch'
                                     for row in f.task.timeline))
        row=next(row for row in f.task.timeline
                 if row['event']=='tracking_fine_current_mismatch')
        audit=row['details']['audit']
        self.assertEqual(audit['source']['frame_id'],original['frame_id'])
        self.assertEqual(audit['source']['capture_timestamp_ms'],original['capture_timestamp_ms'])
        self.assertEqual(audit['source']['bbox_px'],original['visual_evidence']['bbox_px'])
        self.assertEqual(audit['source']['center_pixel'],original['visual_evidence']['center_pixel'])
        self.assertGreater(audit['current']['frame_id'],audit['source']['frame_id'])
        self.assertNotEqual(audit['current']['bbox_px'],audit['source']['bbox_px'])
        self.assertIsNotNone(audit['source']['coarse_proxy_X_mm'])
        self.assertIsNotNone(audit['current']['coarse_proxy_X_mm'])
        self.assertEqual(audit['fine_status'],'localized')
        self.assertEqual(audit['fine_association'],'matched')
        self.assertEqual(len(audit['fine_center_Base_mm']),3)
        self.assertEqual(len(audit['wheel_delta_mm']),2)
        self.assertTrue(all(value>=0 for value in audit['wheel_delta_mm']))
        self.assertIn('memory_residual_px',audit)
        self.assertIn('memory_residual_distance_fraction',audit)
        self.assertTrue(row['details']['continued_follow'])
        self.assertFalse(f.handoffs)
        self.assertFalse(scene.extra)


if __name__=='__main__':
    unittest.main()
