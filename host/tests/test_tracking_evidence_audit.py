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

    def test_moving_handoff_records_original_t1_t2_and_dispatch_without_extra_observation(self):
        helper=pending.PendingFineMotionTests()
        scene=helper.scene(0,slices=20)
        f=scene.f
        helper.until(scene,lambda:f.task.metrics.get('first_moving_handoff') is not None)
        fine=f.created_fines[0]
        saved=next(item for item in f.task.evidence
                   if item['frame']['frame_id']==fine.frame['frame_id'])
        original=saved['frame']['paired_coarse']
        row=next(row for row in f.task.timeline
                 if row['event']=='tracking_moving_handoff')
        audit=row['details']
        self.assertEqual(audit,f.task.metrics['first_moving_handoff'])
        self.assertEqual(audit['frame_id'],original['frame_id'])
        self.assertEqual(audit['capture_timestamp_ms'],original['capture_timestamp_ms'])
        self.assertEqual(saved['frame']['localization_bbox_px'],original['visual_evidence']['bbox_px'])
        self.assertEqual(f.frame,original['frame_id'])
        self.assertEqual(saved['jpeg'],b'original')
        self.assertEqual(saved['observation']['status'],'localized')
        self.assertEqual(saved['observation']['target_association']['status'],'matched')
        self.assertEqual(audit['result_ready_ms'],fine.latest_ready_ms)
        self.assertLess(audit['capture_timestamp_ms'],audit['result_ready_ms'])
        self.assertGreaterEqual(audit['status_sent_ms'],audit['result_ready_ms'])
        t1,t2=audit['capture_motor_state'],audit['handoff_motor_state']
        # The capture-side STATUS follows the plant's10ms snapshot call;
        # retain that real latency rather than relabeling it as exposure time.
        self.assertEqual(t1['_k230_sent_ms'],
                         audit['capture_timestamp_ms']+scene.costs['capture'])
        for sample in (t1,t2):
            raw=next(state for command,at,state in f.motor.replies
                if command=='STATUS' and at==sample['_k230_received_ms'])
            for key in ('ms','encoder_a','encoder_b','run'):
                self.assertEqual(sample[key],raw[key])
        self.assertGreaterEqual(t2['_k230_received_ms'],audit['status_sent_ms'])
        self.assertGreater(t2['encoder_a'],t1['encoder_a'])
        self.assertLess(t2['encoder_b'],t1['encoder_b'])
        self.assertEqual(t2['_k230_sent_ms'],audit['status_sent_ms'])
        self.assertEqual(t2['_k230_received_ms'],audit['status_received_ms'])
        self.assertGreaterEqual(audit['command_sent_ms'],audit['status_received_ms'])
        self.assertGreaterEqual(audit['command_reply_ms'],audit['command_sent_ms'])
        self.assertEqual(audit['plan']['frame_id'],original['frame_id'])
        self.assertTrue(audit['plan']['odometry_predicted'])
        self.assertTrue(audit['memory_bound'])
        self.assertEqual(audit['command'],next(command for at,command in f.motor.commands
            if at==audit['command_sent_ms'] and command.startswith('SPEED,')))
        after_ready=[command for at,command in f.motor.commands
                     if audit['result_ready_ms']<=at<=audit['command_reply_ms']]
        self.assertEqual(after_ready,['STATUS',audit['command']])
        # No coarse camera/track/classification is acquired to build this
        # diagnostic record. The original JPEG and real UART samples suffice.
        self.assertFalse(any(audit['result_ready_ms']<=at<=audit['command_reply_ms']
            for name,at,job in scene.calls if name in ('capture','track','classify')))
        self.assertFalse(f.handoffs)
        self.assertFalse(scene.extra)


if __name__=='__main__':
    unittest.main()
