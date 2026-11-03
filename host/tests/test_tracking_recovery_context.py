"""Existing Task/arm recovery event retains the actual revocation inputs.

Images, encoder plant and arm IO are explicit protocol doubles. These tests
check diagnostic provenance and unchanged recovery dispatch, not this incident's
missing pixels or physical cause.
"""
import copy
import json
import unittest
from unittest.mock import patch

from host.tests import test_robot_tracking as fixtures


class TrackingRecoveryContextTests(unittest.TestCase):
    def scene(self, lift=False):
        f=fixtures.RobotTrackingTests()
        f.setUp()
        f.persisted=[]
        factory=f.begin_arm
        def begin(observation, version):
            arm=factory(observation,version)
            def emit(event,details):
                # Same JSON-like envelope as the existing persistent arm event.
                f.persisted.append(json.loads(json.dumps(dict(event=event,
                    ticks_ms=f.clock.now(),details=details))))
            arm.emit=emit
            return arm
        f.task.begin_arm=begin
        if lift:
            for unused in range(1200):
                f.tick()
                arm=f.task.arm
                sample=(arm.snapshot().get('last_sample') or {}) if arm else {}
                if arm is not None and arm.phase=='lift' and sample.get('axes')==[1,2,3,4,5,6]:
                    break
            else:self.fail('No valid synthetic lift sample')
            self.assertTrue(arm.grasp_confirmed)
        return f

    def recovery(self, f):
        rows=[row for row in f.persisted if row['event']=='recovery_goal_sent']
        self.assertEqual(len(rows),1)
        self.assertEqual(sum(event[0]=='ready2' for event in f.task.arm.io.events),1)
        self.assertTrue(rows[0]['details']['gripper_opens'])
        self.assertEqual(rows[0]['details']['reason'],'target_version_changed')
        return rows[0]

    def test_explicit_epoch_context_keeps_the_original_visual_frame_and_arm_sample(self):
        f=self.scene(lift=True)
        before=copy.deepcopy(f.task.arm.snapshot())
        coarse=copy.deepcopy(f.task.coarse)
        original_version=f.task.arm.target_version
        f.task.memory.invalidate('explicit_epoch_marker')
        f.task.target=f.task.memory.snapshot()
        f.clock.advance(17)
        old_frame,old_packets=f.frame,len(f.motor.commands)
        f.task._step_arm()
        self.assertEqual(f.frame,old_frame)
        self.assertEqual(len(f.motor.commands),old_packets)
        row=self.recovery(f)
        context=row['details']['target_context']
        self.assertEqual(context['reason'],'explicit_epoch_marker')
        self.assertEqual(context['old_target_version'],original_version)
        self.assertGreater(context['new_target_version'],original_version)
        self.assertEqual(context['phase'],'lift')
        self.assertTrue(context['vision_paused_for_grip'])
        self.assertEqual(context['last_sample'],before['last_sample'])
        self.assertEqual(context['grip_confirmation']['delta_counts'],40)
        self.assertEqual(context['grip_confirmation']['sample_time_ms'],
                         before['grip_confirmation']['sample_time_ms'])
        self.assertEqual(context['coarse']['frame_id'],coarse['frame_id'])
        self.assertEqual(context['coarse']['capture_timestamp_ms'],coarse['capture_timestamp_ms'])
        self.assertLess(context['coarse']['capture_timestamp_ms'],row['ticks_ms'])
        for key in ('bbox_px','center_pixel','associated','ambiguous','distance_valid','reason'):
            self.assertEqual(context['coarse']['visual_evidence'][key],
                             coarse['visual_evidence'].get(key))
        self.assertIsNone(context['chassis']['last_invalidation'])
        f.task._step_arm()
        self.assertEqual(len([r for r in f.persisted if r['event']=='recovery_goal_sent']),1)

    def test_chassis_invalidation_context_retains_counts_and_real_reply_times(self):
        f=self.scene(lift=True)
        reference=copy.deepcopy(f.task._arm_motor_reference)
        old_version=f.task.arm.target_version
        before=copy.deepcopy(f.task.arm.snapshot())
        frame=f.frame
        f.motor.a+=round(2/f.config['mm_per_count'])
        f.clock.advance(501)
        f.task.step()
        self.assertEqual(f.frame,frame)
        invalidation=copy.deepcopy(f.task.metrics['last_chassis_invalidation'])
        current=copy.deepcopy(f.task.state)
        context=self.recovery(f)['details']['target_context']
        self.assertEqual(context['reason'],'chassis_moved_during_grasp')
        self.assertEqual(context['old_target_version'],old_version)
        self.assertGreater(context['new_target_version'],old_version)
        self.assertTrue(context['vision_paused_for_grip'])
        self.assertEqual(context['last_sample'],before['last_sample'])
        for key in ('encoder_a','encoder_b','run','ms','_k230_sent_ms','_k230_received_ms'):
            self.assertEqual(context['chassis']['reference'][key],reference.get(key))
            self.assertEqual(context['chassis']['current'][key],current.get(key))
        self.assertEqual(context['chassis']['last_invalidation'],invalidation)
        self.assertEqual(context['chassis']['current']['_k230_received_ms'],f.clock.now())

    def test_absent_context_fields_stay_unknown_and_do_not_fabricate_new_samples(self):
        f=self.scene(lift=True)
        f.task.memory.invalidate('explicit_test_epoch')
        f.task.target=f.task.memory.snapshot()
        f.task.coarse=None
        f.task.arm.last_sample=None
        f.task._arm_motor_reference=None
        packets,frame=f.motor.commands[:],f.frame
        f.task._step_arm()
        context=self.recovery(f)['details']['target_context']
        self.assertEqual(context['reason'],'explicit_test_epoch')
        self.assertIsNone(context['last_sample'])
        self.assertTrue(context['vision_paused_for_grip'])
        self.assertNotIn('held_reference',context)
        self.assertIsNone(context['coarse']['frame_id'])
        self.assertIsNone(context['coarse']['capture_timestamp_ms'])
        self.assertTrue(all(value is None for value in context['coarse']['visual_evidence'].values()))
        self.assertTrue(all(value is None for value in context['chassis']['reference'].values()))
        self.assertEqual(f.motor.commands,packets)
        self.assertEqual(f.frame,frame)

    def test_same_version_arm_invalidation_is_still_given_diagnostic_context(self):
        f=self.scene(lift=True)
        original=f.task.memory.version
        f.task.arm.invalidate_target('explicit_arm_invalidation')
        f.task._step_arm()
        row=next(r for r in f.persisted if r['event']=='recovery_goal_sent')
        self.assertEqual(row['details']['reason'],'explicit_arm_invalidation')
        context=row['details']['target_context']
        self.assertEqual(context['old_target_version'],original)
        self.assertEqual(context['new_target_version'],original)
        self.assertEqual(context['reason'],None)

    def test_healthy_flow_never_constructs_revocation_context_or_adds_arm_events(self):
        f=self.scene()
        with patch.object(f.task,'_arm_target_context',side_effect=AssertionError('unexpected diagnostic')):
            result=f.finish()
        self.assertEqual(result['status'],'succeeded')
        self.assertEqual(result['metrics']['recoveries'],0)
        self.assertEqual(result['grip_confirmation']['delta_counts'],40)
        self.assertFalse(any(r['event']=='recovery_goal_sent' for r in f.persisted))
        self.assertEqual([r['details']['name'] for r in f.persisted if r['event']=='stage_goal_sent'],
                         ['hover','descend','descend','close','lift'])


if __name__=='__main__':
    unittest.main()
