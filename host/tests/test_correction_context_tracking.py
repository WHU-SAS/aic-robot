"""Issued parking-correction context, using real Task/Drive/Memory together.

The UART/servo world is synthetic; this verifies ownership and fresh-frame
ordering, not physical turning accuracy or surveyed object motion.
"""
import copy
import unittest
from unittest.mock import Mock

from host.tests import test_robot_tracking as fixtures


class CorrectionContextTrackingTests(unittest.TestCase):
    def test_real_correction_feedback_stays_real_and_requires_new_static_before_arm(self):
        f=fixtures.RobotTrackingTests();f.setUp();f.y+=40
        # Distinct real UART snapshots cannot contain an entire physical ARC
        # at one timestamp. Preserve the stopped/run0 completion semantics,
        # with1ms request time so encoder interpolation has a real interval.
        f.motor.request_cost_ms=1
        original=f.task.memory.update;observations=[]
        def observed(observation,*args,**kwargs):
            before=copy.deepcopy(kwargs['motion_state'])
            increments=copy.deepcopy(args[0])
            result=original(observation,*args,**kwargs)
            self.assertEqual(kwargs['motion_state'],before)
            self.assertEqual(args[0],increments)
            observations.append(dict(frame=observation['frame_id'],
                stamp=observation['capture_timestamp_ms'],state=before,
                controlled=kwargs['controlled_correction']))
            return result
        f.task.memory.update=observed
        result=f.finish()
        arcs=[(stamp,command)for stamp,command in f.motor.commands if command.startswith('ARC,')]
        self.assertTrue(arcs)
        controlled=[row for row in observations if row['controlled']]
        self.assertTrue(controlled)
        # This plant completes ARC immediately. Context must still cover the
        # real stopped reply, rather than changing its run bit to manufacture
        # an in-motion sample.
        self.assertTrue(all(row['state']['run']==0 for row in controlled))
        self.assertEqual(result['metrics']['target_changes'],0)
        self.assertEqual(len(f.created_fines),2)
        first,verification=f.created_fines
        self.assertGreater(verification.frame['capture_timestamp_ms'],arcs[-1][0])
        self.assertNotEqual(first.frame['frame_id'],verification.frame['frame_id'])
        self.assertEqual(f.handoffs[0][1]['frame_id'],verification.frame['frame_id'])
        self.assertEqual(result['status'],'succeeded')

    def test_completed_between_frames_uses_capture_time_not_late_processing_clock(self):
        f=fixtures.RobotTrackingTests();f.setUp()
        f.task.drive._active_action=None;f.task.drive.phase='settling'
        f.task.memory._last_stamp=1000
        f.task.drive.last_arc_end=dict(at_ms=1050,phase='correcting')
        f.clock.ms=9000
        self.assertFalse(f.task._controlled_correction(1040))
        self.assertTrue(f.task._controlled_correction(1100))
        # A later ordinary frame has already observed completion. No timer or
        # permanent stopped-state marker carries the correction context on.
        f.task.memory._last_stamp=1100
        self.assertFalse(f.task._controlled_correction(1200))

    def test_only_owned_parking_correction_supplies_active_context(self):
        f=fixtures.RobotTrackingTests();f.setUp()
        for action,phase,expected in(('arc','correcting',True),('arc','observe',False),
                ('follow','following',False),(None,'settling',False)):
            with self.subTest(action=action,phase=phase):
                f.task.drive._active_action=action;f.task.drive.phase=phase
                self.assertEqual(f.task._controlled_correction(100),expected)
        f.task.drive._active_action='arc';f.task.drive.phase='correcting'
        f.task.fault='transport uncertain'
        self.assertFalse(f.task._controlled_correction(100))

    def test_static_exposure_starts_near_cycle_before_a_static_result_exists(self):
        f=fixtures.RobotTrackingTests();f.setUp()
        self.assertIsNone(f.task.drive._last_precise)
        f.task.drive.phase='fine_static'
        self.assertTrue(f.task._controlled_correction(100))
        f.task.drive.phase='settling'
        f.task.drive._fine_request={'purpose':'static'}
        self.assertTrue(f.task._controlled_correction(100))
        f.task.drive._fine_request=None
        f.task.drive._last_precise=(0,2,'static')
        f.task.drive.anchor=f.precise(f.point(),dict(frame_id=2,capture_timestamp_ms=50))
        for phase in('following','correcting','settling','fine_static'):
            f.task.drive.phase=phase
            self.assertTrue(f.task._controlled_correction(100))
        for phase in('parked','searching','arm'):
            f.task.drive.phase=phase
            self.assertFalse(f.task._controlled_correction(100))
        f.task.drive.phase='following';f.task.drive._last_precise=None
        self.assertFalse(f.task._controlled_correction(100))

    def test_same_version_far_reacquisition_after_search_has_no_near_context(self):
        f=fixtures.RobotTrackingTests();f.setUp();f.x=650
        image,frame=f.capture();coarse=f.track(image,frame)
        f.task.drive.version=f.task.memory.version
        f.task.drive._last_precise=(0,7,'static')
        f.task.drive.anchor=f.precise(f.point(),dict(frame_id=7,capture_timestamp_ms=0))
        f.task.drive.request_search('left')
        self.assertIsNone(f.task.drive._last_precise)
        state=f.motor.sync()
        decision=f.task.drive.update(coarse,state,frame['capture_timestamp_ms'],
            target_event={'target_valid':True,'visible':True})
        self.assertTrue(decision['command'].startswith('FOLLOW,'))
        self.assertEqual(f.task.drive.phase,'following')
        self.assertIsNone(f.task.drive.anchor)
        # Even a legacy static key cannot turn unanchored coarse following
        # into the near cycle if the fresh command ownership is ordinary.
        f.task.drive._last_precise=(0,7,'static')
        self.assertFalse(f.task._controlled_correction(frame['capture_timestamp_ms']))

    def test_releasing_far_fine_authority_preserves_evidence_but_unlocks_coarse(self):
        f=fixtures.RobotTrackingTests();f.setUp()
        source=f.track(f.point(),dict(frame_id=1,capture_timestamp_ms=100,
            timestamp_ms=100,clock_domain='k230_ticks_ms',image_size_px=[1280,960]))
        f.task.memory.update(source)
        fine=f.precise(f.point(),source)
        self.assertTrue(f.task.memory.bind_precise(fine,source,0))
        self.assertTrue(f.task.memory.snapshot()['anchor_valid'])
        f.task.memory.release_precise()
        self.assertFalse(f.task.memory.snapshot()['anchor_valid'])
        self.assertEqual(f.task.memory.last_precise,fine)
        self.assertEqual(f.task.memory.version,0)

    def test_static_far_transition_does_not_rebind_memory_before_coarse_approach(self):
        f=fixtures.RobotTrackingTests();f.setUp();f.x=650;f.geometry_invalid=True
        bound=Mock(wraps=f.task.memory.bind_precise);f.task.memory.bind_precise=bound
        for unused in range(100):
            f.tick()
            if f.task.job is not None:break
        else:self.fail('No initial static query')
        self.assertEqual(f.task.job['purpose'],'static')
        far_frame=f.task.job['session'].frame['frame_id']
        # The source image really contains the far target. Its initial coarse
        # geometry was unavailable; by result consumption normal coarse
        # navigation has recovered, independently of the fine answer.
        f.geometry_invalid=False
        for unused in range(200):
            f.tick()
            if f.task.decision.get('return_to_coarse'):break
        else:self.fail(f.task.snapshot())
        self.assertEqual(bound.call_count,0)
        self.assertFalse(f.task.memory.snapshot()['anchor_valid'])
        self.assertIsNone(f.task.drive.anchor)
        self.assertIsNone(f.task.drive._last_precise)
        self.assertFalse(f.task._controlled_correction(f.clock.now()))
        self.assertTrue(f.task.decision['command'].startswith('FOLLOW,'))
        result=f.finish()
        moving=[detail for event,detail in f.events if event=='tracking_fine_started'
                and detail['purpose']=='moving']
        self.assertTrue(moving)
        self.assertNotEqual(f.handoffs[0][1]['frame_id'],far_frame)
        self.assertEqual(result['status'],'succeeded')


if __name__=='__main__':unittest.main()
