"""User-selected close/lift policy: feedback only until success or failed grip.

Real Task/PickExecution use an explicit camera, encoder and arm-IO protocol
plant. Costs model the known scheduler conflict; no physical trial is claimed.
"""
import copy
import unittest
from unittest.mock import Mock

from host.tests import test_robot_tracking as fixtures


class GripVisionPauseTests(unittest.TestCase):
    def scene(self, phase='close', observation_ms=0, read_ms=183, pause=True, empty=False):
        f=fixtures.RobotTrackingTests()
        f.setUp()
        f.coarse_cost_ms=observation_ms
        f.empty_first=empty
        f.read_ms=read_ms
        f.fail_next=False;f.lose_next=False
        f.reads=[];f.arm_events=[]
        factory=f.begin_arm
        def begin(observation,version):
            arm=factory(observation,version)
            arm.emit=lambda event,details:f.arm_events.append(dict(
                event=event,at_ms=f.clock.now(),details=copy.deepcopy(details)))
            read=arm.io.read_positions
            def feedback(axes):
                started=f.clock.now();stage=arm.phase
                f.clock.advance(f.read_ms if len(axes)==6 else 30)
                if f.fail_next and stage=='lift':
                    f.fail_next=False
                    f.reads.append(dict(phase=stage,started=started,ended=f.clock.now(),failed=True))
                    raise OSError('synthetic missing feedback')
                values=read(axes)
                if f.lose_next and stage=='lift':
                    f.lose_next=False
                    values[5]=fixtures.PLAN['close'][5]+29
                f.reads.append(dict(phase=stage,started=started,ended=f.clock.now(),
                                    failed=False,positions=list(values)))
                return values
            arm.io.read_positions=feedback
            return arm
        f.task.begin_arm=begin
        if not pause:
            # Explicit old-scheduler comparison; no production configuration.
            f.task._vision_paused_for_grip=lambda state:False
        self.until(f,lambda:f.task.arm is not None and f.task.arm.phase==phase)
        f.paused_frame=f.frame
        f.capture_spy=Mock(wraps=f.task.capture);f.task.capture=f.capture_spy
        f.track_spy=Mock(wraps=f.task.track);f.task.track=f.track_spy
        f.memory_spy=Mock(wraps=f.task.memory.update);f.task.memory.update=f.memory_spy
        return f

    def until(self,f,predicate,limit=2000):
        for unused in range(limit):
            f.tick()
            self.assertIsNone(f.task.fault,f.task.snapshot())
            if predicate():return
        self.fail(dict(phase=f.task.snapshot()['phase'],events=f.arm_events[-4:]))

    def assert_no_vision(self,f):
        self.assertEqual(f.frame,f.paused_frame)
        f.capture_spy.assert_not_called()
        f.track_spy.assert_not_called()
        f.memory_spy.assert_not_called()

    def test_real_close_goal_pauses_all_visual_work_through_lift_and_success(self):
        f=self.scene()
        self.assertTrue(f.task.snapshot()['vision_paused_for_grip'])
        f.task.arm_context=Mock(side_effect=AssertionError('No visual arm context during grip'))
        self.until(f,lambda:f.task.status=='succeeded')
        self.assert_no_vision(f)
        f.task.arm_context.assert_not_called()
        self.assertEqual(f.task.metrics['recoveries'],0)
        self.assertTrue(f.task.arm.grasp_confirmed)
        self.assertEqual(f.task.arm.grip_confirmation['threshold_counts'],30)
        self.assertGreaterEqual(f.task.arm.grip_confirmation['stable_ms'],200)
        self.assertFalse(any(r['event']=='recovery_goal_sent' for r in f.arm_events))

    def test_old_103_and_138ms_gap_costs_no_longer_starve_the_original_success_window(self):
        for cost in (98,133):
            with self.subTest(observation_ms=cost):
                old=self.scene('lift',observation_ms=cost,pause=False)
                self.until(old,lambda:len([r for r in old.arm_events
                    if r['event']=='lift_observation_gap'])>=3)
                gaps=[r['details']['idle_gap_ms'] for r in old.arm_events
                      if r['event']=='lift_observation_gap']
                self.assertTrue(all(g>=cost+5 for g in gaps))
                self.assertEqual(old.task.arm.status,'running')
                new=self.scene('lift',observation_ms=cost)
                self.until(new,lambda:new.task.status=='succeeded')
                self.assert_no_vision(new)
                self.assertFalse(any(r['event']=='lift_observation_gap' for r in new.arm_events))
                self.assertEqual(new.task.arm.policy['poll_interval_ms'],50)
                self.assertEqual(new.task.arm.policy['lift_gripper_stable_ms'],200)
                self.assertEqual(new.task.arm.grip_confirmation['delta_counts'],40)

    def test_old_visual_suspicion_or_changed_camera_inputs_cannot_cancel_an_active_grip(self):
        f=self.scene()
        f.task.target['movement_state']='suspected'
        f.y+=200  # Only future camera input changes; arm feedback still holds.
        self.until(f,lambda:f.task.status=='succeeded')
        self.assert_no_vision(f)
        self.assertEqual(f.task.metrics['recoveries'],0)
        self.assertFalse(any(event[0]=='ready2' for event in f.task.arm.io.events))

    def test_empty_grip_restarts_coarse_observation_with_the_existing_recovery_interlock(self):
        f=self.scene(empty=True)
        self.until(f,lambda:f.task.arm is not None and f.task.arm.status=='recovering')
        self.assert_no_vision(f)
        self.assertFalse(f.task.snapshot()['vision_paused_for_grip'])
        recovery=next(r for r in f.arm_events if r['event']=='recovery_goal_sent')
        self.assertEqual(recovery['details']['reason'],'empty_grip')
        commands=len(f.motor.commands)
        f.tick()
        self.assertEqual(f.frame,f.paused_frame+1)
        self.assertEqual(f.capture_spy.call_count,1)
        self.assertEqual(f.track_spy.call_count,1)
        self.assertEqual(f.memory_spy.call_count,1)
        self.assertFalse(any(c.startswith(('FOLLOW,','SPEED,','ARC,'))
            for stamp,c in f.motor.commands[commands:]))

    def test_real_axis6_lost_grip_recovers_immediately_then_resumes_vision(self):
        f=self.scene('lift')
        f.lose_next=True
        self.until(f,lambda:f.task.arm is not None and f.task.arm.status=='recovering')
        self.assert_no_vision(f)
        recovery=next(r for r in f.arm_events if r['event']=='recovery_goal_sent')
        self.assertEqual(recovery['details']['reason'],'grip_lost')
        self.assertEqual(recovery['at_ms'],f.reads[-1]['ended'])
        self.assertEqual(f.task.arm.grip_confirmation['delta_counts'],29)
        self.assertFalse(f.task.snapshot()['vision_paused_for_grip'])
        f.tick()
        self.assertEqual(f.frame,f.paused_frame+1)
        self.assertEqual(f.memory_spy.call_count,1)

    def test_unknown_feedback_stays_paused_and_retries_the_original_feedback_path(self):
        f=self.scene('lift')
        f.fail_next=True
        self.until(f,lambda:any(r['failed'] for r in f.reads))
        self.assertTrue(f.task.snapshot()['vision_paused_for_grip'])
        self.assertEqual(f.task.arm.feedback_status,'unknown')
        self.assertEqual(f.task.arm.status,'running')
        self.assert_no_vision(f)
        self.until(f,lambda:f.task.status=='succeeded')
        self.assert_no_vision(f)
        self.assertEqual(f.task.metrics['recoveries'],0)

    def test_due_status_and_chassis_revocation_remain_active_without_camera(self):
        for phase in ('close','lift'):
            with self.subTest(phase=phase):
                f=self.scene(phase)
                f.motor.a+=round(2/f.config['mm_per_count'])
                f.clock.advance(501)
                packets=len(f.motor.commands)
                f.task.step()
                self.assert_no_vision(f)
                self.assertIn('STATUS',[c for stamp,c in f.motor.commands[packets:]])
                self.assertEqual(f.task.arm.status,'recovering')
                invalid=f.task.metrics['last_chassis_invalidation']
                self.assertEqual(invalid['tolerance_mm'],1)
                recovery=next(r for r in f.arm_events if r['event']=='recovery_goal_sent')
                self.assertEqual(recovery['details']['target_context']['reason'],
                                 'chassis_moved_during_grasp')
                self.assertTrue(recovery['details']['target_context']['vision_paused_for_grip'])

    def test_hover_descend_and_queued_close_still_observe_before_the_close_goal(self):
        f=self.scene('descend')
        self.assertFalse(f.task.snapshot()['vision_paused_for_grip'])
        while f.task.arm.pending_stage!='close':
            f.tick()
            self.assertFalse(f.task.snapshot()['vision_paused_for_grip'])
        before=f.frame
        f.task._force_observe=True
        f.tick()
        self.assertGreater(f.frame,before)
        self.assertEqual(f.task.arm.phase,'close')
        self.assertTrue(f.task.snapshot()['vision_paused_for_grip'])


if __name__=='__main__':
    unittest.main()
