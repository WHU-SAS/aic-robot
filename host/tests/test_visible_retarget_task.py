"""Current visible cues keep driving while stale fine capabilities are revoked.

Real Task/Memory/Drive/BlueTracker with the promoted one-frame configuration.
Images/pose output and UART/plant are synthetic protocol fixtures, not physical
vision accuracy, timing or driving-success evidence.
"""
import copy
import unittest

from host.tests import test_fine_observation_pause as paused


class VisibleRetargetTaskTests(unittest.TestCase):
    def fixture(self,capability='full',move_during_fine=False):
        helper=paused.FineObservationPauseTests()
        f,changes=helper.fixture('moving',capability,move_during_fine)
        return helper,f,changes

    def until(self,f,predicate,limit=5000):
        for unused in range(limit):
            f.tick()
            if predicate():return
            if f.task.fault is not None:self.fail(f.task.snapshot())
        self.fail(dict(phase=f.task.snapshot()['phase'],target=f.task.memory.snapshot(),
                       commands=f.motor.commands[-12:]))

    def test_first_raw_jump_frame_changes_steering_without_stop_or_search(self):
        unused,f,unused=self.fixture()
        f.x=900
        raw=[];track=f.task.track
        def record_native(image,frame):
            result=track(image,frame);raw.append(copy.deepcopy(result));return result
        f.task.track=record_native
        f.tick()
        self.assertEqual(f.motor.run,1)
        original=raw[-1]['visual_evidence']['center_pixel']
        command_at=len(f.motor.commands)
        f.y+=150
        f.clock.advance(f.task.period)
        f.tick()
        fresh=raw[-1]
        self.assertFalse(fresh['visual_evidence']['associated'])
        self.assertEqual(fresh['visual_evidence']['association_confirmations'],0)
        self.assertEqual(len(fresh['visual_candidates']),1)
        self.assertTrue(fresh['visual_candidates'][0]['distance_valid'])
        self.assertGreater(abs(original[0]-fresh['visual_candidates'][0]['center_pixel'][0]),
                           f.config['tracking']['image_size_px'][0]*
                           f.config['tracking']['max_center_jump_fraction'])
        commands=[command for unused,command in f.motor.commands[command_at:]]
        self.assertNotIn('STOP',commands)
        steering=next((command for command in commands if command.startswith('SPEED,')),None)
        self.assertIsNotNone(steering,commands)
        parts=steering.split(',')
        self.assertEqual(len(parts),4)
        self.assertGreater(int(parts[3]),0)
        left,right=map(int,parts[1:3])
        self.assertLess(left,right)  # Current native region moved to image-left.
        self.assertEqual(f.motor.run,1)
        self.assertFalse(f.task.drive._searching)
        self.assertFalse(f.handoffs)

    def test_visible_target_moved_farther_replaces_endpoint_with_a_new_bound(self):
        unused,f,unused=self.fixture()
        f.x=650
        f.tick()
        initial=next(command for unused,command in f.motor.commands
                     if command.startswith('FOLLOW,'))
        self.assertEqual(len(initial.split(',')),4)
        old_remaining=int(initial.split(',')[3])
        command_at=len(f.motor.commands)
        f.x=900
        f.clock.advance(f.task.period)
        f.tick()
        commands=[command for unused,command in f.motor.commands[command_at:]]
        self.assertNotIn('STOP',commands)
        current=next((command for command in commands if command.startswith('SPEED,')),None)
        self.assertIsNotNone(current,commands)
        parts=current.split(',')
        self.assertEqual(len(parts),4)  # A known farther route never clears to plain SPEED.
        self.assertGreater(int(parts[3]),old_remaining+150)
        self.assertTrue(f.task.coarse['visual_evidence']['distance_valid'])
        self.assertEqual(f.motor.run,1)
        self.assertFalse(f.task.drive._searching)
        self.assertFalse(f.created_fines)
        self.assertFalse(f.handoffs)

    def test_paused_moving_retarget_recomputes_in_motion_and_requires_new_static_handoff(self):
        for capability in ('full','range'):
            with self.subTest(capability=capability):
                helper,f,changes=self.fixture(capability,True)
                old=helper.complete_paused_image(f,changes,'moving')
                old_frame=old['observation']['frame_id']
                command_at=len(f.motor.commands)
                helper.consume_fresh_frame(f,old)
                self.until(f,lambda:len(f.created_fines)>=2)
                new=f.created_fines[1]
                self.assertGreater(new.frame['frame_id'],old_frame)
                new_start=next(row for row in f.fine_start_states
                               if row['frame']==new.frame['frame_id'])
                self.assertEqual(new_start['purpose'],'moving')
                self.assertEqual(new_start['run'],1)
                self.assertNotIn('STOP',[command for unused,command in f.motor.commands[command_at:]])
                self.assertFalse(f.task.drive._searching)
                helper.assert_old_fine_is_not_used(f,old)
                self.until(f,lambda:f.task.drive.anchor is not None and
                           f.task.drive.anchor['frame_id']==new.frame['frame_id'])
                self.assertIn(new.frame['frame_id'],f.bind_attempts)
                self.assertNotIn(old_frame,f.bind_attempts)
                self.assertFalse(f.handoffs)
                self.until(f,lambda:f.task.status=='succeeded')
                self.assertEqual(len(f.handoffs),1)
                handoff=f.handoffs[0][1]
                self.assertNotIn(handoff['frame_id'],(old_frame,new.frame['frame_id']))
                static=next(row for row in f.fine_start_states if row['frame']==handoff['frame_id'])
                self.assertEqual(static['purpose'],'static')
                self.assertEqual(static['run'],0)
                self.assertEqual(handoff['status'],'localized')
                self.assertTrue(f.task.last_parking['verification']['accepted'])
                self.assertNotIn(old_frame,f.bind_attempts)

    def test_visible_jump_with_unusable_coarse_range_keeps_direction_and_requests_fine(self):
        unused,f,unused=self.fixture()
        f.x=900
        f.tick()
        command_at=len(f.motor.commands)
        f.y+=150
        f.geometry_invalid=True
        f.clock.advance(f.task.period)
        f.tick()
        commands=[command for unused,command in f.motor.commands[command_at:]]
        self.assertNotIn('STOP',commands)
        steering=next((command for command in commands if command.startswith('SPEED,')),None)
        self.assertIsNotNone(steering,commands)
        self.assertEqual(len(steering.split(',')),3)
        left,right=map(int,steering.split(',')[1:3])
        self.assertEqual(left+right,0)
        self.assertLess(left,right)
        self.assertFalse(f.task.coarse['visual_evidence']['distance_valid'])
        self.assertFalse(f.task.drive._searching)
        self.assertFalse(f.handoffs)
        self.until(f,lambda:bool(f.created_fines))
        self.assertEqual(f.fine_start_states[0]['purpose'],'moving')
        self.assertEqual(f.fine_start_states[0]['run'],1)
        self.assertFalse(f.task.drive._searching)
        self.assertIsNone(f.task.drive.anchor)

    def test_rejected_binding_replaces_a_pending_drive_STOP_before_transmission(self):
        unused,f,changes=self.fixture(move_during_fine=True)
        f.fine_slices=20
        advance=f.Fine.advance
        def return_block_before_next_capture(session):
            advance(session)
            if changes and not getattr(session,'returned_to_origin',False):
                f.y-=40
                session.returned_to_origin=True
        f.Fine.advance=return_block_before_next_capture
        self.until(f,lambda:f.task.complete is not None)
        done=f.task.complete
        old_frame=done['observation']['frame_id']
        self.assertEqual(done['purpose'],'moving')
        self.assertEqual(done['observation']['status'],'localized')
        self.assertEqual(f.task.drive._active_action,'follow')
        ordinary=f.task.track
        def weak_scaled_protocol_input(point,frame):
            # Task-input fixture: a current associated colour region has
            # expanded4x, so its geometry is weak and size continuity cannot
            # decide whether the completed full pose remains the same target.
            coarse=ordinary(point,frame)
            evidence=dict(coarse['visual_evidence'])
            u,v=evidence['center_pixel']
            box=[int(u)-120,int(v)-120,240,240]
            evidence.update(bbox_px=box,pixels=40000,distance_valid=False,
                navigation_confirmed=False,reason='inconsistent_with_20mm_height')
            evidence.pop('ground_contact_proxy_Base_mm',None)
            coarse.update(bbox_px=box,visual_evidence=evidence,
                          visual_candidates=[dict(evidence)])
            return coarse
        f.task.track=weak_scaled_protocol_input
        command_at=len(f.motor.commands)
        timeline_at=len(f.task.timeline)
        f.tick()
        commands=[command for unused,command in f.motor.commands[command_at:]]
        events=f.task.timeline[timeline_at:]
        self.assertEqual(done['current_consistency'],'unknown')
        self.assertTrue(f.task.coarse['visual_evidence']['associated'])
        self.assertEqual(f.task.coarse['visual_evidence']['bbox_px'][2:],[240,240])
        self.assertTrue(any(row['event']=='tracking_drive' and
                            row['details'].get('command')=='STOP' for row in events))
        # The production Drive first proposed STOP and the production Memory
        # rejected binding. Neither is mocked; only the final intention may
        # reach the UART after the coordinator restores the owned follow.
        self.assertIn(old_frame,f.bind_attempts)
        self.assertTrue(any(row['event']=='tracking_fine_binding_rejected' and
                            row['details']['frame_id']==old_frame for row in events))
        self.assertNotIn('STOP',commands)
        speed=next((command for command in commands if command.startswith('SPEED,')),None)
        self.assertIsNotNone(speed,commands)
        self.assertEqual(len(speed.split(',')),3)
        self.assertEqual(sum(map(int,speed.split(',')[1:3])),0)
        self.assertEqual(f.motor.run,1)
        self.assertEqual(f.task.drive._active_action,'follow')
        self.assertIsNone(f.task.memory.last_precise)
        self.assertIsNone(f.task.drive.anchor)
        self.assertFalse(f.task.drive._searching)
        self.assertFalse(f.handoffs)
        self.assertIsNone(f.task.fault)
        self.until(f,lambda:len(f.created_fines)>=2)
        self.assertEqual(f.fine_start_states[1]['purpose'],'moving')
        self.assertEqual(f.fine_start_states[1]['run'],1)
        self.assertNotIn('STOP',[command for unused,command in f.motor.commands[command_at:]])

    def test_genuine_loss_still_stops_after_retained_fine_remaining_travel(self):
        unused,f,unused=self.fixture()
        self.until(f,lambda:f.task.drive.anchor is not None)
        command_at=len(f.motor.commands)
        f.hide_frames=10000
        self.until(f,lambda:any(command=='STOP' for unused,command in f.motor.commands[command_at:]))
        self.assertEqual(f.motor.run,0)
        self.assertFalse(f.task.target['visible'])
        self.assertFalse(f.handoffs)
        self.assertFalse(f.arms)
        self.assertIsNone(f.task.fault)

    def test_ACK_loss_during_current_visible_steering_still_stops(self):
        unused,f,unused=self.fixture()
        f.x=900
        f.tick()
        original=f.motor.expect;lost=[]
        def lose_changed_speed(command):
            state=original(command)
            if command.startswith('SPEED,') and not lost:
                lost.append(command)
                raise OSError('visible update ACK lost')
            return state
        f.motor.expect=lose_changed_speed
        f.y+=150
        f.clock.advance(f.task.period)
        f.tick()
        self.assertTrue(lost)
        self.assertEqual(f.motor.run,0)
        self.assertEqual(f.motor.commands[-1][1],'STOP')
        self.assertIn('visible update ACK lost',f.task.fault)
        self.assertEqual(f.task.snapshot()['phase'],'fault_wait')
        self.assertFalse(f.handoffs)


if __name__=='__main__':unittest.main()
