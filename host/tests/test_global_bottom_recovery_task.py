"""Task-level bottom-loss recovery with the real tracker/memory/drive.

Native blob images, motor dynamics and precise output are protocol fixtures.
The calibrated coarse projection and coordination code are real; these tests
do not establish physical accuracy, camera quality or field timing.
"""
import copy
import unittest

from device.k230.car.blue_tracker import BlueTracker
from device.k230.car.near_runtime import count_delta
from device.k230.vision.runtime import track_coarse
from host.tests import test_fine_observation_pause as paused
from host.tests import test_robot_tracking as fixtures


class SegmentMotor(fixtures.Motor):
    """Existing fake UART plus nominal segment expiry and loaded-wheel travel."""
    def __init__(self,clock,scale,travel_scale=1.):
        super().__init__(clock,scale)
        self.travel_scale=travel_scale
        self.deadline=None
        self.deadlines=0

    def advance(self):
        now=self.clock.now()
        expired=self.run and self.deadline is not None and now>=self.deadline
        if expired:self.clock.ms=self.deadline
        left,right=self.left_speed,self.right_speed
        try:
            self.left_speed*=self.travel_scale;self.right_speed*=self.travel_scale
            super().advance()
        finally:
            self.left_speed,self.right_speed=left,right
            self.clock.ms=now
        if expired:
            self.run,self.reason=0,3
            self.last_at=now
            self.deadlines+=1

    def expect(self,command):
        name=command.split(',')[0]
        if name=='FOLLOW':
            # Expire an old segment before applying its replacement.
            self.advance();self.deadline=None
        state=super().expect(command)
        if name=='FOLLOW':self.deadline=self.clock.now()+20000
        elif name=='STOP':self.deadline=None
        return state


class BottomScene:
    BOXES={
        # These complete boxes satisfy the real calibrated20mm joint shape
        # check. Preserve the original centres and bottom-exit trajectory.
        'whole_before':[575,730,150,220],
        'partial':[600,760,100,200],
        'partial_lower':[600,800,100,160],
        'partial_left':[0,800,100,160],
        'partial_right':[1180,800,100,160],
        'whole_far':[610,300,80,100],
        # Same remembered near target, with complete pixels but rejected
        # height geometry. A remote80x30 fragment at y300 is not this target:
        # its height differs7.3x and its centre jumps525px after1.5mm reverse.
        'whole_weak':[570,700,160,74],
        'foreign_weak':[610,300,80,30],
    }

    def __init__(self,test,travel_scale=1.):
        self.test=test
        helper=paused.FineObservationPauseTests()
        self.f,unused=helper.fixture('moving',move_during_fine=False)
        f=self.f
        for name in ('whole_before','whole_far'):
            box=self.BOXES[name]
            geometry=f.projection.blob_geometry(box)
            self.test.assertTrue(f.projection.cube_shape_geometry(
                box,geometry)['cube_shape_compatible'],name)
            low,high=f.config['tracking']['height_ratio_range']
            self.test.assertTrue(low<=geometry['observed_to_expected_height_ratio']<=high,name)
        self.test.assertLess(f.projection.blob_geometry(self.BOXES['whole_weak'])[
            'observed_to_expected_height_ratio'],f.config['tracking']['height_ratio_range'][0])
        f.fine_slices=1
        f.fail_every_fine=True
        f.motor=SegmentMotor(f.clock,f.config['mm_per_count'],travel_scale)
        f.motor.on_arc=f.apply_arc
        f.task.link=f.motor
        self.mode='whole_before'
        self.frames=[];self.saw_bottom_exit=False
        tracker=BlueTracker(f.config['tracking'],f.projection)
        scene=self
        class Blob:
            def rect(self):return scene.BOXES[scene.mode]
            def cx(self):x,y,w,h=self.rect();return x+w//2
            def cy(self):x,y,w,h=self.rect();return y+h//2
            def pixels(self):x,y,w,h=self.rect();return w*h*3//4
        class Image:
            def find_blobs(self,*args,**kwargs):return [] if scene.mode=='missing' else [Blob()]
        def track(image,frame):
            result=track_coarse(Image(),frame,tracker.detect_frame)
            self.frames.append(dict(frame=frame['frame_id'],at=frame['capture_timestamp_ms'],
                mode=self.mode,run=f.motor.run,evidence=copy.deepcopy(result['visual_evidence'])))
            return result
        f.task.track=track
        f.task.reset_tracker=tracker.reset

    def step(self,mode=None,ms=100):
        f=self.f
        if mode is not None:self.mode=mode
        before=len(f.motor.commands)
        f.clock.advance(ms);f.tick()
        self.test.assertIsNone(f.task.fault,f.task.snapshot())
        if f.task.target.get('event')=='exited' and f.task.target['last_disappearance_direction']=='bottom':
            self.saw_bottom_exit=True
        return [command for unused,command in f.motor.commands[before:]]

    def begin_reverse(self):
        for mode in ('whole_before','partial','partial_lower','missing'):
            self.step(mode)
        for unused in range(25):
            reverse=next((state for command,at,state in self.f.motor.replies
                          if command=='FOLLOW,-15,-15'),None)
            if reverse is not None:
                self.test.assertTrue(self.saw_bottom_exit)
                self.start=reverse
                self.start_command=len(self.f.motor.commands)
                self.initial_fines=len(self.f.fine_start_states)
                return
            self.step('missing')
        self.test.fail(dict(reason='No continuous bottom reverse',commands=self.f.motor.commands[-15:],
                            phase=self.f.task.snapshot()['phase']))

    def travel(self):
        state=self.f.motor.sync()
        return max(abs(count_delta(self.start[key],state[key]))*self.f.config['mm_per_count']
                   for key in ('encoder_a','encoder_b'))


class GlobalBottomRecoveryTaskTests(unittest.TestCase):
    def test_first_measured_whole_frame_stops_before_navigation_confirmation(self):
        s=BottomScene(self);s.begin_reverse();f=s.f
        f.config['tracking']['confirmation_frames']=3
        commands=s.step('whole_far')
        first=s.frames[-1]
        self.assertTrue(first['evidence']['distance_valid'])
        self.assertFalse(first['evidence']['navigation_confirmed'])
        self.assertIn('STOP',commands)
        command_at=len(f.motor.commands)
        for unused in range(15):
            s.step('whole_far')
            if len(f.fine_start_states)>s.initial_fines:break
        else:self.fail('Measured bottom recovery did not request a new static frame')
        self.assertEqual(f.fine_start_states[-1]['purpose'],'static')
        self.assertGreater(f.fine_start_states[-1]['frame'],first['frame'])
        self.assertFalse(any(c.startswith(('FOLLOW,','SPEED,','ARC,'))
            for at,c in f.motor.commands[command_at:]))
        self.assertFalse(f.handoffs)

    def test_foreign_weak_blue_does_not_stop_or_renew_bottom_recovery(self):
        s=BottomScene(self);s.begin_reverse();f=s.f
        commands_at=len(f.motor.commands)
        for unused in range(100):
            commands=s.step('foreign_weak')
            self.assertFalse(f.task.drive._bottom_full_view(
                f.task.coarse,f.task.target,f.clock.now()))
            if 'STOP' in commands:break
            self.assertEqual(f.motor.run,1)
            self.assertEqual((f.motor.left_speed,f.motor.right_speed),(-15,-15))
            self.assertEqual(len(f.fine_start_states),s.initial_fines)
            self.assertFalse(f.handoffs)
        else:self.fail('The original100mm budget did not stop reverse')
        self.assertGreater(s.travel(),98)
        self.assertLessEqual(s.travel(),100)
        self.assertEqual([c for at,c in f.motor.commands[commands_at:] if c=='STOP'],['STOP'])
        for unused in range(10):
            s.step('foreign_weak')
            self.assertEqual(f.motor.run,0)
            self.assertEqual(len(f.fine_start_states),s.initial_fines)
            self.assertFalse(f.handoffs)
        self.assertIsNone(f.task.drive.anchor)
        self.assertFalse(any(c.startswith('ARC,') or
            c.startswith(('FOLLOW,','SPEED,')) and c.split(',')[1:]!=['-15','-15']
            for at,c in f.motor.commands[commands_at:]))

    def test_task_bottom_exit_keeps_continuous_reverse_through_missing_and_partial(self):
        s=BottomScene(self);s.begin_reverse();f=s.f
        for mode in ('missing','missing','partial_lower','missing','partial_left','missing'):
            commands=s.step(mode)
            self.assertNotIn('STOP',commands)
            self.assertFalse(any(command.startswith('ARC,') for command in commands),commands)
            self.assertFalse(any(command.startswith(('FOLLOW,','SPEED,')) and
                command.split(',')[1:]!=['-15','-15'] for command in commands),commands)
            self.assertEqual(f.motor.run,1)
            self.assertEqual((f.motor.left_speed,f.motor.right_speed),(-15,-15))
            self.assertEqual(len(f.fine_start_states),s.initial_fines)
            self.assertFalse(f.handoffs)

    def test_complete_reappearance_stops_before_new_static_even_with_far_coarse_plan(self):
        for mode in ('whole_far','whole_weak'):
            with self.subTest(mode=mode):
                s=BottomScene(self);s.begin_reverse();f=s.f
                command_at=len(f.motor.commands)
                commands=s.step(mode)
                whole_frame=s.frames[-1]
                self.assertEqual(whole_frame['evidence']['edge_directions'],[])
                if mode=='whole_far':
                    self.assertTrue(whole_frame['evidence']['distance_valid'])
                    self.assertGreater(whole_frame['evidence']['ground_contact_proxy_Base_mm'][0],
                                       f.config['near_parking']['capture_Base_X_mm'])
                else:
                    self.assertFalse(whole_frame['evidence']['distance_valid'])
                    self.assertEqual(whole_frame['evidence']['reason'],'inconsistent_with_20mm_height')
                self.assertIn('STOP',commands)
                stop_at=next(at for at,command in f.motor.commands[command_at:] if command=='STOP')
                self.assertEqual(len(f.fine_start_states),s.initial_fines)
                for unused in range(15):
                    s.step(mode)
                    if len(f.fine_start_states)>s.initial_fines:break
                else:self.fail(f.task.snapshot())
                new=f.fine_start_states[-1]
                self.assertEqual(new['purpose'],'static')
                self.assertEqual(new['run'],0)
                self.assertGreater(new['frame'],whole_frame['frame'])
                capture=next(frame for frame in s.frames if frame['frame']==new['frame'])
                self.assertGreaterEqual(capture['at']-stop_at,f.config['stopped_observation_ms'])
                self.assertFalse(any(command.startswith(('FOLLOW,','SPEED,','ARC,'))
                    for at,command in f.motor.commands[command_at:] if at>stop_at),f.motor.commands[command_at:])
                self.assertFalse(f.handoffs)

    def test_budget_survives_deadline_and_target_versions_until_original100mm(self):
        s=BottomScene(self,travel_scale=.1);s.begin_reverse();f=s.f
        initial_version=f.task.memory.version
        for unused in range(205):s.step('missing')
        self.assertGreaterEqual(f.motor.deadlines,1)
        self.assertGreaterEqual(sum(command=='FOLLOW,-15,-15' for at,command in f.motor.commands),2)
        self.assertLess(s.travel(),40)
        f.motor.expect('STOP')  # Ordinary external pause, not button/link hold.
        for unused in range(5):
            s.step('missing')
            if f.motor.run:break
        self.assertEqual(f.motor.run,1)
        self.assertEqual((f.motor.left_speed,f.motor.right_speed),(-15,-15))
        s.step('partial_left');s.step('partial_right');s.step('missing')
        self.assertGreater(f.task.memory.version,initial_version)
        for unused in range(600):
            commands=s.step('missing')
            if 'STOP' in commands:break
        else:self.fail('The original100mm reverse budget never stopped the task')
        self.assertGreater(s.travel(),98)
        self.assertLessEqual(s.travel(),100)
        after_limit=len(f.motor.commands)
        for mode in ('partial_left','missing','partial_right','missing'):
            s.step(mode)
        self.assertFalse(any(command=='FOLLOW,-15,-15'
            for at,command in f.motor.commands[after_limit:]),
            f.motor.commands[-12:])

    def test_existing_button_and_link_stops_do_not_restart_reverse_on_fresh_frames(self):
        for reason in (2,4):
            with self.subTest(reason=reason):
                s=BottomScene(self);s.begin_reverse();f=s.f
                f.motor.run=0;f.motor.reason=reason
                command_at=len(f.motor.commands)
                for unused in range(12):s.step('missing')
                self.assertEqual(f.motor.run,0)
                self.assertTrue(f.task.snapshot()['motion_wait_reason'])
                self.assertFalse(any(command.startswith(('FOLLOW,','SPEED,','ARC,'))
                    for at,command in f.motor.commands[command_at:]))

    def test_old_bottom_history_after_completed_reacquisition_does_not_start_new_reverse(self):
        s=BottomScene(self);s.begin_reverse();f=s.f
        f.fail_every_fine=False
        # Supply a consistent protocol pose at the new, fully visible region.
        # A successful new static not_parked result completes view recovery;
        # the task can then continue normal forward approach.
        point=f.projection.pixel_to_plane([650,350],10)
        f.x,f.y=point[:2]
        for unused in range(15):
            s.step('whole_far')
            plan=(f.task.decision or {}).get('plan') or {}
            if (f.task.drive._bottom_reverse_budget is None and
                    f.task.drive._active_action=='follow' and plan.get('mode')=='coarse'):
                break
        else:self.fail(f.task.snapshot())
        self.assertEqual(f.task.memory.last_disappearance_direction,'bottom')
        self.assertIsNone(f.task.drive._bottom_reverse_budget)
        self.assertTrue(f.task.target['associated'])
        self.assertEqual(f.task.drive._active_action,'follow')
        self.assertFalse(f.task.drive._search_reverse_required)
        self.assertGreater(len(f.fine_start_states),s.initial_fines)
        self.assertEqual(f.fine_start_states[-1]['purpose'],'static')
        self.assertEqual(f.fine_start_states[-1]['run'],0)
        self.assertIsNone(f.task.drive.anchor)
        self.assertIsNone(f.task.drive._last_precise)
        self.assertFalse(f.task.memory.snapshot()['anchor_valid'])
        self.assertFalse(f.handoffs)
        command_at=len(f.motor.commands)
        for unused in range(50):s.step('missing')
        # Finish the bounded coarse route before the existing loss search.
        # The old bottom hint cannot start another reverse budget or renew
        # the completed forward endpoint.
        self.assertTrue(f.task.drive._searching)
        self.assertIsNone(f.task.drive.anchor)
        self.assertEqual(f.task.memory.last_disappearance_direction,'bottom')
        self.assertIsNone(f.task.drive._bottom_reverse_budget)
        commands=[command for at,command in f.motor.commands[command_at:]]
        self.assertIn('STOP',commands)
        self.assertNotIn('FOLLOW,-15,-15',commands)
        self.assertNotIn('ARC,-10,-10',commands)
        stopped=False
        for command in commands:
            if command=='STOP':stopped=True
            if command.startswith(('FOLLOW,','SPEED,')):
                fields=command.split(',')
                if stopped:
                    self.assertEqual(len(fields),3)
                    self.assertEqual(sum(int(value) for value in fields[1:]),0)
                else:
                    self.assertEqual(len(fields),4)
                    self.assertTrue(all(int(value)>=0 for value in fields[1:]))

    def test_partial_again_while_settling_cannot_use_the_brief_whole_frame_for_fine(self):
        s=BottomScene(self);s.begin_reverse();f=s.f
        original_counts=list(f.task.drive._bottom_reverse_budget['reverse_start_counts'])
        self.assertIn('STOP',s.step('whole_far'))
        for unused in range(3):
            commands=s.step('partial_lower')
            self.assertEqual(len(f.fine_start_states),s.initial_fines)
            self.assertFalse(any(command.startswith('ARC,') for command in commands))
            self.assertFalse(any(command.startswith(('FOLLOW,','SPEED,')) and
                command.split(',')[1:]!=['-15','-15'] for command in commands))
        self.assertEqual(f.task.drive._bottom_reverse_budget['reverse_start_counts'],original_counts)
        self.assertFalse(f.handoffs)

    def test_rejected_response_during_reverse_still_stops_in_fault_wait(self):
        s=BottomScene(self);s.begin_reverse();f=s.f
        expect=f.motor.expect
        rejected=[]
        def reject_status(command):
            state=expect(command)
            if command=='STATUS' and not rejected:
                rejected.append(True)
                raise RuntimeError('Unexpected command result: result=2')
            return state
        f.motor.expect=reject_status
        command_at=len(f.motor.commands)
        f.clock.advance(100);f.tick()
        self.assertTrue(rejected)
        self.assertEqual(f.motor.run,0)
        self.assertEqual(f.motor.commands[-1][1],'STOP')
        self.assertEqual(f.task.snapshot()['phase'],'fault_wait')
        self.assertIn('result=2',f.task.fault)
        for unused in range(10):f.tick()
        self.assertFalse(any(command.startswith(('FOLLOW,','SPEED,','ARC,'))
            for at,command in f.motor.commands[command_at:]))


if __name__=='__main__':unittest.main()
