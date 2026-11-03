"""Known remaining travel must control motion before blocking fine JPEG work.

Protocol camera/solver/UART doubles run the real Task, Memory and Drive. These
tests check ordering and nominal travel, not physical stopping accuracy.
"""
import copy
import unittest

from device.k230.car.near_control import target_x
from device.k230.car.parking_window import bounds_mm
from host.tests import test_pending_fine_motion as pending


class PreJpegMotionTests(unittest.TestCase):
    def scene(self, ratio=0):
        self.helper = pending.PendingFineMotionTests()
        return self.helper.scene(ratio,start_ms=259,slices=20)

    def near_retarget(self, scene, remaining):
        f=scene.f
        self.helper.until(scene,lambda:f.task.complete is not None)
        f.motor.advance()
        f.x=target_x(f.config['workspace'])+remaining
        f.y=sum(bounds_mm(f.config['workspace'],f.x)['Base_Y_mm'])/2
        f.fine_slices=100
        return len(scene.starts),len(scene.motion)

    def test_exhausted_current_coarse_stops_before_constructing_or_compressing_a_retry(self):
        for ratio in (0,.1):
            with self.subTest(ratio=ratio):
                scene=self.scene(ratio)
                f=scene.f
                starts,commands=self.near_retarget(scene,246.5673-target_x(f.config['workspace']))
                self.helper.until(scene,lambda:self.helper.first_stop(scene) is not None)
                stop=self.helper.first_stop(scene)
                self.assertEqual(len(scene.starts),starts)
                self.assertEqual(len(f.created_fines),starts)
                self.assertEqual(stop['purpose'],None)
                self.assertFalse(any(row['command'].startswith(('SPEED,','FOLLOW,'))
                    for row in scene.motion[commands:]))
                self.assertFalse(f.handoffs)
                self.helper.until(scene,lambda:bool(self.helper.starts(scene,'static')))
                static=self.helper.starts(scene,'static')[0]
                self.assertGreater(static['frame']['capture_timestamp_ms'],stop['at'])
                self.assertEqual(f.motor.run,0)
                self.assertFalse(f.handoffs)

    def test_short_remaining_distance_sets_the_existing_profile_before_jpeg_starts(self):
        scene=self.scene()
        f=scene.f
        starts,commands=self.near_retarget(scene,30)
        self.helper.until(scene,lambda:len(scene.starts)>starts or
            self.helper.first_stop(scene) is not None)
        speeds=[row['command'] for row in scene.motion[commands:]
                if row['command'].startswith('SPEED,')]
        self.assertTrue(speeds)
        for command in speeds:
            a,b=map(int,command.split(',')[1:3])
            self.assertEqual(len(command.split(',')),4)
            self.assertGreater(int(command.split(',')[3]),0)
            self.assertGreaterEqual((a+b)/2,f.config['near_parking']['finish_mm_s'])
            self.assertLess((a+b)/2,f.config['near_parking']['cruise_mm_s'])

    def test_recorded_frame38_already_beyond_destination_does_not_request_fine(self):
        scene=self.scene()
        f=scene.f
        self.helper.until(scene,lambda:f.task.job is not None)
        coarse=copy.deepcopy(f.task.coarse)
        f.task._cancel_job('fixture_replay_next_coarse')
        stamp=f.clock.now()-103
        # Minimal visual facts retained in overshoot_repair_repeat_01's real
        # frame38. Its current coarse range was already below target250.324.
        payload=dict(bbox_px=[541,527,169,273],center_pixel=[624.,663.],
            ground_contact_proxy_Base_mm=[246.5673,6.088792,-67.37527],
            image_midline_error_px=-16.,confirmations=1,pixels=42501)
        coarse.update(payload,frame_id=38,capture_timestamp_ms=stamp,timestamp_ms=stamp)
        coarse['visual_evidence'].update(payload,timestamp_ms=stamp,
            visible=True,associated=True,ambiguous=False,distance_valid=True,
            navigation_confirmed=True,reason=None,edge_directions=[])
        drive=f.task.drive
        decision=drive.update(coarse,f.task.state,f.clock.now(),
            target_version=drive.version,target_event=dict(
                visible=True,associated=True,target_valid=True))
        self.assertEqual(decision['command'],'STOP')
        self.assertIsNone(decision['request_fine'])
        self.assertIsNone(drive._fine_request)
        self.assertEqual(decision['status'],'fine_distance_exhausted')

    def test_pre_start_check_catches_travel_spent_since_the_request_was_formed(self):
        scene=self.scene()
        f=scene.f
        self.helper.until(scene,lambda:f.task.job is not None)
        task,drive=f.task,f.task.drive
        request=copy.deepcopy(drive._fine_request)
        image,frame=copy.deepcopy(task.job['session'].point),dict(task.job['session'].frame)
        task._cancel_job('fixture_finished_previous_solver')
        now=f.clock.now()
        frame.update(frame_id=frame['frame_id']+100,capture_timestamp_ms=now)
        point=list(image);point[0]=target_x(f.config['workspace'])+20
        drive._coarse_reference=dict(capture_timestamp_ms=now,point=point,ray=list(point))
        request.update(capture_timestamp_ms=now)
        drive._fine_request=request
        drive._active_action='follow'
        f.clock.advance(300)
        starts=len(f.created_fines)
        task._start_job('moving',image,frame)
        self.assertEqual(len(f.created_fines),starts)
        self.assertIsNone(task.job)
        self.assertEqual(f.motor.run,0)
        self.assertEqual(scene.motion[-1]['command'],'STOP')
        self.assertIsNone(drive._fine_request)
        self.assertEqual(drive.phase,'settling')

    def test_first_normal_capture_keeps_the_existing_cruise_speed(self):
        scene=self.scene()
        f=scene.f
        self.helper.until(scene,lambda:f.task.job is not None)
        self.assertGreater(scene.starts[0]['point'][0],400)
        speed=next(row['command'] for row in reversed(scene.motion)
                   if row['command'].startswith(('SPEED,','FOLLOW,')))
        a,b=map(int,speed.split(',')[1:3])
        self.assertEqual(len(speed.split(',')),4)
        self.assertGreater(int(speed.split(',')[3]),0)
        self.assertEqual((a+b)/2,f.config['near_parking']['cruise_mm_s'])


if __name__=='__main__':
    unittest.main()
