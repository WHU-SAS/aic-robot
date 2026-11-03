"""Moving fine handoff keeps its camera-time distance ahead of optional vision.

Real Task, Drive and Memory run against a deterministic image/UART plant. The
200ms coarse and23ms request costs are explicit scheduling inputs, not measured
hardware latency or claims about physical positioning accuracy.
"""
import copy
import unittest

from host.tests import test_robot_tracking as fixtures
from device.k230.car.continuous_math import forward_x_per_mm
from device.k230.car.near_control import target_x


class FineHandoffPriorityTests(unittest.TestCase):
    def scene(self, failed=False, unassociated=False, completed_at_ready=False):
        f=fixtures.RobotTrackingTests()
        f.setUp()
        f.x=650
        f.trace=[]
        f.ready=None
        f.motor.request_cost_ms=23
        f.config['tracking_task']['fine_observation_overhead_fraction']=0
        capture,track,expect=f.task.capture,f.task.track,f.motor.expect
        def camera():
            f.trace.append(dict(kind='capture',at=f.clock.now()))
            return capture()
        def coarse(point,frame):
            f.trace.append(dict(kind='track',at=f.clock.now()))
            if f.ready is not None:
                f.coarse_cost_ms=200
            return track(point,frame)
        def uart(command):
            f.trace.append(dict(kind='uart',at=f.clock.now(),command=command,
                a=f.motor.a,b=f.motor.b,point=f.point(),
                plan=copy.deepcopy((f.task.decision or {}).get('plan'))))
            return expect(command)
        f.task.capture=camera;f.task.track=coarse;f.motor.expect=uart
        advance=f.Fine.advance
        def fine(session):
            advance(session)
            if session.index or session.phase is not None or f.ready is not None:
                return
            if failed:
                session.latest['status']='no_complete_face'
            if completed_at_ready:
                f.motor.advance()
                f.motor.run=0;f.motor.reason=6
            f.ready=dict(kind='ready',at=f.clock.now(),frame=dict(session.frame),
                point=list(session.point),a=f.motor.a,b=f.motor.b)
            f.trace.append(f.ready)
        f.Fine.advance=fine
        if unassociated:
            f.task._match_fine=lambda *args:{'status':'unknown','reason':'different_source'}
        return f

    def until(self,f,predicate,limit=1800):
        for unused in range(limit):
            f.tick()
            self.assertIsNone(f.task.fault,f.task.snapshot())
            if predicate():return
        self.fail(f.task.snapshot())

    def ready_tail(self,f):
        return f.trace[f.trace.index(f.ready)+1:]

    def first_handoff(self,f):
        self.until(f,lambda:f.ready is not None)
        self.until(f,lambda:any(row['kind']=='capture' for row in self.ready_tail(f)))
        tail=self.ready_tail(f)
        first_camera=next(i for i,row in enumerate(tail) if row['kind']=='capture')
        controls=[row for row in tail[:first_camera] if row['kind']=='uart']
        self.assertTrue(controls,'Fine ready was followed by a camera before any new UART')
        self.assertEqual(controls[0]['command'],'STATUS')
        command=next((row for row in controls[1:]
            if row['command']=='STOP' or row['command'].startswith(('SPEED,','FOLLOW,'))),None)
        self.assertIsNotNone(command,'Fresh STATUS must be followed by the fine distance command before vision')
        self.assertFalse(any(row['kind']=='track' for row in tail[:tail.index(command)]))
        return command

    def wait_for_anchor(self,f):
        self.until(f,lambda:f.task.drive.anchor is not None)
        return copy.deepcopy(f.task.drive.anchor)

    def test_ready_status_and_fine_distance_command_precede_next_200ms_coarse(self):
        f=self.scene()
        command=self.first_handoff(f)
        self.assertTrue(command['command'].startswith('SPEED,'),command)
        self.assertEqual(len(command['command'].split(',')),4)
        # In this plant, only the23ms STATUS lies between ready and dispatch.
        self.assertEqual(command['at']-f.ready['at'],23)
        self.assertEqual(f.task.drive.anchor['frame_id'],f.ready['frame']['frame_id'])

    def test_handoff_subtracts_computing_and_status_wheel_travel_from_fine_distance(self):
        f=self.scene()
        command=self.first_handoff(f)
        self.assertTrue(command['command'].startswith('SPEED,'),command)
        plan=command['plan']
        self.assertEqual(plan['frame_id'],f.ready['frame']['frame_id'])
        # Initial capture state is retained by the real odometry interpolation;
        # compare against physical plant travel rather than reproducing its sums.
        capture_point=f.ready['point']
        uncompensated=(capture_point[0]-target_x(f.config['workspace']))/forward_x_per_mm(f.projection)
        self.assertGreater(uncompensated-plan['remaining_mm'],20)
        self.assertAlmostEqual(plan['predicted_center_Base_mm'][0],command['point'][0],delta=2)
        self.assertAlmostEqual(int(command['command'].split(',')[3]),plan['remaining_mm'],delta=.5)
        self.assertTrue(plan['odometry_predicted'])
        self.assertGreater(sum(abs(row['a'])+abs(row['b']) for row in self.ready_tail(f)
            if row['kind']=='uart'),0)

    def test_fine_distance_already_reached_stops_before_any_new_coarse_work(self):
        f=self.scene()
        advance=f.Fine.advance
        def close_range(session):
            advance(session)
            if session.index or session.phase is not None:return
            # Protocol scenario: the same-frame fine solver reports a shorter
            # range than the coarse proxy. Image/CV accuracy is tested elsewhere.
            point=list(session.point)
            point[0]=target_x(f.config['workspace'])+10
            n=f.projection.normal
            point[2]=(f.projection.offset+10-n[0]*point[0]-n[1]*point[1])/n[2]
            session.latest=f.precise(point,session.frame)
        f.Fine.advance=close_range
        command=self.first_handoff(f)
        self.assertEqual(command['command'],'STOP')
        self.assertEqual(command['at']-f.ready['at'],23)
        self.assertEqual(command['plan']['status'],'stop_for_verification')
        self.assertFalse(f.arms)

    def test_failed_or_unassociated_result_never_creates_a_distance_anchor(self):
        for options in ({'failed':True},{'unassociated':True}):
            with self.subTest(options=options):
                f=self.scene(**options)
                self.until(f,lambda:f.ready is not None)
                self.assertIsNone(f.task.drive.anchor)
                self.assertIsNone(f.task.memory.last_precise)
                self.until(f,lambda:any(row['kind']=='capture' for row in self.ready_tail(f)))
                self.assertIsNone(f.task.drive.anchor)
                self.assertFalse(f.arms)

    def test_mcu_endpoint_completed_during_solve_cannot_revive_old_moving_image(self):
        f=self.scene(completed_at_ready=True)
        self.until(f,lambda:f.ready is not None)
        self.until(f,lambda:any(row['kind']=='capture' for row in self.ready_tail(f)))
        self.assertIsNone(f.task.drive.anchor)
        self.assertFalse(f.arms)
        first_camera=next(i for i,row in enumerate(self.ready_tail(f)) if row['kind']=='capture')
        self.assertFalse(any(row['kind']=='uart' and row['command'].startswith(('SPEED,','FOLLOW,'))
            for row in self.ready_tail(f)[:first_camera]))
        self.assertTrue(any(command=='STATUS' for at,command in f.motor.commands
            if at>=f.ready['at']))

    def test_coarse_range_noise_cannot_replace_the_accepted_fine_distance(self):
        f=self.scene();anchor=self.wait_for_anchor(f)
        track=f.task.track
        def wrong_proxy(point,frame):
            coarse=track(point,frame)
            coarse['ground_contact_proxy_Base_mm'][0]+=400
            coarse['visual_evidence']['ground_contact_proxy_Base_mm']=list(coarse['ground_contact_proxy_Base_mm'])
            return coarse
        f.task.track=wrong_proxy
        version=f.task.memory.version
        for unused in range(3):
            f.task._force_observe=True;f.tick()
            self.assertEqual(f.task.memory.version,version)
            self.assertFalse(f.task.target['confirmed_moved'])
            self.assertEqual(f.task.drive.anchor,anchor)
            self.assertEqual(f.task.decision['plan']['frame_id'],anchor['frame_id'])
            self.assertLess(f.task.decision['plan']['remaining_mm'],250)

    def test_one_far_fragment_cannot_delete_anchor_or_extend_endpoint(self):
        f=self.scene();anchor=self.wait_for_anchor(f)
        previous_remaining=f.task.decision['plan']['remaining_mm']
        track=f.task.track
        def fragment(point,frame):
            coarse=track(point,frame)
            far=[1000.,-200.,point[2]]
            pixel=f.projection.base_to_pixel(far)
            cue=coarse['visual_evidence']
            cue.update(center_pixel=pixel,bbox_px=[int(pixel[0])-9,int(pixel[1])-9,18,19],
                pixels=111,ground_contact_proxy_Base_mm=far,association_confirmations=1)
            coarse.update(center_pixel=pixel,bbox_px=cue['bbox_px'],
                ground_contact_proxy_Base_mm=far,image_midline_error_px=pixel[0]-640)
            return coarse
        f.task.track=fragment;f.task._force_observe=True;f.tick()
        self.assertFalse(f.task.target['confirmed_moved'])
        self.assertEqual(f.task.drive.anchor,anchor)
        if f.task.decision.get('plan'):
            self.assertLessEqual(f.task.decision['plan']['remaining_mm'],previous_remaining+2)
        recent=[row['command'] for row in self.ready_tail(f) if row['kind']=='uart']
        self.assertFalse(any(c.startswith(('SPEED,','FOLLOW,')) and len(c.split(','))==4
            and int(c.split(',')[3])>previous_remaining+2 for c in recent))

    def test_confirmed_real_target_translation_revokes_the_old_distance_anchor(self):
        f=self.scene();anchor=self.wait_for_anchor(f)
        f.coarse_cost_ms=0
        f.y+=20  # Retain same-size/same-candidate continuity while shifting.
        for unused in range(8):
            f.task._force_observe=True;f.tick()
            if f.task.target.get('confirmed_moved'):break
        self.assertTrue(f.task.target['confirmed_moved'],f.task.target)
        self.assertNotEqual(f.task.memory.version,anchor.get('target_version',0))
        self.assertIsNone(f.task.drive.anchor)
        self.assertFalse(f.task.memory.snapshot()['anchor_valid'])

    def test_final_park_and_arm_still_use_a_new_stopped_image_and_pause_after_close(self):
        f=self.scene()
        self.until(f,lambda:f.task.status=='succeeded')
        self.assertEqual(len(f.handoffs),1)
        handoff=f.handoffs[0][1]
        self.assertNotEqual(handoff['frame_id'],f.ready['frame']['frame_id'])
        saved=next(e for e in f.task.evidence if e['frame']['frame_id']==handoff['frame_id'])
        self.assertEqual(saved['frame']['tracking_purpose'],'static')
        self.assertEqual(f.task.last_parking['verification_observation']['frame_id'],handoff['frame_id'])
        closed_at=next(event[1] for event in f.arms[0].io.events
            if event[0]=='goals' and event[3]==fixtures.PLAN['close'])
        self.assertFalse(any(row['kind']=='capture' and row['at']>closed_at for row in f.trace))


if __name__=='__main__':
    unittest.main()
