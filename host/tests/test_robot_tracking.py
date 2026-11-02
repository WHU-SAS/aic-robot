"""Real target/drive/arm state machines with synthetic images and fake UARTs."""
import copy
import json
import unittest
from host.project_paths import ROOT
from device.k230.robot.tracking import TrackingTask, ReobserveRequired
from device.k230.arm.pick_execution import PickExecution
from device.k230.car.ground_projection import GroundProjection
from device.k230.car.near_control import target_x
from device.k230.car.parking_window import bounds_mm
from tools.deploy.k230_car_align import build_bundle
from host.tests.test_car_near_control import PlanarPlant


class Clock:
    def __init__(self): self.ms = 0
    def now(self): return self.ms
    def advance(self, amount): self.ms += amount


class Motor:
    def __init__(self, clock, scale):
        self.clock, self.scale, self.commands = clock, scale, []
        self.a = self.b = 0
        self.run = 0
        self.reason = 0
        self.on_arc = None
        self.left_speed = self.right_speed = 0
        self.last_at = clock.now()
    def advance(self):
        seconds=(self.clock.now()-self.last_at)/1000
        self.last_at=self.clock.now()
        if self.run and seconds:
            left,right=self.left_speed*seconds,self.right_speed*seconds
            self.a+=round(left/self.scale)
            self.b-=round(right/self.scale)
            if self.on_arc:self.on_arc(left,right)
    def sync(self):
        self.advance()
        return dict(ms=self.clock.now(), run=self.run, encoder_a=self.a, encoder_b=self.b,
                    result=0, reason=self.reason, pwm=0, left_pwm=0, right_pwm=0)
    def expect(self, command):
        self.advance()
        self.commands.append((self.clock.now(), command))
        name = command.split(',')[0]
        if name in ('FOLLOW', 'SPEED'):
            self.run,self.reason = 1,0
            self.left_speed,self.right_speed=map(float,command.split(',')[1:])
        if name == 'STOP': self.run,self.reason = 0,1
        if name == 'ARC':
            left,right = map(int,command.split(',')[1:])
            self.a += round(left/self.scale)
            self.b -= round(right/self.scale)
            if self.on_arc: self.on_arc(left,right)
            self.run = 0
            self.reason = 8
        state = self.sync()
        return state


PLAN = {'hover':[2000]*5+[2332], 'descend':[2100]*5+[2332],
        'close':[2100]*5+[2150], 'lift':[2200]*5+[2150]}


class ArmIO:
    def __init__(self, clock, empty=False):
        self.clock, self.empty = clock, empty
        self.positions = list(PLAN['hover'])
        self.events = []
    def send_goals(self, goals, axes):
        self.events.append(('goals',self.clock.now(),list(axes),list(goals)))
        for axis in axes:
            self.positions[axis-1] = goals[axis-1]
            if axis == 6 and goals[5] == PLAN['close'][5] and not self.empty:
                self.positions[5] += 40
    def read_positions(self, axes):
        self.events.append(('read',self.clock.now(),list(axes)))
        return [self.positions[i-1] for i in axes]
    def send_ready2(self):
        self.events.append(('ready2',self.clock.now()))
        return {'goal_sent_ticks_ms':self.clock.now()}
    def close(self): self.events.append(('close',self.clock.now()))


class RobotTrackingTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.config = build_bundle(near_parking=True)[0]
        self.config['tracking_task'] = json.loads((ROOT/'configs/car/tracking_task.json').read_text())
        p = self.config['projection']
        self.projection = GroundProjection(p['K'],p['D'],p['T'],p['plane'],
            p['forward_axis_Base'],p['motion_origin_Base_mm'])
        self.x = target_x(self.config['workspace'])
        self.y = sum(bounds_mm(self.config['workspace'],self.x)['Base_Y_mm'])/2
        self.frame = 0
        self.move_when = lambda: False
        self.moved = self.lift_changes = self.empty_first = self.fail_first_fine = False
        self.fail_every_fine = False
        self.fine_slices = 20
        self.geometry_invalid = False
        self.hide_frames = 0
        self.arms, self.handoffs, self.created_fines, self.events = [], [], [], []
        self.motor = Motor(self.clock,self.config['mm_per_count'])
        self.motor.on_arc = self.apply_arc
        parent = self
        class Fine:
            def __init__(self, config, clock):
                self.phase = None
                self.latest = self.latest_ready_ms = None
                self.ticks = 0
                self.events = []
                self.index = len(parent.created_fines)
                parent.created_fines.append(self)
            def start(self, image, frame, bbox):
                self.point,self.frame = copy.deepcopy(image),dict(frame)
                self.phase = 'geometry'
            def advance(self):
                self.ticks += 1
                parent.clock.advance(20)
                if self.ticks >= parent.fine_slices:
                    self.latest = parent.precise(self.point,self.frame)
                    if parent.fail_every_fine or (parent.fail_first_fine and self.index == 0):
                        self.latest['status'] = 'no_complete_face'
                    self.latest_ready_ms = parent.clock.now()
                    self.phase = None
            def take_evidence(self, label):
                return dict(label=label,jpeg=b'original',frame=dict(self.frame),
                            observation=self.latest or {'status':'processing_interrupted'})
            def cancel(self): self.phase = None
        self.Fine = Fine
        self.task = TrackingTask(self.capture,self.track,self.motor,self.projection,
            self.config,self.clock.now,self.begin_arm,Fine,Fine,
            emit=lambda event,details:self.events.append((event,details)))

    def point(self):
        n = self.projection.normal
        return [self.x,self.y,(self.projection.offset+10-n[0]*self.x-n[1]*self.y)/n[2]]

    def capture(self):
        self.motor.advance()
        self.frame += 1
        if not self.moved and self.move_when():
            self.y += 40
            self.moved = True
            self.moved_at = self.clock.now()
        return self.point(),dict(frame_id=self.frame,capture_timestamp_ms=self.clock.now(),
            timestamp_ms=self.clock.now(),clock_domain='k230_ticks_ms',image_size_px=[1280,960])

    def apply_arc(self, left, right):
        plant = PlanarPlant(self.point(),self.projection.normal,
            self.config['near_parking']['track_width_mm'],self.projection.forward_axis_Base,
            self.projection.motion_origin_Base_mm)
        plant.vl,plant.vr = left,right
        for unused in range(100): plant.advance(.01)
        point = plant.center()
        self.x,self.y = point[:2]

    def track(self, point, frame):
        if self.hide_frames:
            self.hide_frames -= 1
            return dict(frame,schema='aic.vision_observation.v1',mode='coarse',status='lost',
                visual_evidence=dict(visible=False,associated=False,ambiguous=False,
                    distance_valid=False,candidate_count=0,edge_directions=[],
                    timestamp_ms=frame['capture_timestamp_ms']))
        pixel = self.projection.base_to_pixel(point)
        contact = [point[k]-10*self.projection.normal[k] for k in range(3)]
        lifted = (self.lift_changes and self.task.arm is not None and self.task.arm.phase == 'lift')
        distance_valid = not (self.geometry_invalid or lifted)
        if lifted: pixel[1] -= 160
        evidence = dict(visible=True,associated=True,ambiguous=False,candidate_count=1,
            center_pixel=pixel,bbox_px=[int(pixel[0])-30,int(pixel[1])-30,60,60],
            timestamp_ms=frame['capture_timestamp_ms'],distance_valid=distance_valid,edge_directions=[])
        if not distance_valid: evidence['reason']='inconsistent_with_20mm_height'
        if distance_valid: evidence['ground_contact_proxy_Base_mm'] = contact
        return dict(frame,schema='aic.vision_observation.v1',mode='coarse',status='tracking',
            center_pixel=pixel,bbox_px=evidence['bbox_px'],confirmations=3,
            ground_contact_proxy_Base_mm=contact,image_midline_error_px=pixel[0]-640,
            visual_evidence=evidence)

    def precise(self, point, frame):
        return dict(frame,schema='aic.vision_observation.v1',mode='precise',status='localized',
            cube_center_Base_mm=list(point),coordinate_frame='Base',units='mm',
            reference_point='cube_geometric_center',xy_compensation_applied=False,
            quality={'reprojection_rms_px':.3},payload={'status':'complete_face_candidate',
                'cube_center_Base_mm':list(point),'poses':[{'face_reprojection_rms_px':.3}]})

    def begin_arm(self, observation, version):
        self.handoffs.append((self.clock.now(),copy.deepcopy(observation),version))
        io = ArmIO(self.clock,self.empty_first and not self.arms)
        arm = PickExecution(PLAN,{},io,version,self.clock.now)
        self.arms.append(arm)
        return arm

    def tick(self, n=1):
        for unused in range(n):
            self.task.step()
            self.clock.advance(5)

    def finish(self):
        for unused in range(1800):
            self.tick()
            if self.task.status == 'succeeded': return self.task.snapshot()
        self.fail(self.task.snapshot())

    def test_normal_flow_keeps_observing_during_fine_and_grasp(self):
        result = self.finish()
        self.assertEqual(result['status'],'succeeded')
        self.assertEqual(len(self.arms),1)
        self.assertGreater(result['metrics']['observations'],10)
        self.assertLessEqual(result['metrics']['max_observation_gap_ms'],125)
        self.assertTrue(result['grasp_confirmed'])
        self.assertEqual(result['parking']['verification_observation']['frame_id'],
                         self.handoffs[0][1]['frame_id'])
        self.assertTrue(any(item['frame']['frame_id']==self.handoffs[0][1]['frame_id']
                            for item in self.task.evidence))
        self.assertFalse(any(c.startswith(('FOLLOW','SPEED','ARC')) for t,c in self.motor.commands))

    def test_far_start_reaches_parking_and_grasp_with_one_moving_and_one_static_image(self):
        self.x=650
        result=self.finish()
        self.assertTrue(any(c.startswith('FOLLOW,') for t,c in self.motor.commands))
        self.assertEqual(result['metrics']['fine_started'],2)
        self.assertEqual(result['metrics']['arm_attempts'],1)
        self.assertLessEqual(result['metrics']['max_observation_gap_ms'],125)
        self.assertLessEqual(len(self.task.evidence),2)

    def test_move_during_fine_cancels_old_image_and_never_hands_it_to_arm(self):
        self.move_when = lambda:self.frame >= 4
        self.finish()
        self.assertGreaterEqual(self.task.metrics['fine_cancelled'],1)
        self.assertGreaterEqual(self.task.metrics['target_changes'],1)
        self.assertTrue(all(obs['capture_timestamp_ms'] >= self.moved_at and v>0
                            for t,obs,v in self.handoffs))

    def test_move_in_axis4_lead_overwrites_instead_of_sending_old_remaining_axes(self):
        self.move_when = lambda:self.task.arm is not None and self.task.arm.phase == 'descend_lead'
        self.finish()
        first = self.arms[0]
        ready = [v for v in first.io.events if v[0]=='ready2']
        self.assertEqual(len(ready),1)
        self.assertFalse(any(v[0]=='goals' and v[2]==[1,2,3,5,6] for v in first.io.events))
        self.assertGreaterEqual(len(self.arms),2)

    def test_empty_grip_recovers_once_waits_500ms_and_tries_again(self):
        self.empty_first = True
        result = self.finish()
        ready = [v for v in self.arms[0].io.events if v[0]=='ready2']
        self.assertEqual(len(ready),1)
        self.assertGreaterEqual(self.handoffs[1][0]-ready[0][1],500)
        self.assertEqual(result['metrics']['recoveries'],1)

    def test_normal_held_object_image_motion_does_not_open_jaw(self):
        self.lift_changes = True
        self.finish()
        self.assertFalse(any(e[0]=='ready2' for e in self.arms[0].io.events))
        self.assertEqual(self.task.metrics['target_changes'],0)

    def test_failed_static_measurement_changes_view_before_trying_again(self):
        self.fail_first_fine = True
        self.finish()
        self.assertTrue(any(c.startswith('ARC,') for t,c in self.motor.commands))
        self.assertGreaterEqual(self.task.metrics['fine_started'],2)

    def test_planning_blind_interval_requires_a_new_frame_before_hover(self):
        original = self.task.begin_arm
        frame_at_plan = []
        def plan(obs, version):
            arm = original(obs,version)
            frame_at_plan.append(self.frame)
            self.clock.advance(350)
            return arm
        self.task.begin_arm = plan
        while not self.arms: self.tick()
        self.assertFalse(self.arms[0].io.events)
        self.tick()
        self.assertGreater(self.frame,frame_at_plan[0])
        self.assertEqual(self.arms[0].io.events[0][0],'goals')

    def test_planning_move_never_sends_the_old_first_hover(self):
        original = self.task.begin_arm
        def plan(obs, version):
            arm = original(obs,version)
            if len(self.arms)==1:
                self.y += 40
                self.moved = True
                self.moved_at = self.clock.now()
                self.clock.advance(350)
            return arm
        self.task.begin_arm = plan
        self.finish()
        self.assertFalse(any(e[0]=='goals' for e in self.arms[0].io.events))
        self.assertGreaterEqual(len(self.arms),2)

    def test_unknown_after_planning_drops_unsent_engine_without_opening_jaw(self):
        original = self.task.begin_arm
        def plan(obs, version):
            arm = original(obs,version)
            if len(self.arms)==1: self.hide_frames=1
            return arm
        self.task.begin_arm = plan
        self.finish()
        self.assertEqual([e[0] for e in self.arms[0].io.events],['close'])
        self.assertGreaterEqual(len(self.arms),2)

    def test_visible_coarse_geometry_rejection_can_be_rescued_by_full_static_pose(self):
        self.geometry_invalid=True
        self.finish()
        self.assertEqual(len(self.arms),1)
        self.assertFalse(any(c.startswith(('FOLLOW','ARC')) for t,c in self.motor.commands))

    def test_only_scene_plan_unavailable_changes_view_not_bad_calibration(self):
        original=self.task.begin_arm
        attempts=[]
        def plan(obs,version):
            attempts.append(version)
            if len(attempts)==1:raise ReobserveRequired('No IK solution')
            return original(obs,version)
        self.task.begin_arm=plan
        self.finish()
        self.assertGreaterEqual(len(attempts),2)
        self.assertTrue(any(c.startswith('ARC') for t,c in self.motor.commands))

    def test_bad_calibration_waits_without_repeated_motion_or_terminal_fail(self):
        self.task.begin_arm=lambda obs,version:(_ for _ in ()).throw(ValueError('calibration mismatch'))
        self.tick(200)
        self.assertEqual(self.task.status,'running')
        self.assertIn('calibration mismatch',self.task.fault)
        self.assertFalse(any(c.startswith(('ARC','FOLLOW')) for t,c in self.motor.commands))

    def test_superseded_intermediate_images_are_released_without_storage_stops(self):
        self.fail_every_fine = True
        self.tick(700)
        self.assertEqual(self.task.status,'running')
        self.assertGreaterEqual(self.task.metrics['fine_started'],5)
        self.assertLessEqual(len(self.task.evidence),2)
        self.assertGreaterEqual(self.task.metrics['obsolete_frames_released'],3)
        self.assertTrue(all(item['frame']['target_version']==self.task.memory.version
                            for item in self.task.evidence))

    def test_broken_evidence_copy_does_not_skip_exit_stop(self):
        while self.task.job is None: self.tick()
        def bad_evidence(label):raise ValueError('JPEG evidence unavailable')
        self.task.job['session'].take_evidence=bad_evidence
        self.motor.run=1
        self.task.close()
        self.assertEqual(self.motor.run,0)
        self.assertEqual(self.motor.commands[-1][1],'STOP')

    def test_cooperative_planning_keeps_capturing_before_any_servo_goal(self):
        original=self.task.begin_arm
        def plan(obs,version):
            for unused in range(25):
                self.clock.advance(20)
                yield {'stage':'candidate','target_version':version}
            yield {'stage':'complete','target_version':version,'frame_id':obs['frame_id'],
                   'execution':original(obs,version)}
        self.task.begin_arm=plan
        result=self.finish()
        self.assertEqual(result['metrics']['planning_steps'],26)
        self.assertLessEqual(result['metrics']['max_observation_gap_ms'],125)
        self.assertEqual(result['metrics']['arm_attempts'],1)

    def test_target_move_cancels_old_planning_without_emitting_its_hover(self):
        original=self.task.begin_arm
        cancelled=[]
        def plan(obs,version):
            try:
                for unused in range(25):
                    self.clock.advance(20)
                    yield {'stage':'candidate','target_version':version}
                yield {'stage':'complete','target_version':version,'frame_id':obs['frame_id'],
                       'execution':original(obs,version)}
            finally:cancelled.append(version)
        self.task.begin_arm=plan
        self.move_when=lambda:self.task.planning is not None
        self.finish()
        self.assertIn(0,cancelled)
        self.assertGreaterEqual(self.task.metrics['plans_cancelled'],1)
        self.assertTrue(all(version>0 for t,obs,version in self.handoffs))

    def test_cooperative_scene_plan_failure_changes_view_and_retries(self):
        original=self.task.begin_arm
        started=[]
        def plan(obs,version):
            started.append(version)
            yield {'stage':'snapshot','target_version':version}
            if len(started)==1:raise ReobserveRequired('No complete IK')
            yield {'stage':'complete','target_version':version,'frame_id':obs['frame_id'],
                   'execution':original(obs,version)}
        self.task.begin_arm=plan
        self.finish()
        self.assertGreaterEqual(len(started),2)
        self.assertTrue(any(command.startswith('ARC') for t,command in self.motor.commands))

    def test_many_cheap_planning_yields_share_budget_instead_of_each_sleeping(self):
        original=self.task.begin_arm
        planning_time=[]
        def plan(obs,version):
            started=self.clock.now()
            for unused in range(1000):
                self.clock.advance(1)
                yield {'stage':'scalar','target_version':version}
            planning_time.append(self.clock.now()-started)
            yield {'stage':'complete','target_version':version,'frame_id':obs['frame_id'],
                   'execution':original(obs,version)}
        self.task.begin_arm=plan
        result=self.finish()
        self.assertLess(planning_time[0],1600)
        self.assertEqual(result['metrics']['planning_steps'],1001)
        self.assertLessEqual(result['metrics']['max_observation_gap_ms'],125)

    def test_pending_visual_suspicion_does_not_finish_before_next_evidence(self):
        while self.task.arm is None or self.task.arm._lift_stable_since is None:self.tick()
        engine=self.task.arm
        self.task.target['movement_state']='suspected'
        before=len(engine.io.events)
        for unused in range(4):
            self.clock.advance(50)
            self.task._step_arm()
        self.assertGreater(len(engine.io.events),before)
        self.assertEqual(engine.status,'running')
        self.assertTrue(engine.snapshot()['success_ready'])
        self.assertEqual(self.task.status,'running')
        self.task.memory.invalidate('confirmed_outside_claw')
        self.clock.advance(50)
        self.task._step_arm()
        self.assertTrue(any(e[0]=='ready2' for e in engine.io.events))
        self.assertNotEqual(self.task.status,'succeeded')

    def test_resolved_suspicion_releases_success_without_an_extra_fixed_wait(self):
        while self.task.arm is None or self.task.arm._lift_stable_since is None:self.tick()
        self.task.target['movement_state']='suspected'
        for unused in range(4):
            self.clock.advance(50)
            self.task._step_arm()
        self.task.target['movement_state']='holding'
        self.clock.advance(50)
        self.task._step_arm()
        self.assertEqual(self.task.status,'succeeded')


if __name__ == '__main__': unittest.main()
