"""Real target/drive/arm state machines with synthetic images and fake UARTs."""
import copy
import json
import unittest
from unittest.mock import Mock, patch
from host.project_paths import ROOT
from device.k230.robot.tracking import TrackingTask, ReobserveRequired, run_tracking
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
        self.request_cost_ms = 0
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
        self.clock.advance(self.request_cost_ms)
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
        self.fine_cost_ms = 20
        self.coarse_cost_ms = 0
        self.observation_rows = []
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
                parent.clock.advance(parent.fine_cost_ms)
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
        phase=('arm' if self.task.arm is not None else
               'planning' if self.task.planning is not None else
               self.task.job['purpose'] if self.task.job is not None else
               self.task.complete['purpose'] if self.task.complete is not None else 'observe')
        self.observation_rows.append(dict(at=frame['capture_timestamp_ms'],phase=phase,
            run=self.task.state['run'],motor_ms=self.task.state['ms'],
            motor_received=self.task.state.get('_k230_received_ms'),
            completing=self.task.complete is not None,before_arm=self.task._fresh_before_arm))
        self.clock.advance(self.coarse_cost_ms)
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
        self.assertLessEqual(result['metrics']['max_observation_gap_ms'],275)
        self.assertTrue(result['grasp_confirmed'])
        self.assertEqual(result['parking']['verification_observation']['frame_id'],
                         self.handoffs[0][1]['frame_id'])
        self.assertTrue(any(item['frame']['frame_id']==self.handoffs[0][1]['frame_id']
                            for item in self.task.evidence))
        saved = next(item['frame'] for item in self.task.evidence
                     if item['frame']['frame_id']==self.handoffs[0][1]['frame_id'])
        self.assertEqual(saved['paired_coarse']['frame_id'], saved['frame_id'])
        self.assertEqual(saved['paired_coarse']['capture_timestamp_ms'],
                         saved['capture_timestamp_ms'])
        self.assertIsNone(saved['localization_bbox_px'])
        self.assertFalse(any(c.startswith(('FOLLOW','SPEED','ARC')) for t,c in self.motor.commands))

    def test_far_start_reaches_parking_and_grasp_with_one_moving_and_one_static_image(self):
        self.x=650
        result=self.finish()
        self.assertTrue(any(c.startswith('FOLLOW,') for t,c in self.motor.commands))
        self.assertEqual(result['metrics']['fine_started'],2)
        self.assertEqual(result['metrics']['arm_attempts'],1)
        self.assertLessEqual(result['metrics']['max_observation_gap_ms'],275)
        self.assertLessEqual(len(self.task.evidence),2)
        moving = next(item['frame'] for item in self.task.evidence
                      if item['frame']['tracking_purpose']=='moving')
        self.assertEqual(moving['localization_bbox_px'],
                         moving['paired_coarse']['bbox_px'])

    def test_running_chassis_model_disagreement_does_not_cancel_fixed_target(self):
        self.task._observe(force_status=True)
        version = self.task.memory.version
        self.motor.run = 1
        # Real failed ARC increments: the image stayed still while wheel
        # odometry predicted a large yaw. Exercise coordinator STATUS wiring.
        self.motor.a += 526
        self.motor.b -= 8
        self.clock.advance(100)
        self.task._observe(force_status=True)
        self.assertEqual(self.task.memory.version, version)
        self.assertTrue(self.task.target['motion_model_uncertain'])
        self.assertEqual(self.task.metrics['target_changes'], 0)
        self.motor.run = 0
        for unused in range(4):
            self.clock.advance(250)
            self.task._observe(force_status=True)
        self.assertEqual(self.task.memory.version, version)
        self.assertEqual(self.task.metrics['target_changes'], 0)
        self.assertNotEqual(self.task.target['movement_state'], 'suspected')

    def test_brief_unknown_during_static_compute_reuses_original_image(self):
        original=self.task.track
        hidden=[]
        def track(point,frame):
            if (self.task.job is not None and self.task.job['purpose']=='static' and
                    self.task.job['session'].ticks>3 and not hidden):
                self.hide_frames=1
                hidden.append(frame['frame_id'])
            return original(point,frame)
        self.task.track=track
        result=self.finish()
        self.assertTrue(hidden)
        self.assertEqual(result['metrics']['fine_started'],1)
        self.assertEqual(len(self.arms),1)
        self.assertFalse(any(c.startswith(('FOLLOW','ARC')) for t,c in self.motor.commands))

    def test_completed_static_unknown_defers_without_busy_capture_or_rephotograph(self):
        original=self.task.track
        hidden=[]
        def track(point,frame):
            if self.task.complete is not None and not hidden:
                self.hide_frames=1
                hidden.append(frame['capture_timestamp_ms'])
            return original(point,frame)
        self.task.track=track
        result=self.finish()
        self.assertTrue(hidden)
        following=next(row['at'] for row in self.observation_rows if row['at']>hidden[0])
        self.assertGreaterEqual(following-hidden[0],self.task.stationary_period)
        self.assertEqual(result['metrics']['fine_started'],1)
        self.assertEqual(len(self.arms),1)

    def test_run_diagnostics_retain_parking_and_first_hover_transitions(self):
        self.x=650
        result=self.finish()
        events=[row['event'] for row in result['timeline']]
        self.assertIn('tracking_fine_started',events)
        self.assertIn('tracking_fine_completed',events)
        self.assertIn('tracking_first_hover',events)
        stops=[row for row in result['timeline'] if row['event']=='tracking_drive' and
               row['details']['command']=='STOP']
        self.assertEqual(len(stops),1)
        self.assertTrue(stops[0]['details']['status'])
        hover=next(row for row in result['timeline'] if row['event']=='tracking_first_hover')
        self.assertGreater(hover['details']['photo_to_hover_ms'],0)

    def test_failed_moving_fine_and_one_missing_coarse_frame_do_not_split_approach(self):
        self.x=650
        self.fail_first_fine=True
        original=self.task.track
        hidden=[]
        def track(point,frame):
            if self.task.job is not None and self.task.job['purpose']=='moving' and not hidden:
                self.hide_frames=1
                hidden.append(frame['frame_id'])
            return original(point,frame)
        self.task.track=track
        result=self.finish()
        self.assertTrue(hidden)
        self.assertEqual(result['metrics']['target_changes'],0)
        self.assertEqual(result['metrics']['fine_started'],3)
        self.assertEqual(len([c for t,c in self.motor.commands if c=='STOP']),1)
        self.assertEqual(len(self.arms),1)

    def test_arc_timeout_context_survives_target_log_rollover_and_cleanup_stop(self):
        self.fail_every_fine = True
        original = self.motor.expect
        def expect(command):
            state = original(command)
            if command.startswith('ARC,'):
                self.motor.reason = state['reason'] = 3
            return state
        self.motor.expect = expect
        for unused in range(600):
            self.tick()
            if self.task.drive.arc_timeouts:
                break
        self.assertEqual(self.task.drive.arc_timeouts, 1)
        ended = copy.deepcopy(self.task.snapshot()['last_arc_end'])
        self.assertEqual(ended['state']['reason'], 3)
        for index in range(70):
            self.task._record('tracking_target', {'version': index})
        self.task.close()
        self.assertEqual(self.task.state['reason'], 1)
        result = self.task.snapshot()
        self.assertEqual(result['arc_timeouts'], 1)
        self.assertEqual(result['last_arc_end'], ended)

    def test_move_during_fine_cancels_old_image_and_never_hands_it_to_arm(self):
        self.fine_slices = 40
        self.move_when = lambda:self.task.job is not None and self.task.job['session'].ticks>=3
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

    def test_closing_contact_moves_blue_region_before_five_stable_samples_without_reopening(self):
        original=self.task.track
        contact_frames=[]
        def track(point,frame):
            result=original(point,frame)
            if self.task.arm is not None and self.task.arm.phase=='close':
                contact_frames.append(frame['frame_id'])
                shift=min(60,15*len(contact_frames))
                result['center_pixel'][0]+=shift
                result['bbox_px'][0]+=shift
            return result
        self.task.track=track
        result=self.finish()
        self.assertGreaterEqual(len(contact_frames),2)
        self.assertEqual(result['metrics']['target_changes'],0)
        self.assertEqual(result['metrics']['arm_attempts'],1)
        self.assertTrue(result['grasp_confirmed'])
        self.assertFalse(any(e[0]=='ready2' for e in self.arms[0].io.events))

    def test_empty_grip_still_recovers_with_contact_visual_motion(self):
        self.empty_first=True
        original=self.task.track
        def track(point,frame):
            result=original(point,frame)
            if self.task.arm is not None and self.task.arm.phase=='close':
                result['center_pixel'][0]+=40
                result['bbox_px'][0]+=40
            return result
        self.task.track=track
        result=self.finish()
        self.assertEqual(len([e for e in self.arms[0].io.events if e[0]=='ready2']),1)
        self.assertEqual(result['metrics']['recoveries'],1)
        self.assertGreaterEqual(result['metrics']['arm_attempts'],2)

    def test_successful_task_cleanup_preserves_grip_without_ready_or_open_goal(self):
        self.finish()
        arm=self.arms[0]
        before=list(arm.io.positions)
        goals=[e for e in arm.io.events if e[0]=='goals']
        self.task.close()
        self.assertEqual(arm.io.positions,before)
        self.assertEqual([e for e in arm.io.events if e[0]=='goals'],goals)
        self.assertFalse(any(e[0]=='ready2' for e in arm.io.events))

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

    def test_unknown_after_planning_pauses_unsent_hover_without_replanning(self):
        original = self.task.begin_arm
        def plan(obs, version):
            arm = original(obs,version)
            if len(self.arms)==1: self.hide_frames=1
            return arm
        self.task.begin_arm = plan
        self.finish()
        self.assertEqual(len(self.arms),1)
        self.assertEqual(self.task.metrics['fine_started'],1)
        self.assertFalse(any(e[0]=='ready2' for e in self.arms[0].io.events))
        self.assertEqual(self.arms[0].io.events[0][0],'goals')

    def test_unknown_during_planning_keeps_original_unsent_plan(self):
        original=self.task.begin_arm
        hidden=[]
        def plan(obs,version):
            for unused in range(35):
                self.clock.advance(20)
                yield {'stage':'candidate','target_version':version}
            yield {'stage':'complete','target_version':version,'frame_id':obs['frame_id'],
                   'execution':original(obs,version)}
        self.task.begin_arm=plan
        track=self.task.track
        def hide_once(point,frame):
            if self.task.planning is not None and not hidden:
                self.hide_frames=1
                hidden.append(frame['frame_id'])
            return track(point,frame)
        self.task.track=hide_once
        result=self.finish()
        self.assertTrue(hidden)
        self.assertEqual(result['metrics']['plans_cancelled'],0)
        self.assertEqual(result['metrics']['arm_plans'],1)
        self.assertEqual(result['metrics']['fine_started'],1)

    def test_continued_unknown_before_first_hover_can_search_instead_of_waiting_forever(self):
        original=self.task.begin_arm
        def plan(obs,version):
            arm=original(obs,version)
            if len(self.arms)==1:self.hide_frames=8
            return arm
        self.task.begin_arm=plan
        for unused in range(1000):
            self.tick()
            if self.arms and not self.hide_frames:
                break
        else:
            self.fail('The synthetic occlusion never finished')
        # Fake ARC completes a whole turn immediately. After several hidden
        # frames its target is outside the image; do not ask an out-of-frame
        # fake tracking result to authorize a new grasp. Real tracker
        # reacquisition is covered in test_car_tracking_reacquisition.
        self.assertEqual([e[0] for e in self.arms[0].io.events],['close'])
        self.assertTrue(any(c.startswith('ARC,') for t,c in self.motor.commands))
        self.assertIsNone(self.task.fault)
        self.assertEqual(self.task.status,'running')

    def test_ambiguous_before_hover_preserves_one_frame_but_can_reobserve_if_persistent(self):
        for count in (1,8):
            with self.subTest(ambiguous_frames=count):
                self.setUp()
                original=self.task.begin_arm
                remaining=[0]
                def plan(obs,version):
                    arm=original(obs,version)
                    if len(self.arms)==1:remaining[0]=count
                    return arm
                track=self.task.track
                def ambiguous(point,frame):
                    result=track(point,frame)
                    if remaining[0]:
                        remaining[0]-=1
                        result['visual_evidence'].update(ambiguous=True,associated=False,candidate_count=2)
                    return result
                self.task.begin_arm=plan
                self.task.track=ambiguous
                for unused in range(1000):
                    self.tick()
                    if self.arms and not remaining[0]:
                        break
                else:
                    self.fail('The synthetic ambiguity never finished')
                if count==1:
                    result=self.finish()
                    self.assertEqual(result['status'],'succeeded')
                    self.assertEqual(len(self.arms),1)
                    self.assertEqual(result['metrics']['fine_started'],1)
                else:
                    self.assertEqual([e[0] for e in self.arms[0].io.events],['close'])
                    self.assertTrue(any(c.startswith('ARC,') for t,c in self.motor.commands))
                    self.assertIsNone(self.task.fault)
                    self.assertEqual(self.task.status,'running')

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
        self.assertLessEqual(result['metrics']['max_observation_gap_ms'],275)
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
        self.assertLessEqual(result['metrics']['max_observation_gap_ms'],275)

    def _timed_prehover_run(self,stationary_period,status_period,adaptive_idle=False):
        self.setUp()
        self.task.stationary_period=stationary_period
        self.task.stationary_status_period=status_period
        self.coarse_cost_ms=35
        self.motor.request_cost_ms=23
        self.fine_cost_ms=5
        self.fine_slices=80
        original=self.task.begin_arm
        def plan(observation,version):
            for index in range(100):
                self.clock.advance(4)
                yield {'stage':'ik','target_version':version,'index':index}
            yield {'stage':'complete','target_version':version,
                   'frame_id':observation['frame_id'],'execution':original(observation,version)}
        self.task.begin_arm=plan
        for unused in range(2000):
            self.task.step()
            self.clock.advance(self.task.idle_delay_ms() if adaptive_idle else 5)
            goals=[event for arm in self.arms for event in arm.io.events if event[0]=='goals']
            if goals:
                at=goals[0][1]
                return {'first_goal_at':at,'status_queries':sum(
                    command=='STATUS' and tick<=at for tick,command in self.motor.commands),
                    'observations':list(self.observation_rows),
                    'static_photo_at':self.created_fines[0].frame['capture_timestamp_ms'],
                    'target_changes':self.task.metrics['target_changes']}
        self.fail(self.task.snapshot())

    def test_costed_stationary_schedule_reduces_prehover_queries_and_wall_clock(self):
        old=self._timed_prehover_run(100,100)
        new=self._timed_prehover_run(250,500)
        self.assertLess(new['status_queries'],old['status_queries'])
        self.assertLess(new['first_goal_at'],old['first_goal_at'])
        planning=[row for row in new['observations'] if row['phase']=='planning']
        self.assertTrue(planning)
        stationary=[row for row in new['observations'] if row['phase'] in ('static','planning')]
        self.assertTrue(any(row['motor_received']<row['at'] for row in stationary))
        self.assertTrue(any(a['motor_received']==b['motor_received'] and
                            a['motor_ms']==b['motor_ms'] and a['at']!=b['at']
                            for a,b in zip(stationary,stationary[1:])))
        critical=[row for row in new['observations'] if row['completing'] or row['before_arm']]
        self.assertGreaterEqual(len(critical),2)
        self.assertTrue(all(row['motor_received']>=row['at'] for row in critical))
        self.assertEqual(new['target_changes'],0)
        self.costed_comparison={'old':old,'new':new}

    def test_idle_delay_skips_only_progressing_stopped_background_work(self):
        self.assertEqual(self.task.idle_delay_ms(),5)
        self.task.job={'purpose':'static'}
        self.task.target={'movement_state':'consistent'}
        self.assertEqual(self.task.idle_delay_ms(),0)
        self.task.job=None;self.task.planning={'iterator':iter(())}
        self.assertEqual(self.task.idle_delay_ms(),0)
        self.task.target['movement_state']='unknown'
        self.assertEqual(self.task.idle_delay_ms(),0)
        for attribute,value in (('fault','IO wait'),('arm',object()),('state',{'run':1}),
                                ('target',{'movement_state':'suspected'})):
            with self.subTest(attribute=attribute):
                previous=getattr(self.task,attribute)
                setattr(self.task,attribute,value)
                self.assertEqual(self.task.idle_delay_ms(),5)
                setattr(self.task,attribute,previous)
        self.task.planning=None;self.task.complete={'purpose':'static'}
        self.assertEqual(self.task.idle_delay_ms(),5)

    def test_stopped_compute_skips_slice_sleeps_but_keeps_both_fresh_checks(self):
        fixed=self._timed_prehover_run(250,500)
        active=self._timed_prehover_run(250,500,adaptive_idle=True)
        self.assertLess(active['first_goal_at'],fixed['first_goal_at'])
        for key in ('completing','before_arm'):
            fresh=[row for row in active['observations'] if row[key]]
            self.assertTrue(fresh,key)
            self.assertTrue(all(row['motor_received']>=row['at'] for row in fresh))
        self.assertEqual(active['target_changes'],0)

    def test_run_tracking_uses_idle_delay_without_zero_sleep_calls(self):
        task=Mock(status='running')
        calls=[]
        def step():
            calls.append('step')
            if len(calls)==3:task.status='succeeded'
        task.step.side_effect=step
        task.idle_delay_ms.side_effect=[0,5,5]
        task.snapshot.return_value={'status':'succeeded'}
        sleep=Mock()
        with patch('device.k230.robot.tracking.TrackingTask',return_value=task):
            self.assertEqual(run_tracking(sleep_ms=sleep),{'status':'succeeded'})
        self.assertEqual([call.args[0] for call in sleep.call_args_list],[5,5])
        task.close.assert_called_once()

    def test_adaptive_idle_still_observes_motion_and_unknown_before_first_hover(self):
        original=self.task.begin_arm
        hidden=[]
        track=self.task.track
        def observe(point,frame):
            if self.task.planning is not None and not hidden:
                self.hide_frames=1;hidden.append(frame['frame_id'])
            return track(point,frame)
        def plan(observation,version):
            for unused in range(200):
                self.clock.advance(2)
                yield {'stage':'candidate','target_version':version}
            yield {'stage':'complete','target_version':version,'frame_id':observation['frame_id'],
                   'execution':original(observation,version)}
        self.task.track=observe;self.task.begin_arm=plan
        self.move_when=lambda:self.task.planning is not None
        for unused in range(1800):
            self.task.step()
            self.clock.advance(self.task.idle_delay_ms())
            if self.task.status=='succeeded':break
        self.assertTrue(hidden)
        self.assertEqual(self.task.status,'succeeded',self.task.snapshot())
        self.assertGreaterEqual(self.task.metrics['target_changes'],1)
        self.assertFalse(any(event[0]=='goals' for event in self.arms[0].io.events))
        self.assertTrue(any(version>0 for stamp,observation,version in self.handoffs))

    def test_motion_and_moving_fine_keep_original_camera_and_status_cadence(self):
        self.x=650
        self.coarse_cost_ms=35
        self.motor.request_cost_ms=23
        result=self.finish()
        moving=[row for row in self.observation_rows if row['run'] or row['phase']=='moving']
        self.assertGreater(len(moving),10)
        self.assertTrue(all(row['motor_received']>=row['at'] for row in moving))
        gaps=[b['at']-a['at'] for a,b in zip(self.observation_rows,self.observation_rows[1:])
              if (a['run'] or a['phase']=='moving') and (b['run'] or b['phase']=='moving')]
        self.assertTrue(gaps)
        self.assertLessEqual(max(gaps),135)
        arm=[row for row in self.observation_rows if row['phase']=='arm']
        self.assertGreater(len(arm),3)
        self.assertLessEqual(max(b['at']-a['at'] for a,b in zip(arm,arm[1:])),110)
        self.assertTrue(any(row['motor_received']<row['at'] for row in arm))
        self.assertEqual(result['status'],'succeeded')

    def test_stationary_background_deadline_uses_250ms_instead_of_100ms(self):
        while self.task.job is None:self.tick()
        self.assertIsNotNone(self.task.job)
        self.task._last_capture=self.clock.now()-150
        before=self.task.job['session'].ticks
        self.task._advance_background()
        self.assertGreater(self.task.job['session'].ticks,before)

    def test_standalone_status_drift_invalidates_before_next_arm_goal(self):
        while not self.arms:self.tick()
        self.tick()
        arm=self.arms[0]
        arm.pending_stage='descend'
        before_goals=len([event for event in arm.io.events if event[0]=='goals'])
        self.task._last_capture=self.clock.now()
        self.task._last_motor_reply=self.clock.now()-500
        self.task._fresh_before_arm=False
        self.motor.a+=round(.2/self.config['mm_per_count'])
        frame=self.frame
        self.task.step()
        self.assertEqual(self.frame,frame)
        self.assertEqual(len([event for event in arm.io.events if event[0]=='goals']),before_goals)
        self.assertEqual(len([event for event in arm.io.events if event[0]=='ready2']),1)
        self.assertEqual(arm.status,'recovering')

    def test_initial_busy_far_motor_is_not_adopted_from_sync_or_first_status(self):
        for running_at_sync in (False,True):
            with self.subTest(running_at_sync=running_at_sync):
                self.setUp();self.x=650;self.motor.run=1
                if running_at_sync:
                    self.task=TrackingTask(self.capture,self.track,self.motor,self.projection,
                        self.config,self.clock.now,self.begin_arm,self.Fine,self.Fine,
                        emit=lambda event,details:self.events.append((event,details)))
                self.tick()
                self.assertEqual(self.task.decision['status'],'motion_wait')
                self.assertEqual(self.task.decision['motion_wait_reason'],'motor_was_running')
                self.assertFalse(any(command.startswith(('FOLLOW','SPEED','ARC'))
                    for at,command in self.motor.commands))
                self.assertIn('STOP',[command for at,command in self.motor.commands])
                self.assertEqual(self.motor.run,0)

    def test_standalone_status_motion_cancels_plan_before_advancing_it(self):
        while self.task.job is None:self.tick()
        if self.task.job is not None:self.task._cancel_job('fixture_replaced')
        progressed=[]
        def plan():
            progressed.append(True)
            yield {'stage':'candidate','target_version':self.task.memory.version}
        self.task.planning={'iterator':plan(),'target_version':self.task.memory.version,'frame_id':1}
        self.task._arm_motor_reference=dict(self.motor.sync())
        self.task._last_capture=self.clock.now()
        self.task._last_motor_reply=self.clock.now()-500
        self.motor.run=1
        frame=self.frame
        self.task.step()
        self.assertEqual(self.frame,frame)
        self.assertFalse(progressed)
        self.assertIsNone(self.task.planning)
        self.assertEqual(self.task.metrics['plans_cancelled'],1)
        self.assertEqual(self.motor.run,0)

    def test_status_excursion_between_camera_frames_invalidates_static_even_if_it_returns(self):
        self.fine_slices=80
        while self.task.job is None:self.tick()
        self.task._last_capture=self.clock.now()
        self.task._last_motor_reply=self.clock.now()-500
        self.motor.a+=round(.2/self.config['mm_per_count'])
        frame=self.frame
        frame_motor=dict(self.task._frame_motor)
        self.task.step()
        self.assertEqual(self.frame,frame)
        self.assertFalse(self.task.drive._static_ok)
        self.assertEqual(self.task._frame_motor,frame_motor)
        self.motor.a=0
        self.task.job['session'].ticks=79
        self.tick(2)
        self.assertFalse(self.handoffs)
        self.assertFalse(self.arms)
        self.assertIsNone(self.task.drive.parking)

    def test_static_and_planning_drift_share_the_original_photo_reference(self):
        self.fine_slices=80
        progressed=[]
        original=self.task.begin_arm
        def plan(observation,version):
            for index in range(40):
                progressed.append(index)
                self.clock.advance(5)
                yield {'stage':'candidate','target_version':version}
            yield {'stage':'complete','target_version':version,'frame_id':observation['frame_id'],
                   'execution':original(observation,version)}
        self.task.begin_arm=plan
        while self.task.job is None:self.tick()
        original_reference=dict(self.task.drive._fine_request['motor'])
        self.motor.a=round(.06/self.config['mm_per_count'])
        self.task.job['session'].ticks=79
        while self.task.planning is None:self.tick()
        self.assertEqual(self.task._arm_motor_reference,original_reference)
        before=len(progressed)
        self.task._last_capture=self.clock.now()
        self.task._last_motor_reply=self.clock.now()-500
        self.motor.a=round(.12/self.config['mm_per_count'])
        self.task.step()
        self.assertEqual(len(progressed),before)
        self.assertIsNone(self.task.planning)
        self.assertFalse(self.handoffs)
        self.assertEqual(self.task.metrics['plans_cancelled'],1)

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
