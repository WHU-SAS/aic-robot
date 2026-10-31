"""One moving fine image, continuous heading and bounded correction workflow."""
import json
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from device.k230.car import near_runtime
from device.k230.car.parking_window import bounds_mm
from host.car.runtime import load
from host.project_paths import ROOT
from tools.deploy.k230_car_align import build_bundle


class Clock:
    def __init__(self):
        self.ms, self.hooks, self.sleeps = 1000, [], []

    def now(self):
        return self.ms

    def advance(self, ms):
        for hook in self.hooks:
            hook(ms)
        self.ms += ms

    def sleep(self, ms):
        self.sleeps.append(ms)
        self.advance(ms)


class Link:
    def __init__(self, clock, scale):
        self.clock, self.scale = clock, scale
        clock.hooks.append(self.advance)
        self.left = self.right = self.left_speed = self.right_speed = 0
        self.run, self.reason = False, 0
        self.commands, self.timeline = [], []
        self.fail_follow = self.fail_correction = self.stop_on_speed = False
        self.nudge_next_status = False
        self.action_remaining = None
        self.action_reason = None

    def advance(self, ms):
        if self.run:
            active = ms if self.action_remaining is None else min(ms, self.action_remaining)
            self.left += self.left_speed*active/1000/self.scale
            self.right += self.right_speed*active/1000/self.scale
            if self.action_remaining is not None:
                self.action_remaining -= active
                if self.action_remaining <= 0:
                    self.run, self.reason = False, self.action_reason
                    self.left_speed = self.right_speed = 0

    def state(self):
        return {'ms': self.clock.ms+3000000,
                'encoder_a': round(self.left), 'encoder_b': -round(self.right),
                'run': int(self.run), 'reason': self.reason, 'result': 0,
                'pwm': 800 if self.run else 0,
                'left_pwm': 800 if self.run else 0,
                'right_pwm': 800 if self.run else 0}

    def sync(self):
        self.clock.advance(23)
        return self.state()

    def request(self, command):
        self.commands.append(command)
        self.timeline.append((self.clock.ms, command))
        self.clock.advance(23)
        name = command.split(',')[0]
        if name in ('FOLLOW', 'SPEED'):
            self.left_speed, self.right_speed = map(int, command.split(',')[1:])
            self.run = True
            if name == 'FOLLOW' and self.fail_follow:
                raise RuntimeError('FOLLOW ACK absent')
            if name == 'SPEED' and self.stop_on_speed:
                self.run, self.reason = False, 3
        elif name == 'STOP':
            self.run, self.reason = False, 1
            self.left_speed = self.right_speed = 0
        elif name == 'STATUS':
            if self.nudge_next_status:
                self.left += 10
                self.nudge_next_status = False
        elif name in ('MOVE', 'TURN'):
            value = int(command.split(',')[1])
            self.run = True
            if name == 'MOVE':
                self.left_speed = self.right_speed = 25*(1 if value > 0 else -1)
                self.action_remaining, self.action_reason = abs(value)/25*1000, 6
            else:
                self.left_speed, self.right_speed = (0,30) if value > 0 else (30,0)
                self.action_remaining = abs(value)*3.141592653589793/180*119.2/30*1000
                self.action_reason = 7
            if self.fail_correction:
                raise RuntimeError('correction ACK absent')
        elif name != 'HB':
            raise AssertionError(command)
        return self.state()

    def expect(self, command):
        return self.request(command)


class NearRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.workspace, _, self.projection = load()
        self.config, _, _, _ = build_bundle(near_parking=True)
        self.config['workspace'] = self.workspace
        self.config['near_parking'] = json.loads((ROOT/'configs/car/near_parking.json').read_text())
        self.config['near_parking']['capture_Base_X_mm'] = 500
        near, far = bounds_mm(self.workspace)['Base_X_mm']
        self.target = near+(far-near)/4
        self.clock = Clock()
        self.link = Link(self.clock, self.config['mm_per_count'])
        self.frames, self.captures, self.fines, self.track_frames = 0, [], [], []
        self.coarse_x, self.coarse_status = [800, 480], []
        self.moving_fine_status, self.moving_steps, self.moving_x = 'localized', 4, 330
        self.moving_range_only = self.static_range_only = False
        self.moving_height = 10
        self.encode_ms, self.stage_ms = 0, 25
        self.track_ms, self.post_fine_track_ms = 32, None
        self.fine_start_states = []
        self.static_steps, self.static_stage_ms = 3, 250
        self.static_offsets, self.static_y_offsets = [0], [0]
        self.move_during_static = False
        self.fine_error_index = self.fine_start_error_index = None
        self.evidence = []

    def point(self, x, offset=0, height=10):
        y = sum(bounds_mm(self.workspace, x)['Base_Y_mm'])/2+offset
        n = self.projection.normal
        return [x, y, (self.projection.offset+height-n[0]*x-n[1]*y)/n[2]]

    def precise(self, point, frame, status='localized'):
        return {'schema': 'aic.vision_observation.v1', 'mode': 'precise',
                'frame_id': frame['frame_id'], 'capture_timestamp_ms': frame['capture_timestamp_ms'],
                'timestamp_ms': frame['capture_timestamp_ms'], 'clock_domain': 'k230_ticks_ms',
                'image_size_px': [1280, 960], 'status': status,
                'cube_center_Base_mm': list(point), 'coordinate_frame': 'Base', 'units': 'mm',
                'reference_point': 'cube_geometric_center', 'xy_compensation_applied': False}

    def limited_range(self, observation, point):
        observation['status'] = 'no_complete_face'
        del observation['cube_center_Base_mm']
        observation['range_candidate'] = {
            'schema': 'aic.car.side_range.v1', 'status': 'range_candidate',
            'coordinate_frame': 'Base', 'units': 'mm',
            'raw_center_Base_mm': list(point), 'Base_X_mm': point[0],
            'X_interval_mm': [point[0]-.5, point[0]+.5],
            'not_arm_or_parking_authority': True}
        return observation

    def capture(self):
        self.frames += 1
        frame = {'frame_id': self.frames, 'capture_timestamp_ms': self.clock.ms,
                 'clock_domain': 'k230_ticks_ms', 'image_size_px': [1280, 960],
                 'timestamp_source': 'software_snapshot_call'}
        self.captures.append((self.clock.ms, self.link.run))
        self.clock.advance(5)
        return self.frames, frame

    def track(self, image, frame):
        self.track_frames.append(frame['frame_id'])
        index = image-1
        x = self.coarse_x[min(index, len(self.coarse_x)-1)]
        status = self.coarse_status[min(index,len(self.coarse_status)-1)] if self.coarse_status else 'tracking'
        delay = (self.post_fine_track_ms if self.post_fine_track_ms is not None and
                 self.fines and self.fines[0].latest is not None else self.track_ms)
        self.clock.advance(delay)
        return {'schema': 'aic.vision_observation.v1', 'mode': 'coarse',
                'frame_id': frame['frame_id'], 'status': status,
                'capture_timestamp_ms': frame['capture_timestamp_ms'],
                'timestamp_ms': frame['capture_timestamp_ms'], 'clock_domain': 'k230_ticks_ms',
                'image_size_px': [1280, 960], 'center_pixel': [640, 500],
                'bbox_px': [590, 450, 100, 100],
                'ground_contact_proxy_Base_mm': self.point(x, height=0)}

    def fine_type(self):
        owner = self
        class Fine:
            def __init__(self, config, clock_ms, purpose='precise'):
                self.index = len(owner.fines)
                self.purpose = purpose
                owner.fines.append(self)
                self.phase, self.latest, self.starts = None, None, 0
                self.events = []
                self.jpeg = None
            def start(self, image, frame, bbox):
                self.starts += 1
                owner.fine_start_states.append({'commands': list(owner.link.commands),
                    'frame_id': frame['frame_id'], 'bbox': bbox,
                    'left_mm_s': owner.link.left_speed, 'right_mm_s': owner.link.right_speed})
                self.frame, self.phase = dict(frame), 'geometry'
                self.jpeg = bytearray(b'\xff\xd8original-frame-'+str(frame['frame_id']).encode()+b'\xff\xd9')
                if self.index == owner.fine_start_error_index:
                    raise RuntimeError('start interrupted after JPEG ownership')
                if self.index:
                    i = self.index-1
                    x = owner.target+owner.static_offsets[min(i,len(owner.static_offsets)-1)]
                    y = owner.static_y_offsets[min(i,len(owner.static_y_offsets)-1)]
                    self.remaining, self.point = owner.static_steps, owner.point(x,y)
                else:
                    self.remaining, self.point = owner.moving_steps, owner.point(
                        owner.moving_x, height=owner.moving_height)
                owner.clock.advance(owner.encode_ms)
                self.events.append({'stage': 'start', 'capture_timestamp_ms': frame['capture_timestamp_ms']})
            def take_evidence(self, label):
                if self.jpeg is None:
                    return None
                item = {'label':label, 'jpeg':self.jpeg, 'frame':self.frame,
                        'observation':self.latest or {'status':'processing_interrupted','phase':self.phase}}
                self.jpeg = None
                return item
            def advance(self):
                if self.index == owner.fine_error_index:
                    raise RuntimeError('fine processing interrupted')
                owner.clock.advance(owner.static_stage_ms if self.index else owner.stage_ms)
                self.remaining -= 1
                if self.index and owner.move_during_static:
                    owner.link.nudge_next_status = True
                    owner.move_during_static = False
                if self.remaining:
                    return
                self.phase = None
                self.latest = owner.precise(self.point, self.frame,
                    'localized' if self.index else owner.moving_fine_status)
                if owner.static_range_only if self.index else owner.moving_range_only:
                    self.latest = owner.limited_range(self.latest, self.point)
                self.latest_ready_ms = owner.clock.ms
                self.events.append({'stage': 'complete'})
        return Fine

    def run_task(self):
        return near_runtime.run_parking(self.capture, self.track, self.link, self.projection,
            self.config, self.clock.now, self.clock.sleep, fine_factory=self.fine_type(),
            evidence=self.evidence)

    def names(self):
        return [c.split(',')[0] for c in self.link.commands]

    def test_one_moving_image_then_continuous_coarse_heading_and_new_static_verdict(self):
        result = self.run_task()
        self.assertEqual(result['status'], 'parked')
        self.assertTrue(result['verification']['accepted'])
        self.assertEqual(self.names().count('FOLLOW'), 1)
        self.assertEqual(self.names().count('STOP'), 1)
        self.assertNotIn('MOVE', self.names())
        self.assertNotIn('TURN', self.names())
        self.assertEqual(len(result['fine_observations']), 1)
        self.assertEqual(self.fines[0].starts, 1)
        self.assertEqual(self.fine_start_states[-1]['bbox'], None)
        self.assertTrue(all(active for unused,active in self.captures[1:-1]))
        self.assertFalse(self.captures[-1][1])
        self.assertNotIn(self.frames, self.track_frames)  # static verification bypasses coarse
        stopped_ms = result['stop_states'][-1]['ms']-result['stop_states'][0]['ms']
        self.assertGreaterEqual(stopped_ms, 50)
        self.assertLess(stopped_ms, 200)
        self.assertGreater(result['verification_observation']['frame_id'],
                           result['fine_observations'][0]['frame_id'])
        self.assertGreater(self.fines[-1].latest_ready_ms-self.fines[-1].frame['capture_timestamp_ms'],500)
        self.assertFalse(result['automatic_arm_action'])

    def test_near_start_inside_window_uses_one_static_frame_without_moving(self):
        self.coarse_x = [500]
        result = self.run_task()
        self.assertEqual(result['status'], 'parked')
        self.assertFalse(result['motion_performed'])
        self.assertTrue(all(name == 'STATUS' for name in self.names()))
        self.assertTrue(all(not running for unused, running in self.captures))
        self.assertEqual(len(self.captures), 2)  # coarse, then strict static
        self.assertEqual(len(result['verifications']), 1)
        self.assertIs(result['verification_observation'],
                      result['initial_verification']['observation'])
        self.assertEqual([fine.starts for fine in self.fines], [0, 1])
        self.assertEqual([item['label'] for item in self.evidence], ['initial_verify'])
        trace = result['verification']['static_check']
        self.assertTrue(trace['stationary_since_capture'])
        samples = trace['status_samples']
        self.assertEqual(len(samples), self.static_steps)
        self.assertEqual(len(self.link.commands), 1+self.static_steps)
        self.assertEqual([item['received_at_k230_ms'] for item in samples],
                         [stamp+23 for stamp, unused in self.link.timeline[1:]])
        for sample in samples:
            self.assertEqual(sample['state']['encoder_a'], trace['reference_state']['encoder_a'])
            self.assertEqual(sample['state']['encoder_b'], trace['reference_state']['encoder_b'])
            self.assertFalse(sample['state']['run'])

    def test_near_start_solves_before_first_follow_and_uses_remaining_distance(self):
        self.coarse_x = [480]
        self.static_offsets = [20, 0]
        result = self.run_task()
        self.assertEqual(result['status'], 'parked')
        self.assertEqual(self.names().count('FOLLOW'), 1)
        first = next(command for command in self.link.commands if command.startswith('FOLLOW,'))
        self.assertEqual(first, 'FOLLOW,50,50')
        self.assertNotIn('FOLLOW', [command.split(',')[0]
                                   for command in self.fine_start_states[0]['commands']])
        first_plan = result['samples'][0]['plan']
        self.assertEqual(first_plan['mode'], 'encoder_guided')
        self.assertEqual(first_plan['heading_source'], 'fine_odometry')
        self.assertGreater(first_plan['source_capture_age_ms'], 500)
        initial = result['initial_verification']['observation']
        self.assertGreater(result['verification_observation']['frame_id'], initial['frame_id'])
        self.assertEqual([item['label'] for item in self.evidence],
                         ['initial_verify', 'stopped_verify'])
        self.assertEqual(self.fines[0].starts, 0)
        self.assertFalse(result['fine_observations'])
        self.assertNotIn('MOVE', self.names())
        self.assertNotIn('TURN', self.names())

    def test_near_start_can_use_cruise_when_strict_remaining_distance_allows_it(self):
        self.coarse_x = [480]
        self.static_offsets = [80, 0]
        result = self.run_task()
        self.assertEqual(result['status'], 'parked')
        first = next(command for command in self.link.commands if command.startswith('FOLLOW,'))
        self.assertEqual(first, 'FOLLOW,70,70')
        self.assertFalse(self.captures[1][1])

    def test_near_start_too_close_uses_one_reverse_correction_and_new_frame(self):
        self.coarse_x = [480]
        self.static_offsets = [-20, 0]
        result = self.run_task()
        self.assertEqual(result['status'], 'parked')
        self.assertNotIn('FOLLOW', self.names())
        self.assertEqual(self.names().count('MOVE'), 1)
        self.assertIn('MOVE,-20', self.link.commands)
        self.assertTrue(result['motion_performed'])
        self.assertEqual(len(result['verifications']), 2)
        self.assertEqual([item['label'] for item in self.evidence],
                         ['initial_verify', 'corrected_verify'])
        self.assertGreater(result['verification_observation']['frame_id'],
                           result['initial_verification']['observation']['frame_id'])

    def test_near_start_lateral_correction_is_still_limited_to_one_attempt(self):
        self.coarse_x = [480]
        self.static_y_offsets = [25, 25]
        result = self.run_task()
        self.assertEqual(result['status'], 'parking_failed')
        self.assertNotIn('FOLLOW', self.names())
        self.assertNotIn('MOVE', self.names())
        self.assertEqual(self.names().count('TURN'), 1)
        self.assertEqual(len(result['verifications']), 2)

    def test_near_start_range_only_static_result_never_starts_moving(self):
        self.coarse_x = [480]
        self.static_range_only = True
        result = self.run_task()
        self.assertEqual(result['status'], 'no_precise_target')
        self.assertFalse(result['verification']['accepted'])
        self.assertFalse(result['motion_performed'])
        self.assertTrue(all(name == 'STATUS' for name in self.names()))
        self.assertEqual(len(self.evidence), 1)

    def test_near_start_movement_during_static_solve_prevents_start(self):
        self.coarse_x = [480]
        self.move_during_static = True
        result = self.run_task()
        self.assertEqual(result['status'], 'no_precise_target')
        self.assertFalse(result['verification']['accepted'])
        self.assertTrue(all(name == 'STATUS' for name in self.names()))

    def test_near_start_forward_motion_requires_new_verdict_and_allows_one_correction(self):
        self.coarse_x = [480]
        self.static_offsets = [20, 20, 0]
        result = self.run_task()
        self.assertEqual(result['status'], 'parked')
        self.assertEqual(self.names().count('FOLLOW'), 1)
        self.assertEqual(self.names().count('MOVE'), 1)
        self.assertEqual(len(result['verifications']), 2)
        initial_id = result['initial_verification']['observation']['frame_id']
        self.assertTrue(all(item['observation']['frame_id'] > initial_id
                            for item in result['verifications']))
        self.assertEqual([item['label'] for item in self.evidence],
                         ['initial_verify', 'stopped_verify', 'corrected_verify'])

    def test_t1_is_immediate_snapshot_t2_records_actual_encoder_handoff(self):
        result = self.run_task()
        handoff = result['encoder_handoff']
        capture_at = handoff['capture_timestamp_ms']
        first_after_capture = next((stamp,command) for stamp,command in self.link.timeline if stamp >= capture_at)
        self.assertEqual(first_after_capture, (capture_at+5, 'STATUS'))
        self.assertEqual(handoff['t1_state']['ms']-3000000, capture_at+28)
        self.assertGreaterEqual(handoff['t2_state']['ms']-3000000, self.fines[0].latest_ready_ms)
        dl = near_runtime.count_delta(handoff['t1_state']['encoder_a'],handoff['t2_state']['encoder_a'])*self.link.scale
        dr = -near_runtime.count_delta(handoff['t1_state']['encoder_b'],handoff['t2_state']['encoder_b'])*self.link.scale
        self.assertEqual(handoff['mean_mm'], (dl+dr)/2)
        self.assertGreater(dl,0)
        self.assertEqual(result['fine_observations'][0]['capture_timestamp_ms'], capture_at)

    def test_waiting_fine_never_selects_slow_stage_and_jpeg_counts_in_cycle(self):
        self.encode_ms = 251
        result = self.run_task()
        self.assertEqual(result['status'], 'parked')
        start = self.fine_start_states[0]
        self.assertEqual(start['commands'][-1].split(',')[0], 'SPEED')
        cruise = self.config['continuous']['cruise_mm_s']
        self.assertEqual((start['left_mm_s']+start['right_mm_s'])/2,cruise)
        samples = result['samples']
        self.assertGreaterEqual(samples[1]['cycle_ms'], 5+23+32+23+251)
        waiting = [s['plan'] for s in samples if s['plan']['mode'] == 'coarse_waiting_for_fine']
        self.assertTrue(waiting)
        self.assertTrue(all((p['left_mm_s']+p['right_mm_s'])/2 == cruise for p in waiting))

    def test_old_coarse_endpoint_never_stops_pending_fine_or_changes_fine_distance(self):
        self.coarse_x, self.moving_x = [800,320,310,300,290,280], 420
        result = self.run_task()
        self.assertEqual(result['status'], 'parked')
        fine = [s['plan'] for s in result['samples'] if s['plan']['mode'] == 'encoder_guided']
        self.assertTrue(fine)
        self.assertTrue(any(p['predicted_center_Base_mm'][0] > 350 for p in fine))
        self.assertEqual(result['last_plan']['target_Base_X_mm'], self.target)
        self.assertEqual(len(result['fine_observations']),1)

    def test_after_fine_lost_coarse_falls_back_to_encoder_heading_and_still_parks(self):
        self.coarse_status = ['tracking']*6+['lost']
        result = self.run_task()
        self.assertEqual(result['status'],'parked')
        modes = [s['plan']['heading_source'] for s in result['samples'] if s['plan']['mode']=='encoder_guided']
        self.assertIn('coarse',modes)
        self.assertIn('fine_odometry',modes)
        self.assertEqual(self.names().count('FOLLOW'),1)
        self.assertEqual(self.names().count('STOP'),1)

    def test_after_fine_expired_coarse_heading_does_not_expire_the_distance_anchor(self):
        self.post_fine_track_ms=501
        result=self.run_task()
        self.assertEqual(result['status'],'parked')
        plans=[s['plan'] for s in result['samples']]+[result['last_plan']]
        fallback=[p for p in plans if p.get('heading_source')=='fine_odometry']
        self.assertTrue(fallback)
        self.assertTrue(any(p['source_capture_age_ms']>500 for p in fallback))
        self.assertEqual(len(result['fine_observations']),1)

    def test_before_fine_lost_coarse_still_stops(self):
        self.coarse_status = ['tracking','tracking','lost']
        result = self.run_task()
        self.assertEqual(result['status'],'no_target')
        self.assertFalse(result['fine_observations'])
        self.assertFalse(self.link.run)
        self.assertNotIn('verification',result)

    def test_one_failed_fine_is_not_retried_or_replaced_with_coarse_acceptance(self):
        self.moving_fine_status = 'no_complete_face'
        result = self.run_task()
        self.assertEqual(result['status'],'precise_unavailable')
        self.assertEqual(len(result['fine_observations']),1)
        self.assertEqual(len(self.fines),1)
        self.assertEqual(self.fines[0].starts,1)
        self.assertNotIn('verification',result)
        self.assertFalse(self.link.run)

    def test_moving_range_anchor_reaches_a_new_strict_verdict_without_relabeling(self):
        self.moving_range_only, self.moving_height = True, 26
        result = self.run_task()
        self.assertEqual(result['status'], 'parked')
        anchor = result['fine_observations'][0]
        self.assertEqual(anchor['status'], 'no_complete_face')
        self.assertNotIn('cube_center_Base_mm', anchor)
        self.assertEqual(anchor['capture_timestamp_ms'],
                         result['encoder_handoff']['capture_timestamp_ms'])
        guided = [s['plan'] for s in result['samples'] if s['plan']['mode'] == 'encoder_guided']
        self.assertTrue(guided)
        self.assertTrue(all(p['range_only'] for p in guided))
        self.assertTrue(all(p['distance_source'] == 'physical_side_range_candidate' for p in guided))
        self.assertEqual(result['verification_observation']['status'], 'localized')
        self.assertGreater(result['verification_observation']['frame_id'], anchor['frame_id'])
        self.assertEqual(len(result['verifications']), 1)
        self.assertNotIn('MOVE', self.names())
        self.assertNotIn('TURN', self.names())

    def test_static_range_candidate_cannot_accept_window_or_trigger_correction(self):
        self.moving_range_only = self.static_range_only = True
        result = self.run_task()
        self.assertEqual(result['status'], 'no_precise_target')
        self.assertFalse(result['verification']['accepted'])
        self.assertEqual(result['verification_observation']['status'], 'no_complete_face')
        self.assertEqual(len(result['verifications']), 1)
        self.assertNotIn('MOVE', self.names())
        self.assertNotIn('TURN', self.names())
        self.assertFalse(self.link.run)

    def test_default_factory_limits_range_purpose_to_moving_capture(self):
        module = SimpleNamespace(FineSession=self.fine_type())
        with patch.dict('sys.modules', {'physical_side_session': module}):
            result = near_runtime.run_parking(self.capture, self.track, self.link, self.projection,
                self.config, self.clock.now, self.clock.sleep, evidence=self.evidence)
        self.assertEqual(result['status'], 'parked')
        self.assertEqual([fine.purpose for fine in self.fines],
                         ['car_moving_range', 'precise'])

    def test_expensive_fine_step_does_not_refresh_pending_coarse_age(self):
        self.stage_ms = 501
        result = self.run_task()
        self.assertEqual(result['status'],'no_target')
        self.assertEqual(result['after']['status'],'tracking')
        self.assertGreater(self.clock.now()-result['after']['capture_timestamp_ms'],500)
        self.assertFalse(self.link.run)

    def test_depth_correction_is_one_signed_move_then_a_new_verification(self):
        for offset in (-20,20):
            with self.subTest(offset=offset):
                self.setUp()
                self.static_offsets=[offset,0]
                result=self.run_task()
                self.assertEqual(result['status'],'parked')
                self.assertIn('MOVE,%d' % offset,self.link.commands)
                self.assertEqual(self.names().count('MOVE'),1)
                self.assertNotIn('TURN',self.names())
                self.assertEqual(len(result['verifications']),2)
                self.assertEqual(self.fines[0].starts,1)
                self.assertTrue(all(not running for unused,running in self.captures[-2:]))

    def test_both_axes_outside_choose_only_one_turn_and_final_failure_is_explicit(self):
        self.static_offsets=[20,20]
        self.static_y_offsets=[25,25]
        result=self.run_task()
        self.assertEqual(result['status'],'parking_failed')
        self.assertEqual(self.names().count('TURN'),1)
        self.assertNotIn('MOVE',self.names())
        self.assertEqual(len(result['verifications']),2)
        self.assertFalse(result['verification']['accepted'])
        self.assertEqual(self.names().count('FOLLOW'),1)

    def test_inside_but_not_centred_is_accepted_without_microadjustment(self):
        self.static_y_offsets=[16]
        result=self.run_task()
        self.assertEqual(result['status'],'parked')
        self.assertFalse(result['verification']['preferred'])
        self.assertNotIn('MOVE',self.names())
        self.assertNotIn('TURN',self.names())
        self.assertEqual(len(result['verifications']),1)

    def test_encoder_movement_during_static_capture_invalidates_verdict(self):
        self.move_during_static=True
        result=self.run_task()
        self.assertEqual(result['status'],'no_precise_target')
        self.assertFalse(result['verification']['accepted'])
        self.assertNotIn('MOVE',self.names())
        trace = result['verification']['static_check']
        self.assertFalse(trace['stationary_since_capture'])
        self.assertEqual(len(trace['status_samples']), self.static_steps)
        self.assertEqual(trace['status_samples'][0]['state']['encoder_a']-
                         trace['reference_state']['encoder_a'], 10)

    def test_static_trace_retains_movement_even_when_counts_return_to_reference(self):
        self.coarse_x = [500]
        expect, observed = self.link.expect, []
        def passive_motion(command):
            if command == 'STATUS' and len(self.fines) > 1 and self.fines[-1].starts:
                if len(observed) == 0:
                    self.link.left += 10
                elif len(observed) == 1:
                    self.link.left -= 10
                observed.append(command)
            return expect(command)
        with patch.object(self.link, 'expect', side_effect=passive_motion):
            result = self.run_task()
        self.assertEqual(result['status'], 'no_precise_target')
        trace = result['verification']['static_check']
        self.assertFalse(trace['stationary_since_capture'])
        reference = trace['reference_state']['encoder_a']
        self.assertEqual([sample['state']['encoder_a'] for sample in trace['status_samples']],
                         [reference+10, reference, reference])
        self.assertEqual(self.names(), ['STATUS']*(1+self.static_steps))
        self.assertFalse(result['motion_performed'])

    def test_follow_ack_failure_stops_before_first_saved_sample(self):
        self.link.fail_follow=True
        with self.assertRaisesRegex(RuntimeError,'FOLLOW ACK absent'):
            self.run_task()
        self.assertEqual(self.names()[:3],['STATUS','FOLLOW','STOP'])
        self.assertFalse(self.link.run)

    def test_correction_ack_failure_stops_without_retry(self):
        self.static_offsets=[20]
        self.link.fail_correction=True
        with self.assertRaisesRegex(RuntimeError,'correction ACK absent'):
            self.run_task()
        self.assertEqual(self.names().count('MOVE'),1)
        self.assertEqual(self.names().count('STOP'),2)
        self.assertFalse(self.link.run)

    def test_mcu_stop_does_not_restart_or_verify(self):
        self.link.stop_on_speed=True
        result=self.run_task()
        self.assertEqual(result['status'],'motion_stopped')
        self.assertEqual(result['motor_stop_reason'],3)
        self.assertNotIn('verification',result)
        self.assertEqual(self.names().count('FOLLOW'),1)
        self.assertFalse(self.link.run)

    def test_unset_capture_distance_never_syncs_captures_or_moves(self):
        self.config['near_parking']['capture_Base_X_mm']=None
        with patch.object(self.link,'sync',side_effect=AssertionError('sync forbidden')):
            result=self.run_task()
        self.assertEqual(result['status'],'capture_distance_unset')
        self.assertFalse(self.link.commands)
        self.assertFalse(self.captures)
        self.assertFalse(self.fines)

    def test_cycle_includes_capture_status_tracking_and_speed_uart(self):
        result=self.run_task()
        self.assertEqual(result['samples'][0]['cycle_ms'],5+23+32+23)
        self.assertEqual(self.clock.sleeps[0],100-(5+23+32+23))

    def test_evidence_is_two_owned_jpegs_and_never_enters_result_json(self):
        result=self.run_task()
        self.assertEqual(result['status'],'parked')
        self.assertEqual([x['label'] for x in self.evidence],['moving_fine','stopped_verify'])
        self.assertEqual(self.evidence[0]['frame']['frame_id'],result['fine_observations'][0]['frame_id'])
        self.assertEqual(self.evidence[1]['frame']['frame_id'],result['verification_observation']['frame_id'])
        self.assertTrue(all(x['jpeg'].startswith(b'\xff\xd8original-frame-') for x in self.evidence))
        self.assertTrue(all(fine.jpeg is None for fine in self.fines))
        self.assertNotIn('original-frame',json.dumps(result))
        with self.assertRaises(TypeError):
            json.dumps(self.evidence)

    def test_evidence_one_correction_has_exactly_three_distinct_original_frames(self):
        self.static_offsets=[20,0]
        result=self.run_task()
        self.assertEqual(result['status'],'parked')
        self.assertEqual([x['label'] for x in self.evidence],
                         ['moving_fine','stopped_verify','corrected_verify'])
        self.assertEqual(len({x['frame']['frame_id'] for x in self.evidence}),3)
        self.assertEqual(len({bytes(x['jpeg']) for x in self.evidence}),3)
        json.dumps(result)

    def test_failed_moving_fine_keeps_its_only_original_frame(self):
        self.moving_fine_status='no_complete_face'
        result=self.run_task()
        self.assertEqual(result['status'],'precise_unavailable')
        self.assertEqual(len(self.evidence),1)
        self.assertEqual(self.evidence[0]['label'],'moving_fine')
        self.assertEqual(self.evidence[0]['observation']['status'],'no_complete_face')
        self.assertEqual(self.evidence[0]['frame']['capture_timestamp_ms'],
                         result['encoder_handoff']['capture_timestamp_ms'])
        json.dumps(result)

    def test_processing_exception_preserves_captured_moving_or_static_frames(self):
        for index in (0,1):
            with self.subTest(session_index=index):
                self.setUp()
                self.fine_error_index=index
                with self.assertRaisesRegex(RuntimeError,'fine processing interrupted'):
                    self.run_task()
                self.assertFalse(self.link.run)
                self.assertEqual(len(self.evidence),index+1)
                self.assertEqual(self.evidence[-1]['observation']['status'],'processing_interrupted')
                self.assertTrue(self.evidence[-1]['jpeg'])


    def test_start_exception_after_jpeg_ownership_still_preserves_moving_or_static_image(self):
        for index in (0,1):
            with self.subTest(session_index=index):
                self.setUp()
                self.fine_start_error_index=index
                with self.assertRaisesRegex(RuntimeError,'start interrupted after JPEG ownership'):
                    self.run_task()
                self.assertFalse(self.link.run)
                self.assertEqual(len(self.evidence),index+1)
                self.assertTrue(self.evidence[-1]['jpeg'])
                self.assertEqual(self.evidence[-1]['observation']['status'],'processing_interrupted')
                self.assertEqual(self.evidence[-1]['frame']['frame_id'],self.fine_start_states[-1]['frame_id'])


class OdometryTests(unittest.TestCase):
    def state(self, a, b, ms=123456789):
        return {'encoder_a': a, 'encoder_b': b, 'ms': ms}

    def test_capture_interpolation_and_tail_extrapolation_use_k230_timestamps(self):
        odom = near_runtime.Odometry(.01)
        odom.add(self.state(100,-100,999999),1000,1020)
        odom.add(self.state(300,-500,10),1100,1120)
        self.assertEqual(odom.since(1060,1160),[(1.,2.),(1.,2.)])
        self.assertEqual(odom.since(1140,1160),[(.4,.8)])
        self.assertEqual(odom.rows[0][0],1010)
        self.assertEqual(odom.rows[1][0],1110)

    def test_encoder_wrap_preserves_forward_distance_and_ordered_increments(self):
        odom = near_runtime.Odometry(.5)
        odom.add(self.state(2147483640,-2147483640),1000,1000)
        odom.add(self.state(-2147483636,2147483636),1100,1100)
        odom.add(self.state(-2147483626,2147483616),1200,1200)
        self.assertEqual(odom.since(1050,1200),[(5.,5.),(5.,10.)])

    def test_k230_ticks_wrap_uses_ticks_diff_and_ticks_add(self):
        modulus = 1 << 30
        def difference(a,b):
            return ((a-b+modulus//2) % modulus)-modulus//2
        with patch.object(near_runtime,'age_ms',difference), \
                patch('time.ticks_add',side_effect=lambda a,b:(a+b)%modulus,create=True):
            odom = near_runtime.Odometry(.1)
            odom.add(self.state(0,0),modulus-20,0)
            odom.add(self.state(20,-20),80,100)
            self.assertEqual(odom.since(modulus-10,90),[(2.,2.)])


if __name__ == '__main__':
    unittest.main()
