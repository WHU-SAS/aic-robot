"""Actual coarse/memory/drive chain with synthetic blobs, not field validation.

FakeImage supplies find_blobs results; LAB segmentation, camera/servo feedback
and physical driving are not simulated. Production calibration/config and the
real BlueTracker -> track_coarse -> TargetMemory -> DriveController are used.
The final test also exercises TrackingTask's missing -> search dispatch.
"""
import copy
import unittest

from device.k230.car.blue_tracker import BlueTracker
from device.k230.car.ground_projection import GroundProjection
from device.k230.car.target_memory import TargetMemory
from device.k230.car.tracking_drive import DriveController
from device.k230.car.near_control import target_x
from device.k230.car.parking_window import bounds_mm
from device.k230.arm.pick_execution import PickExecution
from device.k230.robot.tracking import TrackingTask
from device.k230.vision.runtime import track_coarse
from tools.deploy.k230_robot import build_bundle
from host.tests.test_robot_tracking import Clock, Motor, ArmIO, PLAN, synthetic_fine_match, is_search_spin


# The flat box comes from fa30's abnormal-motion log. Its being a supplied blue
# blob does not establish that it depicted the real cube in the field image.
FLAT = ([333,222,84,35],1470)
OTHER = ([900,222,80,60],2400)
FAR = ([620,222,54,55],1485)
NEAR = ([523,546,177,274],24000)
WEAK = ([333,222,84,35],100)


class Blob:
    def __init__(self, box, pixels):
        self.box, self.count = list(box), pixels

    def rect(self): return list(self.box)
    def pixels(self): return self.count
    def cx(self): return int(self.box[0]+self.box[2]/2)
    def cy(self): return int(self.box[1]+self.box[3]/2)


class BlobImage:
    def __init__(self, items): self.items = list(items)

    def find_blobs(self, thresholds, **options):
        return [Blob(box,pixels) for box,pixels in self.items
            if pixels >= options['pixels_threshold'] and
               box[2]*box[3] >= options['area_threshold']]


def frame(number, stamp):
    return dict(frame_id=number,capture_timestamp_ms=stamp,timestamp_ms=stamp,
        clock_domain='k230_ticks_ms',image_size_px=[1280,960],
        timestamp_source='synthetic_blob_regression')


def projection(config):
    p = config['projection']
    return GroundProjection(p['K'],p['D'],p['T'],p['plane'],
                            p['forward_axis_Base'],p['motion_origin_Base_mm'])


def synthetic_precise(coarse, model):
    # A same-frame API fixture only; no fake claim of physical-side CV success.
    return dict(coarse,mode='precise',status='localized',coordinate_frame='Base',
        units='mm',reference_point='cube_geometric_center',xy_compensation_applied=False,
        cube_center_Base_mm=model.pixel_to_plane(coarse['center_pixel'],10))


class Chain:
    def __init__(self, config):
        self.config = config
        self.projection = projection(config)
        self.tracker = BlueTracker(config['tracking'],self.projection)
        self.memory = TargetMemory(self.projection,config)
        self.drive = DriveController(self.projection,config)
        self.now = self.number = 0
        self.rows = []

    def sample(self, items, run=0, travel=0, right=None, reason=0, arm=None):
        self.now += 100
        self.number += 1
        coarse = track_coarse(BlobImage(items),frame(self.number,self.now),
                              self.tracker.detect_frame)
        scale = self.config['mm_per_count']
        motor = dict(ms=3000000+self.now,run=run,result=0,reason=reason,
            encoder_a=round(travel/scale),
            encoder_b=-round((travel if right is None else right)/scale),
            left_pwm=1000 if run else 0,right_pwm=1000 if run else 0)
        before = self.drive._motor
        deltas = () if before is None else [
            ((motor['encoder_a']-before['encoder_a'])*scale,
             -(motor['encoder_b']-before['encoder_b'])*scale)]
        target = self.memory.update(coarse,deltas,arm,motion_state=motor)
        decision = self.drive.update(coarse,motor,self.now,
            target_version=self.memory.version,target_event=target,arm_active=arm is not None)
        row = dict(coarse=coarse,target=target,decision=decision)
        self.rows.append(row)
        return row


class TrackingReacquisitionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = build_bundle(tracking=True)[0]['car']

    def setUp(self): self.chain = Chain(copy.deepcopy(self.config))

    def assert_no_forward(self, row):
        command=row['decision'].get('command') or ''
        self.assertTrue(not command.startswith(('FOLLOW,','SPEED,')) or is_search_spin(command))

    def test_ambiguous_legacy_history_cannot_authorize_one_unique_frame(self):
        smaller = ([900,222,40,35],700)
        for unused in range(3):
            row = self.chain.sample([FLAT,smaller])
            self.assertTrue(row['coarse']['visual_evidence']['ambiguous'])
            self.assert_no_forward(row)
        row = self.chain.sample([FLAT])
        evidence = row['coarse']['visual_evidence']
        self.assertEqual(evidence['confirmations'],1)
        self.assertFalse(evidence['navigation_confirmed'])
        self.assert_no_forward(row)
        self.assertIsNone(self.chain.drive._coarse_reference)
        self.assertIsNone(row['decision']['request_fine'])
        row = self.chain.sample([FLAT])
        self.assert_no_forward(row)
        row = self.chain.sample([FLAT])
        self.assertTrue(row['coarse']['visual_evidence']['navigation_confirmed'])
        self.assertEqual(row['decision']['command'],'FOLLOW,51,80')

    def test_initial_far_confirmation_stays_coarse_without_opening_a_fine_job(self):
        for unused in range(2):
            row = self.chain.sample([FAR])
            self.assert_no_forward(row)
            self.assertIsNone(row['decision']['request_fine'])
        row = self.chain.sample([FAR])
        self.assertTrue(row['decision']['command'].startswith('FOLLOW,'))
        self.assertIsNone(row['decision']['request_fine'])

    def test_visual_A_and_navigation_B_cannot_be_different_candidates(self):
        # Former bug: weak A established visual association, then legacy chose
        # larger B and accumulated tracking while visual still named left A.
        self.chain.sample([WEAK])
        for unused in range(3): row = self.chain.sample([FLAT,OTHER])
        coarse,evidence = row['coarse'],row['coarse']['visual_evidence']
        for key in ('bbox_px','center_pixel','ground_contact_proxy_Base_mm'):
            self.assertEqual(coarse[key],evidence[key])
        self.assertEqual(evidence['bbox_px'],FLAT[0])
        self.assertEqual(evidence['confirmations'],3)
        self.assertEqual(row['decision']['command'],'FOLLOW,51,80')
        self.assertLess(row['decision']['plan']['image_error_px'],0)

    def test_single_weak_or_unconfirmed_flash_cannot_create_a_new_route(self):
        for unused in range(4):
            self.chain.sample([FLAT,OTHER])
            row = self.chain.sample([FLAT])
            self.assert_no_forward(row)
            self.assertIsNone(self.chain.drive._coarse_reference)
            row = self.chain.sample([])
            self.assert_no_forward(row)
        self.assertEqual(self.chain.memory.version,0)

    def test_reliable_coarse_route_survives_unknown_without_being_refreshed_by_a_flash(self):
        for unused in range(3): row = self.chain.sample([FAR])
        self.assertTrue(row['decision']['command'].startswith('FOLLOW,'))
        original_stamp = self.chain.drive._coarse_reference['capture_timestamp_ms']
        for index in range(1,4):
            row = self.chain.sample([],run=1,travel=index*5)
            self.assertTrue(row['decision']['command'].startswith('SPEED,'))
            self.assertEqual(self.chain.drive._coarse_reference['capture_timestamp_ms'],original_stamp)
            self.assertIsNone(row['decision']['request_fine'])
        row = self.chain.sample([FAR],run=1,travel=20)
        self.assertFalse(row['coarse']['visual_evidence']['navigation_confirmed'])
        self.assertTrue(row['decision']['command'].startswith('SPEED,'))
        self.assertEqual(self.chain.drive._coarse_reference['capture_timestamp_ms'],original_stamp)
        self.assertEqual(self.chain.memory.version,0)

    def test_search_requires_real_confirmed_reacquisition_before_forward_motion(self):
        self.chain.drive.request_search('left')
        self.chain.sample([])
        row = self.chain.sample([])
        self.assertTrue(is_search_spin(row['decision']['command']))
        for unused in range(2):
            row = self.chain.sample([FAR],run=1)
            self.assert_no_forward(row)
            self.assertNotEqual(row['decision']['status'],'reacquired')
        row = self.chain.sample([FAR],run=1)
        self.assertTrue(row['coarse']['visual_evidence']['navigation_confirmed'])
        self.assertEqual(row['decision']['command'],'STOP')
        row = self.chain.sample([FAR],reason=1)
        self.assertTrue(row['decision']['command'].startswith('FOLLOW,'))

    def test_native_unassociated_partial_cue_cannot_restore_old_precise_authority(self):
        for unused in range(3): row = self.chain.sample([NEAR])
        coarse = row['coarse']
        self.assertTrue(self.chain.memory.bind_precise(
            synthetic_precise(coarse,self.chain.projection),coarse,0))
        # Shrinking from274px to20px breaks native association. It lies
        # within the old box, so it has no independent whole-box translation.
        row = self.chain.sample([([601,673,20,20],200)])
        self.assertFalse(row['coarse']['visual_evidence']['associated'])
        self.assertTrue(row['target']['visible'])
        self.assertFalse(row['target']['associated'])
        self.assertTrue(row['target']['needs_precise'])
        self.assertEqual(row['target']['version'],0)

    def test_close_contact_and_lift_do_not_require_navigation_confirmation(self):
        for unused in range(3): row = self.chain.sample([NEAR])
        coarse = row['coarse']
        self.chain.memory.bind_precise(synthetic_precise(coarse,self.chain.projection),coarse,0)
        close = {'phase':'close','grasp_confirmed':None}
        for box in ([568,552,179,261],[583,551,178,262]):
            row = self.chain.sample([(box,20000)],arm=close)
            self.assertEqual(row['target']['event'],'holding')
            self.assertEqual(row['target']['version'],0)
        lift = {'phase':'lift','grasp_confirmed':True,'held_center_pixel':[670,520]}
        for items in ([([760,510,20,20],200)],[],[([760,510,20,20],200)]):
            row = self.chain.sample(items,arm=lift)
            self.assertEqual(row['target']['version'],0)
            self.assertFalse(row['target']['confirmed_moved'])
        for unused in range(2):
            row = self.chain.sample([([20,510,20,20],200)],arm=lift)
        self.assertEqual(row['target']['version'],1)
        self.assertEqual(row['target']['reason'],'held_target_moved')

    def near_task_success(self, initial_missing=0):
        clock = Clock()
        config = copy.deepcopy(self.config)
        model = projection(config)
        tracker = BlueTracker(config['tracking'],model)
        motor = Motor(clock,config['mm_per_count'])
        x = target_x(config['workspace'])
        y = sum(bounds_mm(config['workspace'],x)['Base_Y_mm'])/2
        n = model.normal
        point = [x,y,(model.offset+10-n[0]*x-n[1]*y)/n[2]]
        number = [0]
        handoffs,arms = [],[]
        def capture():
            number[0] += 1
            image = BlobImage([] if number[0] <= initial_missing else [NEAR])
            image.point = list(point)
            return image,frame(number[0],clock.now())
        class Fine:
            def __init__(self,unused,now):
                self.clock = now
                self.phase = self.latest = self.latest_ready_ms = None
                self.slices = 0
            def start(self,image,stamp,bbox):
                self.point,self.frame = list(image.point),dict(stamp)
                self.phase = 'synthetic_geometry'
            def advance(self):
                clock.advance(20)
                self.slices += 1
                if self.slices == 5:
                    self.latest = dict(self.frame,schema='aic.vision_observation.v1',
                        mode='precise',status='localized',cube_center_Base_mm=self.point,
                        coordinate_frame='Base',units='mm',reference_point='cube_geometric_center',
                        xy_compensation_applied=False,quality={'reprojection_rms_px':.3},
                        payload={'status':'complete_face_candidate','cube_center_Base_mm':self.point,
                                 'poses':[{'face_reprojection_rms_px':.3}]})
                    self.latest_ready_ms = self.clock()
                    self.phase = None
            def take_evidence(self,label):
                return dict(label=label,jpeg=b'',frame=self.frame,observation=self.latest)
            def cancel(self): self.phase = None
        def begin_arm(observation,version):
            handoffs.append(copy.deepcopy(observation))
            arm = PickExecution(PLAN,{},ArmIO(clock),version,clock.now)
            arms.append(arm)
            return arm
        task = TrackingTask(capture,
            lambda image,stamp:track_coarse(image,stamp,tracker.detect_frame),
            motor,model,config,clock.now,begin_arm,Fine,Fine,reset_tracker=tracker.reset)
        task._match_fine = synthetic_fine_match
        for unused in range(1800):
            task.step()
            if task.status == 'succeeded': break
            clock.advance(5)
        result = task.snapshot()
        self.assertIsNone(task.fault)
        self.assertEqual(result['status'],'succeeded')
        self.assertEqual(result['metrics']['fine_started'],1)
        self.assertEqual(result['metrics']['arm_attempts'],1)
        self.assertEqual(len(arms),1)
        self.assertTrue(result['grasp_confirmed'])
        self.assertEqual(handoffs[0]['frame_id'],result['parking']['verification_observation']['frame_id'])
        return task,motor

    def test_real_tracker_near_start_finishes_static_handoff_and_grasp(self):
        task,motor = self.near_task_success()
        self.assertFalse(any(command.startswith(('FOLLOW,','SPEED,','ARC,'))
                             for stamp,command in motor.commands))

    def test_real_tracker_search_then_near_reacquisition_finishes_grasp(self):
        task,motor = self.near_task_success(initial_missing=3)
        self.assertTrue(any(is_search_spin(command) for stamp,command in motor.commands))
        self.assertFalse(any(command.startswith(('FOLLOW,','SPEED,')) and not is_search_spin(command)
                             for stamp,command in motor.commands))

    def run_task(self, scene, ticks=180):
        clock = Clock()
        config = copy.deepcopy(self.config)
        model = projection(config)
        tracker = BlueTracker(config['tracking'],model)
        motor = Motor(clock,config['mm_per_count'])
        number = [0]
        observed = []
        def capture():
            motor.advance()
            number[0] += 1
            items = scene(number[0])
            return BlobImage(items),frame(number[0],clock.now())
        def track(image,stamp):
            result = track_coarse(image,stamp,tracker.detect_frame)
            observed.append(result)
            return result
        class FailedFine:
            def __init__(self,unused,now):
                self.clock = now
                self.phase = self.latest = self.latest_ready_ms = None
            def start(self,image,stamp,bbox):
                self.frame = dict(stamp)
                self.phase = 'synthetic_failure'
            def advance(self):
                self.latest = dict(self.frame,schema='aic.vision_observation.v1',
                                   mode='precise',status='no_complete_face')
                self.latest_ready_ms = self.clock()
                self.phase = None
            def take_evidence(self,label):
                return dict(label=label,jpeg=b'',frame=self.frame,observation=self.latest)
            def cancel(self): self.phase = None
        def no_arm(*unused): self.fail('Synthetic failed fine cannot start the arm')
        task = TrackingTask(capture,track,motor,model,config,clock.now,no_arm,
                            FailedFine,FailedFine,reset_tracker=tracker.reset)
        for unused in range(ticks):
            task.step()
            clock.advance(50)
        return task,motor,observed

    def test_task_missing_dispatch_and_flashes_do_not_repeat_bottom_retreat(self):
        def scene(number):
            if number <= 3: return [FAR]
            if number == 4: return [([620,950,30,10],150)]
            if number <= 8: return []
            return [FLAT] if (number-9)%4 == 0 else []
        task,motor,observed = self.run_task(scene)
        self.assertIsNone(task.fault)
        commands = [command for stamp,command in motor.commands]
        backwards = [command for command in commands if command == 'ARC,-10,-10']
        # One distant cube cannot become a tiny bottom edge in a single frame.
        # That flash supplies no verified departure and earns no blind retreat.
        self.assertEqual(len(backwards),0,commands)
        self.assertTrue(any(is_search_spin(command) for command in commands))
        self.assertTrue(any(row['status'] == 'lost' for row in observed))
        self.assertGreater(task._missing,0)
        self.assertIsNone(task.fault)
        self.assertEqual(task.metrics['arm_attempts'],0)
        first_search = next(stamp for stamp,command in motor.commands if is_search_spin(command))
        self.assertFalse(any(command.startswith(('FOLLOW,','SPEED,')) and not is_search_spin(command)
            for stamp,command in motor.commands if stamp >= first_search))

    def test_task_three_missing_frames_dispatch_search_without_an_exit_event(self):
        def scene(number):
            return [FLAT] if number >= 5 and number%4 == 1 else []
        task,motor,observed = self.run_task(scene,ticks=100)
        self.assertIsNone(task.fault)
        commands = [command for stamp,command in motor.commands]
        self.assertTrue(any(is_search_spin(command) for command in commands))
        self.assertFalse(any(command.startswith(('FOLLOW,','SPEED,')) and not is_search_spin(command)
                             for command in commands))
        # Three initial blank captures trigger the coordinator's missing
        # policy before any blue cue can produce an exit/movement event.
        first_spin = next(stamp for stamp,command in motor.commands if is_search_spin(command))
        self.assertLessEqual(first_spin,300)
        self.assertIsNone(task.fault)
        self.assertEqual(task.metrics['arm_attempts'],0)
        self.assertTrue(all(not (item['visual_evidence'].get('edge_directions')) for item in observed))


if __name__ == '__main__': unittest.main()
