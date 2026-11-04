"""Target loss through real Task/Drive, with explicit protocol/image fixtures.

The historical-pool subtraction is SYNTHETIC. The retained no-target pools
were captured after the field fault in the unchanged scene, not during loss.
Neither is a surveyed vision or physical driving accuracy claim.
"""
import copy
import json
import unittest

from device.k230.robot.tracking import TrackingTask
from device.k230.car.blue_tracker import BlueTracker
from device.k230.vision.runtime import track_coarse
from host.project_paths import ROOT
from host.tests import test_car_tracking_reacquisition as native_fixtures
from host.tests import test_fine_observation_pause as fine_fixtures
from host.tests import test_robot_tracking as task_fixtures

FIELD=ROOT/'data/car/target_outside_no_search_20261104/read_01'


def fresh(packet, frame):
    result=copy.deepcopy(packet)
    result.update(frame)
    for cue in [result.get('visual_evidence') or {}]+list(result.get('visual_candidates') or []):
        cue['timestamp_ms']=frame['capture_timestamp_ms']
    return result


class RangePoolScene:
    def __init__(self, test, source, fine, pools):
        helper=fine_fixtures.FineObservationPauseTests()
        self.f,unused=helper.fixture('static',move_during_fine=False,overhead_fraction=0)
        f=self.f
        self.pools=pools
        self.test=test
        f.fine_slices=1000  # A visible weak cue may keep a static query pending.
        def track(image,frame):
            return fresh(self.pools[(frame['frame_id']-131)%len(self.pools)],frame)
        task=TrackingTask(f.capture,track,f.motor,f.projection,f.config,f.clock.now,
            f.begin_arm,f.Fine,f.Fine)
        f.task=task
        f.frame=130
        pair=fresh(source,dict(frame_id=130,capture_timestamp_ms=0,timestamp_ms=0,
            clock_domain='k230_ticks_ms'))
        precise=copy.deepcopy(fine)
        precise.update(frame_id=130,capture_timestamp_ms=0,timestamp_ms=0)
        task.memory.update(pair,motion_state=task.state)
        test.assertTrue(task.memory.bind_precise(precise,pair,0))
        task.drive.version=0
        task.drive._bind_anchor(precise,pair)
        task.drive._last_precise=(0,130,'moving')
        task.drive.phase='settling'  # The finite approach has already stopped.
        task.target=task.memory.snapshot()
        task.coarse=pair
        task._last_capture=0
        task._frame_motor=dict(task.state)
        task._memory_odom_capture=0

    def step(self):
        self.f.clock.advance(100)
        self.f.tick()
        self.test.assertIsNone(self.f.task.fault)

    def assert_search(self):
        f=self.f
        for unused in range(f.task.loss_confirmation_frames+5):
            self.step()
            if f.task.drive._searching and f.motor.run:
                break
        else:
            self.test.fail(dict(missing=f.task._missing,phase=f.task.drive.phase,
                status=(f.task.decision or {}).get('status'),commands=f.motor.commands))
        self.test.assertGreaterEqual(f.task._missing,f.task.loss_confirmation_frames)
        self.test.assertIsNone(f.task.drive.anchor)
        self.test.assertFalse(f.handoffs)
        self.test.assertFalse(f.arms)
        self.test.assertTrue(any(task_fixtures.is_search_spin(c) for at,c in f.motor.commands))
        self.test.assertEqual(f.task.metrics['fine_started'],0)


class DeferredArcMotor(task_fixtures.Motor):
    """The protocol completes ARC only when the test supplies its final ACK."""
    def expect(self, command):
        if command.startswith('ARC,'):
            self.advance()
            self.commands.append((self.clock.now(),command))
            self.arc=command
            self.run,self.reason=1,0
            self.left_speed=self.right_speed=0
            state=self.sync()
            self.replies.append((command,self.clock.now(),dict(state)))
            return state
        return super().expect(command)

    def finish_arc(self):
        left,right=map(int,self.arc.split(',')[1:])
        self.a+=round(left/self.scale)
        self.b-=round(right/self.scale)
        if self.on_arc:self.on_arc(left,right)
        self.run,self.reason=0,8


class TargetLossSearchTests(unittest.TestCase):
    @staticmethod
    def blank(frame):
        """A synthetic genuinely empty pool, not merely invalid blue geometry."""
        return dict(frame,schema='aic.vision_observation.v1',mode='coarse',
            status='no_target',visual_candidates=[],visual_evidence=dict(
                visible=False,associated=False,ambiguous=False,candidate_count=0,
                distance_valid=False,navigation_confirmed=False,edge_directions=[],
                reason='no_blue_evidence',timestamp_ms=frame['capture_timestamp_ms']))

    @staticmethod
    def forward(command):
        parts=command.split(',')
        return parts[0] in ('FOLLOW','SPEED') and sum(map(float,parts[1:3]))>0

    def next_observation(self,f):
        before=f.task.memory._last_frame
        f.clock.advance(100)
        for unused in range(1000):
            f.tick()
            self.assertIsNone(f.task.fault)
            # A fine job can capture now and only classify the frame after
            # its original heartbeat brackets it. Count processed frames.
            if f.task.memory._last_frame!=before:return
        self.fail('No new coarse observation')

    def running_scene(self,capability=None):
        helper=fine_fixtures.FineObservationPauseTests()
        f,unused=helper.fixture('static',move_during_fine=False,overhead_fraction=0)
        f.fine_slices=1000
        f.x=650 if capability is None else 360
        if capability is None:
            for unused in range(1000):
                f.tick()
                if f.task.drive._active_action=='follow' and f.motor.run:break
            else:self.fail('Coarse finite approach never started')
        else:
            image,frame=f.capture()
            source=f.task.track(image,frame)
            fine=f.precise(image,frame)
            if capability=='range':
                point=fine.pop('cube_center_Base_mm')
                fine.update(status='no_complete_face',range_candidate=dict(
                    schema='aic.car.side_range.v1',status='range_candidate',
                    coordinate_frame='Base',units='mm',Base_X_mm=point[0],
                    raw_center_Base_mm=point,not_arm_or_parking_authority=True))
            f.task.memory.update(source,motion_state=f.task.state)
            self.assertTrue(f.task.memory.bind_precise(fine,source,0))
            f.task.drive.version=0
            f.task.drive._bind_anchor(fine,source)
            f.task.drive._last_precise=(0,frame['frame_id'],'moving')
            f.task.drive.phase='following'
            f.task.target=f.task.memory.snapshot()
            f.task.coarse=source
            f.task._last_capture=frame['capture_timestamp_ms']
            f.task._frame_motor=dict(f.task.state)
            f.clock.advance(100)
            decision=f.task.drive.update(source,f.task.state,f.clock.now(),
                target_version=0,target_event=f.task.target)
            self.assertTrue(self.forward(decision['command']))
            self.assertEqual(len(decision['command'].split(',')),4)
            f.task._command(decision['command'])
            f.task.decision=decision
        return f

    def test_third_blank_revokes_coarse_or_precise_finite_travel_before_its_endpoint(self):
        for capability in(None,'full','range'):
            with self.subTest(capability=capability):
                f=self.running_scene(capability)
                start=len(f.motor.commands)
                original_cap=int(f.motor.commands[-1][1].split(',')[3])
                counts=(f.motor.a,f.motor.b)
                f.task.track=lambda image,frame:self.blank(frame)
                for count in(1,2):
                    self.next_observation(f)
                    self.assertEqual(f.task._missing,count)
                    self.assertTrue(f.motor.run)
                    self.assertEqual(f.task.drive._active_action,'follow')
                    self.assertFalse(f.task.drive._searching)
                    self.assertFalse(any(c=='STOP'for at,c in f.motor.commands[start:]))
                    if capability:self.assertIsNotNone(f.task.drive.anchor)
                continued=[c for at,c in f.motor.commands[start:] if self.forward(c)]
                # Keeping the existing finite segment alive with HB needs no
                # fresh SPEED or distance grant. Any issued update stays finite.
                remaining=[int(c.split(',')[3])for c in continued]
                self.assertTrue(all(b<=a for a,b in zip([original_cap]+remaining,remaining)))
                travel=((f.motor.a-counts[0])-(f.motor.b-counts[1]))*f.config['mm_per_count']/2
                self.assertGreater(travel,0)
                self.assertLess(travel,original_cap)
                self.next_observation(f)
                self.assertEqual(f.task._missing,3)
                self.assertFalse(f.motor.run)
                self.assertEqual(f.motor.commands[-1][1],'STOP')
                self.assertIsNone(f.task.drive.anchor)
                self.assertIsNone(f.task.drive._coarse_reference)
                self.assertFalse(f.task.memory.snapshot()['anchor_valid'])
                self.assertIsNone(f.task.job)
                self.assertIsNone(f.task.complete)
                self.assertIsNone(f.task.drive._fine_request)
                stopped=len(f.motor.commands)
                for unused in range(6):self.next_observation(f)
                self.assertTrue(any(task_fixtures.is_search_spin(c)
                    for at,c in f.motor.commands[stopped:]))
                self.assertFalse(any(self.forward(c)for at,c in f.motor.commands[stopped:]))
                self.assertFalse(f.handoffs)
                self.assertFalse(f.arms)

    def test_confirmed_blank_cancels_pending_moving_image_without_using_its_late_result(self):
        f=self.running_scene()
        f.config['tracking_task']['fine_observation_overhead_fraction']=.1
        # Request one real moving-job lifecycle at this already authorized
        # finite endpoint. Keeping the same scene avoids a synthetic target
        # teleport, and gives the three observed blanks time before the cap.
        decision={}
        f.task.drive._request('moving',f.task.coarse,f.task.state,f.clock.now(),decision,False)
        f.task.decision=decision
        f.task._start_job('moving',f.point(),dict(f.task.coarse))
        self.assertIsNotNone(f.task.job)
        self.assertEqual(f.task.job['purpose'],'moving')
        old=f.task.job['session']
        old_frame=dict(old.frame)
        f.task.track=lambda image,frame:self.blank(frame)
        for count in(1,2,3):
            self.next_observation(f)
            self.assertEqual(f.task._missing,count)
            if count<3:self.assertTrue(f.motor.run)
        self.assertEqual(f.task._missing,3)
        # The third light frame resolves at a cooperative yield. The next
        # original control slot must STOP before another capture or solve.
        captures=f.frame
        ticks=old.ticks
        f.task.step()
        self.assertEqual(f.frame,captures)
        self.assertEqual(old.ticks,ticks)
        self.assertFalse(f.motor.run)
        self.assertIsNone(f.task.job)
        self.assertIsNone(f.task.complete)
        self.assertIsNone(f.task.drive.anchor)
        self.assertIsNone(old.phase)
        # A detached solver's eventual result is historical. It cannot regain
        # ownership after the real task cancelled that image on confirmed loss.
        old.latest=f.precise(old.point,old_frame)
        old.latest_ready_ms=f.clock.now()
        start=len(f.motor.commands)
        for unused in range(6):self.next_observation(f)
        self.assertFalse(any(self.forward(c)for at,c in f.motor.commands[start:]))
        self.assertFalse(f.handoffs)
        self.assertFalse(f.arms)

    def test_edge_exit_does_not_skip_the_three_frame_forward_loss_confirmation(self):
        helper=fine_fixtures.FineObservationPauseTests()
        f,unused=helper.fixture('static',move_during_fine=False,overhead_fraction=0)
        # Pixel trajectory is supplied explicitly below; keep actual protocol
        # encoder increments without the fixture's optional image plant.
        f.motor.on_arc=None
        # The initial complete region is near the LEFT edge, not in the
        # image centre. The real native tracker then sees it become clipped.
        f.x,f.y=f.projection.pixel_to_plane([35,280],10)[:2]
        for unused in range(1000):
            f.tick()
            if f.task.drive._active_action=='follow' and f.motor.run:break
        else:self.fail('Edge-side coarse approach did not start')
        f.x,f.y=f.projection.pixel_to_plane([30,280],10)[:2]
        self.next_observation(f)
        evidence=f.task.coarse['visual_evidence']
        self.assertTrue(evidence['visible'])
        self.assertTrue(evidence['associated'])
        self.assertIn('left',evidence['edge_directions'])
        self.assertFalse(evidence['distance_valid'])
        self.assertEqual(f.task.memory.last_disappearance_direction,'left')
        self.assertTrue(f.motor.run)
        f.task.track=lambda image,frame:self.blank(frame)
        start=len(f.motor.commands)
        for count in(1,2):
            self.next_observation(f)
            self.assertEqual(f.task._missing,count)
            if count==1:
                self.assertEqual(f.task.target['event'],'exited')
                self.assertEqual(f.task.memory.last_disappearance_direction,'left')
            self.assertTrue(f.motor.run)
            self.assertFalse(f.task.drive._searching)
            self.assertFalse(any(c=='STOP'for at,c in f.motor.commands[start:]))
        self.next_observation(f)
        self.assertEqual(f.task._missing,3)
        self.assertFalse(f.motor.run)
        self.assertEqual(f.motor.commands[-1][1],'STOP')
        self.assertIsNone(f.task.drive._coarse_reference)
        for unused in range(5):self.next_observation(f)
        self.assertTrue(any(task_fixtures.is_search_spin(c)for at,c in f.motor.commands[start:]))
        self.assertFalse(f.handoffs)

    def test_fresh_task_on_white_wall_does_not_inherit_a_previous_successful_target(self):
        for previous_success in(False,True):
            with self.subTest(previous_success=previous_success):
                helper=fine_fixtures.FineObservationPauseTests()
                f,unused=helper.fixture('static',move_during_fine=False,overhead_fraction=0)
                f.fine_slices=2
                if previous_success:
                    for unused in range(3000):
                        f.tick()
                        if f.task.status=='succeeded':break
                    else:self.fail('First synthetic task did not succeed')
                    self.assertEqual(len(f.handoffs),1)
                    f.task.close()
                    self.assertFalse(f.motor.run)
                start=len(f.motor.commands)
                f.task=TrackingTask(f.capture,lambda image,frame:self.blank(frame),
                    f.motor,f.projection,f.config,f.clock.now,f.begin_arm,f.Fine,f.Fine)
                self.assertIsNone(f.task.coarse)
                self.assertIsNone(f.task.memory.last_precise)
                self.assertIsNone(f.task.drive.anchor)
                self.assertIsNone(f.task.drive._coarse_reference)
                for unused in range(9):self.next_observation(f)
                self.assertFalse(any(self.forward(c)for at,c in f.motor.commands[start:]))
                self.assertTrue(any(task_fixtures.is_search_spin(c)
                    for at,c in f.motor.commands[start:]))
                self.assertEqual(f.task.metrics['fine_started'],0)
                self.assertEqual(f.task.metrics['arm_attempts'],0)

    def synthetic_materials(self):
        fixture=task_fixtures.RobotTrackingTests()
        fixture.setUp()
        fixture.x=360
        frame=dict(frame_id=130,capture_timestamp_ms=0,timestamp_ms=0,
            clock_domain='k230_ticks_ms',image_size_px=[1280,960])
        point=fixture.point()
        source=fixture.track(point,frame)
        pixel=source['center_pixel']
        box=[pixel[0]-52,pixel[1]-77,105,155]
        source['bbox_px']=box
        source['visual_evidence'].update(bbox_px=box,pixels=10000)
        source['visual_candidates']=[dict(source['visual_evidence'])]
        fine=dict(frame,schema='aic.vision_observation.v1',mode='precise',
            status='no_complete_face',target_association={'status':'matched'},
            range_candidate=dict(schema='aic.car.side_range.v1',status='range_candidate',
                coordinate_frame='Base',units='mm',Base_X_mm=point[0],
                raw_center_Base_mm=point,not_arm_or_parking_authority=True))
        # Pure protocol examples, deliberately >=the500pixel image threshold.
        # A range-valid small patch and a same-scale but distant invalid patch
        # cannot keep a previously stopped cube's identity alive.
        valid=dict(visible=True,bbox_px=[1200,120,40,40],center_pixel=[1220.,140.],
            pixels=600,distance_valid=True,ground_contact_proxy_Base_mm=[1200.,0.,0.],
            measurement_reason=None,timestamp_ms=0)
        invalid=dict(visible=True,bbox_px=[292,66,50,68],center_pixel=[317.,100.],
            pixels=600,distance_valid=False,measurement_reason='inconsistent_with_20mm_height',
            timestamp_ms=0)
        background=self.background_pool(source)
        background['visual_candidates']=[valid,invalid]
        background['visual_evidence']['candidate_count']=2
        return source,fine,background

    def field_materials(self):
        required=[FIELD/'latest_run_summary.json',FIELD/'current_coarse_sequence.json']
        if any(not path.is_file() for path in required):
            self.skipTest('Retained native candidate pools are local-only')
        summary=json.loads(required[0].read_text())
        sequence=json.loads(required[1].read_text())
        return summary['target']['paired_coarse'],summary['target']['last_precise'],sequence

    def background_pool(self, source):
        result=copy.deepcopy(source)
        box=source['visual_evidence']['bbox_px']
        result['visual_candidates']=[v for v in result['visual_candidates'] if v['bbox_px']!=box]
        result['visual_evidence']=dict(visible=True,associated=False,ambiguous=True,
            candidate_count=len(result['visual_candidates']),distance_valid=False,
            navigation_confirmed=False,reason='ambiguous_blue_candidates',edge_directions=[])
        for key in ('bbox_px','center_pixel','ground_contact_proxy_Base_mm'):
            result.pop(key,None)
        return result

    def test_synthetic_historical_pool_without_the_cube_cannot_hide_stopped_range_loss(self):
        source,fine,unused=self.field_materials()
        pool=self.background_pool(source)
        self.assertEqual(len(source['visual_candidates']),27)
        self.assertEqual(len(pool['visual_candidates']),26)
        self.assertIn('range_candidate',fine)
        self.assertNotEqual(fine['status'],'localized')
        RangePoolScene(self,source,fine,[pool]).assert_search()

    def test_entire_same_scene_no_target_pools_search_despite_native_valid_tiny_blobs(self):
        source,fine,sequence=self.field_materials()
        pools=[row['coarse'] for row in sequence]
        self.assertEqual(len(pools),3)
        # Keep ALL candidates, including the real native range false positives.
        self.assertTrue(any(v.get('distance_valid') for p in pools for v in p['visual_candidates']))
        RangePoolScene(self,source,fine,pools).assert_search()

    def test_protocol_range_anchor_does_not_mistake_valid_or_invalid_background_for_the_cube(self):
        source,fine,background=self.synthetic_materials()
        RangePoolScene(self,source,fine,[background]).assert_search()

    def test_protocol_whole_target_without_distance_or_unique_selection_still_prevents_search(self):
        source,fine,unused=self.synthetic_materials()
        for ambiguous in(False,True):
            with self.subTest(ambiguous=ambiguous):
                visible=copy.deepcopy(source)
                visible['visual_evidence'].update(associated=False,ambiguous=ambiguous,
                    distance_valid=False,navigation_confirmed=False,cube_shape_compatible=False,
                    reason='inconsistent_with_20mm_height')
                visible['visual_candidates'][0].update(distance_valid=False,
                    cube_shape_compatible=False,measurement_reason='inconsistent_with_20mm_height')
                scene=RangePoolScene(self,source,fine,[visible])
                for unused in range(12):scene.step()
                self.assertEqual(scene.f.task._missing,0)
                self.assertFalse(scene.f.task.drive._searching)
                self.assertFalse(scene.f.handoffs)

    def test_native_threshold_chain_filters_small_patches_and_keeps_full_far_cubes(self):
        helper=fine_fixtures.FineObservationPauseTests()
        fixture,unused=helper.fixture('static',move_during_fine=False,overhead_fraction=0)
        config=fixture.config['tracking']
        self.assertEqual(config['min_pixels'],500)
        self.assertEqual(config['min_area_px2'],500)
        small=[([1229,127,12,15],80),([1196,124,31,25],242),([1222,124,39,32],348)]
        positives=[native_fixtures.WHOLE_LEFT,native_fixtures.WHOLE_RIGHT]
        class ThresholdImage(native_fixtures.BlobImage):
            def find_blobs(self,thresholds,**options):
                self.options=options
                return super().find_blobs(thresholds,**options)
        image=ThresholdImage(small+positives)
        tracker=BlueTracker(config,fixture.projection)
        coarse=track_coarse(image,native_fixtures.frame(1,100),tracker.detect_frame)
        self.assertEqual(image.options['pixels_threshold'],500)
        self.assertEqual(image.options['area_threshold'],500)
        self.assertFalse(image.options['merge'])
        self.assertEqual({tuple(v['bbox_px']) for v in coarse['visual_candidates']},
            {tuple(box) for box,pixels in positives})
        self.assertTrue(all(v['distance_valid'] for v in coarse['visual_candidates']))

    def test_search_reacquires_qualified_far_cubes_outside_the_previous_image_region(self):
        for initial_size,x in(((105,155),100),((105,155),1000),
                              ((177,274),100),((177,274),1000)):
            with self.subTest(x=x,initial_size=initial_size):
                source,fine,background=self.synthetic_materials()
                pixel=source['center_pixel']
                box0=[pixel[0]-initial_size[0]/2,pixel[1]-initial_size[1]/2]+list(initial_size)
                source['bbox_px']=box0
                source['visual_evidence'].update(bbox_px=box0,
                    pixels=24000 if initial_size==(177,274) else 10000)
                source['visual_candidates']=[dict(source['visual_evidence'])]
                scene=RangePoolScene(self,source,fine,[background])
                scene.assert_search()
                fixture=scene.f
                box=[x,222,54,55]
                tracker=BlueTracker(fixture.config['tracking'],fixture.projection)
                fixture.fine_slices=2
                capture=fixture.task.capture
                def current_image():
                    # If the controller first requests a new static query,
                    # its image API fixture describes THIS far cube, not the
                    # obsolete near point used to seed the previous range.
                    point=fixture.projection.pixel_to_plane([x+27,249],10)
                    fixture.x,fixture.y=point[:2]
                    return capture()
                def native(image,frame):
                    return track_coarse(native_fixtures.BlobImage([(box,1485)]),
                        frame,tracker.detect_frame)
                fixture.task.track=native
                fixture.task.capture=current_image
                fixture.task.reset_tracker=tracker.reset
                start=len(fixture.motor.commands)
                for unused in range(fixture.task.loss_confirmation_frames+6):
                    scene.step()
                    plan=(fixture.task.decision or {}).get('plan') or {}
                    if (not fixture.task.drive._searching and
                            fixture.task.drive._active_action=='follow' and plan.get('mode')=='coarse'):
                        break
                else:self.fail(dict(phase=fixture.task.drive.phase,missing=fixture.task._missing,
                    commands=fixture.motor.commands[start:]))
                self.assertTrue(fixture.task.coarse['visual_evidence']['distance_valid'])
                self.assertTrue(any(len(c.split(','))==4 and c.startswith(('FOLLOW,','SPEED,'))
                    for at,c in fixture.motor.commands[start:]))
                self.assertEqual(fixture.task.drive._search_region['bbox_px'],box)
                self.assertIsNone(fixture.task.drive.anchor)
                self.assertFalse(fixture.handoffs)
                resumed=len(fixture.motor.commands)
                for unused in range(2):
                    scene.step()
                    self.assertEqual(fixture.task._missing,0)
                    self.assertFalse(fixture.task.drive._searching)
                    self.assertTrue(fixture.motor.run)
                    self.assertEqual(fixture.task.drive._active_action,'follow')
                self.assertFalse(any(c=='STOP'for at,c in fixture.motor.commands[resumed:]))

    def test_whole_visible_ambiguous_or_unmeasurable_target_does_not_become_loss(self):
        source,fine,unused=self.field_materials()
        for ambiguous in(False,True):
            with self.subTest(ambiguous=ambiguous):
                packet=copy.deepcopy(source)
                packet['visual_evidence'].update(associated=False,ambiguous=ambiguous,
                    distance_valid=False,navigation_confirmed=False,cube_shape_compatible=False,
                    reason='inconsistent_with_20mm_height')
                box=source['visual_evidence']['bbox_px']
                for candidate in packet['visual_candidates']:
                    if candidate['bbox_px']==box:
                        candidate.update(distance_valid=False,navigation_confirmed=False,
                            cube_shape_compatible=False,measurement_reason='inconsistent_with_20mm_height')
                scene=RangePoolScene(self,source,fine,[packet])
                for unused in range(12):scene.step()
                self.assertEqual(scene.f.task._missing,0)
                self.assertFalse(scene.f.task.drive._searching)
                self.assertFalse(scene.f.handoffs)
                self.assertFalse(any(task_fixtures.is_search_spin(c) for at,c in scene.f.motor.commands))

    def test_true_blank_after_query_pose_cannot_wait_forever_for_identity(self):
        helper=fine_fixtures.FineObservationPauseTests()
        f,unused=helper.fixture('static',move_during_fine=False,overhead_fraction=0)
        f.geometry_invalid=True
        f.x=330
        for unused in range(3000):
            f.tick()
            if f.task.complete is not None:break
        else:self.fail('Static query never completed')
        f.tick()
        self.assertTrue(f.task.drive._awaiting_query_identity)
        self.assertTrue(f.task.memory.snapshot()['anchor_valid'])
        self.assertFalse(f.handoffs)
        f.hide_frames=500
        for unused in range(f.task.loss_confirmation_frames+5):
            f.clock.advance(100)
            f.tick()
            if f.task.drive._searching and f.motor.run:break
        else:self.fail(dict(missing=f.task._missing,phase=f.task.drive.phase,
            status=(f.task.decision or {}).get('status')))
        self.assertTrue(any(task_fixtures.is_search_spin(c) for at,c in f.motor.commands))
        self.assertFalse(f.handoffs)
        self.assertIsNone(f.task.drive.anchor)

    def test_owned_correction_completes_before_confirmed_loss_starts_search(self):
        helper=fine_fixtures.FineObservationPauseTests()
        f,unused=helper.fixture('static',move_during_fine=False,overhead_fraction=0)
        f.y+=40
        f.motor=DeferredArcMotor(f.clock,f.config['mm_per_count'])
        f.motor.on_arc=f.apply_arc
        f.task.link=f.motor
        for unused in range(3000):
            f.tick()
            if f.task.drive._active_action=='arc' and f.motor.run:break
        else:self.fail('Validated window correction never started')
        f.hide_frames=500
        start=len(f.motor.commands)
        for unused in range(f.task.loss_confirmation_frames+1):
            f.clock.advance(100)
            f.tick()
            self.assertEqual(f.task.drive._active_action,'arc')
            self.assertFalse(f.task.drive._searching)
            self.assertTrue(f.motor.run)
        self.assertFalse(any(c=='STOP' or task_fixtures.is_search_spin(c)
            for at,c in f.motor.commands[start:]))
        f.motor.finish_arc()
        for unused in range(f.task.loss_confirmation_frames+5):
            f.clock.advance(100)
            f.tick()
            if f.task.drive._searching and f.motor.run:break
        else:self.fail(dict(missing=f.task._missing,phase=f.task.drive.phase))
        self.assertTrue(any(task_fixtures.is_search_spin(c) for at,c in f.motor.commands[start:]))
        self.assertFalse(f.handoffs)


if __name__=='__main__':
    unittest.main()
