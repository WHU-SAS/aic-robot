"""Persistent drive decisions with real geometry and synthetic motor replies."""
import math
import unittest

from device.k230.car.tracking_drive import DriveController
from device.k230.car.parking_window import bounds_mm
from device.k230.car.near_control import target_x
from host.car.runtime import load
from tools.deploy.k230_car_align import build_bundle
from host.tests.test_car_near_control import PlanarPlant


class TrackingDriveTests(unittest.TestCase):
    def setUp(self):
        self.workspace, unused, self.projection = load()
        self.config, unused, unused, unused = build_bundle(near_parking=True)
        self.drive = DriveController(self.projection, self.config)
        self.target = target_x(self.workspace)

    def point(self, x, offset=0):
        y = sum(bounds_mm(self.workspace, x)['Base_Y_mm'])/2+offset
        n = self.projection.normal
        z = (self.projection.offset+10-n[0]*x-n[1]*y)/n[2]
        return [x, y, z]

    def coarse(self, now, x=650, offset=0, frame=None):
        point = self.point(x, offset)
        n = self.projection.normal
        contact = [point[i]-10*n[i] for i in range(3)]
        return {'schema': 'aic.vision_observation.v1', 'mode': 'coarse',
            'image_size_px': [1280,960], 'frame_id': now if frame is None else frame,
            'capture_timestamp_ms': now, 'timestamp_ms': now, 'clock_domain': 'k230_ticks_ms',
            'status': 'tracking', 'center_pixel': self.projection.base_to_pixel(point),
            'bbox_px': [600,400,40,40], 'ground_contact_proxy_Base_mm': contact,
            'visual_evidence': {'timestamp_ms': now, 'visible': True, 'associated': True,
                                'ambiguous': False, 'distance_valid': True}}

    def missing(self, now):
        value = self.coarse(now)
        value.update(status='lost', visual_evidence={'timestamp_ms': now,
                    'visible': False, 'distance_valid': False})
        value.pop('ground_contact_proxy_Base_mm')
        value.pop('center_pixel')
        return value

    def state(self, now, run=False, travel=0, right=None, reason=0):
        scale = self.config['mm_per_count']
        return {'ms': now+3000000, 'encoder_a': round(travel/scale),
            'encoder_b': -round((travel if right is None else right)/scale),
            'run': int(run), 'reason': reason, 'result': 0,
            'pwm': 1000 if run else 0, 'left_pwm': 1000 if run else 0,
            'right_pwm': 1000 if run else 0}

    def precise(self, now, x, purpose='static', version=1, frame=None, status='localized'):
        pair = self.coarse(now, x=x, frame=frame)
        value = {'schema': 'aic.vision_observation.v1', 'mode': 'precise',
            'frame_id': pair['frame_id'], 'capture_timestamp_ms': now, 'timestamp_ms': now,
            'clock_domain': 'k230_ticks_ms', 'image_size_px': [1280,960], 'status': status,
            'coordinate_frame': 'Base', 'units': 'mm',
            'reference_point': 'cube_geometric_center', 'xy_compensation_applied': False,
            'cube_center_Base_mm': self.point(x)}
        return {'observation': value, 'target_version': version, 'purpose': purpose,
                'paired_coarse': pair, 'result_ready_ms': now+50,
                'request_id': (self.drive._fine_request or {}).get('request_id')}

    def update(self, now, x=650, run=False, travel=0, right=None, reason=0, **kw):
        coarse = kw.pop('coarse', self.coarse(now,x))
        return self.drive.update(coarse, self.state(now,run,travel,right,reason), now,
                                 target_version=kw.pop('target_version',1), **kw)

    def request_static(self, now=1000, x=None):
        x = self.target if x is None else x
        self.update(now,x)
        return self.update(now+50,x)

    def test_far_coarse_uses_distance_and_direction_without_fine(self):
        result = self.update(1000)
        self.assertTrue(result['command'].startswith('FOLLOW,'))
        self.assertIsNone(result['request_fine'])
        self.assertGreater(result['plan']['coarse_Base_X_mm'],470)
        self.assertFalse(result['allow_arm'])

    def test_near_start_stays_still_then_can_follow_accurate_remaining(self):
        result = self.request_static(x=330)
        self.assertEqual(result['request_fine'],'static')
        self.assertIsNone(result['command'])
        result = self.update(1150,330,precise=self.precise(1050,330))
        self.assertTrue(result['command'].startswith('FOLLOW,'))
        self.assertEqual(result['phase'],'following')
        self.assertFalse(result['allow_arm'])

    def test_stopped_recheck_with_forward_gap_resumes_the_continuous_curve(self):
        self.update(1000)
        self.update(1100,450,run=True)
        self.update(1200,443,run=True,travel=7,
                    precise=self.precise(1100,400,'moving'))
        result=self.update(1300,self.target+80,run=True,travel=160)
        self.assertEqual(result['command'],'STOP')
        self.update(1400,self.target+80,travel=160,reason=1)
        result=self.update(1450,self.target+80,travel=160,reason=1)
        self.assertEqual(result['request_fine'],'static')
        self.assertFalse(self.drive._fine_request['initial'])
        # Actual verification can reveal that the nominal wheel prediction
        # stopped early. The new real image, not that old prediction, owns it.
        result=self.update(1550,self.target+80,travel=160,reason=1,
            precise=self.precise(1450,self.target+80))
        self.assertEqual(result['command'],'FOLLOW,70,70')
        self.assertEqual(result['corrections'],1)
        self.assertFalse(result['allow_arm'])

    def test_forward_recheck_runs_to_one_final_photo_in_an_independent_world_plant(self):
        self.update(1000);self.update(1100,450,run=True)
        self.update(1200,443,run=True,travel=7,
                    precise=self.precise(1100,400,'moving'))
        self.update(1300,self.target+80,run=True,travel=160)
        self.update(1400,self.target+80,travel=160,reason=1)
        self.update(1450,self.target+80,travel=160,reason=1)
        complete=self.precise(1450,self.target+80)
        plant=PlanarPlant(self.point(self.target+80),self.projection.normal,
            self.config['track_width_mm'],self.projection.forward_axis_Base,
            self.projection.motion_origin_Base_mm)
        plant.vl=plant.vr=0
        left=right=160.;desired=(0,0);running=False;reason=1
        job=None;requests=[];stops=0;stopped_ms=0;cruise_samples=0
        for index in range(150):
            now=1550+index*100;point=plant.center()
            coarse=self.coarse(now,point[0])
            coarse['center_pixel']=self.projection.base_to_pixel(point)
            coarse['ground_contact_proxy_Base_mm']=[
                point[k]-10*self.projection.normal[k] for k in range(3)]
            if job is not None and now>=job['result_ready_ms']:
                complete=job;job=None
            result=self.drive.update(coarse,self.state(now,running,left,right,reason),now,
                precise=complete,target_version=1,fine_pending=job is not None)
            complete=None
            if result['allow_arm']:
                self.assertEqual(requests,['static'])
                self.assertEqual(stops,1)
                self.assertGreaterEqual(cruise_samples,3)
                self.assertTrue(result['parking']['verification']['accepted'])
                return
            command=result['command']
            if command:
                fields=command.split(',');name=fields[0]
                self.assertIn(name,('FOLLOW','SPEED','STOP'),result)
                if name in ('FOLLOW','SPEED'):
                    desired=tuple(map(int,fields[1:]));running=True;reason=0
                    if point[0]-self.target>36:
                        self.assertAlmostEqual(sum(desired)/2,70,delta=1)
                        cruise_samples+=1
                else:
                    stops+=1;running=False;reason=1;desired=(0,0);stopped_ms=0
            if result['request_fine']:
                self.assertFalse(running)
                purpose=result['request_fine'];requests.append(purpose)
                job=self.precise(now,point[0],purpose)
                job['observation']['cube_center_Base_mm']=list(point)
                job['paired_coarse']=coarse;job['request_id']=result['fine_request_id']
                job['result_ready_ms']=now+800
            for unused in range(10):
                if running:
                    dl,dr=plant.step(desired[0],desired[1],.01)
                else:
                    stopped_ms+=10;damping=math.exp(-.01/.12)
                    plant.vl*=damping;plant.vr*=damping
                    if stopped_ms>=500:plant.vl=plant.vr=0
                    dl,dr=plant.advance(.01)
                left+=dl;right+=dr
        self.fail({'decision':result,'point':plant.center(),'requests':requests})

    def test_lateral_only_recheck_does_not_add_a_tiny_forward_stop(self):
        for offset in (20,40):
            with self.subTest(offset=offset):
                self.setUp();self.update(1000);self.update(1100,450,run=True)
                self.update(1200,443,run=True,travel=7,
                    precise=self.precise(1100,400,'moving'))
                x=self.target+1
                self.update(1300,x,run=True,travel=160)
                self.update(1400,x,travel=160,reason=1)
                self.update(1450,x,travel=160,reason=1)
                complete=self.precise(1450,x)
                complete['observation']['cube_center_Base_mm']=self.point(x,offset)
                complete['paired_coarse']=self.coarse(1450,x,offset)
                result=self.update(1550,x,travel=160,reason=1,
                    coarse=self.coarse(1550,x,offset),precise=complete)
                self.assertEqual(result['phase'],'correcting')
                self.assertTrue(result['command'].startswith('ARC,'),result)
                left,right=map(int,result['command'].split(',')[1:])
                self.assertLess(left,0);self.assertGreater(right,0)
                self.assertEqual(result['corrections'],1)
                self.assertFalse(result['allow_arm'])

    def test_initial_lateral_only_offset_uses_one_arc_instead_of_a_tiny_forward_stop(self):
        for offset in (20,40):
            with self.subTest(offset=offset):
                self.setUp();x=self.target+1;self.request_static(x=x)
                complete=self.precise(1050,x)
                complete['observation']['cube_center_Base_mm']=self.point(x,offset)
                complete['paired_coarse']=self.coarse(1050,x,offset)
                result=self.update(1150,x,coarse=self.coarse(1150,x,offset),precise=complete)
                self.assertTrue(result['command'].startswith('ARC,'),result)
                self.assertEqual(result['corrections'],1)
                self.assertFalse(result['allow_arm'])

    def test_moving_fine_does_not_slow_cruise_and_corrects_heading_baseline(self):
        self.update(1000)
        result = self.update(1100,450,run=True)
        self.assertEqual(result['request_fine'],'moving')
        self.assertEqual(result['command'],'SPEED,70,70')
        result = self.update(1200,443,run=True,travel=7,
                             precise=self.precise(1100,400,'moving'))
        self.assertEqual(result['plan']['heading_source'],'fine_with_paired_coarse_change')
        self.assertEqual(result['command'],'SPEED,70,70')
        self.assertAlmostEqual(result['plan']['predicted_center_Base_mm'][0],393,delta=.1)

    def test_reliable_distance_survives_unknown_visibility_until_stop(self):
        self.update(1000)
        self.update(1100,450,run=True)
        self.update(1200,443,run=True,travel=7,precise=self.precise(1100,400,'moving'))
        result = self.update(1300,run=True,travel=14,coarse=self.missing(1300))
        self.assertEqual(result['command'],'SPEED,70,70')
        self.assertEqual(result['plan']['heading_source'],'fine_odometry')
        result = self.update(1400,run=True,travel=151,coarse=self.missing(1400))
        self.assertEqual(result['command'],'STOP')
        self.assertFalse(result['allow_arm'])

    def test_old_version_fine_result_cannot_reactivate_anchor(self):
        self.update(1000)
        self.update(1100,450,run=True)
        result = self.update(1200,650,run=True,travel=7,target_version=2,
            precise=self.precise(1100,250,'moving',version=1))
        self.assertIsNone(self.drive.anchor)
        self.assertEqual(result['command'],'SPEED,70,70')
        self.assertFalse(result['allow_arm'])

    def test_changed_target_inside_fine_region_stops_before_restarting(self):
        self.update(1000)
        self.update(1100,450,run=True)
        result=self.update(1200,300,run=True,travel=7,target_version=2,
            precise=self.precise(1100,400,'moving',version=1))
        self.assertEqual(result['command'],'STOP')
        self.assertEqual(result['status'],'target_changed_near')
        self.assertIsNone(self.drive.anchor)

    def test_same_version_wrong_frame_or_old_capture_never_binds(self):
        for mismatch in ('pair','time','request'):
            with self.subTest(mismatch=mismatch):
                self.setUp();self.update(1000);self.update(1100,450,run=True)
                precise=self.precise(1100,400,'moving')
                if mismatch=='pair':precise['paired_coarse']['frame_id']=999
                elif mismatch=='time':precise=self.precise(1000,400,'moving')
                else:precise['request_id']=(1,999)
                self.update(1200,443,run=True,travel=7,precise=precise)
                self.assertIsNone(self.drive.anchor)

    def test_paired_coarse_bias_does_not_replace_fine_heading(self):
        self.update(1000)
        pair=self.coarse(1100,450,offset=20)
        self.update(1100,450,run=True,coarse=pair)
        precise=self.precise(1100,400,'moving')
        precise['paired_coarse']=pair
        result=self.update(1200,443,run=True,travel=7,
            coarse=self.coarse(1200,443,offset=20),precise=precise)
        self.assertLess(abs(result['plan']['steering_bearing_deg']),.1)
        self.assertEqual(result['command'],'SPEED,70,70')

    def test_new_static_complete_frame_is_only_parking_authority(self):
        self.request_static()
        result = self.update(1150,self.target,precise=self.precise(1050,self.target))
        self.assertEqual(result['status'],'parked')
        self.assertTrue(result['allow_arm'])
        self.assertEqual(result['parking']['verification_observation']['frame_id'],1050)
        self.drive.restart_observation()
        self.assertIsNone(self.drive.parking)
        self.assertEqual(self.update(1200,self.target)['request_fine'],'static')

    def test_cached_parking_expires_before_repeated_handoff(self):
        self.request_static()
        self.update(1150,self.target,precise=self.precise(1050,self.target))
        result=self.update(1701,self.target)
        self.assertFalse(result['allow_arm'])
        self.assertIsNone(self.drive.parking)
        self.assertEqual(result['request_fine'],'static')

    def test_cached_parking_uses_cumulative_not_per_frame_wheel_drift(self):
        self.request_static()
        self.update(1150,self.target,precise=self.precise(1050,self.target))
        self.assertTrue(self.update(1200,self.target,travel=.06)['allow_arm'])
        result=self.update(1250,self.target,travel=.12)
        self.assertFalse(result['allow_arm'])
        self.assertIsNone(self.drive.parking)

    def test_static_small_recoil_keeps_original_stop_and_cache_reference(self):
        self.request_static()
        # The original rule permits <=0.1mm since the capture reference.
        result=self.update(1150,self.target,travel=.06,
                           precise=self.precise(1050,self.target))
        self.assertTrue(result['allow_arm'])
        timing=result['parking']['verification']['static_timing']
        self.assertEqual(timing['stopped_at_ms'],1000)
        # Do not reset that 0.1mm allowance at solve completion.
        result=self.update(1250,self.target,travel=.12)
        self.assertFalse(result['allow_arm'])
        self.assertIsNone(self.drive.parking)

    def test_cached_parking_does_not_return_after_visibility_gap_or_suspicion(self):
        for condition in ('unknown','ambiguous','suspected'):
            with self.subTest(condition=condition):
                self.setUp();self.request_static()
                complete=self.precise(1050,self.target)
                self.update(1150,self.target,precise=complete)
                coarse=self.coarse(1200,self.target)
                event={}
                if condition=='unknown':coarse=self.missing(1200)
                elif condition=='ambiguous':coarse['visual_evidence']['ambiguous']=True
                else:event={'movement_state':'suspected'}
                self.update(1200,self.target,coarse=coarse,target_event=event)
                result=self.update(1300,self.target,precise=complete)
                self.assertFalse(result['allow_arm'])
                self.assertFalse(result['precise_consumed'])
                self.assertIsNone(self.drive.parking)
                self.assertEqual(result['request_fine'],'static')

    def test_cached_parking_retracts_when_motor_runs_again(self):
        self.request_static()
        self.update(1150,self.target,precise=self.precise(1050,self.target))
        result=self.update(1200,self.target,run=True)
        self.assertFalse(result['allow_arm'])
        self.assertIsNone(self.drive.parking)
        self.assertEqual(result['command'],'STOP')

    def test_visible_but_unmeasurable_coarse_can_request_full_static_fine(self):
        for reason in ('inconsistent_with_20mm_height','implausible_aspect_ratio'):
            with self.subTest(reason=reason):
                self.setUp()
                for now in (1000,1050):
                    cue=self.missing(now)
                    cue['visual_evidence'].update(visible=True,associated=True,reason=reason)
                    result=self.update(now,coarse=cue)
                self.assertEqual(result['request_fine'],'static')
                self.assertIsNone(result['command'])
                complete=self.precise(1050,self.target)
                complete['paired_coarse']=cue
                cue=self.missing(1150)
                cue['visual_evidence'].update(visible=True,associated=True,reason=reason)
                result=self.update(1150,coarse=cue,precise=complete)
                self.assertTrue(result['precise_consumed'])
                self.assertTrue(result['allow_arm'])

    def test_clipped_side_uses_directional_search_without_fake_distance(self):
        for now in (1000,1050):
            cue=self.missing(now)
            cue['visual_evidence'].update(visible=True,associated=True,
                reason='clipped_blob',edge_directions=['right'])
            result=self.update(now,coarse=cue)
        left,right=map(int,result['command'].split(',')[1:])
        self.assertGreater(left,0);self.assertLess(right,0)
        self.assertIsNone(result['request_fine'])
        self.assertIsNone(self.drive.anchor)

    def test_invalidated_target_latch_blocks_new_fine_and_handoff_in_same_version(self):
        event={'visible':True,'associated':True,'target_valid':False,'event':'moved'}
        self.update(1000,self.target,target_event=event)
        result=self.update(1050,self.target,target_event=event)
        self.assertIsNone(result['request_fine'])
        result=self.update(1150,self.target,target_event=event,
                           precise=self.precise(1050,self.target))
        self.assertFalse(result['precise_consumed'])
        self.assertFalse(result['allow_arm'])
        self.assertIsNone(self.drive.anchor)
        self.assertIsNone(self.drive.parking)
        result=self.update(1250,self.target,target_event={'target_valid':True,'visible':True})
        self.assertEqual(result['request_fine'],'static')

    def test_failed_planning_can_request_a_real_view_change_before_new_static(self):
        self.request_static()
        self.update(1150,self.target,precise=self.precise(1050,self.target))
        self.drive.restart_observation(change_view=True)
        result=self.update(1200,self.target)
        self.assertTrue(result['command'].startswith('ARC,'))
        self.assertIsNone(result['request_fine'])
        self.assertFalse(result['allow_arm'])

    def test_brief_static_occlusion_does_not_discard_a_recovered_same_target(self):
        self.request_static()
        self.update(1100,self.target,coarse=self.missing(1100),fine_pending=True)
        result = self.update(1150,self.target,precise=self.precise(1050,self.target))
        self.assertTrue(result['allow_arm'])
        self.assertTrue(result['precise_consumed'])
        self.assertIsNone(result['request_fine'])

    def test_unknown_when_static_finishes_defers_without_restarting_or_grasping(self):
        self.request_static()
        complete=self.precise(1050,self.target)
        result=self.update(1150,self.target,coarse=self.missing(1150),precise=complete)
        self.assertTrue(result['precise_deferred'])
        self.assertFalse(result['precise_consumed'])
        self.assertFalse(result['allow_arm'])
        self.assertIsNone(result['request_fine'])
        self.assertIsNone(result['command'])
        result=self.update(1250,self.target,precise=complete)
        self.assertTrue(result['precise_consumed'])
        self.assertTrue(result['allow_arm'])

    def test_deferred_static_still_latches_real_chassis_motion(self):
        self.request_static()
        complete=self.precise(1050,self.target)
        self.update(1150,self.target,coarse=self.missing(1150),precise=complete)
        self.update(1200,self.target,travel=.2,coarse=self.missing(1200),fine_pending=True)
        result=self.update(1250,self.target,precise=complete)
        self.assertFalse(result['allow_arm'])
        self.assertIsNone(self.drive.parking)

    def test_deferred_static_does_not_survive_version_change_or_existing_freshness_limit(self):
        for changed in (False,True):
            with self.subTest(changed=changed):
                self.setUp();self.request_static()
                complete=self.precise(1050,self.target)
                self.update(1150,self.target,coarse=self.missing(1150),precise=complete)
                result=self.update(1701 if not changed else 1250,self.target,
                    target_version=2 if changed else 1,precise=complete)
                self.assertFalse(result['allow_arm'])
                self.assertFalse(result['precise_consumed'])
                self.assertFalse(result['precise_deferred'])
                self.assertIsNone(self.drive.parking)

    def test_unknown_coarse_before_fine_keeps_its_encoder_propagated_course(self):
        self.update(1000)
        result=self.update(1100,run=True,travel=7,coarse=self.missing(1100))
        self.assertEqual(result['command'],'SPEED,70,70')
        self.assertEqual(result['plan']['mode'],'coarse_prediction')
        self.assertLess(result['plan']['coarse_Base_X_mm'],650)
        self.assertFalse(result['allow_arm'])
        result=self.update(1200,636,run=True,travel=14)
        self.assertEqual(result['command'],'SPEED,70,70')
        self.assertEqual(result['plan']['mode'],'coarse')

    def test_visible_invalid_geometry_keeps_current_pixel_steering_and_prior_range(self):
        self.update(1000)
        cue=self.coarse(1100,643,offset=35)
        cue['visual_evidence'].update(distance_valid=False,
            reason='inconsistent_with_20mm_height',center_pixel=cue['center_pixel'])
        cue.pop('ground_contact_proxy_Base_mm')
        cue['status']='no_target'
        result=self.update(1100,run=True,travel=7,coarse=cue)
        self.assertTrue(result['command'].startswith('SPEED,'),result)
        self.assertEqual(result['plan']['heading_source'],'current_coarse_pixel')
        left,right=map(int,result['command'].split(',')[1:])
        self.assertNotEqual(left,right)
        self.assertAlmostEqual((left+right)/2,70,delta=4)
        self.assertFalse(result['allow_arm'])

    def test_coarse_only_occlusion_finishes_known_travel_then_waits_without_grasp(self):
        self.update(1000)
        result=self.update(1100,run=True,travel=650-self.target+5,coarse=self.missing(1100))
        self.assertEqual(result['command'],'STOP')
        for now in (1200,1250,1300):
            result=self.update(now,travel=650-self.target+5,coarse=self.missing(now))
            self.assertIsNone(result['command'])
            self.assertIsNone(result['request_fine'])
            self.assertFalse(result['allow_arm'])

    def test_confirmed_target_change_discards_the_old_coarse_course(self):
        self.update(1000)
        result=self.update(1100,run=True,travel=7,coarse=self.missing(1100),
                           target_version=2,target_event={'target_valid':False})
        self.assertEqual(result['command'],'STOP')
        self.assertIsNone(self.drive._coarse_reference)
        self.assertFalse(result['allow_arm'])

    def test_invalid_geometry_can_steer_a_fine_anchor_from_current_pixels(self):
        self.update(1000)
        self.update(1100,450,run=True)
        self.update(1200,443,run=True,travel=7,precise=self.precise(1100,400,'moving'))
        cue=self.coarse(1300,436,offset=25)
        cue['visual_evidence'].update(distance_valid=False,
            reason='inconsistent_with_20mm_height',center_pixel=cue['center_pixel'])
        cue.pop('ground_contact_proxy_Base_mm');cue['status']='no_target'
        result=self.update(1300,run=True,travel=14,coarse=cue)
        self.assertTrue(result['command'].startswith('SPEED,'))
        self.assertGreater(abs(result['plan']['steering_bearing_deg']),1)
        self.assertAlmostEqual(result['plan']['predicted_center_Base_mm'][0],386,delta=.1)
        self.assertFalse(result['allow_arm'])

    def test_failed_moving_fine_keeps_cruise_and_retries_on_a_later_live_frame(self):
        self.update(1000)
        self.update(1100,450,run=True)
        failed=self.precise(1100,400,'moving',status='no_complete_face')
        result=self.update(1200,443,run=True,travel=7,precise=failed)
        self.assertTrue(result['precise_consumed'])
        self.assertEqual(result['command'],'SPEED,70,70')
        self.assertIsNone(result['request_fine'])
        result=self.update(1300,436,run=True,travel=14)
        self.assertEqual(result['command'],'SPEED,70,70')
        self.assertEqual(result['request_fine'],'moving')
        request=self.drive._fine_request['request_id']
        result=self.update(1400,run=True,travel=21,coarse=self.missing(1400),fine_pending=True)
        self.assertEqual(result['command'],'SPEED,70,70')
        self.assertIsNone(result['request_fine'])
        self.assertEqual(self.drive._fine_request['request_id'],request)

    def test_static_chassis_drift_does_not_accept_precise_inside_window(self):
        self.request_static()
        result = self.update(1150,self.target,travel=.2,precise=self.precise(1050,self.target))
        self.assertFalse(result['allow_arm'])
        self.assertIsNone(self.drive.parking)

    def test_no_correction_count_limit_and_each_retry_requires_new_static(self):
        now = 1000
        x = self.target-10
        self.request_static(now,x)
        for index in range(5):
            frame = now+50 if index == 0 else now
            result = self.update(now+150,x,travel=-index,precise=self.precise(frame,x))
            self.assertTrue(result['command'].startswith('ARC,'),result)
            self.assertEqual(result['corrections'],index+1)
            self.assertFalse(result['allow_arm'])
            result = self.update(now+250,x,travel=-index-1,reason=8)
            self.assertIsNone(result['request_fine'])
            result = self.update(now+300,x,travel=-index-1,reason=8)
            self.assertEqual(result['request_fine'],'static')
            now += 300

    def test_failed_static_changes_real_view_and_alternates_actions(self):
        now=1000;x=self.target
        self.request_static(now,x)
        commands=[]
        for index in range(3):
            frame=now+50 if index == 0 else now
            result=self.update(now+150,x,travel=index,
                precise=self.precise(frame,x,status='no_complete_face'))
            commands.append(result['command'])
            self.update(now+250,x,travel=index+1,reason=8)
            self.update(now+300,x,travel=index+1,reason=8)
            now+=300
        self.assertTrue(commands[0].startswith('ARC,-'))
        self.assertTrue(commands[1].startswith('ARC,5,-') or commands[1].startswith('ARC,6,-'))
        self.assertEqual(commands[2],'ARC,-10,-10')

    def test_search_is_opposed_wheel_arc_and_stops_when_reacquired(self):
        missing=self.missing(1000)
        self.update(1000,coarse=missing,target_event={'event':'exited',
                    'last_disappearance_direction':'right'})
        result=self.update(1050,coarse=self.missing(1050))
        left,right=map(int,result['command'].split(',')[1:])
        self.assertGreater(left,0);self.assertLess(right,0)
        result=self.update(1150,run=True,coarse=self.coarse(1150))
        self.assertEqual(result['command'],'STOP')
        self.assertEqual(result['status'],'reacquired')

    def test_search_can_continue_without_new_terminal_limit(self):
        self.drive.request_search('left')
        self.update(1000,coarse=self.missing(1000))
        for index in range(8):
            result=self.update(1050+100*index,coarse=self.missing(1050+100*index),reason=8)
            self.assertTrue(result['command'].startswith('ARC,'))
            self.assertEqual(result['status'],'searching')

    def test_bottom_exit_reverses_once_then_rotates_instead_of_retreating_forever(self):
        self.update(1000,coarse=self.missing(1000),target_event={'event':'exited',
                    'last_disappearance_direction':'bottom'})
        first=self.update(1050,coarse=self.missing(1050))
        self.assertEqual(first['command'],'ARC,-10,-10')
        self.update(1150,travel=-10,reason=8,coarse=self.missing(1150))
        second=self.update(1200,travel=-10,reason=8,coarse=self.missing(1200))
        left,right=map(int,second['command'].split(',')[1:])
        self.assertLess(left,0);self.assertGreater(right,0)
        third=self.update(1300,travel=-10,reason=8,coarse=self.missing(1300))
        self.assertNotEqual(third['command'],'ARC,-10,-10')

    def test_top_exit_does_not_invent_a_table_distance_or_search_motion(self):
        self.update(1000)
        result=self.update(1100,run=True,coarse=self.missing(1100),
            target_event={'event':'exited','last_disappearance_direction':'top'})
        self.assertEqual(result['command'],'STOP')
        self.assertFalse(self.drive._searching)
        self.assertIsNone(self.update(1200,coarse=self.missing(1200))['command'])

    def test_search_stops_for_associated_airborne_cue_without_claiming_distance(self):
        self.update(1000,coarse=self.missing(1000),target_event={'search':True})
        self.update(1050,coarse=self.missing(1050))
        cue=self.missing(1150)
        cue['visual_evidence'].update(visible=True,associated=True,
                                    reason='inconsistent_with_20mm_height')
        result=self.update(1150,run=True,coarse=cue,
                           target_event={'visible':True,'associated':True})
        self.assertEqual(result['command'],'STOP')
        self.assertIsNone(self.drive.anchor)
        self.assertFalse(result['allow_arm'])
        self.assertIsNone(self.update(1250,coarse=cue)['command'])

    def test_arm_active_never_commands_nonzero_motion(self):
        self.update(1000)
        result=self.update(1100,run=True,arm_active=True)
        self.assertEqual(result['command'],'STOP')
        for now in (1200,1300):
            result=self.update(now,arm_active=True)
            self.assertIsNone(result['command'])
            self.assertFalse(result['allow_arm'])

    def test_existing_heartbeat_stop_waits_for_owner_not_blind_restart(self):
        self.update(1000)
        result=self.update(1100,reason=4)
        self.assertEqual(result['status'],'motion_wait')
        self.assertIsNone(result['command'])
        self.assertEqual(self.update(1200)['status'],'motion_wait')
        self.drive.resume_motion()
        self.assertTrue(self.update(1300)['command'].startswith('FOLLOW,'))

    def test_arc_abnormal_cutoff_never_counts_as_completed_correction(self):
        for reason in (0,1,2,4,6,7):
            with self.subTest(reason=reason):
                self.setUp();x=self.target-10;self.request_static(x=x)
                self.assertTrue(self.update(1150,x,precise=self.precise(1050,x))['command'].startswith('ARC,'))
                result=self.update(1250,x,reason=reason)
                self.assertEqual(result['status'],'motion_wait')
                self.assertIsNone(result['command'])
                self.assertIsNone(result['request_fine'])
                self.assertFalse(result['allow_arm'])
                self.assertEqual(self.update(1350,x)['status'],'motion_wait')

    def test_arc_deadline_requires_new_static_image_instead_of_latching_forever(self):
        x=self.target-10;self.request_static(x=x)
        old=self.precise(1050,x)
        launched=self.update(1150,x,precise=old)
        self.assertTrue(launched['command'].startswith('ARC,'))
        self.update(1250,x,run=True,travel=-.5,right=-.6)
        ended=self.update(6150,x,travel=-2,right=-3,reason=3)
        self.assertNotEqual(ended['status'],'motion_wait')
        self.assertIsNone(ended['command'])
        self.assertIsNone(ended['request_fine'])
        self.assertIsNone(self.drive.anchor)
        self.assertIsNone(self.drive._coarse_reference)
        self.assertEqual(ended['arc_timeouts'],1)
        self.assertEqual(ended['last_arc_end']['command'],launched['command'])
        self.assertEqual(ended['last_arc_end']['state']['reason'],3)
        next_frame=self.update(6200,x,travel=-2,right=-3,reason=3,precise=old)
        self.assertEqual(next_frame['request_fine'],'static')
        self.assertNotEqual(next_frame['fine_request_id'],old['request_id'])
        self.assertFalse(next_frame['precise_consumed'])
        refused=self.update(6300,x,travel=-2,right=-3,reason=3,
            precise=old,fine_pending=True)
        self.assertFalse(refused['allow_arm'])
        self.assertFalse(refused['precise_consumed'])
        self.assertIsNone(refused['command'])
        fresh=self.precise(6200,self.target)
        accepted=self.update(6400,self.target,travel=-2,right=-3,reason=3,precise=fresh)
        self.assertTrue(accepted['allow_arm'])
        self.assertEqual(accepted['parking']['verification_observation']['frame_id'],6200)

    def test_far_new_target_can_follow_after_an_arc_deadline(self):
        x=self.target-10;self.request_static(x=x)
        self.update(1150,x,precise=self.precise(1050,x))
        self.update(6150,x,travel=-2,right=-3,reason=3)
        result=self.update(6250,650,travel=-2,right=-3,reason=3,target_version=2)
        self.assertEqual(result['command'],'FOLLOW,70,70')
        self.assertEqual(result['arc_timeouts'],1)
        self.assertFalse(result['allow_arm'])
        self.assertIsNone(self.drive.anchor)

    def test_existing_deadline_is_segment_completion_not_task_failure(self):
        self.update(1000)
        result=self.update(21000,reason=3)
        self.assertTrue(result['command'].startswith('FOLLOW,'))
        self.assertEqual(result['status'],'running')

    def test_ambiguous_blue_blob_never_drives_toward_largest_candidate(self):
        coarse=self.coarse(1000)
        coarse['visual_evidence']['ambiguous']=True
        self.assertIsNone(self.update(1000,coarse=coarse)['command'])

    def test_indefinite_stopped_observation_does_not_accumulate_odometry_rows(self):
        for index in range(1000):
            now=1000+index*100
            self.update(now,coarse=self.missing(now))
        self.assertLessEqual(len(self.drive.odometry.rows),2)

    def test_motor_sampling_time_is_uart_midpoint_and_old_reply_cannot_settle(self):
        first=self.state(1000)
        first.update(_k230_sent_ms=1000,_k230_received_ms=1010)
        self.drive.update(self.coarse(5000),first,5000,target_version=1)
        self.assertEqual(self.drive.odometry.rows[0][0],1005)
        self.drive.update(self.coarse(5100),first,5100,target_version=1)
        self.assertIsNone(self.drive.stopped_at_ms)
        second=self.state(1050)
        second.update(_k230_sent_ms=1050,_k230_received_ms=1060)
        self.drive.update(self.coarse(5200),second,5200,target_version=1)
        self.assertEqual(self.drive.stopped_at_ms,1010)

    def test_initial_running_motor_does_not_get_adopted_as_our_follow(self):
        result=self.update(1000,run=True)
        self.assertEqual(result['status'],'motion_wait')
        self.assertEqual(result['command'],'STOP')
        self.assertIsNone(self.update(1100)['command'])

    def test_repeated_precise_payload_does_not_repeat_an_arc(self):
        x=self.target-10;self.request_static(x=x)
        precise=self.precise(1050,x)
        first=self.update(1150,x,precise=precise)
        self.assertTrue(first['command'].startswith('ARC,'))
        result=self.update(1250,x,run=True,precise=precise)
        self.assertEqual(result['command'],'HB')
        self.assertEqual(self.drive.corrections,1)

    def test_precise_consumed_only_marks_the_current_valid_request(self):
        self.request_static()
        wrong=self.precise(1050,self.target);wrong['request_id']=(1,999)
        self.assertFalse(self.update(1150,self.target,precise=wrong)['precise_consumed'])
        good=self.precise(1150,self.target)
        result=self.update(1250,self.target,precise=good)
        self.assertTrue(result['precise_consumed'])
        self.assertTrue(result['allow_arm'])

    def test_cancelled_job_cannot_use_a_new_request_for_the_same_fresh_frame(self):
        self.request_static()
        old=self.precise(1050,self.target)
        self.drive.cancel_fine()
        again=self.update(1100,self.target,coarse=self.coarse(1050,self.target))
        self.assertNotEqual(again['fine_request_id'],old['request_id'])
        result=self.update(1150,self.target,precise=old)
        self.assertFalse(result['precise_consumed'])
        self.assertFalse(result['allow_arm'])
        old.pop('request_id')
        result=self.update(1200,self.target,precise=old)
        self.assertFalse(result['precise_consumed'])
        self.assertFalse(result['allow_arm'])

    def test_far_moving_fine_and_coast_reach_parking_in_an_independent_world_plant(self):
        plant=PlanarPlant(self.point(650,10),self.projection.normal,
            self.config['track_width_mm'],self.projection.forward_axis_Base,
            self.projection.motion_origin_Base_mm)
        plant.vl=plant.vr=0
        left=right=0.;desired=(0,0);running=False;reason=0
        job=None;requests=[];moving_updates=0;arc=None;stopped_ms=0
        for index in range(400):
            now=1000+index*100
            point=plant.center()
            coarse=self.coarse(now,point[0])
            coarse['center_pixel']=self.projection.base_to_pixel(point)
            coarse['ground_contact_proxy_Base_mm']=[
                point[k]-10*self.projection.normal[k] for k in range(3)]
            complete=None
            if job is not None and now>=job['result_ready_ms']:
                complete=job;job=None
            result=self.drive.update(coarse,self.state(now,running,left,right,reason),now,
                precise=complete,target_version=1,fine_pending=job is not None)
            if result['allow_arm']:
                self.assertTrue(result['parking']['verification']['accepted'])
                self.assertEqual(requests,['moving','static'])
                self.assertGreaterEqual(moving_updates,6)
                self.assertLess(abs(plant.center()[0]-self.target),6)
                return
            command=result['command']
            if command:
                pieces=command.split(',');name=pieces[0]
                if name in ('FOLLOW','SPEED'):
                    desired=tuple(map(int,pieces[1:]));running=True;reason=0
                elif name=='STOP':
                    desired=(0,0);running=False;reason=1;stopped_ms=0
                elif name=='ARC':
                    # A perfect relative-wheel executor isolates the outer
                    # strategy. Its world transform is midpoint integrated,
                    # not computed by the controller's predict_center.
                    arc=list(map(float,pieces[1:]));running=True;reason=0
            if job is not None and job['purpose']=='moving':
                moving_updates+=1
                self.assertTrue(running)
                self.assertGreaterEqual(sum(desired)/2,60)
            if result['request_fine']:
                purpose=result['request_fine'];requests.append(purpose)
                # The snapshot is immutable while new world frames continue.
                job=self.precise(now,point[0],purpose)
                job['observation']['cube_center_Base_mm']=list(point)
                job['paired_coarse']=coarse
                job['request_id']=result['fine_request_id']
                job['result_ready_ms']=now+800
            for unused in range(10):
                if arc is not None:
                    longest=max(abs(v) for v in arc)
                    move=[math.copysign(min(abs(v),.15*abs(v)/longest),v)
                          if v else 0 for v in arc]
                    plant.vl,plant.vr=move[0]/.01,move[1]/.01
                    dl,dr=plant.advance(.01)
                    arc=[v-d for v,d in zip(arc,move)]
                    if max(abs(v) for v in arc)<1e-8:
                        arc=None;running=False;reason=8;plant.vl=plant.vr=0
                elif running:
                    dl,dr=plant.step(desired[0],desired[1],.01)
                else:
                    stopped_ms+=10
                    damping=math.exp(-.01/.12)
                    plant.vl*=damping;plant.vr*=damping
                    if stopped_ms>=500:plant.vl=plant.vr=0
                    dl,dr=plant.advance(.01)
                left+=dl;right+=dr
        self.fail({'decision':result,'point':plant.center(),'requests':requests})


if __name__ == '__main__':
    unittest.main()
