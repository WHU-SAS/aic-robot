"""Persistent drive decisions with real geometry and synthetic motor replies."""
import copy
import math
import unittest

from device.k230.car.tracking_drive import DriveController
from device.k230.car.parking_window import bounds_mm
from device.k230.car.near_control import target_x, predict_center
from device.k230.car.blue_tracker import BlueTracker
from device.k230.car.target_memory import TargetMemory
from device.k230.vision.runtime import track_coarse
from device.k230.car.observations import for_approach
from host.car.runtime import load
from tools.deploy.k230_car_align import build_bundle
from host.tests.test_car_near_control import PlanarPlant
from host.tests.test_car_visual_evidence import Blob, Image


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
                'ambiguous': False, 'distance_valid': True, 'navigation_confirmed':True,
                'confirmations':3, 'association_confirmations':3,
                'bbox_px':[600,400,40,40],
                'center_pixel':self.projection.base_to_pixel(point),
                'ground_contact_proxy_Base_mm':contact}}

    def missing(self, now):
        value = self.coarse(now)
        value.update(status='lost', visual_evidence={'timestamp_ms': now,
                    'visible': False, 'distance_valid': False})
        value.pop('ground_contact_proxy_Base_mm')
        value.pop('center_pixel')
        return value

    def weak_cue(self, now, confirmed=1):
        value=self.missing(now)
        value['visual_evidence'].update(visible=True,associated=True,ambiguous=False,
            distance_valid=False,navigation_confirmed=False,confirmations=0,
            association_confirmations=confirmed,reason='inconsistent_with_20mm_height',
            bbox_px=[333,223,84,34],center_pixel=[375.,240.])
        return value

    def native_cue(self, now, *blobs, tracker=None):
        tracker = tracker or BlueTracker(self.config['tracking'],self.projection)
        frame = {'frame_id':now,'capture_timestamp_ms':now,'timestamp_ms':now,
                 'clock_domain':'k230_ticks_ms','image_size_px':[1280,960]}
        return for_approach(track_coarse(Image(*blobs),frame,tracker.detect_frame),'coarse')

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

    def failed_static_after_late_moving_fine(self, bottom=False, moving_point=None):
        """Incident ordering with retained frame-21 geometry, synthetic replies.

        The successful moving solve finishes after the coarse travel has
        already stopped. This is not a replay of the released source images.
        """
        self.update(1000)
        requested = self.update(1100,450,run=True)
        self.assertEqual(requested['request_fine'],'moving')
        moving = self.precise(1100,413.1257,'moving')
        moving['observation']['cube_center_Base_mm'] = (list(moving_point) if moving_point is not None
            else [413.1257,13.76052,-59.41067])
        pair = moving['paired_coarse']
        for key, value in (('center_pixel',[649.,346.]),
                ('bbox_px',[604,277,109,142]),
                ('ground_contact_proxy_Base_mm',[457.5352,10.15517,-79.11276])):
            pair[key] = value
            pair['visual_evidence'][key] = value
        moving['result_ready_ms'] = 4730
        travel = 450-self.target+5
        stopped = self.update(2000,run=True,travel=travel,coarse=self.missing(2000),
                              fine_pending=True)
        self.assertEqual(stopped['command'],'STOP')
        self.update(2100,travel=travel,reason=1,coarse=self.missing(2100),fine_pending=True)
        self.update(2150,self.target,travel=travel,reason=1,fine_pending=True)
        self.assertEqual(self.drive._fine_request['request_id'],moving['request_id'])
        consumed = self.update(4836,self.target,travel=travel,reason=1,precise=moving)
        self.assertTrue(consumed['precise_consumed'])
        self.assertIsNotNone(self.drive.anchor)
        self.assertFalse(consumed['allow_arm'])
        self.assertEqual(self.update(4900,self.target,travel=travel,reason=1)['request_fine'],
                         'static')
        failed = self.precise(4900,self.target,status='no_complete_face')
        failed['result_ready_ms'] = 5800
        cue = self.clipped_near(5850) if bottom else self.coarse(5850,self.target)
        adjusted = self.update(5850,self.target,travel=travel,reason=1,precise=failed,coarse=cue)
        self.assertEqual(adjusted['command'],'FOLLOW,-15,-15' if bottom else 'ARC,-5,5')
        self.assertFalse(adjusted['allow_arm'])
        return travel, moving

    def clipped_near(self, now, edge='bottom'):
        cue = self.coarse(now,self.target)
        cue['visual_evidence'].update(distance_valid=False,navigation_confirmed=False,
            reason='clipped_blob',edge_directions=[edge],bbox_px=[457,656,208,304],
            center_pixel=[562.,809.])
        cue['visual_evidence'].pop('ground_contact_proxy_Base_mm')
        cue.pop('ground_contact_proxy_Base_mm')
        return cue

    def finish_near_view(self, now, left, right, coarse):
        self.update(now,run=True,travel=left,right=right,coarse=coarse(now))
        stopped = self.update(now+100,travel=left,right=right,reason=8,coarse=coarse(now+100))
        self.assertIsNone(stopped['request_fine'])
        return self.update(now+150,travel=left,right=right,reason=8,coarse=coarse(now+150))

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
            coarse['visual_evidence'].update(center_pixel=coarse['center_pixel'],
                ground_contact_proxy_Base_mm=coarse['ground_contact_proxy_Base_mm'])
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
        near,far=bounds_mm(self.workspace,self.target+1)['Base_Y_mm']
        for offset in ((far-near)/2+10,(far-near)/2+30):
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
        near,far=bounds_mm(self.workspace,self.target+1)['Base_Y_mm']
        for offset in ((far-near)/2+10,(far-near)/2+30):
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

    def test_changed_visible_target_inside_fine_region_remeasures_without_stopping(self):
        self.update(1000)
        self.update(1100,450,run=True)
        result=self.update(1200,300,run=True,travel=7,target_version=2,
            precise=self.precise(1100,400,'moving',version=1))
        self.assertEqual(result['command'],'SPEED,70,70')
        self.assertEqual(result['request_fine'],'moving')
        self.assertNotEqual(result['phase'],'settling')
        self.assertIsNone(self.drive.anchor)

    def test_first_real_native_large_scale_jump_revokes_full_and_range_without_stopping(self):
        for capability in ('full','range'):
            with self.subTest(capability=capability):
                self.setUp()
                self.config['tracking']['confirmation_frames']=1
                tracker=BlueTracker(self.config['tracking'],self.projection)
                memory=TargetMemory(self.projection,self.config)
                def cube_blob(x,y):
                    n=self.projection.normal
                    foot=[x,y,(self.projection.offset-n[0]*x-n[1]*y)/n[2]]
                    u,v=self.projection.base_to_pixel(foot)
                    top=self.projection.base_to_pixel([foot[k]+20*n[k] for k in range(3)])
                    size=round(abs(v-top[1]))
                    return Blob([round(u-size/2),round(v-size+1),size,size],size*size)
                old=self.native_cue(1000,cube_blob(1200,0),tracker=tracker)
                first=memory.update(old,motion_state=self.state(1000))
                self.drive.update(old,self.state(1000),1000,target_version=0,target_event=first)
                fine=self.precise(1000,1200,'moving',version=0)['observation']
                point=[old['visual_evidence']['ground_contact_proxy_Base_mm'][k]+
                       10*self.projection.normal[k] for k in range(3)]
                fine['cube_center_Base_mm']=point
                if capability=='range':
                    fine.pop('cube_center_Base_mm')
                    fine.update(status='no_complete_face',range_candidate={
                        'schema':'aic.car.side_range.v1','status':'range_candidate',
                        'coordinate_frame':'Base','units':'mm','Base_X_mm':point[0],
                        'raw_center_Base_mm':point,'not_arm_or_parking_authority':True})
                self.assertTrue(memory.bind_precise(fine,old,0))
                self.drive._bind_anchor(fine,old)
                fresh=self.native_cue(1100,cube_blob(300,50),tracker=tracker)
                untouched=copy.deepcopy(fresh)
                evidence=fresh['visual_evidence']
                self.assertFalse(evidence['associated'])
                self.assertEqual(evidence['association_confirmations'],0)
                self.assertTrue(evidence['distance_valid'])
                self.assertGreater(evidence['bbox_px'][2]/old['bbox_px'][2],
                                   self.config['tracking']['max_size_ratio'])
                target=memory.update(fresh,[(7,7)],motion_state=self.state(1100,True,7))
                self.assertTrue(target['tracking_update'])
                self.assertEqual(target['version'],1)
                self.assertFalse(target['anchor_valid'])
                self.assertIsNotNone(memory._query_visual)
                self.assertFalse(memory.bind_precise(fine,old,0))
                result=self.drive.update(fresh,self.state(1100,True,7),1100,
                    target_version=memory.version,target_event=target)
                self.assertTrue(result['command'].startswith('SPEED,'),result)
                self.assertEqual(result['request_fine'],'moving')
                self.assertNotEqual(result['phase'],'settling')
                self.assertFalse(self.drive._searching)
                self.assertIsNone(self.drive.anchor)
                self.assertEqual(fresh,untouched)

    def test_expired_visible_retarget_cue_cannot_start_bearing_only_motion(self):
        self.update(1000)
        coarse=self.weak_cue(1000,confirmed=1)
        raw=dict(coarse['visual_evidence'])
        result=self.update(1601,run=True,coarse=coarse,target_version=2,
            target_event=dict(event='tracking_update',tracking_update=True,
                navigation_cue=raw,visible=True,associated=True,
                visibility='visible',target_valid=True))
        self.assertEqual(result['command'],'STOP')
        self.assertIsNone(result['request_fine'])
        self.assertFalse(self.drive._bearing_only)

    def test_unsent_STOP_can_restore_only_explicit_previous_follow_ownership(self):
        self.update(1000)
        self.update(1100,450,run=True)
        stop=self.update(1200,443,run=True,travel=7,
            precise=self.precise(1100,self.target,'moving'))
        self.assertEqual(stop['command'],'STOP')
        self.assertIsNone(self.drive._active_action)
        state=dict(self.drive._motor)
        self.assertFalse(self.drive.retarget_following(2))
        self.assertTrue(self.drive.retarget_following(2,pending_follow=True))
        self.assertEqual(self.drive._active_action,'follow')
        self.assertEqual(self.drive._motor,state)
        self.assertIsNone(self.drive.anchor)
        self.assertEqual(self.drive.phase,'following')
        self.drive._motion_hold='existing_motor_stop_4'
        self.assertFalse(self.drive.retarget_following(3,pending_follow=True))

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

    def test_late_moving_success_static_failure_and_bottom_clip_stay_in_near_recovery(self):
        travel, moving = self.failed_static_after_late_moving_fine(bottom=True)
        original = dict(self.drive._near_recovery)
        # Several current clipped views retain ONE slow reverse FOLLOW.
        # Neither a 10mm boundary nor an elapsed observation restarts it.
        for now,distance in ((5950,5),(6250,10),(6550,20)):
            backed = self.update(now,run=True,travel=travel-distance,
                                 coarse=self.clipped_near(now),precise=moving)
            self.assertEqual(backed['command'],'HB')
            self.assertIsNone(backed['request_fine'])
            self.assertFalse(self.drive._searching)
            self.assertIsNone(self.drive.anchor)
            self.assertFalse(backed['allow_arm'])
            self.assertFalse(backed['precise_consumed'])
            self.assertEqual(self.drive.fine_requests,2)
            self.assertEqual(self.drive._active_action,'recovery_reverse')
        self.assertEqual(self.drive._near_recovery['capture_timestamp_ms'],1100)
        self.assertEqual(self.drive._near_recovery['point'],original['point'])
        complete_cue = lambda now:self.coarse(now,self.target)
        stopped = self.update(6650,run=True,travel=travel-30,coarse=complete_cue(6650))
        self.assertEqual(stopped['command'],'STOP')
        self.assertIsNone(stopped['request_fine'])
        self.update(6750,travel=travel-30,reason=1,coarse=complete_cue(6750))
        inspect = self.update(6800,travel=travel-30,reason=1,coarse=complete_cue(6800))
        self.assertEqual(inspect['request_fine'],'static')
        self.assertIsNone(inspect['command'])
        self.assertEqual(self.drive.fine_requests,3)
        # The old moving solve cannot satisfy this new stopped request, even
        # though its geometry still guides recovery and its version matches.
        old = self.update(6850,travel=travel-30,reason=1,
            coarse=complete_cue(6850),precise=moving,fine_pending=True)
        self.assertFalse(old['precise_consumed'])
        self.assertFalse(old['allow_arm'])
        self.assertIsNone(old['command'])
        fresh = self.precise(6800,self.target)
        accepted = self.update(6900,travel=travel-30,reason=1,
            coarse=complete_cue(6900),precise=fresh)
        self.assertTrue(accepted['allow_arm'])
        self.assertEqual(accepted['parking']['verification_observation']['capture_timestamp_ms'],6800)

    def test_near_recovery_reference_survives_odometry_pruning_without_refreshing_capture(self):
        travel, unused = self.failed_static_after_late_moving_fine()
        raw = list(self.drive._near_recovery['point'])
        self.finish_near_view(5950,travel-5,travel+5,self.clipped_near)
        self.finish_near_view(6250,travel-15,travel-5,self.clipped_near)
        for now in range(6500,9000,50):
            self.update(now,travel=travel-15,right=travel-5,reason=8,
                        coarse=self.missing(now),fine_pending=True)
        reference = self.drive._near_recovery
        self.assertEqual(reference['capture_timestamp_ms'],1100)
        expected = predict_center(raw,[(travel,travel),(-5,5),(-10,-10)],
            self.config['track_width_mm'],self.projection.normal,
            self.projection.forward_axis_Base,self.projection.motion_origin_Base_mm)
        actual = self.drive._recovery_point(reference,9000)
        for a,b in zip(actual,expected):
            self.assertAlmostEqual(a,b,delta=.1)
        self.assertLessEqual(len(self.drive.odometry.rows),12)
        self.assertFalse(self.drive._searching)

    def test_bottom_view_recovery_requires_new_stable_associated_post_stop_coarse(self):
        for condition in ('missing','ambiguous','unassociated','suspected','unconfirmed',
                          'stale','before_stop','reused'):
            with self.subTest(condition=condition):
                self.setUp()
                travel, unused = self.failed_static_after_late_moving_fine(bottom=True)
                self.update(5950,run=True,travel=travel-5,coarse=self.clipped_near(5950))
                self.update(6050,travel=travel-10,reason=8,coarse=self.missing(6050))
                cue = self.clipped_near(6100)
                event = {}
                if condition == 'missing':
                    cue = self.missing(6100)
                    # A query-only native flash still cannot bypass this gate.
                    event = {'precise_candidate':True,'associated':False}
                elif condition == 'ambiguous':cue['visual_evidence']['ambiguous'] = True
                elif condition == 'unassociated':cue['visual_evidence']['associated'] = False
                elif condition == 'suspected':event = {'movement_state':'suspected'}
                elif condition == 'unconfirmed':cue['visual_evidence']['association_confirmations'] = 2
                elif condition == 'stale':cue = self.clipped_near(5000)
                elif condition == 'before_stop':cue = self.clipped_near(6000)
                else:cue = self.clipped_near(5850)
                result = self.update(6100,travel=travel-10,reason=8,coarse=cue,target_event=event)
                self.assertIsNone(result['command'])
                self.assertIsNone(result['request_fine'])
                self.assertFalse(result['allow_arm'])
                self.assertFalse(self.drive._searching)
                self.assertIn('bottom_view_frame_ms',self.drive._near_recovery)
                # A later true associated image can resume the same budget,
                # without restarting an expensive static measurement.
                result = self.update(6200,travel=travel-10,reason=8,
                                     coarse=self.clipped_near(6200))
                self.assertEqual(result['command'],'FOLLOW,-15,-15')
                self.assertIsNone(result['request_fine'])
                self.assertEqual(self.drive.fine_requests,2)

    def test_bottom_view_recovery_waits_for_stable_complete_cue_before_one_new_static(self):
        travel, unused = self.failed_static_after_late_moving_fine(bottom=True)
        self.update(5950,run=True,travel=travel-5,coarse=self.clipped_near(5950))
        self.update(6050,travel=travel-10,reason=8,coarse=self.missing(6050))
        for count in (1,2,3):
            now=6000+count*100
            cue=self.coarse(now,self.target)
            cue['visual_evidence'].update(association_confirmations=count,
                navigation_confirmed=False,distance_valid=False,
                reason='inconsistent_with_20mm_height')
            result=self.update(now,travel=travel-10,reason=8,coarse=cue)
            self.assertIsNone(result['command'])
            self.assertFalse(result['allow_arm'])
            self.assertEqual(result['request_fine'],'static' if count==3 else None)
        self.assertEqual(self.drive.fine_requests,3)
        self.assertNotIn('bottom_view_frame_ms',self.drive._near_recovery)

    def test_bottom_view_recovery_invalidated_during_retreat_stops_and_clears_permission(self):
        for condition in ('moved','version','exited'):
            with self.subTest(condition=condition):
                self.setUp()
                travel, unused=self.failed_static_after_late_moving_fine(bottom=True)
                event={'target_valid':False,'event':'moved'}
                version=1
                if condition=='version':event={};version=2
                elif condition=='exited':
                    event={'event':'exited','last_disappearance_direction':'bottom'}
                result=self.update(5950,run=True,travel=travel-5,
                    coarse=self.clipped_near(5950),target_event=event,target_version=version)
                self.assertEqual(result['command'],'STOP')
                self.assertIsNone(result['request_fine'])
                self.assertIsNone(self.drive._near_recovery)
                self.assertFalse(result['allow_arm'])

    def test_bottom_view_recovery_side_clip_uses_bounded_turn_before_requesting_static(self):
        travel, unused=self.failed_static_after_late_moving_fine(bottom=True)
        result=self.finish_near_view(5950,travel-10,travel-10,
                                    lambda now:self.clipped_near(now,'right'))
        self.assertEqual(result['command'],'ARC,5,-5')
        self.assertIsNone(result['request_fine'])
        self.assertIn('bottom_view_frame_ms',self.drive._near_recovery)

    def test_continuous_reverse_stops_immediately_when_current_cue_loses_authority(self):
        for condition in ('missing','ambiguous','unassociated','suspected','unconfirmed','stale'):
            with self.subTest(condition=condition):
                self.setUp()
                travel,unused=self.failed_static_after_late_moving_fine(bottom=True)
                start=list(self.drive._near_recovery['reverse_start_counts'])
                cue=self.clipped_near(5950);event={}
                if condition=='missing':cue=self.missing(5950)
                elif condition=='ambiguous':cue['visual_evidence']['ambiguous']=True
                elif condition=='unassociated':cue['visual_evidence']['associated']=False
                elif condition=='suspected':event={'movement_state':'suspected'}
                elif condition=='unconfirmed':cue['visual_evidence']['association_confirmations']=2
                else:cue=self.clipped_near(5000)
                result=self.update(5950,run=True,travel=travel-1,coarse=cue,target_event=event)
                self.assertEqual(result['command'],'STOP')
                self.assertIsNone(result['request_fine'])
                self.assertIsNone(self.drive._active_action)
                self.assertEqual(self.drive._near_recovery['reverse_start_counts'],start)
                self.update(6050,travel=travel-2,reason=1,coarse=self.missing(6050))
                resumed=self.update(6100,travel=travel-2,reason=1,coarse=self.clipped_near(6100))
                self.assertEqual(resumed['command'],'FOLLOW,-15,-15')
                self.assertEqual(self.drive._active_action,'recovery_reverse')
                self.assertEqual(self.drive._near_recovery['reverse_start_counts'],start)

    def test_continuous_reverse_100mm_budget_does_not_renew_without_new_geometry(self):
        travel,unused=self.failed_static_after_late_moving_fine(bottom=True)
        start=list(self.drive._near_recovery['reverse_start_counts'])
        # Nominal15mm/s, with the existing0.12s coast model braking before100.
        for index in range(1,7):
            now=5850+index*1000
            result=self.update(now,run=True,travel=travel-index*15,coarse=self.clipped_near(now))
            self.assertEqual(result['command'],'HB')
        result=self.update(12350,run=True,travel=travel-97.5,coarse=self.clipped_near(12350))
        self.assertEqual(result['command'],'HB')
        result=self.update(12450,run=True,travel=travel-99,coarse=self.clipped_near(12450))
        self.assertEqual(result['command'],'STOP')
        self.assertTrue(self.drive._near_recovery['reverse_exhausted'])
        left=right=travel-99.5
        self.update(12550,travel=left,right=right,reason=1,coarse=self.clipped_near(12550))
        for index in range(3):
            now=12600+index*300
            result=self.update(now,travel=left,right=right,reason=1 if index==0 else 8,
                               coarse=self.clipped_near(now))
            self.assertTrue(result['command'].startswith('ARC,'),result)
            dl,dr=map(int,result['command'].split(',')[1:])
            self.assertLess(dl*dr,0)
            self.assertIsNone(result['request_fine'])
            self.assertEqual(self.drive._near_recovery['reverse_start_counts'],start)
            self.assertEqual(self.drive.fine_requests,2)
            self.update(now+100,run=True,travel=left,right=right,coarse=self.clipped_near(now+100))
            left+=dl;right+=dr
            self.update(now+250,travel=left,right=right,reason=8,coarse=self.clipped_near(now+250))

    def test_continuous_reverse_deadline_reobserves_without_resetting_distance_budget(self):
        travel,unused=self.failed_static_after_late_moving_fine(bottom=True)
        start=list(self.drive._near_recovery['reverse_start_counts'])
        self.update(5950,run=True,travel=travel-2,coarse=self.clipped_near(5950))
        ended=self.update(25850,travel=travel-20,reason=3,coarse=self.clipped_near(25850))
        self.assertIsNone(ended['command'])
        self.assertIsNone(ended['request_fine'])
        resumed=self.update(25900,travel=travel-20,reason=3,coarse=self.clipped_near(25900))
        self.assertEqual(resumed['command'],'FOLLOW,-15,-15')
        self.assertEqual(self.drive._near_recovery['reverse_start_counts'],start)
        self.assertEqual(self.drive._active_action,'recovery_reverse')
        self.assertIsNone(self.drive._motion_hold)

    def test_continuous_reverse_stops_previous_arc_or_forward_before_taking_ownership(self):
        for previous in ('arc','follow'):
            with self.subTest(previous=previous):
                self.setUp()
                if previous=='arc':
                    travel,unused=self.failed_static_after_late_moving_fine()
                    now=5950
                else:
                    self.request_static(x=330)
                    self.update(1150,330,precise=self.precise(1050,330))
                    travel=2;now=1250
                self.assertEqual(self.drive._active_action,previous)
                self.drive.restart_observation(change_view=True)
                result=self.update(now,run=True,travel=travel,coarse=self.clipped_near(now))
                self.assertEqual(result['command'],'STOP')
                self.update(now+100,travel=travel,reason=1,coarse=self.clipped_near(now+100))
                result=self.update(now+150,travel=travel,reason=1,coarse=self.clipped_near(now+150))
                self.assertEqual(result['command'],'FOLLOW,-15,-15')
                self.assertEqual(self.drive._active_action,'recovery_reverse')

    def test_near_recovery_unknown_waits_but_confirmed_invalidation_revokes_old_geometry(self):
        for condition in ('unknown','moved','exited','version'):
            with self.subTest(condition=condition):
                self.setUp()
                travel, unused = self.failed_static_after_late_moving_fine()
                event = {}
                version = 1
                if condition == 'moved':
                    event = {'target_valid':False,'event':'moved'}
                elif condition == 'exited':
                    event = {'event':'exited','last_disappearance_direction':'bottom'}
                elif condition == 'version':
                    version = 2
                result = self.update(5950,travel=travel,reason=8,coarse=self.missing(5950),
                    target_version=version,target_event=event)
                self.assertFalse(result['allow_arm'])
                self.assertIsNone(result['request_fine'])
                if condition == 'unknown':
                    self.assertIsNotNone(self.drive._near_recovery)
                    self.assertFalse(self.drive._searching)
                else:
                    self.assertIsNone(self.drive._near_recovery)

    def test_near_recovery_after_bottom_retreat_uses_old_geometry_for_the_next_turn(self):
        for actual_turn, direction in ((5,1),(10,-1)):
            with self.subTest(actual_turn=actual_turn):
                self.setUp()
                # This steering fixture follows the current window centreline
                # and table height; incident replay keeps its original XYZ.
                travel, unused = self.failed_static_after_late_moving_fine(
                    moving_point=self.point(413.1257,5))
                left,right = travel-actual_turn,travel+actual_turn
                self.finish_near_view(5950,left,right,self.clipped_near)
                self.finish_near_view(6250,left-10,right-10,
                    lambda now:self.coarse(now,self.target))
                # Rejected range does not select a default search direction.
                # Actual encoder yaw, including overshoot, changes the turn.
                failed = self.precise(6400,self.target,status='no_complete_face')
                failed['paired_coarse'] = self.weak_cue(6400,3)
                failed['result_ready_ms'] = 7300
                result = self.update(7350,travel=left-10,right=right-10,reason=8,
                    coarse=self.weak_cue(7350,3),precise=failed)
                command_left,command_right = map(int,result['command'].split(',')[1:])
                self.assertGreater(direction*command_right,0)
                self.assertLess(direction*command_left,0)
                self.assertFalse(self.drive._searching)
                self.assertFalse(result['allow_arm'])

    def test_search_is_opposed_signed_follow_and_stops_when_reacquired(self):
        missing=self.missing(1000)
        self.update(1000,coarse=missing,target_event={'event':'exited',
                    'last_disappearance_direction':'right'})
        result=self.update(1050,coarse=self.missing(1050))
        self.assertEqual(result['command'],'FOLLOW,20,-20')
        left,right=map(int,result['command'].split(',')[1:])
        self.assertGreater(left,0);self.assertLess(right,0)
        result=self.update(1150,run=True,coarse=self.coarse(1150))
        self.assertEqual(result['command'],'STOP')
        self.assertEqual(result['status'],'reacquired')

    def test_search_can_continue_without_new_terminal_limit(self):
        self.drive.request_search('left')
        self.update(1000,coarse=self.missing(1000))
        self.assertEqual(self.update(1050,coarse=self.missing(1050))['command'],'FOLLOW,-20,20')
        for index in range(8):
            now=1150+100*index
            result=self.update(now,run=True,travel=-2*index,right=2*index,
                               coarse=self.missing(now))
            self.assertEqual(result['command'],'HB')
            self.assertEqual(result['phase'],'searching')
        # The unchanged MCU20s segment stop is observed; a new segment is
        # allowed only after the normal stopped observation interval.
        for now in (21050,41100,61150):
            self.update(now-100,run=True,travel=-398,right=398,coarse=self.missing(now-100))
            stopped=self.update(now,travel=-400,right=400,reason=3,coarse=self.missing(now))
            self.assertIsNone(stopped['command'])
            self.assertNotEqual(stopped['status'],'motion_wait')
            resumed=self.update(now+50,travel=-400,right=400,reason=3,
                                coarse=self.missing(now+50))
            self.assertEqual(resumed['command'],'FOLLOW,-20,20')

    def test_new_unique_candidate_confirms_before_changing_search_direction(self):
        for prior, box, expected in (
                ('left',[900,300,80,80],'SPEED,20,-20'),
                ('right',[300,300,80,80],'SPEED,-20,20')):
            with self.subTest(prior=prior):
                self.setUp();self.drive.request_search(prior)
                self.update(1000,coarse=self.missing(1000))
                tracker=BlueTracker(self.config['tracking'],self.projection)
                for index, now in enumerate((1050,1150,1250)):
                    cue=self.native_cue(now,Blob(box),tracker=tracker)
                    cue['visual_evidence']['associated']=False
                    self.assertTrue(cue['visual_candidates'][0]['distance_valid'])
                    result=self.update(now,run=index>0,coarse=cue,target_event={'associated':False})
                    if index==0:
                        self.assertEqual(result['command'],
                            'FOLLOW,-20,20' if prior=='left' else 'FOLLOW,20,-20')
                    elif index==1:
                        self.assertEqual(result['command'],'HB')
                        self.assertEqual(self.drive._search_direction,prior)
                    else:self.assertEqual(result['command'],expected)
                self.assertTrue(self.drive._searching)
                self.assertIsNone(self.drive._coarse_reference)
                self.assertIsNone(result.get('plan'))
                self.assertFalse(result['allow_arm'])

    def test_bottom_retreat_keeps_budget_then_uses_the_current_right_candidate(self):
        self.update(1000,coarse=self.missing(1000),target_event={
            'search':True,'last_disappearance_direction':'bottom'})
        tracker=BlueTracker(self.config['tracking'],self.projection)
        cue=self.native_cue(1050,Blob([900,300,80,80]),tracker=tracker)
        cue['visual_evidence']['associated']=False
        self.assertEqual(self.update(1050,coarse=cue)['command'],'ARC,-10,-10')
        cue=self.native_cue(1150,Blob([900,300,80,80]),tracker=tracker)
        cue['visual_evidence']['associated']=False
        self.update(1150,travel=-10,reason=8,coarse=cue)
        cue=self.native_cue(1200,Blob([900,300,80,80]),tracker=tracker)
        cue['visual_evidence']['associated']=False
        self.assertEqual(self.update(1200,travel=-10,reason=8,coarse=cue)['command'],'FOLLOW,20,-20')
        self.assertTrue(self.drive._search_reverse_done)

    def test_associated_canonical_target_owns_heading_before_other_raw_candidates(self):
        for weak in (True,False):
            with self.subTest(selected_range_valid=not weak):
                self.setUp()
                memory=TargetMemory(self.projection,self.config)
                tracker=BlueTracker(self.config['tracking'],self.projection)
                primary=Blob([900,400,80,80],3200)
                for now in (700,800,900):
                    coarse=self.native_cue(now,primary,tracker=tracker)
                    memory.update(coarse,motion_state=self.state(now))
                self.drive.request_search('right' if weak else 'left')
                for now in (1000,1050):
                    coarse=self.native_cue(now,tracker=tracker)
                    event=memory.update(coarse,motion_state=self.state(now))
                    self.update(now,coarse=coarse,target_event=event,target_version=memory.version)
                selected=Blob([900,500,84,34],1800) if weak else primary
                coarse=self.native_cue(1150,selected,Blob([300,300,80,80],3200),tracker=tracker)
                event=memory.update(coarse,motion_state=self.state(1150,run=True))
                evidence=coarse['visual_evidence']
                self.assertTrue(evidence['associated'])
                self.assertFalse(evidence['ambiguous'])
                self.assertTrue(event['associated'])
                self.assertEqual(evidence['bbox_px'],selected.box)
                self.assertFalse(evidence['navigation_confirmed'])
                self.assertEqual(sum(c['distance_valid'] for c in coarse['visual_candidates']),
                                 1 if weak else 2)
                result=self.update(1150,run=True,coarse=coarse,target_event=event,
                                   target_version=memory.version)
                self.assertEqual(self.drive._search_direction,'right')
                self.assertEqual(result['command'],'HB' if weak else 'SPEED,20,-20')
                self.assertFalse(result['reset_tracker'])
                self.assertIsNone(self.drive._coarse_reference)
                self.assertFalse(result['allow_arm'])

    def test_recorded_870px_flash_cannot_replace_confirmed_search_direction(self):
        class RecordedBlob(Blob):
            def __init__(self,box,pixels,center):
                super().__init__(box,pixels);self.center=center
            def cx(self):return self.center[0]
            def cy(self):return self.center[1]
        memory=TargetMemory(self.projection,self.config)
        tracker=BlueTracker(self.config['tracking'],self.projection)
        original=RecordedBlob([548,279,112,141],14402,[604.,348.])
        for now in (700,800,900):
            coarse=self.native_cue(now,original,tracker=tracker)
            memory.update(coarse,motion_state=self.state(now))
        self.drive.request_search('right')

        def sample(now, blobs=(), run=True):
            coarse=self.native_cue(now,*blobs,tracker=tracker)
            event=memory.update(coarse,motion_state=self.state(now,run=run))
            result=self.update(now,run=run,coarse=coarse,target_event=event,
                               target_version=memory.version)
            return coarse,event,result

        sample(1000,run=False);sample(1050,run=False);sample(1200)
        # Native measurements retained in search_half_turn_01: no reduced
        # pixel threshold, invented centroid or claim that this patch was A.
        patch=RecordedBlob([528,233,69,35],870,[557.,248.])
        raw=[patch,
            RecordedBlob([531,234,7,8],32,[534.,237.]),
            RecordedBlob([412,236,17,8],63,[420.,239.]),
            RecordedBlob([586,238,7,7],31,[589.,241.]),
            RecordedBlob([412,247,87,18],630,[453.,256.]),
            RecordedBlob([597,249,15,10],70,[603.,254.]),
            RecordedBlob([416,261,15,6],30,[423.,264.]),
            RecordedBlob([561,280,15,5],35,[568.,282.]),
            RecordedBlob([578,282,13,5],35,[584.,284.])]
        coarse,event,result=sample(1600,raw)
        evidence=coarse['visual_evidence']
        self.assertEqual(evidence['bbox_px'],patch.box)
        self.assertEqual(evidence['center_pixel'],[557.,248.])
        self.assertTrue(evidence['distance_valid'])
        self.assertFalse(evidence['navigation_confirmed'])
        self.assertEqual(evidence['association_confirmations'],1)
        self.assertEqual(len(coarse['visual_candidates']),9)
        # The actual calibration gives this left-of-midline ray a positive
        # (left) bearing; preserving right here is the deliberate confirmation gate.
        self.assertGreater(self.drive._coarse_direction(coarse,1600),0)
        self.assertEqual(result['command'],'HB')
        self.assertEqual(self.drive._search_direction,'right')
        self.assertEqual(self.drive._search_region['bbox_px'],original.box)
        self.assertIsNone(self.drive._coarse_reference)
        self.assertIsNone(self.drive.anchor)
        self.assertFalse(result['allow_arm'])
        repeated=self.update(1650,run=True,coarse=coarse,target_event=event,
                             target_version=memory.version)
        self.assertEqual(repeated['command'],'HB')
        self.assertEqual(self.drive._search_hint['confirmations'],1)
        ambiguous,unused,result=sample(1700,[Blob([200,400,80,80],3200),
                                            Blob([900,300,80,80],3200)])
        self.assertTrue(ambiguous['visual_evidence']['ambiguous'])
        self.assertEqual(result['command'],'HB')
        self.assertIsNone(self.drive._search_hint)
        for now in (1800,1900):
            coarse,event,result=sample(now,raw)
            self.assertEqual(result['command'],'HB')
            self.assertEqual(self.drive._search_direction,'right')
        coarse,event,result=sample(2000,raw)
        # These original companion patches can still confuse native selection.
        # Direction-only confirmation belongs to B's own three raw frames;
        # it neither borrows another cue's count nor grants B an identity/range.
        self.assertEqual(self.drive._search_hint['confirmations'],3)
        self.assertEqual(self.drive._search_direction,'left')
        self.assertIsNone(self.drive._coarse_reference)
        self.assertIsNone(self.drive.anchor)
        self.assertFalse(result['allow_arm'])

    def test_search_changes_direction_with_speed_and_heartbeats_continuously(self):
        self.drive.request_search('left')
        self.update(1000,coarse=self.missing(1000))
        self.assertEqual(self.update(1050,coarse=self.missing(1050))['command'],'FOLLOW,-20,20')
        for now, box, expected in (
                (1150,[900,300,80,80],'HB'),
                (1250,[900,300,80,80],'HB'),
                (1350,[900,300,80,80],'SPEED,20,-20'),
                (1450,[900,300,80,80],'HB'),
                (1550,[300,300,80,80],'HB'),
                (1650,[300,300,80,80],'HB'),
                (1750,[300,300,80,80],'SPEED,-20,20'),
                (1850,[300,300,80,80],'HB')):
            cue=self.native_cue(now,Blob(box))
            result=self.update(now,run=True,coarse=cue)
            self.assertEqual(result['command'],expected)
            self.assertEqual(self.drive._active_action,'search_follow')
            self.assertIsNone(self.drive._coarse_reference)
            self.assertIsNone(result['request_fine'])
        result=self.update(1950,run=True,coarse=self.missing(1950),target_version=2)
        self.assertEqual(result['command'],'HB')

    def test_search_midline_deadband_waits_for_confirmation_without_single_frame_stop(self):
        self.drive.request_search('right')
        self.update(1000,coarse=self.missing(1000))
        self.update(1050,coarse=self.missing(1050))
        tracker=BlueTracker(self.config['tracking'],self.projection)
        for now in (1150,1250,1350):
            cue=self.native_cue(now,Blob([600,300,80,80]),tracker=tracker)
            result=self.update(now,run=True,coarse=cue)
            self.assertEqual(result['command'],'STOP' if now==1350 else 'HB')
        self.assertEqual(result['status'],'reacquired')
        self.assertFalse(self.drive._searching)

    def test_self_consistent_clipped_cue_can_steer_without_a_distance(self):
        for prior, box, expected in (
                ('right',[0,300,80,80],'SPEED,-20,20'),
                ('left',[1200,300,80,80],'SPEED,20,-20')):
            with self.subTest(box=box):
                self.setUp();self.drive.request_search(prior)
                self.update(1000,coarse=self.missing(1000));self.update(1050,coarse=self.missing(1050))
                for now in (1150,1250,1350):
                    cue=self.native_cue(now,Blob(box,3200))
                    cue['visual_evidence']['associated']=False
                    self.assertEqual(cue['visual_evidence']['reason'],'clipped_blob')
                    result=self.update(now,run=True,coarse=cue,target_event={'associated':False})
                    self.assertEqual(result['command'],expected if now==1350 else 'HB')
                self.assertIsNone(result['request_fine'])
                self.assertNotIn('ground_contact_proxy_Base_mm',cue['visual_evidence'])
                self.assertNotIn('ground_contact_proxy_Base_mm',cue)
                self.assertIsNone(self.drive._coarse_reference)

    def test_recorded_27_and_31_pixel_noise_never_override_search_direction(self):
        class RecordedBlob(Blob):
            def __init__(self,box,pixels,center):
                super().__init__(box,pixels);self.center=center
            def cx(self):return self.center[0]
            def cy(self):return self.center[1]
        tracker=BlueTracker(self.config['tracking'],self.projection)
        target=RecordedBlob([604,277,109,142],11587,[649.,346.])
        for now in (800,850,900):trusted=self.native_cue(now,target,tracker=tracker)
        self.drive.request_search('right')
        self.update(1000,coarse=self.missing(1000),target_event={'last_seen':trusted})
        self.update(1050,coarse=self.missing(1050),target_event={'last_seen':trusted})
        for now, blob in (
                (1150,RecordedBlob([353,341,9,5],31,[357.,343.])),
                (1250,RecordedBlob([0,915,11,5],27,[6.,917.]))):
            cue=self.native_cue(now,blob)
            self.assertTrue(cue['visual_evidence']['associated'])
            self.assertEqual(len(cue['visual_candidates']),1)
            self.assertGreaterEqual(cue['visual_evidence']['pixels'],self.config['tracking']['min_pixels'])
            self.assertFalse(cue['visual_evidence']['distance_valid'])
            result=self.update(now,run=True,coarse=cue,target_event={'last_seen':trusted})
            self.assertEqual(result['command'],'HB')
            self.assertEqual(self.drive._search_direction,'right')
            self.assertFalse(result['allow_arm'])

    def test_unique_self_consistent_clipped_cue_beats_an_ambiguous_noise_patch(self):
        self.drive.request_search('right')
        self.update(1000,coarse=self.missing(1000));self.update(1050,coarse=self.missing(1050))
        for now in (1150,1250,1350):
            cue=self.native_cue(now,Blob([0,300,80,80],3200),Blob([353,341,9,5],31))
            self.assertTrue(cue['visual_evidence']['ambiguous'])
            self.assertEqual(len(cue['visual_candidates']),2)
            result=self.update(now,run=True,coarse=cue)
            self.assertEqual(result['command'],'SPEED,-20,20' if now==1350 else 'HB')
        self.assertTrue(self.drive._searching)
        self.assertIsNone(self.drive._coarse_reference)
        self.assertFalse(result['allow_arm'])

    def test_unique_raw_measurement_steers_when_selected_weak_noise_is_elsewhere(self):
        self.drive.request_search('left')
        self.update(1000,coarse=self.missing(1000));self.update(1050,coarse=self.missing(1050))
        cue=self.native_cue(1150,Blob([353,341,9,5],31))
        cue['visual_evidence']['association_confirmations']=99
        measured=self.native_cue(1150,Blob([900,300,80,80]))
        cue['visual_candidates']+=measured['visual_candidates']
        original=dict(cue['visual_evidence'])
        result=self.update(1150,run=True,coarse=cue,target_event={'associated':False})
        self.assertEqual(result['command'],'HB')
        self.assertTrue(result['reset_tracker'])
        self.assertEqual(cue['visual_evidence'],original)
        self.assertIsNone(self.drive._coarse_reference)
        self.assertTrue(self.drive._searching)
        self.assertIsNone(result.get('plan'))
        self.assertFalse(result['allow_arm'])
        # The owner honors reset after this frame. A freshly selected B must
        # then accumulate its own confirmations without repeated resets.
        tracker=BlueTracker(self.config['tracking'],self.projection)
        for now in (1250,1350,1450):
            cue=self.native_cue(now,Blob([900,300,80,80]),tracker=tracker)
            result=self.update(now,run=True,coarse=cue)
            self.assertFalse(result['reset_tracker'])
            self.assertEqual(result['command'],
                'STOP' if now==1450 else 'SPEED,20,-20' if now==1350 else 'HB')
        self.assertEqual(result['status'],'reacquired')

    def test_stable_native_weak_visual_lock_stops_search_without_range_or_navigation(self):
        tracker=BlueTracker(self.config['tracking'],self.projection)
        for now in (800,850,900):
            trusted=self.native_cue(now,Blob([900,400,80,80]),tracker=tracker)
        self.assertTrue(trusted['visual_evidence']['navigation_confirmed'])
        self.drive.request_search('right')
        self.update(1000,coarse=self.missing(1000),target_event={'last_seen':trusted})
        self.update(1050,coarse=self.missing(1050))
        tracker=BlueTracker(self.config['tracking'],self.projection)
        for now in (1150,1250,1350):
            cue=self.native_cue(now,Blob([900,500,84,34],1800),tracker=tracker)
            evidence=cue['visual_evidence']
            self.assertTrue(evidence['associated'])
            self.assertFalse(evidence['distance_valid'])
            self.assertFalse(evidence['navigation_confirmed'])
            result=self.update(now,run=True,coarse=cue)
            self.assertEqual(result['command'],'STOP' if now==1350 else 'HB')
            self.assertIsNone(self.drive._coarse_reference)
            self.assertIsNone(self.drive.anchor)
            self.assertFalse(result['allow_arm'])
        self.assertEqual(result['phase'],'visual_tracking')
        self.assertEqual(result['status'],'visual_tracking')
        self.assertFalse(self.drive._searching)
        self.assertTrue(self.drive._search_inspecting)
        self.update(1450,reason=1,coarse=self.native_cue(1450,Blob([900,500,84,34],1800),tracker=tracker))
        result=self.update(1500,reason=1,coarse=self.native_cue(1500,Blob([900,500,84,34],1800),tracker=tracker))
        self.assertEqual(result['request_fine'],'static')
        self.assertEqual(result['status'],'visual_tracking')

    def test_search_ambiguous_independent_candidates_do_not_choose_the_largest(self):
        self.drive.request_search('right')
        self.update(1000,coarse=self.missing(1000));self.update(1050,coarse=self.missing(1050))
        cue=self.native_cue(1150,Blob([200,300,100,100]),Blob([900,300,80,80]))
        self.assertTrue(cue['visual_evidence']['ambiguous'])
        self.assertEqual(sum(c['distance_valid'] for c in cue['visual_candidates']),2)
        result=self.update(1150,run=True,coarse=cue)
        self.assertEqual(result['command'],'HB')
        self.assertEqual(self.drive._search_direction,'right')

    def test_search_keeps_existing_button_and_heartbeat_stops(self):
        for reason in (2,4):
            with self.subTest(reason=reason):
                self.setUp();self.drive.request_search('left')
                self.update(1000,coarse=self.missing(1000));self.update(1050,coarse=self.missing(1050))
                result=self.update(1150,reason=reason,coarse=self.missing(1150))
                self.assertEqual(result['status'],'motion_wait')
                self.assertIsNone(result['command'])
                self.assertEqual(self.update(1250,coarse=self.missing(1250))['status'],'motion_wait')

    def test_static_query_is_not_navigation_or_arm_authority_until_identity_returns(self):
        for x in (self.target,330):
            with self.subTest(x=x):
                self.setUp()
                event={'visible':False,'associated':False,'precise_candidate':True,'target_valid':False}
                self.update(1000,coarse=self.weak_cue(1000,3),target_event=event)
                result=self.update(1050,coarse=self.weak_cue(1050,4),target_event=event)
                self.assertEqual(result['request_fine'],'static')
                self.assertFalse(result['target_visible'])
                request_id=result['fine_request_id']
                self.assertTrue(self.drive._fine_request['query_only'])
                pending=self.update(1100,coarse=self.weak_cue(1100,5),
                    target_event=event,fine_pending=True)
                self.assertEqual(self.drive._fine_request['request_id'],request_id)
                self.assertIsNone(pending['request_fine'])
                complete=self.precise(1050,x)
                complete['paired_coarse']=self.weak_cue(1050,4)
                result=self.update(1150,coarse=self.weak_cue(1150,6),
                                   target_event=event,precise=complete)
                self.assertTrue(result['precise_consumed'])
                self.assertFalse(result['precise_deferred'])
                self.assertFalse(result['allow_arm'])
                self.assertIsNone(result['command'])
                self.assertEqual(result['status'],'waiting_for_identity')
                result=self.update(1250,coarse=self.weak_cue(1250,7),
                    target_event={'visible':True,'associated':True,'target_valid':True})
                if x==self.target:
                    self.assertTrue(result['allow_arm'])
                    self.assertIsNone(result['command'])
                else:
                    self.assertTrue(result['command'].startswith('FOLLOW,'))
                    self.assertFalse(result['allow_arm'])

    def test_query_version_change_revokes_old_request_and_completion(self):
        event={'visible':False,'associated':False,'precise_candidate':True,'target_valid':False}
        self.update(1000,coarse=self.weak_cue(1000,3),target_event=event)
        self.update(1050,coarse=self.weak_cue(1050,4),target_event=event)
        old=self.precise(1050,self.target)
        old['paired_coarse']=self.weak_cue(1050,4)
        result=self.update(1150,coarse=self.weak_cue(1150,5),target_version=2,
                           target_event=event,precise=old)
        self.assertFalse(result['precise_consumed'])
        self.assertFalse(result['allow_arm'])
        self.assertIsNone(result['command'])
        self.assertEqual(result['fine_request_id'][0],2)
        self.assertNotEqual(result['fine_request_id'],old['request_id'])

    def test_failed_first_query_inspects_a_real_new_view_without_global_search(self):
        event={'visible':False,'associated':False,'precise_candidate':True,'target_valid':False}
        self.update(1000,coarse=self.weak_cue(1000,3),target_event=event)
        self.update(1050,coarse=self.weak_cue(1050,4),target_event=event)
        failed=self.precise(1050,self.target,status='no_complete_face')
        failed['paired_coarse']=self.weak_cue(1050,4)
        result=self.update(1150,coarse=self.weak_cue(1150,5),target_event=event,precise=failed)
        self.assertTrue(result['precise_consumed'])
        self.assertEqual(result['command'],'FOLLOW,-20,20')
        self.assertEqual(result['phase'],'visual_tracking')
        self.assertEqual(result['status'],'changing_view')
        self.assertFalse(self.drive._searching)
        self.assertTrue(self.drive._search_inspecting)
        self.assertEqual(self.drive._active_action,'inspection_follow')
        self.assertIsNone(result['request_fine'])
        self.assertEqual(self.drive._adjust_count,0)
        self.assertTrue(self.drive._search_view_pending)
        result=self.update(5000,run=True,travel=-2,right=2,
                           coarse=self.weak_cue(5000,6),target_event=event)
        self.assertEqual(result['command'],'HB')
        self.assertIsNone(result['request_fine'])
        self.assertTrue(self.drive._search_view_pending)
        result=self.update(5100,run=True,travel=-11,right=11,
                           coarse=self.weak_cue(5100,7),target_event=event)
        self.assertEqual(result['command'],'STOP')
        self.assertFalse(result['allow_arm'])
        self.assertIsNone(self.drive._coarse_reference)
        self.assertGreaterEqual(self.drive._search_view_yaw_deg,10)
        self.update(5200,travel=-11,right=11,reason=1,
                    coarse=self.weak_cue(5200,8),target_event=event)
        result=self.update(5250,travel=-11,right=11,reason=1,
                           coarse=self.weak_cue(5250,9),target_event=event)
        self.assertEqual(result['request_fine'],'static')
        failed=self.precise(5250,self.target,status='no_complete_face')
        failed['paired_coarse']=self.weak_cue(5250,9)
        result=self.update(5350,travel=-11,right=11,reason=1,
                           coarse=self.weak_cue(5350,10),target_event=event,precise=failed)
        self.assertEqual(result['command'],'FOLLOW,-20,20')
        self.assertEqual(result['status'],'changing_view')
        self.assertFalse(self.drive._searching)
        self.assertTrue(self.drive._search_inspecting)
        self.assertEqual(self.drive._active_action,'inspection_follow')
        self.assertFalse(result['allow_arm'])

    def test_inspection_unknown_frames_pause_and_only_confirmed_loss_reenters_search(self):
        event={'visible':False,'associated':False,'precise_candidate':True,'target_valid':False}
        self.update(1000,coarse=self.weak_cue(1000,3),target_event=event)
        self.update(1050,coarse=self.weak_cue(1050,4),target_event=event)
        failed=self.precise(1050,self.target,status='no_complete_face')
        failed['paired_coarse']=self.weak_cue(1050,4)
        self.update(1150,coarse=self.weak_cue(1150,5),target_event=event,precise=failed)
        missing={'visible':False,'associated':False,'target_valid':False,'precise_candidate':False}
        first=self.update(1250,run=True,travel=-2,right=2,
                          coarse=self.missing(1250),target_event=missing)
        self.assertEqual(first['command'],'STOP')
        self.assertEqual(first['status'],'waiting_for_target')
        self.assertEqual(first['phase'],'observing')
        second=self.update(1350,travel=-2,right=2,reason=1,
                           coarse=self.missing(1350),target_event=missing)
        self.assertIsNone(second['command'])
        self.assertIsNone(second['request_fine'])
        self.assertFalse(self.drive._searching)
        self.assertTrue(self.drive._search_inspecting)
        # A current query candidate returning during the pause still belongs
        # to the local view change, even if an old missing notification exists.
        resumed=self.update(1450,travel=-2,right=2,reason=1,
            coarse=self.weak_cue(1450,6),target_event=dict(event,search=True))
        self.assertEqual(resumed['command'],'FOLLOW,-20,20')
        self.assertEqual(resumed['status'],'changing_view')
        self.assertFalse(self.drive._searching)
        self.assertEqual(self.drive._active_action,'inspection_follow')
        self.update(1550,run=True,travel=-3,right=3,
                    coarse=self.missing(1550),target_event=missing)
        self.update(1650,travel=-3,right=3,reason=1,coarse=self.missing(1650),
                    target_event=dict(missing,search=True,last_disappearance_direction='right'))
        result=self.update(1700,travel=-3,right=3,reason=1,coarse=self.missing(1700),
                           target_event=missing)
        self.assertEqual(result['command'],'FOLLOW,20,-20')
        self.assertTrue(self.drive._searching)
        self.assertFalse(self.drive._search_inspecting)
        self.assertEqual(self.drive._active_action,'search_follow')

    def test_inspection_epoch_change_stops_old_turn_before_new_static_query(self):
        event={'visible':False,'associated':False,'precise_candidate':True,'target_valid':False}
        self.update(1000,coarse=self.weak_cue(1000,3),target_event=event)
        self.update(1050,coarse=self.weak_cue(1050,4),target_event=event)
        failed=self.precise(1050,self.target,status='no_complete_face')
        failed['paired_coarse']=self.weak_cue(1050,4)
        self.update(1150,coarse=self.weak_cue(1150,5),target_event=event,precise=failed)
        result=self.update(1250,run=True,travel=-2,right=2,coarse=self.weak_cue(1250,6),
                           target_event=event,target_version=2)
        self.assertEqual(result['command'],'STOP')
        self.assertFalse(self.drive._searching)
        self.assertFalse(self.drive._search_view_pending)
        self.assertIsNone(result['request_fine'])
        self.assertFalse(result['allow_arm'])

    def test_inspection_centers_supported_side_clip_without_reentering_global_search(self):
        event={'visible':False,'associated':False,'precise_candidate':True,'target_valid':False}
        self.update(1000,coarse=self.weak_cue(1000,3),target_event=event)
        self.update(1050,coarse=self.weak_cue(1050,4),target_event=event)
        failed=self.precise(1050,self.target,status='no_complete_face')
        failed['paired_coarse']=self.weak_cue(1050,4)
        self.update(1150,coarse=self.weak_cue(1150,5),target_event=event,precise=failed)
        tracker=BlueTracker(self.config['tracking'],self.projection)
        for now in (1170,1200,1250):
            cue=self.native_cue(now,Blob([1200,300,80,80],3200),tracker=tracker)
        self.assertFalse(cue['visual_evidence']['distance_valid'])
        self.assertEqual(cue['visual_evidence']['association_confirmations'],3)
        result=self.update(1250,run=True,travel=-2,right=2,coarse=cue,target_event=event)
        self.assertEqual(result['command'],'SPEED,20,-20')
        self.assertEqual(result['status'],'changing_view')
        self.assertEqual(result['phase'],'visual_tracking')
        self.assertEqual(self.drive._active_action,'inspection_follow')
        self.assertFalse(self.drive._searching)
        self.assertIsNone(result['request_fine'])

    def test_bottom_exit_reverses_once_then_rotates_instead_of_retreating_forever(self):
        self.update(1000,coarse=self.missing(1000),target_event={'event':'exited',
                    'last_disappearance_direction':'bottom'})
        first=self.update(1050,coarse=self.missing(1050))
        self.assertEqual(first['command'],'ARC,-10,-10')
        self.update(1150,travel=-10,reason=8,coarse=self.missing(1150))
        second=self.update(1200,travel=-10,reason=8,coarse=self.missing(1200))
        self.assertEqual(second['command'],'FOLLOW,-20,20')
        left,right=map(int,second['command'].split(',')[1:])
        self.assertLess(left,0);self.assertGreater(right,0)
        third=self.update(1300,travel=-10,reason=8,coarse=self.missing(1300))
        self.assertNotEqual(third['command'],'ARC,-10,-10')

    def test_seven_weak_flashes_do_not_restart_bottom_retreat_or_create_forward_route(self):
        self.update(1000,coarse=self.missing(1000),target_event={
            'search':True,'last_disappearance_direction':'bottom'})
        result=self.update(1050,coarse=self.missing(1050))
        commands=[result['command']]
        self.update(1150,travel=-10,reason=8,coarse=self.missing(1150))
        commands.append(self.update(1200,travel=-10,reason=8,coarse=self.missing(1200))['command'])
        for cycle in range(7):
            start=1300+cycle*400
            result=self.update(start,run=True,coarse=self.weak_cue(start),
                target_event={'visible':True,'associated':True,'event':'reappeared'})
            if result['command']:commands.append(result['command'])
            self.assertNotEqual(result['status'],'reacquired')
            self.assertIsNone(self.drive._coarse_reference)
            self.assertEqual(result['command'],'HB')
            for index in range(1,4):
                now=start+index*100
                result=self.update(now,run=True,
                    coarse=self.missing(now),target_event={
                        'search':index==3,'last_disappearance_direction':'bottom'})
                if result['command']:commands.append(result['command'])
        self.assertEqual(commands.count('ARC,-10,-10'),1)
        self.assertEqual(commands.count('FOLLOW,-20,20'),1)
        self.assertTrue(all(c in ('ARC,-10,-10','FOLLOW,-20,20','HB') for c in commands))

    def test_search_stops_old_follow_instead_of_heartbeating_its_invalid_route(self):
        for changed in (False,True):
            with self.subTest(changed=changed):
                self.setUp();self.update(1000)
                event={'last_disappearance_direction':'bottom',
                       'event':'exited' if changed else 'none','search':not changed}
                version=2 if changed else 1
                result=self.update(1100,run=True,travel=7,coarse=self.missing(1100),
                    target_version=version,target_event=event)
                self.assertEqual(result['command'],'STOP')
                self.assertEqual(result['status'],'starting_search')
                self.assertIsNone(self.drive._coarse_reference)
                self.update(1200,travel=7,reason=1,coarse=self.missing(1200),target_version=version)
                result=self.update(1250,travel=7,reason=1,coarse=self.missing(1250),target_version=version)
                self.assertEqual(result['command'],'ARC,-10,-10')
                self.assertEqual(self.update(1350,run=True,travel=6,
                    coarse=self.missing(1350),target_version=version)['command'],'HB')

    def test_search_does_not_adopt_an_old_correction_arc_as_its_own(self):
        x=self.target-10;self.request_static(x=x)
        self.update(1150,x,precise=self.precise(1050,x))
        result=self.update(1250,run=True,coarse=self.missing(1250),
            target_event={'search':True,'last_disappearance_direction':'left'})
        self.assertEqual(result['command'],'STOP')
        self.assertEqual(result['status'],'starting_search')

    def test_navigation_confirmation_not_single_cue_ends_search_then_new_loss_can_reverse(self):
        self.update(1000,coarse=self.missing(1000),target_event={
            'search':True,'last_disappearance_direction':'bottom'})
        self.update(1050,coarse=self.missing(1050))
        for count in (1,2):
            now=1050+count*100;cue=self.coarse(now)
            cue['status']='confirming'
            cue['visual_evidence'].update(confirmations=count,navigation_confirmed=False,
                association_confirmations=count)
            result=self.update(now,run=True,coarse=cue)
            self.assertEqual(result['command'],'HB')
            self.assertTrue(self.drive._searching)
            self.assertIsNone(self.drive._coarse_reference)
        result=self.update(1350,run=True,coarse=self.coarse(1350))
        self.assertEqual(result['command'],'STOP')
        self.assertEqual(result['status'],'reacquired')
        self.assertFalse(self.drive._searching)
        self.update(1450,reason=1,coarse=self.missing(1450),target_version=2,
            target_event={'event':'exited','last_disappearance_direction':'bottom'})
        result=self.update(1500,reason=1,coarse=self.missing(1500),target_version=2)
        self.assertEqual(result['command'],'ARC,-10,-10')

    def test_canonical_evidence_cannot_authorize_another_top_level_candidate(self):
        cue=self.coarse(1000,offset=35)
        candidate_b=self.coarse(1000,offset=-100)
        for key in ('center_pixel','bbox_px','ground_contact_proxy_Base_mm'):
            cue[key]=candidate_b[key]
        result=self.update(1000,coarse=cue)
        self.assertTrue(result['command'].startswith('FOLLOW,'),result)
        left,right=map(int,result['command'].split(',')[1:])
        self.assertLess(left,right)
        self.assertEqual(self.drive._coarse_reference['point'],
                         cue['visual_evidence']['ground_contact_proxy_Base_mm'])

    def test_initial_unconfirmed_valid_coarse_waits_without_starting_static_fine(self):
        for count in (1,2,3):
            now=900+count*100;cue=self.coarse(now)
            cue['status']='tracking' if count==3 else 'confirming'
            cue['visual_evidence'].update(confirmations=count,navigation_confirmed=count==3,
                association_confirmations=count)
            result=self.update(now,coarse=cue)
            self.assertIsNone(result['request_fine'])
            if count<3:self.assertIsNone(result['command'])
            else:self.assertEqual(result['command'],'FOLLOW,70,70')

    def test_canonical_normalization_preserves_failed_coarse_status(self):
        for status in ('lost','no_target'):
            cue=self.weak_cue(1000)
            cue['status']=status
            self.assertEqual(self.drive._canonical_coarse(cue)['status'],status)

    def test_top_exit_does_not_invent_a_table_distance_or_search_motion(self):
        self.update(1000)
        result=self.update(1100,run=True,coarse=self.missing(1100),
            target_event={'event':'exited','last_disappearance_direction':'top'})
        self.assertEqual(result['command'],'STOP')
        self.assertFalse(self.drive._searching)
        self.assertIsNone(self.update(1200,coarse=self.missing(1200))['command'])

    def test_stable_unmeasurable_cue_ends_search_and_enters_local_inspection(self):
        self.update(1000,coarse=self.missing(1000),target_event={
            'search':True,'last_seen':self.coarse(900)})
        self.update(1050,coarse=self.missing(1050))
        self.assertEqual(self.update(1150,run=True,coarse=self.weak_cue(1150))['command'],'HB')
        result=self.update(1250,run=True,coarse=self.weak_cue(1250,3),
                           target_event={'visible':True,'associated':True})
        self.assertEqual(result['command'],'STOP')
        self.assertEqual(result['phase'],'visual_tracking')
        self.assertEqual(result['status'],'visual_tracking')
        self.assertFalse(self.drive._searching)
        self.assertTrue(self.drive._search_inspecting)
        self.assertIsNone(self.drive.anchor)
        self.assertFalse(result['allow_arm'])
        self.update(1350,coarse=self.weak_cue(1350,4),reason=1)
        result=self.update(1400,coarse=self.weak_cue(1400,5),reason=1)
        self.assertEqual(result['request_fine'],'static')
        self.assertFalse(self.drive._searching)
        self.assertTrue(self.drive._search_inspecting)

    def test_failed_search_inspection_turns_before_rechecking_without_adjust_retreats(self):
        self.update(1000,coarse=self.missing(1000),target_event={
            'search':True,'last_disappearance_direction':'bottom','last_seen':self.coarse(900)})
        self.assertEqual(self.update(1050,coarse=self.missing(1050))['command'],'ARC,-10,-10')
        self.update(1150,travel=-10,reason=8,coarse=self.weak_cue(1150,3))
        result=self.update(1200,travel=-10,reason=8,coarse=self.weak_cue(1200,4))
        self.assertEqual(result['request_fine'],'static')
        failed=self.precise(1200,self.target,status='no_complete_face')
        failed['paired_coarse']=self.weak_cue(1200,4)
        result=self.update(1300,travel=-10,reason=8,
            coarse=self.weak_cue(1300,5),precise=failed)
        self.assertEqual(result['command'],'FOLLOW,-20,20')
        self.assertFalse(self.drive._searching)
        self.assertTrue(self.drive._search_inspecting)
        self.assertEqual(self.drive._active_action,'inspection_follow')
        self.assertEqual(self.drive._adjust_count,0)
        result=self.update(1400,run=True,travel=-12,right=-8,coarse=self.weak_cue(1400,6))
        self.assertEqual(result['command'],'HB')
        result=self.update(11400,run=True,travel=-12,right=-8,coarse=self.weak_cue(11400,7))
        self.assertEqual(result['command'],'HB')
        self.assertTrue(self.drive._search_view_pending)
        # Equal wheel travel contributes no yaw, however long it continues.
        result=self.update(11500,run=True,travel=100,right=104,coarse=self.weak_cue(11500,8))
        self.assertEqual(result['command'],'HB')
        self.assertTrue(self.drive._search_view_pending)
        result=self.update(11600,run=True,travel=91,right=113,coarse=self.weak_cue(11600,9))
        self.assertEqual(result['command'],'STOP')
        self.assertGreaterEqual(self.drive._search_view_yaw_deg,10)
        self.assertFalse(self.drive._search_view_pending)
        result=self.update(11700,travel=91,right=113,reason=1,coarse=self.weak_cue(11700,10))
        self.assertIsNone(result['request_fine'])
        result=self.update(11750,travel=91,right=113,reason=1,coarse=self.weak_cue(11750,11))
        self.assertEqual(result['request_fine'],'static')
        self.assertTrue(self.drive._search_reverse_done)

    def test_confirmed_navigation_can_end_search_while_a_new_view_is_pending(self):
        self.update(1000,coarse=self.missing(1000),target_event={
            'search':True,'last_seen':self.coarse(900)})
        self.update(1050,coarse=self.weak_cue(1050,3))
        failed=self.precise(1050,self.target,status='no_complete_face')
        failed['paired_coarse']=self.weak_cue(1050,3)
        result=self.update(1150,coarse=self.weak_cue(1150,4),precise=failed)
        self.assertEqual(result['command'],'FOLLOW,-20,20')
        self.assertTrue(self.drive._search_view_pending)
        result=self.update(1250,run=True,coarse=self.coarse(1250))
        self.assertEqual(result['command'],'STOP')
        self.assertEqual(result['status'],'reacquired')
        self.assertFalse(self.drive._searching)

    def test_search_version_changes_do_not_renew_the_same_bottom_retreat(self):
        self.update(1000,coarse=self.missing(1000),target_event={
            'search':True,'last_disappearance_direction':'bottom'})
        self.update(1050,coarse=self.missing(1050))
        result=self.update(1150,travel=-10,reason=8,coarse=self.missing(1150),target_version=2,
            target_event={'search':True,'last_disappearance_direction':'bottom'})
        self.assertTrue(self.drive._searching)
        self.assertTrue(self.drive._search_reverse_done)
        result=self.update(1200,travel=-10,reason=8,coarse=self.missing(1200),target_version=2)
        self.assertEqual(result['command'],'FOLLOW,-20,20')

    def test_version_change_during_search_motion_keeps_the_retreat_budget(self):
        self.update(1000,coarse=self.missing(1000),target_event={
            'search':True,'last_disappearance_direction':'bottom'})
        self.update(1050,coarse=self.missing(1050))
        result=self.update(1150,run=True,travel=-1,coarse=self.missing(1150),target_version=2,
            target_event={'event':'exited','last_disappearance_direction':'bottom'})
        self.assertEqual(result['command'],'STOP')
        self.assertTrue(self.drive._search_reverse_done)
        self.update(1250,travel=-1,reason=1,coarse=self.missing(1250),target_version=2)
        result=self.update(1300,travel=-1,reason=1,coarse=self.missing(1300),target_version=2)
        self.assertEqual(result['command'],'FOLLOW,-20,20')

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
            coarse['visual_evidence'].update(center_pixel=coarse['center_pixel'],
                ground_contact_proxy_Base_mm=coarse['ground_contact_proxy_Base_mm'])
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
