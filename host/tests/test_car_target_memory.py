"""Target changes tested against independent world-frame vehicle motion."""
import copy
import math
import unittest

from device.k230.car.target_memory import TargetMemory


class Projection:
    normal = [0., 0., 1.]
    offset = 0.
    forward_axis_Base = [1., 0., 0.]
    motion_origin_Base_mm = [0., 0., 0.]

    def base_to_pixel(self, point):
        return [640+500*point[1]/point[0], 480+20000/point[0]]

    def camera_table_foot(self):
        return [0., 0., 0.]

    def pixel_to_plane(self,pixel,height_mm=0):
        x=20000/(pixel[1]-480)
        return [x,(pixel[0]-640)*x/500,height_mm]


class TargetMemoryTests(unittest.TestCase):
    def setUp(self):
        self.projection = Projection()
        self.config = {'track_width_mm':100, 'tracking_task':{
            'movement_pixel_tolerance_px':12, 'movement_distance_fraction':.10,
            'movement_confirmation_frames':2}}
        self.memory = TargetMemory(self.projection,self.config)
        self.stamp = 0

    def coarse(self, point=(500,30,0), evidence=None):
        self.stamp += 100
        pixel = self.projection.base_to_pixel(point)
        result = {'status':'tracking','mode':'coarse','frame_id':self.stamp,
            'capture_timestamp_ms':self.stamp,'timestamp_ms':self.stamp,
            'center_pixel':pixel,'bbox_px':[int(pixel[0])-10,int(pixel[1])-20,20,20],
            'ground_contact_proxy_Base_mm':list(point)}
        if evidence is not None:
            result['visual_evidence'] = dict(visible=True,associated=True,
                ambiguous=False,candidate_count=1,center_pixel=pixel,
                bbox_px=result['bbox_px'],edge_directions=[],distance_valid=True,
                ground_contact_proxy_Base_mm=list(point),**evidence)
        return result

    def visual(self, point=(500,30,0), **changes):
        result=self.coarse(point)
        evidence=dict(visible=True,associated=True,ambiguous=False,candidate_count=1,
            center_pixel=result['center_pixel'],bbox_px=result['bbox_px'],
            edge_directions=[],distance_valid=True,ground_contact_proxy_Base_mm=list(point))
        evidence.update(changes);result['visual_evidence']=evidence
        return result

    def absent(self):
        self.stamp += 100
        return {'status':'lost','frame_id':self.stamp,
            'capture_timestamp_ms':self.stamp,'visual_evidence':{
                'visible':False,'associated':False,'ambiguous':False,'candidate_count':0}}

    def precise(self, coarse, point=(490,35,10)):
        return {'schema':'aic.vision_observation.v1','mode':'precise','status':'localized',
            'frame_id':coarse['frame_id'],'capture_timestamp_ms':coarse['capture_timestamp_ms'],
            'timestamp_ms':coarse['capture_timestamp_ms'],'clock_domain':'k230_ticks_ms',
            'image_size_px':[1280,960],'coordinate_frame':'Base','units':'mm',
            'reference_point':'cube_geometric_center','xy_compensation_applied':False,
            'cube_center_Base_mm':list(point)}

    def seed(self):
        coarse=self.coarse();self.memory.update(coarse)
        fine=self.precise(coarse)
        self.assertTrue(self.memory.bind_precise(fine,coarse,0))
        return coarse,fine

    def test_coarse_and_fine_reference_points_are_kept_separate(self):
        coarse,fine=self.seed()
        self.assertNotEqual(coarse['ground_contact_proxy_Base_mm'],fine['cube_center_Base_mm'])
        for unused in range(20):
            state=self.memory.update(self.coarse())
            self.assertFalse(state['confirmed_moved'])
            self.assertTrue(state['anchor_valid'])
        self.assertEqual(state['version'],0)

    def test_delayed_bind_rebases_same_frame_coarse_without_resetting_vehicle_motion(self):
        self.seed()
        # A different view changes the coarse proxy. Its own fine pair, not
        # the fine geometric centre, becomes the next comparison baseline.
        capture=self.coarse((500,40,0));self.memory.update(capture)
        current=self.coarse((490,40,0))
        self.memory.update(current,[(10,10)])
        fine=self.precise(capture,(495,45,10))
        # First invalidate was not reached: candidate is within12px here.
        self.assertTrue(self.memory.bind_precise(fine,capture,0,[(10,10)]))
        for unused in range(3):state=self.memory.update(self.coarse((490,40,0)))
        self.assertEqual(state['version'],0)
        # Integer bbox edges can leave subpixel prediction/quantization residue.
        self.assertLess(state['residual_px'],2.)

    def test_bind_requires_original_frame_time_current_version_and_precise_capability(self):
        coarse,fine=self.seed()
        for change in ({'frame_id':999},{'capture_timestamp_ms':None},
                       {'capture_timestamp_ms':999},{'status':'no_complete_face'}):
            altered=dict(fine);altered.update(change)
            self.assertFalse(self.memory.bind_precise(altered,coarse,0))
        self.assertFalse(self.memory.bind_precise(fine,coarse,1))

    def test_vehicle_forward_reverse_and_turn_do_not_look_like_target_motion(self):
        # Independent midpoint world-frame integration, not predict_center.
        world=[500.,30.];x=y=yaw=0.
        self.seed()
        for dl,dr in [(5,5)]*15+[(2,4)]*8+[(-3,-3)]*10+[(3,-2)]*5:
            angle=(dr-dl)/100
            distance=(dl+dr)/2
            # Small midpoint integration error is far below detector tolerance.
            x+=distance*math.cos(yaw+angle/2)
            y+=distance*math.sin(yaw+angle/2);yaw+=angle
            dx,dy=world[0]-x,world[1]-y
            point=[math.cos(yaw)*dx+math.sin(yaw)*dy,
                   -math.sin(yaw)*dx+math.cos(yaw)*dy,0]
            state=self.memory.update(self.coarse(point),[(dl,dr)])
            self.assertFalse(state['confirmed_moved'],state)
            self.assertEqual(state['version'],0)

    def test_current_calibration_axes_and_origin_preserve_stationary_target(self):
        from host.car.runtime import load
        from device.k230.car.continuous_math import table_forward
        unused,config,projection=load()
        config['tracking_task']=self.config['tracking_task']
        config['track_width_mm']=119.2
        self.projection=projection;self.memory=TargetMemory(projection,config)
        n=projection.normal;f=table_forward(n,projection.forward_axis_Base)
        left=[n[1]*f[2]-n[2]*f[1],n[2]*f[0]-n[0]*f[2],n[0]*f[1]-n[1]*f[0]]
        origin=projection.motion_origin_Base_mm
        point=[500.,0.,(projection.offset-n[0]*500)/n[2]]
        q=[sum((point[k]-origin[k])*axis[k] for k in range(3)) for axis in (f,left,n)]
        self.memory.update(self.coarse(point));x=y=yaw=0.
        for dl,dr in [(5,5)]*15+[(2,4)]*8+[(-3,-3)]*10+[(3,-2)]*5:
            angle=(dr-dl)/119.2;distance=(dl+dr)/2
            x+=distance*math.cos(yaw+angle/2)
            y+=distance*math.sin(yaw+angle/2);yaw+=angle
            dx,dy=q[0]-x,q[1]-y
            a=math.cos(yaw)*dx+math.sin(yaw)*dy
            b=-math.sin(yaw)*dx+math.cos(yaw)*dy
            current=[origin[k]+f[k]*a+left[k]*b+n[k]*q[2] for k in range(3)]
            state=self.memory.update(self.coarse(current),[(dl,dr)])
            self.assertEqual(state['version'],0,state)
            self.assertFalse(state['confirmed_moved'])

    def test_motion_confirmation_invalidates_slow_result_and_requires_reacquisition(self):
        coarse,fine=self.seed()
        first=self.memory.update(self.coarse((500,100,0)))
        self.assertEqual(first['event'],'suspected');self.assertEqual(first['version'],0)
        moved=self.memory.update(self.coarse((500,100,0)))
        self.assertEqual(moved['event'],'moved');self.assertTrue(moved['confirmed_moved'])
        self.assertEqual(moved['version'],1);self.assertFalse(moved['anchor_valid'])
        self.assertFalse(self.memory.bind_precise(fine,coarse,0))
        for unused in range(2):state=self.memory.update(self.coarse((500,100,0)))
        self.assertTrue(state['target_valid'])
        # Returning to the first spot does not resurrect its old plan.
        self.memory.update(self.coarse())
        state=self.memory.update(self.coarse())
        self.assertEqual(state['version'],2)
        self.assertFalse(self.memory.bind_precise(fine,coarse,0))

    def test_continuously_moving_target_can_be_confirmed_without_stopping(self):
        self.seed()
        self.memory.update(self.coarse((500,100,0)))
        state=self.memory.update(self.coarse((500,150,0)))
        self.assertTrue(state['confirmed_moved'])
        self.assertEqual(state['version'],1)

    def test_single_frame_jump_and_opposite_noise_do_not_accumulate(self):
        self.seed()
        self.memory.update(self.coarse((500,100,0)))
        state=self.memory.update(self.coarse())
        self.assertEqual(state['version'],0)
        self.memory.update(self.coarse((500,100,0)))
        state=self.memory.update(self.coarse((500,-100,0)))
        self.assertEqual(state['version'],0)

    def test_blue_mask_expansion_does_not_turn_proxy_351_to_267_into_target_motion(self):
        def observation(height,proxy):
            value=self.visual((proxy,0,0),bbox_px=[600,600,80,height],
                              center_pixel=[640,600+height/2])
            value['bbox_px']=[600,600,80,height]
            value['center_pixel']=[640,600+height/2]
            return value
        initial=observation(66,351);self.memory.update(initial)
        self.memory.bind_precise(self.precise(initial,(275,0,10)),initial,0)
        changed=self.memory.update(observation(238,267))
        # This mask grows beyond native identity continuity. It is unknown,
        # rather than a new distance or independent target-motion evidence.
        self.assertEqual(changed['visibility'],'unknown')
        self.assertTrue(changed['anchor_valid'])
        self.assertFalse(changed['confirmed_moved'])
        for unused in range(15):state=self.memory.update(observation(238,267))
        self.assertEqual(state['version'],0)
        self.assertTrue(state['target_valid'])

    def test_approach_turn_and_small_encoder_error_do_not_accumulate_proxy_bias(self):
        world=[600.,30.];x=y=yaw=0.
        def observation(point,bias):
            value=self.visual(point)
            u,v=value['center_pixel'];size=12000/point[0]
            box=[u-size/2,v-size/2,size,size]
            value['bbox_px']=box;value['visual_evidence']['bbox_px']=box
            proxy=[point[0]+bias,point[1],0]
            value['ground_contact_proxy_Base_mm']=proxy
            value['visual_evidence']['ground_contact_proxy_Base_mm']=proxy
            return value
        self.memory.update(observation([600,30,0],76))
        commands=[(7,7)]*20+[(5,6)]*6+[(6,5)]*6+[(7,7)]*8
        for index,(dl,dr) in enumerate(commands):
            angle=(dr-dl)/100;distance=(dl+dr)/2
            x+=distance*math.cos(yaw+angle/2)
            y+=distance*math.sin(yaw+angle/2);yaw+=angle
            dx,dy=world[0]-x,world[1]-y
            point=[math.cos(yaw)*dx+math.sin(yaw)*dy,
                   -math.sin(yaw)*dx+math.cos(yaw)*dy,0]
            state=self.memory.update(observation(point,76*(1-(index+1)/len(commands))),
                                     [(dl*1.01,dr*1.01)])
            self.assertEqual(state['version'],0,state)
            self.assertNotEqual(state['movement_state'],'suspected')

    def test_arc_wheel_model_moving65px_with_nearly_fixed_image_is_not_target_motion(self):
        self.seed()
        # Real run7466: an ARC's nominal axle model moved the expected box
        # about65px while the observed blue box barely moved. Wheel travel is
        # real feedback, not proof of the body pose or external target motion.
        for unused in range(8):
            state=self.memory.update(self.coarse(),[(0,13)],motion_state={'run':1})
            self.assertEqual(state['version'],0)
            self.assertEqual(state['movement_state'],'consistent')
            self.assertTrue(state['motion_model_uncertain'])
            self.assertFalse(state['needs_precise'])
            self.assertTrue(state['anchor_valid'])
        # Do not carry the preceding ego-model residual into the first
        # stationary frames and then interrupt the final fine verification.
        for unused in range(4):
            state=self.memory.update(self.coarse(),motion_state={'run':0})
            self.assertEqual(state['version'],0)
            self.assertFalse(state['motion_model_uncertain'])
            self.assertFalse(state['needs_precise'])

    def test_field_arc_counts_and_box_expose_model_error_in_current_calibration(self):
        from tools.deploy.k230_car_align import build_bundle
        from device.k230.car.ground_projection import GroundProjection
        config=build_bundle(near_parking=True)[0];p=config['projection']
        self.projection=GroundProjection(p['K'],p['D'],p['T'],p['plane'],
            p['forward_axis_Base'],p['motion_origin_Base_mm'])
        self.memory=TargetMemory(self.projection,config)
        # 7466 ARC: a42421->42947, b-38066->-38074; observed box
        # [581,565,182,289] barely changed. The calibrated wheel model predicts
        # 63.6px. This ray supplies a coarse proxy, not measured fine truth.
        box=[581,565,182,289];pixel=[672,709.5]
        def observation():
            value=self.visual(self.projection.pixel_to_plane(pixel,0),
                              bbox_px=box,center_pixel=pixel)
            value.update(bbox_px=box,center_pixel=pixel)
            return value
        initial=observation();self.memory.update(initial)
        self.memory.bind_precise(self.precise(initial,
            self.projection.pixel_to_plane(pixel,10)),initial,0)
        scale=config['mm_per_count']
        first=self.memory.update(observation(),[(526*scale,8*scale)],motion_state={'run':1})
        self.assertGreater(first['residual_px'],60)
        self.assertTrue(first['motion_model_uncertain'])
        second=self.memory.update(observation(),motion_state={'run':1})
        self.assertEqual(second['version'],0)
        self.assertFalse(second['needs_precise'])
        self.assertEqual(second['movement_state'],'consistent')

    def test_wheel_increments_still_mean_motion_when_status_has_just_stopped(self):
        self.seed()
        for unused in range(3):
            state=self.memory.update(self.coarse(),[(0,13)],motion_state={'run':0})
            self.assertEqual(state['version'],0)
            self.assertTrue(state['motion_model_uncertain'])
            self.assertNotEqual(state['movement_state'],'suspected')

    def test_commanded_motion_does_not_need_encoder_counts_to_enter_motion_policy(self):
        self.seed()
        # A stalled/quantized/weak drive may report no increment. The car is
        # still in its motion phase. Unchanged pixels with no encoder motion
        # must not invent a target displacement or a suspected-motion pause.
        for index in range(1,10):
            state=self.memory.update(self.coarse(),
                                     motion_state={'run':1})
            self.assertEqual(state['version'],0)
            self.assertEqual(state['movement_state'],'consistent')
            self.assertFalse(state['needs_precise'])
        # The mandatory stopped fine pose, rather than an ideal wheel model,
        # resolves a possible slow displacement in the field of view.
        current=self.coarse((500,66,0))
        self.memory.update(current,motion_state={'run':0})
        fine=self.precise(current,(490,71,10))
        self.assertTrue(self.memory.bind_precise(fine,current,0))
        self.assertEqual(self.memory.last_precise['cube_center_Base_mm'],[490,71,10])

    def test_observed_image_jump_during_motion_still_invalidates_inflight_fine(self):
        # An in-flight result is not yet an accepted travel anchor.
        coarse=self.coarse();self.memory.update(coarse);fine=self.precise(coarse)
        raw=self.visual((500,330,0),associated=False,reason='unassociated_blue')
        before=copy.deepcopy(raw)
        first=self.memory.update(raw,motion_state={'run':1})
        self.assertEqual(first['event'],'tracking_update')
        self.assertEqual(first['movement_state'],'consistent')
        self.assertFalse(first['confirmed_moved'])
        self.assertEqual(first['version'],1)
        self.assertTrue(first['target_valid'])
        self.assertTrue(first['associated'])
        self.assertFalse(first['anchor_valid'])
        self.assertEqual(raw,before)
        self.assertFalse(first['navigation_cue']['associated'])
        self.assertEqual(self.memory._query_visual['center_pixel'],raw['visual_evidence']['center_pixel'])
        self.assertFalse(self.memory.bind_precise(fine,coarse,0))
        second=self.memory.update(self.visual((500,340,0),associated=False,
            reason='unassociated_blue'),motion_state={'run':1})
        self.assertFalse(second['confirmed_moved'])
        self.assertEqual(second['version'],1)
        self.assertFalse(self.memory.bind_precise(fine,coarse,0))

    def test_moving_visual_update_is_not_later_reported_as_a_stationary_exception(self):
        self.memory.update(self.coarse())  # No accepted fine distance yet.
        self.memory.update(self.visual((500,330,0)),motion_state={'run':1})
        state=self.memory.update(self.visual((500,340,0)),motion_state={'run':0})
        self.assertEqual(state['version'],1)
        self.assertEqual(state['reason'],'tracking_update')
        self.assertFalse(state['confirmed_moved'])

    def test_moving_jump_and_return_update_both_positions_without_restoring_old_fine(self):
        coarse=self.coarse();self.memory.update(coarse);fine=self.precise(coarse)
        first=self.memory.update(self.visual((500,330,0)),motion_state={'run':1})
        self.assertEqual(first['version'],1)
        for unused in range(4):
            state=self.memory.update(self.coarse(),motion_state={'run':1})
            self.assertEqual(state['version'],2)
            self.assertTrue(state['needs_precise'])
            self.assertFalse(state['confirmed_moved'])
        self.assertEqual(state['movement_state'],'consistent')
        self.assertFalse(self.memory.bind_precise(fine,coarse,0))

    def test_moving_colour_region_growth_is_not_an_independent_position_jump(self):
        self.seed()
        for unused in range(3):
            state=self.memory.update(self.visual((500,330,0),
                bbox_px=[890,490,160,60]),motion_state={'run':1})
            self.assertEqual(state['version'],0)
            self.assertNotEqual(state['event'],'suspected')
            self.assertTrue(state['anchor_valid'])
            self.assertEqual(state['visibility'],'unknown')

    def test_unassociated_large_mask_expansion_without_common_translation_keeps_fine(self):
        unused,fine=self.seed()
        expanded=self.visual((500,330,0),associated=False,
            bbox_px=[660,440,360,120],center_pixel=[970,520],
            reason='unassociated_blue')
        state=self.memory.update(expanded,motion_state={'run':1})
        self.assertEqual(state['version'],0)
        self.assertFalse(state['tracking_update'])
        self.assertTrue(state['anchor_valid'])
        self.assertEqual(state['last_precise'],fine)

    def test_unknown_and_shape_discontinuity_do_not_confirm_a_moving_visual_jump(self):
        self.memory.update(self.coarse())  # Exercise unanchored following.
        self.memory.update(self.visual((500,330,0)),motion_state={'run':1})
        self.memory.update(self.absent(),motion_state={'run':1})
        state=self.memory.update(self.visual((500,330,0)),motion_state={'run':1})
        self.assertEqual(state['version'],1)
        # The other side of a gap has no same-frame identity proof; a large
        # aspect/size change alone is not a confirmed new object position.
        state=self.memory.update(self.visual((500,-330,0),bbox_px=[250,400,4,4]),
                                 motion_state={'run':1})
        self.assertEqual(state['version'],1)

    def test_passive_tiny_wheel_counts_do_not_enable_running_retarget_policy(self):
        self.seed()
        state=self.memory.update(self.visual((500,200,0)),[(.01,.01)],motion_state={'run':0})
        self.assertEqual(state['version'],0)
        self.assertEqual(state['movement_state'],'suspected')
        self.assertFalse(state['tracking_update'])
        self.assertIsNone(state['navigation_cue'])
        for unused in range(3):
            state=self.memory.update(self.visual((500,200,0)),[],motion_state={'run':0})
        self.assertEqual(state['version'],1)
        self.assertFalse(state['anchor_valid'])
        self.assertTrue(state['needs_precise'])
        self.assertFalse(state['tracking_update'])

    def test_large_encoder_explained_image_turn_does_not_revoke_completed_fine(self):
        for run in (0,1):
            with self.subTest(run=run):
                self.setUp()
                original,fine=self.seed()
                deltas=[(0,60)]
                current=self.memory._propagate(self.memory._motion_reference,deltas)
                raw=self.visual(current,associated=False,reason='unassociated_blue')
                self.assertGreater(abs(raw['center_pixel'][0]-original['center_pixel'][0]),
                                   1280*.15)
                state=self.memory.update(raw,deltas,motion_state={'run':run})
                self.assertEqual(state['version'],0)
                self.assertFalse(state['tracking_update'])
                self.assertTrue(state['anchor_valid'])
                self.assertEqual(state['last_precise'],fine)

    def test_moving_target_explicitly_leaving_view_still_invalidates(self):
        coarse,fine=self.seed()
        # Same target moves across the image before its left side is clipped;
        # an arbitrary one-frame weak cue at the far edge is not identity.
        for x in range(600,0,-100):
            self.memory.update(self.visual(distance_valid=False,
                center_pixel=[x,520],bbox_px=[x-10,510,20,20]),motion_state={'run':1})
        self.memory.update(self.visual(distance_valid=False,center_pixel=[6,520],
            bbox_px=[0,510,12,20],edge_directions=['left']),motion_state={'run':1})
        state=self.memory.update(self.absent(),motion_state={'run':1})
        self.assertEqual(state['event'],'exited')
        # The visible same-target translation has already revoked its anchor;
        # the later departure may begin a subsequent unanchored epoch.
        self.assertGreaterEqual(state['version'],1)
        self.assertFalse(self.memory.bind_precise(fine,coarse,0))

    def test_observable_slow_whole_box_translation_accumulates_in_short_window(self):
        self.seed()
        moved=False
        for index in range(1,9):
            state=self.memory.update(self.coarse((500,30+3*index,0)))
            moved=moved or state['confirmed_moved']
        self.assertTrue(moved)
        self.assertGreaterEqual(self.memory.version,1)

    def test_depth_change_is_observed_as_well_as_direction(self):
        # Perspective-consistent box growth plus vertical translation, not
        # just a coarse proxy number changing under an unchanged small box.
        def observation(x):
            value=self.coarse((x,.06*x,0))
            u,v=value['center_pixel'];size=10000/x
            value['bbox_px']=[u-size/2,v-size/2,size,size]
            return value
        self.memory.update(observation(500))
        self.memory.update(observation(300))
        state=self.memory.update(observation(300))
        self.assertTrue(state['confirmed_moved'])
        self.assertGreater(state['residual_px'],12)

    def test_occlusion_defers_authority_until_consistent_same_version_reappearance(self):
        coarse,fine=self.seed()
        state=self.memory.update(self.absent())
        self.assertFalse(state['confirmed_moved']);self.assertEqual(state['version'],0)
        self.assertTrue(state['anchor_valid']);self.assertTrue(state['needs_precise'])
        self.assertTrue(self.memory.bind_precise(fine,coarse,0))
        reappeared=self.coarse();state=self.memory.update(reappeared)
        self.assertEqual(state['event'],'reappeared');self.assertFalse(state['needs_precise'])
        self.memory.bind_precise(fine,coarse,0)
        self.assertFalse(self.memory.snapshot()['needs_precise'])
        self.memory.bind_precise(self.precise(reappeared),reappeared,0)
        self.assertFalse(self.memory.snapshot()['needs_precise'])

    def test_unanchored_edge_departure_is_distinct_from_a_central_occlusion(self):
        # A coarse-valid target can already be near the edge. Its continuous
        # weak clipped remainder then supports an explicit departure.
        near=self.visual(center_pixel=[20,520],bbox_px=[10,510,20,20])
        self.memory.update(near)
        self.memory.update(self.visual(distance_valid=False,
            center_pixel=[6,520],bbox_px=[0,510,12,20],edge_directions=['left']))
        state=self.memory.update(self.absent())
        self.assertEqual(state['event'],'exited');self.assertEqual(state['version'],1)
        self.assertFalse(state['anchor_valid'])
        self.assertEqual(state['last_disappearance_direction'],'left')
        for unused in range(20):state=self.memory.update(self.absent())
        self.assertEqual(state['version'],1)

    def test_recovery_repeated_invalidation_is_latched_until_fresh_reacquisition(self):
        self.seed()
        self.memory.invalidate('empty_grip')
        for unused in range(30):self.memory.invalidate('empty_grip')
        self.assertEqual(self.memory.version,1)
        self.assertFalse(self.memory.snapshot()['anchor_valid'])
        for unused in range(2):state=self.memory.update(self.coarse())
        self.assertTrue(state['target_valid'])
        self.memory.invalidate('another_real_recovery')
        self.assertEqual(self.memory.version,2)

    def test_new_epoch_full_query_can_seed_weak_identity_without_reviving_old_fine(self):
        old_coarse,old_fine=self.seed()
        self.memory.invalidate('target_changed')
        # Supplying the new version cannot re-label a pre-change photo.
        self.assertFalse(self.memory.bind_precise(old_fine,old_coarse,1))
        current=self.visual((500,230,0),distance_valid=False,
            reason='inconsistent_with_20mm_height',association_confirmations=3)
        state=self.memory.update(current)
        self.assertTrue(state['precise_candidate'])
        self.assertFalse(state['associated'])
        self.assertFalse(state['target_valid'])
        fine=self.precise(current)
        self.assertFalse(self.memory.bind_precise(fine,current,1))
        fine['target_association']={'status':'matched'}
        self.assertFalse(self.memory.bind_precise(fine,current,0))
        range_only=dict(fine,status='no_complete_face')
        self.assertFalse(self.memory.bind_precise(range_only,current,1))
        self.assertTrue(self.memory.bind_precise(fine,current,1))
        state=self.memory.snapshot()
        self.assertTrue(state['anchor_valid'])
        self.assertTrue(state['target_valid'])
        self.assertFalse(state['associated'])
        self.assertTrue(state['needs_precise'])
        # Only a subsequent current image can resume target identity.
        state=self.memory.update(self.visual((500,230,0),distance_valid=False,
            reason='inconsistent_with_20mm_height',association_confirmations=4))
        self.assertTrue(state['associated'])
        self.assertFalse(state['needs_precise'])
        self.assertEqual(state['version'],1)
        self.assertFalse(self.memory.bind_precise(old_fine,old_coarse,1))

    def test_recovery_latch_cannot_clear_and_retrigger_during_ready_delay(self):
        self.seed();self.memory.invalidate('empty_grip')
        for unused in range(10):
            state=self.memory.update(self.coarse(),arm_state={'status':'recovering','phase':'recovery'})
        self.assertFalse(state['target_valid']);self.assertEqual(state['version'],1)
        for unused in range(2):
            state=self.memory.update(self.coarse((500,100,0)),
                                     arm_state={'status':'recovering','phase':'recovery'})
        self.assertFalse(state['confirmed_moved']);self.assertEqual(state['version'],1)
        for unused in range(2):
            state=self.memory.update(self.coarse((500,100,0)),
                                     arm_state={'status':'reobserve_ready','phase':'recovery'})
        self.assertTrue(state['target_valid'])

    def test_holding_phase_does_not_misread_normal_lift_or_self_occlusion(self):
        self.seed()
        for point in ((400,100,0),(200,-300,0),(700,200,0)):
            state=self.memory.update(self.visual(point,distance_valid=False),
                arm_state={'phase':'lift','grasp_confirmed':True})
            self.assertEqual(state['visibility'],'holding')
            self.assertFalse(state['table_distance_valid'])
            self.assertFalse(state['confirmed_moved'])
        state=self.memory.update(self.absent(),arm_state={'phase':'lift','grasp_confirmed':None})
        self.assertEqual(state['version'],0)
        self.assertFalse(state['confirmed_moved'])

    def test_confirmed_close_pending_lift_is_already_holding_without_fresh_fk(self):
        self.seed()
        arm={'phase':'close','pending_stage':'lift','grasp_confirmed':True}
        for point in ((500,60,0),(500,70,0)):
            state=self.memory.update(self.visual(point),arm_state=arm)
            self.assertEqual(state['event'],'holding')
            self.assertFalse(state['table_distance_valid'])
            self.assertEqual(state['version'],0)
        state=self.memory.update(self.absent(),arm_state=arm)
        self.assertEqual(state['version'],0)

    def test_unconfirmed_close_can_detect_target_outside_contact_region(self):
        self.seed()
        arm={'phase':'close','pending_stage':None,'grasp_confirmed':None}
        for unused in range(2):state=self.memory.update(self.visual((500,150,0)),arm_state=arm)
        self.assertEqual(state['event'],'moved')
        self.assertEqual(state['version'],1)
        self.assertIsNone(arm['grasp_confirmed'])

    def test_real_close_frame_sequence_is_contact_not_tabletop_target_motion(self):
        # 7466: fixed target, close at182023, these two frames at182791/182895
        # used to invalidate version34 before the five grip samples completed.
        def observation(box,center):
            value=self.visual(distance_valid=False,bbox_px=box,center_pixel=center)
            value.update(bbox_px=box,center_pixel=center)
            return value
        initial=observation([523,546,177,274],[607,682])
        self.memory.update(initial)
        self.memory.bind_precise(self.precise(initial),initial,0)
        arm={'phase':'close','pending_stage':None,'grasp_confirmed':None}
        for box,center in (([568,552,179,261],[657,682]),
                           ([583,551,178,262],[672,682])):
            state=self.memory.update(observation(box,center),arm_state=arm)
            self.assertEqual(state['event'],'holding')
            self.assertEqual(state['version'],0)
            self.assertTrue(state['anchor_valid'])
        self.assertIsNone(arm['grasp_confirmed'])
        state=self.memory.update(self.absent(),arm_state=arm)
        self.assertEqual(state['version'],0)

    def test_close_unknown_and_ambiguous_frames_break_external_confirmation(self):
        self.seed();arm={'phase':'close','grasp_confirmed':None}
        first=self.memory.update(self.visual((500,150,0)),arm_state=arm)
        self.assertEqual(first['event'],'suspected')
        self.memory.update(self.absent(),arm_state=arm)
        self.memory.update(self.visual((500,150,0)),arm_state=arm)
        state=self.memory.update(self.visual((500,150,0),associated=False,
            ambiguous=True,candidate_count=2),arm_state=arm)
        self.assertEqual(state['version'],0)
        state=self.memory.update(self.visual((500,150,0)),arm_state=arm)
        self.assertEqual(state['event'],'suspected')
        state=self.memory.update(self.visual((500,150,0)),arm_state=arm)
        self.assertEqual(state['version'],1)

    def test_close_can_use_explicit_contact_reference_without_claiming_grip(self):
        self.seed()
        arm={'phase':'close','grasp_confirmed':None,'contact_center_pixel':[770,520]}
        for unused in range(3):
            state=self.memory.update(self.visual((500,130,0)),arm_state=arm)
        self.assertEqual(state['version'],0)
        self.assertEqual(state['event'],'holding')
        self.assertIsNone(arm['grasp_confirmed'])

    def test_fresh_held_region_follows_normal_lift_and_short_occlusion(self):
        self.seed()
        for point in ((400,100,0),(200,-100,0),(700,200,0)):
            observation=self.visual(point,distance_valid=False)
            state=self.memory.update(observation,arm_state={'phase':'lift',
                'grasp_confirmed':True,'held_center_pixel':observation['center_pixel']})
            self.assertEqual(state['version'],0)
            self.assertFalse(state['confirmed_moved'])
        self.memory.update(self.absent(),arm_state={'phase':'lift',
            'grasp_confirmed':True,'held_center_pixel':[640,500]})
        state=self.memory.update(self.visual((500,30,0),distance_valid=False),
            arm_state={'phase':'lift','grasp_confirmed':True,'held_center_pixel':[670,520]})
        self.assertEqual(state['version'],0)

    def test_two_associated_frames_far_from_trusted_held_region_invalidate(self):
        self.seed()
        arm={'phase':'lift','grasp_confirmed':True,'held_center_pixel':[670,520]}
        first=self.memory.update(self.visual((500,230,0),distance_valid=False),arm_state=arm)
        self.assertEqual(first['event'],'suspected');self.assertFalse(first['confirmed_moved'])
        second=self.memory.update(self.visual((500,240,0),distance_valid=False),arm_state=arm)
        self.assertTrue(second['confirmed_moved']);self.assertEqual(second['version'],1)
        self.assertEqual(second['reason'],'held_target_moved');self.assertFalse(second['anchor_valid'])
        for unused in range(20):
            state=self.memory.update(self.visual((500,240,0),distance_valid=False),arm_state=arm)
        self.assertEqual(state['version'],1);self.assertFalse(state['confirmed_moved'])

    def test_holding_external_motion_requires_trusted_region(self):
        self.seed()
        for unused in range(3):
            state=self.memory.update(self.visual((500,230,0),distance_valid=False),
                arm_state={'phase':'lift','grasp_confirmed':True})
        self.assertEqual(state['version'],0)
        for unused in range(2):
            state=self.memory.update(self.visual((500,230,0),distance_valid=False,
                associated=False,reason='unassociated_blue'),arm_state={'phase':'lift',
                'grasp_confirmed':True,'held_center_pixel':[670,520]})
        self.assertEqual(state['version'],1)
        self.assertTrue(state['confirmed_moved']);self.assertTrue(state['associated'])

    def test_single_unassociated_held_cue_does_not_prove_external_motion(self):
        self.seed();arm={'phase':'lift','grasp_confirmed':True,'held_center_pixel':[670,520]}
        state=self.memory.update(self.visual((500,230,0),distance_valid=False,
            associated=False,reason='unassociated_blue'),arm_state=arm)
        self.assertEqual(state['event'],'suspected');self.assertEqual(state['version'],0)
        state=self.memory.update(self.absent(),arm_state=arm)
        self.assertEqual(state['event'],'holding');self.assertEqual(state['version'],0)
        state=self.memory.update(self.visual((500,230,0),distance_valid=False,
            associated=False,reason='unassociated_blue'),arm_state=arm)
        self.assertEqual(state['version'],0)

    def test_held_external_candidate_requires_consistent_shape_and_size(self):
        self.seed();arm={'phase':'lift','grasp_confirmed':True,'held_center_pixel':[670,520]}
        self.memory.update(self.visual((500,230,0),distance_valid=False,
            associated=False,reason='unassociated_blue'),arm_state=arm)
        state=self.memory.update(self.visual((500,230,0),distance_valid=False,
            bbox_px=[800,400,3,3],associated=False,reason='unassociated_blue'),arm_state=arm)
        self.assertEqual(state['version'],0)
        state=self.memory.update(self.visual((500,230,0),distance_valid=False,
            bbox_px=[800,400,3,200],associated=False,reason='implausible_aspect_ratio'),arm_state=arm)
        self.assertEqual(state['event'],'holding');self.assertEqual(state['version'],0)

    def test_held_ambiguous_cues_break_external_motion_confirmation(self):
        self.seed();arm={'phase':'lift','grasp_confirmed':True,'held_center_pixel':[670,520]}
        self.memory.update(self.visual((500,230,0),distance_valid=False,
            associated=False,reason='unassociated_blue'),arm_state=arm)
        state=self.memory.update(self.visual((500,230,0),distance_valid=False,
            associated=False,ambiguous=True,candidate_count=2),arm_state=arm)
        self.assertEqual(state['event'],'holding');self.assertEqual(state['version'],0)
        state=self.memory.update(self.visual((500,230,0),distance_valid=False,
            associated=False,reason='unassociated_blue'),arm_state=arm)
        self.assertEqual(state['version'],0)

    def test_held_region_tolerance_scales_with_large_near_blob(self):
        initial=self.visual(bbox_px=[620,470,100,100])
        self.memory.update(initial)
        self.memory.bind_precise(self.precise(initial),initial,0)
        arm={'phase':'lift','grasp_confirmed':True,'held_center_pixel':[670,520]}
        for unused in range(3):
            state=self.memory.update(self.visual((500,130,0),distance_valid=False,
                bbox_px=[650,400,100,100]),arm_state=arm)
        self.assertEqual(state['version'],0)

    def test_partial_visible_corner_does_not_shrink_original_contact_region(self):
        initial=self.visual(bbox_px=[551,401,238,238])
        self.memory.update(initial)
        self.memory.bind_precise(self.precise(initial),initial,0)
        # A20px patch100px from the FK centre still lies within the original
        # complete cube. Its own width cannot replace the238px reference.
        for phase in ('close','lift'):
            arm={'phase':phase,'grasp_confirmed':phase=='lift',
                 'held_center_pixel':[670,520]}
            for unused in range(3):
                state=self.memory.update(self.visual((500,130,0),distance_valid=False,
                    center_pixel=[770,520],bbox_px=[760,510,20,20]),arm_state=arm)
                self.assertEqual(state['version'],0)
                self.assertEqual(state['event'],'holding')
        # Taking the cube beyond that frozen region is still observable.
        for unused in range(2):
            state=self.memory.update(self.visual(distance_valid=False,
                associated=False,reason='unassociated_blue',center_pixel=[1170,520],
                bbox_px=[1160,510,20,20]),arm_state=arm)
        self.assertEqual(state['version'],1)
        self.assertEqual(state['reason'],'held_target_moved')
        for unused in range(5):
            state=self.memory.update(self.absent(),arm_state=arm)
        self.assertEqual(state['version'],1)

    def test_external_candidate_cannot_expand_the_frozen_contact_region(self):
        self.seed();arm={'phase':'close','grasp_confirmed':None}
        for unused in range(2):
            state=self.memory.update(self.visual((500,180,0),bbox_px=[700,450,100,100]),
                                     arm_state=arm)
        self.assertEqual(state['version'],1)

    def test_contact_region_is_rebuilt_for_the_next_attempt(self):
        initial=self.visual(bbox_px=[551,401,238,238])
        self.memory.update(initial)
        self.memory.bind_precise(self.precise(initial),initial,0)
        arm={'phase':'close','grasp_confirmed':None}
        self.memory.update(self.visual(),arm_state=arm)
        self.memory.update(self.visual(),arm_state={'phase':'recovery'})
        next_target=self.visual()
        self.memory.update(next_target)
        self.memory.bind_precise(self.precise(next_target),next_target,0)
        for unused in range(2):state=self.memory.update(self.visual((500,150,0)),arm_state=arm)
        self.assertEqual(state['version'],1)

    def test_visible_but_unusable_range_does_not_become_absent(self):
        state=self.memory.update(self.visual(distance_valid=False,
            reason='inconsistent_with_20mm_height'))
        self.assertTrue(state['visible']);self.assertEqual(state['visibility'],'unknown')
        self.assertFalse(state['associated'])
        self.assertFalse(state['table_distance_valid'])
        self.assertFalse(state['confirmed_moved'])
        self.assertTrue(state['needs_precise'])

    def test_full_fine_rescues_visible_coarse_geometry_without_inventing_range(self):
        coarse=self.visual(distance_valid=False,reason='inconsistent_with_20mm_height')
        self.memory.update(coarse)
        self.assertTrue(self.memory.bind_precise(self.precise(coarse),coarse,0))
        state=self.memory.snapshot()
        self.assertTrue(state['anchor_valid'])
        self.assertFalse(state['associated'])
        self.assertTrue(state['needs_precise'])
        for unused in range(5):
            state=self.memory.update(self.visual(distance_valid=False,
                reason='inconsistent_with_20mm_height'))
            self.assertFalse(state['needs_precise'])
            self.assertFalse(state['table_distance_valid'])
            self.assertEqual(state['movement_state'],'consistent')
            self.assertEqual(state['version'],0)

    def test_rescued_centroid_ray_is_encoder_propagated_in_actual_calibration(self):
        from host.car.runtime import load
        from device.k230.car.continuous_math import table_forward
        unused,config,projection=load()
        config['tracking_task']=self.config['tracking_task'];config['track_width_mm']=119.2
        self.projection=projection;self.memory=TargetMemory(projection,config)
        n=projection.normal;f=table_forward(n,projection.forward_axis_Base)
        center=[500.,0.,(projection.offset+10-n[0]*500)/n[2]]
        coarse=self.visual(center,distance_valid=False)
        self.memory.update(coarse)
        self.memory.bind_precise(self.precise(coarse,center),coarse,0)
        for index in range(1,16):
            current=[center[k]-f[k]*index*10 for k in range(3)]
            state=self.memory.update(self.visual(current,distance_valid=False),[(10,10)])
            self.assertFalse(state['needs_precise'],state)
            self.assertEqual(state['version'],0)
            self.assertFalse(state['table_distance_valid'])

    def test_range_only_cannot_clear_unverified_coarse_geometry(self):
        coarse=self.visual(distance_valid=False,reason='inconsistent_with_20mm_height')
        self.memory.update(coarse)
        fine=self.precise(coarse);point=fine.pop('cube_center_Base_mm')
        fine['status']='no_complete_face'
        fine['range_candidate']={'schema':'aic.car.side_range.v1','status':'range_candidate',
            'coordinate_frame':'Base','units':'mm','raw_center_Base_mm':point,
            'Base_X_mm':point[0],'not_arm_or_parking_authority':True}
        self.assertTrue(self.memory.bind_precise(fine,coarse,0))
        self.assertTrue(self.memory.snapshot()['needs_precise'])

    def test_rescued_geometry_can_revalidate_same_pose_after_unknown_gap(self):
        coarse=self.visual(distance_valid=False)
        self.memory.update(coarse)
        fine=self.precise(coarse);self.memory.bind_precise(fine,coarse,0)
        self.memory.update(self.absent())
        current=self.visual(distance_valid=False)
        state=self.memory.update(current)
        self.assertFalse(state['needs_precise'])
        self.assertTrue(self.memory.bind_precise(fine,coarse,0,[]))
        self.assertFalse(self.memory.snapshot()['needs_precise'])
        self.assertTrue(self.memory.bind_precise(self.precise(current),current,0))
        self.assertFalse(self.memory.snapshot()['needs_precise'])

    def test_rescued_geometry_gap_ends_on_first_visible_valid_range_frame(self):
        coarse=self.visual(distance_valid=False);self.memory.update(coarse)
        self.memory.bind_precise(self.precise(coarse),coarse,0)
        self.memory.update(self.absent())
        reappeared=self.visual()
        state=self.memory.update(reappeared)
        self.assertEqual(state['event'],'reappeared')
        self.memory.update(self.visual())
        self.memory.bind_precise(self.precise(reappeared),reappeared,0,[])
        self.assertFalse(self.memory.snapshot()['needs_precise'])

    def test_rescued_image_reference_detects_movement_with_no_coarse_distance(self):
        coarse=self.visual(distance_valid=False)
        self.memory.update(coarse);self.memory.bind_precise(self.precise(coarse),coarse,0)
        first=self.memory.update(self.visual((500,100,0),distance_valid=False))
        self.assertEqual(first['event'],'suspected');self.assertTrue(first['needs_precise'])
        second=self.memory.update(self.visual((500,100,0),distance_valid=False))
        self.assertTrue(second['confirmed_moved']);self.assertEqual(second['version'],1)
        for unused in range(2):state=self.memory.update(self.visual((500,100,0),distance_valid=False))
        self.assertTrue(state['target_valid']);self.assertTrue(state['needs_precise'])

    def test_slow_fine_does_not_clear_geometry_doubt_after_current_pixel_changed(self):
        coarse=self.visual(distance_valid=False);self.memory.update(coarse)
        self.memory.update(self.visual((500,100,0),distance_valid=False))
        self.assertFalse(self.memory.bind_precise(self.precise(coarse),coarse,0,[]))
        self.assertIsNone(self.memory.last_precise)
        self.assertTrue(self.memory.snapshot()['needs_precise'])

    def test_late_full_query_cannot_seed_a_weak_target_that_moved_during_the_solve(self):
        for recovering in (False,True):
            with self.subTest(recovering=recovering):
                self.setUp()
                if recovering:
                    self.seed();self.memory.invalidate('target_changed')
                original_fine=self.memory.last_precise
                source=self.visual((500,230,0),distance_valid=False,
                    association_confirmations=3)
                state=self.memory.update(source)
                self.assertTrue(state['precise_candidate'])
                self.assertFalse(state['associated'])
                # Native identity remains unique and continuous, but this
                # query-only colour is not part of target motion history.
                state=self.memory.update(self.visual((500,270,0),distance_valid=False,
                    association_confirmations=4))
                self.assertTrue(state['precise_candidate'])
                self.assertFalse(state['associated'])
                fine=self.precise(source);fine['target_association']={'status':'matched'}
                self.assertFalse(self.memory.bind_precise(fine,source,int(recovering),[]))
                state=self.memory.snapshot()
                self.assertEqual(state['version'],int(recovering))
                self.assertEqual(state['last_precise'],original_fine)
                self.assertEqual(state['target_valid'],not recovering)
                self.assertFalse(state['anchor_valid'])
                self.assertTrue(state['needs_precise'])

    def test_late_weak_full_query_uses_current_input_and_encoder_rebased_native_ray(self):
        for change in ('stationary','mask_expansion','encoder_motion'):
            with self.subTest(change=change):
                self.setUp()
                source=self.visual(distance_valid=False,association_confirmations=3)
                self.memory.update(source)
                deltas=[(250,250)] if change=='encoder_motion' else []
                point=(250,30,0) if deltas else (500,30,0)
                current=self.visual(point,distance_valid=False,association_confirmations=4)
                if change=='mask_expansion':
                    current['visual_evidence']['bbox_px']=[640,480,60,60]
                state=self.memory.update(current,deltas)
                self.assertFalse(state['associated'])
                self.assertIsNone(state['last_seen'])
                fine=self.precise(source);fine['target_association']={'status':'matched'}
                if deltas:
                    # The same image change would be stale target geometry
                    # without capture-to-current-image wheel evidence.
                    self.assertFalse(self.memory.bind_precise(fine,source,0,[]))
                self.assertTrue(self.memory.bind_precise(fine,source,0,deltas))
                self.assertFalse(self.memory.snapshot()['associated'])
                state=self.memory.update(dict(current,frame_id=self.stamp+100,
                    capture_timestamp_ms=self.stamp+100,timestamp_ms=self.stamp+100))
                self.assertTrue(state['associated'])
                self.assertFalse(state['needs_precise'])
                self.assertEqual(state['version'],0)

    def test_late_weak_full_query_needs_current_unique_input(self):
        for interrupted in ('missing','ambiguous'):
            with self.subTest(interrupted=interrupted):
                self.setUp()
                source=self.visual(distance_valid=False,association_confirmations=3)
                self.memory.update(source)
                current=(self.absent() if interrupted=='missing' else
                    self.visual(distance_valid=False,associated=False,
                        ambiguous=True,candidate_count=2))
                self.memory.update(current)
                fine=self.precise(source);fine['target_association']={'status':'matched'}
                self.assertFalse(self.memory.bind_precise(fine,source,0,[]))
                self.assertIsNone(self.memory.last_precise)
                self.assertIsNone(self.memory._identity_visual)

    def test_completed_full_and_range_compare_current_input_without_mutating_target_history(self):
        for capability in ('full','range'):
            with self.subTest(capability=capability):
                self.setUp();self.seed()
                source=self.visual(association_confirmations=3)
                self.memory.update(source)
                fine=self.precise(source)
                if capability=='range':
                    point=fine.pop('cube_center_Base_mm')
                    fine.update(status='no_complete_face',range_candidate={
                        'schema':'aic.car.side_range.v1','status':'range_candidate',
                        'coordinate_frame':'Base','units':'mm','Base_X_mm':point[0],
                        'raw_center_Base_mm':point,'not_arm_or_parking_authority':True})
                # One changed coarse region is not the two-frame movement
                # confirmation needed to revoke an accepted travel anchor.
                self.stamp+=600
                state=self.memory.update(self.visual((500,100,0),
                    association_confirmations=1,navigation_confirmed=False),
                    motion_state={'run':1})
                self.assertTrue(state['associated'])
                self.assertEqual(state['version'],0)
                before=copy.deepcopy(self.memory.snapshot())
                self.assertEqual(self.memory.precise_consistency(fine,source,[]),'unknown')
                self.assertEqual(self.memory.snapshot(),before)
                self.memory.update(self.absent())
                self.assertEqual(self.memory.precise_consistency(fine,source,[]),'unknown')
                self.memory.update(self.visual(distance_valid=False,
                    bbox_px=[0,915,11,5],center_pixel=[6,917],reason='clipped_blob',
                    association_confirmations=3))
                self.assertEqual(self.memory.precise_consistency(fine,source,[]),'unknown')

    def test_gap_new_candidate_cannot_revoke_or_inherit_old_full_or_range(self):
        for capability in ('full','range'):
            with self.subTest(capability=capability):
                self.setUp()
                original=self.visual(association_confirmations=3,navigation_confirmed=True)
                self.memory.update(original)
                fine=self.precise(original)
                if capability=='range':
                    point=fine.pop('cube_center_Base_mm')
                    fine.update(status='no_complete_face',range_candidate={
                        'schema':'aic.car.side_range.v1','status':'range_candidate',
                        'coordinate_frame':'Base','units':'mm','Base_X_mm':point[0],
                        'raw_center_Base_mm':point,'not_arm_or_parking_authority':True})
                self.assertTrue(self.memory.bind_precise(fine,original,0))
                self.memory.last_disappearance_direction='left'
                identity=copy.deepcopy(self.memory._identity_visual)
                center=list(self.memory._last_center)
                self.memory.update(self.absent())
                self.stamp+=600  # Native tracker identity has expired too.
                for count in (1,2):
                    candidate=self.visual((500,230,0),association_confirmations=count,
                        navigation_confirmed=False,bbox_px=[867,517,6,6])
                    self.assertEqual(candidate['visual_evidence']['center_pixel'],[870.,520.])
                    state=self.memory.update(candidate,motion_state={'run':0})
                    self.assertFalse(state['associated'])
                    self.assertEqual(state['visibility'],'unknown')
                    self.assertFalse(state['table_distance_valid'])
                    self.assertEqual(state['version'],0)
                    self.assertTrue(state['anchor_valid'])
                    self.assertEqual(state['last_seen'],original)
                    self.assertEqual(self.memory._identity_visual,identity)
                    self.assertEqual(self.memory._last_center,center)
                    self.assertEqual(state['last_disappearance_direction'],'left')
                candidate=self.visual((500,230,0),association_confirmations=3,
                    navigation_confirmed=True,bbox_px=[867,517,6,6])
                state=self.memory.update(candidate,motion_state={'run':0})
                self.assertEqual(state['version'],0)
                self.assertTrue(state['anchor_valid'])
                self.assertTrue(state['target_valid'])
                self.assertFalse(state['associated'])
                self.assertTrue(state['needs_precise'])
                self.assertEqual(state['last_precise'],fine)
                self.assertEqual(state['paired_coarse'],original)
                self.assertEqual(state['last_seen'],original)
                self.assertEqual(state['last_disappearance_direction'],'left')
                self.assertTrue(self.memory.bind_precise(fine,original,0))
                self.assertFalse(self.memory.bind_precise(fine,original,1))
                # The old target can return without being replaced by the
                # other candidate or manufacturing another target version.
                current=self.visual(association_confirmations=4,
                    navigation_confirmed=True)
                self.memory.update(current)
                fresh=self.precise(current)
                fresh['target_association']={'status':'matched'}
                self.assertTrue(self.memory.bind_precise(fresh,current,0))
                self.assertTrue(self.memory.snapshot()['anchor_valid'])
                self.assertEqual(self.memory.last_precise,fresh)

    def test_ambiguous_targets_do_not_fall_back_to_legacy_largest_blob(self):
        self.seed()
        state=self.memory.update(self.visual((300,200,0),associated=False,
            ambiguous=True,candidate_count=2))
        self.assertTrue(state['visible']);self.assertFalse(state['associated'])
        self.assertEqual(state['visibility'],'ambiguous')
        self.assertFalse(state['table_distance_valid']);self.assertEqual(state['version'],0)

    def test_unanchored_unique_unassociated_jump_can_be_confirmed_as_new_location(self):
        self.memory.update(self.coarse())
        for unused in range(2):state=self.memory.update(self.visual((400,100,0),
            associated=False,reason='unassociated_blue'))
        self.assertEqual(state['event'],'moved');self.assertTrue(state['associated'])
        self.assertEqual(state['version'],1)

    def test_duplicate_and_old_frames_do_not_confirm_motion(self):
        self.seed()
        jump=self.coarse((500,100,0));self.memory.update(jump)
        for unused in range(10):state=self.memory.update(jump)
        self.assertEqual(state['version'],0)
        old=dict(jump);old['capture_timestamp_ms']-=100;old['frame_id']-=100
        self.assertEqual(self.memory.update(old)['version'],0)

    def test_history_size_is_constant(self):
        self.seed()
        for unused in range(3000):self.memory.update(self.coarse())
        self.assertLessEqual(len(self.memory._motion_rows),self.memory._motion_row_limit)
        self.assertTrue(all(self.stamp-row[0] <= self.memory._motion_window_ms
                            for row in self.memory._motion_rows))


if __name__=='__main__':unittest.main()
