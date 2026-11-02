"""Target changes tested against independent world-frame vehicle motion."""
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
        self.assertTrue(changed['measurement_inconsistent'])
        self.assertTrue(changed['appearance_changed'])
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

    def test_explicit_edge_departure_is_distinct_from_a_central_occlusion(self):
        self.seed()
        self.memory.update(self.visual((500,-600,0),distance_valid=False,
            edge_directions=['left']))
        state=self.memory.update(self.absent())
        self.assertEqual(state['event'],'exited');self.assertEqual(state['version'],1)
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
        for point in ((500,100,0),(500,150,0)):
            state=self.memory.update(self.visual(point),arm_state=arm)
            self.assertEqual(state['event'],'holding')
            self.assertFalse(state['table_distance_valid'])
            self.assertEqual(state['version'],0)
        state=self.memory.update(self.absent(),arm_state=arm)
        self.assertEqual(state['version'],0)

    def test_unconfirmed_close_does_not_claim_a_held_object(self):
        self.seed()
        arm={'phase':'close','pending_stage':None,'grasp_confirmed':None}
        for unused in range(2):state=self.memory.update(self.visual((500,150,0)),arm_state=arm)
        self.assertEqual(state['event'],'moved')
        self.assertEqual(state['version'],1)

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
        self.seed();arm={'phase':'lift','grasp_confirmed':True,'held_center_pixel':[670,520]}
        for unused in range(3):
            state=self.memory.update(self.visual((500,130,0),distance_valid=False,
                bbox_px=[650,400,100,100]),arm_state=arm)
        self.assertEqual(state['version'],0)

    def test_visible_but_unusable_range_does_not_become_absent(self):
        state=self.memory.update(self.visual(distance_valid=False,
            reason='inconsistent_with_20mm_height'))
        self.assertTrue(state['visible']);self.assertEqual(state['visibility'],'visible')
        self.assertFalse(state['table_distance_valid'])
        self.assertFalse(state['confirmed_moved'])
        self.assertTrue(state['needs_precise'])

    def test_full_fine_rescues_visible_coarse_geometry_without_inventing_range(self):
        coarse=self.visual(distance_valid=False,reason='inconsistent_with_20mm_height')
        self.memory.update(coarse)
        self.assertTrue(self.memory.bind_precise(self.precise(coarse),coarse,0))
        self.assertFalse(self.memory.snapshot()['needs_precise'])
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
        self.assertTrue(self.memory.bind_precise(self.precise(coarse),coarse,0,[]))
        self.assertTrue(self.memory.snapshot()['needs_precise'])

    def test_ambiguous_targets_do_not_fall_back_to_legacy_largest_blob(self):
        self.seed()
        state=self.memory.update(self.visual((300,200,0),associated=False,
            ambiguous=True,candidate_count=2))
        self.assertTrue(state['visible']);self.assertFalse(state['associated'])
        self.assertEqual(state['visibility'],'ambiguous')
        self.assertFalse(state['table_distance_valid']);self.assertEqual(state['version'],0)

    def test_unique_unassociated_jump_can_be_confirmed_as_new_location(self):
        self.seed()
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
