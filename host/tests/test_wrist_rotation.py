"""Opt-in wrist geometry and exact default fallback, with no hardware or images."""
import copy
import json
import math
import unittest

from host.project_paths import ROOT
from tools.deploy.k230_pick import build_bundle
from device.k230.pick.cube_orientation import estimate,plane_basis
from device.k230.pick.kinematics import Geometry
from device.k230.pick.numeric import cross,dot
from device.k230.pick.wrist_plan import direction_error,jaw_direction,_stages

STAGES=('hover','descend','close','lift')
IDENTITY=[[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.]]


def face(angle,used=True,area=1000.):
    a=math.radians(angle)
    return {'used_for_center':used,'area_px2':area,'selected_pose':{
        'inward_face_normal_Camera':[math.cos(a),math.sin(a),0.],
        'face_reprojection_rms_px':.3},'poses':[{
        'inward_face_normal_Camera':[0.,0.,1.],'face_reprojection_rms_px':.1}]}


class CubeOrientationTests(unittest.TestCase):
    def test_selected_sides_use_equal_fourfold_mean_instead_of_diagonal_normal(self):
        result=estimate({'side_faces':[face(10,True,1000),face(102,True,1200),
            face(40,False,1000000)]},IDENTITY,{'normal':[0.,0.,1.]})
        self.assertEqual(result['status'],'estimated')
        self.assertEqual(result['side_count'],2)
        self.assertAlmostEqual(result['yaw_table_deg'],11.)
        self.assertEqual(result['period_deg'],90)

    def test_single_selected_pose_and_camera_rotation_define_yaw(self):
        a=math.radians(33)
        T=[[math.cos(a),-math.sin(a),0.,300.],[math.sin(a),math.cos(a),0.,20.],
            [0.,0.,1.,-50.],[0.,0.,0.,1.]]
        result=estimate({'side_faces':[face(-23)]},T,{'normal':[0.,0.,1.]})
        self.assertAlmostEqual(result['yaw_table_deg'],10.)
        self.assertEqual(result['side_count'],1)

    def test_top_only_missing_or_malformed_normals_are_unavailable(self):
        top=face(0);top['selected_pose']['inward_face_normal_Camera']=[0.,0.,1.]
        inputs=[None,{}, {'poses':[face(20)['selected_pose']]},{'side_faces':'invalid'},
            {'side_faces':[None,{},top]}]
        for normal in (None,[0.,0.,0.],[1.,2.],[float('nan'),0.,1.],[True,0.,0.]):
            bad=face(0);bad['selected_pose']['inward_face_normal_Camera']=normal
            inputs.append({'side_faces':[bad]})
        for found in inputs:
            with self.subTest(found=found):
                self.assertEqual(estimate(found,IDENTITY,{'normal':[0.,0.,1.]})['status'],'unavailable')

    def test_invalid_table_transform_or_selected_pose_quality_does_not_steer_wrist(self):
        for plane,T in (({},IDENTITY),({'normal':[0.,0.,0.]},IDENTITY),
                        ({'normal':[0.,0.,1.]},None),({'normal':[0.,0.,1.]},[[1.]])):
            self.assertEqual(estimate({'side_faces':[face(10)]},T,plane)['status'],'unavailable')
        for rms in (None,2.01,float('inf')):
            bad=face(10);bad['selected_pose']['face_reprojection_rms_px']=rms
            self.assertEqual(estimate({'side_faces':[bad]},IDENTITY,{'normal':[0.,0.,1.]})['status'],'unavailable')


class WristPlanningTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg,unused,unused_manifest=build_bundle()
        cls.plane=cls.cfg['vision']['localization']['table_plane_Base_mm']

    def geometry(self,xy,enabled=True,serialized=False):
        cfg=json.loads(json.dumps(self.cfg)) if serialized else self.cfg
        # Keep the earlier zero-offset geometry assertions as historical checks.
        pick=dict(cfg['pick'],cube_offset_toward_moving_finger_mm=0);pick['wrist_alignment_enabled']=enabled
        n=self.plane['normal'];z=(self.plane['offset_mm']-n[0]*xy[0]-n[1]*xy[1])/n[2]
        return Geometry(cfg['model'],pick,z)

    def orientation(self,angle):
        x,y,n=plane_basis(self.plane);a=math.radians(angle)
        axis=[math.cos(a)*u+math.sin(a)*v for u,v in zip(x,y)]
        return {'status':'estimated','axes_Base':[axis,cross(n,axis)],
            'table_normal_Base':n,'yaw_table_deg':angle,'side_count':2}

    def test_disabled_flag_preserves_frozen_corrected_baseline_targets(self):
        g=self.geometry([248,4],enabled=False)
        result=g.plan([.248,.004],[2048]*6,self.orientation(45))
        expected={'hover':[2067,2496,2789,2622,2048,2332],
            'descend':[2067,2666,2884,2358,2048,2332],
            'close':[2067,2666,2884,2358,2048,2142],
            'lift':[2067,2496,2789,2622,2048,2142]}
        for stage in STAGES:self.assertEqual(result[stage],expected[stage])
        self.assertNotIn('wrist_alignment',result)

    def test_missing_top_only_or_invalid_orientation_keeps_original_goals(self):
        g=self.geometry([248,4]);start=[2048]*6;start[4]=1707
        old=self.geometry([248,4],enabled=False).plan([.248,.004],start)
        bad=dict(self.orientation(20),table_normal_Base=[float('nan'),0.,1.])
        for orientation in (None,{},bad,{'status':'estimated'},
                            estimate({'poses':[face(20)['selected_pose']]},IDENTITY,self.plane)):
            result=g.plan([.248,.004],start,orientation)
            self.assertEqual(result['wrist_alignment']['status'],'held_current_fallback')
            for stage in STAGES:self.assertEqual(result[stage],old[stage])

    def test_no_complete_aligned_candidate_uses_exact_old_four_stage_fallback(self):
        xy=[275,0];g=self.geometry(xy);start=[2048]*6
        result=g.plan([v/1000 for v in xy],start,self.orientation(30))
        old=self.geometry(xy,enabled=False).plan([v/1000 for v in xy],start)
        self.assertEqual(result['wrist_alignment']['status'],'held_current_fallback')
        self.assertGreater(result['wrist_alignment']['counts']['candidate_attempts'],0)
        for stage in STAGES:
            self.assertEqual(result[stage],old[stage])
            self.assertEqual(result[stage][4],start[4])

    def test_center_yaws_and_initial_rolls_align_with_fixed_stage_semantics(self):
        xy=[248,4];g=self.geometry(xy)
        for yaw in range(0,46,5):
            for initial in (-30,0,30):
                with self.subTest(yaw=yaw,initial=initial):
                    start=[2048]*6;start[4]=2048-round(initial*4096/360)
                    orientation=self.orientation(yaw)
                    result=g.plan([v/1000 for v in xy],start,orientation)
                    detail=result['wrist_alignment']
                    if detail['status']=='held_current_fallback':
                        old=self.geometry(xy,enabled=False).plan([v/1000 for v in xy],start)
                        for stage in STAGES:self.assertEqual(result[stage],old[stage])
                        continue
                    self.assertEqual(detail['status'],'aligned')
                    self.assertLessEqual(detail['counts']['candidate_attempts'],40)
                    self.assertLessEqual(detail['counts']['ik_calls'],120)
                    self.assertLessEqual(detail['stage_direction_errors_deg']['descend'],1.)
                    self.assertEqual(detail['direction_error_deg'][1],detail['stage_direction_errors_deg']['descend'])
                    self.assertEqual(result['opening_count'],2332)
                    self.assertEqual(result['extra_closing_counts'],20)
                    self.assertEqual(result['close'][:5],result['descend'][:5])
                    self.assertEqual(result['close'][5],result['nominal_closing_count']-20)
                    self.assertTrue(all(result[s][4]==result['descend'][4] for s in STAGES))
                    self.assertGreaterEqual(min(result['open_finger_clearances_mm'].values()),0.)
                    self.assertLessEqual(max(result['open_finger_clearances_mm'].values()),20.)
                    self.assertEqual(result['open_finger_height_reference'],'table_Base_Z_at_target_XY')
                    self.assertGreaterEqual(min(result['closed_finger_clearances_mm'].values()),0.)
                    self.assertLessEqual(max(result['closed_finger_clearances_mm'].values()),20.)
                    for stage,height in (('hover',80.),('descend',15.),('lift',80.)):
                        q=g.angles(result[stage]);q[5]=g.open_angle
                        center=g.alignment_midpoint(q);lows=g.metrics(q)[2]
                        self.assertLess(math.hypot(center[0]*1000-xy[0],center[1]*1000-xy[1]),2.)
                        self.assertLess(abs(min(lows.values())*1000-g.table_z_m*1000-height),2.)

    def test_straight_cube_does_not_turn_ninety_degrees_for_an_earlier_tilt(self):
        # Frozen input from the reported143240 run. The old tilt-first loop
        # returned axis0 at5deg/88.68deg before reaching axis1 at7.5deg/-2.64deg.
        xy=[240.1945,-3.275456];start=[2050,1139,2955,2958,2046,2324]
        orientation={'status':'estimated','yaw_table_deg':0.0038187543704580468,
            'axes_Base':[[0.998474583515104,-0.0008804176085793556,-0.05520625815226152],
                         [-6.654816842774445e-05,0.9998529420427943,-0.01714904836165123]],
            'table_normal_Base':[0.05521323795686063,0.01712656279594587,0.9983276912923511]}
        pick=dict(self.cfg['pick'],wrist_alignment_enabled=True,
                  cube_offset_toward_moving_finger_mm=-3)
        g=Geometry(self.cfg['model'],pick,-66.86218411503955)
        result=g.plan([v/1000 for v in xy],start,orientation)
        detail=result['wrist_alignment']
        self.assertEqual(detail['status'],'aligned')
        self.assertEqual(detail['axis_preference'],[1,0])
        self.assertEqual(detail['cube_axis_index'],1)
        self.assertEqual(result['descend_tilt_deg'],7.5)
        self.assertLess(detail['servo5_rotation_from_start_deg'],3.)
        self.assertEqual(result['descend'][4],2078)
        self.assertLess(detail['stage_direction_errors_deg']['descend'],1.)
        # The selected small rotation is a complete grasp, rather than merely
        # a direction root: all stages retain it and both jaws overlap the cube.
        for stage in STAGES:self.assertEqual(result[stage][4],2078)
        for key in ('open_finger_clearances_mm','closed_finger_clearances_mm'):
            self.assertTrue(all(0<=v<=20 for v in result[key].values()))
        self.assertEqual(result['close'][:5],result['descend'][:5])
        self.assertEqual(result['close'][5],result['nominal_closing_count']-20)

    def test_other_face_pair_is_available_when_the_nearer_pair_cannot_complete(self):
        # Here the nearer pair has no complete four-stage pose at any allowed
        # tilt, whereas the other pair really is reachable. Do not turn the
        # preference into a hard rejection of the second valid pair.
        xy=[230,-60];g=self.geometry(xy)
        g.pick['cube_offset_toward_moving_finger_mm']=-3
        start=[2048]*6;start[4]=2048-round(-30*4096/360)
        orientation=self.orientation(45)
        result=g.plan([v/1000 for v in xy],start,orientation)
        detail=result['wrist_alignment']
        self.assertEqual(detail['status'],'aligned')
        self.assertEqual(detail['axis_preference'],[1,0])
        self.assertEqual(detail['cube_axis_index'],0)
        self.assertGreater(detail['servo5_rotation_from_start_deg'],50.)
        self.assertLess(detail['stage_direction_errors_deg']['descend'],1.)
        self.assertTrue(all(result[s][4]==result['descend'][4] for s in STAGES))
        for key in ('open_finger_clearances_mm','closed_finger_clearances_mm'):
            self.assertTrue(all(0<=v<=20 for v in result[key].values()))
        # Independently exhaust the nearer pair with the unmodified geometry
        # checks, so this tests a physical case rather than mocking loop order.
        from device.k230.pick.wrist_plan import TILTS,roll_candidates
        counters={'scalar_evaluations':0,'ik_calls':0};near_count=0
        for tilt in TILTS:
            for roll,index,unused in roll_candidates(g,[.230,-.060],tilt,
                    orientation,g.angles(start)[4],counters):
                if index!=1:continue
                near_count+=1
                with self.assertRaises(ValueError):
                    _stages(g,[.230,-.060],start,tilt,roll,orientation,counters)
        self.assertGreater(near_count,0)

    def test_a_jaw_above_cube_when_open_is_not_rescued_by_acceptable_closed_height(self):
        g=self.geometry([248,4]);orientation=self.orientation(5)
        # This former candidate has acceptable closed height but an open
        # jaw above20mm. It must stay rejected even if another pose succeeds.
        roll=math.radians(85.25390625);start=[2048]*6
        low,ql=g.solve([.248,.004],.015,10,g.angles(start),roll)
        self.assertGreater(max(g.metrics(ql)[2].values()),g.table_z_m+.020)
        contact,unused=g.nominal_contact(ql,orientation)
        closed=list(ql);closed[5]=contact;goals=g.counts(closed);goals[5]-=20
        self.assertLessEqual(max(g.metrics(g.angles(goals))[2].values()),g.table_z_m+.020)
        with self.assertRaisesRegex(ValueError,'Open fingers'):
            _stages(g,[.248,.004],start,10,roll,orientation,{'ik_calls':0})
        result=g.plan([.248,.004],start,orientation)
        if result['wrist_alignment']['status']=='aligned':
            self.assertTrue(all(0<=h<=20 for h in result['open_finger_clearances_mm'].values()))
            self.assertTrue(all(0<=h<=20 for h in result['closed_finger_clearances_mm'].values()))

    def test_45_degree_cube_uses_the_bounded_half_grid_and_independent_high_tilts(self):
        g=self.geometry([248,4]);result=g.plan([.248,.004],[2048]*6,self.orientation(45))
        detail=result['wrist_alignment']
        self.assertEqual(detail['status'],'aligned')
        self.assertEqual(result['descend_tilt_deg'],7.5)
        self.assertEqual(result['hover_tilt_deg'],15)
        self.assertEqual(result['lift_tilt_deg'],15)
        self.assertEqual(detail['stage_tilts_deg'],result['stage_tilts_deg'])
        self.assertLess(detail['stage_direction_errors_deg']['descend'],1.)
        self.assertGreater(detail['stage_direction_errors_deg']['hover'],1.)
        self.assertTrue(all(result[s][4]==result['descend'][4] for s in STAGES))
        self.assertTrue(all(0<=v<=20 for v in result['open_finger_clearances_mm'].values()))
        self.assertTrue(all(0<=v<=20 for v in result['closed_finger_clearances_mm'].values()))

    def test_serialized_board_model_and_host_package_share_identical_targets(self):
        for yaw in (0,15,30,45):
            orientation=self.orientation(yaw);start=[2048]*6;start[4]=2389
            a=self.geometry([248,4]).plan([.248,.004],start,orientation)
            b=self.geometry([248,4],serialized=True).plan([.248,.004],start,orientation)
            for stage in STAGES:self.assertEqual(a[stage],b[stage])

    def test_aligned_points_and_contact_width_match_independent_host_URDF(self):
        import numpy as np
        from host.apps.pick_validation_pc import parameters
        from host.arm.pick_validation_geometry import OpenTipPickGeometry
        unused,mapping_config=parameters()
        host=OpenTipPickGeometry(ROOT,mapping_config.mapping,opening_count=2332,
            urdf_path='models/genkiarm/genkiarm_corrected.urdf',cube_offset_toward_moving_finger_mm=0)
        g=self.geometry([248,4]);host.table_z_m=g.table_z_m
        for yaw in (0,15,30,45):
            orientation=self.orientation(yaw);result=g.plan([.248,.004],[2048]*6,orientation)
            for stage in STAGES:
                q=host.angles(result[stage]);unused,points,lows,trans=host.metrics(q)
                unused,bpoints,blows,btrans=g.metrics(q)
                for link in ('wan2','zhua'):
                    np.testing.assert_allclose(points[link],bpoints[link],atol=1e-10)
                    self.assertAlmostEqual(lows[link],blows[link],delta=1e-10)
            if result['wrist_alignment']['status']!='aligned':continue
            for stage in ('descend','close'):
                q=host.angles(result[stage]);lows=host.metrics(q)[2]
                self.assertGreaterEqual(min(lows.values()),g.table_z_m)
                self.assertLessEqual(max(lows.values()),g.table_z_m+.020)
            q=host.angles(result['descend']);axis=host.metrics(q)[3]['wan2'][:3,1]
            expected=20.*sum(abs(np.dot(axis,v)) for v in orientation['axes_Base']+[orientation['table_normal_Base']])
            self.assertAlmostEqual(expected,result['nominal_cube_width_mm'],delta=1e-9)

    def test_authorized_offset_moves_cube_toward_fixed_jaw_after_wrist_rotation(self):
        # Independent host FK checks sign and length, rather than comparing
        # two uses of the same compensated midpoint implementation.
        import numpy as np
        from host.apps.pick_validation_pc import parameters
        from host.arm.pick_validation_geometry import OpenTipPickGeometry
        unused,cfg=parameters()
        host=OpenTipPickGeometry(ROOT,cfg.mapping,opening_count=2332,
            urdf_path='models/genkiarm/genkiarm_corrected.urdf',
            cube_offset_toward_moving_finger_mm=-3)
        aligned=0
        for yaw in (0,15,30,45):
            g=self.geometry([248,4]);g.pick['cube_offset_toward_moving_finger_mm']=-3
            result=g.plan([.248,.004],[2048]*6,self.orientation(yaw))
            self.assertEqual(result['cube_offset_toward_moving_finger_mm'],-3)
            self.assertEqual(result['target_xy_Base_mm'],[248.,4.])
            if result['wrist_alignment']['status']=='aligned':
                aligned+=1
                self.assertLess(result['wrist_alignment']['stage_direction_errors_deg']['descend'],1.)
            for stage in ('hover','descend','lift'):
                q=host.angles(result[stage]);q[5]=g.open_angle
                mid,tips,unused2,unused3=host.metrics(q)
                direction=np.array(tips['zhua'][:2])-tips['wan2'][:2]
                direction/=np.linalg.norm(direction)
                virtual=mid[:2]-.003*direction
                np.testing.assert_allclose(virtual,host.alignment_midpoint(q)[:2],atol=1e-10)
                self.assertLess(np.linalg.norm(virtual-[.248,.004]),.002)
                delta=virtual-mid[:2]
                self.assertAlmostEqual(float(np.dot(delta,direction)),-.003,places=10)
        self.assertGreater(aligned,0)

    def test_same_fixed_jaw_offset_is_retained_when_orientation_falls_back(self):
        g=self.geometry([248,4]);g.pick['cube_offset_toward_moving_finger_mm']=-3
        start=[2048]*6;start[4]=1707
        result=g.plan([.248,.004],start,None)
        expected=g._plan_held([.248,.004],start)
        self.assertEqual(result['wrist_alignment']['status'],'held_current_fallback')
        self.assertEqual(result['cube_offset_toward_moving_finger_mm'],-3)
        for stage in STAGES:self.assertEqual(result[stage],expected[stage])


if __name__=='__main__':unittest.main()
