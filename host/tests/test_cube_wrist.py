"""Cube orientation and roll-aware IK; no camera, serial or physical motion."""
import math,json,sys,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
from host.project_paths import ROOT
sys.path.insert(0,str(ROOT/'device/k230/pick'))
from cube_orientation import estimate,plane_basis
from wrist_alignment import error,closure
from numeric import dot,matvec,transpose,rotation,matmul
from kinematics import Geometry
from tools.deploy.k230_pick import build_bundle


def synthetic_found(degrees,T,plane,second=True):
    x,y,n=plane_basis(plane);a=math.radians(degrees);sides=[]
    R=[row[:3] for row in T[:3]]
    for angle,area in [(a,20000.),(a+math.pi/2,16000.)][:2 if second else 1]:
        v=[math.cos(angle)*xx+math.sin(angle)*yy for xx,yy in zip(x,y)]
        p={'inward_face_normal_Camera':matvec(transpose(R),v),'face_reprojection_rms_px':.8}
        sides.append({'area_px2':area,'selected_pose':p})
    return {'side_faces':sides,'cube_center_Base_mm':[229.5,25.53,-56.]}


class CubeWristTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg,_,_=build_bundle();cls.plane=cls.cfg['vision']['localization']['table_plane_Base_mm']
        cls.T=cls.cfg['extrinsics']['T_Base_from_Camera_mm'];cls.start=cls.cfg['ready_control']['goals']
    def geometry(self,xy):
        n=self.plane['normal'];z=(self.plane['offset_mm']-n[0]*xy[0]*1000-n[1]*xy[1]*1000)/n[2]
        return Geometry(self.cfg['model'],self.cfg['pick'],z)
    def test_two_normals_fuse_modulo_90_not_toward_diagonal(self):
        for yaw in (-89,-46,-45,-44,-15,0,30,44,45,46,89,179):
            found=synthetic_found(yaw,self.T,self.plane);o=estimate(found,self.T,self.plane)
            residual=(o['yaw_table_deg']-yaw+45)%90-45
            self.assertAlmostEqual(residual,0,places=8)
            self.assertEqual(o['side_count'],2)
            self.assertLess(max(o['side_residual_deg']),1e-8)
            self.assertAlmostEqual(dot(o['axes_Base'][0],o['axes_Base'][1]),0,places=10)
    def test_translation_does_not_change_direction_and_single_side_works(self):
        found=synthetic_found(22,self.T,self.plane,False);a=estimate(found,self.T,self.plane)
        T=[list(r) for r in self.T]
        for row in T[:3]:row[3]+=10000
        self.assertEqual(a,estimate(found,T,self.plane));self.assertEqual(a['side_count'],1)
        self.assertEqual(estimate({},self.T,self.plane)['status'],'unavailable')
    def test_near_cube_roll_maintains_coordinates_and_stage_height(self):
        for xy in ([.2295,.02553],[.245,-.028]):
            g=self.geometry(xy)
            for yaw in (-45,-30,-15,0,15,30,45):
                o=estimate(synthetic_found(yaw,self.T,self.plane),self.T,self.plane)
                p=g.plan(xy,self.start,o)
                self.assertEqual(p['wrist_alignment']['status'],'aligned',(xy,yaw))
                self.assertLess(max(p['wrist_alignment']['stage_direction_errors_deg']),1)
                for stage,height in (('hover',80),('descend',15),('lift',80)):
                    self.assertEqual(p[stage][4],p['close'][4])
                    # Lift has closed fingers, so verify the planned open-tool
                    # target with the open encoder before applying the close goal.
                    q=list(p[stage]);q[5]=self.cfg['pick']['opening_count'];report=g.report(q)
                    self.assertLess(math.dist(report['alignment_center_Base_mm'][:2],[v*1000 for v in xy]),2)
                    self.assertLess(abs(min(report['link_clearance_above_table_mm'].values())-height),2)
                self.assertLessEqual(max(p['wrist_alignment']['closed_finger_clearances_mm'].values()),20)
                for stage in ('hover','descend','close','lift'):
                    for j,c in zip(g.joints,p[stage]):
                        self.assertLessEqual(j['software_count_limits'][0],c);self.assertLessEqual(c,j['software_count_limits'][1])
    def test_far_unreachable_rotated_grasp_falls_back_to_original_goals(self):
        xy=[.30987,.03692];g=self.geometry(xy)
        o=estimate(synthetic_found(-45,self.T,self.plane),self.T,self.plane)
        p=g.plan(xy,self.start,o);original=g.plan(xy,self.start)
        self.assertEqual(p['wrist_alignment']['status'],'held_current')
        self.assertIn('alignment_failure',p['wrist_alignment'])
        for stage in ('hover','descend','close','lift'):self.assertEqual(p[stage],original[stage])
    def test_previous_narrow_boundary_scene_uses_independent_stage_tilts(self):
        # Same-pitch planning previously needed a narrow 13.2-degree pose.
        # Independent elevated stages allow a lower grasp pitch and more overlap.
        xy=[.2672202,.02377807];g=self.geometry(xy)
        o=estimate(synthetic_found(-38.1356,self.T,self.plane,False),self.T,self.plane)
        p=g.plan(xy,self.start,o);w=p['wrist_alignment']
        self.assertEqual(w['status'],'aligned')
        self.assertNotEqual(p['stage_tilts_deg']['hover'],p['stage_tilts_deg']['descend'])
        self.assertLess(max(w['closed_finger_clearances_mm'].values()),19)
        self.assertGreater(abs(w['servo5_change_deg']),30)
        self.assertLess(max(w['stage_direction_errors_deg']),1)
        self.assertGreaterEqual(min(w['closed_finger_clearances_mm'].values()),0)
        self.assertLessEqual(max(w['closed_finger_clearances_mm'].values()),20)
        for stage in ('hover','descend','close','lift'):
            self.assertEqual(p[stage][4],p['hover'][4])
            for j,c in zip(g.joints,p[stage]):
                self.assertLessEqual(j['software_count_limits'][0],c)
                self.assertLessEqual(c,j['software_count_limits'][1])
    def test_latest_scene_has_reachable_high_and_low_poses_with_one_roll(self):
        xy=[.2703452,-.01172861];g=self.geometry(xy)
        o=estimate(synthetic_found(36.23959,self.T,self.plane,False),self.T,self.plane)
        p=g.plan(xy,self.start,o);w=p['wrist_alignment']
        self.assertEqual(w['status'],'aligned')
        self.assertGreater(abs(w['servo5_change_deg']),40)
        self.assertNotEqual(p['stage_tilts_deg']['hover'],p['stage_tilts_deg']['descend'])
        self.assertEqual(p['stage_tilts_deg']['close'],p['stage_tilts_deg']['descend'])
        self.assertLess(max(w['stage_direction_errors_deg']),1)
        self.assertGreaterEqual(min(w['closed_finger_clearances_mm'].values()),0)
        self.assertLess(max(w['closed_finger_clearances_mm'].values()),20)
        for stage,height in (('hover',80),('descend',15),('lift',80)):
            self.assertEqual(p[stage][4],p['close'][4])
            self.assertGreaterEqual(p['stage_tilts_deg'][stage],0)
            self.assertLessEqual(p['stage_tilts_deg'][stage],45)
            q=list(p[stage]);q[5]=self.cfg['pick']['opening_count'];report=g.report(q)
            self.assertLess(math.dist(report['alignment_center_Base_mm'][:2],[x*1000 for x in xy]),2)
            self.assertLess(abs(min(report['link_clearance_above_table_mm'].values())-height),2)
            # Verify pitch metadata describes the encoded joints, within rounding.
            actual=180-math.degrees(sum(g.angles(q)[1:4]))
            self.assertLess(abs(actual-p['stage_tilts_deg'][stage]),.3)
            for j,c in zip(g.joints,p[stage]):
                self.assertLessEqual(j['software_count_limits'][0],c)
                self.assertLessEqual(c,j['software_count_limits'][1])
    def test_width_is_cube_support_along_actual_3d_closure_axis(self):
        g=self.geometry([.2295,.02553]);o=estimate(synthetic_found(30,self.T,self.plane),self.T,self.plane)
        p=g.plan([.2295,.02553],self.start,o);q=g.angles(p['descend']);d=closure(g,q)
        axes=o['axes_Base']+[o['table_normal_Base']]
        support=[dot(d,[sum(10*signs[j]*axes[j][i] for j in range(3)) for i in range(3)])
            for signs in ((a,b,c) for a in (-1,1) for b in (-1,1) for c in (-1,1))]
        self.assertAlmostEqual(p['nominal_cube_width_mm'],max(support)-min(support),places=8)
    def test_app_passes_visual_orientation_and_keeps_four_stage_policy(self):
        from host.tests.test_standalone_pick import VoiceDispatchTests
        app=VoiceDispatchTests().app();calls=[];logged=[]
        found=synthetic_found(30,self.T,self.plane)
        class Camera:
            def __init__(self,*a):pass
            def open(self):pass
            def close(self):pass
            def snapshot(self):return SimpleNamespace(save=lambda *a,**k:None)
        def run(action,stage):calls.append((list(action['goals']),stage));return {'source':'arrival_gate'}
        with patch.object(app,'CameraSession',Camera),patch.object(app,'localize',return_value=found),\
             patch.object(app,'arm_snapshot',return_value=[{'position':c} for c in self.start]),\
             patch.object(app,'run_pick_stage_direct',side_effect=run),\
             patch.object(app,'log',side_effect=lambda *a:logged.append(a)),\
             patch.dict(sys.modules,{'cv2':SimpleNamespace(imread=lambda p:object()),'control':SimpleNamespace()}):
            p=app.run_once(self.cfg,execute=True)
        self.assertEqual(p['wrist_alignment']['status'],'aligned')
        self.assertEqual([s['name'] for c,s in calls],['hover','descend','close','lift'])
        for goals,stage in calls:self.assertEqual(stage['policy'],self.cfg['pick']['standalone_stage_policy'])
        self.assertEqual([c for c,s in calls],[p[k] for k in ('hover','descend','close','lift')])
        self.assertTrue(any(e[0]=='descend_position_sample' for e in logged))

if __name__=='__main__':unittest.main()
