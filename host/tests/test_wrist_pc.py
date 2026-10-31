"""Shared PC/K230 wrist planning and unchanged fixed-wrist model export."""
import copy
import hashlib
import itertools
import json
import math
import unittest
from unittest.mock import Mock, patch

import numpy as np

from host.project_paths import ROOT
from host.apps import pick_validation_pc as pc
from host.arm.portable_model import export_model
from host.vision.config import table_height_at
from tools.deploy import k230_pick as deploy
from device.k230.pick.kinematics import Geometry as PortableGeometry
from device.k230.pick.cube_orientation import estimate, plane_basis
from device.k230.pick.numeric import cross

PROFILE = ROOT/'configs/arm/pick_wrist_rotation.json'
STAGES = ('hover','descend','close','lift')
# Hash of the actual existing deployment model before extracting export_model.
# Only source organization changes; joint/mesh ordering and float bytes do not.
MODEL_SHA_BEFORE_EXPORT_REFACTOR = 'f8be2575ab8c14ae276bcabfbf9baec1fdc5d171fc419bee7cc91f9323534dc1'


class WristPcTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.pick, cls.config = pc.parameters(PROFILE)
        cls.board_config, cls.files, cls.manifest = deploy.build_bundle(PROFILE)
        cls.plane = cls.board_config['vision']['localization']['table_plane_Base_mm']
        cls.T = cls.board_config['extrinsics']['T_Base_from_Camera_mm']

    def side_result(self, yaw_deg):
        ex, ey, n = plane_basis(self.plane)
        angle = math.radians(yaw_deg)
        a = [math.cos(angle)*x+math.sin(angle)*y for x,y in zip(ex,ey)]
        axes = [a,cross(n,a)]
        faces = []
        for axis in axes:
            camera = [sum(self.T[j][i]*axis[j] for j in range(3)) for i in range(3)]
            pose = {'inward_face_normal_Camera':camera,'face_reprojection_rms_px':.3}
            faces.append({'used_for_center':True,'area_px2':1200,'selected_pose':pose,'poses':[pose]})
        return {'status':'complete_face_candidate','side_faces':faces}

    def make(self, yaw=30, start_roll=0, offset=(0,0,0), shape='vision'):
        start = [2048]*6
        start[4] = 2048-round(start_roll*4096/360)
        raw = self.side_result(yaw)
        result = {'cube_center_Base_mm':[248.,4.,-43.]}
        if shape=='vision': result['vision'] = raw
        elif shape=='observation': result['observation'] = {'payload':raw}
        elif shape=='payload': result['payload'] = raw
        else: raise AssertionError(shape)
        geometry, plan = pc.make_plan(result,start,self.pick,self.config,offset)
        return geometry, plan, start, result, raw

    def test_model_export_is_byte_identical_to_existing_corrected_deployment(self):
        model = export_model(ROOT,self.config.mapping,urdf_path=self.pick['model_urdf'])
        blob = json.dumps(model,sort_keys=True,separators=(',',':')).encode()
        self.assertEqual(hashlib.sha256(blob).hexdigest(),MODEL_SHA_BEFORE_EXPORT_REFACTOR)
        self.assertEqual(model,self.board_config['model'])
        for joint in model['joints'][1:4]:
            self.assertEqual(joint['axis'],[0.,-1.,0.])

    def test_unspecified_export_still_preserves_the_original_historical_urdf(self):
        original = export_model(ROOT,self.config.mapping)
        explicit = export_model(ROOT,self.config.mapping,urdf_path='models/genkiarm/genkiarm.urdf')
        self.assertEqual(original,explicit)
        self.assertNotEqual(original,self.board_config['model'])
        for joint in original['joints'][1:4]:
            self.assertEqual(joint['axis'],[0.,1.,0.])

    def test_pc_and_serialized_board_targets_agree_with_independent_host_fk(self):
        statuses = set()
        for yaw in (0,15,30,45):
            for roll in (-30,0,30):
                with self.subTest(cube_yaw=yaw,start_roll=roll):
                    geometry, plan, start, unused, raw = self.make(yaw,roll)
                    orientation = estimate(raw,self.T,self.plane)
                    board = PortableGeometry(json.loads(json.dumps(self.board_config['model'])),
                                             self.pick,plan['table_z_Base_mm'])
                    expected = board.plan([.248,.004],start,orientation)
                    status = plan['wrist_alignment']['status']
                    statuses.add(status)
                    self.assertIn(status,('aligned','held_current_fallback'))
                    self.assertEqual(plan['cube_orientation'],orientation)
                    if status=='aligned':
                        for stage,key in (('descend','open'),('close','closed')):
                            self.assertEqual(plan[key+'_finger_height_reference'],
                                             'table_Base_Z_at_target_XY')
                            lows = geometry.metrics(geometry.angles(plan[stage]))[2]
                            for link,low in lows.items():
                                clearance = low*1000-plan['table_z_Base_mm']
                                self.assertGreaterEqual(clearance,-1e-8)
                                self.assertLessEqual(clearance,20+1e-8)
                                self.assertAlmostEqual(
                                    plan[key+'_finger_clearances_mm'][link],clearance,places=8)
                    else:
                        baseline_pick = dict(self.pick,wrist_alignment_enabled=False)
                        baseline = PortableGeometry(self.board_config['model'],baseline_pick,
                                                    plan['table_z_Base_mm']).plan([.248,.004],start)
                        for stage in STAGES:
                            self.assertEqual(plan[stage],baseline[stage])
                            self.assertEqual(plan[stage][4],start[4])
                        self.assertIsNone(geometry.contact_orientation)
                    for stage in STAGES:
                        self.assertEqual(plan[stage],expected[stage])
                        report = geometry.report(plan[stage])
                        self.assertEqual(plan['target_FK'][stage],report)
                        # Report values are from the independent host URDF FK,
                        # rather than merely checking shared solver agreement.
                        q = geometry.angles(plan[stage])
                        _, tips, lows, _ = geometry.metrics(q)
                        for key,link in (('fixed_tip_center_Base_mm','wan2'),
                                         ('moving_tip_center_Base_mm','zhua')):
                            np.testing.assert_allclose(report[key],np.array(tips[link])*1000,atol=1e-9)
                        if stage in ('hover','descend'):
                            target_height = self.pick[stage+'_tip_height_mm']
                            self.assertLess(abs(min(lows.values())*1000-plan['table_z_Base_mm']-target_height),2)
                            self.assertLess(np.linalg.norm(np.array(report['alignment_center_Base_mm'][:2])-[248,4]),2)
                        self.assertEqual(plan[stage][4],plan['descend'][4])
                    self.assertEqual(plan['model_urdf'],self.pick['model_urdf'])
                    self.assertTrue(plan['model_correction_applied'])
                    self.assertIn('final poses only',plan['height_reference'])
        # This grid can become fully reachable when the user changes offsets.
        # Missing-side fallback is checked independently below.
        self.assertIn('aligned',statuses)

    def test_observation_payload_and_live_or_offline_selected_face_shapes_are_equivalent(self):
        expected = None
        for shape in ('vision','observation','payload'):
            with self.subTest(shape=shape):
                unused, plan, unused2, unused3, unused4 = self.make(shape=shape)
                goals = [plan[name] for name in STAGES]
                if expected is None: expected = goals
                else: self.assertEqual(goals,expected)
                self.assertEqual(plan['cube_orientation']['status'],'estimated')

    def test_pc_contact_report_uses_the_same_cube_oriented_3d_support_width(self):
        geometry, plan, unused, unused2, raw = self.make(15,0)
        self.assertEqual(plan['wrist_alignment']['status'],'aligned')
        orientation = estimate(raw,self.T,self.plane)
        q = geometry.angles(plan['descend'])
        axis = geometry.metrics(q)[3]['wan2'][:3,1]
        basis = orientation['axes_Base']+[orientation['table_normal_Base']]
        corners = [sum((sign*.010*np.array(vector) for sign,vector in zip(signs,basis)),np.zeros(3))
                   for signs in itertools.product((-1,1),repeat=3)]
        projections = [float(np.dot(axis,corner)) for corner in corners]
        expected_width = (max(projections)-min(projections))*1000
        report = geometry.report(plan['descend'])
        self.assertAlmostEqual(report['estimated_cube_projection_mm'],expected_width,places=8)
        self.assertAlmostEqual(plan['nominal_cube_width_mm'],expected_width,places=8)
        self.assertEqual(report['alignment_nominal_contact_count'],plan['nominal_closing_count'])
        self.assertEqual(plan['close'][5],plan['nominal_closing_count']-20)

    def test_missing_side_fallback_retains_all_fixed_wrist_stage_goals(self):
        for raw in ({'poses':[{}]}, {'side_faces':[]}):
            for roll in (-30,0,30):
                with self.subTest(raw=raw,start_roll=roll):
                    start = [2048]*6;start[4] = 2048-round(roll*4096/360)
                    vision = {'cube_center_Base_mm':[248,4,-43],'vision':raw}
                    geometry, plan = pc.make_plan(vision,start,self.pick,self.config)
                    baseline_pick = dict(self.pick,wrist_alignment_enabled=False)
                    board = PortableGeometry(self.board_config['model'],baseline_pick,plan['table_z_Base_mm'])
                    baseline = board.plan([.248,.004],start)
                    self.assertEqual(plan['wrist_alignment']['status'],'held_current_fallback')
                    for stage in STAGES:
                        self.assertEqual(plan[stage],baseline[stage])
                        self.assertEqual(plan[stage][4],start[4])
                    self.assertIsNone(geometry.contact_orientation)

    def test_manual_offsets_are_applied_once_without_mutating_profile_or_source(self):
        profile = copy.deepcopy(self.pick)
        source = {'cube_center_Base_mm':[248.,4.,-43.],'vision':self.side_result(15)}
        before = copy.deepcopy(source)
        start = [2048]*6
        geometry, plan = pc.make_plan(source,start,profile,self.config,(2,-1,3))
        self.assertEqual(profile,self.pick)
        self.assertEqual(source,before)
        self.assertEqual(plan['target_xy_Base_mm'],[250.,3.])
        self.assertEqual(plan['manual_offset_Base_mm'],[2,-1,3])
        self.assertEqual([plan[k] for k in ('hover_min_tip_model_mm','low_min_tip_model_mm','lift_min_tip_model_mm')],
                         [83,18,83])
        table_z = table_height_at(self.board_config['vision']['localization'],[250,3])
        self.assertEqual(plan['table_z_Base_mm'],table_z)
        self.assertEqual(geometry.table_z_m,table_z/1000)
        adjusted = copy.deepcopy(self.pick)
        for key in ('hover_tip_height_mm','descend_tip_height_mm','lift_tip_height_mm'):
            adjusted[key] += 3
        board = PortableGeometry(self.board_config['model'],adjusted,table_z)
        expected = board.plan([.250,.003],start,estimate(source['vision'],self.T,self.plane))
        for name in STAGES:self.assertEqual(plan[name],expected[name])

    def test_flag_disabled_does_not_use_portable_export_or_wrist_planner(self):
        profile = dict(self.pick,wrist_alignment_enabled=False)
        with patch.object(pc,'export_model',side_effect=AssertionError('Must keep old PC path')),\
             patch.object(PortableGeometry,'plan',side_effect=AssertionError('Must keep old PC path')):
            geometry,plan=pc.make_plan({'cube_center_Base_mm':[248,4,-43],
                'vision':self.side_result(30)},[2048]*6,profile,self.config)
        self.assertIs(type(geometry),pc.PickGeometry)
        self.assertNotIn('wrist_alignment',plan)
        self.assertNotIn('cube_orientation',plan)
        self.assertEqual(plan['cube_offset_toward_moving_finger_mm'],-5)

    def test_explicit_export_is_used_by_both_deployer_and_pc(self):
        with patch.object(deploy,'export_model',wraps=export_model) as build_export:
            deploy.build_bundle(PROFILE)
        build_export.assert_called_once()
        self.assertEqual(build_export.call_args.kwargs['urdf_path'],self.pick['model_urdf'])
        with patch.object(pc,'export_model',wraps=export_model) as pc_export:
            self.make()
        pc_export.assert_called_once()
        self.assertEqual(pc_export.call_args.kwargs['urdf_path'],self.pick['model_urdf'])

    def test_pick_deployment_clears_both_new_flat_module_caches_without_running_motion(self):
        with patch.object(deploy,'execute',return_value=['RELEASE_VERIFIED '+self.manifest['release_id']]) as run:
            deploy.verify(Mock(),'/sdcard/aic/pick/releases/test',self.manifest,self.board_config)
        script = run.call_args.args[1]
        self.assertIn("'cube_orientation'",script)
        self.assertIn("'wrist_plan'",script)
        self.assertIn('del sys.modules[name]',script)
        self.assertNotIn('run_pick_stage_direct(',script)
        self.assertNotIn('run_forever(',script)


if __name__=='__main__':unittest.main()
