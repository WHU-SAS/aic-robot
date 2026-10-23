"""Independent convex-intersection oracle plus production planner regressions."""
import math,sys,unittest
from unittest.mock import patch
import numpy as np
from scipy.spatial import ConvexHull
from scipy.optimize import linprog
from host.project_paths import ROOT
sys.path.insert(0,str(ROOT/'device/k230/pick'))
from finger_collision import boxes_overlap,hull_overlap,CubeEnvelope,FingerCollision,validate_plan
from numeric import rotation,transform
from kinematics import Geometry
from tools.deploy.k230_pick import build_bundle
from host.tests.test_cube_wrist import synthetic_found
from cube_orientation import estimate


class CollisionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg,_,_=build_bundle();cls.plane=cls.cfg['vision']['localization']['table_plane_Base_mm']
        cls.T=cls.cfg['extrinsics']['T_Base_from_Camera_mm'];cls.start=cls.cfg['ready_control']['goals']
    def scene(self,xy,yaw):
        n=self.plane['normal'];z=(self.plane['offset_mm']-sum(n[i]*xy[i]*1000 for i in range(2)))/n[2]
        g=Geometry(self.cfg['model'],self.cfg['pick'],z)
        o=estimate(synthetic_found(yaw,self.T,self.plane),self.T,self.plane)
        return g,o
    def test_sat_edge_crossing_touch_and_clear(self):
        R=rotation([0,0,1],math.pi/4)
        self.assertTrue(boxes_overlap([0,0,0],R,[.03,.001,.001],[.01]*3))
        self.assertTrue(boxes_overlap([.02,0,0],np.eye(3).tolist(),[.01]*3,[.01]*3))
        self.assertFalse(boxes_overlap([.0201,0,0],np.eye(3).tolist(),[.01]*3,[.01]*3))
    def test_gjk_against_independent_halfspace_linear_program(self):
        rng=np.random.default_rng(20261023)
        for unused in range(160):
            points=rng.normal(size=(24,3))*.008+rng.uniform(-.03,.03,size=3)
            hull=ConvexHull(points);half=[.01]*3
            exact=linprog([0,0,0],A_ub=hull.equations[:,:3],b_ub=-hull.equations[:,3],bounds=[(-h,h) for h in half],method='highs').success
            self.assertEqual(hull_overlap(points.tolist(),half),exact)
            indices=list(hull.vertices);mapping={int(v):i for i,v in enumerate(indices)}
            neighbors=[set() for _ in indices]
            for face in hull.simplices:
                for i in range(3):
                    a=mapping[int(face[i])];b=mapping[int(face[(i+1)%3])]
                    neighbors[a].add(b);neighbors[b].add(a)
            self.assertEqual(hull_overlap(points[indices].tolist(),half,[list(v) for v in neighbors]),exact)
    def test_geometry_slices_enclose_random_points_in_each_finger_hull(self):
        rng=np.random.default_rng(1)
        for item in self.cfg['model']['points'].values():
            verts=np.array(item['hull']);boxes=item['collision_boxes']
            samples=np.r_[verts,np.array([rng.dirichlet(np.ones(4))@verts[rng.integers(len(verts),size=4)] for _ in range(100)])]
            for p in samples:
                self.assertTrue(any(np.all(np.abs(p-b['center'])<=np.array(b['half'])+1e-10) for b in boxes))
    def test_reported_fixed_jaw_target_is_rejected_and_exact_hull_overlaps(self):
        xy=[.2929968,.003688034];g,o=self.scene(xy,-5.850853);cube=CubeEnvelope(g,xy,o)
        q=g.angles([2103,2779,2904,1996,2030,2389]);hit=cube.pose(g,q)
        self.assertIsNotNone(hit);self.assertEqual(hit['finger'],'wan2')
        pts=np.array([transform(g.transforms(q)['wan2'],v) for v in g.points['wan2']['hull']])
        local=(pts-np.array(cube.center))@np.array(cube.axes).T;h=ConvexHull(local)
        exact=linprog([0,0,0],A_ub=h.equations[:,:3],b_ub=-h.equations[:,3],bounds=[(-v,v) for v in cube.half],method='highs')
        self.assertTrue(exact.success)
    def test_known_clear_rotated_scene_still_plans_with_diagnostics(self):
        xy=[.2528551,.001102018];g,o=self.scene(xy,-15.21932)
        plan=g.plan(xy,self.start,o)
        self.assertEqual(plan['open_finger_collision_check']['status'],'model_clear')
        self.assertFalse(plan['open_finger_collision_check']['actual_motion_certified'])
        self.assertTrue(plan['open_finger_collision_check']['sweep_checked'])
        self.assertGreater(plan['open_finger_collision_check']['pose_checks'],3)
    def test_reported_scene_selects_a_different_clear_target(self):
        xy=[.2929968,.003688034];g,o=self.scene(xy,-5.850853)
        plan=g.plan(xy,self.start,o)
        self.assertNotEqual(plan['descend'],[2103,2779,2904,1996,2030,2389])
        self.assertTrue(plan['open_finger_collision_check']['sweep_checked'])
        self.assertIsNone(g.cube_envelope.pose(g,g.angles(plan['descend'])))

    def test_fallback_cannot_bypass_rejection(self):
        xy=[.269,-.012];g,o=self.scene(xy,30)
        with patch('finger_collision.validate_plan',side_effect=FingerCollision('TEST_COLLISION')) as check:
            with self.assertRaisesRegex(ValueError,'TEST_COLLISION'):g.plan(xy,self.start,o)
        self.assertGreater(check.call_count,1)
    def test_blocked_scene_never_dispatches_a_stage(self):
        from types import SimpleNamespace
        from host.tests.test_standalone_pick import VoiceDispatchTests
        app=VoiceDispatchTests().app();found=synthetic_found(-5.850853,self.T,self.plane)
        found['cube_center_Base_mm']=[292.9968,3.688034,-59.70737]
        class Camera:
            def __init__(self,*a):pass
            def open(self):pass
            def close(self):pass
            def snapshot(self):return SimpleNamespace(save=lambda *a,**k:None)
        with patch.object(app,'CameraSession',Camera),patch.object(app,'localize',return_value=found), \
             patch.object(app,'arm_snapshot',return_value=[{'position':c} for c in self.start]), \
             patch.object(app,'run_pick_stage_direct') as dispatch,patch.object(app,'log'), \
             patch('finger_collision.CubeEnvelope.pose',return_value={'finger':'wan2','test':'all candidates collide'}), \
             patch.dict(sys.modules,{'cv2':SimpleNamespace(imread=lambda p:object()),'control':SimpleNamespace()}):
            with self.assertRaisesRegex(ValueError,'intersects'):app.run_once(self.cfg,execute=True)
        dispatch.assert_not_called()

    def test_missing_geometry_is_not_a_clear_pose(self):
        from copy import deepcopy
        xy=[.25,0];g,o=self.scene(xy,0);g.points=deepcopy(g.points)
        del g.points['wan2']['collision_boxes']
        with self.assertRaisesRegex(FingerCollision,'Missing'):CubeEnvelope(g,xy,o).pose(g,g.angles(self.start))
    def test_sweep_detects_intermediate_contact_even_when_endpoints_clear(self):
        # A translating finger box crosses the cube; neither endpoint overlaps.
        from finger_collision import segment
        from types import SimpleNamespace
        class Cube:
            def pose(self,g,q,inflate=0):return {'finger':'wan2'} if abs(q[0])<=.1+inflate else None
        g=SimpleNamespace(points={k:{'hull':[[1.,0,0]]} for k in ('wan2','zhua')},
            joints=[{'origin_transform':np.eye(4).tolist()} for _ in range(6)])
        with self.assertRaisesRegex(FingerCollision,'intersects'):segment(g,Cube(),[-.2,0,0,0,0,0],[.2,0,0,0,0,0],'test')

if __name__=='__main__':unittest.main()
