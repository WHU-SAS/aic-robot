"""Fitted full-cube projection metadata; pure host math, no image runtime."""
import copy
import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from device.k230.pick import pose
from device.k230.pick.numeric import dot,matvec,rotation


K=[[1200.,0.,640.],[0.,1210.,480.],[0.,0.,1.]]
D=[.12,-.3,.002,-.001,.2]
PROJECTION_FIELDS=('cube_projected_corners_px','cube_projection_source')


def face_points(R,t):
    return [[v+t[i] for i,v in enumerate(matvec(R,obj))] for obj in pose.OBJECT]


def known_pose(R,t,distortion=None):
    distortion=D if distortion is None else distortion
    points=face_points(R,t);quad=[pose.project(p,K,distortion) for p in points]
    calls=[]
    def refiner(*unused):
        calls.append(True)
        return R,t
    values=pose.pose_candidates(quad,K,distortion,refine=True,refiner=refiner)
    return values,points,calls


def pure_side_selection():
    # select_sides itself is arithmetic. Load its real source with image-only
    # imports stubbed, so these metadata tests need neither OpenCV nor NumPy.
    path=Path(pose.__file__).with_name('side_faces.py')
    spec=importlib.util.spec_from_file_location('device.k230.pick._projection_test_sides',path)
    module=importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules,{'cv2':SimpleNamespace(),'numpy':SimpleNamespace(float32=float),'ulab':None}):
        spec.loader.exec_module(module)
    return module


class PoseCubeProjectionTests(unittest.TestCase):
    def test_eight_corners_reuse_the_fitted_front_and_project_the_back(self):
        R=rotation([.8,.6,0.],.6);t=[12.,18.,230.]
        values,front,calls=known_pose(R,t)
        self.assertEqual(len(calls),2)  # One original refinement per planar pose.
        self.assertEqual(len(values),2)
        n=[row[2] for row in R]
        expected=[pose.project(p,K,D) for p in front]
        expected.extend(pose.project([p[i]+20*n[i] for i in range(3)],K,D) for p in front)
        for value in values:
            self.assertEqual(value['cube_projection_source'],'pose_transform')
            self.assertEqual(value['cube_projected_corners_px'],expected)
            self.assertEqual(len(value['cube_projected_corners_px']),8)
            self.assertTrue(all(math.isfinite(v) for q in value['cube_projected_corners_px'] for v in q))

    def test_opposite_vertices_follow_the_actual_inward_normal_sign(self):
        R=rotation([0.,1.,0.],2.);t=[0.,0.,240.]
        values,front,unused=known_pose(R,t)
        original=[row[2] for row in R]
        self.assertLess(dot(original,t),0.)
        inward=[-v for v in original]
        expected=[pose.project([p[i]+20*inward[i] for i in range(3)],K,D) for p in front]
        for value in values:
            self.assertEqual(value['inward_face_normal_Camera'],inward)
            self.assertEqual(value['cube_projected_corners_px'][4:],expected)

    def test_back_vertex_behind_camera_does_not_reject_the_original_pose(self):
        R=rotation([0.,1.,0.],2.3);t=[100.,0.,12.]
        # This near-plane edge case needs an invertible original image. The
        # ordinary lens model is outside its domain at x/z>5 and cannot seed
        # the legacy pose solve; zero distortion isolates optional metadata.
        values,front,calls=known_pose(R,t,distortion=[])
        self.assertTrue(all(p[2]>0 for p in front))
        n=[row[2] for row in R]
        self.assertGreater(dot(n,t),0.)
        self.assertTrue(any(p[2]+20*n[2]<=0 for p in front))
        self.assertEqual(len(values),2)
        self.assertEqual(len(calls),2)
        for value in values:
            self.assertTrue(math.isfinite(value['face_reprojection_rms_px']))
            self.assertEqual(value['cube_center_Camera_mm'],[t[i]+10*n[i] for i in range(3)])
            for key in PROJECTION_FIELDS:self.assertNotIn(key,value)

    def test_projection_resource_error_is_not_hidden_as_unavailable_metadata(self):
        R=rotation([.8,.6,0.],.6);t=[12.,18.,230.]
        q=[pose.project(p,K,D) for p in face_points(R,t)]
        original=pose.project;calls=[]
        def project(point,*args):
            calls.append(point)
            if len(calls)==6:raise MemoryError('back projection allocation')
            return original(point,*args)
        with patch.object(pose,'project',project),self.assertRaises(MemoryError):
            pose.pose_candidates(q,K,D,refine=True,refiner=lambda *unused:(R,t))

    def test_existing_pose_fields_and_ranking_match_pre_metadata_golden(self):
        quad=[[590.743753738651,510.068718873823],[666.7064578634601,509.2667622258538],
              [667.8743279749217,593.0910790251395],[588.5974942298614,595.8767628741152]]
        golden={False:[
            {'face_reprojection_rms_px':0.8139123922322176,'cube_center_Camera_mm':[-6.530773983500895,16.931825846871018,295.0619078695346],'cube_center_pixel':[613.4159589410924,549.4940001309069],'inward_face_normal_Camera':[-0.3918964651198867,-0.006923313331296244,0.9199832761300903]},
            {'face_reprojection_rms_px':1.2209594600731255,'cube_center_Camera_mm':[0.891754413240025,18.10621298675951,294.96310422643546],'cube_center_pixel':[643.6258944532663,554.3357300921355],'inward_face_normal_Camera':[0.3749373145691205,0.11500915741043463,0.919888527950772]}],
            True:[
            {'face_reprojection_rms_px':0.8110008226406863,'cube_center_Camera_mm':[-6.526436695888765,16.99182639879183,295.2079944278949],'cube_center_pixel':[613.4466564148443,549.7059698816643],'inward_face_normal_Camera':[-0.39130259579358356,-0.0016410499497517507,0.9202606073717736]},
            {'face_reprojection_rms_px':1.2060535900622575,'cube_center_Camera_mm':[0.7762762252291875,18.247938677943367,295.7302596872431],'cube_center_pixel':[643.1475480490258,554.7237697871354],'inward_face_normal_Camera':[0.363786822676802,0.12509769115023642,0.9230437234040417]}]}
        for refined,expected in golden.items():
            with self.subTest(refine=refined):
                actual=pose.pose_candidates(quad,K,D,refine=refined)
                self.assertEqual([{k:v for k,v in p.items() if k not in PROJECTION_FIELDS} for p in actual],expected)


class SideMeanProjectionTests(unittest.TestCase):
    def faces(self,narrow=False):
        q1=[[100.,100.],[200.,100.],[200.,300.],[100.,300.]]
        width=50. if narrow else 100.
        q2=[[200.,100.],[200.+width,100.],[200.+width,300.],[200.,300.]]
        result=[]
        for index,(quad,n) in enumerate(((q1,[0.,0.,1.]),(q2,[1.,0.,0.]))):
            value={'face_reprojection_rms_px':.2+index*.1,'cube_center_Camera_mm':[index*2.,0.,240.],
                   'cube_center_pixel':pose.project([index*2.,0.,240.],K,D),'inward_face_normal_Camera':n,
                   'cube_projected_corners_px':[[float(i+index),float(i)] for i in range(8)],
                   'cube_projection_source':'pose_transform'}
            result.append({'face_quad_px':quad,'area_px2':100.*200. if index==0 else width*200.,
                           'physical_edges':[],'poses':[value]})
        return result

    def test_mean_has_no_rigid_projection_but_each_selected_side_keeps_its_own(self):
        module=pure_side_selection();faces=self.faces();before=copy.deepcopy(faces)
        result=module.select_sides(faces,K,D)
        self.assertEqual(result['side_selection'],'mean_of_similar_area_sides')
        aggregate=result['poses'][0]
        self.assertEqual(aggregate['cube_center_Camera_mm'],[1.,0.,240.])
        self.assertEqual(aggregate['face_reprojection_rms_px'],.30000000000000004)
        for key in PROJECTION_FIELDS:self.assertNotIn(key,aggregate)
        for index,face in enumerate(result['side_faces']):
            self.assertTrue(face['used_for_center'])
            self.assertIs(face['selected_pose'],faces[index]['poses'][0])
            for key in PROJECTION_FIELDS:self.assertEqual(face['selected_pose'][key],before[index]['poses'][0][key])
        self.assertEqual(faces,before)

    def test_single_side_and_larger_side_keep_the_selected_rigid_projection(self):
        module=pure_side_selection()
        for faces,rule in ((self.faces()[:1],'single_verified_side'),(self.faces(narrow=True),'larger_verified_side')):
            with self.subTest(rule=rule):
                result=module.select_sides(faces,K,D)
                self.assertEqual(result['side_selection'],rule)
                self.assertIs(result['poses'][0],faces[0]['poses'][0])
                for key in PROJECTION_FIELDS:self.assertIn(key,result['poses'][0])


if __name__=='__main__':unittest.main()
