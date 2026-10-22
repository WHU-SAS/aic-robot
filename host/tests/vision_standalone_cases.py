"""Portable vision matches existing physical-edge cases; dedicated vision env."""
import sys
import unittest
import cv2
import numpy as np
from host.project_paths import ROOT
sys.path.insert(0,str(ROOT/'device/k230/pick'))
from vision import gradient_side,localize

class StandaloneVisionCases(unittest.TestCase):
    def scene(self,scale=1.,block=False):
        K=np.array([[1200.,0.,640.],[0.,1600.,580.],[0.,0.,1.]])
        D=np.zeros(5);rv=np.array([.4,.05,.02]);t=np.array([-10.,-10.,250.])
        side=np.array([[0.,0.,0.],[20.,0.,0.],[20.,20.,0.],[0.,20.,0.]])
        top=np.array([[0.,0.,20.],[20.,0.,20.],[20.,0.,0.],[0.,0.,0.]])
        q=cv2.projectPoints(side,rv,t,K,D)[0].reshape(-1,2);tq=cv2.projectPoints(top,rv,t,K,D)[0].reshape(-1,2)
        im=np.full((960,1280,3),(80,140,170),np.uint8)
        cv2.fillConvexPoly(im,np.rint(tq).astype(np.int32),(160,40,10))
        cv2.fillConvexPoly(im,np.rint(q).astype(np.int32),(240,190,15))
        if block:
            cx=int(q[:2,0].mean());bottom=int(q[:2,1].min()-2)
            cv2.rectangle(im,(cx-12,0),(cx+12,bottom),(35,35,35),-1)
        im=cv2.GaussianBlur(im,(3,3),.7)
        return np.clip(im.astype(float)*scale,0,255).astype(np.uint8),K,D,q
    def test_gradient_edges_preserve_exposure_and_occlusion_behavior(self):
        for scale in (.45,.7,1.):
            im,K,D,q=self.scene(scale,True);hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV)
            result=gradient_side(im,hsv,K.tolist(),D.tolist())
            self.assertIsNotNone(result)
            self.assertLess(np.max(np.linalg.norm(np.array(result['face_quad_px'])-q,axis=1)),4)
            self.assertLessEqual(result['poses'][0]['face_reprojection_rms_px'],2)
    def test_shared_runtime_returns_status_for_legacy_face_fallback(self):
        from unittest.mock import patch
        im,K,D,q=self.scene()
        candidate={'face_quad_px':q.tolist(),'poses':[{'cube_center_Camera_mm':[0.,0.,250.]}]}
        with patch('vision.basic_face',return_value=candidate),patch('vision.prefer_sides',return_value=None),patch('vision.shallow_face',return_value=False),patch('vision.internal_seam',return_value=None):
            result=localize(im,K.tolist(),D.tolist(),np.eye(4).tolist())
        self.assertEqual(result['status'],'complete_face_candidate')

    def test_no_blue_has_no_target(self):
        im,K,D,q=self.scene();im[:]=90
        with self.assertRaisesRegex(ValueError,'No reliable'):
            localize(im,K.tolist(),D.tolist(),np.eye(4).tolist())
    def test_gradient_does_not_invent_missing_seam(self):
        im,K,D,q=self.scene();im[:]=(80,140,170)
        cv2.rectangle(im,(580,530),(680,670),(240,190,15),-1)
        self.assertIsNone(gradient_side(im,cv2.cvtColor(im,cv2.COLOR_BGR2HSV),K.tolist(),D.tolist()))


class MultiFaceOutlineCases(unittest.TestCase):
    def scene(self,angle,side_value=160):
        K=np.array([[1200.,0.,640.],[0.,1600.,580.],[0.,0.,1.]])
        D=np.zeros(5);rv=np.array([angle,.02,.01]);t=np.array([-10.,-10.,250.])
        side=np.array([[0.,0.,0.],[20.,0.,0.],[20.,20.,0.],[0.,20.,0.]])
        top=np.array([[0.,0.,20.],[20.,0.,20.],[20.,0.,0.],[0.,0.,0.]])
        q=cv2.projectPoints(side,rv,t,K,D)[0].reshape(-1,2)
        tq=cv2.projectPoints(top,rv,t,K,D)[0].reshape(-1,2)
        im=np.full((960,1280,3),(80,140,170),np.uint8)
        cv2.fillConvexPoly(im,np.rint(tq).astype(np.int32),(250,90,15))
        cv2.fillConvexPoly(im,np.rint(q).astype(np.int32),(side_value,40,10))
        return cv2.GaussianBlur(im,(3,3),.7),K,D,q

    def test_trimmed_top_corner_matches_overlapping_shared_edge(self):
        from device.k230.pick.face_selection import adjacent_side
        top=[[15.,0.],[125.,0.],[120.,35.],[25.,35.]]
        side=[[0.,40.],[130.,40.],[130.,200.],[0.,200.]]
        self.assertTrue(adjacent_side(top,side))
        # Collinear edges on another object lack enough shared interval.
        self.assertFalse(adjacent_side(top,[[x+100,y] for x,y in side]))
        # A nearby parallel edge without normal proximity is not shared.
        self.assertFalse(adjacent_side(top,[[x,y+25] for x,y in side]))
        # One endpoint crossing a seam cannot validate a rotated object.
        wrong=[[0.,10.],[130.,80.],[130.,220.],[0.,180.]]
        self.assertFalse(adjacent_side(top,wrong))

    def test_bright_top_uses_adjacent_complete_side(self):
        from vision import basic_face
        from host.vision.runtime_cube_faces import verify_single_face
        for angle in (.15,.25):
            im,K,D,q=self.scene(angle,110)
            top=basic_face(cv2.cvtColor(im,cv2.COLOR_BGR2HSV),K.tolist(),D.tolist())
            self.assertIsNotNone(top)
            top['status']='complete_face_candidate'
            pc=verify_single_face(im,K,D,top)
            board=localize(im,K.tolist(),D.tolist(),np.eye(4).tolist())
            for result in (pc,board):
                self.assertTrue('preferred_over_top' in result or result.get('vision_path')=='direct_component_sides')
                self.assertLess(np.max(np.linalg.norm(np.array(result['face_quad_px'])-q,axis=1)),3)

    def test_unrelated_or_incomplete_side_does_not_replace_top(self):
        from vision import basic_face
        from unittest.mock import patch
        from host.vision.runtime_cube_faces import verify_single_face
        im,K,D,q=self.scene(.25,110)
        top=basic_face(cv2.cvtColor(im,cv2.COLOR_BGR2HSV),K.tolist(),D.tolist())
        top['status']='complete_face_candidate'
        side={'status':'complete_face_candidate','poses':top['poses'],
              'face_quad_px':(q+[200,0]).tolist()}
        with patch('host.vision.runtime_cube_faces.split_exposed_cube',return_value=side),patch('device.k230.pick.face_selection.independent_adjacent_side',return_value=None):
            self.assertEqual(verify_single_face(im,K,D,top),top)
        with patch('vision.fast_sides',return_value=None),patch('vision.prefer_sides',return_value=None),patch('vision.split_face',return_value=None),patch('vision.independent_adjacent_side',return_value=None):
            result=localize(im,K.tolist(),D.tolist(),np.eye(4).tolist())
        self.assertNotIn('preferred_over_top',result)
        self.assertTrue(result['top_boundary_verified'])
        self.assertEqual(result['threshold_proposal']['face_quad_px'],top['face_quad_px'])

    def test_two_planes_can_pass_square_pnp_but_must_be_separated(self):
        from host.vision.grasp_face_quality import assess
        from host.vision.runtime_cube_faces import verify_single_face
        from unittest.mock import patch
        for angle in (.15,.25):
            im,K,D,q=self.scene(angle)
            old=assess(im,K,D)
            self.assertEqual(old['status'],'complete_face_candidate')
            self.assertLess(old['poses'][0]['face_reprojection_rms_px'],2)
            self.assertGreater(np.max(np.linalg.norm(np.array(old['face_quad_px'])-q,axis=1)),10)
            pc=verify_single_face(im,K,D,old)
            self.assertIn('rejected_multiface_outline',pc)
            self.assertLess(np.max(np.linalg.norm(np.array(pc['face_quad_px'])-q,axis=1)),3)
            # Reproduce the failing path: the bright-face selector failed and
            # the dim-face selector accepted the combined top/front outline.
            with patch('vision.fast_sides',return_value=None),patch('vision.basic_face',return_value=None):
                board=localize(im,K.tolist(),D.tolist(),np.eye(4).tolist())
            self.assertIn('rejected_multiface_outline',board)
            self.assertLess(np.max(np.linalg.norm(np.array(board['face_quad_px'])-q,axis=1)),3)

    def test_unresolved_interior_seam_rejects_merged_candidate(self):
        from host.vision.grasp_face_quality import assess
        from host.vision.runtime_cube_faces import verify_single_face
        from unittest.mock import patch
        im,K,D,q=self.scene(.25);old=assess(im,K,D)
        with patch('host.vision.runtime_cube_faces.split_exposed_cube',return_value={'status':'no_complete_face'}):
            result=verify_single_face(im,K,D,old)
        self.assertEqual(result['status'],'no_complete_face')
        with patch('vision.fast_sides',return_value=None),patch('vision.prefer_sides',return_value=None),patch('vision.basic_face',return_value=None),patch('vision.split_face',return_value=None),patch('vision.gradient_side',return_value=None),patch('vision.detect_reflection_face',return_value=None):
            with self.assertRaisesRegex(ValueError,'No reliable'):
                localize(im,K.tolist(),D.tolist(),np.eye(4).tolist())

if __name__=='__main__':unittest.main()
