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
    def test_no_blue_has_no_target(self):
        im,K,D,q=self.scene();im[:]=90
        with self.assertRaisesRegex(ValueError,'No reliable'):
            localize(im,K.tolist(),D.tolist(),np.eye(4).tolist())
    def test_gradient_does_not_invent_missing_seam(self):
        im,K,D,q=self.scene();im[:]=(80,140,170)
        cv2.rectangle(im,(580,530),(680,670),(240,190,15),-1)
        self.assertIsNone(gradient_side(im,cv2.cvtColor(im,cv2.COLOR_BGR2HSV),K.tolist(),D.tolist()))

if __name__=='__main__':unittest.main()
