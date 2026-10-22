"""Image-only regression scenes: attached reflection is not a face boundary."""
import sys
import unittest
import cv2
import numpy as np
from host.project_paths import ROOT
from host.vision.face_geometry import pose_candidates
from device.k230.pick.reflection_faces import detect_reflection_face
sys.path.insert(0,str(ROOT/'device/k230/pick'))
from pose import pose_candidates as portable_pose


class ReflectionFaceCases(unittest.TestCase):
    def scene(self,scale=1.,shift=0.,two_faces=False,clipped=False):
        K=np.array([[1200.,0.,320.],[0.,1600.,240.],[0.,0.,1.]])
        D=np.zeros(5)
        r=np.array([.4,-.3 if two_faces else .05,.02])
        t=np.array([-10.+shift,25. if clipped else -10.,250.])
        side=np.array([[0.,0.,0.],[20.,0.,0.],[20.,20.,0.],[0.,20.,0.]])
        top=np.array([[0.,0.,20.],[20.,0.,20.],[20.,0.,0.],[0.,0.,0.]])
        right=np.array([[20.,0.,0.],[20.,0.,20.],[20.,20.,20.],[20.,20.,0.]])
        project=lambda p:cv2.projectPoints(p,r,t,K,D)[0].reshape(-1,2)
        q=project(side);top=project(top);right=project(right)
        im=np.full((480,640,3),(80,140,170),np.uint8)
        bottom=max(0,int(min(q[2:,1])));left=max(0,int(q[:,0].min()-8));edge=min(640,int(q[:,0].max()+12))
        # Blue reflection reaches the image border and is connected to the cube.
        im[bottom:,left:edge]=(135,100,45)
        im[430:,left:edge]=(100,35,5)
        cv2.fillConvexPoly(im,np.rint(top).astype(np.int32),(160,40,10))
        if two_faces:cv2.fillConvexPoly(im,np.rint(right).astype(np.int32),(245,150,15))
        cv2.fillConvexPoly(im,np.rint(q).astype(np.int32),(240,190,15))
        im=cv2.GaussianBlur(im,(3,3),.7)
        return np.clip(im.astype(float)*scale,0,255).astype(np.uint8),K,D,q

    def detect(self,im,K,D,portable=False):
        solver=portable_pose if portable else lambda q,k,d:pose_candidates(np.asarray(q),k,d)
        return detect_reflection_face(im,K.tolist() if portable else K,D.tolist() if portable else D,solver,(0,0,640,480))

    def test_attached_border_reflection_excluded_across_exposure_and_position(self):
        for scale,shift,two in [(1.,0,False),(.7,-25,False),(.45,25,False),(1.,0,True)]:
            with self.subTest(scale=scale,shift=shift,two_faces=two):
                im,K,D,q=self.scene(scale,shift,two)
                broad=cv2.inRange(cv2.cvtColor(im,cv2.COLOR_BGR2HSV),(90,80,35),(140,255,255))
                contours,_=cv2.findContours(broad,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
                self.assertTrue(any(cv2.boundingRect(c)[1]+cv2.boundingRect(c)[3]==480 for c in contours))
                r=self.detect(im,K,D)
                self.assertIsNotNone(r)
                self.assertLess(np.max(np.linalg.norm(np.array(r['face_quad_px'])-q,axis=1)),4)
                self.assertLessEqual(r['poses'][0]['face_reprojection_rms_px'],2)
                self.assertTrue(all(e['support']>=45 and e['rms_px']<=3 for e in r['physical_edges']))
                board=self.detect(im,K,D,True)
                self.assertIsNotNone(board)
                np.testing.assert_allclose(board['face_quad_px'],r['face_quad_px'],atol=1e-5)
                np.testing.assert_allclose(board['poses'][0]['cube_center_Camera_mm'],r['poses'][0]['cube_center_Camera_mm'],atol=.1)

    def test_reflection_alone_is_not_completed_into_a_cube(self):
        im,K,D,_=self.scene();im[:]=(80,140,170)
        im[300:,270:380]=(135,100,45)
        im[430:,270:380]=(100,35,5)
        self.assertIsNone(self.detect(im,K,D))

    def test_true_clipped_face_rejected_even_with_blue_reflection(self):
        im,K,D,q=self.scene(clipped=True)
        self.assertGreater(q[2:,1].max(),480)
        self.assertIsNone(self.detect(im,K,D))

    def test_uniform_rectangle_lacks_top_side_seam(self):
        im,K,D,_=self.scene();im[:]=(80,140,170)
        cv2.rectangle(im,(260,180),(360,320),(240,190,15),-1)
        self.assertIsNone(self.detect(im,K,D))

    def test_missing_bottom_edge_is_not_inferred_from_square_size(self):
        im,K,D,q=self.scene()
        left=int(q[:,0].min())-1;right=int(q[:,0].max())+1
        im[int(q[:2,1].max())+10:,left:right]=(240,190,15)
        self.assertIsNone(self.detect(im,K,D))


if __name__=='__main__':unittest.main()
