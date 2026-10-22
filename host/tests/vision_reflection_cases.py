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

    def test_guided_three_faces_bright_top_and_dark_side(self):
        from device.k230.pick.face_selection import independent_adjacent_side
        for scale,shift in [(1.,0),(.7,-25),(1.,25)]:
            im,K,D,q=self.scene(1,shift,True)
            r=np.array([.4,-.3,.02]);t=np.array([-10.+shift,-10.,250.])
            top3=np.array([[0.,0.,20.],[20.,0.,20.],[20.,0.,0.],[0.,0.,0.]])
            top=cv2.projectPoints(top3,r,t,K,D)[0].reshape(-1,2)+300
            q=q+300;canvas=np.full((960,1280,3),(80,140,170),np.uint8)
            canvas[300:780,300:940]=im
            cv2.fillConvexPoly(canvas,np.rint(top).astype(np.int32),(245,99,0))
            cv2.fillConvexPoly(canvas,np.rint(q).astype(np.int32),(145,36,1))
            canvas=np.clip(cv2.GaussianBlur(canvas,(3,3),.7)*scale,0,255).astype(np.uint8)
            K[0,2]+=300;K[1,2]+=300
            result=independent_adjacent_side(canvas,K.tolist(),D.tolist(),top.tolist())
            self.assertIsNotNone(result)
            self.assertLess(np.max(np.linalg.norm(np.array(result['face_quad_px'])-q,axis=1)),3)
            self.assertLessEqual(result['poses'][0]['face_reprojection_rms_px'],2)
            self.assertTrue(all(e['support']>=45 for e in result['physical_edges']))
            # The same face cannot be associated with an unrelated top.
            self.assertIsNone(independent_adjacent_side(canvas,K.tolist(),D.tolist(),(top+[250,0]).tolist()))
            # Losing the bottom edge must not be repaired using the known size.
            canvas[int(q[:2,1].max())+10:,int(q[:,0].min())-5:int(q[:,0].max())+5]=(145,36,1)
            self.assertIsNone(independent_adjacent_side(canvas,K.tolist(),D.tolist(),top.tolist()))

    def test_refined_portable_pose_matches_opencv_lm(self):
        # Non-square image noise exercises refinement; no recorded user image.
        from device.k230.pick.pose import pose_candidates as solve
        im,K,D,q=self.scene(two_faces=True)
        q=q+np.array([[2.,-1.],[-1.,2.],[1.,-2.],[-2.,1.]])
        original=solve(q.tolist(),K.tolist(),D.tolist())
        refined=solve(q.tolist(),K.tolist(),D.tolist(),refine=True)
        self.assertLess(refined[0]['face_reprojection_rms_px'],original[0]['face_reprojection_rms_px'])
        obj=np.array([[0.,0.,0.],[20.,0.,0.],[20.,20.,0.],[0.,20.,0.]])
        seeds=cv2.solvePnPGeneric(obj,q,K,D,flags=cv2.SOLVEPNP_IPPE)
        errors=[]
        for rv,tv in zip(seeds[1],seeds[2]):
            rv,tv=cv2.solvePnPRefineLM(obj,q,K,D,rv,tv)
            pix=cv2.projectPoints(obj,rv,tv,K,D)[0].reshape(-1,2)
            errors.append(float(np.sqrt(np.mean(np.sum((pix-q)**2,axis=1)))))
        self.assertAlmostEqual(refined[0]['face_reprojection_rms_px'],min(errors),places=4)

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
