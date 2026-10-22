"""Run in scripts/vision: physical geometry, exposure, occlusion and negative scenes."""
import unittest
import cv2
import numpy as np
from host.vision.blue_side_edges import detect_blue_side


class BlueSideEdgeCases(unittest.TestCase):
    def scene(self,scale=1.,block=False,translate_y=0):
        K=np.array([[1200.,0.,320.],[0.,1600.,240.],[0.,0.,1.]])
        D=np.zeros(5)
        rotation=np.array([.4,.05,.02]);translation=np.array([-10.,-10.+translate_y,250.])
        front=np.array([[0.,0.,0.],[20.,0.,0.],[20.,20.,0.],[0.,20.,0.]])
        top=np.array([[0.,0.,20.],[20.,0.,20.],[20.,0.,0.],[0.,0.,0.]])
        q=cv2.projectPoints(front,rotation,translation,K,D)[0].reshape(-1,2)
        tq=cv2.projectPoints(top,rotation,translation,K,D)[0].reshape(-1,2)
        image=np.full((480,640,3),(80,140,170),np.uint8)
        cv2.fillConvexPoly(image,np.rint(tq).astype(np.int32),(160,40,10))
        cv2.fillConvexPoly(image,np.rint(q).astype(np.int32),(240,190,15))
        if block:
            cx=int(q[:2,0].mean());bottom=int(q[:2,1].min()-2)
            cv2.rectangle(image,(cx-12,0),(cx+12,bottom),(35,35,35),-1)
        image=cv2.GaussianBlur(image,(3,3),.7)
        return np.clip(image.astype(float)*scale,0,255).astype(np.uint8),K,D,q

    def test_projected_side_recovered_under_exposure_changes_and_tip_occlusion(self):
        for scale in (.45,.7,1.):
            image,K,D,q=self.scene(scale,block=True)
            result=detect_blue_side(image,K,D,(0,0,640,480))
            self.assertEqual(result['status'],'complete_face_candidate',str(result))
            actual=np.array(result['face_quad_px'])
            self.assertLess(np.max(np.linalg.norm(actual-q,axis=1)),4)
            self.assertLessEqual(result['poses'][0]['face_reprojection_rms_px'],2)
            self.assertTrue(all(e['rms_px']<=3 for e in result['physical_edges']))

    def test_plain_blue_rectangle_without_physical_seam_not_accepted(self):
        image,K,D,_=self.scene()
        image[:]=(80,140,170)
        cv2.rectangle(image,(260,180),(360,320),(240,190,15),-1)
        result=detect_blue_side(image,K,D,(0,0,640,480))
        self.assertEqual(result['status'],'no_complete_face')

    def test_no_blue_has_no_target(self):
        image,K,D,_=self.scene();image[:]=(90,90,90)
        self.assertEqual(detect_blue_side(image,K,D)['status'],'no_complete_face')

    def test_clipped_face_is_not_completed_by_guessing(self):
        image,K,D,_=self.scene(translate_y=35)
        result=detect_blue_side(image,K,D,(0,0,640,480))
        self.assertEqual(result['status'],'no_complete_face')


if __name__=='__main__':unittest.main()
