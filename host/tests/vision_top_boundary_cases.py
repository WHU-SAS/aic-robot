"""A small bright polygon is only a proposal, never proof of a physical face."""
import sys
import unittest
from unittest.mock import patch
import cv2
import numpy as np
from host.project_paths import ROOT
from device.k230.pick.top_boundaries import validate_top_boundary
from device.k230.pick.pose import pose_candidates
sys.path.insert(0,str(ROOT/'device/k230/pick'))
from vision import localize


class TopBoundaryCases(unittest.TestCase):
    def scene(self,inset=0.,exposure=1.,yaw=0.):
        K=np.array([[1200.,0.,640.],[0.,1600.,580.],[0.,0.,1.]])
        D=np.zeros(5);rv=np.array([1.2,.04,.01]);tv=np.array([-10.,-10.,250.])
        obj=np.array([[0.,0.,0.],[20.,0.,0.],[20.,20.,0.],[0.,20.,0.]])
        rotation=cv2.Rodrigues(rv)[0]@cv2.Rodrigues(np.array([0.,0.,np.deg2rad(yaw)]))[0]
        rv=cv2.Rodrigues(rotation)[0]
        project=lambda p:cv2.projectPoints(p,rv,tv,K,D)[0].reshape(-1,2)
        truth=project(obj);inner=obj.copy();inner[:,:2]=inset+inner[:,:2]/20*(20-2*inset)
        # Rotate the inner proposal with the same physical face.
        seed=project(inner);im=np.full((960,1280,3),(80,140,170),np.uint8)
        cv2.fillConvexPoly(im,np.rint(truth).astype(np.int32),(150,55,5))
        cv2.fillConvexPoly(im,np.rint(seed).astype(np.int32),(250,95,5))
        im=np.clip(cv2.GaussianBlur(im,(3,3),.7)*exposure,0,255).astype(np.uint8)
        candidate={'status':'complete_face_candidate','face_quad_px':seed.tolist(),
            'poses':pose_candidates(seed.tolist(),K.tolist(),D.tolist()),'boundary_method':'threshold_test_proposal'}
        return im,K,D,truth,candidate

    def test_complete_face_verified_across_exposure(self):
        for exposure in (.5,.8,1.,1.1):
            im,K,D,q,candidate=self.scene(0,exposure)
            result=validate_top_boundary(im,K,D,candidate)
            self.assertEqual(result['status'],'complete_face_candidate',result.get('reason'))
            self.assertTrue(result['top_boundary_verified'])
            self.assertLess(np.max(np.linalg.norm(np.array(result['face_quad_px'])-q,axis=1)),3)
            self.assertTrue(all(e['support']>=45 for e in result['physical_edges']))

    def test_cyclic_corner_order_keeps_background_edge_check(self):
        im,K,D,q,candidate=self.scene(1)
        for shift in (1,2,3):
            reordered=dict(candidate)
            reordered['face_quad_px']=np.roll(np.array(candidate['face_quad_px']),shift,axis=0).tolist()
            result=validate_top_boundary(im,K,D,reordered)
            self.assertEqual(result['status'],'complete_face_candidate',result.get('reason'))
            expected=np.roll(q,shift,axis=0)
            self.assertLess(np.max(np.linalg.norm(np.array(result['face_quad_px'])-expected,axis=1)),3)

    def test_inset_low_reprojection_proposal_is_remeasured_or_rejected(self):
        for inset in (1.,2.,3.):
            im,K,D,q,candidate=self.scene(inset)
            self.assertLess(candidate['poses'][0]['face_reprojection_rms_px'],.01)
            self.assertGreater(np.max(np.linalg.norm(np.array(candidate['face_quad_px'])-q,axis=1)),5)
            result=validate_top_boundary(im,K,D,candidate)
            if inset==1:self.assertEqual(result['status'],'complete_face_candidate')
            if result['status']=='complete_face_candidate':
                self.assertTrue(result['top_boundary_verified'])
                self.assertLess(np.max(np.linalg.norm(np.array(result['face_quad_px'])-q,axis=1)),3)
                self.assertNotEqual(result['face_quad_px'],candidate['face_quad_px'])
            else:self.assertIn('rejected_top_proposal',result)

    def test_internal_highlight_without_background_edge_is_rejected(self):
        im,K,D,q,candidate=self.scene(1)
        im[:]=(150,55,5)
        cv2.fillConvexPoly(im,np.rint(candidate['face_quad_px']).astype(np.int32),(250,95,5))
        result=validate_top_boundary(im,K,D,candidate)
        self.assertEqual(result['status'],'no_complete_face')
        self.assertIn('Top face boundary not verified',result['reason'])
        self.assertNotIn('poses',result)

    def test_missing_back_edge_and_clipped_face_are_rejected(self):
        im,K,D,q,candidate=self.scene()
        # Extend the blue face above the back edge; its missing edge is not
        # reconstructed from the known 20mm size or the other three edges.
        x0=int(q[:,0].min())-40;x1=int(q[:,0].max())+40
        im[:int(q[:2,1].max())+12,x0:x1]=(250,95,5)
        result=validate_top_boundary(im,K,D,candidate)
        self.assertEqual(result['status'],'no_complete_face')
        im,K,D,q,candidate=self.scene()
        im=cv2.warpAffine(im,np.array([[1.,0.,-600.],[0.,1.,0.]]),(1280,960),borderValue=(80,140,170))
        candidate['face_quad_px']=[[x-600,y] for x,y in candidate['face_quad_px']]
        result=validate_top_boundary(im,K,D,candidate)
        self.assertEqual(result['status'],'no_complete_face')

    def test_rotated_diamond_reaches_final_boundary_validation(self):
        from device.k230.pick.face_selection import shallow_face
        for yaw in (30.,45.,60.,135.,225.):
            im,K,D,q,candidate=self.scene(1,yaw=yaw)
            self.assertTrue(shallow_face(candidate['face_quad_px']))
            with patch('vision.fast_sides',return_value=None),patch('vision.basic_face',return_value=candidate),patch('vision.prefer_sides',return_value=None),patch('vision.split_face',return_value=None),patch('vision.independent_adjacent_side',return_value=None):
                result=localize(im,K.tolist(),D.tolist(),np.eye(4).tolist())
            self.assertTrue(result['top_boundary_verified'])
            self.assertLess(np.max(np.linalg.norm(np.array(result['face_quad_px'])-q,axis=1)),3)
            self.assertGreaterEqual(sum(e['background_required'] for e in result['physical_edges']),2)
            im[:]=(150,55,5)
            cv2.fillConvexPoly(im,np.rint(candidate['face_quad_px']).astype(np.int32),(250,95,5))
            with patch('vision.fast_sides',return_value=None),patch('vision.basic_face',return_value=candidate),patch('vision.prefer_sides',return_value=None),patch('vision.split_face',return_value=None),patch('vision.independent_adjacent_side',return_value=None):
                with self.assertRaisesRegex(ValueError,'Top face boundary not verified'):
                    localize(im,K.tolist(),D.tolist(),np.eye(4).tolist())

    def test_final_localization_cannot_bypass_top_validation(self):
        im,K,D,q,candidate=self.scene(1)
        with patch('vision.fast_sides',return_value=None),patch('vision.basic_face',return_value=candidate),patch('vision.split_face',return_value=None),patch('vision.independent_adjacent_side',return_value=None):
            result=localize(im,K.tolist(),D.tolist(),np.eye(4).tolist())
        self.assertTrue(result['top_boundary_verified'])
        self.assertLess(np.max(np.linalg.norm(np.array(result['face_quad_px'])-q,axis=1)),3)
        im[:]=(150,55,5)
        cv2.fillConvexPoly(im,np.rint(candidate['face_quad_px']).astype(np.int32),(250,95,5))
        with patch('vision.fast_sides',return_value=None),patch('vision.basic_face',return_value=candidate),patch('vision.split_face',return_value=None),patch('vision.independent_adjacent_side',return_value=None):
            with self.assertRaisesRegex(ValueError,'Top face boundary not verified'):
                localize(im,K.tolist(),D.tolist(),np.eye(4).tolist())


if __name__=='__main__':unittest.main()
