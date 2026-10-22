"""Rigid transform correctness, signed residuals and predeclared holdout isolation."""
import unittest
import numpy as np
from tools.analysis.fit_formal_tip_top import rigid_fit,contact_plane,evaluate,fit_from_split


class FormalExtrinsicsTests(unittest.TestCase):
    def setUp(self):
        self.camera={1:np.array([0.,0.,0.]),2:np.array([30.,0.,0.]),
                     3:np.array([0.,40.,0.]),4:np.array([20.,30.,0.]),
                     5:np.array([7.,11.,0.])}
        self.rotation=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
        self.translation=np.array([30.,-40.,100.])
        self.base={g:self.rotation@p+self.translation for g,p in self.camera.items()}

    def test_planar_correspondences_recover_proper_rigid_transform(self):
        T,_,_,_=fit_from_split(self.camera,self.base,[1,2,3,4],[5])
        np.testing.assert_allclose(T[:3,:3],self.rotation,atol=1e-12)
        np.testing.assert_allclose(T[:3,3],self.translation,atol=1e-12)
        self.assertAlmostEqual(np.linalg.det(T[:3,:3]),1.)
        self.assertLess(evaluate(T,self.camera,self.base,[5])['rms_xyz_mm'],1e-10)

    def test_heldout_coordinates_cannot_change_fit(self):
        T,_,_,_=fit_from_split(self.camera,self.base,[1,2,3,4],[5])
        self.camera[5]=np.array([1e6,-1e6,2e6]);self.base[5]=np.array([-1e6,2e6,3e6])
        second,_,_,test=fit_from_split(self.camera,self.base,[1,2,3,4],[5])
        np.testing.assert_array_equal(T,second)
        self.assertEqual(test,[5])
        with self.assertRaises(ValueError):fit_from_split(self.camera,self.base,[1,2,3],[3,5])

    def test_collinear_or_nonfinite_points_rejected(self):
        points=np.array([[0.,0.,0.],[1.,0.,0.],[2.,0.,0.]])
        with self.assertRaises(ValueError):rigid_fit(points,points)
        points[1,0]=float('nan')
        with self.assertRaises(ValueError):rigid_fit(points,points)

    def test_table_offset_follows_up_normal_not_jaw_axis(self):
        points=np.array([[0.,0.,20.],[30.,0.,23.],[0.,40.,20.],[30.,40.,23.]])
        normal,offset,_=contact_plane(points)
        self.assertGreater(normal[2],0)
        centers=points-10*normal
        np.testing.assert_allclose(centers@normal,offset-10,atol=1e-10)

    def test_signed_errors_are_prediction_minus_reference(self):
        metrics=evaluate(np.eye(4),{1:np.array([4.,-3.,2.])},{1:np.zeros(3)},[1])
        self.assertEqual(metrics['groups'][0]['delta_mm'],[4.,-3.,2.])
        self.assertAlmostEqual(metrics['rms_xy_mm'],5.)


if __name__=='__main__':unittest.main()
