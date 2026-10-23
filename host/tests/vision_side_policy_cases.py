"""Side selection is driven by measured faces, not the brightest blue patch."""
import sys
import unittest
from unittest.mock import patch
import cv2
import numpy as np
from host.project_paths import ROOT
from device.k230.pick.side_faces import detect_sides,select_sides,prefer_sides,shared_edge
sys.path.insert(0,str(ROOT/'device/k230/pick'))
from vision import basic_face,localize


class SidePolicyCases(unittest.TestCase):
    def scene(self,yaw=.7,exposure=1.):
        K=np.array([[1200.,0.,640.],[0.,1600.,580.],[0.,0.,1.]])
        D=np.zeros(5);rv=np.array([.4,yaw,.02]);tv=np.array([-10.,-10.,250.])
        front=np.array([[0.,0.,0.],[20.,0.,0.],[20.,20.,0.],[0.,20.,0.]])
        right=np.array([[20.,0.,0.],[20.,0.,20.],[20.,20.,20.],[20.,20.,0.]])
        top=np.array([[0.,0.,20.],[20.,0.,20.],[20.,0.,0.],[0.,0.,0.]])
        project=lambda p:cv2.projectPoints(p,rv,tv,K,D)[0].reshape(-1,2)
        im=np.full((960,1280,3),(80,140,170),np.uint8)
        for face,color in [(top,(245,99,0)),(right,(145,36,1)),(front,(180,120,10))]:
            cv2.fillConvexPoly(im,np.rint(project(face)).astype(np.int32),color)
        im=np.clip(cv2.GaussianBlur(im,(3,3),.7)*exposure,0,255).astype(np.uint8)
        truth=cv2.Rodrigues(rv)[0]@np.array([10.,10.,10.])+tv
        return im,K,D,truth

    def test_two_measured_sides_across_yaw_and_exposure(self):
        for yaw,exposure in ((.7,1.),(.85,.7),(.95,1.)):
            im,K,D,truth=self.scene(yaw,exposure)
            anchor=basic_face(cv2.cvtColor(im,cv2.COLOR_BGR2HSV),K.tolist(),D.tolist())
            self.assertIsNotNone(anchor)
            faces=detect_sides(im,K.tolist(),D.tolist(),anchor)
            self.assertEqual(len(faces),2)
            result=select_sides(faces,K.tolist(),D.tolist())
            ratio=max(f['area_px2'] for f in faces)/min(f['area_px2'] for f in faces)
            self.assertEqual(result['side_selection'],'mean_of_similar_area_sides' if ratio<=1.25 else 'larger_verified_side')
            self.assertEqual(result['selected_side_count'],2 if ratio<=1.25 else 1)
            np.testing.assert_allclose(result['poses'][0]['cube_center_Camera_mm'],truth,atol=3)
            for face in result['side_faces']:
                self.assertTrue(all(e['support']>=45 for e in face['physical_edges']))

    def faces(self,ratio=1.25,separation=2.):
        def face(q,center,normal):
            return {'face_quad_px':q,'area_px2':abs(cv2.contourArea(np.array(q,np.float32))),
                'physical_edges':[], 'poses':[{'cube_center_Camera_mm':center,
                'inward_face_normal_Camera':normal,'face_reprojection_rms_px':.5}],
                'boundary_method':'test_measured_side'}
        a=face([[0.,0.],[100.,0.],[100.,100.],[0.,100.]],[0.,0.,250.],[1.,0.,0.])
        b=face([[100.,0.],[100.+100/ratio,0.],[100.+100/ratio,100.],[100.,100.]],[separation,0.,250.],[0.,0.,1.])
        return a,b

    def test_exact_area_boundary_averages_centers_not_corners(self):
        K=[[1000.,0.,640.],[0.,1000.,480.],[0.,0.,1.]]
        for ratio,rule in ((1.25,'mean_of_similar_area_sides'),(1.26,'larger_verified_side')):
            a,b=self.faces(ratio)
            result=select_sides([b,a],K,[0.]*5)
            self.assertEqual(result['side_selection'],rule)
            self.assertEqual(result['face_quad_px'],a['face_quad_px'])
            expected=[1.,0.,250.] if ratio<=1.25 else [0.,0.,250.]
            self.assertEqual(result['poses'][0]['cube_center_Camera_mm'],expected)

    def test_conflicting_centers_and_nonorthogonal_faces_do_not_average(self):
        K=[[1000.,0.,640.],[0.,1000.,480.],[0.,0.,1.]]
        a,b=self.faces(separation=9.)
        with self.assertRaisesRegex(ValueError,'disagree'):select_sides([a,b],K,[0.]*5)
        a,b=self.faces();b['poses'][0]['inward_face_normal_Camera']=[1.,0.,0.]
        with self.assertRaisesRegex(ValueError,'disagree'):select_sides([a,b],K,[0.]*5)
        b['face_quad_px']=[[x+300,y] for x,y in b['face_quad_px']]
        self.assertFalse(shared_edge(a['face_quad_px'],b['face_quad_px']))
        self.assertEqual(select_sides([a,b],K,[0.]*5)['selected_side_count'],1)

    def test_no_side_is_created_from_uniform_blue_or_missing_edges(self):
        im,K,D,truth=self.scene();anchor=basic_face(cv2.cvtColor(im,cv2.COLOR_BGR2HSV),K.tolist(),D.tolist())
        im[:]=(150,55,5)
        self.assertIsNone(prefer_sides(im,K.tolist(),D.tolist(),anchor))

    def test_missing_bottoms_cannot_be_completed_from_known_cube_size(self):
        im,K,D,truth=self.scene()
        anchor=basic_face(cv2.cvtColor(im,cv2.COLOR_BGR2HSV),K.tolist(),D.tolist())
        faces=detect_sides(im,K.tolist(),D.tolist(),anchor)
        self.assertEqual(len(faces),2)
        cut=int(max(p[1] for f in faces for p in f['face_quad_px'][:2]))+12
        im[cut:,:]=(150,55,5)
        self.assertEqual(detect_sides(im,K.tolist(),D.tolist(),anchor),[])
        a,b=self.faces()
        duplicate=[[x+2,y] for x,y in a['face_quad_px']]
        self.assertFalse(shared_edge(a['face_quad_px'],duplicate))

    def test_dark_side_direct_path_does_not_run_expensive_legacy_searches(self):
        for exposure in (.35,.45,.6):
            im,K,D,truth=self.scene(.7,exposure)
            with patch('vision.dim_face',side_effect=AssertionError('Repeated dim scan')),patch('vision.split_face',side_effect=AssertionError('Repeated top/side scan')):
                result=localize(im,K.tolist(),D.tolist(),np.eye(4).tolist())
            self.assertEqual(result['vision_path'],'direct_component_sides')
            np.testing.assert_allclose(result['cube_center_Base_mm'],truth,atol=3)
            self.assertEqual(result['selected_side_count'],2)

    def test_direct_path_rejects_missing_edges_and_a_reflection_strip(self):
        im,K,D,truth=self.scene();im[:]=(80,140,170)
        cv2.rectangle(im,(600,750),(700,959),(100,35,5),-1)
        with self.assertRaises(ValueError):localize(im,K.tolist(),D.tolist(),np.eye(4).tolist())
        im,K,D,truth=self.scene();im[545:,:]=(150,55,5)
        from device.k230.pick.side_faces import fast_sides
        self.assertIsNone(fast_sides(im,K.tolist(),D.tolist(),cv2.cvtColor(im,cv2.COLOR_BGR2HSV)))

    def test_localization_uses_sides_before_a_bright_top(self):
        im,K,D,truth=self.scene()
        with patch('vision.validate_top_boundary',side_effect=AssertionError('Side result must not use top fallback')):
            result=localize(im,K.tolist(),D.tolist(),np.eye(4).tolist())
        self.assertEqual(result['side_selection'],'mean_of_similar_area_sides')
        np.testing.assert_allclose(result['cube_center_Base_mm'],truth,atol=3)


class TablePoseCases(unittest.TestCase):
    def table(self,reverse=False):
        from device.k230.pick.table_pose import context
        T=[[1.,0.,0.,10.],[0.,0.,1.,20.],[0.,-1.,0.,30.],[0.,0.,0.,1.]]
        return context(T,{'normal':[0.,0.,-2. if reverse else 2.],
                          'offset_mm':-20. if reverse else 20.})

    def pose(self,normal=(0.,0.,1.),height_y=10.,rms=1.):
        return {'inward_face_normal_Camera':list(normal),
                'cube_center_Camera_mm':[0.,height_y,250.],
                'face_reprojection_rms_px':rms}

    def test_lower_pixel_error_does_not_select_tilted_cube_side(self):
        from device.k230.pick.table_pose import constrain_face
        wrong=self.pose((0.,.6,.8),rms=.3);right=self.pose(rms=1.8)
        source={'poses':[wrong,right]};r=constrain_face(source,self.table())
        self.assertEqual(r['poses'][0]['original_pose_rank'],1)
        self.assertEqual(r['poses'][0]['cube_center_Camera_mm'],right['cube_center_Camera_mm'])
        self.assertFalse(r['table_pose_candidates'][0]['table_geometry']['accepted'])
        self.assertEqual(source['poses'],[wrong,right])

    def test_cube_height_rejects_inconsistent_pose_without_clamping(self):
        from device.k230.pick.table_pose import constrain_face
        bad=self.pose(height_y=-5.)
        self.assertIsNone(constrain_face({'poses':[bad]},self.table()))
        self.assertEqual(bad['cube_center_Camera_mm'],[0.,-5.,250.])
        self.assertIsNone(constrain_face({'poses':[self.pose(rms=2.1)]},self.table()))

    def test_plane_scale_sign_and_camera_rotation_are_respected(self):
        from device.k230.pick.table_pose import metrics
        a=metrics(self.pose(),self.table(),'side');b=metrics(self.pose(),self.table(True),'side')
        self.assertEqual(a,b);self.assertEqual(a['center_height_mm'],10.)
        self.assertAlmostEqual(a['normal_deviation_deg'],0.)

    def test_visible_top_inward_normal_must_point_toward_table(self):
        from device.k230.pick.table_pose import constrain_face
        down=self.pose((0.,1.,0.));up=self.pose((0.,-1.,0.))
        result=constrain_face({'poses':[up,down]},self.table(),'top')
        self.assertEqual(result['poses'][0]['original_pose_rank'],1)

    def test_adjacent_side_average_uses_table_validated_poses(self):
        from device.k230.pick.table_pose import context
        a,b=SidePolicyCases().faces(ratio=1.25)
        for face in (a,b):
            wrong=dict(face['poses'][0]);wrong['inward_face_normal_Camera']=[0.,.7,.7141428]
            wrong['face_reprojection_rms_px']=.1;face['poses'].insert(0,wrong)
        table=context(np.eye(4).tolist(),{'normal':[0.,-1.,0.],'offset_mm':-10.})
        r=select_sides([a,b],[[1000.,0.,640.],[0.,1000.,480.],[0.,0.,1.]],[0.]*5,table)
        self.assertEqual(r['side_selection'],'mean_of_similar_area_sides')
        self.assertEqual(r['poses'][0]['cube_center_Camera_mm'],[1.,0.,250.])
        self.assertTrue(all(f['selected_pose']['original_pose_rank']==1 for f in r['side_faces']))

    def test_legacy_exit_cannot_bypass_table_geometry(self):
        im=np.zeros((960,1280,3),np.uint8)
        K=[[1550.,0.,640.],[0.,1550.,480.],[0.,0.,1.]]
        face={'face_quad_px':[[500.,500.],[600.,500.],[600.,650.],[500.,650.]],
              'poses':[self.pose((0.,.7,.7141428),height_y=0.,rms=.1)]}
        table={'normal':[0.,-1.,0.],'offset_mm':-10.}
        with patch('vision.fast_sides',return_value=None),patch('vision.basic_face',return_value=face), \
             patch('vision.prefer_sides',return_value=None),patch('vision.shallow_face',return_value=False), \
             patch('vision.internal_seam',return_value=None):
            with self.assertRaisesRegex(ValueError,'consistent with upright cube'):
                localize(im,K,[0.]*5,np.eye(4).tolist(),table)

    def test_near_frontal_corner_noise_does_not_flip_to_mirrored_tilt(self):
        from device.k230.pick.pose import pose_candidates,project
        from device.k230.pick.table_pose import context,constrain_face
        angle=-np.deg2rad(16.);R=cv2.Rodrigues(np.array([angle,0.,0.]))[0]
        t=np.array([0.,25.,260.]);up=-R[:,1]
        T=np.eye(4);T[:3,:3]=np.array([R[:,0],R[:,2],up])
        table=context(T.tolist(),{'normal':[0.,0.,1.],'offset_mm':float(up@t-10.)})
        K=[[1550.,0.,640.],[0.,1550.,480.],[0.,0.,1.]];D=[0.]*5
        q=np.array([project((R@p+t).tolist(),K,D) for p in
                    ([-10.,-10.,0.],[10.,-10.,0.],[10.,10.,0.],[-10.,10.,0.])])
        noise=np.array([[.3,-.7],[-.6,.4],[.8,-.3],[-.4,.6]])
        truth=t+10*R[:,2]
        for scale in (0.,.5,1.,-1.):
            poses=pose_candidates((q+scale*noise).tolist(),K,D,refine=True)
            result=constrain_face({'poses':poses},table)
            self.assertIsNotNone(result)
            best=result['poses'][0]
            self.assertLess(best['table_geometry']['normal_deviation_deg'],10.)
            np.testing.assert_allclose(best['cube_center_Camera_mm'],truth,atol=3.)

    def test_full_image_table_constraints_preserve_rotated_two_side_policy(self):
        for yaw,exposure in ((.7,1.),(.85,.45),(.95,1.)):
            im,K,D,truth=SidePolicyCases().scene(yaw,exposure)
            R=cv2.Rodrigues(np.array([.4,yaw,.02]))[0]
            up=-R[:,1];T=np.eye(4);T[:3,:3]=np.array([R[:,0],R[:,2],up])
            table={'normal':[0.,0.,1.],'offset_mm':float(up@np.array([-10.,-10.,250.])-20.)}
            result=localize(im,K.tolist(),D.tolist(),T.tolist(),table)
            self.assertIn(result['side_selection'],('mean_of_similar_area_sides','larger_verified_side'))
            np.testing.assert_allclose(result['cube_center_Base_mm'],T[:3,:3]@truth,atol=3.)
            self.assertTrue(result['poses'][0]['table_geometry']['accepted'])


if __name__=='__main__':unittest.main()
