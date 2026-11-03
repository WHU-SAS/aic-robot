"""Distinct nonplanar corner evidence can orient the wrist without fake faces."""
import copy
import math
import unittest

from device.k230.pick.cube_orientation import estimate,plane_basis
from device.k230.pick.numeric import cross,dot,matmul,matvec,rotation,transpose

IDENTITY=[[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.]]
EVIDENCE='nonplanar_cube_five_corners'
METHOD='verified_nonplanar_cube_axes_fourfold_mean'
CANONICAL=[[-10.,0.,20.],[-10.,20.,20.],[10.,20.,20.],
           [10.,0.,0.],[-10.,0.,0.]]
CORNERS=[[-10.,0.,20.],[10.,0.,20.],[10.,0.,0.],[-10.,0.,0.],
         [-10.,20.,20.],[10.,20.,20.],[10.,20.,0.],[-10.,20.,0.]]


def fixture(yaw=17.,T=IDENTITY,normal=(0.,0.,1.),rms=.4):
    plane={'normal':list(normal)}
    x,y,n=plane_basis(plane);angle=math.radians(yaw)
    axes=[[math.cos(angle)*a+math.sin(angle)*b for a,b in zip(x,y)],
          [-math.sin(angle)*a+math.cos(angle)*b for a,b in zip(x,y)],n]
    base_rotation=[row[:3] for row in T[:3]]
    cube_rotation=matmul(transpose(base_rotation),transpose(axes))
    translation=[0.,0.,400.]
    def camera(point):return [a+b for a,b in zip(matvec(cube_rotation,point),translation)]
    def project(point):
        p=camera(point)
        return [640.+900.*p[0]/p[2],480.+900.*p[1]/p[2]]
    center=camera([0.,10.,10.])
    base_center=[sum(T[i][j]*center[j] for j in range(3))+T[i][3] for i in range(3)]
    measured=[[p[0]+rms,p[1]] for p in map(project,CANONICAL)]
    raw={'status':'complete_cube_candidate','evidence_type':EVIDENCE,
        'cube_center_Base_mm':base_center,'cube_pose':{
            'R_Object_to_Camera':cube_rotation,
            't_Object_to_Camera_mm':translation,
            'object_center_mm':[0.,10.,10.],'edge_length_mm':20.,
            'object_points_mm':copy.deepcopy(CANONICAL),'image_points_px':measured,
            'corner_reprojection_rms_px':rms,'cube_center_Camera_mm':center,
            'cube_projected_corners_px':[project(p) for p in CORNERS],
            'cube_projection_source':'pose_transform'}}
    found=dict(raw)
    found['observation']={'status':'localized','cube_center_Base_mm':list(base_center),
        'quality':{'evidence_type':EVIDENCE,'reprojection_rms_px':rms,
            'reprojection_limit_px':3.,'boundary_method':'six_measured_edges_five_corners',
            'vision_path':'shared_nonplanar_cube_corners'},'payload':copy.deepcopy(raw)}
    return found,plane


def legacy_face(yaw=17.,rms=.4):
    angle=math.radians(yaw)
    return {'used_for_center':True,'selected_pose':{
        'inward_face_normal_Camera':[math.cos(angle),math.sin(angle),0.],
        'face_reprojection_rms_px':rms}}


class CubeCornerOrientationTests(unittest.TestCase):
    def assert_yaw(self,result,yaw):
        self.assertEqual(result['status'],'estimated',result)
        self.assertEqual(result['method'],METHOD)
        self.assertEqual(result['period_deg'],90)
        self.assertEqual(result['axis_count'],2)
        self.assertAlmostEqual((result['yaw_table_deg']-yaw+45)%90-45,0.,places=7)
        self.assertLess(max(result['axis_residual_deg']),1e-7)
        self.assertAlmostEqual(dot(result['axes_Base'][0],result['axes_Base'][1]),0.)

    def test_cube_yaws_use_fourfold_mean_without_any_planar_face_fields(self):
        for yaw in (-134.,-89.,-45.,-12.,0.,17.,44.,90.,217.):
            with self.subTest(yaw=yaw):
                found,plane=fixture(yaw)
                self.assertNotIn('poses',found)
                self.assertNotIn('side_faces',found)
                self.assertNotIn('face_reprojection_rms_px',found['cube_pose'])
                result=estimate(found,IDENTITY,plane)
                self.assert_yaw(result,yaw)
                self.assertNotIn('side_count',result)

    def test_camera_rotation_and_tilted_table_are_not_confused_with_cube_yaw(self):
        camera=matmul(rotation([0.,0.,1.],math.radians(33)),
                      rotation([1.,0.,0.],math.radians(-12)))
        T=[row+[value] for row,value in zip(camera,[35.,-21.,70.])]+[[0.,0.,0.,1.]]
        for normal in ([.1,-.2,1.],[-.3,.1,1.],[.2,.3,-2.]):
            with self.subTest(normal=normal):
                found,plane=fixture(23.,T,normal)
                result=estimate(found,T,plane)
                self.assert_yaw(result,23.)
                expected=plane_basis(plane)[2]
                for a,b in zip(result['table_normal_Base'],expected):self.assertAlmostEqual(a,b)

    def test_three_pixel_limit_is_distinct_and_inclusive(self):
        for rms in (0.,2.4,3.):
            found,plane=fixture(rms=rms)
            self.assert_yaw(estimate(found,IDENTITY,plane),17.)
        found,plane=fixture(rms=3.001)
        self.assertEqual(estimate(found,IDENTITY,plane)['status'],'unavailable')

    def test_raw_and_quality_markers_and_rms_must_agree(self):
        for path,value in [
            (('status',),'complete_face_candidate'),
            (('evidence_type',),'physical_face'),
            (('observation','quality','evidence_type'),'physical_face'),
            (('observation','quality','boundary_method'),'four_edges'),
            (('observation','quality','vision_path'),'legacy_recovery'),
            (('observation','quality','reprojection_limit_px'),2.),
            (('observation','quality','reprojection_rms_px'),.400002),
            (('observation','quality','reprojection_rms_px'),float('nan')),
            (('cube_pose','cube_projection_source'),'invented'),
        ]:
            with self.subTest(path=path):
                found,plane=fixture()
                target=found
                for key in path[:-1]:target=target[key]
                target[path[-1]]=value
                found['side_faces']=[legacy_face()]
                self.assertEqual(estimate(found,IDENTITY,plane)['status'],'unavailable')
        found,plane=fixture()
        found['observation']['quality']['reprojection_rms_px']+=1e-7
        self.assert_yaw(estimate(found,IDENTITY,plane),17.)

    def test_corner_evidence_requires_localized_observation_capability(self):
        for status in (None,'failed','no_complete_face','tracking'):
            with self.subTest(status=status):
                found,plane=fixture();found['observation']['status']=status
                found['side_faces']=[legacy_face()]
                self.assertEqual(estimate(found,IDENTITY,plane)['status'],'unavailable')

    def test_present_observation_center_must_match_raw_center(self):
        for center in ([0.,0.], [True,0.,0.], [0.,0.,float('nan')],
                       [0.,0.,float('inf')], [0.,0.,0.]):
            with self.subTest(center=center):
                found,plane=fixture();found['observation']['cube_center_Base_mm']=center
                self.assertEqual(estimate(found,IDENTITY,plane)['status'],'unavailable')
        found,plane=fixture()
        found['observation']['cube_center_Base_mm'][0]+=2e-6
        self.assertEqual(estimate(found,IDENTITY,plane)['status'],'unavailable')
        found,plane=fixture()
        found['observation']['cube_center_Base_mm'][0]+=1e-7
        self.assert_yaw(estimate(found,IDENTITY,plane),17.)

    def test_observation_payload_and_optional_center_are_not_required_after_handoff(self):
        found,plane=fixture();found['observation'].pop('payload')
        self.assert_yaw(estimate(found,IDENTITY,plane),17.)
        found['observation'].pop('cube_center_Base_mm')
        self.assert_yaw(estimate(found,IDENTITY,plane),17.)

    def test_incomplete_corner_payload_never_borrows_legacy_side_orientation(self):
        for path in [('cube_pose',),('observation',),('observation','quality'),
                     ('cube_pose','R_Object_to_Camera'),('cube_pose','corner_reprojection_rms_px')]:
            with self.subTest(missing=path):
                found,plane=fixture();target=found
                for key in path[:-1]:target=target[key]
                target.pop(path[-1])
                found['side_faces']=[legacy_face()]
                self.assertEqual(estimate(found,IDENTITY,plane)['status'],'unavailable')
        # Even quality-only corner marking cannot silently consume old sides.
        found,plane=fixture()
        found={'side_faces':[legacy_face()], 'observation':found['observation']}
        self.assertEqual(estimate(found,IDENTITY,plane)['status'],'unavailable')

    def test_geometry_shapes_nonfinite_and_boolean_values_are_unavailable(self):
        values=[('R_Object_to_Camera',[[1.,0.],[0.,1.]]),
                ('R_Object_to_Camera',[[1.,0.,0.],[0.,1.,0.],[0.,0.,float('nan')]]),
                ('t_Object_to_Camera_mm',[0.,0.]),
                ('t_Object_to_Camera_mm',[0.,0.,float('inf')]),
                ('object_center_mm',[0.,10.,float('nan')]),
                ('cube_center_Camera_mm',[True,0.,400.]),
                ('object_points_mm',CANONICAL[:4]),
                ('image_points_px',[[0.,0.]]*4),
                ('image_points_px',[[0.,float('inf')]]*5),
                ('cube_projected_corners_px',[[0.,0.]]*7),
                ('cube_projected_corners_px',[[0.,False]]*8),
                ('corner_reprojection_rms_px',True),
                ('edge_length_mm',21.),('object_center_mm',[0.,10.,11.])]
        for key,value in values:
            with self.subTest(key=key,value=value):
                found,plane=fixture();found['cube_pose'][key]=value
                self.assertEqual(estimate(found,IDENTITY,plane)['status'],'unavailable')
        found,plane=fixture();found['cube_center_Base_mm']=[0.,0.,float('nan')]
        self.assertEqual(estimate(found,IDENTITY,plane)['status'],'unavailable')

    def test_reflection_scale_and_shear_are_not_proper_cube_rotations(self):
        for bad in ([[1.,0.,0.],[0.,1.,0.],[0.,0.,-1.]],
                    [[1.01,0.,0.],[0.,1.,0.],[0.,0.,1.]],
                    [[1.,.01,0.],[0.,1.,0.],[0.,0.,1.]]):
            with self.subTest(rotation=bad):
                found,plane=fixture();found['cube_pose']['R_Object_to_Camera']=bad
                found['side_faces']=[legacy_face()]
                self.assertEqual(estimate(found,IDENTITY,plane)['status'],'unavailable')

    def test_serialized_float_rotation_keeps_its_yaw(self):
        found,plane=fixture(23.)
        found['cube_pose']['R_Object_to_Camera']=[
            [float('%.7g'%v) for v in row] for row in found['cube_pose']['R_Object_to_Camera']]
        result=estimate(found,IDENTITY,plane)
        self.assertEqual(result['status'],'estimated')
        self.assertAlmostEqual(result['yaw_table_deg'],23.,places=5)

    def test_projection_does_not_add_cube_tilt_or_object_z_acceptance_rules(self):
        found,plane=fixture(0.)
        # Runtime already owns table/height acceptance. The wrist consumer
        # does not silently disable orientation by adding a second tilt gate.
        for angle in (19.9,20.1,40.):
            with self.subTest(angle=angle):
                found['cube_pose']['R_Object_to_Camera']=rotation([0.,1.,0.],math.radians(angle))
                self.assertEqual(estimate(found,IDENTITY,plane)['status'],'estimated')
        found['cube_pose']['R_Object_to_Camera']=rotation([1.,0.,0.],math.pi)
        self.assertEqual(estimate(found,IDENTITY,plane)['status'],'estimated')

    def test_degenerate_axis_projection_and_cancelling_fourfold_votes_are_unavailable(self):
        found,plane=fixture(0.)
        found['cube_pose']['R_Object_to_Camera']=rotation([0.,1.,0.],math.pi/2)
        self.assertEqual(estimate(found,IDENTITY,plane)['status'],'unavailable')
        # Proper orthogonal 3D axes can project45deg apart; those votes cancel
        # modulo90, so no artificial mean direction should be returned.
        x=[math.sqrt(.5),0.,math.sqrt(.5)]
        y=[1/math.sqrt(3),1/math.sqrt(3),-1/math.sqrt(3)]
        found['cube_pose']['R_Object_to_Camera']=transpose([x,y,cross(x,y)])
        self.assertEqual(estimate(found,IDENTITY,plane)['status'],'unavailable')

    def test_invalid_table_and_camera_metadata_keep_existing_unavailable_behavior(self):
        found,unused=fixture()
        for T,plane in [(None,{'normal':[0.,0.,1.]}),(IDENTITY,{}),
                        (IDENTITY,{'normal':[0.,0.,0.]}),([[1.]],{'normal':[0.,0.,1.]})]:
            with self.subTest(T=T,plane=plane):
                self.assertEqual(estimate(found,T,plane)['status'],'unavailable')

    def test_legacy_selected_side_path_still_requires_two_pixels(self):
        for rms,status in [(2.,'estimated'),(2.001,'unavailable')]:
            with self.subTest(rms=rms):
                found={'side_faces':[legacy_face(11.,rms)]}
                result=estimate(found,IDENTITY,{'normal':[0.,0.,1.]})
                self.assertEqual(result['status'],status)
                if status=='estimated':
                    self.assertEqual(result['method'],'verified_selected_side_normals_fourfold_mean')
                    self.assertEqual(result['side_count'],1)
                    self.assertAlmostEqual(result['yaw_table_deg'],11.)

    def test_estimation_does_not_mutate_or_relabel_input_evidence(self):
        found,plane=fixture(17.);before=copy.deepcopy(found);plane_before=copy.deepcopy(plane)
        self.assert_yaw(estimate(found,IDENTITY,plane),17.)
        self.assertEqual(found,before)
        self.assertEqual(plane,plane_before)


if __name__=='__main__':unittest.main()
