"""Pure geometry/role tests; no OpenCV, camera, arm or calibration writes."""
import math
from pathlib import Path
import tempfile
import unittest

from tools.analysis.verify_table_plane import (normalized_plane,transform_plane,
    compare_planes,contact_role,manifest_entry,digest)
from device.k230.pick.numeric import rotation


class TablePlaneGeometryTests(unittest.TestCase):
    def test_observed_points_stay_on_the_transformed_plane_with_rotation_and_translation(self):
        R=rotation([0.,0.,1.],.7);translation=[40.,-20.,100.]
        # Include ordinary serialization rounding, not an exactly orthogonal R.
        R=[[round(v,6)for v in row]for row in R]
        T=[row+[translation[i]]for i,row in enumerate(R)]+[[0.,0.,0.,1.]]
        source={'normal':[.2,-.3,.8],'offset_mm':230.}
        plane=transform_plane(source,T)
        for x,y in((0.,0.),(35.,-20.),(-80.,60.)):
            z=(230-.2*x+.3*y)/.8
            point=[sum(R[i][j]*v for j,v in enumerate((x,y,z)))+translation[i]for i in range(3)]
            self.assertAlmostEqual(sum(n*v for n,v in zip(plane['normal'],point)),plane['offset_mm'],places=9)
        self.assertAlmostEqual(sum(v*v for v in plane['normal']),1.,places=12)

    def test_positive_negative_and_scaled_equations_describe_the_same_plane(self):
        plane={'normal':[0.,0.,2.],'offset_mm':-120.}
        opposite={'normal':[0.,0.,-3.],'offset_mm':180.}
        result=compare_planes(plane,opposite,[10.,20.,-60.])
        self.assertAlmostEqual(result['unoriented_normal_angle_deg'],0.)
        self.assertAlmostEqual(result['reference_plane_abs_residual_at_board_center_mm'],0.)
        tilted={'normal':[math.sin(.1),0.,math.cos(.1)],'offset_mm':-60.}
        self.assertAlmostEqual(compare_planes(plane,tilted,[0.,0.,-60.])['unoriented_normal_angle_deg'],math.degrees(.1))

    def test_translation_changes_plane_offset_with_the_required_positive_sign(self):
        T=[[1.,0.,0.,5.],[0.,1.,0.,8.],[0.,0.,1.,-30.],[0.,0.,0.,1.]]
        self.assertEqual(transform_plane({'normal':[0.,0.,1.],'offset_mm':250.},T),
                         {'normal':[0.,0.,1.],'offset_mm':220.})

    def test_handheld_and_unknown_roles_cannot_become_table_from_geometry_or_name(self):
        for entry in(None,{}, {'role':'handheld_board'}, {'role':'flat_looking_board'},
                     {'role':'observed_board_plane'}):
            self.assertEqual(contact_role(False,entry),'observed_board_plane')
        self.assertEqual(contact_role(True),'user_declared_table_contact')
        self.assertEqual(contact_role(False,{'role':'user_declared_table_contact'}),
                         'user_declared_table_contact')

    def test_manifest_sha_mismatch_cannot_promote_a_same_named_image(self):
        with tempfile.TemporaryDirectory()as directory:
            image=Path(directory)/'flat_board.jpg';image.write_bytes(b'original')
            declaration={'image':image.name,'image_sha256':'wrong',
                         'role':'user_declared_table_contact'}
            self.assertEqual(manifest_entry({'images':[declaration]},image),{})
            declaration['image_sha256']=digest(image)
            self.assertEqual(manifest_entry({'images':[declaration]},image),declaration)


if __name__=='__main__':unittest.main()
