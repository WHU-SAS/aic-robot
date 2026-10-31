"""The optional wrist profile consumes the accepted face without recapturing."""
import copy
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch
from host.tests.test_pick_handoff import load_app
from tools.deploy.k230_pick import build_bundle
from tools.deploy.k230_robot import build_bundle as robot_bundle

PROFILE='configs/arm/pick_wrist_rotation.json'


class WristHandoffTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config,_,_=build_bundle(PROFILE)

    def test_board_passes_exact_verified_side_result_to_shared_planner(self):
        app=load_app();config=copy.deepcopy(self.config)
        found={'cube_center_Base_mm':[248.,4.,-55.],
               'side_faces':[{'selected_pose':{'inward_face_normal_Camera':[.1,.9,.1]},'used_for_center':True}]}
        orientation={'status':'estimated','yaw_table_deg':17}
        estimate=Mock(return_value=orientation);geometry=Mock();geometry.plan.return_value={'wrist_alignment':{'status':'aligned'}}
        start=[2048]*6
        with patch.dict(sys.modules,{'cube_orientation':SimpleNamespace(estimate=estimate)}), \
                patch.object(app,'Geometry',return_value=geometry),patch.object(app,'log'), \
                patch.object(app,'perceive',side_effect=AssertionError('must reuse frame')), \
                patch.object(app,'arm_snapshot',side_effect=AssertionError('offline does not read')):
            app._run_target(config,found,False,start)
        estimate.assert_called_once_with(found,config['extrinsics']['T_Base_from_Camera_mm'],
            config['vision']['localization']['table_plane_Base_mm'])
        geometry.plan.assert_called_once_with([.248,.004],start,orientation)

    def test_default_never_estimates_a_wrist_direction(self):
        app=load_app();config=copy.deepcopy(self.config);config['pick'].pop('wrist_alignment_enabled')
        geometry=Mock();estimate=Mock(side_effect=AssertionError('default keeps wrist'))
        with patch.dict(sys.modules,{'cube_orientation':SimpleNamespace(estimate=estimate)}), \
                patch.object(app,'Geometry',return_value=geometry),patch.object(app,'log'):
            app._run_target(config,{'cube_center_Base_mm':[248.,4.,-55.]},False,[2048]*6)
        geometry.plan.assert_called_once_with([.248,.004],[2048]*6)
        estimate.assert_not_called()

    def test_combined_profile_preserves_car_calibration_and_control(self):
        baseline,_,_=robot_bundle('configs/arm/pick_validation_pc.json');rotating,files,_=robot_bundle()
        self.assertEqual(rotating['car'],baseline['car'])
        self.assertEqual(rotating['arm']['model'],baseline['arm']['model'])
        self.assertEqual(rotating['arm']['control'],baseline['arm']['control'])
        self.assertEqual(rotating['arm']['ready_control'],baseline['arm']['ready_control'])
        self.assertEqual(rotating['arm']['pick']['standalone_stage_policy'],baseline['arm']['pick']['standalone_stage_policy'])
        self.assertTrue(rotating['arm']['pick']['wrist_alignment_enabled'])
        self.assertNotIn('wrist_alignment_enabled',baseline['arm']['pick'])
        for name in ('cube_orientation.py','wrist_plan.py'):
            self.assertIn(name,files)


if __name__=='__main__':unittest.main()
