"""Pivot observability and independent validation; no hardware access."""
import json
import tempfile
import unittest
from pathlib import Path
import numpy as np
from tools.analysis.audit_tool_reference import fit_pivot
from tools.capture.collect_fixed_tip_top import freeze_session,build_script
from host.vision.config import load_vision_config


def rotation(axis,angle):
    c=np.cos(angle);s=np.sin(angle)
    if axis=='x':return np.array([[1,0,0],[0,c,-s],[0,s,c]])
    return np.array([[c,0,s],[0,1,0],[-s,0,c]])


def poses():
    p=np.array([2.,12.,108.]);C=np.array([250.,20.,-50.]);result={}
    for i,R in enumerate([np.eye(3),rotation('x',.6),rotation('y',-.5),rotation('x',-.4)@rotation('y',.3)],1):
        T=np.eye(4);T[:3,:3]=R;T[:3,3]=(C-R@p)/1000;result[i]=T
    return p,C,result


class PivotTests(unittest.TestCase):
    def test_recover_known_point_and_pivot(self):
        p,C,T=poses();result=fit_pivot(T)
        np.testing.assert_allclose(result['fitted_local_point_mm'],p,atol=1e-9)
        np.testing.assert_allclose(result['fitted_pivot_Base_mm'],C,atol=1e-9)
        self.assertLess(result['groups'][-1]['error_mm'],1e-9)

    def test_holdout_cannot_bias_fit(self):
        p,C,T=poses();T[4][:3,3]+=[.010,0,0];result=fit_pivot(T)
        np.testing.assert_allclose(result['fitted_local_point_mm'],p,atol=1e-9)
        self.assertAlmostEqual(result['groups'][-1]['error_mm'],10)

    def test_common_axis_is_unobservable(self):
        T={i:np.eye(4) for i in range(1,5)}
        for i,t in T.items():t[:3,:3]=rotation('x',i*.2)
        with self.assertRaisesRegex(ValueError,'not observable'):fit_pivot(T)

    def test_holdout_must_be_independent(self):
        with self.assertRaises(ValueError):fit_pivot(poses()[2],(1,2,3),(3,))

    def test_session_protocols_cannot_mix(self):
        with tempfile.TemporaryDirectory() as tmp:
            session=Path(tmp)
            (session/'session.json').write_text(json.dumps({'sample_type':'fixed_fingertip_cube_top_contact'}))
            with self.assertRaisesRegex(RuntimeError,'Cannot mix'):freeze_session(session,'tool-pivot')

    def test_pivot_manifest_declares_split_before_capture(self):
        with tempfile.TemporaryDirectory() as tmp:
            freeze_session(Path(tmp),'tool-pivot')
            info=json.loads((Path(tmp)/'session.json').read_text())
            self.assertEqual(info['training_groups'],[1,2,3]);self.assertEqual(info['validation_groups'],[4])
            self.assertNotIn('cube_center_definition',info)

    def test_capture_bundle_uses_pivot_semantics_and_read_only_transport(self):
        source=build_script(load_vision_config(),'/sdcard/tool_reference_pivot/test.jpg','tool-pivot')
        compile(source,'<bundle>','exec')
        self.assertIn("'fixed_point_pivot_contact')",source)
        self.assertIn('instruction not in (0x01, 0x02)',source)
        self.assertNotIn('enable_torque',source)


if __name__=='__main__':unittest.main()
