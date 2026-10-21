"""Migration regressions: numerical behavior and paths, never hardware access."""
import hashlib
import json
import subprocess
import tempfile
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
from host.project_paths import ROOT, bootstrap
bootstrap()
from task_controller import Configuration
from blue_pick_geometry import PickGeometry

class LayoutTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.frozen=json.loads((ROOT/'host/tests/fixtures/baseline_geometry.json').read_text())
        cfg=Configuration(ROOT)
        p=json.loads((ROOT/'demos/v0_blue_pick/pick.json').read_text())
        cls.geometry=PickGeometry(ROOT,cfg.mapping,table_z_m=p['table_z_Base_mm']/1000,
            hover_tip_height_mm=p['hover_tip_height_mm'],descend_tip_height_mm=p['descend_tip_height_mm'],
            lift_tip_height_mm=p['lift_tip_height_mm'],opening_count=p['opening_count'])
    def test_encoder_fk_matches_pre_migration(self):
        self.assertEqual([self.geometry.report(c) for c in self.frozen['counts']],self.frozen['reports'])
    def test_four_stage_ik_matches_pre_migration(self):
        self.assertEqual(self.geometry.plan(self.frozen['plan_xy_m'],[2048]*6),self.frozen['plan'])
    def test_current_model_and_control_resources_unchanged(self):
        # PID checksum tracks the user-approved baseline; updated after the
        # 2026-10-20 K230 double-read adoption (P1/P5=24). Geometry stays frozen.
        for name,digest in self.frozen['protected_sha256'].items():
            with self.subTest(name=name):
                self.assertEqual(hashlib.sha256((ROOT/name).read_bytes()).hexdigest(),digest)
    def test_keil_source_include_library_and_scatter_paths_exist(self):
        for project in ROOT.glob('device/mspm0/*/keil/*.uvprojx'):
            for node in ET.parse(project).iter():
                tag=node.tag.split('}')[-1];text=node.text or ''
                paths=text.split(';') if tag=='IncludePath' else [text] if tag in ['FilePath','ScatterFile','Misc'] else []
                for name in paths:
                    if name:
                        self.assertTrue((project.parent/name.replace(chr(92),'/')).exists(),str(project)+': '+name)
    def test_pc_sdk_is_runtime_patch_not_reference(self):
        import scservo_sdk
        self.assertEqual(Path(scservo_sdk.__file__).resolve().parent,ROOT/'third_party/ftservo_python/scservo_sdk')
    def test_board_source_bundle_has_no_host_dependency(self):
        from host.transport.board_vision_source import build_capture_script
        import ast
        tree=ast.parse(build_capture_script())
        imports=[]
        for node in ast.walk(tree):
            if isinstance(node,ast.ImportFrom):imports.append(node.module or '')
            elif isinstance(node,ast.Import):imports.extend(a.name for a in node.names)
        self.assertFalse(any(n.startswith(('host','scipy','serial','cv2','numpy')) for n in imports))
        self.assertFalse(any(n in ('blue_face_core','blue_detector','camera') for n in imports))


    def test_configuration_reference_paths_exist(self):
        keys={'pid_baseline','task_entry','default_pick_entry','source_provenance',
              'entry','motion_defaults_reference','intrinsics','extrinsics'}
        for config in (ROOT/'configs/arm').glob('*.json'):
            for key,value in json.loads(config.read_text()).items():
                if key in keys:
                    self.assertTrue((ROOT/value).is_file(),f'{config.name}: {key}={value}')
        provenance=json.loads((ROOT/'configs/vision/runtime/provenance.json').read_text())
        for model in provenance['models'].values():
            self.assertEqual(hashlib.sha256((ROOT/model['runtime']).read_bytes()).hexdigest(),model['sha256'])

    def test_analysis_help_needs_no_local_capture_data(self):
        for tool in ['tools/calibration/audit_intrinsics.py',
                     'tools/calibration/recheck_intrinsics.py',
                     'tools/analysis/replay_onboard_detector.py']:
            with self.subTest(tool=tool):
                result=subprocess.run([str(ROOT/'scripts/vision'),str(ROOT/tool),'--help'],
                                      cwd=ROOT,text=True,capture_output=True)
                self.assertEqual(result.returncode,0,result.stderr)

    def test_replay_reports_the_requested_missing_image(self):
        with tempfile.TemporaryDirectory() as folder:
            run=Path(folder)/'missing_run';output=Path(folder)/'result'
            result=subprocess.run([str(ROOT/'scripts/vision'),
                str(ROOT/'tools/analysis/replay_onboard_detector.py'),str(run),'--output',str(output)],
                cwd=ROOT,text=True,capture_output=True)
            self.assertEqual(result.returncode,1,result.stderr)
            rows=json.loads((output/'summary.json').read_text())
            self.assertEqual(rows[0]['run'],str(run))
            self.assertIn(str(run/'frame.jpg'),rows[0]['reason'])

if __name__=='__main__':unittest.main()
