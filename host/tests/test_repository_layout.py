"""Migration regressions: numerical behavior and paths, never hardware access."""
import hashlib
import json
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
        p=json.loads((ROOT/'configs/arm/k230_pick_config.json').read_text())
        cls.geometry=PickGeometry(ROOT,cfg.mapping,table_z_m=p['table_z_Base_mm']/1000,
            hover_tip_height_mm=p['hover_tip_height_mm'],descend_tip_height_mm=p['descend_tip_height_mm'],
            lift_tip_height_mm=p['lift_tip_height_mm'],opening_count=p['opening_count'])
    def test_encoder_fk_matches_pre_migration(self):
        self.assertEqual([self.geometry.report(c) for c in self.frozen['counts']],self.frozen['reports'])
    def test_four_stage_ik_matches_pre_migration(self):
        self.assertEqual(self.geometry.plan(self.frozen['plan_xy_m'],[2048]*6),self.frozen['plan'])
    def test_current_model_and_control_resources_unchanged(self):
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
        folder=ROOT/'device/k230/vision'
        detector=(folder/'detect_blue_faces.py').read_text()
        marker='from blue_face_core import extract, median, component_boxes, native_rect_faces\n'
        self.assertIn(marker,detector)
        import ast
        code=(folder/'blue_face_core.py').read_text()+'\n'+detector.replace(marker,'')
        tree=ast.parse(code)
        imports=[n.module or '' for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)]
        self.assertFalse(any(n.startswith(('host','scipy','serial')) for n in imports))

if __name__=='__main__':unittest.main()
