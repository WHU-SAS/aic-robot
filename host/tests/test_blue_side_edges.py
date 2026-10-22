"""Exercise OpenCV tests in the dedicated vision environment."""
import subprocess
import unittest
from host.project_paths import ROOT


class VisionEnvironmentTests(unittest.TestCase):
    def test_standalone_portable_vision_regressions(self):
        result=subprocess.run([str(ROOT/'scripts/vision'),'-m','unittest',
            'host.tests.vision_standalone_cases'],cwd=ROOT,text=True,capture_output=True)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)

    def test_physical_face_regressions(self):
        result=subprocess.run([str(ROOT/'scripts/vision'),'-m','unittest',
            'host.tests.vision_blue_side_edges_cases'],cwd=ROOT,text=True,capture_output=True)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)


if __name__=='__main__':unittest.main()
