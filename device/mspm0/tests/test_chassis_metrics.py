"""Check nominal units and actual 2x convention without hardware."""
from pathlib import Path
import json
import math
import sys
import unittest
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
from tools.diagnostics.chassis_metrics import geometry, displacement, signed_delta

class ChassisMetricsTests(unittest.TestCase):
    def test_nominal_one_turn_and_forward_signs(self):
        config=json.loads((ROOT/'configs/car/drive_nominal.json').read_text())
        g=geometry(config)
        self.assertEqual(g['counts_per_wheel_revolution'],28000)
        self.assertAlmostEqual(g['wheel_circumference_mm'],65*math.pi)
        start={'ms':0,'encoder_a':100,'encoder_b':100}
        end={'ms':1000,'encoder_a':28100,'encoder_b':-27900}
        d=displacement(config,start,end)
        self.assertAlmostEqual(d['left_distance_mm'],65*math.pi)
        self.assertAlmostEqual(d['right_distance_mm'],65*math.pi)
        self.assertFalse(d['calibrated'])
    def test_counter_wrap_and_reverse(self):
        self.assertEqual(signed_delta(-2147483647,2147483647),2)
        self.assertEqual(signed_delta(2147483647,-2147483647),-2)
        self.assertEqual(signed_delta(10,20),-10)
