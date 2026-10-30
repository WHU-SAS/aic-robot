"""Nominal timing, short-time motion prediction and blended wheel commands."""
import math
import unittest
from device.k230.car.continuous_math import (wheel_speeds, predict_bearing,
                                               wheel_targets)


class ContinuousMathTests(unittest.TestCase):
    config = {'cruise_mm_s': 60, 'finish_mm_s': 20, 'max_wheel_mm_s': 80,
              'heading_gain_per_s': 1.5, 'slowdown_gain_per_s': 2}
    track = 119.2

    def test_wheel_speeds_use_mcu_elapsed_time_and_raw_channel_signs(self):
        old = {'ms': 100, 'encoder_a': 1000, 'encoder_b': -2000}
        new = {'ms': 200, 'encoder_a': 1500, 'encoder_b': -2400}
        self.assertEqual(wheel_speeds(old, new, .01),
                         {'left_mm_s': 50, 'right_mm_s': 40})
        # Neither absolute MCU uptime nor unrelated K230 timestamps are an age.
        offset_old = dict(old, ms=1000100, capture_timestamp_ms=17)
        offset_new = dict(new, ms=1000200, capture_timestamp_ms=900000)
        self.assertEqual(wheel_speeds(old, new, .01),
                         wheel_speeds(offset_old, offset_new, .01))

    def test_encoder_and_mcu_milliseconds_wrap(self):
        old = {'ms': 0xFFFFFFCE, 'encoder_a': 2147483640,
               'encoder_b': -2147483640}
        new = {'ms': 50, 'encoder_a': -2147483636,
               'encoder_b': 2147483636}
        self.assertEqual(wheel_speeds(old, new, .5),
                         {'left_mm_s': 100, 'right_mm_s': 100})

    def test_prediction_retains_centroid_bearing_and_midline_installation(self):
        for heading in (-25, 0, 20):
            for bearing in (-12, 0, 15):
                self.assertAlmostEqual(predict_bearing(bearing, 600, heading,
                                       0, 0, self.track), bearing)
        # The same ahead translation affects a mounted 20-degree midline too.
        expected = math.degrees(math.atan2(600 * math.sin(math.radians(20)),
                         600 * math.cos(math.radians(20)) - 10)) - 20
        self.assertAlmostEqual(predict_bearing(0, 600, 20, 10, 10,
                                              self.track), expected)
        self.assertGreater(expected, 0)

    def test_yaw_compensation_has_both_correct_signs(self):
        yaw = math.radians(3)
        # Pure nominal yaw: rotating left reduces a left target bearing.
        d = yaw * self.track / 2
        self.assertAlmostEqual(predict_bearing(10, 600, 13, -d, d,
                                               self.track), 7)
        self.assertAlmostEqual(predict_bearing(-10, 600, -13, d, -d,
                                               self.track), -7)

    def test_prediction_includes_curved_translation(self):
        left, right, bearing, heading, radius = 2, 8, 15, 12, 500
        yaw = (right - left) / self.track
        travel = (left + right) / 2
        theta = math.radians(bearing + heading)
        x = radius * math.cos(theta) - travel * math.sin(yaw) / yaw
        y = radius * math.sin(theta) - travel * (1 - math.cos(yaw)) / yaw
        expected = math.degrees(math.atan2(y, x) - yaw) - heading
        self.assertAlmostEqual(predict_bearing(bearing, radius, heading,
                                               left, right, self.track), expected)
        self.assertNotAlmostEqual(expected, bearing - math.degrees(yaw), places=3)
        # Mirror the whole geometry and wheel travel: prediction mirrors too.
        self.assertAlmostEqual(predict_bearing(-bearing, radius, -heading,
                                               right, left, self.track), -expected)

    def test_zero_bearing_equal_wheels_and_near_distance_slows(self):
        far = wheel_targets(0, 300, self.config, self.track)
        mid = wheel_targets(0, 20, self.config, self.track)
        near = wheel_targets(0, 10, self.config, self.track)
        self.assertEqual((far['left_mm_s'], far['right_mm_s']), (60, 60))
        self.assertEqual((mid['left_mm_s'], mid['right_mm_s']), (40, 40))
        self.assertEqual((near['left_mm_s'], near['right_mm_s']), (20, 20))
        self.assertEqual(far['omega_rad_s'], 0)

    def test_left_right_symmetry_blends_turn_and_forward_motion(self):
        for angle in (1, 10, 30, 60):
            left = wheel_targets(angle, 300, self.config, self.track)
            right = wheel_targets(-angle, 300, self.config, self.track)
            self.assertGreater(left['right_mm_s'], left['left_mm_s'])
            self.assertGreater(left['base_speed_mm_s'], 0)
            self.assertAlmostEqual(left['left_mm_s'], right['right_mm_s'])
            self.assertAlmostEqual(left['right_mm_s'], right['left_mm_s'])
            self.assertAlmostEqual(left['omega_rad_s'], -right['omega_rad_s'])
        self.assertLess(wheel_targets(60, 300, self.config, self.track)
                        ['base_speed_mm_s'], 60)

    def test_wheel_limits_never_reverse_and_report_limited_motion(self):
        for angle in (-179, -90, -30, 0, 30, 90, 179):
            for remaining in (5, 10, 20, 300):
                result = wheel_targets(angle, remaining, self.config, self.track)
                left, right = result['left_mm_s'], result['right_mm_s']
                self.assertGreaterEqual(min(left, right), 0)
                self.assertLessEqual(max(left, right), 80)
                self.assertAlmostEqual(result['base_speed_mm_s'], (left+right)/2)
                self.assertAlmostEqual(result['omega_rad_s'], (right-left)/self.track)
        limited_config = dict(self.config, max_wheel_mm_s=30)
        result = wheel_targets(0, 300, limited_config, self.track)
        self.assertEqual((result['left_mm_s'], result['right_mm_s']), (30, 30))


if __name__ == '__main__':
    unittest.main()
