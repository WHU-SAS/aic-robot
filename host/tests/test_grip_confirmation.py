"""Signed per-goal axis6 confirmation boundaries; no transport or hardware."""
import unittest
from device.k230.arm.pick_stage import classify_grip


class GripConfirmationTests(unittest.TestCase):
    def test_29_fails_and_30_is_inclusively_successful_for_each_goal(self):
        for goal in (2138,2159):
            for delta,expected in ((-40,False),(0,False),(29,False),(30,True),(31,True)):
                with self.subTest(goal=goal,delta=delta):
                    result=classify_grip(goal,[goal+delta]*5)
                    self.assertEqual(result['axis'],6)
                    self.assertEqual(result['goal'],goal)
                    self.assertEqual(result['position'],goal+delta)
                    self.assertEqual(result['delta_counts'],delta)
                    self.assertEqual(result['threshold_counts'],30)
                    self.assertIs(result['grasp_confirmed'],expected)
                    self.assertEqual(result['method'],'axis6_stable_goal_delta')

    def test_last_sample_is_used_instead_of_mean_or_extreme(self):
        goal=2155
        result=classify_grip(goal,[goal+31,goal+30,goal+31,goal+30,goal+29])
        self.assertEqual(result['position'],goal+29)
        self.assertEqual(result['delta_counts'],29)
        self.assertIs(result['grasp_confirmed'],False)


if __name__=='__main__':unittest.main()
