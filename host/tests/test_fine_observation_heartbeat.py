"""Optional coarse work must yield to the existing motor heartbeat schedule.

Uses real Task/Memory/Drive with explicit fake-clock image/solver/UART costs.
This is a scheduling boundary regression, not a K230 worst-case timing claim.
"""
import unittest

from host.tests.test_fine_observation_budget import CostScene


class FineObservationHeartbeatTests(unittest.TestCase):
    def renewal_gaps(self, ratio):
        scene = CostScene(self, purpose='moving', ratio=ratio, slices=60)
        f = scene.f
        scene.until(lambda: f.task.job is not None)
        advance = f.Fine.advance

        def one_long_native_call(session):
            # This nonpreemptible call itself still permits the original next
            # heartbeat before the firmware's existing one-second lease.
            f.fine_cost_ms = 780 if session.ticks == 27 else 20
            return advance(session)

        f.Fine.advance = one_long_native_call
        renewed = [max(at for at, command in f.motor.commands
                       if command.split(',')[0] in ('FOLLOW', 'SPEED', 'HB'))]
        gaps = []
        expect = f.motor.expect

        def record_original_command(command):
            if command.split(',')[0] in ('FOLLOW', 'SPEED', 'HB'):
                gaps.append(f.clock.now() - renewed[0])
                renewed[0] = f.clock.now()
            return expect(command)

        f.motor.expect = record_original_command
        scene.first_complete()
        return scene, gaps

    def test_optional_observation_does_not_push_valid_native_gap_over_motor_lease(self):
        pure, pure_gaps = self.renewal_gaps(0)
        observed, observed_gaps = self.renewal_gaps(.1)
        self.assertFalse(pure.extra)
        self.assertTrue(observed.extra)
        # Before the fix optional capture+track added30ms at an already-due
        # heartbeat: the pure-fine985ms interval became1015ms.
        self.assertEqual(max(pure_gaps), 985)
        self.assertLessEqual(max(observed_gaps), max(pure_gaps))
        self.assertLess(max(observed_gaps), 1000)
        self.assertEqual(observed.f.task.metrics['fine_cancelled'], 0)
        self.assertIsNone(observed.f.task.fault)


if __name__ == '__main__':
    unittest.main()
