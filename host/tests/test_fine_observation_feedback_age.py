"""An old lightweight whole-cube frame must not become a false bottom loss.

Real Task/Memory/Drive/native coarse logic, synthetic images, solver and UART
with explicit fake-clock costs; no physical timing or visual accuracy claim.
"""
import unittest

from host.tests.test_fine_observation_budget import CostScene
from host.tests.test_global_bottom_recovery_task import BottomScene


class FineObservationFeedbackAgeTests(unittest.TestCase):
    def whole_static_scene(self, classify_ms, next_call_ms):
        bottom = BottomScene(self)
        bottom.begin_reverse()
        f = bottom.f
        origin = list(f.task.drive._bottom_reverse_budget['reverse_start_counts'])
        scene = CostScene(self, base=f, slices=110, classify_ms=classify_ms)
        bottom.step('whole_far')
        scene.until(lambda: f.task.job is not None and f.task.job['purpose'] == 'static')
        session = f.task.job['session']
        cancelled = f.task.metrics['fine_cancelled']
        commands_at = len(f.motor.commands)

        def one_long_native_call(row):
            if len(scene.extra) == 1:
                f.fine_cost_ms = next_call_ms

        scene.on_extra = one_long_native_call
        return scene, origin, session, cancelled, commands_at

    def assert_same_stopped_recovery(self, scene, origin, session, cancelled, commands_at):
        f = scene.f
        self.assertEqual(f.task.metrics['fine_cancelled'], cancelled)
        self.assertIs(f.task.job['session'], session)
        self.assertEqual(f.task.drive._bottom_reverse_budget['reverse_start_counts'], origin)
        self.assertTrue(f.task.drive._bottom_reverse_budget['view_recovered'])
        self.assertEqual(f.motor.run, 0)
        self.assertFalse(f.handoffs)
        self.assertFalse(f.arms)
        self.assertFalse(any(command.startswith(('FOLLOW,', 'SPEED,', 'ARC,'))
                             for unused, command in f.motor.commands[commands_at:]))
        self.assertFalse(any(event == 'tracking_fine_observation_changed' and
                             details.get('reason') == 'bottom_view_lost'
                             for event, details in f.events))

    def test_whole_frame_expired_before_encoder_bracket_is_unknown_not_bottom_loss(self):
        scene, origin, session, cancelled, commands_at = self.whole_static_scene(10, 600)
        f = scene.f

        def expired_pending_released():
            if not scene.extra:
                return False
            row = scene.extra[0]
            pending = (f.task.job or {}).get('_light_pending')
            return (f.clock.now() - row['frame']['capture_timestamp_ms'] >
                    f.config['tracking']['max_age_ms'] and
                    (pending is None or pending['frame_id'] != row['frame']['frame_id']))

        scene.until(expired_pending_released)
        self.assertNotIn('target', scene.extra[0])
        self.assert_same_stopped_recovery(scene, origin, session, cancelled, commands_at)

    def test_whole_frame_expiring_during_classification_does_not_cancel_static_fine(self):
        scene, origin, session, cancelled, commands_at = self.whole_static_scene(100, 400)
        scene.until(lambda: bool(scene.extra) and 'target' in scene.extra[0])
        f = scene.f
        row = scene.extra[0]
        stamp = row['frame']['capture_timestamp_ms']
        limit = f.config['tracking']['max_age_ms']
        self.assertLessEqual(row['end'] - row['classify_cost_ms'] - stamp, limit)
        self.assertGreater(row['end'] - stamp, limit)
        self.assertEqual(row['target']['movement_state'], 'consistent')
        self.assertFalse(row['target']['confirmed_moved'])
        self.assert_same_stopped_recovery(scene, origin, session, cancelled, commands_at)


if __name__ == '__main__':
    unittest.main()
