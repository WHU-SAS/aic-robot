"""A bottom recovery stays active until its new static result is accepted.

These are real Task/BlueTracker/Memory/Drive decisions. The native blob images,
motor dynamics and precise pose are protocol fixtures, not hardware evidence.
"""
import unittest

from host.tests.test_global_bottom_recovery_task import BottomScene


class BottomRecoveryFineGapTests(unittest.TestCase):
    def paused_static(self):
        scene = BottomScene(self)
        scene.begin_reverse()
        fixture = scene.f
        self.addCleanup(fixture.task.close)
        point = fixture.projection.pixel_to_plane([650, 350], 10)
        fixture.x, fixture.y = point[:2]
        fixture.fail_every_fine = False
        fixture.fine_slices = 40
        for unused in range(15):
            scene.step('whole_far')
            if fixture.task.job is not None:
                break
        else:
            self.fail(fixture.task.snapshot())
        self.assertEqual(fixture.task.job['purpose'], 'static')
        self.assertEqual(fixture.motor.run, 0)
        self.assertTrue(fixture.task.drive._bottom_reverse_budget['view_recovered'])
        return scene

    def test_partial_or_missing_after_static_solve_resumes_same_reverse_budget(self):
        for mode in ('partial_lower', 'missing'):
            with self.subTest(current_view=mode):
                scene = self.paused_static()
                fixture = scene.f
                original = list(fixture.task.drive._bottom_reverse_budget['reverse_start_counts'])
                captures = len(scene.frames)
                fine_count = len(fixture.fine_start_states)
                scene.mode = mode
                for unused in range(100):
                    scene.step()
                    if fixture.task.complete is not None:
                        break
                else:
                    self.fail(fixture.task.snapshot())
                completed = fixture.task.complete
                old_frame = completed['observation']['frame_id']
                self.assertEqual(completed['purpose'], 'static')
                self.assertEqual(completed['observation']['status'], 'localized')
                self.assertEqual(completed['observation']['target_association']['status'], 'matched')
                # The view changed during Fine.advance, without obtaining an
                # intermediate image or overwriting the owned full-frame JPEG.
                self.assertEqual(len(scene.frames), captures)
                command_at = len(fixture.motor.commands)
                for unused in range(5):
                    scene.step()
                    if fixture.motor.run:
                        break
                commands = [command for at, command in fixture.motor.commands[command_at:]]
                self.assertIn('FOLLOW,-15,-15', commands)
                self.assertEqual((fixture.motor.left_speed, fixture.motor.right_speed), (-15, -15))
                self.assertEqual(fixture.task.drive._active_action, 'bottom_reverse')
                self.assertEqual(fixture.task.drive._bottom_reverse_budget['reverse_start_counts'], original)
                self.assertFalse(fixture.task.drive._bottom_reverse_budget.get('view_recovered'))
                self.assertEqual(len(fixture.fine_start_states), fine_count)
                self.assertIsNone(fixture.task.memory.last_precise)
                self.assertIsNone(fixture.task.drive.anchor)
                self.assertNotIn(old_frame, fixture.bind_attempts)
                self.assertFalse(fixture.handoffs)
                self.assertFalse(fixture.arms)
                for current in ('missing', 'partial_lower', 'partial_left', 'missing'):
                    emitted = scene.step(current)
                    self.assertNotIn('STOP', emitted)
                    self.assertTrue(all(not command.startswith(('FOLLOW,', 'SPEED,', 'ARC,')) or
                        command.split(',')[1:] == ['-15', '-15'] for command in emitted), emitted)
                    self.assertEqual(len(fixture.fine_start_states), fine_count)
                    self.assertEqual(fixture.task.drive._bottom_reverse_budget['reverse_start_counts'], original)
                self.assertIsNone(fixture.task.fault)

    def test_rejected_static_binding_does_not_release_the_reverse_budget(self):
        scene = self.paused_static()
        fixture = scene.f
        original = list(fixture.task.drive._bottom_reverse_budget['reverse_start_counts'])
        scene.mode = 'whole_weak'
        for unused in range(100):
            scene.step()
            if fixture.task.complete is not None:
                break
        else:
            self.fail(fixture.task.snapshot())
        completed = fixture.task.complete
        old_frame = completed['observation']['frame_id']
        self.assertEqual(completed['observation']['status'], 'localized')
        self.assertEqual(completed['observation']['target_association']['status'], 'matched')
        scene.step()
        current = scene.frames[-1]['evidence']
        self.assertEqual(current['edge_directions'], [])
        self.assertFalse(current['distance_valid'])
        self.assertEqual(completed['current_consistency'], 'unknown')
        self.assertIn(old_frame, fixture.bind_attempts)
        self.assertTrue(any(row['event'] == 'tracking_fine_binding_rejected' and
            row['details']['frame_id'] == old_frame for row in fixture.task.timeline))
        budget = fixture.task.drive._bottom_reverse_budget
        self.assertIsNotNone(budget)
        self.assertEqual(budget['reverse_start_counts'], original)
        self.assertIsNone(fixture.task.memory.last_precise)
        self.assertIsNone(fixture.task.drive.anchor)
        self.assertFalse(fixture.handoffs)
        command_at = len(fixture.motor.commands)
        for unused in range(5):
            scene.step('partial_lower')
            if fixture.motor.run:
                break
        emitted = [command for at, command in fixture.motor.commands[command_at:]]
        self.assertIn('FOLLOW,-15,-15', emitted)
        self.assertEqual(fixture.task.drive._bottom_reverse_budget['reverse_start_counts'], original)
        self.assertIsNone(fixture.task.fault)


if __name__ == '__main__':
    unittest.main()
