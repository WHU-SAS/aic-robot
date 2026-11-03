"""Bound actual follow travel while a moving fine image is still computing.

Real Task/Memory/Drive/BlueTracker with a planar plant and protocol image,
solver, UART and clock doubles. These regressions establish control flow and
nominal encoder travel, not physical accuracy or K230 timing guarantees.
"""
import unittest

from device.k230.car.near_control import target_x
from device.k230.car.parking_window import bounds_mm
from host.tests.test_fine_observation_budget import CostScene


class PendingFineMotionTests(unittest.TestCase):
    def scene(self, ratio, slices=100, fail_first=False, **costs):
        scene = CostScene(self, purpose='moving', ratio=ratio, slices=slices, **costs)
        f = scene.f
        f.fail_first_fine = fail_first
        scene.motion = []
        expect = f.motor.expect

        def record_command(command):
            job = f.task.job
            purpose = job['purpose'] if job is not None else None
            source = job['session'].frame['frame_id'] if job is not None else None
            result = expect(command)
            scene.motion.append(dict(command=command, at=f.clock.now(), x=f.x,
                purpose=purpose, source_frame=source, captured_frames=f.frame))
            return result

        f.motor.expect = record_command
        return scene

    def until(self, scene, predicate, limit=10000):
        f = scene.f
        for unused in range(limit):
            f.task.step()
            self.assertIsNone(f.task.fault, f.task.snapshot())
            if predicate():
                return
            # Match run_tracking: pending calculation has no artificial5ms idle.
            f.clock.advance(f.task.idle_delay_ms())
        self.fail(dict(phase=f.task.snapshot()['phase'], x=f.x,
                       commands=f.motor.commands[-10:]))

    def starts(self, scene, purpose):
        f = scene.f
        purposes = {row['frame']: row['purpose'] for row in f.fine_start_states}
        return [row for row in scene.starts if purposes[row['frame']['frame_id']] == purpose]

    def first_stop(self, scene):
        return next((row for row in scene.motion if row['command'] == 'STOP'), None)

    def assert_stop_then_new_static(self, scene, allow_pre_request_stop=False):
        f = scene.f
        self.until(scene, lambda: self.first_stop(scene) is not None)
        stop = self.first_stop(scene)
        near, far = bounds_mm(f.config['workspace'])['Base_X_mm']
        self.assertGreaterEqual(stop['x'], near)
        self.assertLessEqual(stop['x'], far)
        if allow_pre_request_stop:
            self.assertIn(stop['purpose'], (None, 'moving'))
        else:
            self.assertEqual(stop['purpose'], 'moving')
        self.assertFalse(f.handoffs)
        self.until(scene, lambda: bool(self.starts(scene, 'static')))
        static = self.starts(scene, 'static')[0]
        static_frame = static['frame']['frame_id']
        self.assertGreater(static['frame']['capture_timestamp_ms'], stop['at'])
        self.assertGreater(static_frame, stop['source_frame'] or stop['captured_frames'])
        start_state = next(row for row in f.fine_start_states if row['frame'] == static_frame)
        self.assertEqual(start_state['run'], 0)
        self.assertEqual(f.motor.run, 0)
        # Completion of the historical moving image cannot restart translation
        # before a new stopped image supplies the actual parking verdict.
        after = [row for row in scene.motion if stop['at'] < row['at'] <= static['at']]
        self.assertFalse(any(row['command'].startswith(('FOLLOW,', 'SPEED,', 'ARC,'))
                             for row in after), after)
        self.assertFalse(f.handoffs)
        self.assertFalse(f.arms)
        return stop, static

    def test_failed_first_solve_then_two_second_retry_stops_before_passing_the_window(self):
        for ratio in (0, .1):
            with self.subTest(ratio=ratio):
                scene = self.scene(ratio, fail_first=True)
                stop, static = self.assert_stop_then_new_static(scene)
                moving = self.starts(scene, 'moving')
                self.assertEqual(len(moving), 2)
                self.assertGreater(moving[0]['point'][0], moving[1]['point'][0])
                self.assertLess(moving[1]['point'][0] - target_x(scene.f.config['workspace']), 80)
                self.assertEqual(stop['source_frame'], moving[1]['frame']['frame_id'])
                self.assertGreaterEqual(scene.work[id(moving[1]['session'])], 2000)

    def test_new_near_target_uses_finite_travel_during_its_two_second_solve(self):
        for ratio in (0, .1):
            with self.subTest(ratio=ratio):
                scene = self.scene(ratio, slices=20)
                f = scene.f
                self.until(scene, lambda: f.task.complete is not None)
                old_frame = f.created_fines[0].frame['frame_id']
                f.motor.advance()
                f.x = target_x(f.config['workspace']) + 30
                f.y = sum(bounds_mm(f.config['workspace'], f.x)['Base_Y_mm']) / 2
                f.fine_slices = 100
                self.assert_stop_then_new_static(scene)
                moving = self.starts(scene, 'moving')
                self.assertEqual(len(moving), 2)
                self.assertNotIn(old_frame, f.bind_attempts)
                self.assertGreater(f.task.memory.version, 0)
                self.assertLess(moving[1]['point'][0] - target_x(f.config['workspace']), 30)
                self.assertGreaterEqual(scene.work[id(moving[1]['session'])], 2000)

    def test_light_observation_retarget_nearby_also_stops_before_the_solve_finishes(self):
        scene = self.scene(.1, slices=100)
        f = scene.f
        moved = []

        def move_nearer(row):
            if not moved:
                f.x = target_x(f.config['workspace']) + 30
                f.y = sum(bounds_mm(f.config['workspace'], f.x)['Base_Y_mm']) / 2
                moved.append(row['image_frame'])

        scene.on_extra = move_nearer
        self.assert_stop_then_new_static(scene, allow_pre_request_stop=True)
        self.assertEqual(len(moved), 1)
        self.assertEqual(len(self.starts(scene, 'moving')), 1)
        self.assertEqual(len(f.created_fines), 2)  # Original moving, then NEW static.
        self.assertNotIn(moved[0], f.bind_attempts)
        self.assertTrue(any(event == 'tracking_fine_observation_changed' and
                            details.get('continued_follow') is True
                            for event, details in f.events))

    def test_successful_first_solve_keeps_its_one_moving_anchor_and_normal_stop(self):
        for ratio in (0, .1):
            with self.subTest(ratio=ratio):
                scene = self.scene(ratio)
                f = scene.f
                self.until(scene, lambda: self.first_stop(scene) is not None)
                moving = self.starts(scene, 'moving')
                self.assertEqual(len(moving), 1)
                self.assertEqual(f.task.metrics['fine_cancelled'], 0)
                self.assertIn(moving[0]['frame']['frame_id'], f.bind_attempts)
                near, far = bounds_mm(f.config['workspace'])['Base_X_mm']
                self.assertGreaterEqual(self.first_stop(scene)['x'], near)
                self.assertLessEqual(self.first_stop(scene)['x'], far)
                self.assertFalse(f.handoffs)
                if ratio == 0:
                    self.assertFalse(scene.extra)

    def test_absent_distance_cannot_preserve_positive_forward_command(self):
        for ratio in (0, .1):
            with self.subTest(ratio=ratio):
                scene = self.scene(ratio)
                f = scene.f
                self.until(scene, lambda: f.task.job is not None)
                f.task.drive.anchor = f.task.drive._coarse_reference = None
                at = len(scene.motion)
                self.until(scene, lambda: any(row['command'].startswith('SPEED,')
                                               for row in scene.motion[at:]))
                commands = [row['command'] for row in scene.motion[at:]
                            if row['command'].startswith(('SPEED,', 'FOLLOW,'))]
                self.assertTrue(commands)
                for command in commands:
                    left, right = map(int, command.split(',')[1:])
                    self.assertEqual(left + right, 0, command)
                self.assertFalse(f.handoffs)
                self.assertIsNone(f.task.drive.anchor)
                self.assertIsNotNone(f.task.job)

    def test_pending_follow_helper_does_not_take_over_other_motion_owners(self):
        scene = self.scene(.1)
        f = scene.f
        self.until(scene, lambda: f.task.job is not None)
        drive = f.task.drive
        commands_at = len(f.motor.commands)
        for owner in ('arc', 'search_follow', 'recovery_reverse', None):
            with self.subTest(owner=owner):
                drive._active_action = owner
                self.assertIsNone(drive.pending_fine_motion(f.task.state, f.clock.now()))
        drive._active_action = 'follow'
        drive._motion_hold = 'existing_button_or_link_stop'
        self.assertIsNone(drive.pending_fine_motion(f.task.state, f.clock.now()))
        self.assertEqual(len(f.motor.commands), commands_at)

    def test_returned_bad_motor_reply_stops_before_more_native_work_and_stays_held(self):
        for ratio in (0, .1):
            with self.subTest(ratio=ratio):
                scene = self.scene(ratio)
                f = scene.f
                self.until(scene, lambda: f.task.job is not None)
                expect, advance = f.motor.expect, f.Fine.advance
                rejected, native_calls = [], []

                def returned_rejection(command):
                    result = expect(command)
                    if command.startswith('SPEED,') and not rejected:
                        rejected.append(dict(at=f.clock.now(), calls=len(native_calls)))
                        # The production MotorLink normally raises for this
                        # result. Exercise the existing Task/Drive hold rule
                        # when a transport returns the rejected running state.
                        return dict(result, result=2)
                    return result

                def record_native_call(session):
                    native_calls.append(f.clock.now())
                    return advance(session)

                f.motor.expect, f.Fine.advance = returned_rejection, record_native_call
                self.until(scene, lambda: self.first_stop(scene) is not None)
                stop = self.first_stop(scene)
                self.assertEqual(len(rejected), 1)
                self.assertEqual(stop['at'], rejected[0]['at'])
                self.assertEqual(len(native_calls), rejected[0]['calls'])
                self.assertEqual(f.task.drive._motion_hold, 'motor_reply_rejected')
                self.assertEqual(f.motor.run, 0)
                self.until(scene, lambda: f.task.job is None)
                # Include completed-result consumption and subsequent frames,
                # so the old moving result cannot silently restart the car.
                for unused in range(60):
                    f.task.step()
                    f.clock.advance(max(5, f.task.idle_delay_ms()))
                after = [row['command'] for row in scene.motion if row['at'] > stop['at']]
                self.assertNotIn('HB', after)
                self.assertFalse(any(command.startswith(('FOLLOW,', 'SPEED,', 'ARC,'))
                                     for command in after), after)
                self.assertEqual(f.task.drive._motion_hold, 'motor_reply_rejected')
                self.assertEqual(f.motor.run, 0)
                self.assertFalse(f.handoffs)
                self.assertFalse(f.arms)

    def test_near_retarget_stops_before_a_second_jpeg_or_native_call(self):
        scene = self.scene(.1, start_ms=235)
        f = scene.f
        moved, returned = [], []
        start, advance = f.Fine.start, f.Fine.advance

        def move_nearer(row):
            if not moved:
                f.x = target_x(f.config['workspace']) + 30
                f.y = sum(bounds_mm(f.config['workspace'], f.x)['Base_Y_mm']) / 2
                moved.append(row['image_frame'])

        def jpeg_return(session, *args):
            result = start(session, *args)
            returned.append(('jpeg', session.index, f.clock.now()))
            return result

        def advance_return(session):
            result = advance(session)
            returned.append(('advance', session.index, f.clock.now()))
            return result

        scene.on_extra = move_nearer
        f.Fine.start, f.Fine.advance = jpeg_return, advance_return
        self.until(scene, lambda: self.first_stop(scene) is not None)
        stop = self.first_stop(scene)
        # The existing route is already exhausted at the new request. Stop
        # before allocating or compressing another moving image at all.
        self.assertEqual(len(f.created_fines), 1)
        self.assertFalse(any(index == 1 and at <= stop['at']
                             for kind, index, at in returned))
        self.assertFalse(f.handoffs)
        self.assert_stop_then_new_static(scene, allow_pre_request_stop=True)
        self.assertEqual(len(self.starts(scene, 'moving')), 1)

    def test_exhausted_travel_stops_at_first_yield_of_long_native_advance(self):
        for ratio in (0, .1):
            for block_ms in (400, 780):
                for complete_on_return in (False, True):
                    with self.subTest(ratio=ratio, block_ms=block_ms,
                                      complete_on_return=complete_on_return):
                        scene = self.scene(ratio, fail_first=True)
                        f = scene.f
                        returned = []
                        advance = f.Fine.advance

                        def one_long_call(session):
                            block = session.index == 1 and session.ticks == 20
                            f.fine_cost_ms = block_ms if block else 20
                            if block and complete_on_return:
                                f.fine_slices = session.ticks + 1
                            result = advance(session)
                            if block:
                                returned.append(dict(at=f.clock.now(), frame=f.frame))
                            f.fine_cost_ms, f.fine_slices = 20, 100
                            return result

                        f.Fine.advance = one_long_call
                        self.until(scene, lambda: self.first_stop(scene) is not None)
                        stop = self.first_stop(scene)
                        self.assertEqual(len(returned), 1)
                        self.assertEqual(stop['at'], returned[0]['at'])
                        self.assertEqual(stop['captured_frames'], returned[0]['frame'])
                        if not complete_on_return:
                            self.assertEqual(stop['purpose'], 'moving')
                        # The780ms call may itself use more than the remaining
                        # distance. This asserts prompt action at its first yield,
                        # including a completed job before fresh coarse/STATUS,
                        # never a physical stop guarantee inside native execution.
                        self.assertFalse(f.handoffs)


if __name__ == '__main__':
    unittest.main()
