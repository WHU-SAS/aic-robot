"""A cleanup interruption must not replace the latest run with an older task.

Camera, task and file objects are protocol doubles. These tests exercise the
real app finally/ownership order; they do not execute hardware or localization.
"""
import json
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, mock_open, patch

from host.tests.test_robot_app import app


class InterruptedArchiveTests(unittest.TestCase):
    def run_case(self, step_error=None, close_error=None, camera_error=None,
                 link_error=None, light_error=None, archive_error=None, borrowed=False):
        board, trace = app(), []
        item = dict(label='original', jpeg=b'original-jpeg', frame={'frame_id': 73},
                    observation={'status': 'localized'})
        frames_seen, tasks = [], []
        timeline = [{'event': 'tracking_target', 'at_ms': 752000,
                     'details': {'reason': 'held_target_moved'}}]

        def fail(stage, error):
            trace.append(stage)
            if error is not None:
                raise error

        class Task:
            status = 'running'
            def __init__(task, *args, **kwargs):
                task.metrics = {}
                task.frames = args[-1]
                task.frames.append(item)
                frames_seen.append(task.frames)
                tasks.append(task)
            def step(task):
                fail('step', step_error)
                task.status = 'succeeded'
            def idle_delay_ms(task):
                return 0
            def snapshot(task):
                trace.append('snapshot')
                return {'status': task.status, 'timeline': timeline}
            def close(task):
                fail('task_close', close_error)

        camera = SimpleNamespace(open=Mock(side_effect=lambda: trace.append('camera_open')),
            capture=Mock(side_effect=AssertionError('archive must not capture')),
            close=Mock(side_effect=lambda: fail('camera_close', camera_error)))
        link = SimpleNamespace(close=Mock(side_effect=lambda: fail('link_close', link_error)))
        tracker = SimpleNamespace(detect_frame=Mock(), reset=Mock())
        light = SimpleNamespace(off=Mock(side_effect=lambda: fail('light_off', light_error)))
        prepared = dict(arm=SimpleNamespace(), age_ms=lambda a,b:a-b,
            projection=object(), held_geometry=object(), BlueTracker=lambda *a: tracker,
            CameraSession=lambda *a: camera, MotorLink=lambda: link, TrackingTask=Task,
            FineSession=Mock(), StaticSession=Mock(), track_coarse=Mock())
        config = {'arm': {}, 'car': {'tracking': {}, 'vision': {},
                                    'tracking_task': {'period_ms': 100}}}
        archived = []
        def archive(actual_config, frames, status):
            trace.append('archive')
            self.assertIs(actual_config, config)
            self.assertIs(frames, frames_seen[0])
            self.assertEqual(frames, [item])
            self.assertIs(frames[0], item)
            archived.append((frames[0], status))
            if archive_error is not None:
                raise archive_error
            return {'directory': 'synthetic', 'frame_count': 1, 'archive_complete': True}
        files = mock_open()
        result, caught = None, None
        with patch.object(board, 'emit'), patch.object(board, 'archive', side_effect=archive) as save, \
             patch('builtins.open', files):
            try:
                result = board.tracking_task(config, ready_dispatched=True,
                    camera=camera if borrowed else None, prepared=prepared, fill_light=light)
            except BaseException as exc:
                caught = exc
        writes = files().write.call_args_list
        records = [json.loads(call.args[0]) for call in writes]
        self.assertEqual(save.call_count, 1, trace)
        self.assertEqual(len(records), 1, trace)
        self.assertEqual(len(archived), 1)
        self.assertIs(archived[0][0], item)
        self.assertEqual(frames_seen[0], [])
        self.assertEqual(records[0]['timeline'], timeline)
        self.assertTrue(records[0]['ready_pose2_goal_sent'])
        self.assertFalse(records[0]['ready_pose2_arrival_checked'])
        self.assertEqual(records[0]['arm_started_scope'], 'grasp_after_parking')
        self.assertLess(trace.index('light_off'), trace.index('task_close'))
        self.assertLess(trace.index('task_close'), trace.index('link_close'))
        self.assertLess(trace.index('link_close'), trace.index('archive'))
        camera.capture.assert_not_called()
        if borrowed:
            camera.close.assert_not_called()
        else:
            camera.close.assert_called_once()
            self.assertLess(trace.index('task_close'), trace.index('camera_close'))
            self.assertLess(trace.index('camera_close'), trace.index('link_close'))
        return result, caught, records[0], trace

    def test_success_archives_once_after_original_light_and_resource_cleanup(self):
        result, error, record, trace = self.run_case()
        self.assertIsNone(error)
        self.assertEqual(result['status'], 'succeeded')
        self.assertEqual(record['status'], 'succeeded')
        # Existing policy: immediate off at success, and the finally off.
        self.assertEqual(trace.count('light_off'), 2)
        self.assertEqual(record['evidence']['frame_count'], 1)

    def test_step_keyboard_interrupt_preserves_running_snapshot_and_original_exception(self):
        original = KeyboardInterrupt('single user interrupt')
        result, error, record, unused = self.run_case(step_error=original)
        self.assertIsNone(result)
        self.assertIs(error, original)
        self.assertEqual(record['status'], 'running')

    def test_task_close_keyboard_interrupt_still_archives_the_current_running_task(self):
        original = KeyboardInterrupt('interrupt delivered during task.close')
        result, error, record, unused = self.run_case(
            step_error=RuntimeError('task was exiting'), close_error=original)
        self.assertIsNone(result)
        self.assertIs(error, original)
        self.assertEqual(record['status'], 'running')

    def test_each_cleanup_error_preserves_archive_and_propagates_the_actual_failure(self):
        for stage in ('close_error', 'camera_error', 'link_error', 'light_error'):
            with self.subTest(stage=stage):
                original = RuntimeError(stage)
                result, error, record, unused = self.run_case(
                    step_error=KeyboardInterrupt('exit to cleanup'), **{stage: original})
                self.assertIsNone(result)
                self.assertIs(error, original)
                self.assertEqual(record['status'], 'running')

    def test_cleanup_interrupt_does_not_close_the_voice_owned_camera(self):
        original = KeyboardInterrupt('interrupt during borrowed-camera task close')
        unused, error, record, unused = self.run_case(close_error=original, borrowed=True)
        self.assertIs(error, original)
        self.assertEqual(record['status'], 'succeeded')

    def test_archive_error_is_recorded_without_hiding_the_cleanup_interrupt(self):
        original = KeyboardInterrupt('interrupt delivered during cleanup')
        unused, error, record, unused = self.run_case(
            step_error=RuntimeError('task was exiting'), close_error=original,
            archive_error=ValueError('synthetic archive unavailable'))
        self.assertIs(error, original)
        self.assertEqual(record['status'], 'running')
        self.assertEqual(record['archive_error'], 'synthetic archive unavailable')
        self.assertNotIn('evidence', record)


if __name__ == '__main__':
    unittest.main()
