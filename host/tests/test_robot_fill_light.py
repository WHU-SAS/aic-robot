"""Voice/task light ownership, exercised through the board app without hardware."""
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, mock_open, patch

from host.tests.test_robot_app import app, arm_app


class Light:
    def __init__(self, trace):
        self.trace, self.active = trace, False
        trace.append(('light_init', False))

    def set_active(self, active):
        self.active = bool(active)
        self.trace.append(('light', self.active))
        return self.active

    def on(self):
        return self.set_active(True)

    def off(self):
        return self.set_active(False)


class RobotFillLightTests(unittest.TestCase):
    def test_disabled_configuration_does_not_import_hardware_driver(self):
        board = app()
        with patch.dict(sys.modules, {'fill_light': None}):
            self.assertIsNone(board._make_fill_light({}))
            self.assertIsNone(board._make_fill_light({'fill_light': {'enabled': False}}))

    def test_only_approved_enabled_wiring_constructs_driver(self):
        board = app()
        factory = Mock(return_value=object())
        with patch.dict(sys.modules, {'fill_light': SimpleNamespace(FillLight=factory)}):
            for options in ({'enabled': True},
                            {'enabled': True, 'gpio': 25, 'active_high': True},
                            {'enabled': True, 'gpio': 26, 'active_high': False}):
                with self.subTest(options=options), self.assertRaises(ValueError):
                    board._make_fill_light({'fill_light': options})
            factory.assert_not_called()
            self.assertIs(board._make_fill_light({'fill_light': {
                'enabled': True, 'gpio': 26, 'active_high': True}}), factory.return_value)
        factory.assert_called_once_with()

    def run_voice(self, actions=(), failure=None, interrupt_after_wake=False):
        board, arm, trace, lights = app(), arm_app(), [], []
        config = {'arm': {}, 'voice': {}, 'car': {}, 'fill_light': {
            'enabled': True, 'gpio': 26, 'active_high': True}}
        pending = list(actions)

        def factory():
            light = Light(trace)
            lights.append(light)
            return light

        def wait(voice, warmup, on_command):
            self.assertFalse(lights[0].active, 'Light must be off whenever voice waits')
            trace.append(('voice_wait', warmup))
            if not pending:
                raise KeyboardInterrupt()
            action = pending.pop(0)
            on_command(action)
            trace.append(('audio_cleanup', lights[0].active))
            if interrupt_after_wake:
                raise KeyboardInterrupt()
            return action

        def dispatch(action, actual, **options):
            self.assertIs(actual, config)
            self.assertIs(options['fill_light'], lights[0])
            self.assertTrue(options['ready_dispatched'])
            self.assertEqual(lights[0].active, action == 'pick')
            trace.append(('dispatch', action, lights[0].active))
            if failure is not None:
                raise failure
            return {'status': 'complete'}

        with patch.dict(sys.modules, {'app': arm,
                'fill_light': SimpleNamespace(FillLight=factory)}), \
             patch.object(board, 'load', return_value=config), \
             patch.object(board, 'emit'), \
             patch.object(arm, 'wait_for_wake', side_effect=wait), \
             patch.object(arm, 'run_ready', side_effect=lambda cfg, pose, immediate:
                          trace.append(('ready', pose, immediate))), \
             patch.object(board, 'dispatch', side_effect=dispatch):
            with self.assertRaises(KeyboardInterrupt):
                board.run_forever('/synthetic-release')
        self.assertEqual(len(lights), 1, 'One persistent driver owns the whole voice loop')
        self.assertFalse(lights[0].active)
        return trace

    def test_startup_waits_with_light_off_and_without_motion(self):
        trace = self.run_voice()
        self.assertEqual(trace[:2], [('light_init', False), ('voice_wait', 32)])
        self.assertFalse(any(row[0] in ('ready', 'dispatch') for row in trace))
        self.assertNotIn(('light', True), trace)

    def test_pick_response_precedes_on_and_on_precedes_audio_cleanup(self):
        trace = self.run_voice(('pick',))
        self.assertLess(trace.index(('ready', 2, True)), trace.index(('light', True)))
        self.assertLess(trace.index(('light', True)), trace.index(('audio_cleanup', True)))
        self.assertLess(trace.index(('audio_cleanup', True)), trace.index(('dispatch', 'pick', True)))
        self.assertIn(('voice_wait', 0), trace)
        task = trace.index(('dispatch', 'pick', True))
        resumed = trace.index(('voice_wait', 0))
        self.assertIn(('light', False), trace[task+1:resumed])

    def test_ready_voice_command_never_turns_light_on(self):
        trace = self.run_voice(('ready',))
        self.assertIn(('ready', 1, True), trace)
        self.assertIn(('audio_cleanup', False), trace)
        self.assertIn(('dispatch', 'ready', False), trace)
        self.assertNotIn(('light', True), trace)

    def test_voice_interrupt_after_wake_turns_light_off_without_dispatch(self):
        trace = self.run_voice(('pick',), interrupt_after_wake=True)
        self.assertIn(('light', True), trace)
        self.assertEqual(trace[-1], ('light', False))
        self.assertFalse(any(row[0] == 'dispatch' for row in trace))

    def test_task_exception_turns_light_off_before_listening_again(self):
        trace = self.run_voice(('pick',), failure=RuntimeError('synthetic task error'))
        task = trace.index(('dispatch', 'pick', True))
        resumed = trace.index(('voice_wait', 0))
        self.assertIn(('light', False), trace[task+1:resumed])

    def run_tracking(self, sequence):
        board, trace = app(), []
        light = Light(trace)
        light.on()

        class Task:
            def __init__(self):
                self.status, self.grasp_confirmed = 'running', None
                self.metrics = {}
                self.sequence = iter(sequence)

            def step(task):
                self.assertTrue(light.active, 'Recovery/retry must retain the light')
                row = next(task.sequence)
                if isinstance(row, BaseException):
                    trace.append(('step_error', type(row).__name__))
                    raise row
                task.status, task.grasp_confirmed = row
                trace.append(('step', task.status, task.grasp_confirmed, light.active))

            def idle_delay_ms(task):
                trace.append(('idle', task.status, task.grasp_confirmed, light.active))
                return 0

            def snapshot(task):
                return {'status': task.status, 'grasp_confirmed': task.grasp_confirmed}

            def close(task):
                trace.append(('task_close', light.active))

        task = Task()
        camera = SimpleNamespace(capture=Mock())
        link = SimpleNamespace(close=lambda: trace.append(('link_close', light.active)))
        tracker = SimpleNamespace(detect_frame=Mock(), reset=Mock())
        prepared = dict(arm=SimpleNamespace(), age_ms=lambda a,b:a-b,
            projection=object(), held_geometry=object(), BlueTracker=lambda *a: tracker,
            MotorLink=lambda: link, TrackingTask=lambda *a, **kw: task,
            FineSession=Mock(), StaticSession=Mock(), track_coarse=Mock())
        config = {'arm': {}, 'car': {'tracking': {}, 'vision': {},
                                    'tracking_task': {'period_ms': 100}}}
        error, result = None, None
        with patch.object(board, 'emit'), patch('builtins.open', mock_open()), \
             patch.object(board, 'archive', side_effect=lambda cfg, frames, status:
                          trace.append(('archive', status, light.active))):
            try:
                result = board.tracking_task(config, ready_dispatched=True,
                    camera=camera, prepared=prepared, fill_light=light)
            except BaseException as exc:
                error = exc
        return trace, light, result, error

    def test_tracking_empty_recovery_and_early_grip_keep_on_until_terminal_success(self):
        sequence = [('running', True), ('recovering', None),
                    ('running', False), ('running', True), ('succeeded', True)]
        trace, light, result, error = self.run_tracking(sequence)
        self.assertIsNone(error)
        self.assertEqual(result['status'], 'succeeded')
        for row in trace:
            if row[0] == 'idle':
                self.assertEqual(row[-1], row[1] != 'succeeded')
        terminal = trace.index(('step', 'succeeded', True, True))
        idle = trace.index(('idle', 'succeeded', True, False))
        self.assertEqual(trace[terminal+1], ('light', False))
        self.assertLess(terminal, idle)
        self.assertIn(('archive', 'succeeded', False), trace)
        self.assertFalse(light.active)

    def test_tracking_exception_or_interrupt_turns_off_before_cleanup_and_archive(self):
        for failure in (RuntimeError('step error'), KeyboardInterrupt()):
            with self.subTest(failure=type(failure).__name__):
                trace, light, result, error = self.run_tracking([
                    ('running', True), failure])
                self.assertIs(error, failure)
                self.assertIsNone(result)
                self.assertFalse(light.active)
                self.assertIn(('task_close', False), trace)
                self.assertIn(('link_close', False), trace)
                self.assertIn(('archive', 'running', False), trace)

    def run_legacy(self, plan):
        board, trace = app(), []
        light = Light(trace)
        light.on()
        arm = SimpleNamespace(run_from_observation=Mock(return_value=plan))

        def execute(drive, grasp, emit):
            self.assertTrue(light.active)
            result = grasp({'synthetic': 'accepted observation'})
            trace.append(('grasp_returned', light.active))
            return {'status': 'complete', 'arm_plan': result}

        with patch.dict(sys.modules, {'app': arm,
                'robot_task': SimpleNamespace(execute_task=execute)}), \
             patch.object(board, 'emit'), patch('builtins.open', mock_open()), \
             patch.object(board, 'archive', side_effect=lambda *a:
                          trace.append(('archive', light.active))):
            result = board.dispatch('pick', {'arm': {}, 'car': {}},
                                    ready_dispatched=True, fill_light=light)
        self.assertFalse(light.active)
        self.assertEqual(result['status'], 'complete')
        return trace

    def test_legacy_confirmed_grip_turns_off_before_grasp_callback_returns(self):
        trace = self.run_legacy({'status': 'complete', 'grasp_confirmed': True})
        self.assertIn(('grasp_returned', False), trace)
        self.assertIn(('archive', False), trace)

    def test_legacy_complete_or_empty_is_not_an_early_grasp_success(self):
        for plan in ({'status': 'complete'},
                     {'status': 'complete', 'grasp_confirmed': False}):
            with self.subTest(plan=plan):
                trace = self.run_legacy(plan)
                self.assertIn(('grasp_returned', True), trace)
                self.assertIn(('archive', False), trace)

    def test_tracking_dispatch_forwards_the_same_held_light(self):
        board = app()
        light = Light([])
        config = {'arm': {}, 'car': {'tracking_task': {'enabled': True}}}
        with patch.dict(sys.modules, {'app': SimpleNamespace()}), \
             patch.object(board, 'tracking_task', return_value={'status': 'succeeded'}) as run:
            board.dispatch('pick', config, ready_dispatched=True, fill_light=light)
        run.assert_called_once_with(config, True, fill_light=light)


if __name__ == '__main__':
    unittest.main()
