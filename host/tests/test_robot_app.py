"""Unified board resource ownership and voice dispatch, without hardware."""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
from host.project_paths import ROOT


def app():
    spec = importlib.util.spec_from_file_location('test_robot_app', ROOT/'device/k230/robot/app.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class RobotAppTests(unittest.TestCase):
    def test_approach_closes_both_owners_before_handoff_and_keeps_moving_detector(self):
        board = app()
        events = []
        camera = Mock()
        camera.open.side_effect = lambda: events.append('camera_open')
        camera.close.side_effect = lambda: events.append('camera_close')
        link = Mock()
        link.close.side_effect = lambda: events.append('uart_close')
        moving, static = Mock(), Mock()
        result = {'status':'parked'}
        def run(*args, **kwargs):
            self.assertIs(kwargs['fine_factory'], static)
            kwargs['moving_fine_factory'](config, board.time.ticks_ms)
            moving.assert_called_once_with(config, board.time.ticks_ms, purpose='car_moving_range')
            events.append('parking_returned')
            return result
        modules = {
            'camera':SimpleNamespace(CameraSession=lambda cfg:camera),
            'motor_link':SimpleNamespace(MotorLink=lambda:link),
            'ground_projection':SimpleNamespace(GroundProjection=Mock()),
            'blue_tracker':SimpleNamespace(BlueTracker=Mock()),
            'vision_runtime':SimpleNamespace(track_coarse=Mock()),
            'near_runtime':SimpleNamespace(run_parking=run),
            'physical_side_session':SimpleNamespace(FineSession=moving),
            'static_pick_session':SimpleNamespace(StaticPickSession=static)}
        config = {'projection':dict(K=[], D=[], T=[], plane={}), 'tracking':{}, 'vision':{}}
        with patch.dict(sys.modules, modules), \
                patch.object(board.time, 'ticks_ms', lambda:0, create=True), \
                patch.object(board.time, 'sleep_ms', Mock(), create=True):
            self.assertIs(board.approach(config, []), result)
        self.assertEqual(events, ['camera_open', 'parking_returned', 'camera_close', 'uart_close'])

    def test_camera_failure_also_closes_motor_uart(self):
        board = app()
        camera, link = Mock(), Mock()
        camera.open.side_effect = ValueError('camera failed')
        modules = {
            'camera':SimpleNamespace(CameraSession=lambda cfg:camera),
            'motor_link':SimpleNamespace(MotorLink=lambda:link),
            'ground_projection':SimpleNamespace(GroundProjection=Mock()),
            'blue_tracker':SimpleNamespace(BlueTracker=Mock()),
            'vision_runtime':SimpleNamespace(track_coarse=Mock()),
            'near_runtime':SimpleNamespace(run_parking=Mock()),
            'physical_side_session':SimpleNamespace(FineSession=Mock()),
            'static_pick_session':SimpleNamespace(StaticPickSession=Mock())}
        with patch.dict(sys.modules, modules), self.assertRaisesRegex(ValueError, 'camera failed'):
            board.approach({'projection':dict(K=[], D=[], T=[], plane={}), 'tracking':{}, 'vision':{}}, [])
        camera.close.assert_called_once()
        link.close.assert_called_once()

    def test_ready_is_separate_and_never_runs_car_or_grasp(self):
        board = app()
        arm = SimpleNamespace(run_ready=Mock(), run_from_observation=Mock())
        task = SimpleNamespace(execute_task=Mock())
        with patch.dict(sys.modules, {'app':arm, 'robot_task':task}), patch.object(board, 'approach') as drive:
            result = board.dispatch('ready', {'arm':{'control':'accepted'}})
        arm.run_ready.assert_called_once_with({'control':'accepted'})
        drive.assert_not_called()
        task.execute_task.assert_not_called()
        arm.run_from_observation.assert_not_called()
        self.assertEqual(result['status'], 'ready_dispatched')

    def test_boot_waits_and_restarts_voice_without_another_warmup(self):
        board = app()
        events = []
        def wait(config, warmup):
            events.append(('voice', warmup))
            if len(events) > 1:
                raise KeyboardInterrupt()
            return 'pick'
        def dispatch(action, config):
            events.append(('task', action))
            raise ValueError('No complete final face')
        with patch.dict(sys.modules, {'app':SimpleNamespace(wait_for_wake=wait)}), \
                patch.object(board, 'load', return_value={'voice':{}}), \
                patch.object(board, 'emit'), patch.object(board, 'dispatch', side_effect=dispatch):
            with self.assertRaises(KeyboardInterrupt):
                board.run_forever('/release')
        self.assertEqual(events, [('voice',32), ('task','pick'), ('voice',0)])


if __name__ == '__main__':
    unittest.main()
