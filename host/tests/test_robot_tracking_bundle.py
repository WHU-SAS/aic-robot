"""The explicit offline tracking candidate includes its complete runtime."""
import ast
import importlib.util
import json
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, mock_open, patch
from host.project_paths import ROOT
from tools.deploy.k230_robot import build_bundle
from tools.deploy.k230_pick import encoded


def board_app():
    spec=importlib.util.spec_from_file_location('tracking_board_app',ROOT/'device/k230/robot/app.py')
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class TrackingBundleTests(unittest.TestCase):
    def test_tracking_configuration_is_compact_without_losing_any_fields(self):
        config,files,unused=build_bundle(tracking=True)
        self.assertEqual(json.loads(files['config.json']),config)
        self.assertEqual(files['config.json'],(json.dumps(config,ensure_ascii=False,
            sort_keys=True,separators=(',',':'))+'\n').encode())
        self.assertLess(len(files['config.json']),len(encoded(config)))
        legacy,legacy_files,unused=build_bundle()
        self.assertEqual(legacy_files['config.json'],encoded(legacy))

    def test_candidate_has_full_runtime_with_no_counter_limit_or_calibration_change(self):
        baseline=build_bundle()
        config,files,manifest=build_bundle(tracking=True)
        expected=dict(baseline[0]['car'])
        expected['tracking_task']=json.loads((ROOT/'configs/car/tracking_task.json').read_text())
        expected['tracking']=dict(expected['tracking'],confirmation_frames=1)
        self.assertEqual(config['car'],expected)
        self.assertEqual(config['arm'],baseline[0]['arm'])
        self.assertTrue(config['car']['tracking_task']['enabled'])
        self.assertEqual(config['car']['tracking_task']['candidate_confirmation_frames'],1)
        self.assertEqual(config['car']['tracking_task']['loss_confirmation_frames'],3)
        self.assertEqual(config['car']['tracking_task']['movement_confirmation_frames'],2)
        self.assertEqual(baseline[0]['car']['tracking']['confirmation_frames'],3)
        self.assertEqual(manifest['mode'],'continuous_tracking')
        self.assertTrue(manifest['continuous_observation'])
        self.assertTrue(manifest['requires_mspm0_signed_follow'])
        self.assertFalse(manifest['physical_tracking_validated'])
        for name in ('robot_tracking.py','target_memory.py','tracking_drive.py',
                     'cooperative_static_session.py','candidate_association.py'):
            self.assertIn(name,files.keys())
            ast.parse(files[name])
        self.assertIn(b'class PickExecution:',files['control.py'])
        self.assertNotEqual(manifest['release_id'],baseline[2]['release_id'])

    def test_enabled_config_uses_new_entry_and_preserves_already_sent_ready(self):
        app=board_app();arm=SimpleNamespace(run_ready=Mock())
        config={'car':{'tracking_task':{'enabled':True}},'arm':{}}
        with patch.dict(sys.modules,{'app':arm}),\
             patch.object(app,'tracking_task',return_value={'status':'succeeded'}) as tracking,\
             patch.object(app,'approach') as old:
            result=app.dispatch('pick',config,ready_dispatched=True)
        tracking.assert_called_once_with(config,True)
        old.assert_not_called();arm.run_ready.assert_not_called()
        self.assertEqual(result['status'],'succeeded')

    def test_task_keeps_one_camera_and_uart_until_tracking_finishes(self):
        for borrowed, interrupted in ((False,False),(True,False),(False,True),(True,True)):
            with self.subTest(borrowed=borrowed,interrupted=interrupted):
                self.check_task_camera_ownership(borrowed,interrupted)

    def check_task_camera_ownership(self,borrowed,interrupted):
        app=board_app();config=build_bundle(tracking=True)[0];trace=[]
        camera,link=Mock(),Mock()
        camera.open.side_effect=lambda:trace.append('camera_open')
        camera.close.side_effect=lambda:trace.append('camera_close')
        link.close.side_effect=lambda:trace.append('motor_close')
        task=Mock(status='running')
        def step():
            trace.append('tracking_step')
            self.assertNotIn('camera_close',trace)
            if interrupted:raise RuntimeError('tracking interrupted')
            task.status='succeeded'
        task.step.side_effect=step
        task.snapshot.return_value={'status':'succeeded'}
        task.close.side_effect=lambda:trace.append('task_close')
        arm=SimpleNamespace(run_ready=Mock(),begin_from_observation=Mock(),
                            PickPlanUnavailable=ValueError)
        modules={
            'app':arm,'ground_projection':SimpleNamespace(GroundProjection=Mock()),
            'camera':SimpleNamespace(CameraSession=Mock(return_value=camera)),
            'motor_link':SimpleNamespace(MotorLink=Mock(return_value=link)),
            'blue_tracker':SimpleNamespace(BlueTracker=Mock(),age_ms=lambda a,b:a-b),
            'vision_runtime':SimpleNamespace(track_coarse=Mock()),
            'physical_side_session':SimpleNamespace(FineSession=Mock()),
            'cooperative_static_session':SimpleNamespace(CooperativeStaticPickSession=Mock()),
            'robot_tracking':SimpleNamespace(TrackingTask=Mock(return_value=task),ReobserveRequired=ValueError),
            'kinematics':SimpleNamespace(Geometry=Mock())}
        with patch.dict(sys.modules,modules),patch.object(app.time,'ticks_ms',lambda:0,create=True),\
             patch.object(app.time,'sleep_ms',Mock(),create=True),patch('builtins.open',mock_open()):
            if interrupted:
                with self.assertRaisesRegex(RuntimeError,'tracking interrupted'):
                    app.tracking_task(config,ready_dispatched=True,camera=camera if borrowed else None)
            else:
                result=app.tracking_task(config,ready_dispatched=True,camera=camera if borrowed else None)
                self.assertEqual(result['status'],'succeeded')
        expected=['tracking_step','task_close','motor_close'] if borrowed else [
            'camera_open','tracking_step','task_close','camera_close','motor_close']
        self.assertEqual(trace,expected)
        self.assertEqual(task.metrics.update.call_args.kwargs['camera_reused'],borrowed)
        self.assertEqual(modules['camera'].CameraSession.call_count,0 if borrowed else 1)
        arm.run_ready.assert_not_called();arm.begin_from_observation.assert_not_called()

    def test_tracking_dispatch_forwards_only_the_prepared_camera_to_new_entry(self):
        app=board_app();arm=SimpleNamespace(run_ready=Mock());camera=Mock()
        config={'car':{'tracking_task':{'enabled':True}},'arm':{}}
        with patch.dict(sys.modules,{'app':arm}),\
             patch.object(app,'tracking_task',return_value={'status':'succeeded'}) as task:
            app.dispatch('pick',config,ready_dispatched=True,camera=camera)
        task.assert_called_once_with(config,True,camera=camera)
        arm.run_ready.assert_not_called()


if __name__=='__main__':unittest.main()
