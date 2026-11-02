"""Unified board resource ownership and voice dispatch, without hardware."""
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch, mock_open
from host.project_paths import ROOT


def app():
    spec = importlib.util.spec_from_file_location('test_robot_app', ROOT/'device/k230/robot/app.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def arm_app():
    spec=importlib.util.spec_from_file_location('test_arm_voice_app',ROOT/'device/k230/pick/app.py')
    module=importlib.util.module_from_spec(spec)
    mocks={'time':SimpleNamespace(ticks_ms=lambda:0,ticks_diff=lambda a,b:a-b),
           'camera':SimpleNamespace(CameraSession=None),'vision':SimpleNamespace(localize=None),
           'vision_runtime':SimpleNamespace(localize_precise=None),'kinematics':SimpleNamespace(Geometry=None),
           'control':SimpleNamespace(run_pick_stage_direct=None,run_ready_direct=None),
           'voice_commands':SimpleNamespace(registered_commands=None,resolve_events=None)}
    with patch.dict(sys.modules,mocks):spec.loader.exec_module(module)
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
        board=app();arm=arm_app();events=[];waits=[]
        def wait(config,warmup,on_command):
            waits.append(warmup);events.append(('voice',warmup))
            if len(waits)>1:raise KeyboardInterrupt()
            on_command('pick');events.append('audio_cleanup');return 'pick'
        def ready(config,pose,immediate):events.append(('ready',pose,immediate))
        def dispatch(action,config,ready_dispatched=False):
            events.append(('task',action,ready_dispatched));raise ValueError('No complete final face')
        with patch.dict(sys.modules,{'app':arm}),patch.object(arm,'wait_for_wake',side_effect=wait),\
             patch.object(arm,'run_ready',side_effect=ready),\
             patch.object(board,'load',return_value={'arm':{},'voice':{}}),\
             patch.object(board,'emit'),patch.object(board,'dispatch',side_effect=dispatch):
            with self.assertRaises(KeyboardInterrupt):board.run_forever('/release')
        self.assertEqual(events,[('voice',32),('ready',2,True),'audio_cleanup',('task','pick',True),('voice',0)])

    def test_initial_listening_never_sends_ready_or_starts_car(self):
        board=app();arm=arm_app()
        with patch.dict(sys.modules,{'app':arm}),\
             patch.object(arm,'wait_for_wake',side_effect=KeyboardInterrupt()),\
             patch.object(arm,'run_ready') as ready,\
             patch.object(board,'load',return_value={'arm':{},'voice':{}}),\
             patch.object(board,'emit'),patch.object(board,'dispatch') as task:
            with self.assertRaises(KeyboardInterrupt):board.run_forever('/release')
        ready.assert_not_called();task.assert_not_called()

    def test_tracking_prepares_camera_once_before_listening_and_lends_across_tasks(self):
        board=app();arm=arm_app();camera=Mock();trace=[];waits=[]
        camera.open.side_effect=lambda:trace.append('camera_open')
        camera.close.side_effect=lambda:trace.append('camera_close')
        config={'arm':{},'voice':{},'car':{'vision':{'sampling':{'warmup_frames':30}},
            'tracking_task':{'enabled':True}}}
        def wait(voice,warmup,on_command):
            waits.append(warmup);trace.append('voice')
            if len(waits)>3:raise KeyboardInterrupt()
            on_command('pick');return 'pick'
        def dispatch(action,actual,ready_dispatched=False,camera=None):
            self.assertIs(actual,config);self.assertIs(camera,prepared)
            self.assertTrue(ready_dispatched)
            trace.append('task')
            if len(waits)==2:raise ValueError('retry scene')
            return {'status':'succeeded'}
        prepared=camera
        factory=Mock(return_value=camera)
        with patch.dict(sys.modules,{'app':arm,'camera':SimpleNamespace(CameraSession=factory)}),\
             patch.object(arm,'wait_for_wake',side_effect=wait),patch.object(arm,'run_ready') as ready,\
             patch.object(board,'load',return_value=config),patch.object(board,'emit'),\
             patch.object(board,'dispatch',side_effect=dispatch),\
             patch.object(board.time,'ticks_ms',lambda:0,create=True),\
             patch.object(board.time,'ticks_diff',lambda a,b:a-b,create=True):
            with self.assertRaises(KeyboardInterrupt):board.run_forever('/release')
        factory.assert_called_once_with(config['car']['vision'])
        camera.open.assert_called_once();camera.close.assert_called_once()
        self.assertEqual(trace,['camera_open','voice','task','voice','task','voice','task',
                                'voice','camera_close'])
        self.assertEqual(waits,[32,0,0,0]);self.assertEqual(ready.call_count,3)

    def test_failed_camera_preparation_retains_voice_and_original_task_open_path(self):
        board=app();arm=arm_app();camera=Mock();waits=[]
        camera.open.side_effect=ValueError('camera unavailable')
        config={'arm':{},'voice':{},'car':{'vision':{},'tracking_task':{'enabled':True}}}
        def wait(voice,warmup,on_command):
            waits.append(warmup)
            if len(waits)>1:raise KeyboardInterrupt()
            on_command('pick');return 'pick'
        with patch.dict(sys.modules,{'app':arm,'camera':SimpleNamespace(CameraSession=lambda cfg:camera)}),\
             patch.object(arm,'wait_for_wake',side_effect=wait),patch.object(arm,'run_ready'),\
             patch.object(board,'load',return_value=config),patch.object(board,'emit') as emit,\
             patch.object(board,'dispatch',return_value={'status':'succeeded'}) as task,\
             patch.object(board.time,'ticks_ms',lambda:0,create=True):
            with self.assertRaises(KeyboardInterrupt):board.run_forever('/release')
        camera.close.assert_called_once()
        task.assert_called_once_with('pick',config,ready_dispatched=True)
        emit.assert_any_call('camera_prepare_error',{'error':'camera unavailable'})

    def test_tracking_loop_uses_task_idle_delay_and_keeps_waiting_sleeps(self):
        board=app();camera=Mock();task=Mock(status='running');steps=[]
        def step():
            steps.append(True)
            if len(steps)==3:task.status='succeeded'
        task.step.side_effect=step
        task.idle_delay_ms.side_effect=[0,5,5]
        task.snapshot.return_value={'status':'succeeded'}
        modules={'app':SimpleNamespace(),
            'ground_projection':SimpleNamespace(GroundProjection=Mock()),
            'camera':SimpleNamespace(CameraSession=Mock()),
            'motor_link':SimpleNamespace(MotorLink=Mock()),
            'blue_tracker':SimpleNamespace(BlueTracker=Mock(),age_ms=lambda a,b:a-b),
            'vision_runtime':SimpleNamespace(track_coarse=Mock()),
            'physical_side_session':SimpleNamespace(FineSession=Mock()),
            'cooperative_static_session':SimpleNamespace(CooperativeStaticPickSession=Mock()),
            'robot_tracking':SimpleNamespace(TrackingTask=Mock(return_value=task),ReobserveRequired=ValueError),
            'kinematics':SimpleNamespace(Geometry=Mock())}
        config={'arm':{'model':{},'pick':{}},'car':{'projection':dict(K=[],D=[],T=[],plane={}),
            'tracking':{},'vision':{},'tracking_task':{'period_ms':100}}}
        with patch.dict(sys.modules,modules),patch.object(board.time,'ticks_ms',lambda:0,create=True),\
             patch.object(board.time,'sleep_ms',Mock(),create=True) as sleep,\
             patch('builtins.open',mock_open()):
            board.tracking_task(config,ready_dispatched=True,camera=camera)
        self.assertEqual([call.args[0] for call in sleep.call_args_list],[5,5])
        task.close.assert_called_once();camera.open.assert_not_called();camera.close.assert_not_called()

    def test_failed_voice_response_cleans_audio_and_never_enters_car_dispatch(self):
        board=app();arm=arm_app();trace=[];waits=[]
        def wait(config,warmup,on_command):
            waits.append(warmup)
            if len(waits)>1:raise KeyboardInterrupt()
            on_command('pick');trace.append('audio_cleanup');return 'pick'
        def ready(*args,**kwargs):trace.append('ready_failed');raise RuntimeError('UART not ready')
        with patch.dict(sys.modules,{'app':arm}),patch.object(arm,'wait_for_wake',side_effect=wait),\
             patch.object(arm,'run_ready',side_effect=ready) as send,\
             patch.object(board,'load',return_value={'arm':{},'voice':{}}),\
             patch.object(board,'emit') as emit,patch.object(board,'dispatch') as task:
            with self.assertRaises(KeyboardInterrupt):board.run_forever('/release')
        send.assert_called_once_with({},pose=2,immediate=True);task.assert_not_called()
        self.assertEqual(waits,[32,0]);self.assertEqual(trace,['ready_failed','audio_cleanup'])
        emit.assert_any_call('task_error',{'error':'UART not ready'})

    def test_direct_pick_dispatch_sends_pose2_before_car_and_never_waits_for_arrival(self):
        board=app();trace=[];arm_config={'accepted':'arm'};car_config={'accepted':'car'}
        arm=SimpleNamespace(run_ready=Mock(side_effect=lambda c,**kw:trace.append(('ready',kw))),
                            run_from_observation=Mock())
        def execute(drive,grasp,emit):
            trace.append('task');parking=drive();return {'status':'parking_failed','parking':parking}
        task=SimpleNamespace(execute_task=execute)
        def drive(*args):
            trace.append('car');self.assertEqual(arm.run_ready.call_count,1)
            arm.run_from_observation.assert_not_called();return {'status':'no_target'}
        with patch.dict(sys.modules,{'app':arm,'robot_task':task}),\
             patch.object(board,'approach',side_effect=drive),patch.object(board,'emit'),\
             patch('builtins.open',mock_open()):
            result=board.dispatch('pick',{'arm':arm_config,'car':car_config})
        arm.run_ready.assert_called_once_with(arm_config,pose=2,immediate=True)
        self.assertEqual(trace,[('ready',{'pose':2,'immediate':True}),'task','car'])
        self.assertEqual(result['status'],'parking_failed')

    def test_parking_failure_records_ready_target_separately_from_grasp_start(self):
        from device.k230.robot.task import execute_task
        board=app();arm=SimpleNamespace(run_ready=Mock(),run_from_observation=Mock())
        parking={'status':'no_precise_target','verification':{'accepted':False}}
        writer=mock_open()
        with patch.dict(sys.modules,{'app':arm,'robot_task':SimpleNamespace(execute_task=execute_task)}), \
             patch.object(board,'approach',return_value=parking),patch.object(board,'emit'), \
             patch('builtins.open',writer):
            result=board.dispatch('pick',{'arm':{},'car':{}})
        arm.run_ready.assert_called_once_with({},pose=2,immediate=True)
        arm.run_from_observation.assert_not_called()
        saved=json.loads(''.join(call.args[0] for call in writer().write.call_args_list))
        for record in (result,saved):
            self.assertEqual(record['status'],'parking_failed')
            self.assertTrue(record['ready_pose2_goal_sent'])
            self.assertFalse(record['ready_pose2_arrival_checked'])
            self.assertFalse(record['arm_started'])
            self.assertEqual(record['arm_started_scope'],'grasp_after_parking')

    def test_already_sent_voice_response_is_never_dispatched_twice(self):
        for action in ('pick','ready'):
            with self.subTest(action=action):
                board=app();arm=SimpleNamespace(run_ready=Mock(),run_from_observation=Mock())
                def execute(drive,grasp,emit):return {'status':'parking_failed','parking':drive()}
                with patch.dict(sys.modules,{'app':arm,'robot_task':SimpleNamespace(execute_task=execute)}),\
                     patch.object(board,'approach',return_value={'status':'no_target'}) as car,\
                     patch.object(board,'emit'),patch('builtins.open',mock_open()):
                    board.dispatch(action,{'arm':{},'car':{}},ready_dispatched=True)
                arm.run_ready.assert_not_called()
                self.assertEqual(car.call_count,1 if action=='pick' else 0)

    def test_direct_pose2_failure_prevents_any_car_or_grasp_execution(self):
        board=app();arm=SimpleNamespace(run_ready=Mock(side_effect=RuntimeError('short UART write')),
                                      run_from_observation=Mock())
        task=SimpleNamespace(execute_task=Mock())
        with patch.dict(sys.modules,{'app':arm,'robot_task':task}),\
             patch.object(board,'approach') as car:
            with self.assertRaisesRegex(RuntimeError,'short UART write'):
                board.dispatch('pick',{'arm':{},'car':{}})
        car.assert_not_called();task.execute_task.assert_not_called();arm.run_from_observation.assert_not_called()


if __name__ == '__main__':
    unittest.main()
