"""Pure boot preparation and fresh per-command control state, without hardware."""
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, mock_open, patch

from device.k230.car.blue_tracker import BlueTracker, age_ms
from device.k230.car.ground_projection import GroundProjection
from device.k230.robot.tracking import TrackingTask, ReobserveRequired
from device.k230.vision.runtime import track_coarse
from tools.deploy.k230_robot import build_bundle
from host.tests import test_robot_tracking as fixtures
from host.tests.test_car_tracking_reacquisition import BlobImage, FAR, NEAR, frame
from host.tests.test_robot_tracking_bundle import board_app


class TrackingPreparationTests(unittest.TestCase):
    def modules(self, trace):
        return {
            'app':SimpleNamespace(run_ready=Mock(side_effect=lambda *args,**kwargs:
                trace.append('ready2')),begin_from_observation_steps=Mock(),
                PickPlanUnavailable=ValueError),
            'vision':SimpleNamespace(),
            'ground_projection':SimpleNamespace(GroundProjection=Mock()),
            'motor_link':SimpleNamespace(MotorLink=Mock()),
            'camera':SimpleNamespace(CameraSession=Mock()),
            'blue_tracker':SimpleNamespace(BlueTracker=Mock(),age_ms=age_ms),
            'vision_runtime':SimpleNamespace(track_coarse=Mock()),
            'physical_side_session':SimpleNamespace(FineSession=Mock()),
            'cooperative_static_session':SimpleNamespace(CooperativeStaticPickSession=Mock()),
            'robot_tracking':SimpleNamespace(TrackingTask=Mock(),ReobserveRequired=ValueError),
            'kinematics':SimpleNamespace(Geometry=Mock())}

    def test_prepare_keeps_only_static_geometry_and_uninstantiated_factories(self):
        app=board_app()
        config=build_bundle(tracking=True)[0]
        trace=[]
        modules=self.modules(trace)
        with patch.dict(sys.modules,modules),patch.object(app.time,'ticks_ms',lambda:100,create=True):
            prepared=app.prepare_tracking(config)
        self.assertEqual(trace,[])
        for name,attribute in (('motor_link','MotorLink'),('camera','CameraSession'),
                ('blue_tracker','BlueTracker'),('robot_tracking','TrackingTask'),
                ('physical_side_session','FineSession'),
                ('cooperative_static_session','CooperativeStaticPickSession')):
            getattr(modules[name],attribute).assert_not_called()
        modules['app'].run_ready.assert_not_called()
        modules['app'].begin_from_observation_steps.assert_not_called()
        modules['ground_projection'].GroundProjection.assert_called_once()
        modules['kinematics'].Geometry.assert_called_once_with(
            config['arm']['model'],config['arm']['pick'],0)
        self.assertIs(prepared['held_geometry'],modules['kinematics'].Geometry.return_value)
        self.assertEqual(prepared['prepare_ms'],0)

    def test_preparation_steps_leave_audio_turns_and_separate_elapsed_time(self):
        app=board_app()
        config=build_bundle(tracking=True)[0]
        clock=fixtures.Clock()
        trace=[]
        modules=self.modules(trace)
        def build_projection(*args):
            clock.advance(7)
            return object()
        def build_geometry(*args):
            clock.advance(11)
            return object()
        modules['ground_projection'].GroundProjection.side_effect=build_projection
        modules['kinematics'].Geometry.side_effect=build_geometry
        with patch.dict(sys.modules,modules),patch.object(app.time,'ticks_ms',clock.now,create=True):
            steps=app.prepare_tracking_steps(config)
            events=[]
            while True:
                event=next(steps)
                events.append(event)
                if 'prepared' in event:
                    break
                clock.advance(160)  # A separate synthetic audio read between groups.
        self.assertEqual([event['stage'] for event in events[:-1]],
            ['vision_loaded','coarse_loaded','fine_loaded','control_loaded'])
        self.assertEqual(events[-1]['prepared']['prepare_ms'],18)
        self.assertEqual(events[-1]['prepared']['prepare_elapsed_ms'],658)
        self.assertEqual(trace,[])
        modules['motor_link'].MotorLink.assert_not_called()
        modules['camera'].CameraSession.assert_not_called()
        modules['robot_tracking'].TrackingTask.assert_not_called()

    def test_two_commands_reuse_static_preparation_but_sync_fresh_control_state(self):
        app=board_app()
        config=build_bundle(tracking=True)[0]
        clock=fixtures.Clock()
        trace=[]
        modules=self.modules(trace)
        motors,trackers,tasks,initials=[],[],[],[]
        class Motor(fixtures.Motor):
            def __init__(self):
                super().__init__(clock,config['car']['mm_per_count'])
                self.a,self.b=100*(len(motors)+1),-200*(len(motors)+1)
                self.syncs=0
                self.closed=False
                motors.append(self)
                trace.append('motor_open')
            def sync(self):
                self.syncs+=1
                return super().sync()
            def close(self):self.closed=True
        def tracker(*args):
            result=BlueTracker(*args)
            trackers.append(result)
            self.assertEqual(result.confirmations,0)
            return result
        def task(*args,**kwargs):
            result=TrackingTask(*args,**kwargs)
            tasks.append(result)
            initials.append(dict(result.state))
            advance=result.step
            def one_turn():
                value=advance()
                self.assertIsNone(result.fault)
                if len(motors)==1:
                    self.assertTrue(any(command.startswith('FOLLOW,') for unused,command
                                        in motors[-1].commands),value)
                    result.status='succeeded'
                else:
                    # The next command starts near the block. Reusing static
                    # preparation must not reuse the previous far approach.
                    self.assertFalse(any(command.startswith(('FOLLOW,','SPEED,','ARC,'))
                        for unused,command in motors[-1].commands),value)
                    if result.metrics['fine_started']==1:
                        result.status='succeeded'
                # The harness ends at the first permitted action (far FOLLOW
                # or near static fine), not at a simulated physical grasp.
                return value
            result.step=one_turn
            return result
        modules['ground_projection'].GroundProjection=Mock(side_effect=GroundProjection)
        modules['kinematics'].Geometry=Mock()
        modules['blue_tracker'].BlueTracker=Mock(side_effect=tracker)
        modules['vision_runtime'].track_coarse=track_coarse
        modules['motor_link'].MotorLink=Motor
        modules['robot_tracking'].TrackingTask=task
        modules['robot_tracking'].ReobserveRequired=ReobserveRequired
        class InertFine:
            def __init__(self,*args):self.phase=self.latest=None
            def start(self,*args):self.phase='hold'
            def advance(self):clock.advance(20)
            def take_evidence(self,*args):return None
            def cancel(self):self.phase=None
        modules['cooperative_static_session'].CooperativeStaticPickSession=InertFine
        captures=[0]
        def capture():
            captures[0]+=1
            return BlobImage([FAR if len(motors)==1 else NEAR]),frame(captures[0],clock.now())
        camera=SimpleNamespace(capture=capture,close=Mock())
        with patch.dict(sys.modules,modules),patch.object(app.time,'ticks_ms',clock.now,create=True),\
                patch.object(app.time,'sleep_ms',clock.advance,create=True),\
                patch.object(app.os,'exitpoint',Mock(),create=True),\
                patch('builtins.open',mock_open()):
            prepared=app.prepare_tracking(config)
            with patch.object(app,'prepare_tracking',side_effect=AssertionError('cache rebuilt')):
                for unused in range(2):
                    wake=dict(recognized_at_ms=clock.now(),ready_goal_sent_at_ms=clock.now(),
                              voice_returned_at_ms=clock.now())
                    result=app.dispatch('pick',config,camera=camera,prepared=prepared,wake_timing=wake)
                    self.assertTrue(result['metrics']['preparation_reused'])
                    self.assertEqual(result['metrics']['wake_timing'],wake)
                    self.assertIsNot(result['metrics']['wake_timing'],wake)
                    clock.advance(100)
        self.assertEqual(trace,['ready2','motor_open','ready2','motor_open'])
        self.assertEqual(len(motors),2)
        self.assertTrue(all(motor.closed and motor.syncs>=2 for motor in motors))
        self.assertTrue(all(motor.commands[0][1]=='STATUS' for motor in motors))
        self.assertEqual([state['encoder_a'] for state in initials],[100,200])
        self.assertEqual([state['encoder_b'] for state in initials],[-200,-400])
        self.assertIsNot(trackers[0],trackers[1])
        self.assertIsNot(tasks[0],tasks[1])
        self.assertIsNot(tasks[0].memory,tasks[1].memory)
        self.assertIsNot(tasks[0].drive,tasks[1].drive)
        self.assertIsNot(tasks[0].evidence,tasks[1].evidence)
        self.assertIs(tasks[0].drive.projection,tasks[1].drive.projection)
        self.assertIsNone(tasks[0].job)
        self.assertIsNone(tasks[1].job)
        self.assertEqual(tasks[0].metrics['fine_started'],0)
        self.assertEqual(tasks[1].metrics['fine_started'],1)
        modules['ground_projection'].GroundProjection.assert_called_once()
        modules['kinematics'].Geometry.assert_called_once()
        modules['app'].begin_from_observation_steps.assert_not_called()
        camera.close.assert_not_called()


if __name__=='__main__':
    unittest.main()
