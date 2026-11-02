"""Standalone arithmetic, stage orchestration and deployment; no hardware access."""
import ast
import contextlib
import importlib.util
import io
import hashlib
import os
import math
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from host.project_paths import ROOT
from tools.deploy import k230_pick as deploy

BOARD=ROOT/'device/k230/pick'
sys.path.insert(0,str(BOARD))
from numeric import rotation,matvec,dot,norm
from pose import pose_candidates,project
from kinematics import Geometry


class BoardMathTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.config,cls.files,cls.manifest=deploy.build_bundle()
    def test_two_pose_geometry_reproduces_distorted_synthetic_squares(self):
        K=[[1200.,0.,640.],[0.,1210.,480.],[0.,0.,1.]];D=[.12,-.3,.002,-.001,.2]
        for axis,angle,t in [([1.,0.,0.],.5,[10.,40.,300.]),([0.,1.,0.],-.8,[-20.,30.,250.]),([1.,0.,0.],0.,[0.,0.,280.])]:
            R=rotation(axis,angle);objects=[[-10,-10,0],[10,-10,0],[10,10,0],[-10,10,0]]
            pixels=[project([v+t[i] for i,v in enumerate(matvec(R,p))],K,D) for p in objects]
            ps=pose_candidates(pixels,K,D);normal=[row[2] for row in R]
            if dot(normal,t)<0:normal=[-v for v in normal]
            expected=[v+10*n for v,n in zip(t,normal)]
            self.assertLess(norm([a-b for a,b in zip(ps[0]['cube_center_Camera_mm'],expected)]),1e-4)
            self.assertLess(ps[0]['face_reprojection_rms_px'],1e-5)
    def test_board_fk_matches_pc_and_four_stage_ik(self):
        from host.apps.pick_validation_pc import make_plan,parameters
        pick,cfg=parameters();vision={'cube_center_Base_mm':[245.,-28.,-65.]};start=[2048]*6
        pc,pp=make_plan(vision,start,pick,cfg)
        board=Geometry(self.config['model'],pick,pp['table_z_Base_mm'])
        bp=board.plan([.245,-.028],start)
        for stage in ('hover','descend','close','lift'):
            self.assertEqual(bp[stage],pp[stage])
            a=board.report(bp[stage])['alignment_center_Base_mm'];b=pc.report(bp[stage])['alignment_center_Base_mm']
            self.assertLess(norm([v-w for v,w in zip(a,b)]),1e-8)
    def test_bundle_has_no_pc_dependencies_or_implicit_motion_calls(self):
        for name,source in self.files.items():
            if not name.endswith('.py'):continue
            tree=ast.parse(source.decode())
            for node in ast.walk(tree):
                if (isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute)
                    and node.func.attr in ('startswith','endswith') and node.args
                    and isinstance(node.args[0],ast.Tuple)):
                    self.fail(name+': MicroPython does not support tuple prefixes')
            imports=[]
            for n in ast.walk(tree):
                if isinstance(n,ast.Import):imports.extend(a.name for a in n.names)
                if isinstance(n,ast.ImportFrom):imports.append(n.module or '')
            self.assertFalse(any(i.startswith(('host','scipy','serial','pathlib')) for i in imports),name)
        self.assertEqual(self.config['pick']['stage_durations_s']['lift'],1)
        self.assertFalse(self.manifest['startup_motion'])
        self.assertFalse(self.manifest['pc_runtime_required'])
        self.assertEqual(self.config['pick']['cube_offset_toward_moving_finger_mm'],0)
        self.assertEqual(self.config['pick']['model_urdf'],'models/genkiarm/genkiarm_corrected.urdf')
        # Control helpers are definitions, not a queued sequence executed on import.
        control=ast.parse(self.files['control.py'].decode())
        for n in control.body:
            if isinstance(n,ast.Expr) and isinstance(n.value,ast.Call):self.fail('Control writes at import')


class VoiceDispatchTests(unittest.TestCase):
    def app(self):
        spec=importlib.util.spec_from_file_location('test_board_app',BOARD/'app.py');module=importlib.util.module_from_spec(spec)
        mocks={'time':SimpleNamespace(ticks_ms=lambda:0,ticks_diff=lambda a,b:a-b),'camera':SimpleNamespace(CameraSession=None),'vision':SimpleNamespace(localize=None),
               'control':SimpleNamespace(run_pick_stage_direct=None,run_ready_direct=None),
               'vision_runtime':__import__('device.k230.vision.runtime',fromlist=['localize_precise'])}
        with patch.dict(sys.modules,mocks):spec.loader.exec_module(module)
        return module
    def test_control_log_interceptor_preserves_both_event_prefixes(self):
        app=self.app()
        with patch.object(app,'log') as log:
            app.record_arm_event('ARM_EVENT {"event":"preflight_verified","rows":[]}')
            log.assert_called_with('arm_feedback',{'event':'preflight_verified','rows':[]})
            app.record_arm_event('PICK_EVENT {"event":"stage_started","name":"hover"}')
            log.assert_called_with('arm_feedback',{'event':'stage_started','name':'hover'})
        with contextlib.redirect_stdout(io.StringIO()) as output:
            app.record_arm_event('PICK_ACTION_PASS','read')
        self.assertEqual(output.getvalue(),'PICK_ACTION_PASS read\n')

    def test_startup_waits_for_wake_before_one_pick(self):
        app=self.app();calls=[]
        def wait(cfg,warmup_chunks,on_command):
            self.assertEqual(cfg,{'wake_word':'思达思达'})
            self.assertEqual(warmup_chunks,32 if not calls else 0)
            calls.append('wait')
            if len(calls)>1:raise KeyboardInterrupt()
            on_command('pick')
            return 'pick'
        with patch.object(app,'load',return_value={'pick':{'cube_offset_toward_moving_finger_mm':-45},'voice':{'wake_word':'思达思达'}}),\
             patch.object(app,'log'),patch.object(app,'wait_for_wake',side_effect=wait),\
             patch.object(app,'run_ready',side_effect=lambda cfg,pose,immediate:calls.append(('ready',pose,immediate))),\
             patch.object(app,'run_once',side_effect=lambda cfg,execute:calls.append(('pick',execute))):
            with self.assertRaises(KeyboardInterrupt):app.run_forever('/fake')
        self.assertEqual(calls,['wait',('ready',2,True),('pick',True),'wait'])
    def test_failure_does_not_automatically_retry_pick(self):
        app=self.app();calls=[]
        def wait(cfg,warmup_chunks,on_command):
            self.assertEqual(cfg,{'wake_word':'思达思达'})
            self.assertEqual(warmup_chunks,32 if not calls else 0)
            calls.append('wait')
            if len(calls)>1:raise KeyboardInterrupt()
            on_command('pick')
            return 'pick'
        def pick(cfg,execute):calls.append('failed_pick');raise ValueError('No face')
        with patch.object(app,'load',return_value={'pick':{'cube_offset_toward_moving_finger_mm':-45},'voice':{'wake_word':'思达思达'}}),\
             patch.object(app,'log'),patch.object(app,'wait_for_wake',side_effect=wait),\
             patch.object(app,'run_ready',side_effect=lambda cfg,pose,immediate:calls.append(('ready',pose,immediate))),\
             patch.object(app,'run_once',side_effect=pick),patch.object(sys,'print_exception',lambda e:None,create=True):
            with self.assertRaises(KeyboardInterrupt):app.run_forever('/fake')
        self.assertEqual(calls,['wait',('ready',2,True),'failed_pick','wait'])
    def test_real_startup_resolves_nested_voice_config_before_readiness(self):
        app=self.app();calls=[]
        class Engine:
            def add_keyword(self,r):calls.append('keyword')
            def reset(self):calls.append('reset')
            def deinit(self):calls.append('engine_closed')
        class Audio:
            def open(self,**kw):return Stream()
            def terminate(self):calls.append('audio_closed')
        class Stream:
            def volume(self,**kw):pass
            def read(self):calls.append('read');return bytes(5120)
            def stop_stream(self):pass
            def close(self):calls.append('stream_closed')
        class StopAtReady(BaseException):pass
        def log(event,*args,**kwargs):
            calls.append(event)
            if event=='voice_ready':raise StopAtReady()
        cfg,unused,manifest=deploy.build_bundle()
        cfg['voice']['commands']['ready']['enrollment_file']=None
        with patch.dict(sys.modules,{'wake_word':SimpleNamespace(engine=lambda s:Engine(),CHUNK=2560),
                                     'media':SimpleNamespace(),'media.pyaudio':SimpleNamespace(PyAudio=Audio,paInt16=0)}),\
             patch.object(app.os,'exitpoint',lambda:None,create=True),\
             patch.object(app,'load',return_value=cfg),patch.object(app,'log',side_effect=log),\
             patch('builtins.open',return_value=io.StringIO('{"kw_id":"sida"}')),patch.object(app,'run_once') as pick:
            with self.assertRaises(StopAtReady):app.run_forever('/fake')
            pick.assert_not_called()
        self.assertIn('voice_ready',calls);self.assertEqual(calls.count('read'),32)
        self.assertEqual(calls[-3:],['stream_closed','audio_closed','engine_closed'])

    def test_voice_initialization_failure_is_reported_and_no_motion_runs(self):
        app=self.app()
        with patch.object(app,'load',return_value={'pick':{'cube_offset_toward_moving_finger_mm':-45},'voice':{}}),\
             patch.object(app,'wait_for_wake',side_effect=RuntimeError('microphone unavailable')),\
             patch.object(app,'run_once') as pick,patch.object(app,'log') as log,\
             patch.object(sys,'print_exception',lambda e:None,create=True):
            with self.assertRaisesRegex(RuntimeError,'microphone unavailable'):app.run_forever('/fake')
            pick.assert_not_called();log.assert_any_call('voice_failed','microphone unavailable')

    def test_audio_resources_close_before_wake_returns(self):
        app=self.app();calls=[]
        class Engine:
            def add_keyword(self,r):pass
            def reset(self):pass
            def push(self,pcm):return [('sida',-.7,-1.9,3040.,3800.)]
            def deinit(self):calls.append('engine_closed')
        class Audio:
            def open(self,**kw):return Stream()
            def terminate(self):calls.append('audio_closed')
        class Stream:
            def volume(self,**kw):pass
            def read(self):return bytes(5120)
            def stop_stream(self):calls.append('stream_stopped')
            def close(self):calls.append('stream_closed')
        with tempfile.TemporaryDirectory() as folder:
            file=Path(folder)/'keyword.json';file.write_text('{"kw_id":"sida"}')
            with patch.dict(sys.modules,{'wake_word':SimpleNamespace(engine=lambda s:Engine(),CHUNK=2560),
                                         'media':SimpleNamespace(),'media.pyaudio':SimpleNamespace(PyAudio=Audio,paInt16=0)}),\
                 patch.object(app.os,'exitpoint',lambda:None,create=True),patch.object(app,'log') as log,\
                 patch.object(app.time,'ticks_ms',side_effect=[0,5000]):
                event=app.wait_for_wake({'enrollment_file':str(file),'detection_sensitivity':'sensitive','wake_word':'思达思达'})
        self.assertEqual(event,'pick')
        self.assertNotIn('voice_listening',[call.args[0] for call in log.call_args_list])
        self.assertEqual(calls,['stream_stopped','stream_closed','audio_closed','engine_closed'])


    def voice_response_order(self,action='pick',fail_ready=False):
        app=self.app();trace=[];config,_,_=deploy.build_bundle()
        keyword_id='sida' if action=='pick' else 'ready'
        class Engine:
            def add_keyword(self,record):pass
            def reset(self):pass
            def push(self,pcm):trace.append('recognized');return [(keyword_id,-.7,-2.,0,10)]
            def deinit(self):trace.append('engine_closed')
        class Audio:
            def open(self,**kw):return Stream()
            def terminate(self):trace.append('audio_closed')
        class Stream:
            def volume(self,**kw):pass
            def read(self):return bytes(5120)
            def stop_stream(self):trace.append('stream_stopped')
            def close(self):trace.append('stream_closed')
        def ready(cfg,pose=1,immediate=False):
            trace.append(('ready_packet',pose,immediate))
            if fail_ready:raise RuntimeError('UART response failed')
        def log(event,*args,**kwargs):trace.append(('log',event))
        with tempfile.TemporaryDirectory() as folder:
            a=Path(folder)/'sida.json';b=Path(folder)/'ready.json'
            a.write_text('{"kw_id":"sida"}');b.write_text('{"kw_id":"ready"}')
            config['voice']['enrollment_file']=str(a)
            config['voice']['commands']['ready']['enrollment_file']=str(b)
            with patch.dict(sys.modules,{'wake_word':SimpleNamespace(engine=lambda s:Engine(),CHUNK=2560),
                     'media':SimpleNamespace(),'media.pyaudio':SimpleNamespace(PyAudio=Audio,paInt16=0)}),\
                 patch.object(app.os,'exitpoint',lambda:None,create=True),patch.object(app,'log',side_effect=log),\
                 patch.object(app,'run_ready',side_effect=ready):
                response,on_command=app.make_wake_response(config)
                result=app.wait_for_wake(config['voice'],warmup_chunks=0,on_command=on_command)
                trace.append('returned')
        self.assertEqual(result,action)
        return response,trace

    def test_recognized_pick_sends_pose2_before_wake_log_and_audio_cleanup(self):
        response,trace=self.voice_response_order()
        self.assertTrue(response['sent']);self.assertIsNone(response['error'])
        sent=trace.index(('ready_packet',2,True))
        self.assertLess(trace.index('recognized'),sent)
        for later in (('log','wake'),'stream_stopped','stream_closed','audio_closed','engine_closed','returned'):
            self.assertLess(sent,trace.index(later))

    def test_ready_keyword_sends_pose1_before_audio_cleanup(self):
        response,trace=self.voice_response_order('ready')
        self.assertTrue(response['sent'])
        self.assertLess(trace.index(('ready_packet',1,True)),trace.index(('log','wake')))
        self.assertLess(trace.index(('ready_packet',1,True)),trace.index('stream_stopped'))

    def test_response_failure_still_cleans_audio_before_caller_can_start_task(self):
        response,trace=self.voice_response_order(fail_ready=True)
        self.assertFalse(response['sent']);self.assertIsInstance(response['error'],RuntimeError)
        self.assertEqual(trace[-5:],['stream_stopped','stream_closed','audio_closed','engine_closed','returned'])

    def test_failed_immediate_response_never_captures_or_retries_and_listens_again(self):
        app=self.app();config,_,_=deploy.build_bundle();waits=[]
        def wait(voice,warmup_chunks,on_command):
            waits.append(warmup_chunks)
            if len(waits)>1:raise KeyboardInterrupt()
            on_command('pick');return 'pick'
        with patch.object(app,'load',return_value=config),patch.object(app,'log') as logs,\
             patch.object(app,'wait_for_wake',side_effect=wait),\
             patch.object(app,'run_ready',side_effect=RuntimeError('UART not ready')) as ready,\
             patch.object(app,'run_once') as capture,patch.object(sys,'print_exception',lambda e:None,create=True):
            with self.assertRaises(KeyboardInterrupt):app.run_forever('/fake')
        ready.assert_called_once_with(config,pose=2,immediate=True)
        capture.assert_not_called();self.assertEqual(waits,[32,0])
        logs.assert_any_call('pick_failed_no_extra_motion','UART not ready')


class ReadyCommandTests(unittest.TestCase):
    app=VoiceDispatchTests.app
    def test_immediate_response_uses_selected_target_before_any_ready_log(self):
        app=self.app();config,_,_=deploy.build_bundle()
        for pose,key,event in ((1,'ready_control','ready_dispatched'),
                               (2,'drive_ready_control','drive_ready_dispatched')):
            with self.subTest(pose=pose):
                trace=[]
                def send(control):trace.append(('packet',control));return {'goal_sent_ticks_ms':123}
                with patch.dict(sys.modules,{'control':SimpleNamespace()}),\
                     patch.object(app,'log',side_effect=lambda name,*a,**kw:trace.append(('log',name))),\
                     patch.object(app,'run_ready_direct',side_effect=send):
                    app.run_ready(config,pose=pose,immediate=True)
                self.assertEqual(trace,[('packet',config[key]),('log',event)])

    def test_unknown_ready_pose_or_unknown_keyword_never_dispatches(self):
        app=self.app();config,_,_=deploy.build_bundle()
        with patch.object(app,'run_ready_direct') as send,patch.object(app,'run_ready',wraps=app.run_ready) as ready:
            with self.assertRaises(ValueError):app.run_ready(config,pose=3)
            ready.reset_mock()
            response,callback=app.make_wake_response(config);callback('unknown')
            ready.assert_not_called();send.assert_not_called();self.assertFalse(response['sent'])

    def test_unregistered_ready_keeps_original_keyword(self):
        from voice_commands import registered_commands
        cfg,_,_=deploy.build_bundle()
        cfg['voice']['commands']['ready']['enrollment_file']=None
        self.assertEqual([c['action'] for c in registered_commands(cfg['voice'])],['pick'])

    def test_distinct_ids_route_and_ambiguous_events_do_not_move(self):
        from voice_commands import registered_commands,resolve_events
        cfg,_,_=deploy.build_bundle()
        cfg['voice']['commands']['ready']['enrollment_file']='/fake/ready.json'
        commands=registered_commands(cfg['voice'])
        self.assertEqual(resolve_events([('ready',-.7,-2.,0,10)],commands)['action'],'ready')
        self.assertEqual(resolve_events([('sida',-.7,-2.,0,10)],commands)['action'],'pick')
        for events in ([('other',)], [('sida',),('ready',)], [{'label':'ready'}], []):
            self.assertIsNone(resolve_events(events,commands))
        cfg['voice']['commands']['ready']['keyword_id']='sida'
        with self.assertRaisesRegex(ValueError,'duplicate'):registered_commands(cfg['voice'])

    def test_ready_uses_existing_goals_and_profiles(self):
        from tools.diagnostics.k230_arm_ready import config_payload
        cfg,_,_=deploy.build_bundle()
        self.assertEqual(cfg['ready_control'],config_payload(1))
        self.assertEqual(cfg['drive_ready_control'],config_payload(2))
        self.assertEqual(cfg['ready_control']['goals'],[2048,1081,3015,3015,2048,2332])
        self.assertEqual(cfg['drive_ready_control']['goals'],[2048,1138,2958,2958,2048,2332])
        self.assertEqual(cfg['ready_control']['goals'][5],2332)
        self.assertEqual(cfg['ready_control']['gripper_opens_deg'],25)
        self.assertEqual(cfg['pick']['opening_count'],2332)
        self.assertEqual(cfg['pick']['opening_angle_deg'],25)
        self.assertEqual(cfg['ready_control']['profiles'],cfg['control']['profiles'])
        self.assertEqual(cfg['ready_control']['registers'],cfg['control']['registers'])
        app=self.app();calls=[]
        with patch.dict(sys.modules,{'control':SimpleNamespace()}),patch.object(app,'log'),\
             patch.object(app,'run_ready_direct',side_effect=lambda config:calls.append(config)),\
             patch.object(app,'run_once') as pick:
            app.run_ready(cfg)
            pick.assert_not_called()
        self.assertEqual(calls,[cfg['ready_control']])

    def test_ready_dispatch_failure_is_not_retried(self):
        app=self.app()
        with patch.dict(sys.modules,{'control':SimpleNamespace()}),patch.object(app,'log'),\
             patch.object(app,'run_ready_direct',side_effect=RuntimeError('write failed')) as run:
            with self.assertRaises(RuntimeError):app.run_ready({'ready_control':{'goals':[2048]*6}})
            run.assert_called_once_with({'goals':[2048]*6})

    def test_ready_command_bypasses_camera_and_returns_to_listening(self):
        app=self.app();cfg={'pick':{'cube_offset_toward_moving_finger_mm':-45},'voice':{}}
        with patch.object(app,'load',return_value=cfg),patch.object(app,'log'),\
             patch.object(app,'wait_for_wake',side_effect=['ready',KeyboardInterrupt()]) as listen,\
             patch.object(app,'run_once') as pick,patch.object(app,'run_ready') as ready:
            with self.assertRaises(KeyboardInterrupt):app.run_forever('/fake')
            ready.assert_called_once_with(cfg);pick.assert_not_called();self.assertEqual(listen.call_count,2)

    def test_both_keywords_loaded_and_actual_tuple_selects_ready(self):
        self.check_dual_listener(32)

    def test_resumed_listener_resets_engine_and_processes_first_audio_chunk(self):
        self.check_dual_listener(0)

    def check_dual_listener(self,warmup_chunks):
        app=self.app();added=[];closed=[];reads=[];resets=[]
        class Engine:
            def add_keyword(self,r):added.append(r['kw_id'])
            def reset(self):resets.append(True)
            def push(self,pcm):
                self_check=resets==[True]
                if not self_check:raise AssertionError('Recognition state must reset before processing audio')
                return [('ready',-.7,-2.,0,10)]
            def deinit(self):closed.append('engine')
        class Audio:
            def open(self,**kw):return Stream()
            def terminate(self):closed.append('audio')
        class Stream:
            def volume(self,**kw):pass
            def read(self):reads.append(1);return bytes(5120)
            def stop_stream(self):pass
            def close(self):closed.append('stream')
        with tempfile.TemporaryDirectory() as folder:
            a=Path(folder)/'sida.json';b=Path(folder)/'ready.json'
            a.write_text('{"kw_id":"sida"}');b.write_text('{"kw_id":"ready"}')
            config={'wake_word':'思达思达','enrollment_file':str(a),'detection_sensitivity':'sensitive',
                    'commands':{'ready':{'keyword_id':'ready','word':'回准备位','action':'ready','enrollment_file':str(b)}}}
            with patch.dict(sys.modules,{'wake_word':SimpleNamespace(engine=lambda s:Engine(),CHUNK=2560),
                    'media':SimpleNamespace(),'media.pyaudio':SimpleNamespace(PyAudio=Audio,paInt16=0)}),\
                 patch.object(app.os,'exitpoint',lambda:None,create=True),patch.object(app,'log') as log:
                self.assertEqual(app.wait_for_wake(config,warmup_chunks=warmup_chunks),'ready')
            log.assert_any_call('voice_ready',{'words':['思达思达','回准备位']})
        self.assertEqual(added,['sida','ready']);self.assertEqual(closed,['stream','audio','engine'])
        self.assertEqual(len(reads),warmup_chunks+1)



class DirectReadyTransportTests(unittest.TestCase):
    def run_direct(self,torques=None,fail_at=None,fail_read_at=None,fail_enable=None,invalid_goal=False,pose=1):
        writes=[];events=[];closed=[];reads=[];states=list([1]*6 if torques is None else torques)
        positions=[1900+i*11 for i in range(6)]
        class Pins:
            UART1_TXD=1;UART1_RXD=2
            def set_function(self,*args):pass
        class Port:
            UART1=1;EIGHTBITS=8;PARITY_NONE=None;STOPBITS_ONE=1
            def __init__(self,*args,**kwargs):pass
            def read(self,*args):raise AssertionError('Use checked torque/position reads only')
            def write(self,packet):
                writes.append(packet);events.append(('write',packet))
                return 0 if len(writes)==fail_at else len(packet)
            def deinit(self):closed.append(True);events.append(('closed',))
        fake_time=SimpleNamespace(ticks_ms=lambda:0,ticks_diff=lambda a,b:a-b,
                                  sleep_ms=lambda n:events.append(('sleep_ms',n)))
        ns={}
        with patch.dict(sys.modules,{'machine':SimpleNamespace(UART=Port,FPIOA=Pins),
                                    'time':fake_time,'ujson':json}):
            exec((ROOT/'device/k230/arm/ready_pose.py').read_text(),ns)
        def read(uart,i,address,size):
            event=('read',i,address,size);events.append(event);reads.append(event)
            if len(reads)==fail_read_at:raise RuntimeError('Read timeout')
            if (address,size)==(40,1):return bytes((states[i-1],))
            self.assertEqual((address,size),(56,2))
            value=positions[i-1];return bytes((value&255,value>>8))
        def enable(uart,i):
            # Stub the existing ACK handler, including its torque readback.
            events.append(('enable_ack',i))
            if i==fail_enable:raise RuntimeError('Torque ACK failed')
            states[i-1]=1;self.assertEqual(read(uart,i,40,1),b'\x01')
        def forbidden(*args,**kwargs):raise AssertionError('No preflight, feedback or recovery')
        ns.update(read_checked=read,enable_torque=enable,signed_position=lambda lo,hi:lo|(hi<<8),
                  preflight=forbidden,feedback=forbidden,snapshot=forbidden,send_sync=forbidden)
        cfg,_,_=deploy.build_bundle();control=cfg['ready_control' if pose==1 else 'drive_ready_control']
        if invalid_goal:control['goals'][0]=-1
        error=None;result=None
        with contextlib.redirect_stdout(io.StringIO()) as output:
            try:result=ns['run_ready_direct'](control)
            except (RuntimeError,ValueError) as exc:error=exc
        return SimpleNamespace(config=control,writes=writes,events=events,closed=closed,error=error,
                               result=result,output=output.getvalue(),positions=positions,encode=ns['sync_frame'])

    def test_all_enabled_reads_only_torque_then_one_final_goal_without_post_reads(self):
        r=self.run_direct();self.assertIsNone(r.error);self.assertEqual(r.closed,[True])
        packet=r.encode(r.config['goals'],r.config['profiles'])
        self.assertEqual(r.events,[('read',i,40,1) for i in range(1,7)]+
                         [('write',packet),('sleep_ms',2),('closed',)])
        self.assertEqual(r.result['torque_enabled_ids'],[])
        self.assertEqual(r.result['servo_reads'],6);self.assertEqual(r.result['torque_writes'],0)
        self.assertEqual(r.result['goal_packet_count'],1);self.assertFalse(r.result['readback_verified'])
        self.assertEqual(r.result['goal_sent_ticks_ms'],0)

    def test_disabled_axes_hold_current_before_enable_and_final_goal(self):
        for torques,disabled in [([1,0,1,0,1,1],[2,4]),([0]*6,list(range(1,7)))]:
            with self.subTest(torques=torques):
                r=self.run_direct(torques=torques);self.assertIsNone(r.error)
                hold=list(r.config['goals'])
                for i in disabled:hold[i-1]=r.positions[i-1]
                hold_packet=r.encode(hold,r.config['profiles'],disabled)
                final_packet=r.encode(r.config['goals'],r.config['profiles'])
                expected=[('read',i,40,1) for i in range(1,7)]
                expected += [('read',i,56,2) for i in disabled]
                expected += [('write',hold_packet),('sleep_ms',2)]
                for i in disabled:expected += [('enable_ack',i),('read',i,40,1)]
                expected += [('write',final_packet),('sleep_ms',2),('closed',)]
                self.assertEqual(r.events,expected)
                self.assertEqual(list(hold_packet[7:-1:8]),disabled)
                self.assertEqual(r.result['torque_enabled_ids'],disabled)
                self.assertEqual(r.result['servo_reads'],6+2*len(disabled))
                self.assertEqual(r.result['hold_packet_count'],1)
                self.assertEqual(r.result['torque_writes'],len(disabled));self.assertEqual(r.closed,[True])

    def test_final_target_packet_matches_existing_sdk(self):
        from host.project_paths import bootstrap
        bootstrap()
        from scservo_sdk import sms_sts,COMM_SUCCESS
        r=self.run_direct();sdk=sms_sts(None);captured=[]
        def tx(address,size,params,length):
            captured.append((address,size,list(params),length));return COMM_SUCCESS
        sdk.syncWriteTxOnly=tx
        for i in range(1,7):
            p=r.config['profiles'][i-1]
            self.assertTrue(sdk.SyncWritePosEx(i,r.config['goals'][i-1],p['speed_raw'],p['acceleration_raw']))
        sdk.groupSyncWrite.txPacket();address,size,params,length=captured[0];packet=r.writes[-1]
        self.assertEqual(packet[:5],bytes((255,255,254,length+4,0x83)))
        self.assertEqual(packet[5:-1],bytes([address,size]+params));self.assertEqual(sum(packet[2:])&255,255)

    def test_short_hold_does_not_enable_or_send_ready(self):
        r=self.run_direct(torques=[1,0,1,1,1,1],fail_at=1)
        self.assertIsInstance(r.error,RuntimeError);self.assertEqual(len(r.writes),1)
        self.assertFalse(any(e[0]=='enable_ack' for e in r.events));self.assertEqual(r.closed,[True])

    def test_enable_ack_or_readback_failure_does_not_send_ready(self):
        for options in ({'fail_enable':2},{'fail_read_at':9}):
            with self.subTest(options=options):
                r=self.run_direct(torques=[1,0,1,0,1,1],**options)
                self.assertIsInstance(r.error,RuntimeError);self.assertEqual(len(r.writes),1)
                self.assertNotIn(('enable_ack',4),r.events);self.assertEqual(r.closed,[True])

    def test_invalid_torque_or_failed_initial_read_never_writes(self):
        for options in ({'torques':[1,1,2,1,1,1]}, {'fail_read_at':3},
                        {'torques':[1,0,1,1,1,1],'fail_read_at':7}):
            with self.subTest(options=options):
                r=self.run_direct(**options);self.assertIsInstance(r.error,RuntimeError)
                self.assertEqual(r.writes,[]);self.assertEqual(r.closed,[True])

    def test_short_final_goal_does_not_monitor_or_send_recovery(self):
        for torques,fail_at in [([1]*6,1),([1,0,1,1,1,1],2)]:
            with self.subTest(torques=torques):
                r=self.run_direct(torques=torques,fail_at=fail_at)
                self.assertIsInstance(r.error,RuntimeError);self.assertEqual(len(r.writes),fail_at)
                self.assertEqual(r.events[-2:],[('write',r.writes[-1]),('closed',)])
                self.assertEqual(r.closed,[True])

    def test_invalid_final_goal_fails_before_torque_reads_or_writes(self):
        r=self.run_direct(torques=[0]*6,invalid_goal=True)
        self.assertIsInstance(r.error,ValueError);self.assertEqual(r.events,[('closed',)])
        self.assertEqual(r.writes,[])

class DirectPickTests(unittest.TestCase):
    def run_stage(self,name,positions=(),short_at=None,fail_read_at=None):
        cfg,files,_=deploy.build_bundle();writes=[];reads=[];closed=[];clock=[0];values=iter(positions)
        class Pins:
            UART1_TXD=1;UART1_RXD=2
            def set_function(self,*args):pass
        class Port:
            UART1=1;EIGHTBITS=8;PARITY_NONE=None;STOPBITS_ONE=1
            def __init__(self,*args,**kwargs):pass
            def write(self,p):
                writes.append((clock[0],p));return 0 if len(writes)==short_at else len(p)
            def deinit(self):closed.append(True)
        def read(uart,i,address,size):
            self.assertTrue(writes,'No pre-dispatch reads allowed')
            self.assertEqual((address,size),(56,2));reads.append((clock[0],i));clock[0]+=30
            if len(reads)==fail_read_at:raise RuntimeError('read timeout')
            v=next(values);return bytes((v&255,v>>8))
        def forbidden(*args,**kwargs):raise AssertionError('Legacy preflight/hold must not run')
        def sleep(ms):clock[0]+=ms
        ns={}
        with patch.dict(sys.modules,{'machine':SimpleNamespace(UART=Port,FPIOA=Pins),
                  'time':SimpleNamespace(ticks_ms=lambda:clock[0],ticks_diff=lambda a,b:a-b,sleep_ms=sleep),'ujson':json}):
            exec(files['control.py'].decode(),ns)
        ns.update(read_checked=read,preflight=forbidden,initialize_hold=forbidden,send_sync=forbidden)
        control=dict(cfg['control']);control['goals']=[2048]*6
        stage={'name':name,'duration_s':1,'policy':cfg['pick']['standalone_stage_policy']}
        error=None;result=None
        with contextlib.redirect_stdout(io.StringIO()) as output:
            try:result=ns['run_pick_stage_direct'](control,stage)
            except RuntimeError as exc:error=exc
        return SimpleNamespace(writes=writes,reads=reads,closed=closed,elapsed=clock[0],error=error,
             result=result,expected=ns['sync_frame'](control['goals'],control['profiles']),output=output.getvalue())

    def packet_ids(self,packet):
        self.assertEqual(packet[:3],b'\xff\xff\xfe')
        self.assertEqual(packet[4:7],bytes((0x83,41,7)))
        self.assertEqual(packet[3]+4,len(packet));self.assertEqual(sum(packet[2:])&255,255)
        return list(packet[7:-1:8])

    def test_hover_requires_all_six_within_inclusive_30_without_six_second_delay(self):
        r=self.run_stage('hover',[2048]*5+[2079]+[2018,2078,2048,2048,2048,2078])
        self.assertIsNone(r.error);self.assertEqual(len(r.reads),12)
        self.assertEqual([i for t,i in r.reads],list(range(1,7))*2)
        self.assertEqual(r.result['errors_actual_minus_goal'],[-30,30,0,0,0,30])
        self.assertLess(r.elapsed,1000);self.assertEqual(r.writes,[(0,r.expected)])
        self.assertEqual(r.closed,[True])

    def test_descend_sends_only_axis4_then_other_five_after_200ms(self):
        r=self.run_stage('descend',[2059]+[2048]*5+[2038,2058,2048,2048,2048,2058])
        self.assertIsNone(r.error);self.assertEqual(len(r.writes),2)
        self.assertEqual(self.packet_ids(r.writes[0][1]),[4]);self.assertEqual(self.packet_ids(r.writes[1][1]),[1,2,3,5,6])
        self.assertEqual(r.writes[1][0]-r.writes[0][0],200)
        self.assertEqual(len(r.reads),12);self.assertGreaterEqual(r.reads[0][0],202)
        # Every partial goal/profile is identical to its entry in the full packet.
        for _,packet in r.writes:
            for start in range(7,len(packet)-1,8):
                entry=packet[start:start+8];offset=7+(entry[0]-1)*8
                self.assertEqual(entry,r.expected[offset:offset+8])
        self.assertEqual(r.result['errors_actual_minus_goal'],[-10,10,0,0,0,10])

    def test_each_gate_poll_uses_fresh_values_not_per_axis_latched_success(self):
        # The first two polls each have a DIFFERENT out-of-band axis.
        r=self.run_stage('hover',[2079]+[2048]*5+[2048,2079]+[2048]*4+[2048]*6)
        self.assertEqual(len(r.reads),18);self.assertEqual(r.result['positions'],[2048]*6)

    def test_close_accepts_five_readings_in_one_plus_minus_one_band(self):
        r=self.run_stage('close',[2200,2201,2199,2200,2201])
        self.assertIsNone(r.error);self.assertEqual(len(r.reads),5)
        self.assertTrue(all(i==6 for t,i in r.reads));self.assertLess(r.elapsed,500)
        self.assertIn('"sample_count": 5',r.output);self.assertEqual(r.writes,[(0,r.expected)])

    def test_close_does_not_accept_creeping_single_count_motion(self):
        r=self.run_stage('close',list(range(2200,2210))+[2209]*5)
        self.assertIsNone(r.error);self.assertGreater(len(r.reads),10)
        self.assertIn('"sample_count": 5',r.output)

    def test_lift_retains_one_second_and_no_encoder_reads(self):
        r=self.run_stage('lift');self.assertIsNone(r.error);self.assertEqual(r.reads,[])
        self.assertEqual(r.writes,[(0,r.expected)]);self.assertEqual(r.elapsed,1002)

    def test_short_writes_and_missing_feedback_do_not_advance(self):
        for name,short_at,count in [('hover',1,1),('descend',1,1),('descend',2,2)]:
            r=self.run_stage(name,short_at=short_at)
            self.assertIsInstance(r.error,RuntimeError);self.assertEqual(len(r.writes),count)
            self.assertEqual(r.reads,[]);self.assertEqual(r.closed,[True])
        for name in ('hover','descend','close'):
            r=self.run_stage(name,fail_read_at=1)
            self.assertIsInstance(r.error,RuntimeError);self.assertEqual(r.closed,[True]);self.assertIsNone(r.result)

    def test_plan_to_stages_reuses_arrival_sample_without_extra_servo_read(self):
        self.check_flow(False)

    def test_descend_gate_read_failure_prevents_close_and_lift(self):
        self.check_flow(True)

    def check_flow(self,fail_read):
        app=VoiceDispatchTests().app();cfg,_,_=deploy.build_bundle();events=[];stages=[]
        class Camera:
            def __init__(self,*args):pass
            def open(self):pass
            def capture(self):return SimpleNamespace(save=lambda *a,**kw:None),{'frame_id':1,'capture_timestamp_ms':0,'clock_domain':'k230_ticks_ms','image_size_px':[1280,960]}
            def close(self):pass
        plan={name:[2048]*6 for name in ('hover','descend','close','lift')}
        class Model:
            def __init__(self,*args):pass
            def plan(self,*args):events.append('plan');return plan
        sample={'goals':plan['descend'],'positions':[2048+i for i in range(6)],
                'errors_actual_minus_goal':list(range(6)),'read_ms':180,'source':'arrival_gate'}
        def stage(c,s):
            events.append(s['name']);stages.append(s)
            if s['name']=='descend':
                if fail_read:raise RuntimeError('read timeout')
                return sample
        with patch.dict(sys.modules,{'cv2':SimpleNamespace(imread=lambda p:object()),'control':SimpleNamespace()}), \
             patch.object(app,'CameraSession',Camera),patch.object(app,'Geometry',Model), \
             patch.object(app,'localize',return_value={'status':'complete_face_candidate','poses':[{'face_reprojection_rms_px':.5}],'cube_center_Base_mm':[240,0,-60]}), \
             patch.object(app,'arm_snapshot',return_value=[{'position':2048} for i in range(6)]) as snapshot, \
             patch.object(app,'log') as logs,patch.object(app,'run_pick_stage_direct',side_effect=stage):
            if fail_read:
                with self.assertRaisesRegex(RuntimeError,'read timeout'):app.run_once(cfg,execute=True)
            else:app.run_once(cfg,execute=True)
        snapshot.assert_called_once();self.assertTrue(all(s['policy']==cfg['pick']['standalone_stage_policy'] for s in stages))
        self.assertEqual(events,['plan','hover','descend'] if fail_read else ['plan','hover','descend','close','lift'])
        if not fail_read:logs.assert_any_call('descend_position_sample',sample)


class DeploymentTests(unittest.TestCase):
    def fat_execute(self,fail_replacement=False):
        original_rename=os.rename;calls=[]
        def rename(source,target):
            if str(source).endswith('.partial'):
                calls.append((source,target))
                if Path(target).exists():raise FileExistsError(17,'FAT target exists')
                if fail_replacement:raise OSError(5,'write failed')
            original_rename(source,target)
        fake_os=SimpleNamespace(rename=rename,remove=os.remove,stat=os.stat)
        def execute(link,source,timeout=60):
            output=io.StringIO()
            with patch.dict(sys.modules,{'os':fake_os,'uhashlib':hashlib}),contextlib.redirect_stdout(output):
                exec(source,{})
            return output.getvalue().splitlines()
        return execute
    def test_fat_replacement_keeps_verified_new_file(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'current.json';path.write_bytes(b'old')
            with patch.object(deploy,'execute',side_effect=self.fat_execute()):
                deploy.write_file(None,str(path),b'new')
            self.assertEqual(path.read_bytes(),b'new')
            self.assertFalse(Path(str(path)+'.replace_previous').exists())
    def test_fat_replacement_failure_restores_original(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'current.json';path.write_bytes(b'old')
            with patch.object(deploy,'execute',side_effect=self.fat_execute(True)):
                with self.assertRaises(OSError):deploy.write_file(None,str(path),b'new')
            self.assertEqual(path.read_bytes(),b'old')
            self.assertFalse(Path(str(path)+'.replace_previous').exists())

    def test_verification_failure_never_activates_startup(self):
        writes=[]
        with tempfile.TemporaryDirectory() as folder:
            cfg,files,manifest=deploy.build_bundle()
            link=SimpleNamespace(port=SimpleNamespace(close=lambda:None))
            with patch.object(deploy,'ROOT',Path(folder)),patch.object(deploy,'build_bundle',return_value=(cfg,files,manifest)),\
                 patch.object(deploy,'CameraLink',return_value=link),patch.object(deploy,'mkdir'),\
                 patch.object(deploy,'write_file',side_effect=lambda link,path,data:writes.append(path)),\
                 patch.object(deploy,'verify',side_effect=RuntimeError('checksum mismatch')),\
                 contextlib.redirect_stdout(io.StringIO()),contextlib.redirect_stderr(io.StringIO()):
                code=deploy.main(['--install','--activate'])
        self.assertEqual(code,1)
        self.assertNotIn('/sdcard/main.py',writes)
        self.assertNotIn('/sdcard/aic/pick/current.json',writes)
