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
        self.assertEqual(self.config['pick']['cube_offset_toward_moving_finger_mm'],-45)
        # Control helpers are definitions, not a queued sequence executed on import.
        control=ast.parse(self.files['control.py'].decode())
        for n in control.body:
            if isinstance(n,ast.Expr) and isinstance(n.value,ast.Call):self.fail('Control writes at import')


class VoiceDispatchTests(unittest.TestCase):
    def app(self):
        spec=importlib.util.spec_from_file_location('test_board_app',BOARD/'app.py');module=importlib.util.module_from_spec(spec)
        mocks={'time':SimpleNamespace(ticks_ms=lambda:0,ticks_diff=lambda a,b:a-b),'camera':SimpleNamespace(CameraSession=None),'vision':SimpleNamespace(localize=None),
               'control':SimpleNamespace(run_pick_stage_direct=None,run_ready_direct=None)}
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
        def wait(cfg,warmup_chunks):
            self.assertEqual(cfg,{'wake_word':'思达思达'})
            self.assertEqual(warmup_chunks,32 if not calls else 0)
            calls.append('wait')
            if len(calls)>1:raise KeyboardInterrupt()
            return 'pick'
        with patch.object(app,'load',return_value={'pick':{'cube_offset_toward_moving_finger_mm':-45},'voice':{'wake_word':'思达思达'}}),\
             patch.object(app,'log'),patch.object(app,'wait_for_wake',side_effect=wait),\
             patch.object(app,'run_once',side_effect=lambda cfg,execute:calls.append(('pick',execute))):
            with self.assertRaises(KeyboardInterrupt):app.run_forever('/fake')
        self.assertEqual(calls,['wait',('pick',True),'wait'])
    def test_failure_does_not_automatically_retry_pick(self):
        app=self.app();calls=[]
        def wait(cfg,warmup_chunks):
            self.assertEqual(cfg,{'wake_word':'思达思达'})
            self.assertEqual(warmup_chunks,32 if not calls else 0)
            calls.append('wait')
            if len(calls)>1:raise KeyboardInterrupt()
            return 'pick'
        def pick(cfg,execute):calls.append('failed_pick');raise ValueError('No face')
        with patch.object(app,'load',return_value={'pick':{'cube_offset_toward_moving_finger_mm':-45},'voice':{'wake_word':'思达思达'}}),\
             patch.object(app,'log'),patch.object(app,'wait_for_wake',side_effect=wait),\
             patch.object(app,'run_once',side_effect=pick),patch.object(sys,'print_exception',lambda e:None,create=True):
            with self.assertRaises(KeyboardInterrupt):app.run_forever('/fake')
        self.assertEqual(calls,['wait','failed_pick','wait'])
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
        log.assert_any_call('voice_listening',{'chunks_read':1,'audio_ms':160},persist=False)
        self.assertEqual(calls,['stream_stopped','stream_closed','audio_closed','engine_closed'])


class ReadyCommandTests(unittest.TestCase):
    app=VoiceDispatchTests.app
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
        self.assertEqual(cfg['ready_control'],config_payload())
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
    def run_direct(self,fail_at=None):
        writes=[];closed=[]
        class Pins:
            UART1_TXD=1;UART1_RXD=2
            def set_function(self,*args):pass
        class Port:
            UART1=1;EIGHTBITS=8;PARITY_NONE=None;STOPBITS_ONE=1
            def __init__(self,*args,**kwargs):pass
            def read(self,*args):raise AssertionError('Direct ready must never read servos')
            def write(self,packet):
                writes.append(packet)
                return 0 if len(writes)==fail_at else len(packet)
            def deinit(self):closed.append(True)
        fake_time=SimpleNamespace(ticks_ms=lambda:0,ticks_diff=lambda a,b:a-b,
                                  sleep_ms=lambda n:self.assertEqual(n,2))
        ns={}
        with patch.dict(sys.modules,{'machine':SimpleNamespace(UART=Port,FPIOA=Pins),
                                    'time':fake_time,'ujson':json}):
            exec((ROOT/'device/k230/arm/ready_pose.py').read_text(),ns)
        cfg,_,_=deploy.build_bundle()
        error=None
        with contextlib.redirect_stdout(io.StringIO()) as output:
            try:ns['run_ready_direct'](cfg['ready_control'])
            except RuntimeError as exc:error=exc
        return cfg,writes,closed,error,output.getvalue()

    def test_exact_final_goals_without_torque_writes_or_servo_reads(self):
        cfg,writes,closed,error,output=self.run_direct()
        self.assertIsNone(error);self.assertEqual(closed,[True]);self.assertEqual(len(writes),1)
        packet=writes[0]
        self.assertEqual(packet[:7],bytes([255,255,254,52,131,41,7]))
        self.assertEqual(sum(packet[2:])&255,255)
        for i in range(6):
            entry=packet[7+i*8:15+i*8];profile=cfg['ready_control']['profiles'][i]
            self.assertEqual(entry[0],i+1);self.assertEqual(entry[1],profile['acceleration_raw'])
            self.assertEqual(entry[2]|entry[3]<<8,cfg['ready_control']['goals'][i])
            self.assertEqual(entry[4:6],b'\x00\x00')
            self.assertEqual(entry[6]|entry[7]<<8,profile['speed_raw'])
        self.assertIn('"torque_writes": 0',output)
        self.assertIn('"servo_reads": 0',output)

    def test_short_goal_write_never_enables_torque_or_sends_recovery(self):
        _,writes,closed,error,_=self.run_direct(fail_at=1)
        self.assertIsInstance(error,RuntimeError);self.assertEqual(len(writes),1);self.assertEqual(closed,[True])


class DirectPickTests(unittest.TestCase):
    def run_stage(self,name,positions=(),short_write=False):
        cfg,files,_=deploy.build_bundle();writes=[];reads=[];closed=[];clock=[0];positions=iter(positions)
        class Pins:
            UART1_TXD=1;UART1_RXD=2
            def set_function(self,*args):pass
        class Port:
            UART1=1;EIGHTBITS=8;PARITY_NONE=None;STOPBITS_ONE=1
            def __init__(self,*args,**kwargs):pass
            def write(self,p):writes.append(p);return 0 if short_write else len(p)
            def deinit(self):closed.append(True)
        def read(uart,i,address,size):
            self.assertTrue(writes,'No pre-dispatch reads allowed')
            self.assertEqual((i,address,size),(6,56,2))
            reads.append(clock[0]);v=next(positions);return bytes((v&255,v>>8))
        def forbidden(*args,**kwargs):raise AssertionError('Legacy checks/hold must not run')
        def sleep(ms):clock[0]+=ms
        ns={}
        with patch.dict(sys.modules,{'machine':SimpleNamespace(UART=Port,FPIOA=Pins),
                  'time':SimpleNamespace(ticks_ms=lambda:clock[0],ticks_diff=lambda a,b:a-b,sleep_ms=sleep),'ujson':json}):
            exec(files['control.py'].decode(),ns)
        ns.update(read_checked=read,preflight=forbidden,initialize_hold=forbidden,send_sync=forbidden)
        control=dict(cfg['control']);control['goals']=[2048]*6
        stage={'name':name,'duration_s':.2,'stationary_s':.5,'tolerance_counts':1}
        error=None
        with contextlib.redirect_stdout(io.StringIO()):
            try:ns['run_pick_stage_direct'](control,stage)
            except RuntimeError as exc:error=exc
        return writes,reads,closed,clock[0],error,ns['sync_frame'](control['goals'],control['profiles'])

    def test_nonclose_stages_only_send_one_goal_packet_and_keep_dwell(self):
        for name in ('hover','descend','lift'):
            writes,reads,closed,elapsed,error,expected=self.run_stage(name)
            self.assertEqual(writes,[expected]);self.assertEqual(reads,[])
            self.assertEqual(closed,[True]);self.assertIsNone(error);self.assertGreaterEqual(elapsed,202)

    def test_close_waits_for_half_second_stillness_after_last_large_change(self):
        # Initial movement must not count as the stationary half second.
        values=[2200,2220,2240]+[2241]*11
        writes,reads,closed,elapsed,error,expected=self.run_stage('close',values)
        self.assertIsNone(error);self.assertEqual(writes,[expected]);self.assertEqual(closed,[True])
        self.assertGreaterEqual(elapsed,602);self.assertTrue(reads)

    def test_short_write_does_not_read_or_send_another_target(self):
        writes,reads,closed,elapsed,error,_=self.run_stage('hover',short_write=True)
        self.assertIsInstance(error,RuntimeError);self.assertEqual(len(writes),1)
        self.assertEqual(reads,[]);self.assertEqual(closed,[True])

    def test_plan_is_followed_by_hover_without_hold_action(self):
        app=VoiceDispatchTests().app();cfg,_,_=deploy.build_bundle();events=[]
        class Camera:
            def __init__(self,*args):pass
            def open(self):pass
            def snapshot(self):return SimpleNamespace(save=lambda *a,**kw:None)
            def close(self):pass
        plan={name:[2048]*6 for name in ('hover','descend','close','lift')}
        class Model:
            def __init__(self,*args):pass
            def plan(self,*args):events.append('plan');return plan
        with patch.dict(sys.modules,{'cv2':SimpleNamespace(imread=lambda p:object()),'control':SimpleNamespace()}), \
             patch.object(app,'CameraSession',Camera),patch.object(app,'Geometry',Model), \
             patch.object(app,'localize',return_value={'cube_center_Base_mm':[240,0,-60]}), \
             patch.object(app,'arm_snapshot',return_value=[{'position':2048} for i in range(6)]), \
             patch.object(app,'log'),patch.object(app,'run_pick_stage_direct',side_effect=lambda c,s:events.append(s['name'])):
            app.run_once(cfg,execute=True)
        self.assertEqual(events,['plan','hover','descend','close','lift'])


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
