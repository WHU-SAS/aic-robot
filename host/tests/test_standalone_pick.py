"""Standalone arithmetic, stage orchestration and deployment; no hardware access."""
import ast
import contextlib
import importlib.util
import io
import math
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
            imports=[]
            for n in ast.walk(tree):
                if isinstance(n,ast.Import):imports.extend(a.name for a in n.names)
                if isinstance(n,ast.ImportFrom):imports.append(n.module or '')
            self.assertFalse(any(i.startswith(('host','scipy','serial','pathlib')) for i in imports),name)
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
        mocks={'camera':SimpleNamespace(CameraSession=None),'vision':SimpleNamespace(localize=None),
               'control':SimpleNamespace(run_pick_action=None)}
        with patch.dict(sys.modules,mocks):spec.loader.exec_module(module)
        return module
    def test_startup_waits_for_wake_before_one_pick(self):
        app=self.app();calls=[]
        def wait(cfg):
            calls.append('wait')
            if len(calls)>1:raise KeyboardInterrupt()
        with patch.object(app,'load',return_value={'pick':{'cube_offset_toward_moving_finger_mm':-45},'voice':{}}),\
             patch.object(app,'log'),patch.object(app,'wait_for_wake',side_effect=wait),\
             patch.object(app,'run_once',side_effect=lambda cfg,execute:calls.append(('pick',execute))):
            with self.assertRaises(KeyboardInterrupt):app.run_forever('/fake')
        self.assertEqual(calls,['wait',('pick',True),'wait'])
    def test_failure_does_not_automatically_retry_pick(self):
        app=self.app();calls=[]
        def wait(cfg):
            calls.append('wait')
            if len(calls)>1:raise KeyboardInterrupt()
        def pick(cfg,execute):calls.append('failed_pick');raise ValueError('No face')
        with patch.object(app,'load',return_value={'pick':{'cube_offset_toward_moving_finger_mm':-45},'voice':{}}),\
             patch.object(app,'log'),patch.object(app,'wait_for_wake',side_effect=wait),\
             patch.object(app,'run_once',side_effect=pick),patch.object(sys,'print_exception',lambda e:None,create=True):
            with self.assertRaises(KeyboardInterrupt):app.run_forever('/fake')
        self.assertEqual(calls,['wait','failed_pick','wait'])
    def test_audio_resources_close_before_wake_returns(self):
        app=self.app();calls=[]
        class Engine:
            def add_keyword(self,r):pass
            def reset(self):pass
            def push(self,pcm):return [{'label':'sida'}]
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
            file=Path(folder)/'keyword.json';file.write_text('{}')
            with patch.dict(sys.modules,{'wake_word':SimpleNamespace(engine=lambda s:Engine(),CHUNK=2560),
                                         'media':SimpleNamespace(),'media.pyaudio':SimpleNamespace(PyAudio=Audio,paInt16=0)}),\
                 patch.object(app.os,'exitpoint',lambda:None,create=True),patch.object(app,'log'):
                event=app.wait_for_wake({'enrollment_file':str(file),'detection_sensitivity':'sensitive','wake_word':'思达思达'})
        self.assertEqual(event,{'label':'sida'})
        self.assertEqual(calls,['stream_stopped','stream_closed','audio_closed','engine_closed'])


class DeploymentTests(unittest.TestCase):
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
