"""Real boot/voice/camera lifecycle with synthetic native timing, no hardware.

Queued and blocking microphone models are sensitivity cases, not claims about
the K230 queue depth, acoustic stability or measured boot performance.
"""
import builtins
import importlib.util
from contextlib import ExitStack
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import patch, mock_open

from host.project_paths import ROOT


def load(name, path):
    spec=importlib.util.spec_from_file_location(name,ROOT/path)
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class BootFixture:
    def __init__(self, queued=True, failed_frame=None, failed_prepare=False,warmup_returns=None):
        self.ms=0;self.trace=[];self.audio_active=False;self.reads=0;self.frames=[]
        self.logs=[];self.queued=queued;self.failed_frame=failed_frame
        self.failed_prepare=failed_prepare;self.preparations=0
        self.warmup_returns=warmup_returns
        self.time=SimpleNamespace(ticks_ms=lambda:self.ms,ticks_diff=lambda a,b:a-b,
                                  sleep_ms=self.advance)
        self.os=SimpleNamespace(EXITPOINT_ENABLE_SLEEP=1,exitpoint=lambda *a:None)
        parent=self
        class Sensor:
            RGB565=1
            def reset(self):parent.advance(200)
            def set_framesize(self,**kwargs):pass
            def set_pixformat(self,*args):pass
            def set_hmirror(self,*args):pass
            def set_vflip(self,*args):pass
            def run(self):parent.event('camera_run')
            def stop(self):parent.event('camera_stop')
            def snapshot(self):
                parent.frames.append(parent.ms);parent.advance(4)
                if len(parent.frames)==parent.failed_frame:raise RuntimeError('warmup snapshot failed')
                return object()
        class Engine:
            def __init__(self,*args):parent.advance(400);parent.event('engine_open')
            def add_keyword(self,keyword):pass
            def reset(self):parent.event('engine_reset')
            def push(self,pcm):
                parent.event('first_detection')
                raise KeyboardInterrupt()
            def deinit(self):parent.event('engine_close')
        class Stream:
            def __init__(self):self.next_packet=1
            def volume(self,**kwargs):pass
            def read(self):
                parent.reads+=1
                if parent.queued:
                    if parent.queued=='one':
                        produced=(parent.ms-parent.audio_started)//160
                        self.next_packet=max(self.next_packet,produced)
                    parent.ms=max(parent.ms,parent.audio_started+self.next_packet*160)
                    self.next_packet+=1
                else:parent.advance(160)
                parent.event('audio_read')
                if parent.warmup_returns is not None and parent.reads<=len(parent.warmup_returns):
                    return parent.warmup_returns[parent.reads-1]
                return bytes(2560*2)
            def stop_stream(self):parent.event('audio_stop')
            def close(self):parent.event('stream_close')
        class PyAudio:
            def open(self,**kwargs):
                parent.advance(100);parent.audio_active=True;parent.audio_started=parent.ms
                parent.event('audio_open');return Stream()
            def terminate(self):parent.audio_active=False;parent.event('audio_close')
        def media_init():
            if parent.audio_active:raise AssertionError('MediaManager.init while audio is active')
            parent.event('media_init')
        def media_close():
            if parent.audio_active:raise AssertionError('MediaManager.deinit while audio is active')
            parent.event('media_close')
        self.modules={
            'os':self.os,'time':self.time,'gc':SimpleNamespace(collect=lambda:None),
            'media.sensor':SimpleNamespace(Sensor=Sensor),
            'media.display':SimpleNamespace(Display=SimpleNamespace(VIRT=0,
                init=lambda *a,**kw:parent.event('display_init'),
                deinit=lambda:parent.event('display_close'))),
            'media.media':SimpleNamespace(MediaManager=SimpleNamespace(init=media_init,deinit=media_close)),
            'media.pyaudio':SimpleNamespace(PyAudio=PyAudio,paInt16=1),
            'wake_word':SimpleNamespace(engine=Engine,CHUNK=2560),
            'vision_runtime':SimpleNamespace(localize_precise=None),
            'kinematics':SimpleNamespace(Geometry=None),
            'control':SimpleNamespace(run_pick_stage_direct=None,run_ready_direct=None),
            'voice_commands':SimpleNamespace(registered_commands=lambda config:[dict(
                word='思达思达',action='pick',keyword_id='sida',enrollment_file='/keyword')],
                resolve_events=lambda *args:None)}

    def advance(self,ms):self.ms+=ms
    def event(self,name):self.trace.append((self.ms,name))
    def prepare_steps(self,config):
        self.preparations+=1;self.event('prepare_start')
        for group in range(4):
            self.advance(50)
            if self.failed_prepare:raise RuntimeError('pure preparation failed')
            yield {'stage':'group_'+str(group),'ms':50}
        self.advance(50);self.event('prepare_complete')
        yield {'prepared':{'prepare_ms':250}}
    def log(self,event,details=None,**kwargs):
        self.logs.append((self.ms,event,details));self.event(event)

    def run(self,audio_chunks=None):
        config={'arm':{},'voice':{'detection_sensitivity':'sensitive'},
            'car':{'tracking_task':{'enabled':True},'vision':{
                'camera':dict(width=1280,height=960,hmirror=False,vflip=False),
                'sampling':dict(warmup_frames=30,warmup_delay_ms=30)}}}
        if audio_chunks is not None:
            config['car']['tracking_task']['cold_start_audio_warmup_chunks']=audio_chunks
        with ExitStack() as stack:
            stack.enter_context(patch.dict(sys.modules,self.modules))
            self.camera=load('cold_camera','device/k230/vision/camera.py')
            stack.enter_context(patch.dict(sys.modules,{'camera':self.camera}))
            self.arm=load('cold_arm','device/k230/pick/app.py')
            self.arm.log=self.log
            stack.enter_context(patch.dict(sys.modules,{'app':self.arm}))
            self.board=load('cold_robot','device/k230/robot/app.py')
            self.board.load=lambda root:config
            self.board.emit=self.log
            self.board.prepare_tracking_steps=self.prepare_steps
            stack.enter_context(patch('builtins.open',mock_open(read_data='{"kw_id":"sida"}')))
            self.board.run_forever('/release')


class RobotColdStartTests(unittest.TestCase):
    def boot(self,**kwargs):
        f=BootFixture(**kwargs)
        with self.assertRaises(KeyboardInterrupt):f.run()
        return f

    def test_ready_waits_for_all_camera_frames_audio_chunks_and_pure_preparation(self):
        f=self.boot()
        names=[name for at,name in f.trace]
        ready_at,unused,details=next(row for row in f.logs if row[1]=='voice_ready')
        self.assertEqual(details['warmup_chunks'],32)
        self.assertEqual(details['discarded_audio_ms'],5120)
        self.assertEqual(f.reads,33)  # 32 discards, then the first detection read.
        self.assertEqual(len(f.frames),30)
        self.assertTrue(all(b-a>=30 for a,b in zip(f.frames,f.frames[1:])))
        self.assertLess(max(f.frames),ready_at)
        self.assertEqual(f.preparations,1)
        self.assertLess(names.index('media_init'),names.index('audio_open'))
        self.assertLess(names.index('audio_read'),names.index('prepare_complete'))
        self.assertLess(names.index('prepare_complete'),names.index('engine_reset'))
        self.assertLess(names.index('engine_reset'),names.index('voice_ready'))
        self.assertLess(names.index('audio_close'),names.index('media_close'))
        self.assertEqual(details['warmup_wall_ms'],details['warmup_read_ms']+details['startup_prepare_ms'])
        # With an explicitly queued fake stream, preparation replaces blocked
        # read time. This is not a measured native queue/performance guarantee.
        self.assertLess(details['warmup_read_ms'],details['discarded_audio_ms'])

    def test_blocking_audio_model_records_real_extra_time_instead_of_claiming_overlap(self):
        f=self.boot(queued=False)
        unused,unused,details=next(row for row in f.logs if row[1]=='voice_ready')
        self.assertEqual(details['warmup_read_ms'],5120)
        self.assertEqual(details['warmup_wall_ms'],5120+details['startup_prepare_ms'])

    def test_empty_or_short_warmup_reads_report_actual_bytes_without_changing_policy(self):
        f=self.boot(warmup_returns=[None,b'',bytes(1600)]+[b'']*29)
        unused,unused,details=next(row for row in f.logs if row[1]=='voice_ready')
        self.assertEqual(details['warmup_chunks'],32)
        self.assertEqual(details['configured_nominal_audio_ms'],5120)
        self.assertEqual(details['discarded_audio_bytes'],1600)
        self.assertEqual(details['discarded_audio_ms'],50)
        self.assertEqual(f.reads,33)

    def test_one_chunk_queue_can_overlap_small_camera_and_module_steps(self):
        f=self.boot(queued='one')
        unused,unused,details=next(row for row in f.logs if row[1]=='voice_ready')
        self.assertEqual(details['warmup_chunks'],32)
        self.assertEqual(len(f.frames),30)
        # Each synthetic frame+module group fits a160ms audio interval, even
        # with only one queued chunk. This does not assume an unlimited FIFO.
        self.assertEqual(details['warmup_wall_ms'],5120)
        self.assertLess(details['warmup_read_ms'],5120)

    def test_reduced_experiment_minimum_cannot_announce_ready_before_preparation(self):
        f=BootFixture()
        with self.assertRaises(KeyboardInterrupt):f.run(audio_chunks=8)
        unused,unused,details=next(row for row in f.logs if row[1]=='voice_ready')
        self.assertEqual(details['configured_warmup_chunks'],8)
        self.assertEqual(details['warmup_chunks'],30)
        self.assertEqual(len(f.frames),30)
        self.assertEqual(f.preparations,1)

    def test_legacy_voice_default_still_discards32_and_preloads_before_listening(self):
        f=BootFixture();imports=[];ordinary=builtins.__import__
        image_modules=('vision','physical_side','edge_profiles','separator_profiles',
            'profile_recovery','cube_pose','cube_corners','cube_observation',
            'ridge_profiles','cube_outline')
        def record(name,*args,**kwargs):
            if name in image_modules:imports.append((f.ms,name,f.reads))
            return ordinary(name,*args,**kwargs)
        with ExitStack() as stack:
            stack.enter_context(patch.dict(sys.modules,f.modules))
            stack.enter_context(patch.dict(sys.modules,{name:SimpleNamespace()
                                                       for name in image_modules}))
            stack.enter_context(patch.dict(sys.modules,{
                'camera':SimpleNamespace(CameraSession=None),'vision':SimpleNamespace(localize=None)}))
            arm=load('cold_standalone_voice','device/k230/pick/app.py')
            arm.log=f.log
            arm.load=lambda root:{'pick':{'cube_offset_toward_moving_finger_mm':0},
                                  'voice':{'detection_sensitivity':'sensitive'}}
            stack.enter_context(patch('builtins.__import__',side_effect=record))
            stack.enter_context(patch('builtins.open',mock_open(read_data='{"kw_id":"sida"}')))
            with self.assertRaises(KeyboardInterrupt):arm.run_forever('/release')
        ready=next(row for row in f.logs if row[1]=='voice_ready')
        self.assertEqual(ready[2]['warmup_chunks'],32)
        self.assertEqual(ready[2]['startup_prepare_ms'],0)
        self.assertEqual([name for at,name,read in imports],list(image_modules))
        self.assertEqual([read for at,name,read in imports],list(range(1,11)))
        self.assertTrue(all(at<ready[0] for at,name,read in imports))

    def test_warmup_failure_releases_audio_before_media_then_restores_voice(self):
        f=BootFixture(failed_frame=4)
        with self.assertRaises(KeyboardInterrupt):f.run()
        names=[name for at,name in f.trace]
        self.assertEqual(names.count('audio_open'),2)
        self.assertEqual(names.count('voice_ready'),1)
        self.assertLess(names.index('stream_close'),names.index('media_close'))
        self.assertLess(names.index('audio_close'),names.index('media_close'))
        self.assertLess(names.index('engine_close'),names.index('media_close'))
        self.assertLess(names.index('media_close'),names.index('voice_ready'))
        self.assertTrue(any(event=='camera_prepare_error' and 'snapshot failed' in details['error']
            for unused,event,details in f.logs))
        self.assertIsNone(f.camera.CameraSession._owner)

    def test_preparation_failure_never_announces_ready_and_releases_in_owner_order(self):
        f=BootFixture(failed_prepare=True)
        with self.assertRaisesRegex(RuntimeError,'pure preparation failed'):f.run()
        names=[name for at,name in f.trace]
        self.assertNotIn('voice_ready',names)
        self.assertLess(names.index('audio_close'),names.index('media_close'))
        self.assertIsNone(f.camera.CameraSession._owner)

    def test_importing_voice_app_does_not_import_the_precise_OpenCV_pipeline(self):
        f=BootFixture();imports=[];ordinary=builtins.__import__
        def record(name,*args,**kwargs):
            imports.append(name);return ordinary(name,*args,**kwargs)
        with patch.dict(sys.modules,f.modules),patch.dict(sys.modules,{
                'camera':SimpleNamespace(CameraSession=None)}),patch('builtins.__import__',side_effect=record):
            load('cold_voice_import','device/k230/pick/app.py')
        self.assertNotIn('vision',imports)
        self.assertNotIn('cv2',imports)


if __name__=='__main__':unittest.main()
