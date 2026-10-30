"""Archive original fine JPEGs after media/UART release, including failed runs."""
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from device.k230.car import align_voice
from device.k230.vision import evidence
from host.project_paths import ROOT


class FineSessionEvidenceTests(unittest.TestCase):
    def session_type(self, options=None):
        # Load the real media adapter without requiring the host's car-only
        # Python environment to install OpenCV. Only the pure solve is replaced.
        source=ROOT/'device/k230/vision/physical_side_session.py'
        spec=importlib.util.spec_from_file_location('_car_evidence_media_session',source)
        module=importlib.util.module_from_spec(spec)
        def solve(bgr,frame,*args,**kwargs):
            if options is not None:
                options.append(kwargs)
            yield {'stage':'complete','observation':{
                'status':'localized','frame_id':frame['frame_id'],
                'capture_timestamp_ms':frame['capture_timestamp_ms']}}
        with patch.dict(sys.modules,{'physical_side':SimpleNamespace(localize_steps=solve)}):
            spec.loader.exec_module(module)
        return module.FineSession

    def test_only_moving_purpose_requests_distance_capability(self):
        options=[]
        session_type=self.session_type(options)
        image=SimpleNamespace(compressed=lambda **kw:
            SimpleNamespace(bytearray=lambda:bytearray(b'original-jpeg')))
        fake_cv=SimpleNamespace(IMREAD_COLOR=1,imdecode=lambda *a:'owned-bgr')
        fake_np=SimpleNamespace(uint8='uint8',frombuffer=lambda value,**kw:value)
        config={'intrinsics':{},'extrinsics':{},'table_plane':{}}
        with patch.dict(sys.modules,{'cv2':fake_cv,'numpy':fake_np}):
            for purpose in ('precise','car_moving_range'):
                session=session_type(config,lambda:1000,purpose=purpose)
                session.start(image,{'frame_id':purpose,'capture_timestamp_ms':321})
                session.advance()
        self.assertEqual([o['include_range'] for o in options],[False,True])

    def test_decoding_and_solving_keep_original_jpeg_until_one_shot_detach(self):
        data=bytearray(b'\xff\xd8original-owned-jpeg\xff\xd9')
        calls=[]
        class Image:
            def compressed(self,quality):
                calls.append(quality)
                return SimpleNamespace(bytearray=lambda:data)
        session=self.session_type()({'intrinsics':{},'extrinsics':{},'table_plane':{}},lambda:1000)
        frame={'frame_id':7,'capture_timestamp_ms':321,'timestamp_source':'software_snapshot_call'}
        session.start(Image(),frame,[10,20,30,40])
        fake_cv=SimpleNamespace(IMREAD_COLOR=1,imdecode=lambda *a:'owned-bgr')
        fake_np=SimpleNamespace(uint8='uint8',frombuffer=lambda value,**kw:value)
        with patch.dict(sys.modules,{'cv2':fake_cv,'numpy':fake_np}):
            session.advance()
        self.assertIsNotNone(session.jpeg)
        session.advance()
        self.assertEqual(session.latest['status'],'localized')
        item=session.take_evidence('moving_fine')
        self.assertIsInstance(item['jpeg'],bytes)
        self.assertEqual(item['jpeg'],bytes(data))
        detached_original=item['jpeg']
        data[:]=b'native buffer overwritten after camera close'
        self.assertEqual(item['jpeg'],detached_original)
        self.assertEqual(item['jpeg'],b'\xff\xd8original-owned-jpeg\xff\xd9')
        self.assertEqual(calls,[95])
        self.assertEqual(item['frame'],frame)
        self.assertEqual(item['observation']['capture_timestamp_ms'],321)
        self.assertIsNone(session.jpeg)
        self.assertIsNone(session.take_evidence('duplicate'))

    def test_interrupted_solve_retains_original_capture_metadata_and_failure_phase(self):
        data=bytearray(b'owned-before-interruption')
        image=SimpleNamespace(compressed=lambda **kw:SimpleNamespace(bytearray=lambda:data))
        session=self.session_type()({},lambda:10)
        self.assertIsNone(session.take_evidence('not_captured'))
        session.start(image,{'frame_id':9,'capture_timestamp_ms':4})
        item=session.take_evidence('moving_fine')
        self.assertEqual(item['jpeg'],bytes(data))
        self.assertIsInstance(item['jpeg'],bytes)
        self.assertEqual(item['observation'],{'status':'processing_interrupted','phase':'decode'})
        self.assertEqual(item['frame']['capture_timestamp_ms'],4)


class CarNearArchiveTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        self.config={
            'runtime_version':{'release_id':'candidate-release','algorithm_sha256':'source-digest'},
            'projection':{'K':[],'D':[],'T':[],'plane':{}},
            'alignment':{},'tracking':{},'voice':{},'vision':{},'near_parking':{'capture_Base_X_mm':500}}
        self.originals=[b'\xff\xd8original moving\x00\xff\xd9',b'\xff\xd8original stopped\xff\xd9']
        self.events=[]
        self.archives=[]

    def frames(self,count=2):
        return [{'label':label,'jpeg':bytearray(data),
            'frame':{'frame_id':index+1,'capture_timestamp_ms':123+index},
            'observation':{'status':'localized' if index else 'no_complete_face'}}
            for index,(label,data) in enumerate(zip(
                ['moving_fine','stopped_verify'],self.originals))][:count]

    def evidence_class(self):
        owner=self
        class LocalEvidence(evidence.EvidenceRun):
            def __init__(self,root,task,config,version=None):
                owner.assertEqual(root,'/sdcard/aic/vision/evidence')
                owner.events.append('archive')
                super().__init__(str(owner.root/'evidence'),task,config,version)
                owner.archives.append(self)
        return LocalEvidence

    def test_archive_writes_owned_bytes_metadata_config_and_runtime_version(self):
        frames=self.frames()
        with patch.dict(sys.modules,{'evidence':SimpleNamespace(EvidenceRun=self.evidence_class())}):
            result=align_voice.archive_fine_frames(self.config,frames,'precise_unavailable','no complete face')
        self.assertEqual(frames,[])
        self.assertEqual(result['frame_count'],2)
        directory=Path(result['directory'])
        manifest=json.loads((directory/'manifest.json').read_text())
        self.assertEqual(manifest['version'],self.config['runtime_version'])
        self.assertEqual(manifest['status'],'precise_unavailable')
        self.assertEqual(manifest['error'],'no complete face')
        self.assertEqual(json.loads((directory/'config.json').read_text()),self.config)
        for entry,data in zip(manifest['frames'],self.originals):
            self.assertEqual((directory/entry['image_file']).read_bytes(),data)
            self.assertEqual(entry['image_sha256'],hashlib.sha256(data).hexdigest())
            self.assertEqual(entry['image_bytes'],len(data))
        json.dumps(result)

    def test_archive_write_failure_is_reported_and_ram_is_cleared(self):
        frames=self.frames()
        archive_type=self.evidence_class()
        with patch.dict(sys.modules,{'evidence':SimpleNamespace(EvidenceRun=archive_type)}), \
                patch.object(archive_type,'save_jpeg',side_effect=OSError(28,'disk full')), \
                patch('builtins.print') as printed:
            result=align_voice.archive_fine_frames(self.config,frames,'parked')
        self.assertEqual(frames,[])
        self.assertIn('disk full',result['archive_error'])
        printed.assert_called_once()
        self.assertIn('CAR_NEAR archive_error',printed.call_args[0][0])
        self.assertFalse(self.archives[0].manifest['archive_complete'])

    def run_voice_cycle(self,runtime_error=None,archive_failure=False):
        owner=self
        class EndCycle(Exception):
            pass
        warmups=[]
        def wake(voice,warmup):
            warmups.append(warmup)
            if len(warmups)>1:
                raise EndCycle()
        class Camera:
            def __init__(self,config):
                pass
            def open(self):
                owner.events.append('camera_open')
            def capture(self):
                raise AssertionError('Capture belongs to the runtime stub')
            def close(self):
                owner.events.append('camera_close')
        class Link:
            def close(self):
                owner.events.append('uart_close')
        def run_parking(*args,**kwargs):
            owner.events.append('runtime')
            kwargs['evidence'].extend(owner.frames(1 if runtime_error else 2))
            if runtime_error:
                raise RuntimeError(runtime_error)
            return {'status':'parked','automatic_arm_action':False}
        def opened(path,mode='r'):
            if mode=='r':
                owner.assertEqual(path,'/test/config.json')
                return io.StringIO(json.dumps(owner.config))
            owner.assertEqual((path,mode),('/sdcard/aic/car_near/runs.jsonl','a'))
            owner.events.append('json_log')
            return open(owner.root/'runs.jsonl',mode)
        archive_type=self.evidence_class()
        original_init=archive_type.__init__
        def after_closed(instance,*args,**kwargs):
            owner.assertIn('camera_close',owner.events)
            owner.assertIn('uart_close',owner.events)
            owner.assertLess(owner.events.index('camera_close'),owner.events.index('uart_close'))
            if archive_failure:
                raise OSError(28,'archive unavailable')
            original_init(instance,*args,**kwargs)
        with patch.dict(sys.modules,{
                'straight_voice':SimpleNamespace(wait_for_wake=wake),
                'visual_align':SimpleNamespace(align_once=None,approach_until_near=None),
                'ground_projection':SimpleNamespace(GroundProjection=lambda *args:None),
                'motor_link':SimpleNamespace(MotorLink=Link),
                'camera':SimpleNamespace(CameraSession=Camera),
                'near_runtime':SimpleNamespace(run_parking=run_parking),
                'vision_runtime':SimpleNamespace(track_coarse=None),
                'blue_tracker':SimpleNamespace(BlueTracker=lambda *args:None),
                'evidence':SimpleNamespace(EvidenceRun=archive_type)}), \
                patch.object(archive_type,'__init__',after_closed), \
                patch.object(align_voice,'open',side_effect=opened,create=True), \
                patch.object(align_voice,'time',SimpleNamespace(ticks_ms=lambda:1000,sleep_ms=lambda ms:None)), \
                patch.object(align_voice.os,'exitpoint',create=True), \
                patch('builtins.print'):
            with self.assertRaisesRegex(RuntimeError,runtime_error) if runtime_error else self.assertRaises(EndCycle):
                align_voice.run_forever('/test')
        return warmups

    def test_voice_archives_only_after_camera_and_uart_close_then_writes_json(self):
        self.assertEqual(self.run_voice_cycle(),[32,0])
        self.assertEqual(self.events,['camera_open','runtime','camera_close','uart_close','archive','json_log'])
        result=json.loads((self.root/'runs.jsonl').read_text())
        self.assertEqual(result['status'],'parked')
        self.assertEqual(result['evidence']['frame_count'],2)
        self.assertNotIn('original moving',json.dumps(result))

    def test_voice_runtime_exception_still_archives_after_both_resources_close(self):
        self.run_voice_cycle(runtime_error='fine interrupted')
        self.assertEqual(self.events,['camera_open','runtime','camera_close','uart_close','archive'])
        manifest=json.loads(Path(self.archives[0].manifest_path).read_text())
        self.assertEqual(manifest['status'],'runtime_error')
        self.assertEqual(manifest['error'],'fine interrupted')
        self.assertEqual(len(manifest['frames']),1)
        self.assertEqual((Path(self.archives[0].directory)/'moving_fine.jpg').read_bytes(),self.originals[0])
        self.assertFalse((self.root/'runs.jsonl').exists())

    def test_voice_archive_error_does_not_discard_motion_result_or_listener_restart(self):
        self.assertEqual(self.run_voice_cycle(archive_failure=True),[32,0])
        result=json.loads((self.root/'runs.jsonl').read_text())
        self.assertEqual(result['status'],'parked')
        self.assertIn('archive unavailable',result['evidence']['archive_error'])
        self.assertEqual(self.events,['camera_open','runtime','camera_close','uart_close','json_log'])


if __name__=='__main__':
    unittest.main()
