"""Save inference bytes and rejected observations before dispatch; no hardware."""
import importlib.util
import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from host.project_paths import ROOT
from device.k230.vision import evidence


def load_app():
    name='test_pick_evidence_app'
    spec=importlib.util.spec_from_file_location(name,ROOT/'device/k230/pick/app.py')
    module=importlib.util.module_from_spec(spec)
    imports={'camera':SimpleNamespace(CameraSession=None),'vision':SimpleNamespace(localize=None),
             'vision_runtime':SimpleNamespace(localize_precise=None),'kinematics':SimpleNamespace(Geometry=None),
             'control':SimpleNamespace(run_pick_stage_direct=None,run_ready_direct=None),
             'voice_commands':SimpleNamespace(registered_commands=None,resolve_events=None)}
    with patch.dict(sys.modules,imports):spec.loader.exec_module(module)
    return module


class PickEvidenceTests(unittest.TestCase):
    def exercise(self,folder,status,archive_error=False):
        app=load_app();calls=[];saved_paths=[];raw=b'original JPEG95 bytes';created=[]
        class Camera:
            def __init__(self,*args):pass
            def open(self):calls.append('open')
            def capture(self):
                class Image:
                    def save(self,path,quality):
                        self_check=quality==95
                        if not self_check:raise AssertionError(quality)
                        saved_paths.append(path);Path(path).write_bytes(raw)
                return Image(),{'frame_id':7,'capture_timestamp_ms':None,'clock_domain':None,'image_size_px':[1280,960]}
            def close(self):calls.append('close')
        def create(root,task,config,version):
            if archive_error:raise OSError('archive unavailable')
            run=evidence.EvidenceRun(folder,task,config,version);created.append(run);return run
        result={'status':status,'reason':'boundary unsupported' if status!='localized' else None,
                'payload':{'cube_center_Base_mm':[250,5,-43]}}
        def localize(im,*args,**kwargs):
            self.assertEqual(calls,['open','close'])
            calls.append('detect')
            return result
        config={'vision':{'localization':{'table_pose_constraints_enabled':False}},
                'intrinsics':{'K':'snapshot'},'extrinsics':{'T':'snapshot'},
                '_evidence_version':{'release_id':'test'}}
        with patch.dict(sys.modules,{'evidence':SimpleNamespace(EvidenceRun=create),
            'cv2':SimpleNamespace(imread=lambda p:Path(p).read_bytes())}),\
            patch.object(app,'CameraSession',Camera),patch.object(app,'localize_precise',side_effect=localize),\
            patch.object(app.time,'ticks_ms',lambda:1,create=True),\
            patch.object(app.time,'ticks_diff',lambda a,b:a-b,create=True),patch.object(app,'log') as log:
            if status=='localized':found=app.perceive(config)
            else:
                with self.assertRaisesRegex(ValueError,'boundary unsupported'):app.perceive(config)
                found=None
        return created,saved_paths,raw,found,log

    def test_success_archives_exact_inference_image_and_original_metadata(self):
        with tempfile.TemporaryDirectory() as folder:
            runs,paths,raw,found,log=self.exercise(folder,'localized')
            self.assertEqual(len(paths),1)
            run=runs[0];manifest=json.loads(Path(run.manifest_path).read_text())
            self.assertEqual(manifest['status'],'localized')
            self.assertEqual(Path(paths[0]).read_bytes(),raw)
            self.assertEqual(manifest['frames'][0]['observation']['status'],'localized')
            self.assertIsNone(manifest['frames'][0]['frame']['capture_timestamp_ms'])
            self.assertEqual(found['cube_center_Base_mm'],[250,5,-43])
            self.assertEqual(manifest['version'],{'release_id':'test'})
            self.assertEqual(json.loads((Path(run.directory)/'config.json').read_text())['intrinsics'],{'K':'snapshot'})
            log.assert_any_call('vision_evidence_saved',{'directory':run.directory,
                'manifest':run.manifest_path,'status':'localized','archive_complete':True})

    def test_rejected_localization_retains_image_and_reason(self):
        with tempfile.TemporaryDirectory() as folder:
            runs,paths,raw,_,_=self.exercise(folder,'no_complete_face')
            manifest=json.loads(Path(runs[0].manifest_path).read_text())
            self.assertEqual(manifest['status'],'perception_error')
            self.assertTrue(manifest['archive_complete'])
            self.assertIn('boundary unsupported',manifest['error'])
            self.assertEqual(manifest['frames'][0]['observation']['status'],'no_complete_face')
            self.assertEqual(Path(paths[0]).read_bytes(),raw)

    def test_archive_full_error_is_reported_without_changing_detection(self):
        app=load_app();result={'cube_center_Base_mm':[1,2,3]}
        with patch.dict(sys.modules,{'evidence':SimpleNamespace(EvidenceRun=lambda *a,**k:(_ for _ in ()).throw(OSError('full')))}),\
             patch.object(app,'_perceive',return_value=result) as detect,patch.object(app,'log') as log:
            found=app.perceive({'vision':{},'intrinsics':{},'extrinsics':{}})
        self.assertIs(found,result);detect.assert_called_once_with({'vision':{},'intrinsics':{},'extrinsics':{}},None)
        log.assert_any_call('vision_archive_error','full')

    def test_archive_finalizes_before_any_motion(self):
        app=load_app();events=[]
        class Run:
            directory='/evidence';manifest_path='/evidence/manifest.json'
            def finish(self,*a):events.append('archive_finished');return {'status':'localized','archive_complete':True}
        config={'vision':{'localization':{'table_plane_Base_mm':{'normal':[0,0,1],'offset_mm':0}}},
                'intrinsics':{},'extrinsics':{},'control':{},'model':{},
                'pick':{'stage_durations_s':{'close_minimum_stationary_s':.5},
                        'gripper_stationary_tolerance_counts':1,'standalone_stage_policy':{}}}
        class Geometry:
            def __init__(self,*a):pass
            def plan(self,*a):return {s:[] for s in ('hover','descend','close','lift')}
        with patch.dict(sys.modules,{'evidence':SimpleNamespace(EvidenceRun=lambda *a,**k:Run()),'control':SimpleNamespace()}),\
             patch.object(app,'_perceive',return_value={'cube_center_Base_mm':[250,0,-40]}),\
             patch.object(app,'arm_snapshot',side_effect=lambda c:events.append('read_arm') or [{'position':2048}]*6),\
             patch.object(app,'Geometry',Geometry),patch.object(app,'run_pick_stage_direct',side_effect=lambda c,s:events.append(s['name'])),\
             patch.object(app,'log'):
            app.run_once(config,execute=True)
        self.assertEqual(events,['archive_finished','read_arm','hover','descend','close','lift'])


class PcVisionContextTests(unittest.TestCase):
    def test_calibration_and_code_snapshot_written_before_detection_failure(self):
        spec=importlib.util.spec_from_file_location('test_pc_evidence_context',ROOT/'host/vision/localize_pick_cube.py')
        module=importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules,{'cv2':SimpleNamespace(imread=lambda p:None)}):
            spec.loader.exec_module(module)
            with tempfile.TemporaryDirectory() as folder:
                with self.assertRaisesRegex(RuntimeError,'Expected a completed'):
                    module.localize(Path(folder)/'bad.jpg',folder,
                                    ROOT/'configs/vision/trials/fixed_tip_top_20261022.json')
                context=json.loads((Path(folder)/'vision_context.json').read_text())
        self.assertEqual(context['schema'],'aic.vision_context.v1')
        self.assertIn('camera_matrix',context['intrinsics'])
        self.assertIn('T_Base_from_Camera_mm',context['extrinsics'])
        self.assertEqual(context['table_plane'],context['vision']['localization']['table_plane_Base_mm'])
        self.assertIn('device/k230/pick/vision.py',context['version']['source_sha256'])
        self.assertEqual(context['calibration_sources']['intrinsics_sha256'],context['extrinsics']['intrinsics_sha256'])


class PcFailureCollectionTests(unittest.TestCase):
    def test_rejected_pc_image_automatically_enters_dataset_without_motion(self):
        from host.apps import pick_validation_pc as pc
        pick,config=pc.parameters()
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);image=root/'original.jpg'
            image.write_bytes(b'original offline JPEG')
            def reject(command,**kwargs):
                out=Path(command[command.index('--output')+1])
                self.assertEqual((out/'frame.jpg').read_bytes(),image.read_bytes())
                self.assertEqual(command[command.index('--config')+1],str(root/pick['vision_config']))
                (out/'vision_candidates.json').write_text('{"status":"no_complete_face","reason":"edge"}')
                raise pc.subprocess.CalledProcessError(2,command,stderr='edge rejected')
            with patch.object(pc,'ROOT',root),patch.object(pc,'parameters',return_value=(pick,config)), \
                 patch.object(pc,'perceive_onboard',side_effect=AssertionError('offline replay must not open USB')), \
                 patch.object(pc.subprocess,'run',side_effect=reject), \
                 patch.object(pc,'run_action') as arm,patch.object(pc.signal,'signal'), \
                 patch('host.vision.hard_dataset.add_case',return_value='archived-case') as collect, \
                 contextlib.redirect_stdout(io.StringIO()),contextlib.redirect_stderr(io.StringIO()):
                result=pc.main(['--image',str(image)])
            run=next(root.glob('data/logs/*'))
            record=json.loads((run/'execution.json').read_text())
            self.assertEqual((run/'frame.jpg').read_bytes(),image.read_bytes())
            self.assertEqual(collect.call_args.args[0],run)
            self.assertEqual(record['vision_config'],pick['vision_config'])
            self.assertEqual(record['perception_backend'],'pc_offline')
        self.assertEqual(result,1);arm.assert_not_called()
        self.assertEqual(record['hard_case_id'],'archived-case')
        self.assertFalse(record['motion_performed'])
        self.assertEqual(collect.call_args.args[1],'recognition_failure')


if __name__=='__main__':unittest.main()
