"""Board-only perception protocol, exact release binding and retained SD evidence."""
import ast
import binascii
import copy
import contextlib
from contextlib import contextmanager
import hashlib
import io
import json
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

from host.transport import k230_pick_perception as transport


def encoded(value):
    return (json.dumps(value, ensure_ascii=False, sort_keys=True)+'\n').encode()


def found():
    return {
        'status':'complete_face_candidate', 'cube_center_Base_mm':[245.,-28.,-43.],
        'cube_center_Camera_mm':[3.,12.,240.], 'face_quad_px':[[1,2],[3,2],[3,4],[1,4]],
        'poses':[{'face_reprojection_rms_px':.4}],
        'perception_timing_ms':{'jpeg_save':12,'jpeg_read':9,'visual_localization':340},
        'observation':{
            'schema':'aic.vision_observation.v1','mode':'precise','status':'localized',
            'coordinate_frame':'Base','units':'mm','reference_point':'cube_geometric_center',
            'xy_compensation_applied':False,'cube_center_Base_mm':[245.,-28.,-43.],
            'frame_id':74,'capture_timestamp_ms':105,'timestamp_ms':105,
            'clock_domain':'k230_ticks_ms','image_size_px':[1280,960],
            'quality':{'reprojection_rms_px':.4},
        },
    }


APP_SOURCE = b'''import json
import vision
def load(root):
 with open(root+'/config.json') as f:return json.load(f)
def log(event,details=None,persist=True):
 print('AIC_PICK '+json.dumps({'event':event,'details':details}))
def perceive(config):
 if config.get('test_error'):raise ValueError(config['test_error'])
 log('vision_evidence_saved',{'directory':'/sdcard/aic/vision/evidence/example',
     'manifest':'/sdcard/aic/vision/evidence/example/manifest.json',
     'archive_complete':True,'status':'localized'})
 result=config['test_found']
 result['implementation']=vision.IMPLEMENTATION
 return result
def run_once(*args,**kwargs):raise AssertionError('No motion entry allowed')
def run_from_observation(*args,**kwargs):raise AssertionError('No motion entry allowed')
def arm_snapshot(*args,**kwargs):raise AssertionError('No servo reads allowed')
'''


def bundle():
    config = {
        'vision':{'camera':{'width':1280,'height':960,'hmirror':False,'vflip':False},
            'localization':{'intrinsics':'configs/vision/runtime/camera_intrinsics.json',
                'extrinsics':'configs/vision/trials/selected_extrinsics.json',
                'table_pose_constraints_enabled':True,
                'table_plane_Base_mm':{'normal':[0,0,1],'offset_mm':-53}}},
        'intrinsics':{'camera_matrix':[[1200,0,640],[0,1200,480],[0,0,1]],
                      'distortion_coefficients':[0]*5,'image_width':1280,'image_height':960},
        'extrinsics':{'T_Base_from_Camera_mm':[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]},
        'voice':{'wake_word':'思达思达'},'test_found':found(),
    }
    files = {'app.py':APP_SOURCE, 'vision.py':b'IMPLEMENTATION="selected candidate"\n',
             'camera.py':b'"""No camera init at import in this protocol fixture."""\n',
             'control.py':b'def run_pick_stage_direct(*args):raise AssertionError("No motion")\n',
             'config.json':encoded(config)}
    manifest = {'schema':'aic.standalone_pick.v1','release_id':'selected-test-release',
                'files':{name:{'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest()}
                         for name,data in files.items()}}
    files['manifest.json'] = encoded(manifest)
    return config, files, manifest


class OnboardPerceptionTests(unittest.TestCase):
    def setUp(self):
        self.config, self.files, self.manifest = bundle()
        self.remote = '/sdcard/aic/pick/releases/'+self.manifest['release_id']
        self.result = {
            'schema':transport.RESULT_SCHEMA,'request_id':'this-request',
            'release_id':self.manifest['release_id'],'release_path':self.remote,
            'status':'localized','motion_performed':False,'verified_files':True,
            'config_sha256':self.manifest['files']['config.json']['sha256'],
            'manifest_sha256':hashlib.sha256(self.files['manifest.json']).hexdigest(),
            'found':found(),'verification_ms':21,'perception_ms':1200,'request_total_ms':1221,
            'evidence':{'directory':'/sdcard/aic/vision/evidence/run_1',
                        'manifest':'/sdcard/aic/vision/evidence/run_1/manifest.json',
                        'archive_complete':True,'status':'localized'},
        }

    @contextmanager
    def link(self, responses=None):
        events = []
        case = self
        class Link:
            def __init__(self, port):
                events.append(('open',port))
                self.port = SimpleNamespace(close=lambda:events.append('close'))
            def execute(self, source, timeout):
                self_check = events[-1]=='stopped'
                if not self_check:raise AssertionError('Previous listener must fully stop first')
                events.append(('execute',source,timeout))
                if responses is not None:
                    yield from responses()
                else:
                    yield b'AIC_PICK {"event":"vision_evidence_saved"}'
                    yield (transport.RESULT_PREFIX+json.dumps(case.result)).encode()
        with patch.object(transport,'CameraLink',Link),\
             patch.object(transport,'_stop_previous_script',side_effect=lambda l:events.append('stopped')),\
             patch.object(transport.uuid,'uuid4',return_value=SimpleNamespace(hex='this-request')),\
             patch('tools.deploy.k230_pick.build_bundle',return_value=(self.config,self.files,self.manifest)) as build:
            yield SimpleNamespace(events=events,build=build)

    def test_success_returns_exact_original_capture_and_json_geometry_without_image_export(self):
        with tempfile.TemporaryDirectory() as folder, self.link() as link:
            result = transport.perceive(folder,'configs/arm/pick_model_check.json','/dev/test-k230')
            output = Path(folder)
            self.assertEqual(result['cube_center_Base_mm'],[245.,-28.,-43.])
            self.assertEqual(result['observation'],found()['observation'])
            self.assertEqual(result['vision']['face_quad_px'],found()['face_quad_px'])
            self.assertEqual(result['evidence'],self.result['evidence'])
            self.assertEqual(result['release_id'],self.manifest['release_id'])
            self.assertFalse(result['xy_compensation_applied'])
            self.assertFalse(result['motion_performed'])
            self.assertEqual(result['perception_timing_ms'],found()['perception_timing_ms'])
            self.assertEqual(json.loads((output/'vision_record.json').read_text()),result)
            context=json.loads((output/'vision_context.json').read_text())
            self.assertEqual(context['intrinsics'],self.config['intrinsics'])
            self.assertEqual(context['extrinsics'],self.config['extrinsics'])
            self.assertEqual(context['table_plane'],self.config['vision']['localization']['table_plane_Base_mm'])
            self.assertEqual(context['version']['source_sha256']['app.py'],self.manifest['files']['app.py']['sha256'])
            diagnostic=json.loads((output/'onboard_perception.json').read_text())
            self.assertEqual(diagnostic['status'],'localized')
            self.assertIn('transport_timing_s',diagnostic)
            self.assertIn('vision_evidence_saved',(output/'camera_log.txt').read_text())
            self.assertEqual(sorted(p.name for p in output.iterdir()),
                             ['camera_log.txt','onboard_perception.json','vision_context.json','vision_record.json'])
        link.build.assert_called_once_with('configs/arm/pick_model_check.json')
        self.assertEqual(link.events[0],('open','/dev/test-k230'))
        self.assertEqual(link.events[-1],'close')

    def assert_failure_saved(self, responses=None):
        with tempfile.TemporaryDirectory() as folder, self.link(responses) as link:
            with self.assertRaises((RuntimeError,ValueError,TimeoutError)):
                transport.perceive(folder)
            output=Path(folder)
            self.assertFalse((output/'vision_record.json').exists())
            self.assertTrue((output/'vision_context.json').exists())
            diagnostic=json.loads((output/'onboard_perception.json').read_text())
            self.assertEqual(diagnostic['status'],'error')
            self.assertIn('error',diagnostic)
            self.assertTrue((output/'camera_log.txt').exists())
            self.assertEqual(link.events[-1],'close')
            return diagnostic

    def test_board_rejection_keeps_original_error_and_evidence_and_never_returns_old_xyz(self):
        self.result.update(status='error',error='No reliable physical face')
        self.result.pop('found')
        record=self.assert_failure_saved()
        self.assertEqual(record['board_result']['error'],'No reliable physical face')
        self.assertEqual(record['board_result']['evidence'],self.result['evidence'])

    def test_request_release_or_manifest_configuration_mismatch_is_rejected(self):
        original=copy.deepcopy(self.result)
        changes=({'request_id':'old-request'},{'release_id':'wrong-release'},
                 {'release_path':'/sdcard/aic/car_align/releases/other'},
                 {'schema':'old-schema'},{'verified_files':False},
                 {'config_sha256':'0'*64},{'manifest_sha256':'0'*64},
                 {'motion_performed':True})
        for change in changes:
            with self.subTest(change=change):
                self.result=copy.deepcopy(original);self.result.update(change)
                self.assert_failure_saved()

    def test_missing_duplicate_malformed_and_nonobject_terminal_results_fail(self):
        line=(transport.RESULT_PREFIX+json.dumps(self.result)).encode()
        for lines in ([],[line,line],[(transport.RESULT_PREFIX+'{broken').encode()],
                      [(transport.RESULT_PREFIX+'[]').encode()]):
            with self.subTest(lines=len(lines)):
                self.assert_failure_saved(lambda:iter(lines))

    def test_transport_failure_after_a_result_is_not_accepted_as_success(self):
        def interrupted():
            yield (transport.RESULT_PREFIX+json.dumps(self.result)).encode()
            raise TimeoutError('USB response interrupted')
        self.assert_failure_saved(interrupted)

    def test_failed_range_coarse_compensated_or_nonfinite_result_cannot_supply_a_target(self):
        original=copy.deepcopy(self.result)
        changes=[('status','range_candidate'),('cube_center_Base_mm',[float('nan'),0,0]),
                 ('cube_center_Base_mm',[True,0,0])]
        for key,value in changes:
            with self.subTest(key=key,value=value):
                self.result=copy.deepcopy(original);self.result['found'][key]=value
                self.assert_failure_saved()
        for key,value in (('mode','coarse'),('status','no_complete_face'),
                          ('xy_compensation_applied',True),('range_only',True),
                          ('odometry_predicted',True),('timestamp_ms',999),('clock_domain','host_clock')):
            with self.subTest(key=key,value=value):
                self.result=copy.deepcopy(original);self.result['found']['observation'][key]=value
                self.assert_failure_saved()
        self.result=copy.deepcopy(original)
        self.result['found']['observation']['quality']['reprojection_rms_px']=2.01
        self.assert_failure_saved()

    def test_existing_success_file_is_not_overwritten_or_reused_and_no_usb_is_opened(self):
        with tempfile.TemporaryDirectory() as folder, patch.object(transport,'CameraLink') as connect:
            saved=Path(folder)/'vision_record.json';saved.write_text('old result retained')
            with self.assertRaises(FileExistsError):transport.perceive(folder)
            self.assertEqual(saved.read_text(),'old result retained')
            connect.assert_not_called()

    def test_usb_open_failure_still_saves_context_and_diagnostics(self):
        with tempfile.TemporaryDirectory() as folder, self.link(),\
             patch.object(transport,'CameraLink',side_effect=RuntimeError('unplugged')):
            with self.assertRaisesRegex(RuntimeError,'unplugged'):transport.perceive(folder)
            output=Path(folder)
            self.assertTrue((output/'vision_context.json').exists())
            self.assertIn('unplugged',json.loads((output/'onboard_perception.json').read_text())['error'])
            self.assertFalse((output/'vision_record.json').exists())

    def test_deploy_import_stays_lazy_to_avoid_the_pc_entry_import_cycle(self):
        tree=ast.parse(Path(transport.__file__).read_text())
        top_imports=[node.module for node in tree.body if isinstance(node,ast.ImportFrom)]
        self.assertNotIn('tools.deploy.k230_pick',top_imports)


class GeneratedBoardPerceptionTests(unittest.TestCase):
    def execute_script(self, corrupt=None, source_error=None):
        config, files, manifest=bundle()
        if source_error:
            config['test_error']=source_error
            files['config.json']=encoded(config)
            manifest['files']['config.json']={'bytes':len(files['config.json']),
                'sha256':hashlib.sha256(files['config.json']).hexdigest()}
            files['manifest.json']=encoded(manifest)
        with tempfile.TemporaryDirectory() as folder:
            remote=Path(folder)/'selected';remote.mkdir()
            for name,data in files.items():(remote/name).write_bytes(data)
            if corrupt is not None:(remote/corrupt).write_bytes(b'corrupt')
            main=Path(folder)/'main.py';main.write_bytes(b'original robot startup')
            current=Path(folder)/'current.json';current.write_bytes(b'original pointer')
            original_jpeg=Path(folder)/'original.jpg';original_jpeg.write_bytes(b'original JPEG stays on SD')
            clock=[100]
            def ticks():clock[0]+=1;return clock[0]
            stale={}
            for name in ('app','vision','camera','control'):
                module=ModuleType(name);module.IMPLEMENTATION='wrong cached source';stale[name]=module
            imports=dict(stale,uhashlib=hashlib,
                         time=SimpleNamespace(ticks_ms=ticks,ticks_diff=lambda a,b:a-b))
            old_path=list(sys.path)
            try:
                with patch.dict(sys.modules,imports),contextlib.redirect_stdout(io.StringIO()) as stdout:
                    script=transport.build_script(str(remote),manifest,
                        hashlib.sha256(files['manifest.json']).hexdigest(),'generated-request')
                    exec(compile(script,'<generated board perception>','exec'),{})
                    output=stdout.getvalue()
                    if corrupt is None and source_error is None:
                        self.assertNotEqual(sys.modules['app'],stale['app'])
                        self.assertEqual(Path(sys.modules['app'].__file__).parent,remote)
                        self.assertIsNot(sys.modules['vision'],stale['vision'])
                        self.assertNotIn('camera',sys.modules)
                        self.assertNotIn('control',sys.modules)
            finally:sys.path[:]=old_path
            self.assertEqual(main.read_bytes(),b'original robot startup')
            self.assertEqual(current.read_bytes(),b'original pointer')
            self.assertEqual(original_jpeg.read_bytes(),b'original JPEG stays on SD')
            terminal=[line[len(transport.RESULT_PREFIX):] for line in output.splitlines()
                      if line.startswith(transport.RESULT_PREFIX)]
            self.assertEqual(len(terminal),1)
            self.assertEqual(output.splitlines()[-1],'EXPORT_DONE')
            return json.loads(terminal[0]),script

    def test_actual_generated_script_hashes_sources_and_config_then_uses_fresh_app_perceive(self):
        result,script=self.execute_script()
        self.assertEqual(result['status'],'localized')
        self.assertEqual(result['found']['implementation'],'selected candidate')
        self.assertTrue(result['verified_files'])
        self.assertEqual(result['evidence']['manifest'],
                         '/sdcard/aic/vision/evidence/example/manifest.json')
        calls=[node for node in ast.walk(ast.parse(script)) if isinstance(node,ast.Call)]
        app_calls=[node.func.attr for node in calls if isinstance(node.func,ast.Attribute)
                   and isinstance(node.func.value,ast.Name) and node.func.value.id=='app']
        self.assertEqual(sorted(app_calls),['load','perceive'])
        self.assertNotIn('b2a_base64',script)
        self.assertNotIn('write_file',script)
        self.assertNotIn('/sdcard/main.py',script)
        self.assertNotIn('/current.json',script)

    def test_corrupt_source_config_or_manifest_fails_before_perception(self):
        for name in ('app.py','vision.py','control.py','config.json','manifest.json'):
            with self.subTest(name=name):
                result,_=self.execute_script(corrupt=name)
                self.assertEqual(result['status'],'error')
                self.assertFalse(result['verified_files'])
                self.assertNotIn('found',result)
                self.assertIn('mismatch',result['error'])

    def test_perception_failure_emits_one_error_result_without_retry_or_old_center(self):
        result,_=self.execute_script(source_error='No face this frame')
        self.assertTrue(result['verified_files'])
        self.assertEqual(result['status'],'error')
        self.assertEqual(result['error'],'No face this frame')
        self.assertNotIn('found',result)

    def test_missing_installed_release_is_a_clear_error_and_does_not_install(self):
        _,files,manifest=bundle()
        with tempfile.TemporaryDirectory() as folder:
            missing=Path(folder)/'not_installed'
            clock=SimpleNamespace(ticks_ms=lambda:1,ticks_diff=lambda a,b:a-b)
            with patch.dict(sys.modules,{'uhashlib':hashlib,'time':clock}),\
                 contextlib.redirect_stdout(io.StringIO()) as stdout:
                exec(transport.build_script(str(missing),manifest,
                    hashlib.sha256(files['manifest.json']).hexdigest(),'missing-request'),{})
            result=json.loads(next(line[len(transport.RESULT_PREFIX):]
                for line in stdout.getvalue().splitlines() if line.startswith(transport.RESULT_PREFIX)))
            self.assertEqual(result['status'],'error')
            self.assertIn('not installed',result['error'])
            self.assertFalse(missing.exists())


if __name__=='__main__':unittest.main()
