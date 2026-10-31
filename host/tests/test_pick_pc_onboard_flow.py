"""Live PC pick uses native perception; offline replay never opens hardware."""
import contextlib
import copy
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from host.apps import pick_validation_pc as app
from host.project_paths import ROOT

PROFILE = ROOT/'configs/arm/pick_model_check.json'


class OnboardPcFlowTests(unittest.TestCase):
    def setUp(self):
        self.pick,self.config = app.parameters(PROFILE)
        self.vision = {'status':'localized','cube_center_Base_mm':[245.,-28.,-51.],
            'cube_center_Camera_mm':[1.,2.,200.],
            'xy_compensation_applied':False,'motion_performed':False,
            'release_id':'this-release','release_path':'/sdcard/aic/pick/releases/this-release',
            'request_id':'this-request','perception_timing_ms':{'capture':100,'visual_localization':300},
            'evidence':{'directory':'/sdcard/this-evidence','manifest':'/sdcard/this-evidence/manifest.json',
                'archive_complete':True,'status':'localized'},
            'observation':{'frame_id':12,'capture_timestamp_ms':900,
                'clock_domain':'k230_ticks_ms','status':'localized'}}

    def exercise(self,reply=None,offline=False,detect_only=True,side_effect=None):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            image = root/'original.jpg'
            image.write_bytes(b'offline-original-jpeg')
            plan = {name:[2048]*6 for name in ('hover','descend','close','lift')}
            plan['table_z_Base_mm'] = -60
            def replay(command,**kwargs):
                self.assertTrue(offline)
                self.assertTrue(command[0].endswith('/scripts/vision'))
                self.assertTrue(command[1].endswith('/host/vision/localize_pick_cube.py'))
                out = Path(command[command.index('--output')+1])
                self.assertEqual((out/'frame.jpg').read_bytes(),image.read_bytes())
                (out/'vision_record.json').write_text(json.dumps(self.vision))
            def onboard(output,pick_config=None,port=None):
                self.assertFalse(offline)
                self.assertEqual(pick_config,PROFILE)
                self.assertEqual(port,'/dev/test-k230')
                if side_effect:
                    return side_effect(output)
                return copy.deepcopy(self.vision) if reply is None else reply
            with patch.object(app,'ROOT',root), \
                    patch.object(app,'parameters',return_value=(self.pick,self.config)), \
                    patch.object(app,'perceive_onboard',side_effect=onboard) as perceive, \
                    patch.object(app.subprocess,'run',side_effect=replay) as subprocess_run, \
                    patch.object(app,'run_action',side_effect=AssertionError('No arm access in this test')) as action, \
                    patch.object(app,'make_plan',return_value=(SimpleNamespace(report=lambda c:{}),plan)) as planning, \
                    patch.object(app.signal,'signal'), \
                    contextlib.redirect_stdout(io.StringIO()),contextlib.redirect_stderr(io.StringIO()):
                arguments = ['--pick-config',str(PROFILE),'--camera-port','/dev/test-k230']
                if offline:
                    arguments += ['--image',str(image),'--start-counts','2100','2101','2102','2103','2104','2105']
                if detect_only:
                    arguments += ['--detect-only']
                code = app.main(arguments)
            folder = next(root.glob('data/logs/*'))
            record = json.loads((folder/'execution.json').read_text())
            return SimpleNamespace(code=code,record=record,perceive_calls=perceive.call_count,
                subprocess_calls=subprocess_run.call_count,arm_calls=action.call_count,
                plan_args=planning.call_args,files=[p.name for p in folder.iterdir()])

    def test_live_detect_only_keeps_board_result_and_metadata_without_export_or_pc_solve(self):
        result = self.exercise()
        self.assertEqual(result.code,0)
        self.assertEqual((result.perceive_calls,result.subprocess_calls,result.arm_calls),(1,0,0))
        self.assertNotIn('frame.jpg',result.files)
        self.assertEqual(result.record['perception_backend'],'k230')
        self.assertEqual(result.record['vision'],self.vision)
        for key in ('perception_timing_ms','evidence','release_id','release_path','request_id'):
            self.assertEqual(result.record[key],self.vision[key])
        self.assertGreaterEqual(result.record['timing_s']['onboard_perception'],0.)
        self.assertEqual(result.record['timing_s']['vision'],.3)
        self.assertFalse(result.record['torque_enable_attempted'])

    def test_failed_or_incomplete_native_result_cannot_reach_arm_or_planner(self):
        for update in ({'status':'no_complete_face'}, {'cube_center_Base_mm':[]},
                       {'cube_center_Base_mm':[1.,float('nan'),3.]},
                       {'cube_center_Base_mm':[1.,2.]}, {'cube_center_Base_mm':[1.,True,3.]},
                       {'xy_compensation_applied':True}):
            with self.subTest(update=update):
                value = dict(self.vision,**update)
                result = self.exercise(reply=value,detect_only=False)
                self.assertEqual(result.code,1)
                self.assertEqual(result.arm_calls,0)
                self.assertIsNone(result.plan_args)
                self.assertFalse(result.record['motion_performed'])
                self.assertFalse(result.record['torque_enable_attempted'])

    def test_native_exception_does_not_reuse_stale_disk_result(self):
        def fail(folder):
            (folder/'vision_record.json').write_text(json.dumps(self.vision))
            (folder/'onboard_perception.json').write_text(json.dumps({
                'request_id':'failed-current-request','status':'no_complete_face',
                'evidence':{'directory':'/sdcard/failed-this-request','archive_complete':True}}))
            raise RuntimeError('No result for the current request')
        result = self.exercise(side_effect=fail,detect_only=False)
        self.assertEqual(result.code,1)
        self.assertEqual(result.arm_calls,0)
        self.assertIsNone(result.plan_args)
        self.assertNotIn('vision',result.record)
        self.assertEqual(result.record['onboard_perception']['request_id'],'failed-current-request')
        self.assertIn('No result for the current request',result.record['error'])

    def test_offline_detect_replays_the_supplied_image_without_onboard_or_servo_calls(self):
        result = self.exercise(offline=True)
        self.assertEqual(result.code,0)
        self.assertEqual((result.perceive_calls,result.subprocess_calls,result.arm_calls),(0,1,0))
        self.assertIn('frame.jpg',result.files)
        self.assertEqual(result.record['perception_backend'],'pc_offline')
        self.assertNotIn('onboard_perception',result.record['timing_s'])

    def test_offline_plan_uses_only_explicit_start_counts_without_hardware(self):
        result = self.exercise(offline=True,detect_only=False)
        self.assertEqual(result.code,0)
        self.assertEqual(result.arm_calls,0)
        self.assertEqual(result.perceive_calls,0)
        self.assertEqual(result.record['status'],'plan_only_no_motion')
        self.assertEqual(result.plan_args[0][1],[2100,2101,2102,2103,2104,2105])

    def test_offline_image_execute_remains_forbidden_before_setup(self):
        with patch.object(app,'parameters',side_effect=AssertionError('must not configure hardware')):
            with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
                app.main(['--image','saved.jpg','--execute'])


if __name__=='__main__':
    unittest.main()
