"""Near parking carries the accepted calibration/window and no arm executor."""
import ast
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from host.project_paths import ROOT
from tools.deploy import k230_car_align as deploy


class NearBundleTests(unittest.TestCase):
    def test_paired_calibration_and_full_inset_window_are_preserved(self):
        config, files, remote, startup = deploy.build_bundle(near_parking=True)
        approach = json.loads((ROOT / 'configs/car/approach.json').read_text())
        self.assertEqual(approach['vision_config'], 'configs/vision/trials/model_alignment_20261030.json')
        self.assertEqual(approach['workspace_config'], 'configs/car/workspace_model_alignment_20261030.json')
        formal = json.loads((ROOT / approach['vision_config']).read_text())['localization']
        intrinsics = (ROOT / formal['intrinsics']).read_bytes()
        extrinsics = (ROOT / formal['extrinsics']).read_bytes()
        workspace = json.loads((ROOT / approach['workspace_config']).read_text())
        self.assertEqual(config['intrinsics'], json.loads(intrinsics))
        self.assertEqual(config['extrinsics'], json.loads(extrinsics))
        self.assertEqual(config['table_plane'], formal['table_plane_Base_mm'])
        self.assertEqual(config['workspace'], workspace)
        self.assertEqual(config['extrinsics']['intrinsics_sha256'], hashlib.sha256(intrinsics).hexdigest())
        line = config['workspace']['centerline']['local_Base_Y_line']
        self.assertEqual(line['intrinsics_sha256'], hashlib.sha256(intrinsics).hexdigest())
        self.assertEqual(line['extrinsics_sha256'], hashlib.sha256(extrinsics).hexdigest())
        self.assertEqual(config['workspace']['normalized_window']['inset_mm'], 10)
        self.assertEqual(config['projection']['T'], config['extrinsics']['T_Base_from_Camera_mm'])
        self.assertEqual(config['forward']['near_Base_X_mm'], approach['near_localization_start_Base_X_mm'])
        policy = json.loads((ROOT / 'configs/car/near_parking.json').read_text())
        self.assertEqual(config['near_parking'], policy)
        self.assertEqual(config['stopped_observation_ms'], policy['stopped_observation_ms'])
        self.assertIn('capture_Base_X_mm', config['near_parking'])
        self.assertNotIn('trigger_Base_X_mm', config['near_parking'])
        self.assertIn('continuous', config)
        self.assertFalse(config.get('automatic_approach', False))
        self.assertEqual(json.loads(files['config.json']), config)
        self.assertIn(remote, startup)

    def test_flat_bundle_has_required_pure_vision_dependencies_and_no_arm_runtime(self):
        _, files, _, _ = deploy.build_bundle(near_parking=True)
        car = ('near_vision', 'near_control', 'near_runtime', 'parking_window')
        geometry = ('pose', 'numeric', 'reflection_faces', 'side_faces',
                    'face_selection', 'table_pose', 'top_boundaries')
        for folder, names in (('car', car), ('pick', geometry)):
            for name in names:
                self.assertEqual(files[name + '.py'],
                    (ROOT / 'device/k230' / folder / (name + '.py')).read_bytes())
                ast.parse(files[name + '.py'])
        self.assertEqual(files['vision_runtime.py'], (ROOT / 'device/k230/vision/runtime.py').read_bytes())
        for name in ('physical_side', 'physical_side_session', 'side_range', 'evidence'):
            self.assertEqual(files[name+'.py'],
                (ROOT/'device/k230/vision'/(name+'.py')).read_bytes())
        config=json.loads(files['config.json'])
        for name, digest in config['runtime_version']['source_sha256'].items():
            self.assertEqual(digest, hashlib.sha256(files[name]).hexdigest())
        for name in ('app.py', 'control.py', 'kinematics.py', 'ready_pose.py',
                     'pick_stage.py', 'voice_commands.py', 'model.json'):
            self.assertNotIn(name, files)
        self.assertNotIn('model', json.loads(files['config.json']))

    def test_legacy_modes_do_not_enable_or_depend_on_near_parking(self):
        for options in ({}, {'approach_mode': True}, {'automatic': True}, {'continuous': True}):
            config, files, _, _ = deploy.build_bundle(**options)
            self.assertNotIn('near_parking', config)
            self.assertNotIn('near_runtime.py', files)
            self.assertNotIn('intrinsics', config)
            self.assertNotIn('workspace', config)
            self.assertEqual(config['stopped_observation_ms'], json.loads(
                (ROOT / 'configs/car/straight.json').read_text())['stopped_observation_ms'])
        with self.assertRaises(ValueError):
            deploy.build_bundle(automatic=True, near_parking=True)

    def test_near_cli_records_mode_and_creates_the_matching_log_directory(self):
        remote = '/sdcard/aic/car_align/releases/test_near'
        link = Mock()
        with tempfile.TemporaryDirectory() as folder:
            with patch.object(deploy, 'ROOT', Path(folder)), \
                    patch.object(deploy, 'build_bundle', return_value=({}, {'config.json': b'{}'}, remote, 'pass\n')) as build, \
                    patch.object(deploy, 'CameraLink', return_value=link), \
                    patch.object(deploy, 'stop_previous_script'), \
                    patch.object(deploy, 'write_file') as write, \
                    patch.object(deploy, 'mkdir') as mkdir, patch('builtins.print'):
                self.assertEqual(deploy.main(['--near-parking', '--install']), 0)
            build.assert_called_once_with(False, False, False, True)
            self.assertEqual([call.args[1] for call in mkdir.call_args_list],
                             [remote, '/sdcard/aic/car_near'])
            self.assertEqual(write.call_args.args[1], remote + '/config.json')
            record = json.loads(next((Path(folder) / 'data/deployments').glob('*/deployment.json')).read_text())
            self.assertEqual(record['mode'], 'near_visual_parking')
            self.assertEqual(record['log_path'], '/sdcard/aic/car_near/runs.jsonl')
            self.assertFalse(record['motion_commands_sent'])
            self.assertFalse(record['arm_commands_sent'])
            self.assertFalse(record['activated'])
            link.port.close.assert_called_once()

    def test_cli_selects_one_mode(self):
        for previous_mode in ('--approach', '--auto-approach', '--continuous-approach'):
            with patch('sys.stderr'), self.assertRaises(SystemExit) as error:
                deploy.main(['--near-parking', previous_mode])
            self.assertEqual(error.exception.code, 2)


if __name__ == '__main__':
    unittest.main()
