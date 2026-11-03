"""Combined release preserves both baselines and activates only on next boot."""
import ast
import base64
import copy
import io
import json
from contextlib import redirect_stdout
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from host.project_paths import ROOT
from tools.deploy import k230_robot as deploy


class RobotBundleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.car_source = deploy.car_bundle(near_parking=True)
        cls.arm_source = deploy.arm_bundle(deploy.DEFAULT_PICK_CONFIG)

    def build(self, car=None, arm=None):
        with patch.object(deploy, 'car_bundle', return_value=copy.deepcopy(
                self.car_source if car is None else car)) as car_build, \
                patch.object(deploy, 'arm_bundle', return_value=copy.deepcopy(
                self.arm_source if arm is None else arm)) as arm_build:
            result = deploy.build_bundle()
        car_build.assert_called_once_with(near_parking=True)
        arm_build.assert_called_once_with(deploy.DEFAULT_PICK_CONFIG)
        return result

    def test_complete_nested_configs_keep_the_accepted_motion_and_calibration(self):
        config, files, manifest = self.build()
        self.assertEqual(set(config), {'car', 'arm', 'voice', 'fill_light'})
        self.assertEqual(config['fill_light'], {'enabled':True, 'gpio':26, 'active_high':True})
        self.assertEqual(config['car'], self.car_source[0])
        self.assertEqual(config['arm'], self.arm_source[0])
        self.assertEqual(config['voice'], self.arm_source[0]['voice'])
        self.assertEqual(json.loads(files['config.json']), config)
        self.assertEqual(config['car']['intrinsics'], config['arm']['intrinsics'])
        self.assertEqual(config['car']['extrinsics'], config['arm']['extrinsics'])
        self.assertEqual(config['car']['table_plane'],
                         config['arm']['vision']['localization']['table_plane_Base_mm'])
        self.assertFalse(manifest['startup_motion'])
        self.assertFalse(manifest['pc_runtime_required'])
        self.assertEqual(manifest['completion'], 'arm_stage_completed_with_encoder_grip_confirmation')

    def test_promoted_arm_default_keeps_authorized_motion_rules_and_shared_raw_calibration(self):
        config,unused,unused_manifest=self.build()
        arm=config['arm'];car=config['car'];pick=arm['pick']
        self.assertEqual(pick['model_urdf'],'models/genkiarm/genkiarm_corrected.urdf')
        self.assertEqual(pick['vision_config'],'configs/vision/trials/model_alignment_20261030.json')
        self.assertEqual(pick['cube_offset_toward_moving_finger_mm'],0)
        self.assertTrue(pick['wrist_alignment_enabled'])
        self.assertEqual(pick['status'],'user_accepted_arm_stage_baseline')
        self.assertEqual((pick['opening_angle_deg'],pick['opening_count']),(25,2332))
        self.assertEqual(pick['alignment_reference'],'open_fingertip_midpoint')
        self.assertEqual(pick['forward_extension_mm'],0)
        self.assertEqual([pick[k] for k in
            ('hover_tip_height_mm','descend_tip_height_mm','lift_tip_height_mm')],[80,15,80])
        self.assertEqual(pick['extra_closing_counts'],20)
        self.assertEqual(pick['stage_durations_s']['lift'],1)
        self.assertEqual(pick['standalone_stage_policy'],{
            'hover_tolerance_counts':30,'descend_tolerance_counts':10,
            'descend_axis4_lead_ms':200,'gripper_stable_samples':5,
            'gripper_stable_tolerance_counts':1,'poll_interval_ms':50,
            'grip_confirmation_delta_counts':30})
        self.assertEqual(arm['control']['profiles'],[
            {'speed_raw':v,'acceleration_raw':a} for v,a in
            [(150,5),(600,20),(600,20),(600,20),(150,5),(150,5)]])
        expected_pid=[(24,0,100,0),(48,24,160,48),(40,12,140,24),
                      (20,3,60,16),(24,0,60,0),(8,0,60,0)]
        self.assertEqual([tuple(row[k] for k in ('21','23','22','25'))
                          for row in arm['control']['registers']],expected_pid)
        self.assertEqual(arm['ready_control']['goals'],[2048,1081,3015,3015,2048,2332])
        self.assertEqual(arm['drive_ready_control']['goals'],[2048,1138,2958,2958,2048,2332])
        self.assertEqual(arm['ready_control']['ready_pose_number'],1)
        self.assertEqual(arm['drive_ready_control']['ready_pose_number'],2)
        self.assertEqual(arm['ready_control']['angles_deg'],[0,-85,85,85,0,25])
        self.assertEqual(arm['drive_ready_control']['angles_deg'],[0,-80,80,80,0,25])
        self.assertEqual(arm['drive_ready_control']['profiles'],arm['ready_control']['profiles'])
        self.assertEqual(arm['extrinsics']['geometry_model_sha256'],
                         deploy.digest((ROOT/pick['model_urdf']).read_bytes()))
        self.assertEqual(car['intrinsics'],arm['intrinsics'])
        self.assertEqual(car['extrinsics']['T_Base_from_Camera_mm'],
                         arm['extrinsics']['T_Base_from_Camera_mm'])
        self.assertEqual(car['projection']['T'],arm['extrinsics']['T_Base_from_Camera_mm'])
        self.assertEqual(car['table_plane'],arm['vision']['localization']['table_plane_Base_mm'])
        for joint in arm['model']['joints'][1:4]:
            self.assertEqual(joint['axis'],[0.,-1.,0.])

    def test_default_bundle_is_the_accepted_profile_and_fixed_wrist_remains_explicit(self):
        default=deploy.build_bundle()
        self.assertEqual(default,deploy.build_bundle(deploy.DEFAULT_PICK_CONFIG))
        fixed=deploy.build_bundle(ROOT/'configs/arm/pick_validation_pc.json')
        self.assertNotIn('wrist_alignment_enabled',fixed[0]['arm']['pick'])
        for key in ('car','voice'):
            self.assertEqual(default[0][key],fixed[0][key])
        for key in ('model','control','ready_control','drive_ready_control','intrinsics','extrinsics'):
            self.assertEqual(default[0]['arm'][key],fixed[0]['arm'][key])

    def test_flat_sources_and_manifest_include_both_executors_and_static_solver(self):
        unused, files, manifest = self.build()
        for source in (self.car_source[1], self.arm_source[1]):
            for name, data in source.items():
                if name not in ('config.json', 'manifest.json'):
                    self.assertEqual(files[name], data, name)
        for name, relative in (
                ('robot_task.py', 'device/k230/robot/task.py'),
                ('robot_app.py', 'device/k230/robot/app.py'),
                ('fill_light.py', 'device/k230/robot/fill_light.py'),
                ('static_pick_session.py', 'device/k230/vision/static_pick_session.py')):
            self.assertEqual(files[name], (ROOT / relative).read_bytes())
        for name, data in files.items():
            if name.endswith('.py'):
                ast.parse(data, filename=name)
            if name != 'manifest.json':
                self.assertEqual(manifest['files'][name],
                                 {'bytes':len(data), 'sha256':deploy.digest(data)})
        self.assertEqual(json.loads(files['manifest.json']), manifest)
        unhashed = dict(manifest)
        del unhashed['release_id']
        self.assertEqual(manifest['release_id'], deploy.digest(deploy.encoded(unhashed))[:16])
        self.assertNotIn('manifest.json', manifest['files'])

    def test_identical_common_files_are_shared_but_conflicting_files_fail(self):
        car, arm = copy.deepcopy(self.car_source), copy.deepcopy(self.arm_source)
        self.assertEqual(car[1]['pose.py'], arm[1]['pose.py'])
        arm[1]['pose.py'] += b'\n# conflicting release\n'
        with self.assertRaisesRegex(ValueError, 'Common file differs: pose.py'):
            self.build(car, arm)

    def test_intrinsics_extrinsics_and_table_must_match(self):
        for name in ('intrinsics', 'extrinsics', 'table_plane'):
            car = copy.deepcopy(self.car_source)
            if name == 'table_plane':
                car[0][name]['offset_mm'] += 1
            elif name == 'intrinsics':
                car[0][name]['camera_matrix'][0][0] += 1
            else:
                car[0][name]['T_Base_from_Camera_mm'][0][3] += 1
            with self.subTest(name=name), self.assertRaisesRegex(ValueError, name):
                self.build(car=car)

    def test_projection_cannot_disagree_with_formal_scaled_camera_or_table(self):
        for name in ('K', 'D', 'T', 'plane'):
            car = copy.deepcopy(self.car_source)
            value = car[0]['projection'][name]
            if name == 'plane':
                value['offset_mm'] += 1
            elif name == 'D':
                value[0] += .01
            else:
                value[0][0] += 1
            with self.subTest(name=name), self.assertRaisesRegex(ValueError, 'projection'):
                self.build(car=car)

    def test_motion_reference_must_match_the_window_coordinate_transform(self):
        for key in ('forward_axis_Base','motion_origin_Base_mm'):
            car=copy.deepcopy(self.car_source)
            car[0]['projection'][key][0] += .1
            with self.subTest(key=key), self.assertRaisesRegex(ValueError,'projection'):
                self.build(car=car)

    def test_verification_hashes_manifest_and_refreshes_all_flat_imports_without_motion(self):
        config, unused, manifest = self.build()
        remote = deploy.REMOTE + '/releases/' + manifest['release_id']
        with patch.object(deploy, 'execute', return_value=[
                'ROBOT_RELEASE_VERIFIED ' + manifest['release_id']]) as execute:
            deploy.verify(Mock(), remote, manifest, config)
        source = execute.call_args.args[1]
        ast.parse(source)
        self.assertIn(deploy.digest(deploy.encoded(manifest)), source)
        self.assertIn('del sys.modules[name]', source)
        self.assertIn('for name in modules:__import__(name)', source)
        for name in ('robot_task', 'robot_app', 'static_pick_session', 'app', 'vision', 'control'):
            self.assertIn(repr(name), source)
        self.assertNotIn('run_forever(', source)
        self.assertNotIn('UART(', source)
        self.assertNotIn('CameraSession(', source)

    def test_failed_verification_is_not_reported_as_verified(self):
        config, unused, manifest = self.build()
        with patch.object(deploy, 'execute', return_value=['hash/import failure']), \
                self.assertRaisesRegex(RuntimeError, 'import/hash'):
            deploy.verify(Mock(), '/sdcard/aic/robot/releases/test', manifest, config)


class RobotDeploymentTests(unittest.TestCase):
    def bundle(self):
        config = {'car':{}, 'arm':{}, 'voice':{}}
        files = {'config.json':deploy.encoded(config), 'robot_app.py':b'pass\n'}
        manifest = {'schema':'aic.robot.v1', 'release_id':'test-release',
                    'startup_motion':False, 'files':{}}
        files['manifest.json'] = deploy.encoded(manifest)
        return config, files, manifest

    def record(self, folder):
        path = next((Path(folder) / 'data/deployments').glob('*/deployment.json'))
        return path.parent, json.loads(path.read_text())

    def test_dry_build_never_opens_hardware(self):
        with tempfile.TemporaryDirectory() as folder, \
                patch.object(deploy, 'ROOT', Path(folder)), \
                patch.object(deploy, 'build_bundle', return_value=self.bundle()), \
                patch.object(deploy, 'CameraLink') as connect, patch('builtins.print'):
            self.assertEqual(deploy.main([]), 0)
            output, record = self.record(folder)
            self.assertEqual(record['status'], 'built_no_hardware_access')
            self.assertEqual(record['remote'], '/sdcard/aic/robot/releases/test-release')
            self.assertFalse(record['activated'])
            connect.assert_not_called()
            startup = (output / 'main.py').read_text()
            self.assertIn('from robot_app import run_forever', startup)
            self.assertIn(record['remote'], startup)

    def test_activation_requires_explicit_install(self):
        with patch('sys.stderr'), self.assertRaises(SystemExit) as error:
            deploy.main(['--activate'])
        self.assertEqual(error.exception.code, 2)

    def test_install_verifies_and_preserves_current_startup(self):
        bundle = self.bundle()
        link = Mock()
        with tempfile.TemporaryDirectory() as folder, \
                patch.object(deploy, 'ROOT', Path(folder)), \
                patch.object(deploy, 'build_bundle', return_value=bundle), \
                patch.object(deploy, 'CameraLink', return_value=link) as connect, \
                patch.object(deploy, 'stop_previous_script') as stop, \
                patch.object(deploy, 'mkdir') as mkdir, patch.object(deploy, 'write_file') as write, \
                patch.object(deploy, 'verify') as verify, patch('builtins.print'):
            self.assertEqual(deploy.main(['--install', '--port', '/dev/test-k230']), 0)
            unused, record = self.record(folder)
        connect.assert_called_once_with('/dev/test-k230')
        stop.assert_called_once_with(link)
        self.assertEqual([call.args for call in mkdir.call_args_list],
                         [(link, record['remote']), (link, '/sdcard/aic/pick')])
        verify.assert_called_once_with(link, record['remote'], bundle[2], bundle[0])
        self.assertEqual(record['status'], 'installed_verified_not_active')
        self.assertFalse(record['activated'])
        self.assertTrue(all(call.args[1].startswith(record['remote'] + '/')
                            for call in write.call_args_list))
        self.assertEqual(write.call_count, len(bundle[1]))
        link.port.close.assert_called_once()

    def test_activation_backs_up_previous_main_and_only_replaces_boot_after_verify(self):
        bundle = self.bundle()
        link, order = Mock(), []
        previous = b'# previous standalone release\n'
        backup_line = 'BACKUP ' + base64.b64encode(previous).decode()
        with tempfile.TemporaryDirectory() as folder, \
                patch.object(deploy, 'ROOT', Path(folder)), \
                patch.object(deploy, 'build_bundle', return_value=bundle), \
                patch.object(deploy, 'CameraLink', return_value=link), \
                patch.object(deploy, 'stop_previous_script'), patch.object(deploy, 'mkdir'), \
                patch.object(deploy, 'write_file', side_effect=lambda *args:order.append(args[1])) as write, \
                patch.object(deploy, 'verify', side_effect=lambda *args:order.append('verified')), \
                patch.object(deploy, 'execute', return_value=[backup_line]) as execute, \
                patch('builtins.print'):
            self.assertEqual(deploy.main(['--install', '--activate']), 0)
            output, record = self.record(folder)
            self.assertEqual((output / 'previous_main.py').read_bytes(), previous)
        self.assertEqual(order[-2:], ['verified', '/sdcard/main.py'])
        self.assertTrue(record['activated'])
        self.assertEqual(record['status'], 'activated_next_boot_only')
        self.assertFalse(record['motion_commands_sent'])
        self.assertFalse(record['arm_commands_sent'])
        self.assertEqual(execute.call_count, 1)
        self.assertNotIn('reset', execute.call_args.args[1])
        self.assertEqual(write.call_args.args[1], '/sdcard/main.py')
        link.port.close.assert_called_once()

    def test_import_failure_prevents_activation(self):
        link = Mock()
        with tempfile.TemporaryDirectory() as folder, \
                patch.object(deploy, 'ROOT', Path(folder)), \
                patch.object(deploy, 'build_bundle', return_value=self.bundle()), \
                patch.object(deploy, 'CameraLink', return_value=link), \
                patch.object(deploy, 'stop_previous_script'), patch.object(deploy, 'mkdir'), \
                patch.object(deploy, 'write_file') as write, \
                patch.object(deploy, 'verify', side_effect=RuntimeError('import failed')), \
                patch('builtins.print'):
            with patch('sys.stderr'):
                self.assertEqual(deploy.main(['--install', '--activate']), 1)
            unused, record = self.record(folder)
        self.assertEqual(record['status'], 'failed')
        self.assertFalse(record['activated'])
        self.assertNotIn('/sdcard/main.py', [call.args[1] for call in write.call_args_list])
        link.port.close.assert_called_once()

    def test_sd_read_error_is_not_treated_as_a_missing_startup_backup(self):
        def execute_backup(unused_link, source):
            printed = io.StringIO()
            with patch('builtins.open', side_effect=OSError(5, 'SD read failed')), \
                    redirect_stdout(printed):
                exec(source, {})
            return printed.getvalue().splitlines()
        link = Mock()
        with tempfile.TemporaryDirectory() as folder, \
                patch.object(deploy, 'ROOT', Path(folder)), \
                patch.object(deploy, 'build_bundle', return_value=self.bundle()), \
                patch.object(deploy, 'CameraLink', return_value=link), \
                patch.object(deploy, 'stop_previous_script'), patch.object(deploy, 'mkdir'), \
                patch.object(deploy, 'write_file') as write, \
                patch.object(deploy, 'verify'), \
                patch.object(deploy, 'execute', side_effect=execute_backup), \
                redirect_stdout(io.StringIO()), patch('sys.stderr'):
            self.assertEqual(deploy.main(['--install', '--activate']), 1)
            unused, record = self.record(folder)
        self.assertFalse(record['activated'])
        self.assertEqual(record['status'], 'failed')
        self.assertNotIn('/sdcard/main.py', [call.args[1] for call in write.call_args_list])
        link.port.close.assert_called_once()


if __name__ == '__main__':
    unittest.main()
