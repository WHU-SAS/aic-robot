"""Evidence mechanics only: actual geometry remains covered by vision tests."""
import hashlib
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from host.vision.hard_dataset import (add_case, calibration_snapshot, import_evidence,
                                     label_case, list_cases, replay, replay_case, summarize)


class HardVisionDatasetTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.dataset = self.root / 'dataset'
        self.run = self.root / 'run'
        self.run.mkdir()
        self.image = self.run / 'frame.jpg'
        self.image.write_bytes(b'original JPEG fixture bytes, not decoded by mechanics tests')
        self.intr = dict(camera_matrix=[[800, 0, 640], [0, 800, 480], [0, 0, 1]],
                         distortion_coefficients=[0] * 5, image_width=1280, image_height=960)
        self.ext = dict(T_Base_from_Camera_mm=[[1, 0, 0, 10], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        self.config = self.root / 'vision.json'
        self.write(self.config, self.bundle())
        self.old = dict(status='localized', cube_center_Base_mm=[200, 10, -50])
        self.write(self.run / 'vision_record.json', self.old)
        self.loader = Mock(return_value=SimpleNamespace(shape=(960, 1280, 3)))
        self.localizer = Mock(return_value=dict(status='localized', cube_center_Base_mm=[202, 9, -50],
                             quality={'reprojection_rms_px': 0.7, 'side_selection': 'larger_side'},
                             payload={'poses': [{'table_geometry': {'face_kind': 'side'}}]}))

    def bundle(self):
        return dict(intrinsics=self.intr, extrinsics=self.ext,
                    table_plane=dict(normal=[0, 0, 1], offset_mm=-60))

    def write(self, path, obj):
        path.write_text(json.dumps(obj))

    def folder(self, ident):
        return self.dataset / 'cases' / ident

    def add(self, **kwargs):
        return add_case(self.run, 'inaccurate', 'visibly behind cube', self.dataset, **kwargs)

    def test_copy_is_exact_and_duplicate_preserves_original_with_appended_labels(self):
        ident = self.add(vision_config=self.config)
        case = self.folder(ident) / 'case.json'
        original = case.read_bytes()
        (self.run / 'vision_record.json').write_text('{"status":"different later prediction"}')
        same = self.add(vision_config=self.config)
        self.assertEqual(ident, same)
        self.assertEqual(case.read_bytes(), original)
        self.assertEqual((case.parent / 'image.jpg').read_bytes(), self.image.read_bytes())
        self.assertEqual(len(list_cases(self.dataset)[0]['labels']), 2)
        self.assertEqual(len(list((case.parent / 'evidence').glob('*_vision_record.json'))), 2)

    def test_same_image_different_calibrations_are_distinct_cases(self):
        first = self.add(vision_config=self.config)
        self.ext['T_Base_from_Camera_mm'][0][3] = 20
        self.write(self.config, self.bundle())
        second = self.add(vision_config=self.config)
        self.assertNotEqual(first, second)
        self.assertEqual(len(list_cases(self.dataset)), 2)

    def test_missing_calibration_skips_without_reading_image_or_running_algorithm(self):
        ident = self.add()
        report = replay_case(self.folder(ident), image_loader=self.loader, localizer=self.localizer)
        self.assertEqual(report['replay_status'], 'skipped')
        self.loader.assert_not_called()
        self.localizer.assert_not_called()

    def test_comparison_calibration_never_scores_historical_center_or_computes_delta(self):
        ident = self.add(vision_config=self.config)
        label_case(self.dataset, ident, 'inaccurate', expected={'center_Base_mm': [202, 9, -50]})
        report = replay_case(self.folder(ident), image_loader=self.loader, localizer=self.localizer)
        self.assertEqual(report['calibration_role'], 'comparison')
        self.assertIsNone(report['delta_mm'])
        self.assertEqual(report['evaluation']['status'], 'uncertain')
        frame = self.localizer.call_args.args[1]
        self.assertIsNone(frame['capture_timestamp_ms'])
        self.assertIsNone(frame['clock_domain'])

    def test_original_calibration_delta_is_prediction_change_not_ground_truth(self):
        self.write(self.run / 'config.json', self.bundle())
        ident = self.add()
        report = replay_case(self.folder(ident), image_loader=self.loader, localizer=self.localizer)
        self.assertEqual(report['delta_mm'], [2, -1, 0])
        self.assertEqual(report['evaluation']['status'], 'uncertain')
        self.assertIn('not accuracy', report['delta_meaning'])
        original = (self.folder(ident) / 'case.json').read_bytes()
        label_case(self.dataset, ident, 'inaccurate', 'manually surveyed',
                   dict(center_Base_mm=[202, 9, -50], tolerance_mm=1, face_kind='side'))
        report = replay_case(self.folder(ident), image_loader=self.loader, localizer=self.localizer)
        self.assertEqual(report['evaluation']['status'], 'pass')
        self.assertEqual(original, (self.folder(ident) / 'case.json').read_bytes())

    def test_override_does_not_replace_saved_original_and_tampering_is_rejected(self):
        self.write(self.run / 'config.json', self.bundle())
        ident = self.add()
        self.ext['T_Base_from_Camera_mm'][0][3] = 999
        self.write(self.config, self.bundle())
        report = replay_case(self.folder(ident), self.config, self.localizer, self.loader)
        self.assertIn('comparison_config_ignored', report)
        self.assertEqual(self.localizer.call_args.args[3]['T_Base_from_Camera_mm'][0][3], 10)
        path = self.folder(ident) / 'calibration.json'
        obj = json.loads(path.read_text())
        obj['extrinsics']['T_Base_from_Camera_mm'][0][3] = 888
        self.write(path, obj)
        with self.assertRaisesRegex(ValueError, 'calibration SHA256'):
            replay_case(self.folder(ident), localizer=self.localizer, image_loader=self.loader)

    def test_original_image_hash_is_checked_before_localization(self):
        ident = self.add(vision_config=self.config)
        (self.folder(ident) / 'image.jpg').write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError, 'image SHA256'):
            replay_case(self.folder(ident), localizer=self.localizer, image_loader=self.loader)
        self.localizer.assert_not_called()

    @unittest.skipUnless(importlib.util.find_spec('cv2'), 'JPEG evidence decoding uses vision environment')
    def test_board_failures_import_success_requires_explicit_selection(self):
        self.valid_jpeg()
        self.write(self.run / 'config.json', self.bundle())
        manifest = dict(schema='aic.vision_evidence.v1', config_file='config.json',
                        version={'release': 'abc'}, frames=[
                            dict(label='failed', image_file='frame.jpg', observation=dict(status='no_complete_face'),
                                 image_sha256=hashlib.sha256(self.image.read_bytes()).hexdigest(), image_bytes=self.image.stat().st_size),
                            dict(label='succeeded', image_file='frame.jpg', observation=self.old),
                            dict(label='missing', image_file='lost.jpg', observation=dict(status='failed'))])
        path = self.run / 'evidence_manifest.json'
        self.write(path, manifest)
        result = import_evidence(path, self.dataset)
        self.assertEqual(len(result['imported']), 1)
        self.assertEqual(len(result['skipped']), 2)
        case = json.loads((self.folder(result['imported'][0]) / 'case.json').read_text())
        self.assertEqual(case['calibration_role'], 'original')
        self.assertEqual(case['source_version'], {'release': 'abc'})
        self.assertEqual(list_cases(self.dataset)[0]['labels'][0]['reason'], 'recognition_failure')
        result = import_evidence(path, self.dataset, 'inaccurate', 'manual report')
        self.assertEqual(len(result['imported']), 2)
        self.assertEqual(len(list_cases(self.dataset)), 1)

    def test_board_corrupt_image_rejected_and_missing_file_is_not_recovered_from_elsewhere(self):
        path = self.run / 'manifest.json'
        self.write(path, dict(schema='aic.vision_evidence.v1', frames=[dict(
            image_file='frame.jpg', image_sha256='bad', observation={'status': 'failed'})]))
        with self.assertRaisesRegex(ValueError, 'SHA256 mismatch'):
            import_evidence(path, self.dataset)

    def test_mismatched_intrinsics_extrinsics_files_are_not_replayed(self):
        self.write(self.root / 'intr.json', self.intr)
        self.write(self.root / 'ext.json', dict(self.ext, intrinsics_sha256='not the saved file'))
        self.write(self.config, dict(localization=dict(intrinsics='intr.json', extrinsics='ext.json')))
        with self.assertRaisesRegex(ValueError, 'SHA256 mismatch'):
            calibration_snapshot(self.config)

    def test_pc_context_preserves_original_config_and_frame_metadata(self):
        context = dict(self.bundle(), schema='aic.vision_context.v1', version={'source_sha256': {'a.py': 'abc'}})
        self.write(self.run / 'vision_context.json', context)
        observation = dict(self.old, frame_id='camera:7', capture_timestamp_ms=123,
                           clock_domain='board-boot-1', timestamp_source='sensor', image_size_px=[1280, 960])
        self.write(self.run / 'vision_record.json', dict(self.old, observation=observation))
        ident = self.add()
        case = json.loads((self.folder(ident) / 'case.json').read_text())
        self.assertEqual(case['source_version'], context['version'])
        self.assertEqual(case['calibration_role'], 'original')
        replay_case(self.folder(ident), localizer=self.localizer, image_loader=self.loader)
        self.assertEqual(self.localizer.call_args.args[1]['capture_timestamp_ms'], 123)
        self.assertEqual(self.localizer.call_args.args[1]['clock_domain'], 'board-boot-1')

    def test_wrong_pc_capture_hash_is_rejected(self):
        self.write(self.run / 'capture.json', dict(sha256='wrong'))
        with self.assertRaisesRegex(ValueError, 'PC capture image SHA256 mismatch'):
            self.add()

    def test_legacy_mutable_config_reference_is_not_promoted_to_original(self):
        self.write(self.run / 'execution.json', {'vision_config': str(self.config)})
        ident = self.add()
        case = json.loads((self.folder(ident) / 'case.json').read_text())
        self.assertIsNone(case['calibration_file'])

    def valid_jpeg(self):
        import cv2
        import numpy as np
        ok, encoded = cv2.imencode('.jpg', np.zeros((960, 1280, 3), dtype=np.uint8))
        self.assertTrue(ok)
        self.image.write_bytes(encoded.tobytes())

    def test_comparison_can_check_recognition_but_skips_metric_base_labels(self):
        ident = self.add(vision_config=self.config)
        label_case(self.dataset, ident, 'inaccurate', expected=dict(
            status='localized', face_kind='side', center_Base_mm=[0, 0, 0], tolerance_mm=1))
        report = replay_case(self.folder(ident), image_loader=self.loader, localizer=self.localizer)
        self.assertEqual(report['evaluation']['status'], 'pass')
        self.assertEqual(report['evaluation']['checks'], {'status': True, 'face_kind': True})
        self.assertIn('center_skipped', report['evaluation'])
        self.assertNotIn('human_center_error_mm', report['evaluation'])
        self.localizer.return_value = {'status': 'no_complete_face', 'reason': 'rejected'}
        report = replay_case(self.folder(ident), image_loader=self.loader, localizer=self.localizer)
        self.assertEqual(report['evaluation']['status'], 'fail')

    def test_face_summary_uses_actual_table_metrics_and_verified_face_fallbacks(self):
        from device.k230.pick.table_pose import context, metrics
        pose = dict(face_reprojection_rms_px=0.7, cube_center_Camera_mm=[200, 10, -50],
                    inward_face_normal_Camera=[1, 0, 0])
        plane = dict(normal=[0, 0, 1], offset_mm=-60)
        pose['table_geometry'] = metrics(pose, context(self.ext['T_Base_from_Camera_mm'], plane), 'side')
        self.assertEqual(summarize(dict(status='localized', payload={'poses': [pose]}))['face_kind'], 'side')
        self.assertEqual(summarize(dict(status='localized', payload={'top_boundary_verified': True}))['face_kind'], 'top')
        self.assertEqual(summarize(dict(status='localized', payload={'side_selection': 'larger_side'}))['face_kind'], 'side')
        self.assertIsNone(summarize(dict(status='localized', payload={}))['face_kind'])

    @unittest.skipUnless(importlib.util.find_spec('cv2'), 'JPEG evidence decoding uses vision environment')
    def test_board_perception_error_is_eligible_and_invalid_jpeg_is_partial(self):
        self.valid_jpeg()
        self.write(self.run / 'config.json', self.bundle())
        path = self.run / 'manifest.json'
        self.write(path, dict(schema='aic.vision_evidence.v1', status='perception_error',
                   config_file='config.json', frames=[dict(label='failed', image_file='frame.jpg', observation=None)]))
        result = import_evidence(path, self.dataset)
        self.assertEqual(len(result['imported']), 1)
        self.assertEqual(result['status'], 'complete')
        self.image.write_bytes(b'\xff\xd8not a decodable JPEG\xff\xd9')
        result = import_evidence(path, self.dataset)
        self.assertEqual(result['status'], 'partial')
        self.assertEqual(result['imported'], [])
        self.assertEqual(result['skipped'][0]['image_status'], 'invalid')
        self.image.write_bytes(b'\xff\xd8truncated')
        self.assertEqual(import_evidence(path, self.dataset)['status'], 'partial')

    def test_replay_reports_coverage_and_expectations_separately(self):
        for value in range(4):
            self.image.write_bytes(bytes([value]))
            self.add()
        results = [dict(case_id='a', replay_status='completed', new={'status': 'localized'}, evaluation={'status': 'pass'}),
                   dict(case_id='b', replay_status='completed', new={'status': 'no_complete_face'}, evaluation={'status': 'fail'}),
                   dict(case_id='c', replay_status='failed', reason='RuntimeError'),
                   dict(case_id='d', replay_status='skipped', reason='Missing calibration')]
        with patch('host.vision.hard_dataset.replay_case', side_effect=results):
            result = replay(self.dataset, output=self.root / 'report')
        self.assertEqual(result['summary'], dict(localized=1, rejected=1, errors=1, skipped=1,
                         expectation_pass=1, expectation_fail=1, expectation_uncertain=2))
        self.assertEqual(result['check_status'], 'fail')
        saved = json.loads(Path(result['report']).read_text())
        self.assertEqual(saved['summary'], result['summary'])
        empty = replay(self.root / 'empty', output=self.root / 'empty_report')
        self.assertEqual(empty['check_status'], 'fail')

    def test_replay_check_cli_exits_nonzero_only_for_reported_failures(self):
        from tools.analysis.vision_hard_cases import main
        with patch('tools.analysis.vision_hard_cases.replay', return_value={'check_status': 'fail'}), patch('builtins.print'):
            with self.assertRaises(SystemExit) as raised:
                main(['replay', '--check'])
            self.assertEqual(raised.exception.code, 1)
        with patch('tools.analysis.vision_hard_cases.replay', return_value={'check_status': 'uncertain'}), patch('builtins.print'):
            self.assertEqual(main(['replay', '--check'])['check_status'], 'uncertain')

    def test_invalid_human_labels_and_traversal_are_rejected(self):
        ident = self.add()
        with self.assertRaisesRegex(ValueError, 'finite'):
            label_case(self.dataset, ident, 'inaccurate', expected={'center_Base_mm': [float('nan'), 0, 0]})
        with self.assertRaisesRegex(ValueError, 'Invalid case ID'):
            label_case(self.dataset, '../bad', 'selected')
        with self.assertRaisesRegex(ValueError, 'Reason must'):
            label_case(self.dataset, ident, 'looks good')


if __name__ == '__main__':
    unittest.main()
