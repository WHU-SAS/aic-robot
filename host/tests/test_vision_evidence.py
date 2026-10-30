"""Raw visual evidence survives retries, reboot-like clocks and failures."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from device.k230.vision import evidence


class EvidenceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / 'nested' / 'evidence'
        self.config = {'camera': {'width': 1280, 'height': 960}, 'intrinsics_sha256': 'intrinsic-id'}
        self.version = {'release_id': 'release-a', 'algorithm_sha256': 'code-id'}
        self.frame = {'frame_id': 9, 'capture_timestamp_ms': None, 'clock_domain': None,
                      'timestamp_source': 'unavailable', 'image_size_px': [1280, 960]}
        self.jpeg = b'\xff\xd8' + bytes(range(256)) * 50 + b'\xff\xd9'

    def run_record(self):
        return evidence.EvidenceRun(str(self.root), 'arm_pick', self.config, self.version)

    def read_manifest(self, run):
        return json.loads(Path(run.manifest_path).read_text())

    def test_reboot_same_clock_creates_new_run_without_overwrite(self):
        with patch.object(evidence, '_ticks', return_value=123):
            first = self.run_record()
            first.save_jpeg('scene', self.jpeg, self.frame)
            first.finish('success')
            before = Path(first.manifest_path).read_bytes()
            second = self.run_record()
            second.save_jpeg('scene', b'new-original', self.frame)
            second.finish('failed', 'edge missing')
            third = self.run_record()
        self.assertEqual(len({first.directory, second.directory, third.directory}), 3)
        self.assertEqual(Path(first.manifest_path).read_bytes(), before)
        self.assertEqual((Path(first.directory) / 'scene.jpg').read_bytes(), self.jpeg)
        self.assertTrue(third.directory.endswith('_2'))

    def test_reserved_frame_has_durable_preliminary_manifest(self):
        run = self.run_record()
        target = run.new_frame('scene', self.frame)
        saved = self.read_manifest(run)
        self.assertEqual(saved['schema'], 'aic.vision_evidence.v1')
        self.assertEqual(saved['status'], 'recording')
        self.assertFalse(saved['archive_complete'])
        self.assertEqual(saved['frames'][0]['image_status'], 'pending')
        self.assertIsNone(saved['frames'][0]['image_sha256'])
        self.assertFalse(Path(target).exists())
        self.assertEqual(Path(target).parent, Path(run.directory))

    def test_config_version_and_frame_snapshots_preserve_unknown_capture_time(self):
        run = self.run_record()
        run.save_jpeg('scene', self.jpeg, self.frame)
        self.config['camera']['width'] = 1
        self.version['release_id'] = 'mutated'
        self.frame['capture_timestamp_ms'] = 888
        saved = run.finish('success')
        self.assertIsNone(saved['frames'][0]['frame']['capture_timestamp_ms'])
        self.assertIsNone(saved['frames'][0]['frame']['clock_domain'])
        self.assertEqual(saved['frames'][0]['frame']['timestamp_source'], 'unavailable')
        self.assertEqual(saved['version']['release_id'], 'release-a')
        self.assertEqual(json.loads((Path(run.directory) / saved['config_file']).read_text())['camera']['width'], 1280)
        self.assertNotIn('capture_timestamp_ms', {k: v for k, v in saved.items() if k != 'frames'})
        self.assertEqual(saved['created_time_source'], 'local_ticks_not_capture_time_or_utc')

    def test_original_jpeg_bytes_and_sha_are_from_actual_saved_file(self):
        run = self.run_record()
        target = run.save_jpeg('scene', memoryview(self.jpeg), self.frame)
        self.assertEqual(Path(target).read_bytes(), self.jpeg)
        saved = run.finish('success')['frames'][0]
        self.assertEqual(saved['image_sha256'], hashlib.sha256(self.jpeg).hexdigest())
        self.assertEqual(saved['image_bytes'], len(self.jpeg))
        self.assertEqual(saved['image_status'], 'complete')

    def test_native_save_target_hashes_actual_native_file_and_pairs_results(self):
        run = self.run_record()
        path_a = run.new_frame('moving', {'frame_id': 1, 'capture_timestamp_ms': 10})
        path_b = run.new_frame('stopped', {'frame_id': 2, 'capture_timestamp_ms': 30})
        Path(path_a).write_bytes(b'actual native jpeg A')
        Path(path_b).write_bytes(b'actual native jpeg B')
        observation = {'status': 'no_complete_face', 'reason': 'bottom reflection'}
        run.record_observation('moving', observation)
        observation['reason'] = 'modified by caller'
        run.record_observation('stopped', {'status': 'localized', 'cube_center_Base_mm': [240, 1, -50]})
        self.assertEqual(self.read_manifest(run)['frames'][0]['observation']['reason'], 'bottom reflection')
        saved = run.finish('success')
        self.assertEqual([x['frame']['frame_id'] for x in saved['frames']], [1, 2])
        self.assertEqual(saved['frames'][1]['observation']['cube_center_Base_mm'], [240, 1, -50])
        self.assertEqual(saved['frames'][0]['image_sha256'], hashlib.sha256(b'actual native jpeg A').hexdigest())

    def test_failure_still_keeps_original_and_diagnostics(self):
        run = self.run_record()
        run.save_jpeg('scene', self.jpeg, self.frame, {'status': 'failed', 'reason': 'no edge'})
        saved = run.finish('vision_failed', ValueError('no edge'))
        self.assertEqual(saved['status'], 'vision_failed')
        self.assertEqual(saved['error'], 'no edge')
        self.assertTrue(saved['archive_complete'])
        self.assertEqual((Path(run.directory) / 'scene.jpg').read_bytes(), self.jpeg)
        self.assertEqual(self.read_manifest(run), saved)

    def test_missing_or_empty_image_cannot_be_finalized_as_success(self):
        run = self.run_record()
        run.new_frame('missing', self.frame)
        empty_path = run.new_frame('empty', self.frame)
        Path(empty_path).write_bytes(b'')
        saved = run.finish('success')
        self.assertEqual(saved['status'], 'partial')
        self.assertEqual(saved['requested_status'], 'success')
        self.assertFalse(saved['archive_complete'])
        self.assertEqual([x['image_status'] for x in saved['frames']], ['missing', 'empty'])
        self.assertIsNone(saved['frames'][0]['image_bytes'])
        self.assertEqual(saved['frames'][1]['image_bytes'], 0)

    def test_same_label_and_finalized_run_cannot_overwrite_saved_evidence(self):
        run = self.run_record()
        target = run.save_jpeg('scene', self.jpeg, self.frame)
        with self.assertRaisesRegex(ValueError, 'already reserved'):
            run.save_jpeg('scene', b'replacement', self.frame)
        self.assertEqual(Path(target).read_bytes(), self.jpeg)
        run.finish('success')
        frozen = Path(run.manifest_path).read_bytes()
        with self.assertRaisesRegex(ValueError, 'finalized'):
            run.new_frame('later', self.frame)
        with self.assertRaisesRegex(ValueError, 'finalized'):
            run.record_observation('scene', {})
        with self.assertRaisesRegex(ValueError, 'finalized'):
            run.finish('replacement')
        self.assertEqual(Path(run.manifest_path).read_bytes(), frozen)

    def test_existing_untracked_jpeg_is_not_overwritten(self):
        run = self.run_record()
        target = Path(run.directory) / 'scene.jpg'
        target.write_bytes(b'irreplaceable')
        with self.assertRaisesRegex(ValueError, 'overwrite'):
            run.new_frame('scene', self.frame)
        self.assertEqual(target.read_bytes(), b'irreplaceable')

    def test_invalid_names_cannot_escape_run_directory(self):
        for invalid in ('', '../escape', 'a/b', 'a\\b', 'x.y', 'x y', '\u56fe'):
            with self.assertRaises(ValueError):
                evidence.EvidenceRun(str(self.root), invalid, {})
        run = self.run_record()
        for invalid in ('../escape', '', 'a/b'):
            with self.assertRaises(ValueError):
                run.new_frame(invalid, self.frame)
        with self.assertRaisesRegex(ValueError, 'Unknown'):
            run.record_observation('never_reserved', {})

    def test_save_disk_error_propagates_and_preliminary_record_survives(self):
        run = self.run_record()
        original_open = open
        def failing_open(path, mode='r', *args, **kwargs):
            if str(path).endswith('.jpg') and mode == 'wb':
                raise OSError(28, 'disk full')
            return original_open(path, mode, *args, **kwargs)
        with patch('builtins.open', side_effect=failing_open):
            with self.assertRaisesRegex(OSError, 'disk full'):
                run.save_jpeg('scene', self.jpeg, self.frame)
        self.assertEqual(self.read_manifest(run)['frames'][0]['label'], 'scene')
        saved = run.finish('archive_failed', 'disk full')
        self.assertEqual(saved['status'], 'partial')
        self.assertEqual(saved['error'], 'disk full')

    def test_hash_or_atomic_manifest_failure_propagates_without_claiming_completion(self):
        run = self.run_record()
        run.save_jpeg('scene', self.jpeg, self.frame)
        with patch.object(evidence, '_file_digest', side_effect=OSError(5, 'read error')):
            with self.assertRaisesRegex(OSError, 'read error'):
                run.finish('success')
        self.assertEqual(self.read_manifest(run)['status'], 'recording')
        with patch.object(evidence.os, 'rename', side_effect=OSError(28, 'disk full')):
            with self.assertRaisesRegex(OSError, 'disk full'):
                run.finish('success')
        self.assertEqual(self.read_manifest(run)['status'], 'recording')
        self.assertTrue(Path(run.manifest_path + '.tmp').exists())
        run.finish('success')
        self.assertEqual(self.read_manifest(run)['status'], 'success')

    def test_fat_rename_over_existing_manifest_supports_all_updates(self):
        real_rename = evidence.os.rename
        def fat_rename(source, target):
            if Path(target).exists():
                raise OSError(17, 'FAT target exists')
            return real_rename(source, target)
        with patch.object(evidence.os, 'rename', side_effect=fat_rename):
            run = self.run_record()
            run.save_jpeg('scene', self.jpeg, self.frame, {'status': 'localized'})
            saved = run.finish('success')
        self.assertEqual(self.read_manifest(run), saved)
        self.assertEqual(saved['frames'][0]['image_bytes'], len(self.jpeg))
        self.assertFalse(Path(run.manifest_path + '.replace_previous').exists())

    def test_fat_failed_replacement_restores_prior_manifest_and_keeps_pending(self):
        run = self.run_record()
        prior = Path(run.manifest_path).read_bytes()
        real_rename = evidence.os.rename
        fail_install = [True]
        def fat_rename(source, target):
            if Path(target).exists():
                raise OSError(17, 'FAT target exists')
            if str(source) == run.manifest_path + '.tmp' and fail_install[0]:
                fail_install[0] = False
                raise OSError(5, 'replacement failed')
            return real_rename(source, target)
        with patch.object(evidence.os, 'rename', side_effect=fat_rename):
            with self.assertRaisesRegex(OSError, 'replacement failed'):
                run.new_frame('scene', self.frame)
            self.assertEqual(Path(run.manifest_path).read_bytes(), prior)
            self.assertTrue(Path(run.manifest_path + '.tmp').exists())
            self.assertFalse(Path(run.manifest_path + '.replace_previous').exists())
            saved = run.finish('archive_failed', 'replacement failed')
        self.assertEqual(saved['status'], 'partial')
        self.assertEqual(self.read_manifest(run)['error'], 'replacement failed')

    def test_unresolved_replacement_backup_is_never_overwritten_or_removed(self):
        run = self.run_record()
        prior = Path(run.manifest_path).read_bytes()
        backup = Path(run.manifest_path + '.replace_previous')
        backup.write_bytes(b'prior recovery metadata')
        with self.assertRaisesRegex(OSError, 'Unresolved'):
            run.new_frame('scene', self.frame)
        self.assertEqual(backup.read_bytes(), b'prior recovery metadata')
        self.assertEqual(Path(run.manifest_path).read_bytes(), prior)

    def test_short_jpeg_writes_retry_without_reencoding_or_truncation(self):
        class ShortWriter:
            def __init__(self):
                self.data = bytearray()
                self.calls = 0
            def write(self, data):
                count = min(len(data), 7)
                self.data.extend(data[:count])
                self.calls += 1
                return count
        writer = ShortWriter()
        evidence._write_all(writer, memoryview(self.jpeg))
        self.assertEqual(bytes(writer.data), self.jpeg)
        self.assertGreater(writer.calls, 1)
        self.assertEqual(hashlib.sha256(writer.data).hexdigest(), hashlib.sha256(self.jpeg).hexdigest())

    def test_zero_none_negative_or_impossible_write_counts_raise(self):
        class InvalidWriter:
            def __init__(self, result):
                self.result = result
            def write(self, data):
                return self.result
        for invalid in (0, None, -1, len(self.jpeg) + 1):
            with self.subTest(invalid=invalid):
                with self.assertRaisesRegex(OSError, 'valid progress'):
                    evidence._write_all(InvalidWriter(invalid), self.jpeg)

    def test_failed_partial_jpeg_is_not_reported_as_complete(self):
        run = self.run_record()
        original_open = open
        class FullThenZero:
            def __init__(self, stream):
                self.stream, self.written = stream, False
            def __enter__(self):
                return self
            def __exit__(self, *args):
                self.stream.close()
            def write(self, data):
                if self.written:
                    return 0
                self.written = True
                return self.stream.write(data[:7])
            def flush(self):
                self.stream.flush()
        def short_open(path, mode='r', *args, **kwargs):
            stream = original_open(path, mode, *args, **kwargs)
            return FullThenZero(stream) if str(path).endswith('.jpg') and mode == 'wb' else stream
        with patch('builtins.open', side_effect=short_open):
            with self.assertRaisesRegex(OSError, 'valid progress'):
                run.save_jpeg('scene', self.jpeg, self.frame)
        saved = run.finish('archive_failed', 'JPEG write stopped')
        self.assertEqual(saved['status'], 'partial')
        self.assertEqual(saved['frames'][0]['image_status'], 'incomplete')
        self.assertEqual(saved['frames'][0]['image_bytes'], 7)
        self.assertEqual(saved['frames'][0]['image_expected_bytes'], len(self.jpeg))
        self.assertEqual(saved['frames'][0]['image_sha256'], hashlib.sha256(self.jpeg[:7]).hexdigest())

    def test_unicode_json_uses_binary_byte_counts_on_native_style_writes(self):
        original_dumps, original_open = json.dumps, open
        json_modes = []
        class NativeTextWriter:
            def __init__(self, stream):
                self.stream = stream
            def __enter__(self):
                return self
            def __exit__(self, *args):
                self.stream.close()
            def write(self, data):
                self.stream.write(data)
                return len(data.encode('utf-8'))
            def flush(self):
                self.stream.flush()
        def native_open(path, mode='r', *args, **kwargs):
            stream = original_open(path, mode, *args, **kwargs)
            if str(path).endswith('.json.tmp'):
                json_modes.append(mode)
                if mode == 'w':
                    return NativeTextWriter(stream)
            return stream
        # MicroPython json emits Unicode directly, unlike CPython's default
        # escaped ASCII. Its text streams then report encoded byte counts.
        with patch.object(evidence.json, 'dumps', side_effect=lambda value: original_dumps(value, ensure_ascii=False)):
            with patch('builtins.open', side_effect=native_open):
                run = evidence.EvidenceRun(str(self.root), 'voice_pick', {'wake_word': '思达思达'})
                run.save_jpeg('scene', self.jpeg, self.frame, {'reason': '未识别到完整方块'})
                run.finish('vision_failed', '方块被遮挡')
        self.assertTrue(json_modes)
        self.assertEqual(set(json_modes), {'wb'})
        self.assertIn('思达思达'.encode('utf-8'), (Path(run.directory) / 'config.json').read_bytes())
        self.assertEqual(json.loads((Path(run.directory) / 'config.json').read_text())['wake_word'], '思达思达')
        self.assertEqual(self.read_manifest(run)['error'], '方块被遮挡')
        self.assertEqual(self.read_manifest(run)['frames'][0]['observation']['reason'], '未识别到完整方块')

    def test_root_io_error_is_not_misreported_as_collision(self):
        with patch.object(evidence.os, 'mkdir', side_effect=OSError(13, 'permission denied')):
            with self.assertRaisesRegex(OSError, 'permission denied'):
                self.run_record()


if __name__ == '__main__':
    unittest.main()
