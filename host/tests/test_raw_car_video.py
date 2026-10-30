import ast
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from tools.diagnostics import record_car_video
from device.k230.car.video_storage import recording_space, required_free_bytes, write_video_packet


class RawCarVideoTests(unittest.TestCase):
    def test_full_card_is_rejected_and_one_minute_budget_has_headroom(self):
        full = (512, 512, 1023744, 0, 0, 0, 0, 0, 0, 255)
        space = recording_space(full, 60000)
        self.assertEqual(space['available_bytes'], 0)
        self.assertFalse(space['sufficient'])
        need = required_free_bytes(60000)
        self.assertGreater(need, 4096 * 1000 // 8 * 60)
        self.assertLess(need, 50 * 1024 * 1024)
        enough = (512, 512, 1023744, (need + 511) // 512, (need + 511) // 512)
        self.assertTrue(recording_space(enough, 60000)['sufficient'])

    def test_packet_short_writes_are_completed_or_report_no_progress(self):
        class Stream:
            def __init__(self, writes): self.writes = iter(writes); self.saved = b''
            def write(self, data):
                count = next(self.writes)
                self.saved += bytes(data[:count])
                return count
        stream = Stream([2, 3])
        write_video_packet(stream, b'frame')
        self.assertEqual(stream.saved, b'frame')
        with self.assertRaisesRegex(OSError, 'no progress'):
            write_video_packet(Stream([2, 0]), b'frame')

    def test_recording_script_has_no_detection_audio_or_motion_interfaces(self):
        script = record_car_video.build_script('/sdcard/car_video/example', 5000)
        ast.parse(script)
        for token in ('UART(', 'run_pick', 'run_ready', 'MotorLink', 'find_blobs', 'pyaudio'):
            self.assertNotIn(token, script)
        self.assertIn('1280', script)
        self.assertIn('960', script)

    def test_default_only_builds_and_never_accesses_board(self):
        with tempfile.TemporaryDirectory() as folder, patch.object(
                record_car_video, 'CameraLink', side_effect=AssertionError('No hardware')):
            self.assertEqual(record_car_video.main(['--output', folder, '--duration', '5']), 0)
            result = json.loads((Path(folder) / 'result.json').read_text())
            self.assertEqual(result['status'], 'built_no_hardware_access')
            self.assertFalse(result['motion_performed'])
            self.assertTrue(result['raw_video_only'])


if __name__ == '__main__': unittest.main()
