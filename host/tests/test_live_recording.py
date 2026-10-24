import io
import json
import os
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch

from device.k230.car import live_recording
from host.car.video_timing import frame_times
from tools.diagnostics import replay_k230_car_tracker as replay


class LiveRecordingTests(unittest.TestCase):
    def test_sd_recording_preserves_matching_frames_and_metadata_and_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            def local(path): return root / path.removeprefix('/sdcard/')
            class OS:
                def mkdir(self, path): local(path).mkdir()
                def stat(self, path): return local(path).stat()
                def rename(self, source, target): local(source).rename(local(target))
            real_open = open
            def sd_open(path, *args): return real_open(local(path), *args)
            class Image:
                def save(self, path, quality):
                    self.path = path; self.quality = quality
                    local(path).write_bytes(b'example JPEG frame')
            clock = SimpleNamespace(ticks_ms=lambda: 0, ticks_diff=lambda a, b: a - b)
            with patch.object(live_recording, 'os', OS()), patch.object(live_recording, 'time', clock), \
                    patch('builtins.open', side_effect=sd_open):
                recorder = live_recording.SDFrameRecorder('/sdcard/car_live/test', {'annotated': True})
                row = recorder.write(Image(), {'capture_timestamp_ms': 100, 'status': 'tracking'})
                recorder.close()
                with self.assertRaises(FileExistsError):
                    live_recording.SDFrameRecorder('/sdcard/car_live/test', {})
            self.assertEqual(row['filename'], 'frame_000001.jpg')
            self.assertTrue((root / 'car_live/test/frame_000001.jpg').is_file())
            self.assertFalse(list(root.rglob('*.partial.jpg')))
            rows = [json.loads(s) for s in (root / 'car_live/test/frames.jsonl').read_text().splitlines()]
            self.assertEqual(rows[0]['capture_timestamp_ms'], 100)
            self.assertEqual(json.loads((root / 'car_live/test/summary.json').read_text())['frames'], 1)

    def test_video_timing_uses_actual_periods_including_clock_wrap_and_slow_saves(self):
        rows = [{'capture_timestamp_ms': 999},
                {'capture_timestamp_ms': 19, 'frame_period_ms': 100},
                {'capture_timestamp_ms': 269, 'frame_period_ms': 250}]
        times, duration = frame_times(rows, 10)
        self.assertEqual(times, [0, .1, .35])
        self.assertAlmostEqual(duration, .525)
        with self.assertRaises(ValueError): frame_times([], 10)

    def test_listener_recovery_waits_for_stop_and_reports_full_traceback(self):
        class Link:
            def __init__(self, lines):
                self.started = False; self.lines = iter(lines); self.commands = []
                self.port = SimpleNamespace(write=self.write, flush=lambda: None)
            def write(self, data):
                if data.startswith(b'exec('): self.started = True
                return len(data)
            def command(self, *args): self.commands.append(args)
            def read(self, size): return b'\x00' * size
            def output(self): return next(self.lines, b'') if self.started else b''
        clock = [0.]
        fake_time = SimpleNamespace(monotonic=lambda: clock[0],
                                   sleep=lambda value: clock.__setitem__(0, clock[0] + value))
        with patch.object(replay, 'time', fake_time):
            link = Link([b'AIC_PICK {"event":"voice_ready"}\n'])
            self.assertTrue(replay.restore_listener(link))
            self.assertIn((0x87, 4), link.commands)
            self.assertIn((0x0D, 4), link.commands)
            link = Link([b'Traceback (most recent call last):\n', b'  File "app.py", line 7\nRuntimeError: full cause\n'])
            with self.assertRaisesRegex(RuntimeError, 'full cause'):
                replay.restore_listener(link)


if __name__ == '__main__': unittest.main()
