import io
import json
import os
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import Mock, patch

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

    def restore_fixture(self,chunks):
        clock=[0.]
        class Link:
            def __init__(self):
                self.commands=[];self.upload=None;self.closed_at=None;self.uploaded_at=None
                self.port=SimpleNamespace(port='/dev/fake-k230',write=self.write,
                    flush=lambda:None,close=self.close)
            def write(self, data):
                self.upload=data;self.uploaded_at=clock[0]
                return len(data)
            def command(self, *args): self.commands.append(args)
            def close(self):self.closed_at=clock[0]
        class Receiver:
            in_waiting=65536
            def __init__(self):self.chunks=iter(chunks);self.closed=False
            def read(self,size):
                chunk=next(self.chunks,b'')
                if isinstance(chunk,Exception):raise chunk
                return chunk
            def write(self,data):raise AssertionError('Boot observation must never transmit')
            def close(self):self.closed=True
        fake_time=SimpleNamespace(monotonic=lambda:clock[0],
            sleep=lambda value:clock.__setitem__(0,clock[0]+value))
        return Link(),Receiver(),fake_time

    def test_listener_recovery_hard_resets_then_only_receives_new_boot(self):
        link,receiver,clock=self.restore_fixture([b'AIC_ROBOT {"event":"configured"}\n',
            b'AIC_PICK {"event":"voice_',b'ready"}\n'])
        def connect(port):
            self.assertEqual(port,'/dev/fake-k230')
            self.assertIsNotNone(link.closed_at)
            return receiver
        with patch.object(replay,'time',clock),\
             patch('tools.diagnostics.test_k230_motor_uart.stop_previous_script') as stop,\
             patch('host.apps.k230_status.open_receiver',side_effect=connect) as opened:
            lines=replay.restore_listener(link)
        stop.assert_called_once_with(link);opened.assert_called_once()
        self.assertIn(b'machine.reset()',link.upload)
        self.assertNotIn(b'exec(',link.upload)
        self.assertNotIn(b'open(',link.upload)
        self.assertNotEqual(len(link.upload)%64,0)
        self.assertEqual(link.commands,[(0x05,len(link.upload))])
        self.assertGreaterEqual(link.closed_at-link.uploaded_at,1)
        self.assertEqual(lines[-1],'AIC_PICK {"event":"voice_ready"}')
        self.assertTrue(receiver.closed)

    def test_listener_recovery_handles_usb_reenumeration_and_receiver_disconnect(self):
        link,receiver,clock=self.restore_fixture([b'AIC_PICK {"event":"voice_ready"}\n'])
        first=SimpleNamespace(in_waiting=1,read=Mock(side_effect=OSError('USB reset')),close=Mock())
        with patch.object(replay,'time',clock),\
             patch('tools.diagnostics.test_k230_motor_uart.stop_previous_script'),\
             patch('host.apps.k230_status.open_receiver',side_effect=[OSError('missing'),first,receiver]) as opened:
            self.assertTrue(replay.restore_listener(link))
        self.assertEqual(opened.call_count,3)
        first.close.assert_called_once();self.assertTrue(receiver.closed)

    def test_listener_recovery_reports_full_boot_traceback_and_closes_raw_receiver(self):
        link,receiver,clock=self.restore_fixture([b'Traceback (most recent call last):\n',
            b'  File "app.py", line 7\nRuntimeError: full cause\n'])
        with patch.object(replay,'time',clock),\
             patch('tools.diagnostics.test_k230_motor_uart.stop_previous_script'),\
             patch('host.apps.k230_status.open_receiver',return_value=receiver):
            with self.assertRaisesRegex(RuntimeError, 'full cause'):
                replay.restore_listener(link)
        self.assertTrue(receiver.closed)

    def test_listener_recovery_timeout_keeps_boot_log_and_closes_raw_receiver(self):
        link,receiver,clock=self.restore_fixture([b'AIC_ROBOT {"event":"configured"}\n'])
        with patch.object(replay,'time',clock),\
             patch('tools.diagnostics.test_k230_motor_uart.stop_previous_script'),\
             patch('host.apps.k230_status.open_receiver',return_value=receiver):
            with self.assertRaisesRegex(TimeoutError,'(?s)voice_ready.*configured'):
                replay.restore_listener(link)
        self.assertTrue(receiver.closed)


if __name__ == '__main__': unittest.main()
