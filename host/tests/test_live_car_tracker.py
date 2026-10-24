import ast
import contextlib
import io
import json
from pathlib import Path
import struct
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from host.car.runtime import load
from device.k230.pick import pose
from device.k230.vision import runtime
from tools.diagnostics.live_car_tracker import build_script, main, read_preview, execute_live


class LiveCarTrackerTests(unittest.TestCase):
    def test_native_loop_confirms_loses_and_recovers_without_robot_interfaces(self):
        _, config, geometry = load()
        script = build_script(config, geometry, 800, 100)
        ast.parse(script)
        for token in ('UART(', 'run_pick', 'run_ready', 'MotorLink'):
            self.assertNotIn(token, script)
        clock = SimpleNamespace(ms=0)
        fake_time = SimpleNamespace(ticks_ms=lambda: clock.ms,
            ticks_diff=lambda a, b: a - b, sleep_ms=lambda value: setattr(clock, 'ms', clock.ms + value))
        camera_events = []
        class Blob:
            def rect(self): return (662, 232, 94, 108)
            def pixels(self): return 9000
            def cx(self): return 708
            def cy(self): return 285
        class Image:
            def __init__(self, present): self.present = present; self.detected = False
            def find_blobs(self, *args, **kwargs):
                self.detected = True
                return [Blob()] if self.present else []
            def draw_rectangle(self, *args, **kwargs):
                if not self.detected: raise AssertionError('Annotations preceded detection')
            def draw_line(self, *args, **kwargs): pass
        class Camera:
            def __init__(self, config, preview): self.number = 0
            def open(self): camera_events.append('open')
            def capture(self):
                self.number += 1
                return Image(self.number not in (4, 5)), {
                    'frame_id': self.number, 'capture_timestamp_ms': clock.ms,
                    'image_size_px': [1280, 960], 'clock_domain': 'k230_ticks_ms'}
            def show(self, image): camera_events.append('show')
            def close(self): camera_events.append('close')
        def config_open(path):
            if path == '/sdcard/aic/pick/current.json': return io.StringIO('{"release":"/test_release"}')
            if path == '/test_release/config.json':
                return io.StringIO(json.dumps({'vision': {'camera': {'width': 1280,
                    'height': 960, 'hmirror': False, 'vflip': False}}}))
            raise AssertionError('Unexpected board file access: ' + path)
        output = io.StringIO()
        with patch.dict(sys.modules, {'time': fake_time, 'os': SimpleNamespace(exitpoint=lambda: None),
                'camera': SimpleNamespace(CameraSession=Camera), 'vision_runtime': runtime, 'pose': pose}), \
                patch('builtins.open', side_effect=config_open), contextlib.redirect_stdout(output):
            exec(script, {'__name__': '__main__'})
        rows = [json.loads(line[len('CAR_LIVE '):]) for line in output.getvalue().splitlines()
                if line.startswith('CAR_LIVE ')]
        self.assertEqual([row['status'] for row in rows],
            ['confirming', 'confirming', 'tracking', 'lost', 'no_target', 'confirming', 'confirming', 'tracking'])
        self.assertIsNone(rows[3]['bbox_px'])
        self.assertIsNone(rows[3]['estimated_table_distance_mm'])
        self.assertGreater(rows[2]['estimated_table_distance_mm'], 0)
        self.assertEqual(camera_events.count('open'), 1)
        self.assertEqual(camera_events.count('show'), 8)
        self.assertEqual(camera_events[-1], 'close')

    def test_preview_reads_verified_binary_protocol_and_rejects_unbounded_size(self):
        jpeg = b'\xff\xd8test\xff\xd9'
        class Link:
            def __init__(self, size): self.parts = [struct.pack('<III', 1280, 960, size), jpeg]; self.commands = []
            def command(self, *args): self.commands.append(args)
            def read(self, size): return self.parts.pop(0)
        link = Link(len(jpeg))
        self.assertEqual(read_preview(link), jpeg)
        self.assertEqual(link.commands, [(0x81, 12), (0x82, len(jpeg))])
        with self.assertRaises(RuntimeError): read_preview(Link(8 * 1024 * 1024))

    def test_default_builds_only_and_never_opens_camera_or_server(self):
        with (tempfile.TemporaryDirectory() as folder,
                patch('tools.diagnostics.live_car_tracker.CameraLink', side_effect=AssertionError('No hardware')),
                patch('tools.diagnostics.live_car_tracker.LiveView', side_effect=AssertionError('No server'))):
            self.assertEqual(main(['--output', folder]), 0)
            result = json.loads((Path(folder) / 'result.json').read_text())
            self.assertEqual(result['status'], 'built_no_hardware_access')
            self.assertFalse(result['motion_performed'])

    def test_transport_yields_when_no_telemetry_and_reassembles_partial_lines(self):
        class Link:
            def __init__(self):
                self.chunks = iter([b'', b'', b'CAR_LIVE_RE', b'ADY\r\nEXPORT_DONE\n'])
                self.port = SimpleNamespace(write=lambda data: len(data), flush=lambda: None)
            def output(self): return next(self.chunks)
            def command(self, *args): pass
        with patch('tools.diagnostics.live_car_tracker.time.sleep'):
            rows = list(execute_live(Link(), 'print(1)', 2))
        self.assertEqual(rows, [b'', b'', b'CAR_LIVE_READY'])


if __name__ == '__main__': unittest.main()
