"""Formal contact capture order, read-only UART behavior and image integrity."""
import ast
import base64
import contextlib
import hashlib
import io
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch
from host.project_paths import ROOT
from host.vision.config import load_vision_config
from tools.capture.collect_fixed_tip_top import build_script, source_without_local_imports, summarize, download_image


class FixedTopCaptureTests(unittest.TestCase):
    def run_board(self, fail=False):
        events=[]
        class Image:
            def copy(self):events.append('copy');return self
            def save(self,path,**kw):Path(path).write_bytes(b'raw-jpeg')
        class Camera:
            def __init__(self,config):pass
            def open(self):events.append('warmup')
            def snapshot(self):events.append('snapshot');return Image()
            def stop(self):events.append('camera_stop')
            def close(self):events.append('camera_close')
        class FPIOA:
            UART1_TXD=3;UART1_RXD=4
            def set_function(self,*args):pass
        class UART:
            UART1=1;EIGHTBITS=8;PARITY_NONE=0;STOPBITS_ONE=1
            def __init__(self,*args,**kwargs):pass
            def deinit(self):events.append('uart_close')
        def read(uart,sid,address,size):
            events.append(('read',sid,address,size))
            if fail and sid==3:return None,0,'no response'
            self.assertIn((address,size),[(56,4),(40,1)])
            return (bytes((sid,8,0,0)) if address==56 else bytes((0,))),0,'OK'
        modules={'ujson':json,'uhashlib':hashlib,
                 'os':types.SimpleNamespace(mkdir=lambda path:None),
                 'time':types.SimpleNamespace(ticks_ms=lambda:1,sleep_ms=lambda ms:None)}
        namespace={'CameraSession':Camera,'UART':UART,'FPIOA':FPIOA,'BAUD':1000000,
                   'read_registers':read,'signed_position':lambda lo,hi:lo+(hi<<8)}
        output=io.StringIO()
        with tempfile.TemporaryDirectory() as tmp,patch.dict(sys.modules,modules),contextlib.redirect_stdout(output):
            source=source_without_local_imports(ROOT/'device/k230/diagnostics/capture_fixed_tip_top.py')
            exec(compile(source,'<fixed-top>','exec'),namespace)
            namespace['capture_fixed_tip_top'](load_vision_config(),str(Path(tmp)/'frame.jpg'))
        lines=output.getvalue().splitlines()
        record=json.loads(next(line[12:] for line in lines if line.startswith('PAIR_RECORD ')))
        self.assertEqual(lines[-1],'EXPORT_DONE')
        return events,record

    def test_photo_is_bracketed_by_six_axis_reads_without_writes(self):
        events,record=self.run_board()
        before=events[:events.index('snapshot')]
        after=events[events.index('copy')+1:]
        self.assertEqual(sum(isinstance(e,tuple) for e in before),36)
        self.assertEqual(sum(isinstance(e,tuple) for e in after),36)
        self.assertEqual(record['status'],'raw_pair_saved_on_camera_sd')
        self.assertEqual(record['camera_capture']['sha256'],hashlib.sha256(b'raw-jpeg').hexdigest())
        self.assertEqual([summarize(record)[str(i)]['span_counts'] for i in range(1,7)],[0]*6)
        self.assertIn('uart_close',events)
        self.assertIn('camera_close',events)
        self.assertFalse(record['torque_writes_sent'])

    def test_missing_servo_feedback_retains_incomplete_record_and_cleans_up(self):
        events,record=self.run_board(fail=True)
        self.assertEqual(record['status'],'incomplete')
        self.assertIn('ID3',record['error'])
        self.assertNotIn('snapshot',events)
        self.assertIn('uart_close',events)
        self.assertIn('camera_close',events)

    def test_bundle_only_has_read_only_servo_helpers(self):
        source=build_script(load_vision_config(),'/sdcard/formal_fixed_tip_top/test.jpg')
        tree=ast.parse(source)
        imports=[n.module for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)]
        self.assertNotIn('uart1_servo_read',imports)
        self.assertNotIn('camera',imports)
        self.assertNotIn('enable_torque',source)
        # Existing packet constructor rejects WRITE/SYNC_WRITE, including instruction 3.
        namespace={}
        helper=ast.parse((ROOT/'device/k230/diagnostics/uart1_servo_read.py').read_text())
        selected=[n for n in helper.body if isinstance(n,ast.FunctionDef) and n.name=='make_packet']
        exec(compile(ast.Module(body=selected,type_ignores=[]),'<read-packet>','exec'),{'IDS':(1,2,3,4,5,6)},namespace)
        with self.assertRaises(ValueError):namespace['make_packet'](2,3,(40,1))

    def test_corrupt_transfer_is_rejected(self):
        class Link:
            def execute(self,*args,**kwargs):
                yield b'IMAGE_DATA 0 '+base64.b64encode(b'wrong')
        with self.assertRaisesRegex(RuntimeError,'SHA256 mismatch'):
            download_image(Link(),{'remote':'/sdcard/pair.jpg','size':5,'sha256':hashlib.sha256(b'right').hexdigest()})


if __name__=='__main__':unittest.main()
