"""Session timing separates fake queue wait, decode and solver-call intervals."""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch


class PhysicalSideSessionTests(unittest.TestCase):
    def test_original_frame_and_owned_jpeg_keep_separate_queue_and_work_timing(self):
        now = [100]
        data = bytearray(b'original-jpeg95')
        frame = {'frame_id':7, 'capture_timestamp_ms':80}
        result = dict(frame, status='no_complete_face', range_candidate={'distance_only':True})
        def steps(image, captured, bbox, *args, **kwargs):
            self.assertEqual(captured, frame)
            self.assertEqual(bbox, [10,20,30,40])
            self.assertTrue(kwargs['include_range'])
            now[0] += 10
            yield {'stage':'edges'}
            now[0] += 15
            yield {'stage':'complete', 'observation':result}
        path = Path(__file__).resolve().parents[2]/'device/k230/vision/physical_side_session.py'
        spec = importlib.util.spec_from_file_location('timed_physical_side_session', path)
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {'physical_side':SimpleNamespace(localize_steps=steps)}):
            spec.loader.exec_module(module)
        session = module.FineSession({'intrinsics':{}, 'extrinsics':{}, 'table_plane':{},
            'near_parking':{'fine_budget_ms':20}}, lambda:now[0], purpose='car_moving_range')
        def compressed(quality):
            self.assertEqual(quality,95)
            now[0] += 7
            return SimpleNamespace(bytearray=lambda:data)
        def decode(*args):
            now[0] += 30
            return object()
        session.start(SimpleNamespace(compressed=compressed), frame, [10,20,30,40])
        now[0] += 2000  # No advance call: queue time must not become decode time.
        cv = SimpleNamespace(imdecode=decode, IMREAD_COLOR=1)
        np = SimpleNamespace(frombuffer=lambda value,dtype:value, uint8=object())
        with patch.dict(sys.modules, {'cv2':cv, 'numpy':np, 'ulab':None}):
            session.advance()
            now[0] += 100
            session.advance()
        self.assertIs(session.latest,result)
        self.assertEqual(session.latest_ready_ms,2262)
        self.assertEqual(session.timing, {'start_at_ms':100, 'jpeg_ms':7,
            'first_advance_at_ms':2107, 'decode_ms':30, 'first_solver_at_ms':2137,
            'solver_calls':2, 'solver_call_ms':25, 'advance_calls':2, 'ready_at_ms':2262})
        evidence = session.take_evidence('moving')
        data[:] = b'released'
        self.assertEqual(evidence['jpeg'],b'original-jpeg95')
        self.assertEqual(evidence['frame']['capture_timestamp_ms'],80)
        self.assertEqual(evidence['frame']['localization_timing'],session.timing)
        self.assertNotIn('localization_timing',frame)
        self.assertIs(evidence['observation'],result)
        session.advance()
        self.assertEqual(session.timing['advance_calls'],2)


if __name__ == '__main__':
    unittest.main()
