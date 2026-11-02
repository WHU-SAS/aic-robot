"""Stopped-frame cooperative localization with fake decoder/geometry only."""
import copy
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from device.k230.vision import cooperative_static_session as session_module
from device.k230.vision.runtime import localize_precise


class CooperativeStaticPickSessionTests(unittest.TestCase):
    def setUp(self):
        self.now = 100
        self.config = {
            'intrinsics': {'camera_matrix': [[500., 0., 640.], [0., 510., 480.], [0., 0., 1.]],
                           'image_width': 1280, 'image_height': 960,
                           'distortion_coefficients': [0., 0., 0., 0., 0.]},
            'extrinsics': {'T_Base_from_Camera_mm': [[1., 0., 0., 0.], [0., 1., 0., 0.],
                                                   [0., 0., 1., 0.], [0., 0., 0., 1.]]},
            'table_plane': {'normal': [0., 0., 1.], 'offset_mm': -60.},
        }
        self.frame = {'frame_id': 7, 'capture_timestamp_ms': 80,
                      'clock_domain': 'k230_ticks_ms',
                      'timestamp_source': 'software_snapshot_call',
                      'image_size_px': [1280, 960]}
        self.data = bytearray(b'\xff\xd8same-owned-jpeg95\xff\xd9')
        self.jpeg = SimpleNamespace(bytearray=lambda: self.data)
        self.image = SimpleNamespace(compressed=self.compress)
        self.bgr = object()
        self.compress_calls = []
        self.trace = []
        self.received = []
        self.closed_generators = []
        self.progress = [(5, 'segment'), (30, 'side_geometry')]
        self.complete_ms = 7
        self.include_complete = True
        self.raw = {'status': 'complete_face_candidate',
                    'cube_center_Base_mm': [250., 20., -50.],
                    'cube_center_Camera_mm': [0., 0., 250.],
                    'poses': [{'face_reprojection_rms_px': 1.}],
                    'side_selection': 'verified_side',
                    'vision_path': 'shared_physical_side'}
        self.session = self.make_session()
        self.actual_decode = session_module._decode
        self.decode = patch.object(session_module, '_decode', side_effect=self.decode_image).start()
        self.generator = patch.object(session_module, 'localize_steps', side_effect=self.raw_steps).start()
        self.precise = patch.object(session_module, 'localize_precise', wraps=localize_precise).start()
        self.addCleanup(patch.stopall)

    def make_session(self, config=None):
        return session_module.CooperativeStaticPickSession(config or self.config, lambda: self.now)

    def compress(self, quality):
        self.compress_calls.append(quality)
        self.now += 2
        return self.jpeg

    def decode_image(self, jpeg):
        self.assertIs(jpeg, self.jpeg)
        self.trace.append('decode')
        self.now += 3
        return self.bgr

    def raw_steps(self, *args):
        token = len(self.received)
        self.received.append(args)
        try:
            for step in self.progress:
                if isinstance(step, BaseException):
                    raise step
                delay, stage = step
                self.now += delay
                self.trace.append(stage)
                yield {'stage': stage}
            if self.include_complete:
                self.now += self.complete_ms
                self.trace.append('complete')
                yield {'stage': 'complete', 'result': self.raw}
        finally:
            self.closed_generators.append(token)

    def finish(self):
        for _ in range(20):
            if self.session.phase is None:
                return
            self.session.advance()
        self.fail('cooperative session did not finish')

    def test_decode_and_each_geometry_step_yield_to_the_owner(self):
        self.session.start(self.image, self.frame)
        self.assertEqual(self.trace, [])
        self.session.advance()
        self.assertEqual(self.trace, ['decode'])
        self.assertEqual(self.session.phase, 'geometry')
        self.assertIsNone(self.session.latest)
        self.session.advance()
        self.assertEqual(self.trace, ['decode', 'segment'])
        self.assertIsNone(self.session.latest)
        self.session.advance()
        self.assertEqual(self.trace, ['decode', 'segment', 'side_geometry'])
        self.assertIsNone(self.session.latest)
        self.session.advance()
        self.assertEqual(self.trace, ['decode', 'segment', 'side_geometry', 'complete'])
        self.assertEqual(self.session.latest['status'], 'localized')
        self.assertIsNone(self.session.phase)
        self.assertIsNone(self.session.bgr)
        self.assertIsNone(self.session.steps)
        self.assertEqual(self.closed_generators, [0])
        self.precise.assert_called_once()

    def test_one_jpeg95_full_frame_bgr_and_bbox_only_provenance(self):
        bbox = [10, 20, 30, 40]
        self.session.start(self.image, self.frame, bbox)
        self.frame['frame_id'] = 99
        self.frame['image_size_px'][0] = 640
        bbox[0] = 999
        self.finish()
        self.assertEqual(self.compress_calls, [95])
        self.decode.assert_called_once_with(self.jpeg)
        self.assertEqual(len(self.received), 1)
        image, K, D, T, plane = self.received[0]
        self.assertIs(image, self.bgr)
        self.assertEqual(K, self.config['intrinsics']['camera_matrix'])
        self.assertEqual(D, self.config['intrinsics']['distortion_coefficients'])
        self.assertEqual(T, self.config['extrinsics']['T_Base_from_Camera_mm'])
        self.assertIs(plane, self.config['table_plane'])
        self.assertEqual(self.session.bbox, [10, 20, 30, 40])
        self.assertEqual(self.session.latest['frame_id'], 7)
        self.assertEqual(self.session.latest['image_size_px'], [1280, 960])
        self.assertEqual(self.session.latest['reference_point'], 'cube_geometric_center')
        self.assertFalse(self.session.latest['xy_compensation_applied'])
        self.assertIs(self.session.latest['payload'], self.raw)

    def test_unknown_capture_time_is_never_replaced_with_finish_time(self):
        self.frame.update(capture_timestamp_ms=None, clock_domain=None, timestamp_source='unavailable')
        self.session.start(self.image, self.frame)
        self.finish()
        self.assertIsNone(self.session.latest['capture_timestamp_ms'])
        self.assertIsNone(self.session.latest['timestamp_ms'])
        self.assertEqual(self.session.latest_ready_ms, self.now)
        self.assertGreater(self.now, 100)

    def test_events_measure_each_step_and_soft_overrun_does_not_abort(self):
        self.session.start(self.image, self.frame)
        self.finish()
        self.assertEqual([(e['stage'], e['ms']) for e in self.session.events],
                         [('jpeg95', 2), ('decode', 3), ('segment', 5),
                          ('side_geometry', 30), ('complete', 7)])
        for event in self.session.events:
            self.assertEqual(event['soft_budget_ms'], 20)
            self.assertEqual(event['over_budget'], event['ms'] > 20)
        self.assertEqual(self.session.latest['status'], 'localized')

    def test_budget_precedence_tracking_then_near_then_default(self):
        for tracking, near, expected in ((8, 12, 8), (None, 12, 12), (None, None, 20)):
            with self.subTest(tracking=tracking, near=near):
                config = copy.deepcopy(self.config)
                if tracking is not None:
                    config['tracking_task'] = {'fine_budget_ms': tracking}
                if near is not None:
                    config['near_parking'] = {'fine_budget_ms': near}
                session = self.make_session(config)
                session.start(self.image, self.frame)
                session.advance()
                self.assertTrue(all(e['soft_budget_ms'] == expected for e in session.events))
                session.take_evidence('budget')

    def test_budget_boundary_is_inclusive_and_still_reports_each_step(self):
        config = dict(self.config, tracking_task={'fine_budget_ms': 5})
        self.session = self.make_session(config)
        self.session.start(self.image, self.frame)
        self.finish()
        indexed = {event['stage']: event for event in self.session.events}
        self.assertFalse(indexed['segment']['over_budget'])
        self.assertTrue(indexed['side_geometry']['over_budget'])
        self.assertEqual(self.session.latest['status'], 'localized')

    def test_success_evidence_owns_bytes_and_retains_exact_original_observation(self):
        self.session.start(self.image, self.frame)
        self.finish()
        observation = self.session.latest
        before = bytes(self.data)
        item = self.session.take_evidence('stopped_verify')
        self.data[:] = b'media released'
        self.assertEqual(item['jpeg'], before)
        self.assertIsInstance(item['jpeg'], bytes)
        self.assertEqual(item['frame']['frame_id'], 7)
        self.assertIs(item['observation'], observation)
        self.assertIs(self.session.latest, observation)
        self.assertEqual(item['events'], self.session.events)
        self.assertIsNone(self.session.take_evidence('duplicate'))

    def test_two_pixel_limit_is_still_enforced_after_generator_completes(self):
        self.raw['poses'][0]['face_reprojection_rms_px'] = 2.01
        self.session.start(self.image, self.frame)
        self.finish()
        result = self.session.latest
        self.assertEqual(result['status'], 'no_complete_face')
        self.assertIn('2px', result['reason'])
        self.assertNotIn('cube_center_Base_mm', result)
        self.assertNotIn('range_candidate', result)
        self.assertEqual(self.session.take_evidence('rejected')['observation']['status'], 'no_complete_face')

    def test_original_geometry_rejection_is_a_failed_precise_observation(self):
        self.progress = [ValueError('No face pose consistent with upright cube on calibrated table')]
        self.session.start(self.image, self.frame)
        self.finish()
        result = self.session.latest
        self.assertEqual(result['status'], 'no_complete_face')
        self.assertIn('calibrated table', result['reason'])
        self.assertNotIn('cube_center_Base_mm', result)
        self.assertNotIn('range_candidate', result)
        self.assertEqual(result['frame_id'], 7)
        self.assertEqual(result['capture_timestamp_ms'], 80)
        self.assertEqual(self.session.latest_ready_ms, self.now)
        self.assertIsNone(self.session.bgr)
        self.assertIsNone(self.session.steps)

    def test_cancel_before_decode_is_idempotent_and_keeps_original_evidence(self):
        self.session.start(self.image, self.frame)
        self.session.cancel()
        self.session.cancel()
        self.session.advance()
        self.decode.assert_not_called()
        self.assertIsNone(self.session.phase)
        self.assertIsNone(self.session.latest)
        self.assertIsNone(self.session.latest_ready_ms)
        self.assertIsNone(self.session.bgr)
        self.assertIsNone(self.session.steps)
        item = self.session.take_evidence('cancelled')
        self.assertEqual(item['jpeg'], bytes(self.data))
        self.assertEqual(item['observation']['status'], 'processing_interrupted')
        self.assertTrue(item['observation']['cancelled'])

    def test_cancel_closes_started_generator_and_releases_bgr(self):
        self.session.start(self.image, self.frame)
        self.session.advance()
        self.session.advance()
        old = self.session.steps
        self.assertEqual(self.closed_generators, [])
        self.session.cancel()
        self.session.cancel()
        self.assertEqual(self.closed_generators, [0])
        with self.assertRaises(StopIteration):
            next(old)
        self.assertIsNone(self.session.steps)
        self.assertIsNone(self.session.bgr)
        self.assertIsNone(self.session.latest)
        self.assertIsNone(self.session.latest_ready_ms)
        self.assertIs(self.session.jpeg, self.jpeg)
        self.precise.assert_not_called()

    def test_take_evidence_during_processing_cancels_and_detaches(self):
        self.session.start(self.image, self.frame)
        self.session.advance()
        self.session.advance()
        item = self.session.take_evidence('target_moved')
        self.assertEqual(self.closed_generators, [0])
        self.assertEqual(item['observation']['status'], 'processing_interrupted')
        self.assertEqual(item['observation']['phase'], 'geometry')
        self.assertTrue(item['observation']['cancelled'])
        self.assertEqual(item['jpeg'], bytes(self.data))
        self.assertIsNone(self.session.jpeg)
        self.assertIsNone(self.session.phase)
        self.session.advance()
        self.assertIsNone(self.session.latest)
        self.precise.assert_not_called()

    def test_undetached_evidence_cannot_be_overwritten_even_after_cancel_or_complete(self):
        self.session.start(self.image, self.frame)
        with self.assertRaisesRegex(RuntimeError, 'Detach'):
            self.session.start(self.image, dict(self.frame, frame_id=8))
        self.session.cancel()
        with self.assertRaisesRegex(RuntimeError, 'Detach'):
            self.session.start(self.image, dict(self.frame, frame_id=8))
        self.session.take_evidence('cancelled')
        self.session.start(self.image, dict(self.frame, frame_id=8))
        self.finish()
        with self.assertRaisesRegex(RuntimeError, 'Detach'):
            self.session.start(self.image, dict(self.frame, frame_id=9))

    def test_restarted_job_cannot_publish_cancelled_frames_result(self):
        self.session.start(self.image, self.frame)
        self.session.advance()
        self.session.advance()
        old = self.session.steps
        self.session.take_evidence('old')
        self.session.start(self.image, dict(self.frame, frame_id=8, capture_timestamp_ms=90))
        self.assertIsNone(self.session.latest)
        self.assertIsNone(self.session.latest_ready_ms)
        with self.assertRaises(StopIteration):
            next(old)
        self.finish()
        self.assertEqual(self.session.latest['frame_id'], 8)
        self.assertEqual(self.session.latest['capture_timestamp_ms'], 90)
        item = self.session.take_evidence('new')
        self.assertEqual(item['frame']['frame_id'], 8)
        self.assertEqual(self.compress_calls, [95, 95])

    def test_unexpected_generator_error_keeps_evidence_and_is_reraised(self):
        self.progress = [(5, 'segment'), OSError('geometry resources lost')]
        self.session.start(self.image, self.frame)
        self.session.advance()
        self.session.advance()
        with self.assertRaisesRegex(OSError, 'resources lost'):
            self.session.advance()
        self.assertIsNone(self.session.bgr)
        self.assertIsNone(self.session.steps)
        self.assertIsNone(self.session.latest)
        self.assertIsNone(self.session.latest_ready_ms)
        item = self.session.take_evidence('interrupted')
        self.assertEqual(item['jpeg'], bytes(self.data))
        self.assertEqual(item['observation']['status'], 'processing_interrupted')
        self.assertEqual(item['frame']['frame_id'], 7)

    def test_decode_valueerror_is_not_misreported_as_geometry_rejection(self):
        self.session.start(self.image, self.frame)
        self.decode.side_effect = ValueError('corrupt JPEG')
        with self.assertRaisesRegex(ValueError, 'corrupt JPEG'):
            self.session.advance()
        self.generator.assert_not_called()
        self.precise.assert_not_called()
        self.assertIsNone(self.session.latest)
        self.assertIsNone(self.session.latest_ready_ms)
        item = self.session.take_evidence('decode_error')
        self.assertEqual(item['observation']['status'], 'processing_interrupted')
        self.assertEqual(item['observation']['phase'], 'decode')
        self.assertEqual(item['jpeg'], bytes(self.data))

    def test_generator_without_complete_result_is_an_error_not_a_failed_detection(self):
        self.include_complete = False
        self.progress = [(5, 'segment')]
        self.session.start(self.image, self.frame)
        self.session.advance()
        self.session.advance()
        with self.assertRaisesRegex(RuntimeError, 'without a result'):
            self.session.advance()
        self.assertIsNone(self.session.latest)
        self.assertIsNone(self.session.latest_ready_ms)
        self.assertIsNone(self.session.bgr)
        self.assertIsNone(self.session.steps)
        item = self.session.take_evidence('broken_generator')
        self.assertEqual(item['jpeg'], bytes(self.data))
        self.assertEqual(item['observation']['status'], 'processing_interrupted')

    def test_no_extra_work_is_done_after_complete_or_detach(self):
        self.session.start(self.image, self.frame)
        self.finish()
        trace = list(self.trace)
        events = list(self.session.events)
        self.session.advance()
        self.session.advance()
        self.assertEqual(self.trace, trace)
        self.assertEqual(self.session.events, events)
        self.session.take_evidence('done')
        self.session.advance()
        self.assertEqual(self.trace, trace)

    def test_calibrated_table_plane_is_required(self):
        for config in ({}, dict(self.config, table_plane=None)):
            with self.subTest(config=config):
                with self.assertRaisesRegex(ValueError, 'table plane'):
                    session_module.CooperativeStaticPickSession(config, lambda: self.now)

    def test_decoder_consumes_jpeg_bytes_without_reencoding(self):
        # Exercise the actual helper using tiny module stubs, not OpenCV.
        seen = []
        fake_cv = SimpleNamespace(IMREAD_COLOR=1,
            imdecode=lambda data, flag: seen.append((data, flag)) or self.bgr)
        fake_np = SimpleNamespace(uint8='uint8', frombuffer=lambda value, **kw: value)
        with patch.dict(sys.modules, {'cv2': fake_cv, 'numpy': fake_np,
                                     'ulab': SimpleNamespace(numpy=fake_np)}):
            result = self.actual_decode(self.jpeg)
        self.assertIs(result, self.bgr)
        self.assertEqual(seen, [(self.data, 1)])
        self.assertEqual(self.compress_calls, [])


if __name__ == '__main__':
    unittest.main()
