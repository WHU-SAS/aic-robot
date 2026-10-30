"""Stopped-frame recovery preserves pixels, timestamps and strict authority."""
import importlib.util
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from device.k230.vision import static_pick_session as session_module
from device.k230.vision.runtime import localize_precise

HAS_VISION = importlib.util.find_spec('cv2') is not None


class StaticPickSessionTests(unittest.TestCase):
    def setUp(self):
        self.now = 100
        self.config = {
            'intrinsics':{'camera_matrix':[[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]],
                'image_width':1280,'image_height':960,'distortion_coefficients':[0.]*5},
            'extrinsics':{'T_Base_from_Camera_mm':[[1.,0.,0.,0.],[0.,1.,0.,0.],
                [0.,0.,1.,0.],[0.,0.,0.,1.]]},
            'table_plane':{'normal':[0.,0.,1.],'offset_mm':-60.}}
        self.frame = {'frame_id':7,'capture_timestamp_ms':80,
            'clock_domain':'k230_ticks_ms','timestamp_source':'software_snapshot_call',
            'image_size_px':[1280,960]}
        self.data = bytearray(b'\xff\xd8original-jpeg95\xff\xd9')
        self.compress_calls = []
        self.jpeg = SimpleNamespace(bytearray=lambda:self.data)
        self.image = SimpleNamespace(compressed=self.compress)
        self.bgr = object()
        self.session = session_module.StaticPickSession(self.config,lambda:self.now)
        self.raw = {'status':'complete_face_candidate','cube_center_Base_mm':[250.,20.,-50.],
            'cube_center_Camera_mm':[0.,0.,250.],
            'poses':[{'face_reprojection_rms_px':1.}],
            'side_selection':'verified_side','vision_path':'shared_physical_side'}

    def compress(self, quality):
        self.compress_calls.append(quality)
        return self.jpeg

    def solve(self, fn=None):
        def precise(image,frame,intr,ext,plane):
            self.assertIs(image,self.bgr)
            self.assertIs(plane,self.config['table_plane'])
            return localize_precise(image,frame,intr,ext,plane,
                localizer=fn or (lambda *args:self.raw))
        with patch.object(session_module,'_decode',return_value=self.bgr) as decode, \
                patch.object(session_module,'localize_precise',side_effect=precise) as localize:
            self.session.advance()
        return decode, localize

    def test_single_compression_decode_and_solve_retain_original_pixels(self):
        self.session.start(self.image,self.frame,[10,20,30,40])
        self.frame['frame_id'] = 99
        decode,localize = self.solve()
        self.session.advance()
        decode.assert_called_once_with(self.jpeg)
        self.assertEqual(localize.call_count,1)
        self.assertEqual(self.compress_calls,[95])
        self.assertEqual(self.session.latest['frame_id'],7)
        self.assertEqual(self.session.latest['status'],'localized')
        self.assertEqual(self.session.latest['reference_point'],'cube_geometric_center')
        self.assertFalse(self.session.latest['xy_compensation_applied'])
        self.assertIs(self.session.latest['payload'],self.raw)
        self.assertIsNone(self.session.bgr)
        self.assertIsNone(self.session.phase)
        item = self.session.take_evidence('stopped_verify')
        original = bytes(self.data)
        self.data[:] = b'media released'
        self.assertEqual(item['jpeg'],original)
        self.assertIsInstance(item['jpeg'],bytes)
        self.assertEqual(item['frame']['frame_id'],7)
        self.assertIs(item['observation'],self.session.latest)
        self.assertIsNone(self.session.take_evidence('duplicate'))

    def test_completion_clock_does_not_replace_unknown_capture_time(self):
        self.frame.update(capture_timestamp_ms=None,clock_domain=None,timestamp_source='unavailable')
        self.session.start(self.image,self.frame)
        def raw(*args):
            self.now = 3100
            return self.raw
        self.solve(raw)
        self.assertIsNone(self.session.latest['capture_timestamp_ms'])
        self.assertIsNone(self.session.latest['timestamp_ms'])
        self.assertEqual(self.session.latest_ready_ms,3100)
        self.assertEqual(self.session.events[-1],{'stage':'complete','ms':3000})

    def test_table_rejection_is_failed_observation_without_range_authority(self):
        self.session.start(self.image,self.frame)
        def rejected(image,K,D,T,plane):
            self.assertIs(plane,self.config['table_plane'])
            raise ValueError('No face pose consistent with upright cube on calibrated table')
        self.solve(rejected)
        self.assertEqual(self.session.latest['status'],'no_complete_face')
        self.assertNotIn('cube_center_Base_mm',self.session.latest)
        self.assertNotIn('range_candidate',self.session.latest)
        self.assertEqual(self.session.take_evidence('stopped_verify')['observation']['status'],'no_complete_face')

    def test_existing_two_pixel_limit_is_preserved(self):
        self.raw['poses'][0]['face_reprojection_rms_px'] = 2.01
        self.session.start(self.image,self.frame)
        self.solve()
        self.assertEqual(self.session.latest['status'],'no_complete_face')
        self.assertIn('2px',self.session.latest['reason'])
        self.assertNotIn('cube_center_Base_mm',self.session.latest)

    def test_unexpected_solver_error_still_retains_evidence_for_finally(self):
        self.session.start(self.image,self.frame)
        with patch.object(session_module,'_decode',return_value=self.bgr), \
                patch.object(session_module,'localize_precise',side_effect=OSError('decoder resources lost')):
            with self.assertRaises(OSError):
                self.session.advance()
        self.assertIsNone(self.session.bgr)
        item = self.session.take_evidence('stopped_verify')
        self.assertEqual(item['jpeg'],bytes(self.data))
        self.assertEqual(item['observation'],{'status':'processing_interrupted','phase':'geometry'})
        self.assertIsNone(self.session.latest_ready_ms)

    def test_decode_error_does_not_lose_original_or_claim_completion(self):
        self.session.start(self.image,self.frame)
        with patch.object(session_module,'_decode',side_effect=ValueError('corrupt JPEG')), \
                patch.object(session_module,'localize_precise') as localize:
            with self.assertRaisesRegex(ValueError,'corrupt JPEG'):
                self.session.advance()
        localize.assert_not_called()
        item = self.session.take_evidence('stopped_verify')
        self.assertEqual(item['observation'],{'status':'processing_interrupted','phase':'decode'})
        self.assertEqual(item['jpeg'],bytes(self.data))

    def test_evidence_cannot_be_silently_overwritten_on_reuse(self):
        self.session.start(self.image,self.frame)
        with self.assertRaisesRegex(RuntimeError,'Detach'):
            self.session.start(self.image,self.frame)
        self.solve()
        self.session.take_evidence('first')
        self.session.start(self.image,dict(self.frame,frame_id=8))
        self.assertIsNone(self.session.latest)
        self.assertIsNone(self.session.latest_ready_ms)
        self.assertEqual(self.session.take_evidence('second')['frame']['frame_id'],8)

    def test_missing_table_or_moving_range_purpose_is_not_accepted(self):
        for config in ({},dict(self.config,table_plane=None)):
            with self.assertRaisesRegex(ValueError,'table plane'):
                session_module.StaticPickSession(config,lambda:1)
        with self.assertRaisesRegex(ValueError,'stopped precise'):
            session_module.StaticPickSession(self.config,lambda:1,purpose='car_moving_range')

    def test_decoder_consumes_owned_jpeg_without_reencoding(self):
        seen = []
        fake_cv = SimpleNamespace(IMREAD_COLOR=1,
            imdecode=lambda data,flag:seen.append((data,flag)) or self.bgr)
        fake_np = SimpleNamespace(uint8='uint8',frombuffer=lambda value,**kw:value)
        with patch.dict(sys.modules,{'cv2':fake_cv,'numpy':fake_np}):
            result = session_module._decode(self.jpeg)
        self.assertIs(result,self.bgr)
        self.assertEqual(seen,[(self.data,1)])
        self.assertEqual(self.compress_calls,[])

    @unittest.skipUnless(HAS_VISION,'Dedicated vision environment')
    def test_real_default_wrapper_success_runs_fast_once_and_never_legacy(self):
        from device.k230.pick import vision
        from device.k230.vision import physical_side
        self.session.start(self.image,self.frame)
        with patch.object(session_module,'_decode',return_value=self.bgr), \
                patch.object(physical_side,'localize',return_value=self.raw) as fast, \
                patch.object(vision,'_localize_legacy') as legacy:
            self.session.advance()
        self.assertEqual(fast.call_count,1)
        legacy.assert_not_called()
        self.assertIs(fast.call_args[0][0],self.bgr)
        self.assertIs(fast.call_args[0][-1],self.config['table_plane'])
        self.assertEqual(self.session.latest['status'],'localized')

    @unittest.skipUnless(HAS_VISION,'Dedicated vision environment')
    def test_real_default_wrapper_recovery_uses_same_bgr_and_table_once(self):
        from device.k230.pick import vision
        from device.k230.vision import physical_side
        self.session.start(self.image,self.frame)
        with patch.object(session_module,'_decode',return_value=self.bgr) as decode, \
                patch.object(physical_side,'localize',side_effect=ValueError('weak contrast')) as fast, \
                patch.object(vision,'_localize_legacy',return_value=self.raw) as legacy:
            self.session.advance()
        self.assertEqual(decode.call_count,1)
        self.assertEqual(fast.call_count,1)
        self.assertEqual(legacy.call_count,1)
        self.assertIs(fast.call_args[0][0],legacy.call_args[0][0])
        self.assertIs(legacy.call_args[0][-1],self.config['table_plane'])
        self.assertEqual(self.session.latest['status'],'localized')
        self.assertEqual(self.session.latest['payload']['shared_side_recovery']['fast_side_reason'],'weak contrast')


if __name__ == '__main__':
    unittest.main()
