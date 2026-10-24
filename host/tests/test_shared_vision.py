"""Shared vision contracts and camera ownership; hardware is mocked."""
import importlib.util
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import patch
from host.project_paths import ROOT
from device.k230.vision.runtime import localize_precise,track_coarse


def frame(stamp=100):
    return {'frame_id':4,'capture_timestamp_ms':stamp,'clock_domain':'k230_ticks_ms' if stamp is not None else None,
        'timestamp_source':'software_snapshot_call' if stamp is not None else 'unavailable','image_size_px':[1280,960]}


class ObservationTests(unittest.TestCase):
    def setUp(self):
        self.intr={'camera_matrix':[[10.,0.,5.],[0.,20.,6.],[0.,0.,1.]],'image_width':640,'image_height':480,'distortion_coefficients':[0.]*5}
        self.ext={'T_Base_from_Camera_mm':[[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.]]}
        self.raw={'status':'complete_face_candidate','cube_center_Base_mm':[240.,20.,-50.],
            'poses':[{'face_reprojection_rms_px':1.}], 'side_selection':'larger_verified_side'}

    def precise(self,fn,metadata=None):return localize_precise(object(),metadata or frame(),self.intr,self.ext,localizer=fn)

    def test_preserve_legacy_raw_and_uncompensated_center(self):
        seen=[]
        def fn(*args):seen.append(args);return self.raw
        result=self.precise(fn)
        self.assertIs(result['payload'],self.raw)
        self.assertEqual(result['status'],'localized');self.assertEqual(result['coordinate_frame'],'Base')
        self.assertEqual(result['cube_center_Base_mm'],[240.,20.,-50.]);self.assertFalse(result['xy_compensation_applied'])
        self.assertEqual(result['capture_timestamp_ms'],100);self.assertEqual(result['timestamp_ms'],100)
        self.assertEqual(seen[0][1],[[20.,0.,10.],[0.,40.,12.],[0.,0.,1.]])
        self.assertEqual(self.intr['camera_matrix'][0][0],10.)

    def test_saved_frame_missing_time_is_not_refreshed(self):
        result=self.precise(lambda *args:self.raw,frame(None))
        self.assertIsNone(result['timestamp_ms']);self.assertEqual(result['timestamp_source'],'unavailable')

    def test_precise_failure_has_no_stale_coordinate_or_payload(self):
        raw=dict(self.raw,status='no_complete_face',reason='missing edge')
        result=self.precise(lambda *args:raw)
        self.assertEqual(result['status'],'no_complete_face')
        self.assertNotIn('cube_center_Base_mm',result);self.assertNotIn('payload',result)
        self.assertEqual(result['reason'],'missing edge')
        def fail(*args):raise ValueError('bad seam')
        self.assertEqual(self.precise(fail)['reason'],'bad seam')

    def test_existing_reprojection_limit_cannot_be_bypassed(self):
        self.raw['poses'][0]['face_reprojection_rms_px']=2.01
        self.assertEqual(self.precise(lambda *args:self.raw)['status'],'no_complete_face')
        self.assertNotIn('cube_center_Base_mm',self.precise(lambda *args:self.raw))

    def test_coarse_callback_receives_native_image_and_capture_time_once(self):
        im=object();calls=[]
        def detector(image,stamp):
            calls.append((image,stamp));return {'status':'tracking','timestamp_ms':stamp,
                'center_pixel':[650.,600.],'image_midline_error_px':10.,'ground_contact_proxy_Base_mm':[500.,20.,-60.],'confirmations':3}
        result=track_coarse(im,frame(),detector)
        self.assertEqual(calls,[(im,100)]);self.assertEqual(result['mode'],'coarse')
        self.assertNotIn('cube_center_Base_mm',result)
        self.assertEqual(result['ground_contact_proxy_Base_mm'],[500.,20.,-60.])

    def test_lost_result_drops_stale_selected_fields(self):
        result=track_coarse(object(),frame(),lambda *a:{'status':'lost','center_pixel':[4,5],'ground_contact_proxy_Base_mm':[1,2,3]})
        self.assertNotIn('center_pixel',result);self.assertNotIn('ground_contact_proxy_Base_mm',result['payload'])

    def test_coarse_rejects_fake_precise_position_and_wrong_clock(self):
        with self.assertRaises(ValueError):track_coarse(object(),frame(),lambda *a:{'status':'tracking','cube_center_Base_mm':[1,2,3]})
        with self.assertRaises(ValueError):track_coarse(object(),frame(),lambda *a:{'status':'tracking','timestamp_ms':200})
        with self.assertRaises(ValueError):track_coarse(object(),frame(None),lambda *a:{'status':'tracking'})


class CameraOwnershipTests(unittest.TestCase):
    def setUp(self):
        self.events=[];self.clock=10;self.fail=False
        owner=self
        class Sensor:
            RGB565=1
            def __init__(self):owner.events.append('sensor_new')
            def reset(self):owner.events.append('reset')
            def set_framesize(self,**kw):pass
            def set_pixformat(self,*args):pass
            def set_hmirror(self,*args):pass
            def set_vflip(self,*args):pass
            def run(self):
                owner.events.append('run')
                if owner.fail:raise RuntimeError('sensor failure')
            def stop(self):owner.events.append('stop')
            def snapshot(self):owner.clock+=7;return object()
        display=SimpleNamespace(VIRT=0,init=lambda *a,**k:self.events.append('display_init'),deinit=lambda:self.events.append('display_close'))
        media=SimpleNamespace(init=lambda:self.events.append('media_init'),deinit=lambda:self.events.append('media_close'))
        fake_os=SimpleNamespace(EXITPOINT_ENABLE_SLEEP=1,exitpoint=lambda *a:None)
        fake_time=SimpleNamespace(ticks_ms=lambda:self.clock,sleep_ms=lambda *a:None)
        spec=importlib.util.spec_from_file_location('camera_test_shared',ROOT/'device/k230/vision/camera.py')
        self.module=importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules,{'os':fake_os,'time':fake_time,'media.sensor':SimpleNamespace(Sensor=Sensor),'media.display':SimpleNamespace(Display=display),'media.media':SimpleNamespace(MediaManager=media)}):spec.loader.exec_module(self.module)
        self.config={'camera':{'width':1280,'height':960,'hmirror':False,'vflip':False},'sampling':{'warmup_frames':0,'warmup_delay_ms':0}}

    def test_second_owner_cannot_reset_or_close_first(self):
        first=self.module.CameraSession(self.config);second=self.module.CameraSession(self.config);first.open()
        with self.assertRaises(RuntimeError):second.open()
        second.close();self.assertNotIn('media_close',self.events);self.assertEqual(self.events.count('reset'),1)
        first.snapshot();first.close();second.open();second.close()
        self.assertEqual(self.events.count('media_close'),2)

    def test_capture_records_call_time_and_boot_local_unique_id(self):
        camera=self.module.CameraSession(self.config);camera.open();image,a=camera.capture();image,b=camera.capture();camera.close()
        self.assertEqual(a['capture_timestamp_ms'],10);self.assertEqual(b['capture_timestamp_ms'],17)
        self.assertGreater(b['frame_id'],a['frame_id']);self.assertEqual(a['timestamp_source'],'software_snapshot_call')
        with self.assertRaises(RuntimeError):camera.snapshot()

    def test_partial_open_failure_releases_ownership(self):
        camera=self.module.CameraSession(self.config);self.fail=True
        with self.assertRaisesRegex(RuntimeError,'sensor failure'):camera.open()
        self.assertIsNone(self.module.CameraSession._owner);self.assertIn('media_close',self.events)
        self.fail=False;self.module.CameraSession(self.config).open().close()


if __name__=='__main__':unittest.main()
