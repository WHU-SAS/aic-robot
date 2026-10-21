"""Vision extraction boundaries, temporal evidence and the board adapter without hardware."""
import ast
import contextlib
import copy
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch
from host.project_paths import ROOT
from host.transport.board_vision_source import build_capture_script
from host.vision.config import load_vision_config

# Load pure board modules without importing any camera/native media module.
core_path = ROOT / 'device/k230/vision/blue_face_core.py'
spec = importlib.util.spec_from_file_location('blue_face_core', core_path)
core = importlib.util.module_from_spec(spec)
spec.loader.exec_module(core)
spec = importlib.util.spec_from_file_location('blue_detector', core_path.with_name('blue_detector.py'))
blue = importlib.util.module_from_spec(spec)
with patch.dict(sys.modules, {'blue_face_core': core}):
    spec.loader.exec_module(blue)


class TemporalTests(unittest.TestCase):
    def face(self, shift=0, source='native_rects', kind='unknown'):
        return {'kind':kind, 'detector':source,
                'quad_px':[[100+shift,500],[160+shift,500],[160+shift,560],[100+shift,560]]}

    def test_two_frames_cannot_confirm(self):
        f={'candidates':[self.face()]}
        self.assertFalse(blue.matched_three_frames([copy.deepcopy(f), copy.deepcopy(f)]))

    def test_three_jittered_frames_confirm(self):
        frames=[{'candidates':[self.face(n)]} for n in (0,1,2)]
        self.assertTrue(blue.matched_three_frames(frames))

    def test_moving_or_mixed_physical_faces_do_not_confirm(self):
        frames=[{'candidates':[self.face(n)]} for n in (0,1,12)]
        self.assertFalse(blue.matched_three_frames(frames))
        frames=[{'candidates':[self.face(0,source)]} for source in ('native_rects','cube_edges','fitted_edges')]
        self.assertFalse(blue.matched_three_frames(frames))

    def test_unverified_legacy_side_is_not_confirmation(self):
        frames=[{'candidates':[self.face(source='fitted_edges',kind='side')]} for _ in range(3)]
        self.assertFalse(blue.matched_three_frames(frames))

    def test_cube_outline_order_can_rotate_without_losing_confirmation(self):
        q=[[100,500],[130,480],[160,500],[160,540],[130,560],[100,540]]
        frames=[{'cube_outlines':[{'corners_px':p}], 'candidates':[]}
                for p in (q,q[2:]+q[:2],list(reversed(q)))]
        self.assertTrue(blue.matched_three_frames(frames))


class ExtractionTests(unittest.TestCase):
    def test_validated_core_is_byte_identical_to_v0(self):
        baseline=json.loads((ROOT/'demos/v0_blue_pick/baseline.json').read_text())
        self.assertEqual(hashlib.sha256(core_path.read_bytes()).hexdigest(),baseline['source_core_sha256'])

    def test_no_blue_does_not_fabricate_a_target(self):
        image=types.SimpleNamespace(find_blobs=lambda *args,**kwargs: [])
        result=blue.BlueCubeDetector(load_vision_config()).detect_frame(image)
        self.assertEqual(result,{'status':'no_blue_cube','candidates':[]})

    def test_clipped_color_component_is_not_accepted_as_a_face(self):
        blob=types.SimpleNamespace(rect=lambda:(0,490,100,100), pixels=lambda:7000)
        image=types.SimpleNamespace(find_blobs=lambda *args,**kwargs:[blob],get_pixel=lambda x,y:(0,0,200))
        component={'bbox_px':[0,490,100,100],'sampled_pixels':7000,'clipped':True,'clipped_sides':['left']}
        with patch.object(blue,'component_boxes',return_value=[component]), patch.object(blue,'extract') as extract:
            result=blue.BlueCubeDetector(load_vision_config()).detect_frame(image)
        self.assertEqual(result['status'],'no_complete_HSV_face')
        extract.assert_not_called()

    def test_whole_unknown_silhouette_is_not_a_valid_square_face(self):
        blob=types.SimpleNamespace(rect=lambda:(100,490,100,100), pixels=lambda:7000)
        image=types.SimpleNamespace(find_blobs=lambda *args,**kwargs:[blob],get_pixel=lambda x,y:(0,0,200))
        component={'bbox_px':[100,490,100,100],'sampled_pixels':7000,'clipped':False}
        unknown={'kind':'unknown','detector':'fitted_edges','quad_px':[[100,490],[200,490],[200,590],[100,590]]}
        with patch.object(blue,'component_boxes',return_value=[component]), \
             patch.object(blue,'extract',return_value={'candidates':[unknown]}), \
             patch.object(blue,'native_rect_faces',return_value=([],{'available':True})):
            result=blue.BlueCubeDetector(load_vision_config()).detect_frame(image)
        self.assertFalse(result['candidates'])
        self.assertEqual(result['status'],'no_complete_HSV_face')


class BundleTests(unittest.TestCase):
    def run_bundle(self, fail_on_media=False):
        events=[]
        class Image:
            def find_blobs(self,*args,**kwargs):
                events.append(('roi',kwargs['roi']))
                return []
        class Sensor:
            RGB565=1
            def reset(self):events.append('reset')
            def set_framesize(self,**kw):events.append(('size',kw))
            def set_pixformat(self,*args):pass
            def set_hmirror(self,*args):pass
            def set_vflip(self,*args):pass
            def run(self):events.append('run')
            def snapshot(self):events.append('snapshot');return Image()
            def stop(self):events.append('stop')
        class Display:
            VIRT=1
            @staticmethod
            def init(*args,**kwargs):events.append('display_init')
            @staticmethod
            def deinit():events.append('display_close')
        class Media:
            @staticmethod
            def init():
                events.append('media_init')
                if fail_on_media:raise RuntimeError('media unavailable')
            @staticmethod
            def deinit():events.append('media_close')
        fake_time=types.SimpleNamespace(ticks_ms=lambda:0,ticks_diff=lambda a,b:a-b,sleep_ms=lambda n:None)
        fake_os=types.SimpleNamespace(exitpoint=lambda *args:None,EXITPOINT_ENABLE_SLEEP=1)
        modules={'media':types.ModuleType('media'),
                 'media.sensor':types.SimpleNamespace(Sensor=Sensor),
                 'media.display':types.SimpleNamespace(Display=Display),
                 'media.media':types.SimpleNamespace(MediaManager=Media),
                 'ujson':json,'uhashlib':hashlib,'time':fake_time,'os':fake_os}
        output=io.StringIO()
        with patch.dict(sys.modules,modules), patch.object(sys,'print_exception',lambda exc:None,create=True), contextlib.redirect_stdout(output):
            exec(compile(build_capture_script(),'<board bundle>','exec'),{'__name__':'board_bundle'})
        return events,output.getvalue().splitlines()

    def test_adapter_preserves_hashed_protocol_and_bounded_empty_burst(self):
        events,lines=self.run_bundle()
        data=b''.join(line.split(' ',2)[2].encode() for line in lines if line.startswith('FEATURES_PART '))
        digest=next(line.split() for line in lines if line.startswith('FEATURES_HASH '))
        self.assertEqual(int(digest[1]),len(data))
        self.assertEqual(digest[2],hashlib.sha256(data).hexdigest())
        report=json.loads(data)
        self.assertEqual(report['schema'],'aic.blue_faces.v1')
        self.assertEqual(report['frame_count'],9)
        self.assertTrue(all(f['status']=='no_blue_cube' for f in report['frames']))
        self.assertEqual(events.count('snapshot'),39)
        self.assertEqual(events.count('stop'),1)
        self.assertEqual(events.count('media_close'),1)
        self.assertEqual(lines[-1],'EXPORT_DONE')
        self.assertFalse(report['robot_commands_sent'])

    def test_partial_camera_initialization_is_cleaned_up(self):
        events,lines=self.run_bundle(fail_on_media=True)
        self.assertIn('stop',events)
        self.assertIn('display_close',events)
        self.assertNotIn('media_close',events)
        self.assertTrue(any(line.startswith('DETECTION_ERROR ') for line in lines))
        self.assertEqual(lines[-1],'EXPORT_DONE')


if __name__=='__main__':unittest.main()
