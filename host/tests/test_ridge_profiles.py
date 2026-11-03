"""Real B-step/G-ridge alternatives, same-image ownership and honest typing.

Saved material acceptance is regression evidence, not independent grasp or98%
accuracy evidence. PC bundle imports remain in a separate host process.
"""
import ast
import hashlib
import importlib.util
import json
import math
import subprocess
import unittest
from unittest.mock import patch

from host.project_paths import ROOT

HAS_VISION=importlib.util.find_spec('cv2') is not None
if HAS_VISION:
    import cv2
    import numpy as np
    from device.k230.vision import (ridge_profiles,separator_profiles,
        profile_recovery,edge_profiles,cube_corners,physical_side)
    from device.k230.vision.candidate_association import associate_fine_candidate

LOCAL=ROOT/'data/car/full_process_tracking_20261103/post_static_anchor_corrections_01'
_BUNDLES=None


def host_bundles():
    global _BUNDLES
    if _BUNDLES is None:
        source='''import json,hashlib
from pathlib import Path
from tools.deploy.k230_pick import build_bundle as arm_bundle
from tools.deploy.k230_robot import build_bundle as robot_bundle
unused,standalone,sm=arm_bundle()
unused,tracking,tm=robot_bundle(tracking=True)
result={}
for name in ('ridge_profiles.py','separator_profiles.py','profile_recovery.py'):
    original=(Path('device/k230/vision')/name).read_bytes()
    result[name]={}
    for mode,files,manifest in (('standalone',standalone,sm),('tracking',tracking,tm)):
        data=files[name]
        result[name][mode]={'source_equal':data==original,'bytes':len(data),
            'sha256':hashlib.sha256(data).hexdigest(),'manifest':manifest['files'][name]}
print(json.dumps(result,separators=(',',':')))
'''
        result=subprocess.run([str(ROOT/'scripts/host'),'-c',source],cwd=ROOT,
            text=True,capture_output=True)
        if result.returncode:raise AssertionError(result.stderr)
        _BUNDLES=json.loads(result.stdout)
    return _BUNDLES


class RidgeBundleTests(unittest.TestCase):
    def test_standalone_and_tracking_contain_the_same_source_recovery_modules(self):
        for name,modes in host_bundles().items():
            source=(ROOT/'device/k230/vision'/name).read_bytes()
            ast.parse(source)
            for mode,item in modes.items():
                with self.subTest(module=name,mode=mode):
                    self.assertTrue(item['source_equal'])
                    self.assertEqual(item['bytes'],len(source))
                    self.assertEqual(item['sha256'],hashlib.sha256(source).hexdigest())
                    self.assertEqual(item['manifest']['bytes'],len(source))
                    self.assertEqual(item['manifest']['sha256'],item['sha256'])


@unittest.skipUnless(HAS_VISION,'Dedicated local CV environment')
class RidgeProfileTests(unittest.TestCase):
    def materials(self,folder,prefix,count):
        directory=LOCAL/folder
        paths=[directory/'config.json']+[directory/('%s_%02d.%s'%(prefix,n,suffix))
            for n in range(1,count+1) for suffix in ('json','jpg')]
        if any(not p.is_file() for p in paths):
            self.skipTest('Retained full-camera materials are local-only: '+folder)
        config=json.loads((directory/'config.json').read_text())['car']
        for n in range(1,count+1):
            record=json.loads((directory/('%s_%02d.json'%(prefix,n))).read_text())
            image_path=directory/record['image_file']
            if record.get('image_sha256'):
                self.assertEqual(hashlib.sha256(image_path.read_bytes()).hexdigest(),record['image_sha256'])
            image=cv2.imread(str(image_path));self.assertIsNotNone(image)
            yield config,record,image

    def recover(self,config,record,image,method='auto',**options):
        events=list(profile_recovery.recover_steps(image,record['frame'],record['bbox_px'],
            config['intrinsics'],config['extrinsics'],config['table_plane'],top_method=method,**options))
        self.assertEqual(sum(e['stage']=='complete' for e in events),1)
        return events[-1]['observation']

    def measured_call(self,config,record,image,method='auto',**options):
        with patch.object(separator_profiles,'cue24',wraps=separator_profiles.cue24) as cue,\
             patch.object(separator_profiles,'measure',wraps=separator_profiles.measure) as B,\
             patch.object(ridge_profiles,'measure',wraps=ridge_profiles.measure) as G,\
             patch.object(edge_profiles,'spread_edge',wraps=edge_profiles.spread_edge) as outer:
            observation=self.recover(config,record,image,method,**options)
        return observation,dict(cue=cue.call_count,B=B.call_count,ridge=G.call_count,outer=outer.call_count)

    def assert_same_frame(self,observation,record):
        for key in ('frame_id','capture_timestamp_ms','clock_domain','image_size_px','timestamp_source'):
            if key in record['frame']:self.assertEqual(observation[key],record['frame'][key],key)
        association=associate_fine_candidate(observation,record['paired_coarse'])
        self.assertEqual(association['status'],'matched',association)
        expected=('full_cube_corner_pose'if observation['quality'].get('evidence_type')==
                  'nonplanar_cube_face_back_top'else'full_pose')
        self.assertEqual(association['geometry_capability'],expected)

    def assert_G_face(self,observation,record):
        self.assertEqual(observation['status'],'localized',observation)
        quality,payload=observation['quality'],observation['payload']
        self.assertEqual(quality['evidence_type'],'physical_face_with_G_ridge')
        self.assertEqual(quality['reprojection_limit_px'],2.)
        self.assertLessEqual(quality['reprojection_rms_px'],2.)
        self.assertEqual(quality['separator_evidence'],'G_ridge_bidirectional_prominence')
        self.assertEqual(payload['status'],'complete_face_candidate')
        self.assertEqual(payload['boundary_method'],'four_measured_edges_with_G_ridge')
        self.assertEqual(payload['vision_path'],'shared_physical_face_G_ridge')
        self.assertEqual(observation['profile_recovery']['top_method'],'ridge')
        edges=payload['physical_edges'];self.assertEqual(len(edges),4)
        for edge in edges:
            self.assertEqual(edge['sampled_stations'],64)
            self.assertGreaterEqual(edge['support'],45)
            self.assertLessEqual(edge['rms_px'],3.)
        top=edges[0]
        self.assertEqual(top['appearance_method'],'same_frame_G_ridge')
        self.assertEqual(top['station_evidence'],'G_peak_two_flanks_DN')
        self.assertIs(top['original_RGB8_step_evidence'],False)
        seam=payload['seam_evidence']
        self.assertEqual(seam['evidence_type'],'G_peak_two_flanks_DN')
        self.assertIs(seam['original_RGB8_step_evidence'],False)
        self.assertGreaterEqual(seam['prominence_min_DN'],2.)
        self.assertTrue(payload['poses'][0]['table_geometry']['accepted'])
        self.assertFalse(observation['motion_performed'])
        self.assertFalse(observation['xy_compensation_applied'])
        self.assert_same_frame(observation,record)

    def assert_public_G_geometry(self,observation,record):
        if observation.get('quality',{}).get('evidence_type')=='physical_face_with_G_ridge':
            self.assert_G_face(observation,record)
            return
        self.assertEqual(observation['status'],'localized',observation)
        quality,payload=observation['quality'],observation['payload']
        self.assertEqual(quality['evidence_type'],'nonplanar_cube_face_back_top')
        self.assertEqual(quality['reprojection_limit_px'],3.)
        self.assertLessEqual(quality['reprojection_rms_px'],3.)
        self.assertEqual(payload['evidence_type'],'nonplanar_cube_face_back_top')
        self.assertEqual(payload['boundary_method'],
            'seven_measured_edges_front_and_back_top_corners')
        self.assertEqual(payload['vision_path'],'shared_nonplanar_cube_face_back_top')
        self.assertEqual(payload['status'],'complete_cube_candidate')
        self.assertNotIn('poses',payload)
        self.assertNotIn('face_quad_px',payload)
        pose=payload['cube_pose']
        self.assertEqual(len(pose['image_points_px']),6)
        self.assertEqual(len(pose['object_points_mm']),6)
        self.assertEqual(len(payload['observed_edges']),7)
        self.assertEqual(len(payload['half_corner_errors_px']),12)
        self.assertTrue(all(0<=v<=3 for v in payload['half_corner_errors_px']))
        for edge in payload['observed_edges']:
            self.assertEqual(edge['quality']['sampled_stations'],64)
            self.assertGreaterEqual(edge['quality']['support'],45)
            self.assertLessEqual(edge['quality']['rms_px'],3.)
        self.assertEqual(payload['observed_edges'][0]['name'],'front_top')
        top=payload['observed_edges'][0]['quality']
        self.assertEqual(top['appearance_method'],'same_frame_G_ridge')
        self.assertEqual(top['station_evidence'],'G_peak_two_flanks_DN')
        self.assertIs(top['original_RGB8_step_evidence'],False)
        geometry=pose['table_geometry']
        self.assertTrue(geometry['accepted'])
        self.assertLessEqual(geometry['normal_deviation_deg'],20.)
        self.assertLessEqual(abs(geometry['height_error_mm']),8.)
        self.assertLessEqual(geometry['object_up_deviation_deg'],20.)
        self.assertFalse(observation['motion_performed'])
        self.assertFalse(observation['xy_compensation_applied'])
        self.assertEqual(observation['profile_recovery']['top_method'],'ridge')
        self.assert_same_frame(observation,record)

    def test_three_real_frontal_frames_auto_G_forced_B_reject_and_forced_G_agree(self):
        count=0
        for config,record,image in self.materials('live_static_06_final_timing','fresh',3):
            with self.subTest(image=record['image_file']):
                auto,counts=self.measured_call(config,record,image)
                self.assertEqual(counts,dict(cue=1,B=0,ridge=1,outer=3))
                self.assert_G_face(auto,record)
                rejected,counts=self.measured_call(config,record,image,'B')
                self.assertEqual(counts,dict(cue=1,B=1,ridge=0,outer=0))
                self.assertEqual(rejected['status'],'no_complete_face')
                self.assertNotIn('cube_center_Base_mm',rejected)
                forced,counts=self.measured_call(config,record,image,'ridge')
                self.assertEqual(counts,dict(cue=1,B=0,ridge=1,outer=3))
                self.assertEqual(forced['cube_center_Base_mm'],auto['cube_center_Base_mm'])
                self.assertEqual(forced['payload']['face_quad_px'],auto['payload']['face_quad_px'])
                count+=1
        self.assertEqual(count,3)

    def test_real_frontal_public_fast_path_preserves_typed_G_and_capture_identity(self):
        for config,record,image in self.materials('live_static_06_final_timing','fresh',3):
            with self.subTest(image=record['image_file']):
                events=list(physical_side.localize_steps(image,record['frame'],record['bbox_px'],
                    config['intrinsics'],config['extrinsics'],config['table_plane']))
                self.assertEqual(sum(e['stage']=='complete' for e in events),1)
                self.assert_public_G_geometry(events[-1]['observation'],record)

    def test_real_old25_auto_remains_B_and_matches_forced_B_coordinates(self):
        count=0
        for folder,prefix,n in (('live_static_01','static',5),('live_static_02','verify',20)):
            for config,record,image in self.materials(folder,prefix,n):
                with self.subTest(folder=folder,image=record['image_file']):
                    auto,counts=self.measured_call(config,record,image)
                    self.assertEqual(counts,dict(cue=1,B=1,ridge=0,outer=3))
                    forced,counts=self.measured_call(config,record,image,'B')
                    self.assertEqual(counts,dict(cue=1,B=1,ridge=0,outer=3))
                    self.assertEqual(auto['status'],'localized',auto)
                    self.assertEqual(auto['profile_recovery']['top_method'],'B')
                    self.assertEqual(auto['cube_center_Base_mm'],forced['cube_center_Base_mm'])
                    self.assertEqual(auto['payload']['face_quad_px'],forced['payload']['face_quad_px'])
                    self.assertEqual(auto['payload']['physical_edges'],forced['payload']['physical_edges'])
                    self.assertNotIn('evidence_type',auto['quality'])
                    self.assertLessEqual(auto['quality']['reprojection_rms_px'],2.)
                    self.assert_same_frame(auto,record)
                    count+=1
        self.assertEqual(count,25)

    def test_prepared_and_explicit_same_image_cue_avoid_repeating_the_proposal(self):
        config,record,image=next(self.materials('live_static_06_final_timing','fresh',3))
        prepared=cube_corners.prepare(image,record['bbox_px'])
        with patch.object(separator_profiles,'cue24',wraps=separator_profiles.cue24) as cue,\
             patch.object(separator_profiles,'measure',wraps=separator_profiles.measure) as B,\
             patch.object(ridge_profiles,'measure',wraps=ridge_profiles.measure) as G:
            first=self.recover(config,record,image,prepared=prepared)
            self.assertIn('separator_cue',prepared)
            cached=self.recover(config,record,image,prepared=prepared)
            explicit=self.recover(config,record,image,'ridge',cue=prepared['separator_cue'])
        self.assertEqual(cue.call_count,1)
        self.assertEqual(B.call_count,0)
        self.assertEqual(G.call_count,3)
        for other in (cached,explicit):
            self.assertEqual(other['cube_center_Base_mm'],first['cube_center_Base_mm'])
            self.assertEqual(other['payload']['face_quad_px'],first['payload']['face_quad_px'])

    def test_raw_G_ridge_requires_a_real_peak_and_blue_on_both_flanks(self):
        height,width=160,240
        image=np.zeros((height,width,3),np.uint8)
        image[:,:,0]=220;image[:,:,2]=10
        y=np.arange(height).reshape((-1,1))
        image[:,:,1]=np.array(60+18*np.exp(-((y-80)/3.)**2)+np.zeros((height,width)),np.uint8)
        bbox=[20,20,200,120]
        # Known synthetic search geometry is not support; actual peak/flank
        # pixels must still earn every station through the production fitter.
        cue=([0.,1.,-80.],{'method':'synthetic_search_window_only'})
        def measure(values):
            hsv=cv2.cvtColor(values,cv2.COLOR_BGR2HSV)
            return ridge_profiles.measure(values,hsv,bbox,physical_side._fit_edge,
                physical_side.fit,physical_side.residual,cue=cue)
        edge,details=measure(image)
        self.assertEqual(edge[1]['support'],64)
        self.assertIs(details['original_RGB8_step_evidence'],False)
        monotonic=image.copy();monotonic[:,:,1]=np.array(30+.5*y+np.zeros((height,width)),np.uint8)
        wrong_flank=image.copy();wrong_flank[85:,:,:]=[0,0,220]
        for name,values in (('monotonic_G',monotonic),('red_lower_flank',wrong_flank)):
            with self.subTest(image=name),self.assertRaises(ValueError):measure(values)


if __name__=='__main__':unittest.main()
