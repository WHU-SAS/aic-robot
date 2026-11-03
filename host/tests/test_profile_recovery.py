"""Same-owned-image recovery, real saved captures and full-pipeline negatives.

Saved-image acceptance checks image/control contracts, not surveyed physical
accuracy or K230 timing. Local camera materials are intentionally not in Git.
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
    from device.k230.vision import physical_side, profile_recovery
    from device.k230.vision.candidate_association import associate_fine_candidate

LOCAL=ROOT/'data/car/full_process_tracking_20261103/post_static_anchor_corrections_01'
HELPERS=('edge_profiles.py','separator_profiles.py','profile_recovery.py')
_HOST_BUNDLES=None


def host_bundles():
    """Build real releases in their host environment, away from vision NumPy."""
    global _HOST_BUNDLES
    if _HOST_BUNDLES is None:
        source='''import hashlib,json
from pathlib import Path
from tools.deploy.k230_pick import build_bundle as standalone_bundle
from tools.deploy.k230_robot import build_bundle as robot_bundle
config,tracking,tm=robot_bundle(tracking=True)
unused,standalone,sm=standalone_bundle()
helpers={}
for name in ('edge_profiles.py','separator_profiles.py','profile_recovery.py'):
    original=(Path('device/k230/vision')/name).read_bytes()
    item={}
    for mode,files,manifest in (('tracking',tracking,tm),('standalone',standalone,sm)):
        data=files[name]
        item[mode]={'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest(),
            'matches_source':data==original,'manifest':manifest['files'][name]}
    helpers[name]=item
print(json.dumps({'helpers':helpers,'vision':{key:config['car'][key]
    for key in ('intrinsics','extrinsics','table_plane')}},separators=(',',':')))
'''
        result=subprocess.run([str(ROOT/'scripts/host'),'-c',source],cwd=ROOT,
            text=True,capture_output=True)
        if result.returncode:
            raise AssertionError('Host bundle probe failed: '+result.stderr)
        _HOST_BUNDLES=json.loads(result.stdout)
    return _HOST_BUNDLES


class ProfileRecoveryBundleTests(unittest.TestCase):
    def test_tracking_and_standalone_ship_the_same_three_source_helpers(self):
        bundles=host_bundles()['helpers']
        for name in HELPERS:
            with self.subTest(helper=name):
                source=(ROOT/'device/k230/vision'/name).read_bytes()
                ast.parse(source)
                for mode in ('tracking','standalone'):
                    packaged=bundles[name][mode]
                    self.assertTrue(packaged['matches_source'])
                    self.assertEqual(packaged['bytes'],len(source))
                    self.assertEqual(packaged['sha256'],hashlib.sha256(source).hexdigest())
                    self.assertEqual(packaged['manifest']['bytes'],len(source))
                    self.assertEqual(packaged['manifest']['sha256'],packaged['sha256'])


@unittest.skipUnless(HAS_VISION,'Dedicated vision environment')
class ProfileRecoveryTests(unittest.TestCase):
    def saved_series(self,folder,prefix,count):
        directory=LOCAL/folder
        required=[directory/'config.json']+[directory/('%s_%02d.%s'%(prefix,n,suffix))
            for n in range(1,count+1) for suffix in ('json','jpg')]
        if any(not p.is_file() for p in required):
            self.skipTest('Same-position camera originals are local-only: '+folder)
        config=json.loads((directory/'config.json').read_text())['car']
        rows=[]
        for n in range(1,count+1):
            record=json.loads((directory/('%s_%02d.json'%(prefix,n))).read_text())
            image=cv2.imread(str(directory/record['image_file']))
            self.assertIsNotNone(image)
            rows.append((record,image))
        return config,rows

    def localize(self,config,record,image,**options):
        return list(physical_side.localize_steps(image,record['frame'],record['bbox_px'],
            config['intrinsics'],config['extrinsics'],config['table_plane'],**options))

    def assert_measured_pose(self,config,record,events):
        self.assertEqual(sum(e['stage']=='complete' for e in events),1)
        self.assertEqual(events[-1]['stage'],'complete')
        observation=events[-1]['observation']
        self.assertEqual(observation['status'],'localized',observation)
        for key in ('frame_id','capture_timestamp_ms','clock_domain','timestamp_source','image_size_px'):
            if key in record['frame']:
                self.assertEqual(observation[key],record['frame'][key],key)
        self.assertEqual(observation['reference_point'],'cube_geometric_center')
        self.assertFalse(observation['motion_performed'])
        self.assertFalse(observation['xy_compensation_applied'])
        self.assertNotIn('range_candidate',observation)
        self.assertLessEqual(observation['quality']['reprojection_rms_px'],2.)
        payload=observation['payload']
        self.assertEqual(len(payload['physical_edges']),4)
        for edge in payload['physical_edges']:
            self.assertEqual(edge['sampled_stations'],64)
            self.assertGreaterEqual(edge['support'],45)
            self.assertLessEqual(edge['rms_px'],3.)
        selected=payload['poses'][0]
        table=selected['table_geometry']
        self.assertTrue(table['accepted'])
        self.assertLessEqual(table['normal_deviation_deg'],20.)
        self.assertLessEqual(abs(table['center_height_mm']-10),8.)
        # Independently relate the reported Base centre to the same archived
        # table, so range-only or compensated coordinates cannot impersonate it.
        plane=config['table_plane'];length=math.sqrt(sum(v*v for v in plane['normal']))
        height=(sum(a*b for a,b in zip(plane['normal'],observation['cube_center_Base_mm']))-
            plane['offset_mm'])/length
        self.assertAlmostEqual(height,table['center_height_mm'],places=3)
        if observation.get('profile_recovery') is not None:
            self.assertEqual(observation['profile_recovery']['candidate_limit'],1)
            self.assertEqual(observation['search']['candidate_limit'],1)
            self.assertEqual(observation['search']['measured_seed_count'],1)
        return observation

    def check_series(self,folder,prefix,count):
        config,rows=self.saved_series(folder,prefix,count)
        recovered=0
        for record,image in rows:
            with self.subTest(image=record['image_file']):
                # Exercise this four-measured-edge helper itself. The public
                # selector may now prefer the separately approved cube policy.
                events=list(profile_recovery.recover_steps(image,record['frame'],
                    record['bbox_px'],config['intrinsics'],config['extrinsics'],
                    config['table_plane']))
                observation=self.assert_measured_pose(config,record,events)
                recovered+=observation.get('profile_recovery',{}).get('method')==\
                    'same_frame_single_profile_face'
        self.assertEqual(recovered,count)

    def test_five_original_captures_localize_with_original_capture_and_quality_gates(self):
        self.check_series('live_static_01','static',5)

    def test_twenty_new_captures_localize_with_original_capture_and_quality_gates(self):
        self.check_series('live_static_02','verify',20)

    def test_public_old25_keeps_original_successes_then_uses_explicit_policy_per_result(self):
        counts={'original':0,'cube':0,'profile':0}
        for folder,prefix,count in (('live_static_01','static',5),
                                    ('live_static_02','verify',20)):
            config,rows=self.saved_series(folder,prefix,count)
            for record,image in rows:
                with self.subTest(folder=folder,image=record['image_file']):
                    original=list(physical_side._original_localize_steps(image,
                        record['frame'],record['bbox_px'],config['intrinsics'],
                        config['extrinsics'],config['table_plane']))[-1]['observation']
                    events=self.localize(config,record,image)
                    observation=events[-1]['observation']
                    self.assertEqual(observation['status'],'localized',observation)
                    if original['status']=='localized':
                        counts['original']+=1
                        self.assertEqual(observation['cube_center_Base_mm'],
                                         original['cube_center_Base_mm'])
                        self.assertEqual(observation['payload'],original['payload'])
                        self.assertFalse(any(e['stage']=='recovery_selected' for e in events))
                        self.assert_measured_pose(config,record,events)
                    elif observation['quality'].get('evidence_type')==\
                            'nonplanar_cube_five_corners':
                        counts['cube']+=1
                        self.assertEqual(observation['quality']['reprojection_limit_px'],3.)
                        self.assertLessEqual(observation['quality']['reprojection_rms_px'],3.)
                        self.assertEqual(observation['payload']['status'],'complete_cube_candidate')
                        self.assertNotIn('poses',observation['payload'])
                        self.assertEqual(len(observation['payload']['observed_edges']),6)
                        for edge in observation['payload']['observed_edges']:
                            self.assertEqual(edge['quality']['sampled_stations'],64)
                            self.assertGreaterEqual(edge['quality']['support'],45)
                    else:
                        counts['profile']+=1
                        self.assert_measured_pose(config,record,events)
                    self.assertEqual(observation['frame_id'],record['frame']['frame_id'])
                    self.assertEqual(observation['capture_timestamp_ms'],
                                     record['frame']['capture_timestamp_ms'])
                    association=associate_fine_candidate(observation,record['paired_coarse'])
                    self.assertEqual(association['status'],'matched',association)
        # Direct cube acceptance23/25 is not the public cube count: already
        # accepted original faces return first and retain their old2px result.
        self.assertEqual(sum(counts.values()),25)
        self.assertGreater(counts['original'],0)
        self.assertGreater(counts['cube'],0)

    def test_original_success_never_enters_recovery_or_changes_its_pose(self):
        config,rows=self.saved_series('live_static_01','static',5)
        record,image=rows[0]
        before=list(physical_side._original_localize_steps(image,record['frame'],record['bbox_px'],
            config['intrinsics'],config['extrinsics'],config['table_plane']))[-1]['observation']
        self.assertEqual(before['status'],'localized')
        with patch.object(profile_recovery,'recover_steps',side_effect=AssertionError('unneeded recovery')):
            events=self.localize(config,record,image)
        after=self.assert_measured_pose(config,record,events)
        self.assertEqual(after['cube_center_Base_mm'],before['cube_center_Base_mm'])
        self.assertEqual(after['payload']['face_quad_px'],before['payload']['face_quad_px'])
        self.assertFalse(any(e['stage']=='profile_recovery_start' for e in events))

    def test_moving_range_keeps_original_limited_authority_without_static_recovery(self):
        evidence=LOCAL/'evidence'
        if any(not (evidence/name).is_file() for name in ('manifest.json','config.json')):
            self.skipTest('Original moving capture is local-only')
        manifest=json.loads((evidence/'manifest.json').read_text())
        entry=next((row for row in manifest['frames'] if row['frame']['frame_id']==30),None)
        if entry is None:self.skipTest('Original moving frame30 is local-only')
        path=evidence/entry['image_file']
        if not path.is_file():self.skipTest('Original moving JPEG is local-only')
        config=json.loads((evidence/'config.json').read_text())['car']
        record=dict(frame=entry['frame'],bbox_px=entry['frame']['localization_bbox_px'])
        image=cv2.imread(str(path))
        before=list(physical_side._original_localize_steps(image,record['frame'],record['bbox_px'],
            config['intrinsics'],config['extrinsics'],config['table_plane'],include_range=True))[-1]['observation']
        with patch.object(profile_recovery,'recover_steps',side_effect=AssertionError('static recovery in motion')):
            events=self.localize(config,record,image,include_range=True)
        after=events[-1]['observation']
        self.assertEqual(after['status'],before['status'])
        self.assertEqual(after['range_candidate'],before['range_candidate'])
        self.assertTrue(after['range_candidate']['not_arm_or_parking_authority'])
        self.assertNotIn('cube_center_Base_mm',after)
        self.assertEqual(after['capture_timestamp_ms'],record['frame']['capture_timestamp_ms'])
        self.assertFalse(any(e['stage']=='profile_recovery_start' for e in events))

    def test_recovery_itself_cannot_localize_blue_cards_halos_or_missing_internal_faces(self):
        config=host_bundles()['vision']
        x,y,w,h=500,550,160,240
        black=np.zeros((960,1280,3),np.uint8)
        blue=np.array([255,75,15],np.uint8)
        mask=np.zeros((960,1280),np.float32);mask[y:y+h,x:x+w]=1
        rectangle=black.copy();rectangle[y:y+h,x:x+w]=blue
        smooth=rectangle.copy()
        for row in range(y,y+h):smooth[row,x:x+w]=np.array(blue*(.6+.4*(row-y)/(h-1)),np.uint8)
        halo=(cv2.GaussianBlur(mask,(0,0),6)[:,:,None]*blue).astype(np.uint8)
        ring=np.clip(cv2.GaussianBlur(mask,(0,0),6)-cv2.GaussianBlur(mask,(0,0),1),0,1)
        ring=(np.minimum(1,3*ring)[:,:,None]*blue).astype(np.uint8)
        images=(('black',black),('solid_blue_card',rectangle),('smooth_blue_no_divider',smooth),
                ('blue_halo_no_divider',halo),('halo_ring_black_core',ring))
        for number,(name,image) in enumerate(images):
            with self.subTest(image=name):
                frame=dict(frame_id=1000+number,capture_timestamp_ms=42,timestamp_ms=42,
                    clock_domain='synthetic_test',image_size_px=[1280,960])
                events=list(profile_recovery.recover_steps(image,frame,[x,y,w,h],
                    config['intrinsics'],config['extrinsics'],config['table_plane']))
                self.assertEqual(sum(e['stage']=='complete' for e in events),1)
                result=events[-1]['observation']
                self.assertEqual(result['status'],'no_complete_face',result)
                self.assertNotIn('cube_center_Base_mm',result)
                self.assertNotIn('range_candidate',result)
                self.assertEqual(result['frame_id'],frame['frame_id'])
                self.assertEqual(result['capture_timestamp_ms'],42)
                self.assertEqual(result['search']['usable_sides'],0)


if __name__=='__main__':unittest.main()
