"""Same-frame prominence-band recovery, without weaker evidence gates.

Saved photos are development regressions, not surveyed corners, independent
98% acceptance or real grasp evidence. Original successful G pixels stay exact.
"""
import importlib.util
import json
import math
import unittest
from unittest.mock import patch

from host.project_paths import ROOT

HAS_VISION=importlib.util.find_spec('cv2') is not None
if HAS_VISION:
    import cv2
    import numpy as np
    from device.k230.vision import (ridge_profiles as g,physical_side as p,
        separator_profiles as s,cube_corners,profile_recovery)

HISTORY=ROOT/'data/car/full_process_tracking_20261103/post_static_anchor_corrections_01'
BATCH=ROOT/'data/vision/lit_validation/20261104_100443_212133'


@unittest.skipUnless(HAS_VISION,'Dedicated local CV environment')
class RidgeProfileConsensusTests(unittest.TestCase):
    def prepare(self,folder,prefix,n):
        record=folder/('%s_%02d.json'%(prefix,n))
        if not record.is_file():self.skipTest('Local-only retained original unavailable')
        meta=json.loads(record.read_text());image=cv2.imread(str(folder/meta['image_file']))
        if image is None:self.skipTest('Local-only retained JPEG unavailable')
        prep=cube_corners.prepare(image,meta['bbox_px'])
        cue=s.cue24(prep['smooth'],prep['hsv'],prep['local_bbox'],p.fit,p.residual)
        return image,meta,prep,cue

    def frozen(self):
        source=ROOT/'data/vision/lit_validation_20261104_pc_preparation_01/frozen_bundle/ridge_profiles.py'
        if not source.is_file():self.skipTest('Frozen local original G module unavailable')
        spec=importlib.util.spec_from_file_location('_frozen_original_G',source)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        return module

    def test_original_success_skips_joint_sampling_and_keeps_exact_pixels(self):
        h,w=160,240;y=np.arange(h).reshape((-1,1))
        im=np.zeros((h,w,3),np.uint8);im[:,:,0]=220;im[:,:,2]=10
        im[:,:,1]=np.array(60+18*np.exp(-((y-80)/3.)**2)+np.zeros((h,w)),np.uint8)
        hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV);cue=([0.,1.,-80.],{'method':'synthetic_window_only'})
        args=(im,hsv,[20,20,200,120],p._fit_edge,p.fit,p.residual)
        with patch.object(g,'_joint_ridge',side_effect=AssertionError('Original success must return unchanged')):
            edge,detail=g.measure(*args,cue=cue,audit=True)
        self.assertEqual(edge[1]['support'],64)
        self.assertNotIn('peak_selection',detail)
        self.assertNotIn('peak_selection',edge[1])
        self.assertTrue(all(v['supported']for v in detail['station_rows']))

    def test_no_bidirectional_peak_or_wrong_flank_cannot_gain_support(self):
        h,w=160,240;y=np.arange(h).reshape((-1,1))
        im=np.zeros((h,w,3),np.uint8);im[:,:,0]=220;im[:,:,2]=10
        im[:,:,1]=np.array(60+18*np.exp(-((y-80)/3.)**2)+np.zeros((h,w)),np.uint8)
        monotonic=im.copy();monotonic[:,:,1]=np.array(30+.5*y+np.zeros((h,w)),np.uint8)
        red=im.copy();red[85:]=[0,0,220]
        cue=([0.,1.,-80.],{'method':'synthetic_window_only'})
        for name,values in(('monotonic',monotonic),('red_flank',red)):
            with self.subTest(case=name),self.assertRaises(ValueError):
                g.measure(values,cv2.cvtColor(values,cv2.COLOR_BGR2HSV),[20,20,200,120],
                    p._fit_edge,p.fit,p.residual,cue=cue)

    def test_two_real_failed_lines_require_actual_2DN_blue_flanks_and_original_fitter(self):
        old=self.frozen()
        for n,count in((1,63),(4,45)):
            with self.subTest(frame=n):
                unused,meta,prep,cue=self.prepare(BATCH,'frame',n)
                args=(prep['smooth'],prep['hsv'],prep['local_bbox'],p._fit_edge,p.fit,p.residual)
                with self.assertRaises(ValueError):old.measure(*args,cue=cue)
                edge,detail=g.measure(*args,cue=cue,audit=True)
                self.assertEqual(edge[1]['support'],count)
                self.assertLessEqual(edge[1]['rms_px'],3.)
                self.assertEqual(detail['joint_model_count'],25)
                self.assertEqual(detail['peak_selection'],'joint_flank_prominence_line_consensus')
                self.assertEqual(detail['minimum_prominence_DN'],2.)
                self.assertIs(detail['original_RGB8_step_evidence'],False)
                self.assertGreaterEqual(max(detail['station_ids'])-min(detail['station_ids']),50)
                self.assertEqual(len(set(detail['station_ids'])),len(detail['station_ids']))
                fitted=p._fit_edge(detail['measured_points'],detail['station_ids'])
                self.assertEqual(fitted[0],edge[0])
                self.assertEqual(fitted[2],edge[2])
                line=cue[0];slope=-line[0]/line[1];length=math.sqrt(1+slope*slope)
                normal=[-slope/length,1/length];tangent=[1/length,slope/length]
                points=np.array(detail['measured_points']);xx=points[:,0:1];yy=points[:,1:2]
                middle=g._G(prep['smooth'],xx,yy,tangent)
                upper=g._G(prep['smooth'],xx-8*normal[0],yy-8*normal[1],tangent)
                lower=g._G(prep['smooth'],xx+8*normal[0],yy+8*normal[1],tangent)
                for j,row in enumerate(detail['station_rows']):
                    self.assertTrue(row['both_blue']);self.assertTrue(row['supported'])
                    self.assertAlmostEqual(row['rise_DN'],float(middle[j,0]-upper[j,0]),places=4)
                    self.assertAlmostEqual(row['fall_DN'],float(middle[j,0]-lower[j,0]),places=4)
                    self.assertGreaterEqual(row['rise_DN'],2.)
                    self.assertGreaterEqual(row['fall_DN'],2.)
                    px,py=row['point']
                    for sign in(-1,1):
                        color=prep['hsv'][int(round(py+sign*8*normal[1])),int(round(px+sign*8*normal[0]))]
                        self.assertTrue(90<=color[0]<=140 and color[1]>=80 and color[2]>=35)

    def test_joint_fallback_adds_only_two_batched_G_strips(self):
        unused,meta,prep,cue=self.prepare(BATCH,'frame',1);shapes=[];original=g._G
        def record(im,mx,my,tangent):
            shapes.append(mx.shape);return original(im,mx,my,tangent)
        with patch.object(g,'_G',record),patch.object(cv2,'remap',wraps=cv2.remap)as remap:
            g.measure(prep['smooth'],prep['hsv'],prep['local_bbox'],p._fit_edge,p.fit,p.residual,cue=cue)
        # Original peak/profile4 + joint2 + unchanged full-bbox terminal4.
        self.assertEqual(len(shapes),10)
        self.assertEqual(shapes[4:6],[(31,64),(21,64)])
        self.assertEqual(remap.call_count,30)

    def test_first_recovered_face_still_passes_original_2px_geometry(self):
        image,meta,unused,unused=self.prepare(BATCH,'frame',1)
        cfg=json.loads((BATCH/'config.json').read_text())['car']
        ob=list(profile_recovery.recover_steps(image,meta['frame'],meta['bbox_px'],
            cfg['intrinsics'],cfg['extrinsics'],cfg['table_plane'],top_method='ridge'))[-1]['observation']
        self.assertEqual(ob['status'],'localized',ob)
        self.assertEqual(ob['quality']['evidence_type'],'physical_face_with_G_ridge')
        self.assertEqual(ob['quality']['reprojection_limit_px'],2.)
        self.assertLessEqual(ob['quality']['reprojection_rms_px'],2.)
        self.assertEqual(len(ob['payload']['physical_edges']),4)
        self.assertEqual(ob['payload']['physical_edges'][0]['support'],63)
        self.assertTrue(ob['payload']['poses'][0]['table_geometry']['accepted'])
        self.assertEqual(ob['frame_id'],meta['frame']['frame_id'])
        self.assertEqual(ob['capture_timestamp_ms'],meta['frame']['capture_timestamp_ms'])
        self.assertIs(ob['payload']['seam_evidence']['original_RGB8_step_evidence'],False)

    def test_retained_original_successful_G_edges_are_bitwise_unchanged(self):
        old=self.frozen();count=0
        for folder,prefix,nmax in(('live_static_01','static',5),('live_static_02','verify',20),
                ('live_static_06_final_timing','fresh',3)):
            for n in range(1,nmax+1):
                unused,meta,prep,cue=self.prepare(HISTORY/folder,prefix,n)
                args=(prep['smooth'],prep['hsv'],prep['local_bbox'],p._fit_edge,p.fit,p.residual)
                try:before=old.measure(*args,cue=cue,audit=True)
                except ValueError:continue
                with self.subTest(folder=folder,frame=n):
                    self.assertEqual(g.measure(*args,cue=cue,audit=True),before)
                    count+=1
        self.assertGreaterEqual(count,3)


if __name__=='__main__':unittest.main()
