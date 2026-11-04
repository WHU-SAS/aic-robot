"""Real-image wall material/coverage regressions and native container contract.

Local capture replay is not board API/timing or surveyed grasp validation.
No SciPy or deploy imports enter the dedicated CV environment.
"""
import copy
import hashlib
import importlib.util
import json
import unittest
from unittest.mock import patch
from host.project_paths import ROOT

HAS_VISION=importlib.util.find_spec('cv2') is not None
if HAS_VISION:
    import cv2
    import numpy as np
    from device.k230.vision import physical_side as p,edge_profiles as profiles,cube_observation as cube

BASE=ROOT/'data/vision/near_supplement/20261104_005624'
HARD=ROOT/'data/vision/hard_cases/cases/93fffb2593baf848-f5ffc532eb74'


@unittest.skipUnless(HAS_VISION,'Dedicated local CV environment')
class WallProfileTests(unittest.TestCase):
    def test_exact_bounded_histogram_noise_matches_full_numpy_medians(self):
        rng=np.random.default_rng(230)
        images=[np.full(shape,value,dtype=np.uint8)for shape in((3,3),(4,5),(25,24))for value in(0,127,255)]
        images +=[rng.integers(0,256,size=shape,dtype=np.uint8)for shape in
            ((2,2),(3,4),(4,3),(7,9),(8,8),(25,24),(24,25),(201,200))for unused in range(6)]
        for image in images:
            value=image.astype(np.float64)
            hh=(value[:-1:2,:-1:2]-value[1::2,:-1:2]-value[:-1:2,1::2]+value[1::2,1::2])/2
            expected=float(np.median(np.abs(hh-np.median(hh))))/.67448975
            self.assertEqual(profiles._wall_noise_sigma(image),expected)

    def load(self,name):
        if name=='H061':
            path=ROOT/'data/vision/acceptance_20261104/residual_before_six/H061_measured_front_back_geometry.json'
            required=[path,HARD/'image.jpg',HARD/'calibration.json',HARD/'case.json']
            if any(not q.is_file()for q in required):self.skipTest('H061 camera evidence is local-only')
            packet=json.loads(path.read_text());measurement=packet['measurements'][0]
            cfg=json.loads((HARD/'calibration.json').read_text());image_path=HARD/'image.jpg'
            metadata=packet['face_observation']['profile_recovery'];expected=json.loads((HARD/'case.json').read_text())['image_sha256']
        else:
            number=int(name[-2:]);path=BASE/'fast_pipeline_review/with_front_back_20261104.json'
            required=[path,BASE/'config.json',BASE/('frame_%02d.jpg'%number),BASE/('frame_%02d.json'%number)]
            if any(not q.is_file()for q in required):self.skipTest('Retained front/back capture is local-only')
            observation=next(q for q in json.loads(path.read_text())['rows']if q['number']==number)['observation']
            raw=observation['payload'];pose=raw['cube_pose'];measurement=dict(evidence_type=raw['evidence_type'],
                object_points_mm=pose['object_points_mm'],image_points_px=pose['image_points_px'],observed_edges=raw['observed_edges'],
                half_corner_errors_px=raw['half_corner_errors_px'],roi_px=observation['search']['roi_px'])
            cfg=json.loads((BASE/'config.json').read_text())['car'];image_path=BASE/('frame_%02d.jpg'%number)
            expected=json.loads((BASE/('frame_%02d.json'%number)).read_text())['image_sha256'];metadata=observation['profile_recovery']
        self.assertEqual(hashlib.sha256(image_path.read_bytes()).hexdigest(),expected)
        image=cv2.imread(str(image_path));ox,oy,w,h=measurement['roi_px'];raw=image[oy:oy+h,ox:ox+w].copy()
        im=cv2.GaussianBlur(raw,(3,3),.7);hsv=cv2.cvtColor(raw,cv2.COLOR_BGR2HSV)
        q=[[v[0]-ox,v[1]-oy]for v in measurement['image_points_px'][:4]]
        proposal=metadata.get('face_proposal',{});seed=proposal.get('crease_seed_roi_px',[q[1],q[2]])if proposal.get('side','right')=='right'else[q[1],q[2]]
        terminal=metadata.get('separator_profile',{}).get('terminal_support',{})
        anchor=terminal.get('last_point_px')if'full_bbox'in terminal.get('method','')else None
        return cfg,measurement,(p,im,hsv,seed[0],seed[1],q[1],q[2],anchor)

    def cube_result(self,cfg,measurement,edge):
        value=copy.deepcopy(measurement);ox,oy=value['roi_px'][:2]
        def restore(line):return[line[0],line[1],line[2]-line[0]*ox-line[1]*oy]
        line=restore(edge[0]);halves=[restore(v)for v in edge[2]]
        es=value['observed_edges'];es[1]=dict(name='front_right',line_px=line,half_lines_px=halves,quality=edge[1])
        value['image_points_px'][1]=p.intersect(es[0]['line_px'],line);value['image_points_px'][2]=p.intersect(line,es[2]['line_px'])
        errors=[]
        for half in(0,1):
            q=[p.intersect(es[i-1 if i else 3]['half_lines_px'][half],es[i]['half_lines_px'][half])for i in range(4)]
            q.extend([p.intersect(es[4]['half_lines_px'][half],es[5]['half_lines_px'][half]),p.intersect(es[5]['half_lines_px'][half],es[6]['half_lines_px'][half])])
            errors.extend(p.distance(a,b)for a,b in zip(value['image_points_px'],q))
        value['half_corner_errors_px']=errors;intr=cfg['intrinsics'];K=[list(row)for row in intr['camera_matrix']]
        K[0]=[v*1280/intr['image_width']for v in K[0]];K[1]=[v*960/intr['image_height']for v in K[1]]
        result=cube.build_result(value,K,intr['distortion_coefficients'],cfg['extrinsics']['T_Base_from_Camera_mm'],cfg['table_plane'])
        self.assertLessEqual(max(errors),3.)
        self.assertTrue(result['table_geometry']['accepted'])
        self.assertLessEqual(result['cube_pose']['corner_reprojection_rms_px'],3.)
        self.assertEqual(len(result['cube_pose']['cube_projected_corners_px']),8)
        return result

    def test_actual_H061_and_new03_new04_keep_original_station_half_cube_and_table_limits(self):
        for name,expected in [('H061',1.7223384004),('new03',2.7245150202),('new04',1.9105134578)]:
            with self.subTest(image=name):
                cfg,measurement,args=self.load(name);before=copy.deepcopy(measurement);pixels=args[1].tobytes();colors=args[2].tobytes()
                with patch.object(p,'_native_contrasts',wraps=p._native_contrasts)as batch,\
                     patch.object(p,'pose_candidates',side_effect=AssertionError('wall measurement cannot solve a pose')):
                    edge,audit=profiles.wall_edge(*args)
                self.assertIsNotNone(edge,audit)
                self.assertEqual(batch.call_count,1)
                self.assertEqual(audit['native_RGB_profile_batches'],1)
                self.assertLessEqual(audit['proposal_count'],3)
                self.assertEqual(audit['profile_shape'],[25,64])
                self.assertEqual(audit['sample_fraction'],[.15,.85])
                self.assertIsNone(audit['capture_timestamp_ms'])
                self.assertIsNone(audit['clock_domain'])
                quality=edge[1];self.assertEqual(quality['sampled_stations'],64)
                self.assertGreaterEqual(quality['support'],45)
                self.assertLessEqual(quality['rms_px'],3.)
                self.assertFalse(quality['broad_HSV_outside_nonblue_verified'])
                self.assertTrue(quality['original_RGB8_step_evidence'])
                result=self.cube_result(cfg,measurement,edge)
                self.assertAlmostEqual(result['cube_pose']['corner_reprojection_rms_px'],expected,places=5)
                self.assertEqual(measurement,before)
                self.assertEqual(args[1].tobytes(),pixels);self.assertEqual(args[2].tobytes(),colors)

    def test_native_Nx4_and_PC_Nx1x4_use_identical_actual_Hough_segments(self):
        unused,unused,args=self.load('H061');original=cv2.HoughLinesP;captured=[]
        def native(*a,**kw):
            result=original(*a,**kw);captured.append(result.copy());return result
        with patch.object(cv2,'HoughLinesP',native):expected,audit=profiles.wall_edge(*args)
        self.assertIsNotNone(expected,audit)
        for shape in((-1,4),(-1,1,4)):
            with patch.object(cv2,'HoughLinesP',return_value=captured[0].reshape(shape)):
                actual,details=profiles.wall_edge(*args)
            self.assertEqual(actual,expected)
            self.assertEqual(details['selected'],audit['selected'])

    def test_whole_wall_measurement_is_used_by_the_public_static_pipeline(self):
        required=[HARD/'image.jpg',HARD/'calibration.json',HARD/'case.json']
        if any(not q.is_file()for q in required):self.skipTest('H061 camera evidence is local-only')
        cfg=json.loads((HARD/'calibration.json').read_text())
        frame=dict(frame_id='H061_public_static',capture_timestamp_ms=None,
            clock_domain=None,image_size_px=[1280,960],timestamp_source='unavailable')
        image=cv2.imread(str(HARD/'image.jpg'));pixels=image.tobytes()
        observation=list(p.localize_steps(image,frame,None,cfg['intrinsics'],
            cfg['extrinsics'],cfg['table_plane']))[-1]['observation']
        self.assertEqual(observation['status'],'localized',observation)
        self.assertEqual(observation['frame_id'],frame['frame_id'])
        self.assertIsNone(observation['capture_timestamp_ms'])
        self.assertFalse(observation['motion_performed'])
        self.assertEqual(observation['payload']['evidence_type'],'nonplanar_cube_face_back_top')
        audit=observation['profile_recovery']['whole_wall_profile']
        self.assertFalse(audit['pose_used']);self.assertFalse(audit['table_height_used'])
        self.assertEqual(audit['native_RGB_profile_batches'],1)
        self.assertLessEqual(observation['quality']['reprojection_rms_px'],3.)
        self.assertEqual(image.tobytes(),pixels)

    def test_native_None_and_empty_arrays_are_image_rejections_without_RGB_resampling(self):
        unused,unused,args=self.load('H061')
        for output in(None,np.empty((0,4),dtype=np.int32),np.empty((0,1,4),dtype=np.int32)):
            with patch.object(cv2,'HoughLinesP',return_value=output),\
                 patch.object(p,'_native_contrasts',side_effect=AssertionError('no Hough candidate means no RGB bank')):
                edge,audit=profiles.wall_edge(*args)
            self.assertIsNone(edge);self.assertEqual(audit['native_RGB_profile_batches'],0)
            self.assertIn('No observed Hough',audit['reason'])

    def test_an_unavailable_board_API_is_not_silently_replaced_by_another_algorithm(self):
        unused,unused,args=self.load('H061')
        with patch.object(cv2,'Canny',side_effect=TypeError('unsupported native signature')):
            with self.assertRaisesRegex(TypeError,'unsupported native signature'):profiles.wall_edge(*args)

    def test_solid_blue_or_flat_gray_contains_no_observed_wall(self):
        for color in([240,50,5],[100,100,100]):
            image=np.zeros((180,160,3),dtype=np.uint8);image[:]=color;hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
            edge,audit=profiles.wall_edge(p,image,hsv,[80,40],[80,140],[80,30],[80,150],[80,30])
            self.assertIsNone(edge,audit)
            self.assertEqual(audit['native_RGB_profile_batches'],0)

    def test_real_hand_lifted_and_printed_tape_do_not_gain_a_precise_cube(self):
        if not(BASE/'config.json').is_file():self.skipTest('Retained negative captures are local-only')
        cfg=json.loads((BASE/'config.json').read_text())['car']
        cases=[(BASE/('hexagon_review/real_hand_frame_%04d.jpg'%n),box)for n,box in
            [(30,[362,270,102,126]),(510,[424,126,107,134]),(900,[494,82,102,108])]]
        cases.append((ROOT/'data/car/full_process_tracking_20261103/overshoot_repair_repeat_01/evidence/tracking_5_moving.jpg',[285,669,328,92]))
        if any(not path.is_file()for path,box in cases):self.skipTest('Real negative images are local-only')
        for path,box in cases:
            with self.subTest(image=path.name):
                image=cv2.imread(str(path));frame=dict(frame_id='offline_negative',capture_timestamp_ms=None,
                    clock_domain=None,image_size_px=[1280,960],timestamp_source='unavailable')
                observation=list(p.localize_steps(image,frame,box,cfg['intrinsics'],cfg['extrinsics'],cfg['table_plane']))[-1]['observation']
                self.assertEqual(observation['status'],'no_complete_face',observation)
                self.assertNotIn('cube_center_Base_mm',observation)
                self.assertNotIn('payload',observation)
                self.assertFalse(observation['motion_performed'])


if __name__=='__main__':unittest.main()
