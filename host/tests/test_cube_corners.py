"""Real retained captures through measured corners, pose and shared runtime.

Local images are not versioned. These tests check measurement provenance,
schemas and same-frame integration, not surveyed physical or grasp accuracy.
Deployment builds run in a separate host process to keep SciPy out of the
dedicated CV environment's NumPy process.
"""
import ast
import hashlib
import importlib.util
import json
import math
import subprocess
import unittest
from types import SimpleNamespace

from host.project_paths import ROOT

HAS_VISION = importlib.util.find_spec('cv2') is not None
if HAS_VISION:
    import cv2
    from device.k230.vision import cube_corners, cube_observation, physical_side
    from device.k230.vision.runtime import localize_precise
    from device.k230.vision.candidate_association import associate_fine_candidate

LOCAL = ROOT/'data/car/full_process_tracking_20261103/post_static_anchor_corrections_01'
HELPERS = ('cube_pose.py', 'cube_corners.py', 'cube_observation.py')
_BUNDLES = None


class CubeLayoutTests(unittest.TestCase):
    def test_vertical_halo_wall_does_not_replace_the_top_depth_corner(self):
        from device.k230.vision import cube_corners as corners
        polygon=[[30.,90.],[64.,25.],[206.,37.],[227.,159.],
                 [210.,300.],[30.,300.]]
        mirrored,evidence=corners._mirrored_layout(
            SimpleNamespace(distance=math.dist),polygon,[24,24,186,272])
        self.assertFalse(mirrored)
        self.assertEqual(evidence['contour_corner_px'],[30.,90.])

    def test_horizontal_back_edge_does_not_reverse_the_opposite_view(self):
        from device.k230.vision import cube_corners as corners
        polygon=[[30.,44.],[166.,24.],[209.,65.],[209.,235.],[30.,235.]]
        mirrored,evidence=corners._mirrored_layout(
            SimpleNamespace(distance=math.dist),polygon,[24,24,186,230])
        self.assertTrue(mirrored)
        self.assertEqual(evidence['contour_corner_px'],[209.,65.])


def host_bundles():
    global _BUNDLES
    if _BUNDLES is None:
        source = '''import hashlib,json
from pathlib import Path
from tools.deploy.k230_pick import build_bundle as standalone_bundle
from tools.deploy.k230_robot import build_bundle as robot_bundle
unused,tracking,tm=robot_bundle(tracking=True)
unused,standalone,sm=standalone_bundle()
helpers={}
for name in ('cube_pose.py','cube_corners.py','cube_observation.py'):
    original=(Path('device/k230/vision')/name).read_bytes()
    helpers[name]={}
    for mode,files,manifest in (('tracking',tracking,tm),('standalone',standalone,sm)):
        data=files[name]
        helpers[name][mode]={'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest(),
            'matches_source':data==original,'manifest':manifest['files'][name]}
print(json.dumps(helpers,separators=(',',':')))
'''
        result = subprocess.run([str(ROOT/'scripts/host'), '-c', source],
            cwd=ROOT, text=True, capture_output=True)
        if result.returncode:
            raise AssertionError('Cube bundle probe failed: '+result.stderr)
        _BUNDLES = json.loads(result.stdout)
    return _BUNDLES


class CubeCornerBundleTests(unittest.TestCase):
    def test_tracking_and_standalone_include_all_three_exact_source_modules(self):
        bundles = host_bundles()
        for name in HELPERS:
            source = (ROOT/'device/k230/vision'/name).read_bytes()
            ast.parse(source)
            for mode in ('tracking', 'standalone'):
                with self.subTest(module=name, mode=mode):
                    item = bundles[name][mode]
                    self.assertTrue(item['matches_source'])
                    self.assertEqual(item['bytes'], len(source))
                    self.assertEqual(item['sha256'], hashlib.sha256(source).hexdigest())
                    self.assertEqual(item['manifest']['bytes'], len(source))
                    self.assertEqual(item['manifest']['sha256'], item['sha256'])


@unittest.skipUnless(HAS_VISION, 'Dedicated local CV environment')
class CubeCornerSavedCaptureTests(unittest.TestCase):
    def saved_materials(self):
        definitions = [('live_static_03', ['static_%02d'%n for n in range(1, 11)]),
            ('exposure_controlled', ['%s_%02d'%(mode, n)
                for mode in ('auto', 'manual_gain2', 'manual_gain4')
                for n in range(1, 4)])]
        metadata = []
        for folder, stems in definitions:
            directory = LOCAL/folder
            required = [directory/'config.json']+[directory/(stem+'.json')
                for stem in stems]
            if any(not path.is_file() for path in required):
                self.skipTest('Retained current-frame materials are local-only: '+folder)
            config = json.loads((directory/'config.json').read_text())['car']
            for stem in stems:
                record = json.loads((directory/(stem+'.json')).read_text())
                path = directory/record['image_file']
                if not path.is_file():
                    self.skipTest('Retained full-camera JPEG is local-only: '+str(path))
                metadata.append((config, record, path))
        self.assertEqual(len(metadata), 19)
        for config, record, path in metadata:
            if record.get('image_sha256'):
                self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),
                                 record['image_sha256'])
            image = cv2.imread(str(path))
            self.assertIsNotNone(image, str(path))
            self.assertEqual(image.shape, (960, 1280, 3))
            yield config, record, image, str(path.relative_to(LOCAL))

    def paired_coarse(self, record):
        coarse = record.get('paired_coarse') or record.get('coarse')
        self.assertIsInstance(coarse, dict)
        self.assertEqual(coarse['frame_id'], record['frame']['frame_id'])
        self.assertEqual(coarse['capture_timestamp_ms'],
                         record['frame']['capture_timestamp_ms'])
        return coarse

    def bbox(self, record):
        return record.get('bbox_px') or self.paired_coarse(record)['bbox_px']

    def assert_current_cube_observation(self, observation, record):
        self.assertEqual(observation['status'], 'localized', observation)
        self.assertEqual(observation['reference_point'], 'cube_geometric_center')
        self.assertFalse(observation['motion_performed'])
        self.assertFalse(observation['xy_compensation_applied'])
        for key in ('frame_id', 'capture_timestamp_ms', 'clock_domain',
                    'image_size_px', 'timestamp_source'):
            if key in record['frame']:
                self.assertEqual(observation[key], record['frame'][key], key)
        quality = observation['quality']
        self.assertEqual(quality['evidence_type'], 'nonplanar_cube_five_corners')
        self.assertEqual(quality['reprojection_limit_px'], 3.)
        self.assertLessEqual(quality['reprojection_rms_px'], 3.)
        self.assertEqual(quality['boundary_method'], 'six_measured_edges_five_corners')
        payload = observation['payload']
        self.assertEqual(payload['status'], 'complete_cube_candidate')
        self.assertNotIn('poses', payload)
        self.assertNotIn('side_faces', payload)
        self.assertEqual(len(payload['observed_edges']), 6)
        for edge in payload['observed_edges']:
            measured = edge['quality']
            self.assertEqual(measured['sampled_stations'], 64)
            self.assertGreaterEqual(measured['support'], 45)
            self.assertLessEqual(measured['rms_px'], 3.)
        errors = payload['half_corner_errors_px']
        self.assertEqual(len(errors), 10)
        self.assertTrue(all(type(v) in (int, float) and 0 <= v <= 3
                            for v in errors))
        self.assertEqual(len(payload['cube_pose']['cube_projected_corners_px']), 8)
        self.assertEqual(payload['cube_pose']['cube_projection_source'], 'pose_transform')
        self.assertTrue(payload['cube_pose']['table_geometry']['accepted'])
        association = associate_fine_candidate(observation, self.paired_coarse(record))
        self.assertEqual(association['status'], 'matched', association)
        self.assertEqual(association['geometry_capability'], 'full_cube_corner_pose')
        self.assertEqual(association['frame_id'], observation['frame_id'])
        self.assertEqual(association['capture_timestamp_ms'], observation['capture_timestamp_ms'])

    def test_real19_public_localizer_returns_explicit_cube_quality_and_same_frame_match(self):
        count = 0
        for config, record, image, name in self.saved_materials():
            count += 1
            with self.subTest(image=name):
                events = list(physical_side.localize_steps(image, record['frame'],
                    self.bbox(record), config['intrinsics'], config['extrinsics'],
                    config['table_plane']))
                self.assertEqual(sum(e['stage'] == 'complete' for e in events), 1)
                self.assertEqual(events[-1]['stage'], 'complete')
                self.assert_current_cube_observation(events[-1]['observation'], record)
        self.assertEqual(count, 19)

    def assert_measured_corner_packet(self, measurement):
        edges = measurement['observed_edges']
        self.assertEqual(len(edges), 6)
        self.assertEqual(len({e['name'] for e in edges}), 6)
        points = measurement['image_points_px']
        self.assertEqual(len(points), 5)
        self.assertEqual(len(measurement['object_points_mm']), 5)
        errors = measurement['half_corner_errors_px']
        self.assertEqual(len(errors), 10)
        self.assertTrue(all(type(v) in (int, float) and math.isfinite(v)
                            and 0 <= v <= 3 for v in errors))
        # Corners are intersections of the transmitted measured lines, not
        # bbox corners or the output of a fitted cube projected back to pixels.
        pairs = ((0, 1), (1, 2), (2, 3), (4, 5), (4, 0))
        for j, (a, b) in enumerate(pairs):
            actual = physical_side.intersect(edges[a]['line_px'], edges[b]['line_px'])
            self.assertLess(math.hypot(actual[0]-points[j][0], actual[1]-points[j][1]), 1e-5)
            for half in (0, 1):
                value = physical_side.intersect(edges[a]['half_lines_px'][half],
                                                edges[b]['half_lines_px'][half])
                self.assertAlmostEqual(math.hypot(value[0]-points[j][0],
                    value[1]-points[j][1]), errors[half*5+j], places=5)

    def test_real19_direct_measurements_build_without_packet_adaptation(self):
        count = 0
        for config, record, image, name in self.saved_materials():
            count += 1
            with self.subTest(image=name):
                intrinsics = config['intrinsics']
                K = [list(row) for row in intrinsics['camera_matrix']]
                K[0] = [v*1280/intrinsics['image_width'] for v in K[0]]
                K[1] = [v*960/intrinsics['image_height'] for v in K[1]]
                D = intrinsics['distortion_coefficients']
                T = config['extrinsics']['T_Base_from_Camera_mm']
                iterator = cube_corners.corner_steps(image, self.bbox(record),
                    intrinsics, config['extrinsics'], config['table_plane'])
                raw, reasons = None, []
                try:
                    for event in iterator:
                        if event['stage'] != 'candidate':
                            if event.get('reason'):reasons.append(event['reason'])
                            continue
                        measured = event['measurement']
                        self.assert_measured_corner_packet(measured)
                        try:
                            raw = cube_observation.build_result(measured, K, D, T,
                                                                config['table_plane'])
                        except ValueError as exc:
                            reasons.append(str(exc))
                            continue
                        break
                finally:
                    iterator.close()
                self.assertIsNotNone(raw, reasons)
                observation = localize_precise(None, record['frame'], intrinsics,
                    config['extrinsics'], config['table_plane'], localizer=lambda *args: raw)
                self.assert_current_cube_observation(observation, record)
        self.assertEqual(count, 19)


if __name__ == '__main__':
    unittest.main()
