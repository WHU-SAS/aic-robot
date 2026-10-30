"""Offline regression for the explicit arm-model alignment profile; no hardware."""
import copy
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

from host.project_paths import ROOT
from host.apps.pick_validation_pc import parameters, make_plan
from host.arm.urdf_fk import URDFForwardKinematics
from tools.deploy.k230_pick import build_bundle

BOARD = ROOT / 'device/k230/pick'
sys.path.insert(0, str(BOARD))
from kinematics import Geometry

PROFILE = ROOT / 'configs/arm/pick_model_check.json'
STAGES = ('hover', 'descend', 'close', 'lift')
# Fixed facts from the original default, captured before introducing an explicit
# corrected-model profile. These guard the intentionally retained default baseline.
OLD_GOALS = {
    'hover': [2193, 2470, 2825, 2613, 2048, 2389],
    'descend': [2193, 2643, 2922, 2343, 2048, 2389],
    'close': [2193, 2643, 2922, 2343, 2048, 2167],
    'lift': [2193, 2470, 2825, 2613, 2048, 2167],
}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load(path):
    return json.loads(Path(path).read_text())


class ArmModelAlignmentTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.pick, cls.config = parameters(PROFILE)
        cls.bundle, cls.files, cls.manifest = build_bundle(PROFILE)
        cls.model_path = ROOT / cls.pick['model_urdf']
        cls.fk = URDFForwardKinematics(str(cls.model_path))
        cls.old_fk = URDFForwardKinematics(str(ROOT / 'models/genkiarm/genkiarm.urdf'))
        cls.board = Geometry(cls.bundle['model'], cls.pick, -67.)

    def test_explicit_model_has_measured_mount_and_pitch_axes(self):
        joints = self.fk.joints
        np.testing.assert_allclose(joints['Rotation']['rpy'], [0., -math.pi/2, math.pi], atol=1e-10)
        for name in ('Rotation2', 'Rotation3', 'Rotation4'):
            np.testing.assert_array_equal(joints[name]['axis'], [0., -1., 0.])
        np.testing.assert_array_equal(joints['Rotation5']['axis'], [0., 0., 1.])
        np.testing.assert_array_equal(joints['Rotation5']['rpy'], [0., 0., 0.])
        # Model repair must not change encoding references, sign, or limits.
        old_mapping = load(ROOT / 'configs/arm/joint_mapping.json')['joints']
        for exported, original in zip(self.bundle['model']['joints'], old_mapping):
            for key in ('zero_count', 'positive_direction', 'software_count_limits'):
                self.assertEqual(exported[key], original[key])

    def test_candidate_is_explicit_and_calibration_is_bound_to_actual_files(self):
        self.assertNotIn('model_urdf', parameters()[0])
        self.assertEqual(self.model_path, ROOT / 'models/genkiarm/genkiarm_corrected.urdf')
        ext = self.bundle['extrinsics']
        self.assertEqual(ext['geometry_model_sha256'], sha(self.model_path))
        self.assertEqual(ext['joint_mapping_sha256'], sha(ROOT / 'configs/arm/joint_mapping.json'))
        self.assertEqual(self.pick['alignment_reference'], 'open_fingertip_midpoint')
        self.assertEqual(self.pick['cube_offset_toward_moving_finger_mm'], 0)
        self.assertFalse(self.manifest['startup_motion'])
        self.assertFalse(self.manifest['pc_runtime_required'])

    def test_retained_default_configuration_and_targets_remain_unchanged(self):
        cfg, _, _ = build_bundle()
        self.assertEqual(cfg['pick'], load(ROOT / 'configs/arm/pick_validation_pc.json'))
        self.assertNotIn('model_urdf', cfg['pick'])
        # Compare against the retained historical model's parsed geometry,
        # independent of comments, whitespace and URDF serialization.
        for exported in cfg['model']['joints']:
            j = self.old_fk.joints[exported['urdf_joint_name']]
            expected = np.eye(4)
            expected[:3,:3] = self.old_fk._rpy_to_matrix(j['rpy'])
            expected[:3,3] = j['xyz']
            np.testing.assert_allclose(exported['origin_transform'], expected, atol=1e-12, rtol=0)
            np.testing.assert_array_equal(exported['axis'], j['axis'])
        for name in ('Rotation2','Rotation3','Rotation4'):
            np.testing.assert_array_equal(self.old_fk.joints[name]['axis'], [0.,1.,0.])
        self.assertAlmostEqual(self.old_fk.joints['Rotation']['rpy'][2], 0.)
        self.assertAlmostEqual(self.old_fk.joints['Rotation5']['rpy'][2], 0.)
        pick, config = parameters()
        _, plan = make_plan({'cube_center_Base_mm': [245., -28., -65.]}, [2048]*6, pick, config)
        self.assertNotIn('model_urdf', plan)
        self.assertFalse(plan['model_correction_applied'])
        for stage in STAGES:
            self.assertEqual(plan[stage], OLD_GOALS[stage], stage)

    def test_pc_and_portable_fk_agree_for_joint_frames_and_fingertips(self):
        rng = np.random.default_rng(20261030)
        samples = [[2043, 2081, 2105, 2026, 2051, 2395], [2048]*6]
        for _ in range(32):
            samples.append([int(rng.integers(j['software_count_limits'][0],
                                             j['software_count_limits'][1]+1))
                            for j in self.bundle['model']['joints']])
        pc, _ = make_plan({'cube_center_Base_mm': [253., 27., -55.]},
                          [2048]*6, self.pick, self.config)
        for counts in samples:
            with self.subTest(counts=counts):
                q = pc.angles(counts)
                angles = dict(zip(pc.names, q))
                board_ts = self.board.transforms(q.tolist())
                for link, ts in board_ts.items():
                    expected = self.fk.forward_kinematics(angles, target_link=link)[0]
                    np.testing.assert_allclose(ts, expected, atol=2e-11, rtol=0)
                a = pc.metrics(q)
                b = self.board.metrics(q.tolist())
                for link in ('wan2', 'zhua'):
                    np.testing.assert_allclose(a[1][link], b[1][link], atol=2e-11, rtol=0)
                    self.assertAlmostEqual(a[2][link], b[2][link], places=10)
                np.testing.assert_allclose(pc.alignment_midpoint(q),
                                           self.board.alignment_midpoint(q.tolist()), atol=2e-11, rtol=0)

    def test_body_orientation_fixes_the_mount_pi_gauge_ambiguity(self):
        # Both historical hypotheses place the same gripper at the same Base
        # location. Only the upstream body orientation distinguishes them.
        for counts in ([2043,2081,2105,2026,2051,2395], [2048,2600,2700,2500,1820,2389]):
            q = [self.config.mapping.count_to_radians(i, counts[i-1]) for i in range(1,7)]
            names = [self.config.mapping.joint(i)['urdf_joint_name'] for i in range(1,7)]
            old_q = list(q)
            old_q[4] += math.pi
            for link in ('wan2', 'zhua'):
                old = self.old_fk.forward_kinematics(dict(zip(names, old_q)), target_link=link)[0]
                new = self.fk.forward_kinematics(dict(zip(names, q)), target_link=link)[0]
                np.testing.assert_allclose(new, old, atol=2e-10, rtol=0)
            old = self.old_fk.forward_kinematics(dict(zip(names, old_q)), target_link='wan')[0]
            new = self.fk.forward_kinematics(dict(zip(names, q)), target_link='wan')[0]
            np.testing.assert_allclose(new[:3,3], old[:3,3], atol=2e-10, rtol=0)
            np.testing.assert_allclose(old[:3,:3].T @ new[:3,:3], np.diag([-1.,-1.,1.]), atol=2e-10, rtol=0)

    def test_four_stage_ik_agrees_at_nine_positions_and_wrist_angles(self):
        for xy in ([242.,10.], [253.,27.], [264.,45.]):
            for c5 in (1820,2048,2276):
                with self.subTest(xy_mm=xy, c5=c5):
                    start = [2048]*6
                    start[4] = c5
                    pc, pp = make_plan({'cube_center_Base_mm': [*xy,-55.]}, start,
                                       self.pick, self.config)
                    board = Geometry(self.bundle['model'], self.pick, pp['table_z_Base_mm'])
                    bp = board.plan([v/1000 for v in xy], start)
                    self.assertEqual(pp['tilt_deg'], bp['tilt_deg'])
                    self.assertEqual(pp['model_urdf'], self.pick['model_urdf'])
                    self.assertEqual(pp['geometry_model_sha256'], sha(self.model_path))
                    self.assertTrue(pp['model_correction_applied'])
                    for stage in STAGES:
                        self.assertEqual(pp[stage], bp[stage], stage)
                        # Rotation5 remains the chosen starting angle; this
                        # test does not authorize automatic orientation changes.
                        self.assertEqual(pp[stage][4], c5)
                        counts = pp[stage]
                        for j, value in zip(self.bundle['model']['joints'], counts):
                            low, high = j['software_count_limits']
                            self.assertLessEqual(low, value)
                            self.assertLessEqual(value, high)
                        a, b = pc.report(counts), board.report(counts)
                        for key in ('alignment_center_Base_mm', 'fixed_tip_center_Base_mm',
                                    'moving_tip_center_Base_mm'):
                            np.testing.assert_allclose(a[key], b[key], atol=2e-8, rtol=0)
                    # Independently check the requested open-jaw approach poses,
                    # rather than passing just because both solvers agree.
                    for stage, height in [('hover',self.pick['hover_tip_height_mm']),
                                          ('descend',self.pick['descend_tip_height_mm'])]:
                        report = pc.report(pp[stage])
                        self.assertLess(np.linalg.norm(np.array(report['alignment_center_Base_mm'][:2])-xy), 2.)
                        minimum = min(report['link_min_z_mm'].values())
                        self.assertLess(abs(minimum-pp['table_z_Base_mm']-height), 2.)

    def _bad_pair(self, field):
        # Keep all other data valid. Both entry points must reject a mismatched
        # model/calibration before any camera or serial action can be reached.
        vision = copy.deepcopy(self.bundle['vision'])
        ext = copy.deepcopy(self.bundle['extrinsics'])
        ext[field] = '0'*64
        pick = copy.deepcopy(self.pick)
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            epath, vpath, ppath = (tmp/'extrinsics.json', tmp/'vision.json', tmp/'pick.json')
            epath.write_text(json.dumps(ext))
            vision['localization']['extrinsics'] = str(epath)
            vpath.write_text(json.dumps(vision))
            pick['vision_config'] = str(vpath)
            ppath.write_text(json.dumps(pick))
            for action in (parameters, build_bundle):
                with self.subTest(field=field, entry=action.__name__):
                    with self.assertRaises(ValueError):
                        action(ppath)

    def test_explicit_profile_rejects_wrong_model_hash(self):
        self._bad_pair('geometry_model_sha256')

    def test_explicit_profile_rejects_wrong_mapping_hash(self):
        self._bad_pair('joint_mapping_sha256')


if __name__ == '__main__':
    unittest.main()
