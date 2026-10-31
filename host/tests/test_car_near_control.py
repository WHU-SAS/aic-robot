"""Fine-centre prediction, parking semantics and an independent moving plant."""
import copy
import json
import math
import unittest

from device.k230.car.near_control import near_plan, predict_center, verify_parked, correction_plan, target_x
from device.k230.car.parking_window import bounds_mm
from host.car.runtime import load
from host.project_paths import ROOT


class PlanarPlant:
    """World-frame midpoint integration, velocity ramps and exponential coast.

    Synthetic nominal mechanics, not identification of the physical robot.
    Its world pose and delayed absolute images do not use controller prediction.
    """
    def __init__(self, point, normal, track):
        self.n, self.track = normal, track
        f = [1-normal[0]**2, -normal[0]*normal[1], -normal[0]*normal[2]]
        norm = math.sqrt(sum(v*v for v in f))
        self.f = [v/norm for v in f]
        self.l = [normal[1]*self.f[2]-normal[2]*self.f[1],
                  normal[2]*self.f[0]-normal[0]*self.f[2],
                  normal[0]*self.f[1]-normal[1]*self.f[0]]
        self.target = [sum(a*b for a,b in zip(point, basis))
                       for basis in (self.f, self.l, self.n)]
        self.x = self.y = self.yaw = 0
        self.vl = self.vr = 30

    def center(self):
        dx, dy = self.target[0]-self.x, self.target[1]-self.y
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        x, y = c*dx+s*dy, -s*dx+c*dy
        return [self.f[k]*x+self.l[k]*y+self.n[k]*self.target[2] for k in range(3)]

    def advance(self, dt):
        distance = (self.vl+self.vr)*dt/2
        angle = (self.vr-self.vl)*dt/self.track
        self.x += distance*math.cos(self.yaw+angle/2)
        self.y += distance*math.sin(self.yaw+angle/2)
        self.yaw += angle
        return self.vl*dt, self.vr*dt

    def step(self, left, right, dt):
        # Include a small unequal actuator response; vision closes the loop.
        self.vl += max(-150*dt, min(150*dt, .95*left-self.vl))
        self.vr += max(-150*dt, min(150*dt, right-self.vr))
        return self.advance(dt)

    def coast(self, tau):
        for unused in range(100):
            self.vl *= math.exp(-.01/tau)
            self.vr *= math.exp(-.01/tau)
            self.advance(.01)
        self.vl = self.vr = 0


class NearControlTests(unittest.TestCase):
    def setUp(self):
        self.workspace, _, self.projection = load()
        self.config = json.loads((ROOT/'configs/car/near_parking.json').read_text())
        self.near, self.far = bounds_mm(self.workspace)['Base_X_mm']
        self.target = self.near+(self.far-self.near)/4

    def center(self, x, offset=0):
        y = sum(bounds_mm(self.workspace, x)['Base_Y_mm'])/2 + offset
        n = self.projection.normal
        z = (self.projection.offset + 10 - n[0]*x - n[1]*y)/n[2]
        return [x, y, z]

    def observation(self, point, stamp=1000):
        return {'schema': 'aic.vision_observation.v1', 'mode': 'precise',
                'frame_id': stamp, 'capture_timestamp_ms': stamp,
                'timestamp_ms': stamp, 'clock_domain': 'k230_ticks_ms',
                'image_size_px': [1280, 960], 'status': 'localized',
                'coordinate_frame': 'Base', 'units': 'mm',
                'reference_point': 'cube_geometric_center',
                'xy_compensation_applied': False, 'cube_center_Base_mm': list(point)}

    def range_observation(self, point, stamp=1000):
        result = self.observation(point, stamp)
        result['status'] = 'no_complete_face'
        del result['cube_center_Base_mm']
        result['range_candidate'] = {
            'schema': 'aic.car.side_range.v1', 'status': 'range_candidate',
            'coordinate_frame': 'Base', 'units': 'mm',
            'raw_center_Base_mm': list(point), 'Base_X_mm': point[0],
            'X_interval_mm': [point[0]-.5, point[0]+.5],
            'not_arm_or_parking_authority': True}
        return result

    def plan(self, observation, now=1000, **kw):
        return near_plan(observation, self.projection, self.workspace,
                         self.config, now, **kw)

    def verify(self, observation, now=1000, stopped_at=900, ready=1000, stationary=True):
        return verify_parked(observation, self.projection, self.workspace,
            self.config, now, stopped_at, ready, stationary_since_capture=stationary)

    def test_encoder_prediction_contains_translation_yaw_and_order(self):
        point = [300, 20, 10]
        self.assertEqual(predict_center(point, [(20, 20)], 100, [0, 0, 1]),
                         [280, 20, 10])
        d = 100*math.pi/4
        rotated = predict_center(point, [(-d, d)], 100, [0, 0, 1])
        for actual, expected in zip(rotated, [20, -300, 10]):
            self.assertAlmostEqual(actual, expected)
        ordered = predict_center(point, [(5, 10), (10, 5)], 100, [0, 0, 1])
        collapsed = predict_center(point, [(15, 15)], 100, [0, 0, 1])
        self.assertGreater(abs(ordered[1]-collapsed[1]), .1)

    def test_tilted_table_height_is_preserved_and_inverse_motion_recovers_center(self):
        point = self.center(330, 12)
        deltas = [(5, 9), (7, 4), (2, 8)]
        moved = predict_center(point, deltas, self.config['track_width_mm'],
                               self.projection.normal)
        recovered = predict_center(moved, [(-l, -r) for l,r in reversed(deltas)],
                        self.config['track_width_mm'], self.projection.normal)
        self.assertAlmostEqual(sum(a*b for a,b in zip(point,self.projection.normal)),
                               sum(a*b for a,b in zip(moved,self.projection.normal)))
        for actual, expected in zip(recovered, point):
            self.assertAlmostEqual(actual, expected)

    def test_capture_time_stays_original_and_current_prediction_uses_all_increments(self):
        observation = self.observation(self.center(330, 5))
        original = copy.deepcopy(observation)
        deltas = [(3, 4), (2, 2)]
        result = self.plan(observation, 1200, wheel_deltas=deltas)
        expected = predict_center(observation['cube_center_Base_mm'], deltas,
                                  self.config['track_width_mm'], self.projection.normal)
        self.assertEqual(result['capture_timestamp_ms'], 1000)
        self.assertEqual(result['source_capture_age_ms'], 200)
        self.assertTrue(result['odometry_predicted'])
        self.assertEqual(result['predicted_center_Base_mm'], expected)
        self.assertEqual(observation, original)

    def test_older_anchor_requires_explicit_odometry_and_encoder_guided(self):
        observation = self.observation(self.center(330))
        for kw in ({}, {'encoder_guided': True}, {'wheel_deltas': [(2, 2)]}):
            self.assertEqual(self.plan(observation, 1800, **kw)['status'],
                             'no_precise_target')
        result = self.plan(observation, 1800, wheel_deltas=[(2, 2)], encoder_guided=True)
        self.assertEqual(result['status'], 'follow')
        self.assertEqual(result['source_capture_age_ms'], 800)
        self.assertEqual(result['capture_timestamp_ms'], 1000)
        self.assertTrue(result['odometry_predicted'])
        self.assertEqual(self.plan(observation, 999, wheel_deltas=[],
                                  encoder_guided=True)['status'], 'no_precise_target')

    def test_failed_or_coarse_vision_never_becomes_precise_window_acceptance(self):
        for change in ({'status': 'no_complete_face'}, {'mode': 'coarse'},
                       {'reference_point': 'ground_contact_proxy'},
                       {'xy_compensation_applied': True},
                       {'cube_center_Base_mm': [float('nan'), 0, 0]}):
            observation = self.observation(self.center(self.target))
            observation.update(change)
            self.assertEqual(self.plan(observation)['status'], 'no_precise_target')
            self.assertFalse(self.verify(observation)['accepted'])

    def test_limited_range_drives_distance_without_upgrading_failed_pose(self):
        point = self.center(350, 8)
        point[2] += 14  # Deliberately inconsistent with the complete table pose.
        observation = self.range_observation(point)
        original = copy.deepcopy(observation)
        result = self.plan(observation, 1800, wheel_deltas=[(20, 21)],
                           encoder_guided=True, steering_bearing_deg=2)
        self.assertEqual(result['status'], 'follow')
        self.assertEqual(result['distance_source'], 'physical_side_range_candidate')
        self.assertTrue(result['range_only'])
        self.assertEqual(result['raw_center_Base_mm'], point)
        self.assertEqual(result['capture_timestamp_ms'], 1000)
        self.assertEqual(result['predicted_center_Base_mm'],
            predict_center(point, [(20, 21)], self.config['track_width_mm'], self.projection.normal))
        self.assertEqual(observation, original)
        self.assertNotIn('accepted', result)
        self.assertNotIn('cube_center_Base_mm', observation)

    def test_range_inside_window_can_request_stop_but_never_verify_parked(self):
        observation = self.range_observation(self.center(self.target))
        result = self.plan(observation)
        self.assertEqual(result['status'], 'stop_for_verification')
        self.assertTrue(result['window']['inside'])
        verdict = self.verify(observation)
        self.assertEqual(verdict['status'], 'no_precise_target')
        self.assertFalse(verdict['accepted'])
        self.assertEqual(correction_plan(verdict), {'status': 'unavailable'})

    def test_strict_localized_takes_precedence_over_an_attached_range(self):
        observation = self.range_observation(self.center(450))
        observation.update(self.observation(self.center(330)))
        result = self.plan(observation)
        self.assertEqual(result['distance_source'], 'precise')
        self.assertFalse(result['range_only'])
        self.assertEqual(result['raw_center_Base_mm'], self.center(330))
        observation['xy_compensation_applied'] = True
        self.assertEqual(self.plan(observation)['status'], 'no_precise_target')

    def test_range_contract_and_original_capture_freshness_are_required(self):
        for field, value in (('schema', 'other'), ('status', 'localized'),
                             ('coordinate_frame', 'Camera'), ('units', 'm'),
                             ('not_arm_or_parking_authority', False),
                             ('Base_X_mm', 999), ('raw_center_Base_mm', [330, 0, float('nan')])):
            observation = self.range_observation(self.center(330))
            observation['range_candidate'][field] = value
            self.assertEqual(self.plan(observation)['status'], 'no_precise_target')
        observation = self.range_observation(self.center(330))
        self.assertEqual(self.plan(observation, 1800)['status'], 'no_precise_target')
        observation['mode'] = 'coarse'
        self.assertEqual(self.plan(observation)['status'], 'no_precise_target')
        observation = self.range_observation(self.center(330))
        observation['capture_timestamp_ms'] = observation['timestamp_ms'] = None
        self.assertEqual(self.plan(observation)['status'], 'no_precise_target')

    def test_coast_estimate_requests_one_stop_before_shifted_target(self):
        observation = self.observation(self.center(self.target+1))
        moving = self.plan(observation, speeds={'left_mm_s': 15, 'right_mm_s': 15})
        self.assertEqual(moving['status'], 'stop_for_verification')
        self.assertGreater(moving['predicted_center_Base_mm'][0], self.target)
        self.assertLess(moving['predicted_stop_center_Base_mm'][0], self.target)
        self.assertTrue(moving['predicted_stop_window']['inside'])
        self.assertNotIn('accepted', moving)
        self.assertEqual(self.plan(observation)['status'], 'follow')
        self.assertFalse(moving['automatic_arm_action'])

    def test_window_membership_alone_is_not_a_centered_or_near_target(self):
        far_inside = self.plan(self.observation(self.center(self.far-.1)))
        self.assertTrue(far_inside['window']['inside'])
        self.assertEqual(far_inside['status'], 'follow')
        lateral = self.plan(self.observation(self.center(self.target+1, 16)),
                          speeds={'left_mm_s': 15, 'right_mm_s': 15})
        self.assertTrue(lateral['window']['inside'])
        self.assertGreater(abs(lateral['image_error_px']), 48)
        self.assertEqual(lateral['status'], 'stop_for_verification')
        verification = self.verify(self.observation(self.center(self.target,16)))
        self.assertTrue(verification['accepted'])
        self.assertFalse(verification['preferred'])

    def test_near_overshoot_is_not_repaired_with_reverse_or_false_success(self):
        result = self.plan(self.observation(self.center(self.near-.1)))
        self.assertEqual(result['status'], 'stop_for_verification')
        self.assertNotIn('left_mm_s', result)
        self.assertFalse(self.verify(self.observation(self.center(self.near-.1)))['accepted'])

    def test_actual_formal_window_and_camera_midline_are_used_without_grasp_offsets(self):
        observation = self.observation(self.center(self.target))
        result = self.verify(observation)
        self.assertTrue(result['accepted'])
        self.assertTrue(result['preferred'])
        self.assertEqual(result['status'], 'parked')
        self.assertLess(abs(result['image_error_px']), .02)
        self.assertEqual(result['cube_center_Base_mm'], observation['cube_center_Base_mm'])
        self.assertEqual(self.workspace['normalized_window']['inset_mm'], 10)
        self.assertAlmostEqual(result['target_Base_X_mm'], 247.2972376453)
        far_inside = self.verify(self.observation(self.center(self.far-.1)))
        self.assertEqual(far_inside['status'], 'parked')
        self.assertTrue(far_inside['accepted'])
        self.assertTrue(far_inside['preferred'])
        for point in (self.center(self.far+.1), self.center(self.target,20)):
            outside = self.verify(self.observation(point))
            self.assertFalse(outside['accepted'])
            self.assertFalse(outside['preferred'])
        point = self.center(self.target)
        point[1] = 0
        self.assertFalse(self.verify(self.observation(point))['accepted'])

    def test_shifted_destination_drives_past_old_centre_but_keeps_full_acceptance_window(self):
        old_center = (self.near+self.far)/2
        self.assertAlmostEqual(old_center-target_x(self.workspace), (self.far-self.near)/4)
        self.assertEqual(self.plan(self.observation(self.center(old_center)))['status'], 'follow')
        self.assertEqual(self.plan(self.observation(self.center(self.target)))['status'], 'stop_for_verification')
        # A valid stopped result elsewhere inside the same window remains final.
        self.assertTrue(self.verify(self.observation(self.center(old_center)))['accepted'])
        for x in (self.near, self.far):
            verdict = self.verify(self.observation(self.center(x)))
            self.assertTrue(verdict['accepted'])
            self.assertEqual(verdict['target_Base_X_mm'], self.target)

    def test_static_slow_solve_is_accepted_only_for_a_new_stopped_frame(self):
        observation = self.observation(self.center(self.target), stamp=2000)
        result = self.verify(observation, now=3501, stopped_at=1900, ready=3500)
        self.assertTrue(result['accepted'])
        self.assertEqual(result['capture_timestamp_ms'], 2000)
        for timing in ({'now': 3501, 'stopped_at': 2100, 'ready': 3500},
                       {'now': 3501, 'stopped_at': 1900, 'ready': 3500, 'stationary': False},
                       {'now': 4001, 'stopped_at': 1900, 'ready': 3500},
                       {'now': 3499, 'stopped_at': 1900, 'ready': 3500}):
            self.assertFalse(self.verify(observation, **timing)['accepted'])

    def test_one_delayed_fine_anchor_then_encoder_guidance_stops_once(self):
        for offset in (-15, 0, 15):
            with self.subTest(offset_mm=offset):
                plant = PlanarPlant(self.center(500, offset), self.projection.normal,
                                    self.config['track_width_mm'])
                plant.vl = plant.vr = 60
                observation = self.observation(plant.center())
                increments = []
                # One absolute image, solved 2.5s later while normal travel continues.
                for unused in range(50):
                    increments.append(plant.step(60, 60, .05))
                for index in range(400):
                    now = 3500+index*50
                    result = self.plan(observation, now, wheel_deltas=increments,
                        encoder_guided=True,
                        speeds={'left_mm_s': plant.vl, 'right_mm_s': plant.vr})
                    if result['status'] != 'follow':
                        break
                    self.assertTrue(0 <= result['left_mm_s'] <= 80)
                    self.assertTrue(0 <= result['right_mm_s'] <= 80)
                    increments.append(plant.step(result['left_mm_s'], result['right_mm_s'], .05))
                self.assertEqual(result['status'], 'stop_for_verification')
                self.assertEqual(result['capture_timestamp_ms'], 1000)
                plant.coast(self.config['coast_s'])
                stopped_at = now+1000
                final = self.verify(self.observation(plant.center(), stopped_at+1),
                    now=stopped_at+2501, stopped_at=stopped_at, ready=stopped_at+2501)
                self.assertTrue(final['accepted'], final)
                self.assertLess(abs(plant.center()[0]-self.target), 5)

    def test_successful_fine_does_not_force_a_lower_speed_when_remaining_is_large(self):
        result = self.plan(self.observation(self.center(500)), 3500,
            wheel_deltas=[(30, 30)], encoder_guided=True)
        self.assertEqual(result['status'], 'follow')
        self.assertEqual((result['left_mm_s']+result['right_mm_s'])/2, 60)

    def test_stop_at_depth_target_even_if_lateral_steering_has_not_finished(self):
        result = self.plan(self.observation(self.center(self.target, 25)))
        self.assertFalse(result['window']['inside'])
        self.assertEqual(result['status'], 'stop_for_verification')
        self.assertNotIn('left_mm_s', result)

    def test_coarse_heading_overrides_steering_but_never_changes_fine_distance(self):
        observation = self.observation(self.center(350, 10))
        left = self.plan(observation, steering_bearing_deg=10)
        right = self.plan(observation, steering_bearing_deg=-10)
        self.assertLess(left['left_mm_s'], left['right_mm_s'])
        self.assertGreater(right['left_mm_s'], right['right_mm_s'])
        self.assertEqual(left['predicted_center_Base_mm'],right['predicted_center_Base_mm'])
        self.assertEqual(left['target_Base_X_mm'],right['target_Base_X_mm'])
        self.assertEqual(left['heading_source'],'coarse')
        for bearing in (-20,20):
            stopped = self.plan(self.observation(self.center(self.target)),
                                steering_bearing_deg=bearing)
            self.assertEqual(stopped['status'],'stop_for_verification')

    def test_single_correction_selects_signed_move_or_lateral_turn(self):
        for x in (self.far+5, self.near-5):
            verdict = self.verify(self.observation(self.center(x)))
            plan = correction_plan(verdict)
            self.assertEqual(plan['status'], 'move')
            self.assertEqual(plan['expected_reason'], 6)
            self.assertEqual(plan['distance_mm'], round(x-self.target))
            self.assertEqual(plan['command'], 'MOVE,%d' % plan['distance_mm'])
        for offset in (-25,25):
            verdict = self.verify(self.observation(self.center(self.far+5,offset)))
            plan = correction_plan(verdict)
            self.assertEqual(plan['status'], 'turn')
            self.assertEqual(plan['expected_reason'], 7)
            self.assertLessEqual(abs(plan['angle_deg']), 30)
            self.assertGreater(plan['angle_deg']*verdict['bearing_deg'], 0)
        self.assertEqual(correction_plan(self.verify(self.observation(self.center(self.target)))),
                         {'status': 'none'})
        self.assertEqual(correction_plan({'status': 'no_precise_target', 'accepted': False}),
                         {'status': 'unavailable'})


if __name__ == '__main__':
    unittest.main()
