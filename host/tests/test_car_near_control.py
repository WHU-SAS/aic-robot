"""Fine-centre prediction, parking semantics and an independent moving plant."""
import copy
import json
import math
from types import SimpleNamespace
import unittest

from device.k230.car.near_control import near_plan, predict_center, verify_parked, correction_plan, target_x
from device.k230.car.parking_window import bounds_mm, evaluate
from host.car.runtime import load
from host.project_paths import ROOT


class PlanarPlant:
    """World-frame midpoint integration, velocity ramps and exponential coast.

    Synthetic nominal mechanics, not identification of the physical robot.
    Its world pose and delayed absolute images do not use controller prediction.
    """
    def __init__(self, point, normal, track, forward_axis=None, origin=None):
        self.n, self.track = normal, track
        self.origin = origin or [0., 0., 0.]
        axis = forward_axis or [1., 0., 0.]
        component = sum(a*b for a,b in zip(axis, normal))
        f = [axis[k]-component*normal[k] for k in range(3)]
        norm = math.sqrt(sum(v*v for v in f))
        self.f = [v/norm for v in f]
        self.l = [normal[1]*self.f[2]-normal[2]*self.f[1],
                  normal[2]*self.f[0]-normal[0]*self.f[2],
                  normal[0]*self.f[1]-normal[1]*self.f[0]]
        offset = [point[k]-self.origin[k] for k in range(3)]
        self.target = [sum(a*b for a,b in zip(offset, basis))
                       for basis in (self.f, self.l, self.n)]
        self.x = self.y = self.yaw = 0
        self.vl = self.vr = 30

    def center(self):
        dx, dy = self.target[0]-self.x, self.target[1]-self.y
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        x, y = c*dx+s*dy, -s*dx+c*dy
        return [self.origin[k]+self.f[k]*x+self.l[k]*y+self.n[k]*self.target[2]
                for k in range(3)]

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

    def correction(self, verification):
        return correction_plan(verification, self.projection, self.workspace, self.config)

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
                                  self.config['track_width_mm'], self.projection.normal,
                                  self.projection.forward_axis_Base,
                                  self.projection.motion_origin_Base_mm)
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
            predict_center(point, [(20, 21)], self.config['track_width_mm'], self.projection.normal,
                           self.projection.forward_axis_Base,
                           self.projection.motion_origin_Base_mm))
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
        self.assertEqual(self.correction(verdict), {'status': 'unavailable'})

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
        self.assertAlmostEqual(result['target_Base_X_mm'], 250.3243743214)
        far_inside = self.verify(self.observation(self.center(self.far-.1)))
        self.assertEqual(far_inside['status'], 'parked')
        self.assertTrue(far_inside['accepted'])
        self.assertTrue(far_inside['preferred'])
        y_limits = bounds_mm(self.workspace,self.target)['Base_Y_mm']
        outside_offset = (y_limits[1]-y_limits[0])/2+.1
        for point in (self.center(self.far+.1), self.center(self.target,outside_offset)):
            outside = self.verify(self.observation(point))
            self.assertFalse(outside['accepted'])
            self.assertFalse(outside['preferred'])
        point = self.center(self.target)
        point[1] = 0
        # Reexpressing the camera-centred physical window in the corrected
        # model's Base puts BaseY=0 inside it; BaseY=0 is not the target line.
        self.assertTrue(self.verify(self.observation(point))['accepted'])
        point[1] = bounds_mm(self.workspace, self.target)['Base_Y_mm'][0] - .1
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
        self.assertEqual(result['static_timing'], {'stopped_at_ms': 1900,
            'capture_timestamp_ms': 2000, 'result_ready_ms': 3500, 'checked_at_ms': 3501})
        cases = (
            ({'now': 3501, 'stopped_at': 2100, 'ready': 3500}, 'capture_before_stop'),
            ({'now': 3501, 'stopped_at': 1900, 'ready': 3500, 'stationary': False}, 'chassis_not_stationary'),
            ({'now': 4001, 'stopped_at': 1900, 'ready': 3500}, 'static_result_expired'),
            ({'now': 3499, 'stopped_at': 1900, 'ready': 3500}, 'check_before_result'),
            ({'now': 3501, 'stopped_at': 1900, 'ready': 1999}, 'result_before_capture'))
        for timing, reason in cases:
            with self.subTest(reason=reason):
                rejected = self.verify(observation, **timing)
                self.assertFalse(rejected['accepted'])
                self.assertEqual(rejected['reason_code'], reason)
                self.assertEqual(rejected['static_timing'], {'stopped_at_ms': timing['stopped_at'],
                    'capture_timestamp_ms': 2000, 'result_ready_ms': timing['ready'],
                    'checked_at_ms': timing['now']})

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
        self.assertEqual((result['left_mm_s']+result['right_mm_s'])/2, 70)

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

    def test_single_arc_targets_the_agreed_depth_and_image_midline(self):
        target_y = sum(bounds_mm(self.workspace, self.target)['Base_Y_mm'])/2
        for x, offset in ((self.far+5, 0), (self.near-5, 0),
                          (self.far+5, -25), (self.far+5, 25)):
            with self.subTest(x=x, offset=offset):
                verdict = self.verify(self.observation(self.center(x, offset)))
                plan = self.correction(verdict)
                self.assertEqual(plan['status'], 'arc')
                self.assertEqual(plan['expected_reason'], 8)
                self.assertEqual(plan['command'], 'ARC,%d,%d' %
                                 (plan['left_mm'], plan['right_mm']))
                self.assertAlmostEqual(plan['target_Base_X_mm'], self.target)
                self.assertAlmostEqual(plan['target_Base_Y_mm'], target_y)
                self.assertLess(plan['nominal_target_error_mm'], 1.6)
                self.assertTrue(plan['nominal_predicted_window']['inside'])
                self.assertGreater(plan['nominal_window_margin_mm'], 3.)
                self.assertLessEqual(abs(plan['nominal_yaw_deg']), 30.)
                self.assertNotIn('accepted', plan)
        self.assertEqual(self.correction(self.verify(self.observation(self.center(self.target)))),
                         {'status': 'none'})
        self.assertEqual(self.correction({'status': 'no_precise_target', 'accepted': False}),
                         {'status': 'unavailable'})

    def test_latest_reverse_correction_preserves_lateral_room_and_targets_both_axes(self):
        # Raw pre-correction observation from robot_task_32483_0. The former
        # MOVE,-6 only corrected depth while starting3mm from a lateral edge.
        point = [242.6074, -10.35546, -54.81823]
        verdict = self.verify(self.observation(point))
        self.assertTrue(verdict['window']['lateral_allowed'])
        self.assertFalse(verdict['window']['longitudinal_allowed'])
        plan = self.correction(verdict)
        self.assertLess(plan['left_mm'], 0)
        self.assertLess(plan['right_mm'], plan['left_mm'])
        self.assertLess(plan['nominal_target_error_mm'], 1.)
        self.assertGreater(plan['nominal_window_margin_mm'], 4.)
        self.assertTrue(plan['nominal_predicted_window']['inside'])
        self.assertIn('uncalibrated axle lever arm', plan['prediction_basis'])

    def test_single_arc_can_reverse_one_wheel_and_hold_the_other(self):
        # The old forward TURN,-5 used up the available depth. A signed wheel
        # arc moves toward the same destination instead of accepting a1mm rim.
        # Keep this synthetic case outside the current expanded right edge.
        x = self.target-6
        lo,hi = bounds_mm(self.workspace,x)['Base_Y_mm']
        point = self.center(x,-(hi-lo)/2-2)
        verdict = self.verify(self.observation(point))
        travel = self.config['track_width_mm']*math.radians(5)
        old = predict_center(point, [(travel, 0)], self.config['track_width_mm'],
            self.projection.normal, self.projection.forward_axis_Base,
            self.projection.motion_origin_Base_mm)
        self.assertFalse(evaluate(old[0], old[1], self.workspace)['longitudinal_allowed'])
        plan = self.correction(verdict)
        self.assertEqual(plan['left_mm'], 0)
        self.assertLess(plan['right_mm'], 0)
        self.assertLess(plan['nominal_target_error_mm'], 1.)
        self.assertGreater(plan['nominal_window_margin_mm'], 4.)
        self.assertTrue(plan['nominal_predicted_window']['inside'])

    def test_lateral_error_at_the_right_depth_can_use_opposite_wheel_distances(self):
        x = self.target-1
        lo,hi = bounds_mm(self.workspace,x)['Base_Y_mm']
        point = self.center(x,-(hi-lo)/2-.1)
        plan = self.correction(self.verify(self.observation(point)))
        self.assertLess(plan['left_mm']*plan['right_mm'], 0)
        self.assertLess(plan['nominal_target_error_mm'], 1.)
        self.assertTrue(plan['nominal_predicted_window']['inside'])
        self.assertGreater(plan['nominal_window_margin_mm'], 4.)

    def test_signed_arc_reaches_destination_in_independent_world_frame_plant(self):
        point = [242.6074, -10.35546, -54.81823]
        # Exercise the actual paired calibration plus a visibly rotated and
        # translated synthetic motion frame. Neither plant uses predict_center.
        rotated = SimpleNamespace(normal=[0., 0., 1.],
            forward_axis_Base=[math.cos(.35), math.sin(.35), 0.],
            motion_origin_Base_mm=[31., -22., 4.])
        for projection in (self.projection, rotated):
            with self.subTest(origin=projection.motion_origin_Base_mm):
                verdict = {'status':'not_parked', 'accepted':False,
                           'cube_center_Base_mm':point}
                plan = correction_plan(verdict, projection, self.workspace, self.config)
                self.assertLess(plan['left_mm'], 0)
                self.assertLess(plan['right_mm'], 0)
                plant = PlanarPlant(point, projection.normal, self.config['track_width_mm'],
                    projection.forward_axis_Base, projection.motion_origin_Base_mm)
                plant.vl, plant.vr = plan['left_mm'], plan['right_mm']
                for unused in range(1000):
                    plant.advance(.001)
                actual = plant.center()
                error = math.hypot(actual[0]-plan['target_Base_X_mm'],
                                   actual[1]-plan['target_Base_Y_mm'])
                self.assertLess(error, 1.5)
                self.assertTrue(evaluate(actual[0], actual[1], self.workspace)['inside'])
                for observed, predicted in zip(actual, plan['nominal_predicted_center_Base_mm']):
                    self.assertAlmostEqual(observed, predicted, places=6)

    def test_unreachable_destination_is_one_bounded_nominal_attempt_not_acceptance(self):
        # Synthetic coordinates test actuator budgeting, not visibility or a
        # guarantee that a far-away target can be corrected in one short move.
        for sign in (-1, 1):
            verdict = self.verify(self.observation(self.center(self.far+500, sign*10000)))
            plan = self.correction(verdict)
            self.assertEqual(plan['status'], 'arc')
            self.assertLessEqual(max(abs(plan['left_mm']), abs(plan['right_mm'])), 100)
            self.assertLessEqual(abs(plan['nominal_yaw_deg']), 30.)
            self.assertFalse(plan['nominal_predicted_window']['inside'])
            self.assertLess(plan['nominal_window_margin_mm'], 0)
            self.assertNotIn('accepted', plan)


if __name__ == '__main__':
    unittest.main()
